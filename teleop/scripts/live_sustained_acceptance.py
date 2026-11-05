"""Mapped sustained inputs within 15 mm / 2 degrees; never direct targets.

Short one-millimetre pulses need not reach the advertised velocity ceiling.
This additional suite verifies the commanded ceiling and reports the slower
wall-clock target rate caused by skipped, still-pending ServoJ periods.
"""
import argparse
import asyncio
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from live_signal_acceptance import LiveSignals
from e6_observer.control_math import angle, pose


class SustainedSignals(LiveSignals):
    async def execute(self, suite):
        # Initial gamepad fixtures intentionally use a 30% stick amplitude.
        # The real keyboard mapper requests the full backend ceiling and is
        # already covered by the corresponding passing shadow matrix.
        source='keyboard'
        await self.phase(5.5, self.neutral(source))
        await self.prepare(source)
        limit = self.status['limits']['translation_m_s']
        part=getattr(self,'part','all')
        for fine in (() if part=='rotation' else (False, True)):
            amount = .002 if fine else min(.015, limit * 4)
            for axis in range(3):
                await self.round_trip(source, 'translation', 'base', axis, amount, fine)
        # The original J6 suite has an intentionally narrower .25-degree
        # assertion. Check this additional mapped, larger tool-Z return here.
        for fine in (() if part=='translation' else (False, True)):
            await self.prepare(source, 'rotation', 'tool')
            origin = copy.deepcopy(self.actual()['controller_pose_raw_mm_deg'])
            q0 = np.asarray(self.actual()['joints_rad'])
            amount = math.radians(.5 if fine else 2)
            outward = await self.increment(source,'rotation','tool',2,1,amount,origin,fine)
            returned = await self.increment(source,'rotation','tool',2,-1,amount,origin,fine,True)
            p0,r0 = pose(origin)
            p1,r1 = pose(returned['actual']['pose_mm_deg'])
            change = np.asarray(outward['actual']['joints_rad']) - q0
            if abs(change[5]) < amount * .5 or np.max(np.abs(change[:5])) > math.radians(.1):
                raise ValueError('Sustained tool Z did not track isolated J6')
            error = dict(position_m=float(np.linalg.norm(p1-p0)),rotation_rad=angle(r0,r1),
                         joint_max_rad=float(np.max(np.abs(np.asarray(self.actual()['joints_rad'])-q0))))
            if error['position_m'] > .001 or error['rotation_rad'] > math.radians(.15) or error['joint_max_rad'] > math.radians(.15):
                raise ValueError('Sustained rotation did not recover new baseline')
            self.checks.append(dict(passed=True,sustained_rotation=True,fine=fine,
                                    outward=outward,returned=returned,return_residual=error))


def measure(directory):
    summary = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    previous = None
    rates = {}
    with (directory/'frames.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            frame = json.loads(line)
            tick = frame['control']['tick']
            action = tick.get('effective_action')
            if action is None:
                continue
            inp = tick['input']
            mode = inp['mode']
            offset = 0 if mode == 'translation' else 3
            axis = int(np.argmax(np.abs(inp['twist'][offset:offset+3])))
            key = (mode,inp['frame'],inp['fine'],axis,tuple(inp['twist']))
            p,r = pose(action['requested_tcp']['controller_pose_mm_deg'])
            current = (key,p,r,action['gateway_accepted_ns'])
            if previous and previous[0] == key:
                delta = float(np.linalg.norm(p-previous[1])) if offset == 0 else angle(r,previous[2])
                cycle_rate = delta/tick['dt_s']
                wall_rate = delta/((current[3]-previous[3])/1e9)
                label = f'{mode}/{inp["frame"]}/fine={inp["fine"]}/axis={axis}'
                rates.setdefault(label,[]).append((cycle_rate,wall_rate))
            previous = current
    limits = summary['final_status']['limits']
    checks = []
    for label, values in sorted(rates.items()):
        cap = limits['translation_m_s'] if label.startswith('translation') else limits['rotation_rad_s']
        cap *= .25 if 'fine=True' in label else 1
        cycle,wall = np.asarray(values).T
        checks.append(dict(label=label,passed=bool(cycle.max() >= .95*cap),
                           configured_cap_m_or_rad_s=cap,cycle_rate_max=float(cycle.max()),
                           wall_rate_p50=float(np.median(wall)),wall_rate_max=float(wall.max())))
    part=summary.get('sustained_selection','all')
    expected={f'translation/base/fine={fine}/axis={axis}' for fine in (False,True) for axis in range(3)} if part!='rotation' else set()
    if part!='translation':expected|={f'rotation/tool/fine={fine}/axis=2' for fine in (False,True)}
    result = dict(passed=bool(summary['passed'] and set(rates)==expected and all(c['passed'] for c in checks)),
                  scope=part,parent_run_passed=summary['passed'],
                  checks=checks,units='m/s for translation; rad/s for rotation',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  semantics='accepted target integration rate and actual accepted-target wall cadence, not physical maximum speed')
    (directory/'sustained_rates.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if not result['passed']:
        raise ValueError('Sustained suite or velocity-ceiling coverage failed; retained evidence')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--fixtures',type=Path,required=True)
    parser.add_argument('--shadow-evidence',type=Path,required=True)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--part',choices=('all','translation','rotation'),default='all')
    args = parser.parse_args()
    runner = SustainedSignals(json.loads(args.fixtures.read_text()),args.evidence,args.shadow_evidence)
    runner.part=args.part
    script_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    try:
        asyncio.run(runner.run_live('sustained'))
    finally:
        # Preserve scope and provenance on aborted runs as well. Never infer
        # an all-axis acceptance from a partial failed invocation.
        summary_path=args.evidence/'summary.json'
        if summary_path.exists():
            summary=json.loads(summary_path.read_text(encoding='utf-8'))
            summary['sustained_selection']=args.part
            summary['sustained_harness_sha256']=script_sha
            changed=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()!=script_sha
            summary['sustained_harness_changed_during_run']=changed
            if changed:summary.update(passed=False,failure='Sustained harness changed during execution')
            summary_path.write_text(json.dumps(summary,indent=2),encoding='utf-8')
    measure(args.evidence)
