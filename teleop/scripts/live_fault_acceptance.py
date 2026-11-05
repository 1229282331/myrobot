"""Stationary fault checks through the production PC/SSH control chain.

Only mapped zero increments are exercised. No enable, recovery, parameter
write, CAN initialization, direct joint command or automatic replay.
"""
import argparse
import asyncio
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np
import websockets
from live_signal_acceptance import LiveSignals

FAULTS = (
    ('duplicate', '重复或乱序'), ('out-of-order', '重复或乱序'),
    ('old-lease', '租约'), ('lease-expiry', '输入看门狗'),
    ('focus', '失焦'), ('disconnected', '输入断开'),
)


class LiveFaults(LiveSignals):
    async def expect_stopped(self, reason, q0, command_rejection=None):
        # Read-only wait: do not keep the failed input or lease alive.
        await asyncio.sleep(1.3)
        status = self.latest['status']
        first = (status.get('first_fault') or {}).get('reason', '')
        # Envelope validation returns the precise rejection to the requester;
        # the core deliberately preserves a generic fail-closed first fault.
        if command_rejection is not None:
            if reason not in command_rejection or '命令被拒绝' not in first:
                raise ValueError('Rejection and stopped fault were not both retained')
        elif reason not in first:
            raise ValueError('Unexpected first fault: ' + first)
        if status['armed'] or not status.get('stop_acknowledged') or not status.get('stopped_feedback'):
            raise ValueError('Injected fault did not confirm Stop and stationary feedback')
        delta = float(np.max(np.abs(np.asarray(self.actual()['joints_rad']) - q0)))
        if delta > math.radians(.05):
            raise ValueError('Stationary fault test moved outside .05 degrees')
        return dict(first_fault=first, stop_acknowledged=True, stopped_feedback=True,
                    actual_joint_change_max_rad=delta)

    async def execute(self, suite):
        n = self.neutral('keyboard')
        selected = getattr(self, 'fault_case', 'all')
        for case, expected in FAULTS:
            if selected not in ('all', case):continue
            await self.phase(5.5, n)
            await self.prepare('keyboard')
            q0 = np.asarray(self.actual()['joints_rad'])
            before = self.latest['control']['stats']['motion_tx']
            await self.phase(.1, dict(n, deadman=True), True)
            rejection=None
            if case == 'lease-expiry':
                await asyncio.sleep(.35)
            else:
                data = copy.deepcopy(n)
                if case in ('duplicate', 'out-of-order', 'old-lease'):
                    # Raw envelope corruption is fault injection, not an
                    # alternative Cartesian or joint-control implementation.
                    self.request_id += 1
                    ident = self.request_id
                    future = asyncio.get_running_loop().create_future()
                    import time
                    self.requests[ident] = (time.perf_counter_ns(), 'input', future)
                    data.update(request_id=ident, seq=self.seq if case=='duplicate' else self.seq-1 if case=='out-of-order' else self.seq+1,
                                lease='rejected-old-lease' if case=='old-lease' else self.status['lease'])
                    await self.ws.send(json.dumps(data))
                    try:
                        await asyncio.wait_for(future, 1)
                    except RuntimeError as exc:
                        if expected not in str(exc): raise
                        rejection=str(exc)
                    else:
                        raise AssertionError('Corrupted envelope was accepted')
                else:
                    data['focused' if case=='focus' else 'connected'] = False
                    await self.rpc(data)
            evidence = await self.expect_stopped(expected, q0, rejection)
            after = self.latest['control']['stats']['motion_tx']
            # With deliberate sender silence, the last mapped zero input is
            # still valid until the unchanged 250 ms lease expires. Zero
            # ServoJ ticks in that interval are expected, rather than replay.
            ceiling = math.ceil((.1+self.status['lease_duration_ms']/1000)/self.status['limits']['period_s'])+1 if case=='lease-expiry' else 2
            if not before < after <= before+ceiling:
                raise ValueError('Stationary zero-input command count was unexpected')
            stop_ns=self.latest['status']['stop_requested_ns']
            if any(e.get('command','').startswith('ServoJ(') and e.get('x5_sent_ns',0)>stop_ns
                   for e in self.latest['control'].get('command_events',[])):
                raise ValueError('A motion command was sent after the stopped fault')
            self.checks.append(dict(case=case, passed=True, command_rejection=rejection,
                zero_motion_commands=after-before,zero_motion_ceiling=ceiling,**evidence))
            print(json.dumps(dict(case=case, passed=True)), flush=True)
        # Independent runs end at the stopped fault. They cannot silently
        # erase rejected in-flight inputs or count a failed prefix as a pass.
        if selected not in ('all', 'second-controller', 'software-stop'):return
        # A rejected second controller must not cancel the legitimate owner.
        await self.phase(5.5,n); await self.prepare('keyboard')
        if selected in ('all', 'second-controller'):
            async with websockets.connect('ws://127.0.0.1:8765/ws/control',proxy=None,
                    additional_headers={'X-E6-Control-Token':os.environ['E6_TELEOP_HARNESS_TOKEN']}) as second:
                hello=json.loads(await second.recv())
                if hello.get('ok'): raise AssertionError('Second controller was accepted')
            await self.phase(.3,n,True)
            self.checks.append(dict(case='second-controller',passed=True,legitimate_owner_still_armed=True))
        await self.rpc(dict(operation='stop'))
        await asyncio.sleep(1.3)
        if self.latest['status']['armed'] or not self.latest['status'].get('stop_acknowledged') or not self.latest['status'].get('stopped_feedback'):
            raise ValueError('Explicit software Stop failed')
        self.checks.append(dict(case='software-stop',passed=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--fixtures',type=Path,required=True)
    p.add_argument('--shadow-evidence',type=Path,required=True)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--case',choices=('all',*(name for name,_ in FAULTS),'second-controller','software-stop'),default='all',
                   help='Independent single-case runs require a fresh owner and retain their own stopped result')
    args=p.parse_args()
    runner=LiveFaults(json.loads(args.fixtures.read_text(encoding='utf-8')),args.evidence,args.shadow_evidence)
    runner.fault_case=args.case
    script_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    try:
        asyncio.run(runner.run_live('stationary-faults'))
    finally:
        summary_path=args.evidence/'summary.json'
        if summary_path.exists():
            value=json.loads(summary_path.read_text(encoding='utf-8'))
            changed=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()!=script_sha
            value.update(fault_harness_sha256=script_sha,fault_harness_changed_during_run=changed,
                         fault_selection=args.case,
                         fault_scope='zero-increment physical arm; no CAN commands',
                         uncovered_physical_faults=['SSH interruption','worker exit','E6/RGB/CAN loss','disk failure'])
            if changed:value.update(passed=False,failure='Fault harness changed during execution')
            summary_path.write_text(json.dumps(value,indent=2),encoding='utf-8')
