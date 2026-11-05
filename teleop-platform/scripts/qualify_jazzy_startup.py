"""Isolated production MoveIt bridge startup comparison, with a virtual plant.

No device connection, input-page qualification or physical response claim.
The existing first-candidate notification may advance the first virtual target;
subsequent targets remain 20 Hz. Every failed trial stays in the result.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
import sys
import time
import resource

import numpy as np
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge
from qualify_jazzy_bridge import Plant


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trials', type=int, default=32)
    parser.add_argument('--api', choices=['ros', 'cpp'], default='ros')
    args = parser.parse_args()
    if not 4 <= args.trials <= 100:
        parser.error('trials must be between 4 and 100')
    args.output.mkdir(parents=True, exist_ok=False)
    plant = Plant(args.bundle, .15)
    bridge = MoveItBridge(args.bundle, plant, motion_profile='normal', api=args.api)
    result = dict(hardware_io=False, api=args.api, virtual_tau_s=.15, trials=[], error=None,
                  clock='same X5 kernel CLOCK_MONOTONIC',
                  scope=f'isolated {args.api} startup; not browser or real E6 latency')
    cpu_start = resource.getrusage(resource.RUSAGE_SELF)
    child_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    wall_start = time.perf_counter()
    sequence = 0
    log = gzip.open(args.output / 'samples.jsonl.gz', 'wt', encoding='utf-8')
    try:
        bridge.start()
        deadline = time.monotonic() + 40
        while not bridge.snapshot()['ready']:
            plant.update()
            if bridge.snapshot()['phase'] == 'failed' or time.monotonic() > deadline:
                raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        for index in range(args.trials):
            # Paired signs keep the virtual robot near its seed. This is not
            # a certified real path and must never be sent to the E6.
            kind = ('joint', 'base', 'tool', 'base')[index % 4]
            sign = 1 if (index // 4) % 2 == 0 else -1
            intent = Intent(linear_m_s=[0.] * 3,
                angular_rad_s=[sign * math.radians(10), 0., 0.] if kind != 'joint' else [0.] * 3,
                joint_rad_s=[0.] * 5 + [sign * math.radians(10)] if kind == 'joint' else [0.] * 6,
                frame='tool' if kind == 'tool' else 'base', deadman=True, gripper_rate_s=0.)
            start = time.perf_counter_ns()
            trial = dict(index=index, kind=kind, sign=sign, start_ns=start,
                         first_candidate=None, first_take_ms=None, candidates=0, error=None)
            result['trials'].append(trial)
            next_input = start
            next_target = start
            while time.perf_counter_ns() - start < 450_000_000:
                plant.update()
                now = time.perf_counter_ns()
                if now >= next_input:
                    sequence += 1
                    bridge.submit(intent, sequence)
                    next_input = now + 50_000_000
                info = bridge.snapshot()
                if info['phase'] == 'failed':
                    raise RuntimeError(str(info))
                candidate = None
                if now >= next_target or (trial['first_take_ms'] is None and bridge.initial_candidate_event.is_set()):
                    candidate = bridge.take()
                    next_target = time.perf_counter_ns() + 50_000_000
                    if candidate:
                        trial['candidates'] += 1
                        if trial['first_take_ms'] is None:
                            trial['first_take_ms'] = (time.perf_counter_ns() - start) / 1e6
                        plant.target = np.array(candidate['joints_rad'])
                        for source in candidate['trajectory_sources']:
                            if source['kind'] != 's' and source['input']['epoch'] != bridge.epoch:
                                raise RuntimeError('Candidate includes a prior input epoch')
                if info.get('first_candidate'):
                    trial['first_candidate'] = info['first_candidate']
                log.write(json.dumps(dict(trial=index, at_ns=time.perf_counter_ns(),
                    model=info, candidate=candidate), separators=(',', ':')) + '\n')
                time.sleep(.002)
            bridge.stop()
            plant.stop_fixture()
            if trial['first_take_ms'] is None:
                trial['error'] = 'No fresh executable candidate in 450 ms'
            until = time.monotonic() + .35
            while time.monotonic() < until:
                plant.update()
                if bridge.take() is not None:
                    raise RuntimeError('Candidate survived release')
                time.sleep(.005)
        for key in ('computed_ns', 'host_received_ns'):
            values = [(t['first_candidate'][key] - t['start_ns']) / 1e6
                      for t in result['trials'] if t['first_candidate']]
            result[key.replace('_ns', '_ms')] = dict(n=len(values),
                p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), max=max(values)) if values else None
        result['passed'] = all(t['error'] is None for t in result['trials'])
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        bridge.close()
        plant.close()
        log.close()
        result['bridge'] = bridge.snapshot()
        result['forbidden_imports'] = [name for name in sys.modules
            if name.startswith(('dobot_sdk', 'can.', 'e6_phosphobot.dashboard', 'e6_phosphobot.gateway'))]
        result['child_exit'] = bridge.process.returncode if bridge.process else None
        cpu_end = resource.getrusage(resource.RUSAGE_SELF)
        child_end = resource.getrusage(resource.RUSAGE_CHILDREN)
        result['cpu'] = dict(wall_s=time.perf_counter()-wall_start,
            host_s=cpu_end.ru_utime+cpu_end.ru_stime-cpu_start.ru_utime-cpu_start.ru_stime,
            children_s=child_end.ru_utime+child_end.ru_stime-child_start.ru_utime-child_start.ru_stime)
        result['passed'] = bool(result.get('passed') and not result['error'] and
                                not result['forbidden_imports'] and result['child_exit'] == 0)
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({k: v for k, v in result.items() if k not in ('trials', 'bridge')}))
    return int(bool(result['error']) or not result.get('passed'))


if __name__ == '__main__':
    raise SystemExit(main())
