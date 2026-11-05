"""Isolated official Servo C++ API probe. Never imports or connects to E6/CAN.

The .05 s solver step is explicit, different from the ROS node's .01 s
internal step. Measurements are virtual, not real-device acceptance.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import select
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from qualify_jazzy_bridge import Plant


def percentiles(values):
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [0, 50, 95, 99, 100])))) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--binary', required=True, help='Path inside the isolated rootfs')
    parser.add_argument('--params', required=True, help='Path inside the isolated rootfs')
    parser.add_argument('--suite', choices=['startup', 'speed', 'directions'], required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = json.loads((args.bundle / 'jazzy-runtime.json').read_text())
    plant = Plant(args.bundle, .15)
    result = dict(hardware_io=False, scope='isolated C++ API with virtual plant',
                  solver_period_s=.05, target_period_s=.05, virtual_tau_s=.15,
                  suite=args.suite, cases=[], error=None)
    period = .05
    process = None
    sequence = 0
    log = gzip.open(args.output / 'samples.jsonl.gz', 'wt', encoding='utf-8')
    child_log = (args.output / 'child.log').open('w')
    try:
        argv = ['chroot', config['rootfs'], '/usr/bin/env', 'ROS_LOCALHOST_ONLY=1',
                'ROS_DOMAIN_ID=87', 'OPENBLAS_NUM_THREADS=1', 'RCUTILS_LOGGING_USE_STDOUT=0',
                'FASTDDS_BUILTIN_TRANSPORTS=UDPv4', 'RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS',
                '/bin/bash', '-c',
                'source /opt/ros/jazzy/setup.bash; export AMENT_PREFIX_PATH="$1:$AMENT_PREFIX_PATH"; '
                'export LD_LIBRARY_PATH="$1/lib:$LD_LIBRARY_PATH"; exec "$2" --ros-args --params-file "$3"',
                'bash', config['plugin_prefix'], args.binary, args.params]
        process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=child_log, text=True, bufsize=1)

        def exchange(epoch, active, kind, frame, velocity, timeout=3):
            nonlocal sequence
            sequence += 1
            plant.update()
            started = time.perf_counter_ns()
            values = [sequence, epoch, int(active), int(kind != 'joint'), int(frame == 'tool'),
                      *plant.q.tolist(), *velocity]
            process.stdin.write(' '.join(map(str, values)) + '\n')
            process.stdin.flush()
            if not select.select([process.stdout], [], [], timeout)[0]:
                raise TimeoutError('C++ probe response timeout')
            line = process.stdout.readline()
            if not line:
                raise RuntimeError('C++ probe exited: ' + str(process.poll()))
            reply = json.loads(line)
            ended = time.perf_counter_ns()
            if (reply['seq'], reply['epoch'], reply['active']) != (sequence, epoch, active):
                raise RuntimeError('Probe response identity mismatch')
            q = np.asarray(reply['joints_rad'])
            if q.shape != (6,) or not np.isfinite(q).all():
                raise RuntimeError('Invalid candidate')
            log.write(json.dumps(dict(sent_ns=started, received_ns=ended, input=values,
                                      response=reply, virtual_actual=plant.q.tolist())) + '\n')
            if active and reply['status'] != 0:
                raise RuntimeError('Official Servo status: ' + str(reply['status']))
            if not active and not np.allclose(q, plant.q, atol=1e-12, rtol=0):
                raise RuntimeError('Release or epoch reset retained an old target')
            return reply, (ended - started) / 1e6

        exchange(0, False, 'joint', 'base', [0.] * 6, timeout=30)
        (args.output / 'binary-maps.txt').write_text(Path('/proc', str(process.pid), 'maps').read_text())
        if args.suite == 'startup':
            cases = [(('joint', 'base', 'tool', 'base')[i % 4], 5 if i % 4 == 0 else 3,
                      1 if (i // 4) % 2 == 0 else -1, 1.) for i in range(32)]
        elif args.suite == 'speed':
            cases = [('joint', 5, sign, speed) for speed in (1., .25) for sign in (1, -1)]
        else:
            cases = [(kind, axis, sign, 1.) for kind in ('joint', 'base', 'tool')
                     for axis in range(6) for sign in (-1, 1)]
        for index, (kind, axis, sign, scale) in enumerate(cases):
            frame = 'tool' if kind == 'tool' else 'base'
            request = sign * scale * (.03 if kind != 'joint' and axis < 3 else math.radians(10))
            velocity = [0.] * 6
            velocity[axis] = request
            before = plant.snapshot()
            start = time.monotonic()
            duration = 6. if args.suite == 'speed' else .45 if args.suite == 'startup' else 1.3
            next_tick = start
            times, responses, samples, gaps = [], [], [], []
            first_changed = None
            previous_target = plant.q.copy()
            previous_velocity = np.zeros(6)
            max_velocity = max_acceleration = 0.
            while time.monotonic() - start < duration:
                plant.update()
                now = time.monotonic()
                if now >= next_tick:
                    # No replay or catch-up. Period starts at this actual call.
                    next_tick = now + period
                    reply, elapsed = exchange(index + 1, True, kind, frame, velocity)
                    responses.append(elapsed)
                    if times:
                        gaps.append(now - times[-1])
                    times.append(now)
                    q = np.array(reply['joints_rad'])
                    target_velocity = (q - previous_target) / period
                    target_acceleration = (target_velocity - previous_velocity) / period
                    max_velocity = max(max_velocity, float(np.max(abs(target_velocity))))
                    max_acceleration = max(max_acceleration, float(np.max(abs(target_acceleration))))
                    previous_target, previous_velocity = q.copy(), target_velocity
                    if first_changed is None and np.max(abs(q - plant.q)) > 1e-9:
                        first_changed = (time.monotonic() - start) * 1000
                    plant.target = q
                samples.append((now - start, plant.q.copy()))
                time.sleep(.001)
            after = plant.snapshot()
            plant.stop_fixture()
            exchange(index + 1, False, kind, frame, [0.] * 6)
            # A new epoch while neutral must also discard every old target.
            hold = plant.q.copy()
            for _ in range(5):
                time.sleep(period)
                exchange(index + 2, False, kind, frame, [0.] * 6)

            def pose(sample):
                v = sample['status']['tool_vector_actual']
                return (np.array([v[k] for k in ('x', 'y', 'z')]) / 1000,
                        Rotation.from_euler('xyz', [v[k] for k in ('rx', 'ry', 'rz')], degrees=True))

            if kind == 'joint':
                delta = np.deg2rad(np.array(after['status']['joint_state']['q_actual']) -
                                  before['status']['joint_state']['q_actual'])
                component = axis
            else:
                p0, r0 = pose(before)
                p1, r1 = pose(after)
                delta = p1 - p0 if axis < 3 else (r1 * r0.inv()).as_rotvec()
                if frame == 'tool':
                    delta = r0.inv().apply(delta)
                component = axis % 3
            norm = float(np.linalg.norm(delta))
            cosine = float(sign * delta[component] / norm) if norm > 1e-9 else 0.
            case = dict(index=index, kind=kind, frame=frame, axis=axis, sign=sign,
                        request_si=request, calls=len(times), first_changed_ms=first_changed,
                        roundtrip_ms=percentiles(responses), period_s=percentiles(gaps),
                        max_target_velocity_rad_s=max_velocity,
                        max_target_acceleration_rad_s2=max_acceleration,
                        direction_cosine=cosine, hold_excursion_rad=float(np.max(abs(plant.q - hold))))
            if args.suite == 'speed':
                steady = [(t, q[axis]) for t, q in samples if 2 < t < 5.8]
                case['speed_ratio'] = float(np.polyfit(*zip(*steady), 1)[0] / request)
            case['passed'] = (first_changed is not None and first_changed < 200 and cosine > .97 and
                              case['hold_excursion_rad'] < 1e-12 and max(gaps, default=0) < .15 and
                              max_velocity <= math.radians(30) + 1e-7 and
                              max_acceleration <= math.radians(120) + 1e-7 and
                              .95 <= case.get('speed_ratio', 1) <= 1.05)
            result['cases'].append(case)
            if not case['passed']:
                raise RuntimeError('Virtual qualification case failed: ' + str(case))
        result['first_changed_ms'] = percentiles([c['first_changed_ms'] for c in result['cases']])
        result['passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if process is not None:
            process.stdin.close()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            result['child_exit'] = process.returncode
            if process.returncode != 0:
                result['passed'] = False
                result['error'] = result['error'] or 'Probe did not exit normally'
            process.stdout.close()
        plant.close()
        child_log.close()
        log.close()
        result['forbidden_imports'] = [name for name in sys.modules
            if name.startswith(('dobot_sdk', 'can.', 'e6_phosphobot.dashboard', 'e6_phosphobot.gateway'))]
        if result['forbidden_imports']:
            result['passed'] = False
            result['error'] = result['error'] or 'Device-related module imported by virtual probe'
        result['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result))
    return int(bool(result['error']) or not result.get('passed') or bool(result['forbidden_imports']))


if __name__ == '__main__':
    raise SystemExit(main())
