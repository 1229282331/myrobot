"""PC offline evaluation against captured controller FK, no device access.

Production input mapping and ControlCore are exercised with a stationary
feedback fixture. This is not PC/SSH/X5 control-chain or live acceptance.
"""
import argparse
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'device_gateway'))
sys.path.insert(0, str(ROOT/'experiments'))
from local_kinematics import LocalKinematics
from e6_observer.control_core import ControlCore
from e6_observer.control_math import angle, controller_pose, pose


def read(path):
    if path.exists(): return json.loads(path.read_text(encoding='utf-8'))
    with gzip.open(str(path)+'.gz', 'rt', encoding='utf-8') as f: return json.load(f)


def percentiles(values):
    return dict(zip(('p50','p95','p99','max'), np.percentile(values, [50,95,99,100]).tolist()))


class Clock:
    def __init__(self): self.ns = 10_000_000_000
    def __call__(self): return self.ns
    def step(self): self.ns += 50_000_000


class LocalDevice:
    def __init__(self, model, clock): self.model = model; self.clock = clock
    def ik(self, raw, seed, deadline):
        p, r = pose(raw)
        q, _ = self.model.solve(p, r, np.radians(seed))
        return np.degrees(q).tolist()
    def fk(self, q, deadline):
        p, r = self.model.fk(np.radians(q))
        return controller_pose(p, r, [0]*6)
    def state(self):
        return dict(x5_receive_monotonic_ns=self.clock(), test_fixture=True, decoded=dict(
            joints_rad=self.model.anchor_q.tolist(), q_actual_deg=np.degrees(self.model.anchor_q).tolist(),
            controller_pose_raw_mm_deg=controller_pose(self.model.anchor_p, self.model.anchor_r, [0]*6),
            robot_mode=4, enable_status=0, error_status=0, collision_state=0, pause_command_flag=1,
            user_index=0, tool_index=0, user_configuration_raw_mm_deg=[0]*6, tool_configuration_raw_mm_deg=[0]*6,
            q_target_deg=np.degrees(self.model.anchor_q).tolist(), run_queued_command=0, current_command_id=0))
    def servo(self, *args): raise AssertionError('Experiment cannot send motion')
    def stop(self, *args): raise AssertionError('Experiment cannot send Stop')
    def grip(self, *args): raise AssertionError('Experiment cannot send CAN')


def evaluate(capture_path, fixtures_path, directory):
    directory.mkdir(parents=True, exist_ok=False)
    capture = read(capture_path); fixtures = read(fixtures_path)
    if capture.get('failure') or any(capture.get(k, 1) for k in ('motion_tx','stop_tx','can_tx')):
        raise ValueError('Incomplete or non-read-only capture')
    sha = hashlib.sha256((ROOT/'web/src/input.ts').read_bytes()).hexdigest()
    if fixtures['production_input_sha256'] != sha: raise ValueError('Production input source changed')
    anchor_q = capture['first_feedback']['decoded']['joints_rad']
    fit = [row for row in capture['rows'] if row['request']['split']=='fit']
    held = [row for row in capture['rows'] if row['request']['split']=='holdout']
    if not held: raise ValueError('Independent holdout missing')
    model = LocalKinematics.fit([r['joints_rad'] for r in fit], [r['pose_mm_deg'] for r in fit],
                               anchor_q, capture['anchor_pose_mm_deg'], math.radians(10))
    errors = []
    for row in held:
        p, r = model.fk(row['joints_rad']); pp, rr = pose(row['pose_mm_deg'])
        errors.append(dict(sample_index=row['index'], position_mm=float(np.linalg.norm(p-pp)*1000),
                           rotation_deg=float(np.degrees(angle(r, rr))), joints_rad=row['joints_rad']))
    (directory/'holdout_errors.json').write_text(json.dumps(errors, indent=2), encoding='utf-8')
    profile = model.serializable(); profile['production_input_sha256'] = sha
    profile['capture_file'] = str(capture_path)
    (directory/'local_model.json').write_text(json.dumps(profile, indent=2), encoding='utf-8')
    cases = []; durations = []; requests = []; failures = []
    with (directory/'replay_frames.jsonl').open('w', encoding='utf-8') as trace:
        for index, row in enumerate(fixtures['rows']):
            clock = Clock(); device = LocalDevice(model, clock); core = ControlCore(device, 'shadow', clock=clock); core.connect()
            def send(value):
                v = copy.deepcopy(value); v.update(session=core.owner, lease=core.lease, seq=core.seq+1)
                return core.command(v, device.state(), clock())
            neutral = copy.deepcopy(fixtures['neutral'][row['source']]); neutral.update(mode=row['mode'], frame=row['frame'])
            send(neutral); core.command(dict(session=core.owner, operation='arm'), device.state(), clock())
            last = None; failure = None
            for tick in range(20):
                send(row['input']); clock.step(); start = time.perf_counter_ns()
                result = core.tick(device.state(), {'rgb':dict(x5_receive_monotonic_ns=clock(),test_fixture=True)})
                elapsed = (time.perf_counter_ns()-start)/1e6; durations.append(elapsed)
                # Test fixture, no network. These timings cannot be relabelled
                # as the original full hardware control transaction.
                trace.write(json.dumps(dict(case=index,tick=tick,input=row['input'],candidate=result['candidate_action'],
                                            ik_result=result['ik_result'],actual=device.state(),local_cpu_ms=elapsed,
                                            test_fixture=True,eligible_for_hardware_acceptance=False))+'\n')
                if not core.armed or result['candidate_action'] is None:
                    failure = core.reason; failures.append(dict(case=index,reason=failure)); break
                last = result['candidate_action']
            if last is not None:
                q_before = copy.deepcopy(core.candidate_q); grip_before = core.grip_target
                send(neutral); clock.step(); released = core.tick(device.state(), {'rgb':dict(x5_receive_monotonic_ns=clock())})
                if released['effective_action'] or not np.array_equal(q_before,core.candidate_q) or grip_before != core.grip_target:
                    raise AssertionError('Release did not preserve local targets')
            cases.append(dict(index=index, **{k:row[k] for k in ('source','mode','frame','fine','axis','sign')}, passed=failure is None, failure=failure))
            if failure is None and row['axis'] != 3:
                requests.append(dict(case=index,joints_rad=last['joints_rad'], target_pose_mm_deg=last['requested_tcp']['controller_pose_mm_deg'],
                                     source=row['source'],mode=row['mode'],frame=row['frame'],fine=row['fine']))
    # Keep the official URDF mismatch visible; it is neither aligned nor
    # used to derive this controller-sampled polynomial.
    from e6_teleop.model import E6Model
    nominal = E6Model(); p, r = nominal.pose(model.anchor_q); nominal.close()
    summary = dict(experiment_only=True,test_fixture=True,eligible_for_hardware_acceptance=False,
                   physical_accuracy_verified=False,hardware_shadow_matrix_passed=False,
                   fit_samples=len(fit),holdout_samples=len(held),fit_condition=model.fit_condition,
                   holdout_position_mm=percentiles([v['position_mm'] for v in errors]),
                   holdout_rotation_deg=percentiles([v['rotation_deg'] for v in errors]),
                   sample_accuracy_check_passed=all(v['position_mm']<=1 and v['rotation_deg']<=1 for v in errors),
                   production_input_sha256=sha,cases=cases,mapping_cases_passed=sum(c['passed'] for c in cases),
                   local_cpu_ms=percentiles(durations),local_cpu_over_50ms=sum(v>50 for v in durations),
                   failures=failures,controller_audit_requests=len(requests),
                   nominal_urdf_anchor_difference=dict(position_mm=float(np.linalg.norm(p-model.anchor_p)*1000),
                                                       rotation_deg=float(np.degrees(angle(r,model.anchor_r)))),
                   physical_motion_tx=0,physical_stop_tx=0,physical_can_tx=0)
    (directory/'audit_requests.json').write_text(json.dumps(requests, indent=2), encoding='utf-8')
    (directory/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='cases'}, indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();evaluate(a.capture,a.fixtures,a.evidence)
