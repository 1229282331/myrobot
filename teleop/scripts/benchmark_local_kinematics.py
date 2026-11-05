"""X5 read-only CPU benchmark with real camera/feedback acquisition load.

Fixtures run locally on X5, not through PC/control SSH. ControlCore remains
shadow; every device write raises. No hardware acceptance can be unlocked.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
from e6_observer.control_core import ControlCore
from e6_observer.control_math import controller_pose, pose
from local_kinematics import LocalKinematics


class ModelDevice:
    def __init__(self, model): self.model = model
    def ik(self, raw, seed, deadline):
        p, r = pose(raw); q, _ = self.model.solve(p, r, np.radians(seed))
        return np.degrees(q).tolist()
    def fk(self, q, deadline):
        p, r = self.model.fk(np.radians(q)); return controller_pose(p, r, [0]*6)
    def servo(self, *args): raise AssertionError('Read-only benchmark refuses motion')
    def stop(self, *args): raise AssertionError('Read-only benchmark refuses Stop')
    def grip(self, *args): raise AssertionError('Read-only benchmark refuses CAN')


def run(profile_path, fixtures_path, source_path, directory, ticks=16):
    from e6_observer.runtime import Hub
    profile = json.loads(profile_path.read_text()); fixtures = json.loads(fixtures_path.read_text())
    if profile.get('experiment_only') is not True or profile.get('eligible_for_hardware_acceptance') is not False:
        raise ValueError('Only explicitly read-only experiment models accepted')
    expected = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if fixtures['production_input_sha256'] != expected or profile['production_input_sha256'] != expected:
        raise ValueError('Production input source mismatch')
    model = LocalKinematics(profile['anchor_joints_rad'],profile['anchor_pose_mm_deg'],profile['joint_span_rad'],profile['coefficients'])
    device = ModelDevice(model)
    directory.mkdir(parents=True, exist_ok=False)
    # Strict observation hub: only its fixed four Dashboard queries, passive
    # feedback, camera acquisition and CAN interface inspection are active.
    hub = Hub('192.168.1.4','/root/remaining-issues-20261002_2226/tools','read-only-local-benchmark')
    cases = []; durations = []; source_ages = []; failure = None; skipped = 0
    first_q = None; sampled_q = []; begin = time.perf_counter_ns(); final_stats = None; warmup = []

    def fresh():
        nonlocal first_q
        with hub.lock:
            state = hub.latest.get('robot'); media = {k:dict(v) for k,v in hub.latest.items() if k in ('rgb','depth')}
            # Avoid copying every acquisition diagnostic under the camera's
            # publish lock for each input and tick. Only required faults are
            # examined here; complete diagnostics are captured once at exit.
            stats = {k:hub.stats.get(k) for k in ('fatal','robot_error','rgb_error')}
        if stats.get('fatal') or stats.get('robot_error') or stats.get('rgb_error'):
            raise RuntimeError('Required real acquisition failed: '+str(stats))
        if state is None or 'rgb' not in media: raise ValueError('Real feedback and RGB not ready')
        d = state['decoded']; q = np.asarray(d['joints_rad'])
        if d['robot_mode'] != 4 or d['enable_status']: raise ValueError('Benchmark requires robot to remain disabled')
        if first_q is None: first_q = q
        if np.max(np.abs(q-first_q)) > np.radians(.1): raise ValueError('Actual posture changed during read-only benchmark')
        return state, media

    try:
        hub.start(); until = time.monotonic()+12
        while True:
            try: fresh(); break
            except ValueError:
                if time.monotonic()>until: raise
                time.sleep(.05)
        # Prepare numerical kernels before starting a control loop. No
        # hardware command is sent; later cycles keep the full 50 ms budget.
        for axis in range(6):
            target_p, target_r = model.fk(model.anchor_q)
            if axis<3: target_p[axis] += .0001
            start = time.perf_counter_ns(); model.solve(target_p,target_r,model.anchor_q)
            warmup.append((time.perf_counter_ns()-start)/1e6)
        until = time.monotonic()+5
        while time.monotonic()<until: fresh(); time.sleep(.05)
        with (directory/'frames.jsonl').open('w', encoding='utf-8') as trace:
            for index, row in enumerate(fixtures['rows']):
                core = ControlCore(device,'shadow'); core.connect()
                def send(value):
                    state, media = fresh(); msg = copy.deepcopy(value)
                    msg.update(session=core.owner,lease=core.lease,seq=core.seq+1)
                    return core.command(msg,state,media['rgb']['x5_receive_monotonic_ns'])
                neutral = copy.deepcopy(fixtures['neutral'][row['source']]); neutral.update(mode=row['mode'],frame=row['frame'])
                send(neutral); state, media = fresh()
                core.command(dict(session=core.owner,operation='arm'),state,media['rgb']['x5_receive_monotonic_ns'])
                deadline = time.perf_counter()+.05; previous = None
                for tick in range(ticks):
                    time.sleep(max(0,deadline-time.perf_counter()))
                    send(row['input']); state, media = fresh(); now = time.perf_counter_ns()
                    result = core.tick(state,media); end = time.perf_counter_ns(); elapsed = (end-now)/1e6
                    durations.append(elapsed); sampled_q.append(state['decoded']['joints_rad'])
                    ages = dict(robot_ms=(now-state['x5_receive_monotonic_ns'])/1e6,rgb_ms=(now-media['rgb']['x5_receive_monotonic_ns'])/1e6)
                    source_ages.append(ages)
                    trace.write(json.dumps(dict(case=index,tick=tick,input=row['input'],pre_action_robot=state,
                        candidate=result['candidate_action'],ik_result=result['ik_result'],effective_action=None,
                        source_ages=ages,media={k:{n:v for n,v in sample.items() if n in ('device_frame_id','x5_receive_monotonic_ns','original_sha256')} for k,sample in media.items()},
                        x5_cycle_started_ns=now,local_cpu_ms=elapsed,
                        period_ms=None if previous is None else (now-previous)/1e6,
                        experiment_only=True,eligible_for_hardware_acceptance=False))+'\n')
                    previous = now
                    if not core.armed or result['candidate_action'] is None: raise RuntimeError('Local candidate rejected: '+core.reason)
                    if elapsed>50: raise RuntimeError('X5 local CPU exceeded 50 ms; no write sent')
                    deadline += .05
                    if time.perf_counter()>deadline: skipped += 1; deadline = time.perf_counter()+.05
                q_before = core.candidate_q.copy(); grip_before = core.grip_target
                send(neutral); time.sleep(max(0,deadline-time.perf_counter())); state, media = fresh(); released = core.tick(state,media)
                if not core.armed: raise RuntimeError('Release observation failed: '+core.reason)
                if not np.array_equal(q_before,core.candidate_q) or grip_before!=core.grip_target or released['effective_action']:
                    raise AssertionError('Release failed to preserve candidate target')
                cases.append(dict(index=index,**{k:row[k] for k in ('source','mode','frame','fine','axis','sign')},passed=True))
                if index%16==0: print(json.dumps(dict(cases=len(cases),cycles=len(durations),max_local_cpu_ms=max(durations))),flush=True)
    except Exception as exc: failure = str(exc)
    finally:
        with hub.lock: final_stats = copy.deepcopy(hub.stats)
        hub.close()
        distribution = lambda values: dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).tolist())) if values else {}
        result = dict(passed=failure is None,failure=failure,experiment_only=True,eligible_for_hardware_acceptance=False,
            scope='X5 local fixture inputs + production ControlCore shadow + real RGB/feedback; not PC/SSH control chain',
            duration_s=(time.perf_counter_ns()-begin)/1e9,cases=cases,cycles=len(durations),
            local_cpu_ms=distribution(durations),source_age_ms={k:distribution([v[k] for v in source_ages]) for k in ('robot_ms','rgb_ms')},
            warmup_cpu_ms=warmup,skipped_periods=skipped,joint_peak_to_peak_deg=np.degrees(np.ptp(sampled_q,axis=0)).tolist() if sampled_q else None,
            final_acquisition_stats=final_stats,model_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
            physical_motion_tx=0,physical_stop_tx=0,physical_can_tx=0,physical_accuracy_verified=False)
        (directory/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('cases','final_acquisition_stats')},indent=2),flush=True)
    if failure: raise RuntimeError(failure)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--profile',type=Path,required=True);p.add_argument('--fixtures',type=Path,required=True)
    p.add_argument('--input-source',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();run(a.profile,a.fixtures,a.input_source,a.evidence)
