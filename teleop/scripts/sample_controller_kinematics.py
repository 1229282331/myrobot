"""X5-only read-only FK sampling, or independent IK/FK target auditing.

One Dashboard owner, passive raw feedback, no camera/CAN or device writes.
One-second query deadlines are diagnostic; production control stays 50 ms.
"""
import argparse
import base64
import json
from pathlib import Path
import socket
import threading
import time
import numpy as np
from e6_observer.control_guardian import DashboardLink
from e6_observer.control_core import BOUNDS
from e6_observer.protocol import FeedbackParser


class ReadOnlyLink(DashboardLink):
    def _query(self, cmd, deadline=None):
        if cmd not in ('RobotMode()', 'GetErrorID()') and not cmd.startswith(('PositiveKin(', 'InverseKin(')):
            raise ValueError('Read-only experiment refuses all device writes')
        return super()._query(cmd, deadline)


def run(requests, directory, ip='192.168.1.4', expected_mode=4, allow_length_envelope=False):
    if expected_mode not in (4,5):raise ValueError('Sampling requires a declared normal idle mode')
    directory.mkdir(parents=True, exist_ok=False)
    done = threading.Event(); lock = threading.Lock(); latest = None; error = []
    count = 0; previous = None; max_gap = 0.; first = None

    def reader():
        nonlocal latest, count, previous, max_gap
        parser = FeedbackParser(extended=True,allow_length_envelope=allow_length_envelope)
        with (directory/'feedback.bin').open('wb') as raw_file, (directory/'receives.jsonl').open('w') as events:
            try:
                with socket.create_connection((ip, 30004), timeout=2) as stream:
                    stream.settimeout(.1)
                    while not done.is_set():
                        try: data = stream.recv(16384)
                        except socket.timeout: continue
                        now = time.perf_counter_ns()
                        if not data: parser.eof(); raise ConnectionError('Feedback disconnected')
                        raw_file.write(data)
                        events.write(json.dumps(dict(x5_receive_ns=now, bytes=len(data)))+'\n')
                        if previous is not None: max_gap = max(max_gap, (now-previous)/1e6)
                        previous = now
                        packets=parser.feed(data)
                        for index,(raw, decoded) in enumerate(packets):
                            value = dict(decoded=decoded, x5_receive_monotonic_ns=now, raw_base64=base64.b64encode(raw).decode())
                            if parser.last_transport_prefixes[index]:value['wire_prefix_base64']=base64.b64encode(parser.last_transport_prefixes[index]).decode()
                            with lock: latest = value; count += 1
            except Exception as exc:
                error.append(str(exc)); done.set()

    def fresh():
        with lock: state = latest
        if error: raise RuntimeError(error[0])
        if state is None or time.perf_counter_ns()-state['x5_receive_monotonic_ns'] > 100_000_000:
            raise ValueError('Read-only experiment needs fresh feedback within 100 ms')
        d = state['decoded']
        if (d['robot_mode'] != expected_mode or d['enable_status'] != int(expected_mode==5)
                or d['error_status'] or d['collision_state'] or d['run_queued_command']):
            raise ValueError('Read-only sampling requires the declared idle/enable state')
        if d['user_index'] or d['tool_index'] or any(abs(v)>1e-8 for v in d['user_configuration_raw_mm_deg']+d['tool_configuration_raw_mm_deg']):
            raise ValueError('Experiment requires existing zero User0/Tool0; never changes them')
        if first and np.max(np.abs(np.asarray(d['joints_rad'])-first['decoded']['joints_rad'])) > np.radians(.1):
            raise ValueError('Actual robot posture changed during read-only experiment')
        return state

    thread = threading.Thread(target=reader, daemon=True); thread.start()
    link = None; rows = []; failure = None; anchor = None; anchor_ns = started = None
    begin = time.perf_counter_ns()
    try:
        until = time.monotonic()+3
        while latest is None:
            if done.wait(.01) or time.monotonic()>until: raise RuntimeError('Passive feedback not ready')
        first = fresh(); link = ReadOnlyLink(ip)
        # Enabled sampling never enables the robot itself. Confirm five seconds
        # of stationary feedback before making even virtual FK requests.
        if expected_mode==5:
            stable=[];until=time.monotonic()+5
            while time.monotonic()<until:
                state=fresh();stable.append(state['decoded']['joints_rad']);done.wait(.05)
            if np.max(np.ptp(stable,axis=0))>np.radians(.03):raise ValueError('Enabled calibration needs stable holding within .03 degrees')
        q0 = np.asarray(first['decoded']['joints_rad'])
        started = time.perf_counter_ns(); anchor = link.fk(np.degrees(q0), started+1_000_000_000)
        anchor_ns = time.perf_counter_ns()
        for index, request in enumerate(requests):
            state = fresh(); q = q0+np.radians(request['delta_deg']) if 'delta_deg' in request else np.asarray(request['joints_rad'])
            if q.shape != (6,) or not np.isfinite(q).all() or np.max(np.abs(q-q0))>np.radians(10)+1e-8 or np.any(np.abs(q)>BOUNDS-np.radians(5)):
                raise ValueError('Virtual query outside declared joint domain')
            start = time.perf_counter_ns(); returned = None
            if request.get('target_pose_mm_deg') is not None:
                returned = link.ik(request['target_pose_mm_deg'], state['decoded']['q_actual_deg'], start+1_000_000_000)
            fk_start = time.perf_counter_ns()
            result = link.fk(np.degrees(q), fk_start+1_000_000_000)
            end = time.perf_counter_ns()
            rows.append(dict(index=index, request=request, joints_rad=q.tolist(), pose_mm_deg=result,
                             controller_ik_joints_deg=returned, input_feedback=state,
                             x5_query_started_ns=start, x5_reply_ns=end, fk_ms=(end-fk_start)/1e6,
                             pair_ms=(end-start)/1e6, events=[dict(e) for e in list(link.events)[-2 if returned else -1:]]))
            if index%100==0: print(json.dumps(dict(samples=len(rows),stage='read-only')), flush=True)
            done.wait(max(0, .05-(time.perf_counter_ns()-start)/1e9))
        fresh()
    except Exception as exc: failure = str(exc)
    finally:
        final = latest; done.set(); thread.join(3)
        if link: link.close()
        result = dict(experiment_only=True, eligible_for_hardware_acceptance=False, physical_accuracy_verified=False,
                      robot_ip=ip, query_deadline_ms=1000, production_transaction_limit_ms=50,
                      expected_idle_mode=expected_mode,
                      first_feedback=first, last_feedback=final, feedback_count=count, receive_gap_max_ms=max_gap,
                      anchor_pose_mm_deg=anchor, anchor_query_ms=(anchor_ns-started)/1e6 if anchor_ns is not None else None,
                      rows=rows, failure=failure, duration_s=(time.perf_counter_ns()-begin)/1e9,
                      motion_tx=link.motion_tx if link else 0, stop_tx=link.stop_tx if link else 0, can_tx=0)
        (directory/'samples.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({k:result[k] for k in ('failure','duration_s','feedback_count','receive_gap_max_ms','motion_tx','stop_tx','can_tx')}), flush=True)
    if failure: raise RuntimeError(failure)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--requests', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--expected-mode',type=int,choices=(4,5),default=4,help='Mode5 only reads an already enabled, stable robot; never enables or moves')
    parser.add_argument('--length-envelope',action='store_true',help='Opt in to the observed exact duplicated-length transport envelope')
    args = parser.parse_args()
    run(json.loads(args.requests.read_text()), args.evidence,expected_mode=args.expected_mode,allow_length_envelope=args.length_envelope)
