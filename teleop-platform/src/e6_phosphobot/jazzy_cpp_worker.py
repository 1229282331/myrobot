"""Owned official Servo C++ subprocess. No E6 SDK, CAN or actuation sockets."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import yaml
from .cpp_cycle import CppCycle
from .latest_pipe import LatestJsonPipe
from .ipc_snapshot import SnapshotPublisher, solver_wait_seconds
from .moveit_config import load_jazzy_bundle
from .motion_profiles import get_profile
from .solver_timing import configure_timing,cycle_options,verify_timing_reply
from .delivery_timing import configure_delivery


def verified_binary(bundle):
    bundle = bundle.resolve()
    manifest = json.loads((bundle / 'cpp-adapter.json').read_text())
    binary = (bundle / manifest['binary']).resolve()
    if not binary.is_relative_to(bundle) or not binary.is_file():
        raise ValueError('Missing/outside official C++ adapter')
    if hashlib.sha256(binary.read_bytes()).hexdigest() != manifest['sha256']:
        raise ValueError('Official C++ adapter checksum mismatch')
    return binary, manifest


def verify_dynamic_protocol(manifest):
    if manifest.get('dynamic_gripper_version')!=3 or manifest.get('gripper_hold_protocol')!='G3':
        raise ValueError('C++ binary does not attest G3 gripper hold and collision support')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--device-period', type=float, default=.05)
    parser.add_argument('--motion-profile', default='commissioning')
    parser.add_argument('--tcp-profile', default='flange')
    parser.add_argument('--collision-attachments',action='store_true')
    parser.add_argument('--writer-paced',action='store_true')
    args = parser.parse_args()
    args.session.mkdir(parents=True, exist_ok=True)
    params, profile = load_jazzy_bundle(args.bundle, args.device_period, args.motion_profile, args.tcp_profile,args.collision_attachments)
    params['moveit_servo'].update(publish_period=.05, max_expected_latency=.2)
    params['update_period'] = .05
    binary, manifest = verified_binary(args.bundle)
    timing=configure_timing(params,manifest)
    delivery=configure_delivery(manifest,args.writer_paced)
    path = args.session / 'params.yaml'
    path.write_text(yaml.safe_dump({'/e6_cpp_adapter': {'ros__parameters': params}}))
    if args.tcp_profile!='flange' and manifest.get('tcp_frames_version')!=1:
        raise ValueError('C++ binary does not attest configurable TCP support')
    dynamic='e6_gripper_joint_names' in params
    if dynamic:verify_dynamic_protocol(manifest)
    (args.session / 'cpp-adapter.json').write_text(json.dumps(manifest, indent=2))
    halt = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: halt.__setitem__(0, True))
    pipe = LatestJsonPipe(sys.stdin.fileno(), sys.stdout.fileno())
    publisher = SnapshotPublisher(pipe)
    profile_description = get_profile(args.motion_profile).description()
    cycle = CppCycle(tcp_profile=args.tcp_profile,
        input_ramp_time_constant_s=get_profile(args.motion_profile).input_ramp_time_constant_s,dynamic_gripper=dynamic,
        orientation_assist=manifest.get('orientation_assist_version')==1,writer_paced=args.writer_paced,
        delivery_timing=delivery,
        **cycle_options(timing))
    cycle.info['timing_profile']=timing
    process = None
    native_logs = None
    result = dict(error=None, api='cpp')
    from .operator_logs import NativeLogDrain, start_logging
    logger,human_sink=start_logging(args.session/'servo.log','e6.servo')
    logger.info('Official MoveIt Servo starting; profile=%s; TCP=%s; period=%s s; binary=%s',
        profile_description,profile['tcp'],args.device_period,manifest['sha256'])
    last_health=None
    last_rejection=None
    try:
        process = subprocess.Popen([str(binary), '--ros-args', '--disable-external-lib-logs', '--params-file', str(path)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   env=dict(os.environ, RCUTILS_LOGGING_USE_STDOUT='0'))
        native_logs = NativeLogDrain(process.stderr, human_sink)
        child = LatestJsonPipe(process.stdout.fileno(), process.stdin.fileno(), limit=16384)
        while not halt[0]:
            if process.poll() is not None:
                raise RuntimeError('Official C++ adapter exited: ' + str(process.returncode))
            now = time.perf_counter_ns()
            incoming = pipe.receive()
            if incoming:
                cycle.update(incoming)
            reply = child.receive()
            if reply:
                verify_timing_reply(reply,timing)
                cycle.complete(reply, time.perf_counter_ns())
            request = cycle.request(time.perf_counter_ns())
            if request:
                data = request.encode('ascii')
                # One request in flight, bounded far below PIPE_BUF. A
                # blocked writer is a broken adapter, never a retry queue.
                if os.write(process.stdin.fileno(), data) != len(data):
                    raise RuntimeError('Partial official solver request')
            if cycle.pending:
                limit = 30_000_000_000 if not cycle.initialized else 2_000_000_000
                if now - cycle.pending['started_ns'] > limit:
                    raise TimeoutError('Official solver response missing')
            state = cycle.snapshot(time.perf_counter_ns())
            health=(state['info'].get('ready'),state['info'].get('phase'),state['info'].get('status'),state['info'].get('error'))
            if health!=last_health:
                logger.info('Servo ready / phase / status / reason: %s',health);last_health=health
            rejected=state['info'].get('last_rejection_detail')
            if rejected is not None and rejected!=last_rejection:
                emit=logger.debug if rejected['reason']=='released_or_expired_input' else logger.warning
                emit('Solver candidate rejected: %s',rejected)
                last_rejection=rejected
            state['info'].update(api='cpp', urdf_sha256=profile['urdf_sha256'],
                                 assembly_id=profile.get('assembly_id'),
                                 servo_pid=process.pid, cpp_binary_sha256=manifest['sha256'],
                                 motion_profile=profile_description,tcp=profile['tcp'])
            publisher.update(state, time.perf_counter_ns())
            # A reply or new input wakes this owner immediately. The
            # timeout also services health/partial writes and shutdown.
            select.select([pipe.read_fd, child.read_fd], [], [],
                          solver_wait_seconds(cycle, time.perf_counter_ns()))
    except EOFError:
        pass
    except Exception as exc:
        logger.exception('Servo worker failed')
        import traceback
        result.update(error=str(exc), traceback=traceback.format_exc())
        try:
            pipe.offer(dict(info=dict(cycle.info, ready=False, phase='failed', error=str(exc)), candidate=None))
            pipe.flush()
        except Exception:
            pass
        print(result['traceback'], file=sys.stderr)
    finally:
        if process is not None:
            process.stdin.close()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
            process.stdout.close()
            result['servo_exit'] = process.returncode
        if native_logs is not None:result['native_logs']=native_logs.close()
        result['snapshot_publications'] = publisher.publications
        logger.info('Servo worker stopped; child exit=%s',result.get('servo_exit'))
        logger.removeHandler(human_sink);human_sink.close()
        result['human_logs']=dict(dropped=human_sink.dropped,write_errors=human_sink.write_errors,error=human_sink.error,writer_alive=human_sink.thread.is_alive())
        (args.session / 'exit.json').write_text(json.dumps(result, indent=2))
    return int(result['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
