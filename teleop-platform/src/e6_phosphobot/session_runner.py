"""X5 resource supervisor. No input integration or device commands here.

SSH stdin EOF ends this session. Gateway owns Stop/feedback; it exits before
camera and the owned ROS /proc mount are released. No process-name killing.
"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

DEFAULT_SERVO_DURATION_S = .1


def atomic_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(path)


def verified_files(root, manifest):
    root = Path(root).resolve()
    for name, expected in manifest.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError('Missing/outside release artifact: ' + name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Artifact checksum differs: ' + name)


def servo_gain_option(config):
    """Carry the already-supported SDK qualification value through the launcher."""
    gain = config.get('servo_gain', 500)
    if type(gain) not in (int, float) or gain not in (500, 750):
        raise ValueError('Servo gain qualification must be 500 or 750')
    return ['--servo-gain', str(int(gain))]


def servo_aheadtime_option(config):
    """Explicit per-command SDK setting; the default remains 50."""
    aheadtime = config.get('servo_aheadtime', 50)
    if type(aheadtime) not in (int, float) or aheadtime not in (20, 50):
        raise ValueError('Servo aheadtime qualification must be 20 or 50')
    return ['--servo-aheadtime', str(int(aheadtime))]


def servo_duration_option(config):
    """Use the followed t=0.1 profile; retain t=0.05 only for shadow evidence."""
    duration = config.get('servo_duration_s', DEFAULT_SERVO_DURATION_S)
    if type(duration) not in (int, float) or duration not in (.05, .1):
        raise ValueError('Servo duration qualification must be 0.05 or 0.1 s')
    if config.get('stage') == 'live' and duration != DEFAULT_SERVO_DURATION_S:
        raise ValueError('Live control requires Servo t=0.1 s; t=0.05 is historical shadow-only configuration')
    return ['--servo-duration-s', str(float(duration))]


def memory_residency_option(config):
    mode=config.get('memory_residency','off')
    if mode not in ('off','current'):raise ValueError('Unknown memory residency mode')
    return ['--memory-residency',mode]


def payload_option(config):
    from .payload import Payload
    return ['--payload', json.dumps(Payload.from_config(config.get('payload')).description())]


def preview_measurement_option(config):
    marker = config.get('latency_marker', False)
    if type(marker) is not bool:
        raise ValueError('latency_marker must be boolean')
    return ['--latency-marker'] if marker else []


def preview_codec_option(config):
    codec=config.get('preview_codec','h264')
    if codec not in ('h264','vp8'):raise ValueError('Unknown preview codec')
    return ['--preview-codec',codec]


def workspace_option(config):
    from .workspace import TestWorkspace
    return ['--test-workspace', json.dumps(TestWorkspace.from_config(config.get('test_workspace')).description())]


def motion_engine_option(config):
    """Startup-fixed route; native commissioning does not require a ROS model."""
    engine = config.get('motion_engine', 'moveit_servo')
    from .tool_frame import get_tool
    tool=get_tool(config.get('tcp_profile','flange'))
    if tool.name!='flange' and engine!='moveit_servo':
        raise ValueError('Grasp TCP requires MoveIt')
    if engine not in ('moveit_servo', 'native_servop'):
        raise ValueError('Unknown motion engine')
    api=config.get('moveit_api','ros')
    if api not in ('ros','cpp') or (engine!='moveit_servo' and api!='ros'):
        raise ValueError('C++ API requires MoveIt route')
    smoothing=config.get('native_smoothing','none')
    if smoothing not in ('none','ruckig') or (engine!='native_servop' and smoothing!='none'):
        raise ValueError('Native smoothing requires native ServoP route')
    if engine == 'native_servop':
        if config.get('motion_profile', 'commissioning') != 'commissioning' and smoothing!='ruckig':
            raise ValueError('Native ServoP higher than commissioning requires Ruckig input smoothing')
        return []
    result=['--moveit-bundle', str(Path(config['rootfs'])/'qualification')]
    if tool.name!='flange':result+=['--tcp-profile',tool.name]
    if api!='ros':result+=['--moveit-api',api]
    return result


def runtime_artifacts(config):
    result=dict(config.get('runtime_manifest',{}))
    if config.get('moveit_api','ros')=='cpp':
        extra=config.get('cpp_runtime_manifest',{})
        if not extra or 'qualification/cpp-adapter.json' not in extra:
            raise ValueError('C++ runtime artifact manifest required')
        if not any(name.endswith('/servo_cpp_adapter') for name in extra):
            raise ValueError('C++ binary artifact manifest required')
        result.update(extra)
    return result


def close_child(process, timeout=15):
    """Signal only our Popen child, never escalate a stuck device owner to kill."""
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f'Owned PID {process.pid} has not exited; resources retained') from exc


def check_children(children, result, event):
    for name, process in children.items():
        code = process.poll()
        if code is None:
            continue
        if name != 'camera_service':
            raise RuntimeError(f'{name} exited unexpectedly ({code})')
        # RGB freshness suspends controls. A capture child exit must not kill
        # the independent feedback/Stop owner or close the operator page.
        if 'camera_failure' not in result:
            result['camera_failure'] = dict(pid=process.pid, exit_code=code)
            event('camera_unavailable', pid=process.pid, exit_code=code,
                  reason='Feedback and Stop owner retained; capture requires recovery')


def adopt_descendants():
    # Linux PR_SET_CHILD_SUBREAPER. If a gateway is killed, its ROS/encoder
    # children become OUR children, allowing ownership-safe cleanup/reaping.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0):
        raise OSError(ctypes.get_errno(), 'Cannot become session subreaper')


def close_adopted():
    from .jazzy_host import finish_owned_group
    direct = []
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[1]) == os.getpid():
                direct.append((int(path.parent.name), int(fields[2]), fields[0]))
        except (OSError, ValueError, IndexError):
            continue
    for pid, pgid, state in direct:
        if state != 'Z':
            if pgid <= 1 or pgid == os.getpgrp():
                # All managed gateway/camera/Jazzy children start a dedicated
                # group. An unfamiliar child is retained for diagnosis.
                raise RuntimeError(f'Unexpected adopted process group for PID {pid}')
            finish_owned_group(pgid)
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass
    # Killing a whole owned group can orphan another generation of children.
    # They are now reparented here too; reap its zombies without touching any
    # external PID. Live descendants are reported instead of silently ignored.
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if pid == 0:
                raise RuntimeError('Owned descendants remain alive after cleanup')
        except ChildProcessError:
            break


def run(config, *, stdin=sys.stdin):
    import fcntl
    runtime = Path(config['runtime'])
    runtime.mkdir(parents=True, exist_ok=False)
    halted = threading.Event()
    reason = ['stdin_closed']
    children = {}
    logs = []
    mounted = False
    lock = None
    cpu_policy = None
    wifi_policy = None
    result = dict(session=config['session'], stage=config['stage'], clean=False,
                  boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(), children={})

    def event(kind, **values):
        row = dict(event=kind, at_ns=time.monotonic_ns(), **values)
        with (runtime/'lifecycle.jsonl').open('a', encoding='utf-8') as out:
            out.write(json.dumps(row, ensure_ascii=False)+'\n')
        try:
            print(json.dumps(row, ensure_ascii=False), flush=True)
        except (BrokenPipeError, OSError):
            halted.set()

    def request_stop(why):
        reason[0] = why
        halted.set()

    def watch_input():
        try:
            for line in stdin:
                if line.strip() == 'stop':
                    request_stop('requested'); return
        finally:
            halted.set()

    def launch(name, args, env):
        log = (runtime/(name+'.log')).open('ab'); logs.append(log)
        process = subprocess.Popen([sys.executable, '-B', '-m', 'e6_phosphobot.'+name, *args],
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=log, env=env,
                                   start_new_session=True)
        children[name] = process
        result['children'][name] = dict(pid=process.pid, argv=args)
        atomic_json(runtime/'owner.json', result)
        event('child_started', name=name, pid=process.pid)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda sig, _: request_stop(signal.Signals(sig).name))
    threading.Thread(target=watch_input, name='ssh-session-input', daemon=True).start()
    try:
        adopt_descendants()
        if config['stage'] not in ('shadow', 'live'):
            raise ValueError('Unknown control stage')
        from .motion_profiles import get_profile
        motion_profile = get_profile(config.get('motion_profile', 'commissioning')).name
        engine_options = motion_engine_option(config)
        result['motion_engine'] = config.get('motion_engine', 'moveit_servo')
        servo_options = servo_gain_option(config) + servo_aheadtime_option(config) + servo_duration_option(config) + workspace_option(config) + memory_residency_option(config) + payload_option(config)
        measurement_options = preview_measurement_option(config)+preview_codec_option(config)
        result['motion_profile'] = motion_profile
        lock = open('/run/e6-phosphobot-session.lock', 'a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        release = Path(config['release'])
        verified_files(release, config['source_manifest'])
        rootfs = Path(config['rootfs']).resolve()
        if engine_options:
            verified_files(rootfs, runtime_artifacts(config))
        # A second control stack or non-session camera user must be released by
        # its owner. A port/lock check never grants permission to terminate it.
        connections = subprocess.check_output(['ss', '-Htnp'], text=True)
        if any('192.168.1.4:'+p in connections for p in ('29999', '30004')):
            raise RuntimeError('E6 connection already owned')
        video = list(Path('/dev').glob('video*'))
        if video and subprocess.run(['fuser', *map(str, video)], capture_output=True).returncode == 0:
            raise RuntimeError('Camera already owned')
        if subprocess.run(['mountpoint', '-q', str(rootfs/'proc')]).returncode == 0:
            raise RuntimeError('ROS proc mount already owned')
        if engine_options:
            # Immutable source copy inside chroot; original SDK/OS packages untouched.
            import shutil
            child_source = rootfs/'qualification/releases'/config['session']/'python'
            shutil.copytree(release/'e6_phosphobot', child_source/'e6_phosphobot')
            subprocess.run(['mount', '-t', 'proc', 'proc', str(rootfs/'proc'), '-o', 'nosuid,nodev,noexec'], check=True)
            mounted = True
        if halted.is_set():
            return result
        from .cpu_policy import SessionCpuPolicy, configured_governor
        cpu_policy = SessionCpuPolicy(configured_governor(config), event=event)
        result['cpu_policy'] = cpu_policy.apply()
        from .wifi_policy import SessionWifiPolicy
        wifi_policy = SessionWifiPolicy(config, event=event)
        result['wifi_policy'] = wifi_policy.apply()
        env = dict(os.environ, PYTHONPATH=str(release)+':'+config['gateway_dependencies'],
                   PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
                   DCW_SDK_ROOT=config['camera_sdk'], DCW_SDK_CONFIG=config['camera_config'])
        if engine_options:
            env['E6_JAZZY_PYTHON_PATH']='/'+str(child_source.relative_to(rootfs))
        health = Path('/dev/shm')/('e6-platform-'+config['session'])/'health.json'
        camera_args=['--camera-tools', config['camera_tools'], '--output', str(runtime/'camera'),
                '--health-file', str(health), '--rtsp-url', 'rtsp://127.0.0.1:18554/gemini', *measurement_options]
        launch('camera_service',camera_args,env)
        def restart_camera(identity):
            previous=children['camera_service']
            code=close_child(previous)
            # SIGKILL may leave only that camera's encoder in its owned group.
            # Never call the generic adopted-child cleanup while gateway lives.
            from .jazzy_host import finish_owned_group
            finish_owned_group(previous.pid)
            result.setdefault('camera_history',[]).append(dict(result['children']['camera_service'],exit_code=code))
            video=list(Path('/dev').glob('video*'))
            if video and subprocess.run(['fuser',*map(str,video)],capture_output=True).returncode==0:
                raise RuntimeError('Camera still owned after capture child exit')
            # Retain completed request/status files; the new process must not
            # replay an earlier recording start or finish.
            for path in (runtime/'camera/recording.request.json',runtime/'camera/final.json',health.with_name('recording.status.json'),health):
                if path.exists():
                    data=path.read_bytes()
                    destination=runtime/('camera-restart-'+identity+'-'+path.name)
                    with destination.open('xb') as saved:saved.write(data)
                    if destination.read_bytes()!=data:raise RuntimeError('Camera restart archive differs')
                    path.unlink()
            result.pop('camera_failure',None)
            launch('camera_service',[*camera_args,'--resume-output'],env)
        from .camera_restart import CameraRestartSupervisor
        recovery=CameraRestartSupervisor(health,config['session'],restart_camera,
                    lambda:children['camera_service'].poll() is None,event)
        args = ['--stage', config['stage'], '--motion-profile', motion_profile, '--camera-health', str(health),
                '--camera-session',config['session'],
                '--camera-recording-dir', str(runtime/'camera'), '--audit-dir', str(runtime/'audit'),
                '--episodes-dir', str(runtime/'episodes'), *engine_options, *servo_options]
        args.extend(['--native-smoothing',config.get('native_smoothing','none')])
        if config.get('acceptance_inputs'):
            args.append('--acceptance-inputs')
        launch('gateway', args, env)
        event('started', runtime=str(runtime), stage=config['stage'])
        while not halted.wait(.2):
            check_children(children, result, event)
            try:recovery.step()
            except Exception as exc:
                # Recovery IPC or its audit disk can fail independently. Keep
                # the device/feedback owner alive; never turn a camera error
                # into an implicit whole-session shutdown.
                result['camera_restart_error']=str(exc)
    except Exception as exc:
        result['error'] = str(exc)
        event('failed', reason=str(exc))
    finally:
        result['stop_reason'] = reason[0]
        cleanup_errors = []
        # Gateway close keeps its independent feedback alive during Stop.
        for name in ('gateway', 'camera_service'):
            if name in children:
                try:
                    code = close_child(children[name])
                    result['children'][name]['exit_code'] = code
                    if code != 0:
                        cleanup_errors.append(f'{name} exit {code}')
                except Exception as exc:
                    cleanup_errors.append(str(exc))
                    # Retain camera/feedback evidence if the device owner sticks.
                    if name == 'gateway':
                        break
        all_exited = all(p.poll() is not None for p in children.values())
        descendants_exited = all_exited
        if all_exited:
            try:
                close_adopted()
            except Exception as exc:
                cleanup_errors.append(str(exc))
                descendants_exited = False
        if wifi_policy is not None:
            if descendants_exited:
                try:
                    wifi_policy.restore()
                except Exception as exc:
                    cleanup_errors.append('Wi-Fi power saving restore: '+str(exc))
            else:
                cleanup_errors.append('Wi-Fi policy retained while owned children remain')
            result['wifi_policy'] = wifi_policy.snapshot()
        if cpu_policy is not None:
            if descendants_exited:
                try:
                    cpu_policy.restore()
                except Exception as exc:
                    cleanup_errors.append('CPU governor restore: '+str(exc))
            else:
                cleanup_errors.append('CPU governor retained while owned children remain')
            result['cpu_policy'] = cpu_policy.snapshot()
        if mounted and all_exited:
            unmount = subprocess.run(['umount', str(rootfs/'proc')], capture_output=True, text=True)
            if unmount.returncode:
                cleanup_errors.append('Owned proc unmount: '+unmount.stderr.strip())
        if cleanup_errors:
            result['cleanup_errors'] = cleanup_errors
        result['clean'] = not cleanup_errors and all_exited
        result['finished_ns'] = time.monotonic_ns()
        atomic_json(runtime/'result.json', result)
        event('finished', result=result)
        for log in logs:
            log.close()
        if lock:
            lock.close()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    result = run(json.loads(args.config.read_text()))
    raise SystemExit(0 if result['clean'] and not result.get('error') else 1)


if __name__ == '__main__':
    main()
