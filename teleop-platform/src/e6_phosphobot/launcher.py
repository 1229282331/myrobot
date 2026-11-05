"""Deployment/lifecycle entry point; deliberately not another control layer."""
import argparse
import getpass
import hashlib
import json
import logging
import os
from pathlib import Path
import select
import shlex
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
import uuid
import urllib.request
import webbrowser

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def listener_owner(port, expected):
    """An open port alone is not evidence that the right app owns it."""
    import psutil
    owners = {c.pid for c in psutil.net_connections(kind='tcp')
              if c.status == psutil.CONN_LISTEN and c.laddr.port == port}
    if not owners:
        return None
    if len(owners) != 1 or None in owners:
        raise RuntimeError(f'Port {port} has ambiguous ownership')
    process = psutil.Process(owners.pop())
    command = process.cmdline()
    base = Path(process.cwd())
    normalize = lambda x: os.path.normcase(str((base / x).resolve()))
    normalized = [normalize(x) for x in command]
    for item in expected:
        choices = item if isinstance(item, tuple) else (item,)
        if not any(normalize(x) in normalized for x in choices):
            raise RuntimeError(f'Port {port} belongs to another process; refusing takeover')
    return dict(pid=process.pid, created=process.create_time(), command=command)


class LocalServices:
    def __init__(self, runtime, reuse):
        self.runtime, self.reuse = runtime, reuse
        self.children = []; self.logs = []; self.reused = []; self.exits = []

    def ensure(self, port, argv, identity, *, app=False):
        owner = listener_owner(port, identity)
        if owner:
            if not self.reuse:
                raise RuntimeError(f'Port {port} already active; use --reuse-local for verified shared services')
            self.reused.append(dict(port=port, **owner)); return
        log = (self.runtime/f'local-{port}.log').open('wb'); self.logs.append(log)
        flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=log, stderr=log,
                                 cwd=ROOT, creationflags=flags)
        self.children.append((child, app))
        deadline = time.monotonic()+45
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError(f'Local service {port} exited ({child.returncode})')
            try:
                with socket.create_connection(('127.0.0.1', port), .2):
                    if app:
                        with urllib.request.urlopen(f'http://127.0.0.1:{port}/status', timeout=2) as response:
                            status = json.load(response)
                        if 'Dobot E6' not in status.get('robots', []):
                            raise RuntimeError('Formal upstream status does not expose the E6 adapter')
                    return
            except (OSError, TimeoutError):
                time.sleep(.1)
        raise TimeoutError(f'Local service {port} did not become ready')

    def close(self):
        errors = []
        for child, app in reversed(self.children):
            if child.poll() is None:
                if app:
                    child.stdin.close()
                else:
                    child.terminate()  # Only our media child; no device connection.
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    errors.append(f'Local PID {child.pid} has not exited; retained')
            code = child.poll()
            self.exits.append(dict(pid=child.pid, service='app' if app else 'media', exit_code=code))
            if app and code is not None and code != 0:
                errors.append(f'Local app PID {child.pid} exited with {code}')
        for log in self.logs:
            log.close()
        return errors


class Tunnels:
    """Separate authenticated SSH TCP transports for control, files and preview."""
    def __init__(self, connect, *, media_only=False):
        self.clients = []; self.servers = []; self.active = set()
        self.media_only = media_only
        self.lock = threading.Lock(); self.halt = threading.Event()
        try:
            for _ in range(1 if media_only else 3):
                self.clients.append(connect())
            if media_only:
                self.media = self.clients[0]
                return
            self.control, self.media, self.preview = self.clients
            control_socket = self.control.get_transport().sock
            previous = control_socket.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
            # Small, fresh intent messages must not wait for a preceding TCP
            # ACK. Only this latency-sensitive transport changes; independent
            # media/preview transports retain their existing settings.
            control_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            current = control_socket.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
            if current != 1:raise RuntimeError('Control SSH TCP_NODELAY was not applied')
            self.control_network = dict(tcp_nodelay=current, previous_tcp_nodelay=previous)
        except BaseException:
            self.close(); raise

    def relay(self, a, b):
        with self.lock:
            self.active.update((a, b))
        try:
            a.settimeout(2); b.settimeout(2)
            while not self.halt.is_set():
                readable, _, _ = select.select([a, b], [], [], .2)
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    (b if source is a else a).sendall(data)
        except (OSError, EOFError):
            pass
        finally:
            # The owner may close the SSH transport while a relay is unwinding.
            # Paramiko Channel.close then raises EOFError; still release both
            # ends and remove ownership entries, including on repeated close.
            try:
                self.close_stream(a)
            finally:
                try:
                    self.close_stream(b)
                finally:
                    with self.lock:
                        self.active.discard(a); self.active.discard(b)

    @staticmethod
    def close_stream(stream):
        try:stream.close()
        except (OSError, EOFError):pass  # Already disconnected, no bytes to retry.

    def start(self):
        manager = self
        class Server(socketserver.ThreadingTCPServer):
            daemon_threads = True
        ports = [(8777, self.media)] if self.media_only else [(8776, self.control), (8777, self.media)]
        for port, client in ports:
            def make_handler(port, client):
                class Forward(socketserver.BaseRequestHandler):
                    def handle(self):
                        try:
                            channel = client.get_transport().open_channel('direct-tcpip',
                                ('127.0.0.1', port), self.request.getpeername(), timeout=3)
                            manager.relay(self.request, channel)
                        except Exception:
                            self.request.close()
                return Forward
            server = Server(('127.0.0.1', port), make_handler(port, client))
            self.servers.append(server)
            threading.Thread(target=server.serve_forever, daemon=True).start()
        if self.media_only:return
        def reverse(channel, *_):
            try:
                peer = socket.create_connection(('127.0.0.1', 8554), timeout=3)
            except OSError:
                channel.close(); return
            threading.Thread(target=self.relay, args=(channel, peer), daemon=True).start()
        self.preview.get_transport().request_port_forward('127.0.0.1', 18554, handler=reverse)

    def close(self):
        self.halt.set()
        for server in self.servers:
            server.shutdown(); server.server_close()
        with self.lock:
            active=list(self.active)
        # Never hold the ownership lock across Paramiko's potentially blocking
        # channel-close write; relays need it to finish their cleanup.
        for connection in active:
            self.close_stream(connection)
        for client in self.clients:
            self.close_stream(client)


def ssh_factory(config, ask_password=False):
    import paramiko
    ssh = paramiko.SSHConfig()
    path = Path.home()/'.ssh/config'
    if path.exists():
        with path.open() as src:
            ssh.parse(src)
    settings = ssh.lookup(config['ssh_host'])
    password = getpass.getpass('X5 SSH 密码（仅本次内存使用）: ') if ask_password else None
    def connect():
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        known = Path(config.get('known_hosts', str(Path.home()/'.ssh/known_hosts'))).expanduser()
        if known.exists():
            client.load_host_keys(str(known))
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        try:
            client.connect(settings.get('hostname', config['ssh_host']),
                port=int(settings.get('port', 22)), username=settings.get('user', config.get('ssh_user', 'root')),
                key_filename=settings.get('identityfile'), password=password, timeout=10,
                auth_timeout=10, banner_timeout=10)
            client.get_transport().set_keepalive(5)
            return client
        except BaseException:
            client.close(); raise
    return connect


def remote(client, argv):
    stdin, stdout, stderr = client.exec_command(shlex.join(list(map(str, argv))), timeout=40)
    stdin.close()
    out, err = stdout.read().decode(), stderr.read().decode()
    if stdout.channel.recv_exit_status():
        raise RuntimeError(err.strip() or out.strip() or 'Remote operation failed')
    return out


def make_payload():
    lock = json.loads((ROOT/'upstream.lock.json').read_text())
    for key in ('phosphobot', 'dobot_sdk'):
        head = subprocess.check_output(['git', '-C', str(ROOT/lock[key]['path']), 'rev-parse', 'HEAD'], text=True).strip()
        if head != lock[key]['commit']:
            raise RuntimeError('Upstream commit differs: '+key)
    files = {}
    for file in (ROOT/'src/e6_phosphobot').glob('*.py'):
        files['e6_phosphobot/'+file.name] = file.read_bytes()
    assets=ROOT/'src/e6_phosphobot/assets/workspace'
    if assets.is_dir():
        from .assembly_model import verified_assets
        info=verified_assets()
        for name in [*info['files'],'manifest.json']:
            files['e6_phosphobot/assets/workspace/'+name]=(assets/name).read_bytes()
    sdk = ROOT/lock['dobot_sdk']['path']
    names = subprocess.check_output(['git', '-C', str(sdk), 'ls-files', 'dobot_sdk'], text=True).splitlines()
    for name in names:
        if name.endswith('.py'):
            files[name] = (sdk/name).read_bytes()
    manifest = {name: hashlib.sha256(data).hexdigest() for name, data in files.items()}
    if not any(name.startswith('dobot_sdk/') for name in files):
        raise RuntimeError('Official SDK source missing')
    return files, manifest


def deploy(client, config, session, runtime):
    files, manifest = make_payload()
    release = config['remote_base']+'/releases/'+session
    config = dict(config, session=session, release=release, source_manifest=manifest,
                  runtime=config['remote_base']+'/sessions/'+session)
    remote(client, ['mkdir', '-p', release])
    with client.open_sftp() as sftp:
        made = {release}
        for name, data in files.items():
            parent = release
            for part in name.split('/')[:-1]:
                parent += '/'+part
                if parent not in made:
                    sftp.mkdir(parent); made.add(parent)
            with sftp.open(release+'/'+name, 'wb') as out:
                # Paramiko waits for every 32 KiB write otherwise; that adds a
                # wireless round trip for each block of the 100 MB model bundle.
                # close() drains all acknowledgements before the manifest is sent.
                out.set_pipelined(True)
                out.write(data)
        serialized = json.dumps(config, ensure_ascii=False, indent=2)
        with sftp.open(release+'/session.json', 'w') as out:
            out.write(serialized)
    (runtime/'deployment.json').write_text(serialized, encoding='utf-8')
    (runtime/'source-sha256.json').write_text(json.dumps(manifest, indent=2))
    return config


def gateway_ready(value, stage, motion_profile=None, servo_config=None):
    from .session_runner import DEFAULT_SERVO_DURATION_S
    feedback = value.get('feedback') or {}
    age = feedback.get('age_ms')
    engine = (servo_config or {}).get('motion_engine', 'moveit_servo')
    capabilities = value.get('capabilities') or {}
    if servo_config is not None and 'payload' in servo_config:
        from .payload import Payload
        if capabilities.get('payload')!=Payload.from_config(servo_config['payload']).description():return False
    if servo_config is not None and servo_config.get('tcp_profile','flange')!='flange':
        from .tool_frame import get_tool
        if capabilities.get('tcp')!=get_tool(servo_config['tcp_profile']).description():return False
    if engine not in ('moveit_servo', 'native_servop'):
        return False
    if servo_config is not None and capabilities.get('motion_engine') != engine:
        return False
    if servo_config is not None and engine=='moveit_servo' and capabilities.get('moveit_api','ros') != servo_config.get('moveit_api','ros'):
        return False
    if servo_config is not None and capabilities.get('native_smoothing','none') != servo_config.get('native_smoothing','none'):
        return False
    if servo_config is not None:
        residency=capabilities.get('memory_residency') or {'mode':'off'}
        if residency.get('mode') != servo_config.get('memory_residency','off'):
            return False
        if residency.get('mode')=='current' and not (type(residency.get('locked_bytes')) is int and residency['locked_bytes']>0):
            return False
    route_ready = (engine == 'native_servop' or (value.get('model') or {}).get('ready'))
    return bool(value.get('stage') == stage and feedback.get('connected') and
                (motion_profile is None or (value.get('capabilities') or {}).get('motion_profile', {}).get('name') == motion_profile) and
                (servo_config is None or all((value.get('capabilities') or {}).get(k) == servo_config.get(k, default)
                    for k, default in (('servo_gain', 500), ('servo_aheadtime', 50), ('servo_duration_s', DEFAULT_SERVO_DURATION_S)))) and
                (servo_config is None or (value.get('camera') or {}).get('latency_marker',False) == servo_config.get('latency_marker',False)) and
                (servo_config is None or (value.get('camera') or {}).get('preview_codec','h264') == servo_config.get('preview_codec','h264')) and
                (servo_config is None or 'test_workspace' not in servo_config or
                 (value.get('capabilities') or {}).get('test_workspace') == servo_config['test_workspace']) and
                isinstance(age, (int, float)) and 0 <= age < 100 and
                (value.get('camera') or {}).get('valid') and route_ready)


def startup_timeout_seconds(config):
    """Cold initialization is separate from every live feedback deadline."""
    value=config.get('startup_timeout_s',120)
    if type(value) not in (int,float) or not 10 <= value <= 300:
        raise ValueError('startup_timeout_s must be a finite number from 10 to 300')
    return float(value)


def run(config, *, connect=None, reuse=False, open_browser=True, halt=None, duration=None):
    from .session_runner import DEFAULT_SERVO_DURATION_S
    config=dict(config)
    startup_timeout=startup_timeout_seconds(config)
    from .payload import Payload
    config['payload']=Payload.from_config(config.get('payload')).description()
    config.setdefault('servo_duration_s',DEFAULT_SERVO_DURATION_S)
    from .motion_profiles import get_profile
    from .workspace import TestWorkspace
    from .session_runner import servo_gain_option, servo_aheadtime_option, servo_duration_option, preview_measurement_option, preview_codec_option, motion_engine_option, memory_residency_option
    motion_engine_option(config)
    from .session_runner import runtime_artifacts
    config=dict(config,runtime_manifest=runtime_artifacts(config))
    servo_gain_option(config)  # Validate before starting local or remote resources.
    servo_aheadtime_option(config)
    servo_duration_option(config)
    memory_residency_option(config)
    from .cpu_policy import configured_governor
    configured_governor(config)
    from .wifi_policy import SessionWifiPolicy
    SessionWifiPolicy(config)  # Validate before acquiring local/remote resources.
    preview_measurement_option(config)
    preview_codec_option(config)
    config = dict(config, motion_profile=get_profile(config.get('motion_profile', 'commissioning')).name,
                  motion_engine=config.get('motion_engine', 'moveit_servo'),
                  test_workspace=TestWorkspace.from_config(config.get('test_workspace')).description())
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import ConnectionClosed, InvalidHandshake
    session = time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
    runtime = ROOT/'runtime/sessions'/session; runtime.mkdir(parents=True)
    halted = halt or threading.Event()
    local = LocalServices(runtime, reuse)
    tunnels = None; channel = None; stdin = None; stdout = None
    result = dict(session=session, stage=config['stage'], motion_profile=config['motion_profile'], remote_started=False, ready=False)
    summary = None
    logger = logging.getLogger('paramiko')
    ssh_log = logging.FileHandler(runtime/'ssh.log', encoding='utf-8')
    logger.addHandler(ssh_log)
    previous_propagate = logger.propagate; logger.propagate = False
    def emit(message):
        print(message, flush=True)
    try:
        media = ROOT/'runtime/mediamtx/mediamtx.exe'
        if digest(media) != config['mediamtx_executable_sha256']:
            raise RuntimeError('MediaMTX executable checksum differs')
        local.ensure(8554, [str(media), str(ROOT/'config/mediamtx.yml')], [media, ROOT/'config/mediamtx.yml'])
        python = ROOT/'upstream/phosphobot/phosphobot/.venv/Scripts/python.exe'
        local.ensure(8765, [str(python), '-B', '-X', 'utf8', str(ROOT/'scripts/run_pc_owned.py'),
                           '--backend', 'hardware-control'],
                     [(ROOT/'scripts/launch_pc.py', ROOT/'scripts/run_pc_owned.py'),
                      '--backend', 'hardware-control'], app=True)
        tunnels = Tunnels(connect or ssh_factory(config)); tunnels.start()
        result['control_network'] = tunnels.control_network
        deployed = deploy(tunnels.media, config, session, runtime)
        command = ['env', 'PYTHONPATH='+deployed['release'], 'python3', '-B', '-m', 'e6_phosphobot.session_runner',
                   '--config', deployed['release']+'/session.json']
        stdin, stdout, stderr = tunnels.control.exec_command(shlex.join(command))
        channel = stdout.channel
        # Merge only supervisor diagnostics; camera/gateway logs stay on X5.
        channel.set_combine_stderr(True)
        result['remote_started'] = True
        emit(f'正在启动 {config["stage"]}；日志：{runtime}')
        startup_begin=time.monotonic()
        deadline = startup_begin+startup_timeout
        result['startup_timeout_s']=startup_timeout
        last_startup_state=None
        with (runtime/'lifecycle.jsonl').open('w', encoding='utf-8') as logfile:
            def collect():
                nonlocal summary
                for line in stdout:
                    logfile.write(line); logfile.flush()
                    try:
                        value = json.loads(line)
                        if value.get('event') == 'finished':
                            summary = value['result']
                    except ValueError:
                        pass
            reader = threading.Thread(target=collect, daemon=True); reader.start()
            try:
                while not halted.is_set():
                    if channel.exit_status_ready():
                        raise RuntimeError('X5 session ended before requested shutdown')
                    try:
                        with ws_connect('ws://127.0.0.1:8776', open_timeout=2, compression=None) as ws:
                            ws.send(json.dumps({'role': 'observe'}))
                            state = json.loads(ws.recv(timeout=2))
                            last_startup_state=state
                            if gateway_ready(state, config['stage'], config['motion_profile'], config):
                                (runtime/'startup-state.json').write_text(json.dumps(state), encoding='utf-8')
                                result['ready'] = True; break
                    except (OSError, TimeoutError, ConnectionClosed, InvalidHandshake):
                        pass
                    if time.monotonic() >= deadline:
                        (runtime/'startup-last-state.json').write_text(json.dumps(
                            dict(elapsed_s=time.monotonic()-startup_begin,
                                 timeout_s=startup_timeout,state=last_startup_state)),
                            encoding='utf-8')
                        raise TimeoutError('Gateway/feedback/camera/Servo did not become ready')
                    halted.wait(.2)
                if result['ready']:
                    result['startup_ready_s']=time.monotonic()-startup_begin
                    emit('工作台已连接；当前未准备控制、未自动使能。Ctrl+C 正常退出。')
                    if open_browser:
                        webbrowser.open('http://127.0.0.1:8765/control?tab=keyboard')
                    begin = time.monotonic()
                    while not halted.wait(.2):
                        if channel.exit_status_ready():
                            raise RuntimeError('X5 session exited unexpectedly; see lifecycle log')
                        if duration is not None and time.monotonic()-begin >= duration:
                            halted.set()
            finally:
                # Ask the existing owner to release. Closing the intent transport
                # is left to gateway cleanup; no additional E6 Stop connection.
                if not channel.exit_status_ready():
                    try:
                        stdin.write('stop\n'); stdin.flush(); channel.shutdown_write()
                    except (OSError, EOFError):
                        pass
                end = time.monotonic()+38
                while not channel.exit_status_ready() and time.monotonic() < end:
                    time.sleep(.1)
                if channel.exit_status_ready():
                    result['remote_exit'] = channel.recv_exit_status()
                else:
                    result['remote_cleanup_uncertain'] = True
                reader.join(2)
                if reader.is_alive():
                    channel.close(); reader.join(2)
    except Exception as exc:
        result['error'] = str(exc)
        emit('未完成：'+str(exc))
    finally:
        result['remote_result'] = summary
        if tunnels:
            tunnels.close()
        result['local_cleanup_errors'] = local.close()
        result['local_exits'] = local.exits
        result['reused_local_services'] = local.reused
        result['clean'] = (not result.get('error') and not result['local_cleanup_errors'] and
                           (not result['remote_started'] or bool(summary and summary['clean'] and
                            result.get('remote_exit') == 0)))
        (runtime/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        emit('本次会话已正常释放。' if result['clean'] else '会话未完整结束，请查看 result.json；不自动杀死未知进程。')
        logger.removeHandler(ssh_log); logger.propagate = previous_propagate; ssh_log.close()
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, default=ROOT/'config/workstation.json')
    p.add_argument('--stage', choices=['shadow', 'live'], default='shadow')
    p.add_argument('--motion-profile', choices=['commissioning', 'intermediate', 'normal'], default='commissioning')
    p.add_argument('--motion-engine', choices=['moveit_servo', 'native_servop'], default='moveit_servo')
    p.add_argument('--moveit-api', choices=['ros', 'cpp'])
    p.add_argument('--workspace-mode', choices=['local','model'])
    p.add_argument('--tcp-profile', choices=['flange','omnipicker-r2','omnipicker-r4'])
    p.add_argument('--native-smoothing', choices=['none','ruckig'], default='none')
    p.add_argument('--servo-duration-s', type=float, choices=[.05, .1])
    p.add_argument('--memory-residency', choices=['off','current'])
    p.add_argument('--preview-codec', choices=['h264', 'vp8'])
    p.add_argument('--latency-marker', action='store_true', default=None)
    p.add_argument('--password', action='store_true')
    p.add_argument('--reuse-local', action='store_true')
    p.add_argument('--no-browser', action='store_true')
    p.add_argument('--acceptance-inputs', action='store_true')
    p.add_argument('--recover-recordings',action='store_true',help='Recover sealed recordings from finished sessions; no device connections')
    p.add_argument('--episode',help='Recover just this retained episode (requires --recover-recordings)')
    args = p.parse_args()
    if args.episode and not args.recover_recordings:p.error('--episode requires --recover-recordings')
    config = dict(json.loads(args.config.read_text()), stage=args.stage, motion_profile=args.motion_profile,
                  motion_engine=args.motion_engine,
                  native_smoothing=args.native_smoothing,
                  acceptance_inputs=args.acceptance_inputs)
    if args.workspace_mode:
        config['test_workspace']=dict(config.get('test_workspace',{}),mode=args.workspace_mode)
    workspace_mode=config.get('test_workspace',{}).get('mode','local')
    config['moveit_api']=args.moveit_api or ('cpp' if workspace_mode=='model' else 'ros')
    if args.recover_recordings:
        if args.stage!='shadow' or args.acceptance_inputs or args.reuse_local:
            p.error('Record recovery is isolated: do not combine live, acceptance inputs, or reused local services')
        from .recording_recovery import run as recover
        stop=threading.Event()
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
        result=recover(config,connect=ssh_factory(config,args.password),open_browser=not args.no_browser,halt=stop,episode=args.episode)
        raise SystemExit(0 if result['clean'] else 1)
    # Optional overrides retain configuration-file values when omitted. These
    # are startup-fixed and checked against the gateway before reporting ready.
    for key in ('servo_duration_s', 'preview_codec', 'latency_marker', 'memory_residency','tcp_profile'):
        value = getattr(args, key)
        if value is not None:
            config[key] = value
    from .session_runner import DEFAULT_SERVO_DURATION_S, servo_duration_option
    config.setdefault('servo_duration_s',DEFAULT_SERVO_DURATION_S)
    servo_duration_option(config)  # Reject obsolete live profiles before SSH/password setup.
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    result = run(config, connect=ssh_factory(config, args.password), reuse=args.reuse_local,
                 open_browser=not args.no_browser, halt=stop)
    raise SystemExit(0 if result['clean'] else 1)
