"""Loopback SSH forwarding; only explicitly started remote observer is owned."""
import os
from pathlib import Path
import select
import shlex
import socket
import socketserver
import threading
import time
import sys
import getpass
import json
import urllib.request
import paramiko


def ssh_loopback_forward(transport, local_port, remote_port=8767):
    class Handler(socketserver.BaseRequestHandler):
        def handle(handler):
            handler.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            try:channel=transport.open_channel('direct-tcpip',('127.0.0.1',remote_port),handler.request.getpeername())
            except (OSError,paramiko.SSHException):return
            try:
                while True:
                    readable,_,_=select.select([handler.request,channel],[],[],1)
                    if handler.request in readable:
                        data=handler.request.recv(65536)
                        if not data:break
                        channel.sendall(data)
                    if channel in readable:
                        data=channel.recv(65536)
                        if not data:break
                        handler.request.sendall(data)
                    if not transport.is_active():break
            except (OSError,EOFError,paramiko.SSHException):pass
            finally:channel.close()
    class Server(socketserver.ThreadingTCPServer):
        # Linux needs this to restart after owned loopback TIME_WAIT sockets.
        # It does not enable SO_REUSEPORT or sharing an active listener.
        allow_reuse_address=sys.platform=='linux'
        daemon_threads=True
    server=Server(('127.0.0.1',local_port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    return server


class Tunnel:
    def __init__(self,host,user='root',remote_root=None,port=8767,wifi_low_latency=False,media_hz=None,control_stage=None,control_role='harness',control_token=None,acceptance_profile=None,kinematics_profile=None,feedback_framing='strict',harness_limits='initial',wifi_reorder_ms=None,ssh_rto_min_ms=None,e6_rto_min_ms=None,pc_rto_min_ms=None,recovery_policy='manual',gripper_command_profile='plan32'):
        if pc_rto_min_ms not in (None,20):raise ValueError('PC minimum RTO requires explicit 20 ms')
        self.pc_rto_min_ms=pc_rto_min_ms;self.pc_tcp_profile=None
        if wifi_reorder_ms not in (None,10) or wifi_reorder_ms is not None and not remote_root:raise ValueError('Temporary reorder setting requires owned gateway and 10 ms')
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('wifi_reorder_ms')!=wifi_reorder_ms:raise ValueError('Wi-Fi reorder differs from acceptance')
        if ssh_rto_min_ms not in (None,20) or ssh_rto_min_ms is not None and (not remote_root or not control_stage):raise ValueError('Peer TCP RTO requires owned control gateway and 20 ms')
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('ssh_rto_min_ms')!=ssh_rto_min_ms:raise ValueError('SSH minimum RTO differs from acceptance')
        if e6_rto_min_ms not in (None,20) or e6_rto_min_ms is not None and (not remote_root or not control_stage):raise ValueError('E6 TCP RTO requires owned control gateway and 20 ms')
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('e6_rto_min_ms')!=e6_rto_min_ms:raise ValueError('E6 minimum RTO differs from acceptance')
        if media_hz is None:media_hz=(acceptance_profile or {}).get('media_hz',10 if control_stage else 20)
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('media_hz')!=media_hz:raise ValueError('Preview rate differs from hardware acceptance')
        if media_hz not in (5,10,20):raise ValueError('media_hz must be 5, 10 or 20')
        self.host,self.user,self.remote_root,self.port=host,user,remote_root,port
        self.archive_client=None;self.archive_server=None;self.archive_port=port+2
        self.client=None;self.server=None;self.remote_stdin=None;self.remote_channel=None
        self.wifi_low_latency=wifi_low_latency
        self.wifi_reorder_ms=wifi_reorder_ms
        self.ssh_rto_min_ms=ssh_rto_min_ms
        self.e6_rto_min_ms=e6_rto_min_ms
        self.media_hz=media_hz
        self.control_stage,self.control_role,self.control_token=control_stage,control_role,control_token
        if recovery_policy not in ('manual','bounded'):raise ValueError('Unknown recovery policy')
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('recovery_policy','manual')!=recovery_policy:raise ValueError('Recovery policy differs from hardware acceptance')
        self.recovery_policy=recovery_policy
        self.control_client=None;self.control_server=None;self.control_port=8770
        self.control_qos=None
        self.telemetry_client=None;self.telemetry_server=None;self.telemetry_port=8771;self.telemetry_qos=None
        self.acceptance_profile=acceptance_profile
        self.kinematics_profile=kinematics_profile
        self.feedback_framing=feedback_framing
        if harness_limits not in ('initial','delivery') or harness_limits=='delivery' and (not control_stage or control_role!='harness' or acceptance_profile):raise ValueError('Delivery test limits are harness-only and cannot replace operator acceptance')
        self.harness_limits=harness_limits
        from .gripper_profile import gripper_profile
        self.gripper_settings=gripper_profile(gripper_command_profile)
        if not control_stage and gripper_command_profile!='plan32':raise ValueError('Gripper command profile requires hardware-control')
        if control_role=='operator' and acceptance_profile and acceptance_profile.get('gripper_command_profile','plan32')!=gripper_command_profile:raise ValueError('Gripper configuration differs from acceptance')
        self.gripper_command_profile=gripper_command_profile

    def tested_limits(self):
        limits=dict((self.acceptance_profile or {}).get('limits',{}))
        if not self.acceptance_profile:
            if self.harness_limits=='delivery':
                import math
                limits.update(translation_m_s=.005,rotation_rad_s=math.radians(2),gripper_s=.1)
            if self.gripper_command_profile=='empty64-medium':limits['gripper_s']=self.gripper_settings['gripper_s']
        if limits.get('gripper_s',.05)>self.gripper_settings['gripper_s']:raise ValueError('Gripper rate exceeds selected command profile')
        return limits

    def start(self):
        try:return self._start()
        except Exception as exc:
            self.close()
            details=''.join(getattr(self,'logs',[]))[-4000:]
            if self.control_token:details=details.replace(self.control_token,'[session-token]')
            if details:raise RuntimeError(str(exc)+'; owned gateway diagnostics: '+details) from exc
            raise

    def _start(self):
        self.client=paramiko.SSHClient()
        self.client.load_system_host_keys()
        self.client.set_missing_host_key_policy(paramiko.RejectPolicy())
        config_path=Path.home()/'.ssh'/'config'
        config=paramiko.SSHConfig.from_path(str(config_path)).lookup(self.host) if config_path.exists() else {}
        options=dict(hostname=config.get('hostname',self.host),username=self.user or config.get('user','root'),
                     port=int(config.get('port',22)),key_filename=config.get('identityfile'),
                     password=os.environ.get('E6_X5_PASSWORD'),timeout=8,auth_timeout=8)
        try:self.client.connect(**options)
        except paramiko.AuthenticationException:
            if options['password'] or not sys.stdin.isatty():raise
            options['password']=getpass.getpass('X5 SSH password (this session only): ')
            self.client.connect(**options)
        transport=self.client.get_transport();transport.set_keepalive(5)
        from .pc_tcp_latency import inspect_pc_route,configuration
        self.pc_tcp_profile=inspect_pc_route(transport.sock,self.pc_rto_min_ms)
        if self.control_role=='operator' and self.acceptance_profile and self.acceptance_profile.get('pc_transport')!=configuration(self.pc_tcp_profile):
            raise ValueError('PC TCP transport differs from hardware acceptance')
        if self.control_role=='operator' and self.acceptance_profile:
            expected_network=self.acceptance_profile.get('e6_network')
            _,out,err=self.client.exec_command('ip -j route get 192.168.1.4')
            rows=json.loads(out.read())
            if out.channel.recv_exit_status() or len(rows)!=1:raise ValueError('E6 route inspection failed')
            row=rows[0]
            actual_network=dict(robot_ip='192.168.1.4',interface=row.get('dev'),source=row.get('prefsrc'),
                                gateway=row.get('gateway'),direct_ethernet=row.get('dev')=='eth0' and not row.get('gateway'))
            if actual_network!=expected_network:raise ValueError('E6 network differs from accepted direct link')
            expected=self.acceptance_profile.get('wifi_driver_ps_on')
            if expected not in ('Y','N'):raise ValueError('Accepted Wi-Fi driver PS parameter missing')
            _,out,err=self.client.exec_command('cat /sys/module/aic8800_fdrv/parameters/ps_on')
            actual=out.read().decode().strip()
            if out.channel.recv_exit_status() or actual!=expected:raise ValueError('Wi-Fi driver PS differs from acceptance; no device session started')
        # Small clock/telemetry messages should not wait for TCP coalescing.
        transport.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.server=ssh_loopback_forward(transport,self.port)
        self.archive_client=paramiko.SSHClient()
        self.archive_client.load_system_host_keys()
        self.archive_client.set_missing_host_key_policy(paramiko.RejectPolicy())
        self.archive_client.connect(**options)
        archive_transport=self.archive_client.get_transport();archive_transport.set_keepalive(5)
        archive_transport.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.archive_server=ssh_loopback_forward(archive_transport,self.archive_port)
        if self.remote_root:
            import secrets
            instance=secrets.token_urlsafe(20)
            root=shlex.quote(self.remote_root)
            remote_profile=None
            if self.kinematics_profile:
                # Use the same verifier on both hosts; never upload an unchecked
                # collection of coefficients or treat an experiment as acceptance.
                from .kinematics import verify_local_profile
                profile_path=Path(self.kinematics_profile).resolve()
                metadata=verify_local_profile(profile_path)
                if self.acceptance_profile and metadata['profile_sha256']!=self.acceptance_profile.get('kinematics_profile_sha256'):
                    raise ValueError('Operator calibration differs from hardware acceptance')
                profile=json.loads(profile_path.read_text(encoding='utf-8'))
                directory=self.remote_root+'/kinematics-'+metadata['profile_sha256']
                _,out,err=self.client.exec_command('mkdir -p '+shlex.quote(directory))
                if out.channel.recv_exit_status():raise RuntimeError(err.read().decode())
                import hashlib
                with self.client.open_sftp() as sftp:
                    for local,name in ((profile_path,'profile.json'),(profile_path.parent/profile['capture_file'],profile['capture_file'])):
                        target=directory+'/'+name;sftp.put(str(local),target)
                        with sftp.open(target,'rb') as uploaded:
                            if hashlib.sha256(uploaded.read()).digest()!=hashlib.sha256(local.read_bytes()).digest():raise ValueError('Uploaded calibration checksum mismatch')
                remote_profile=directory+'/profile.json'
            command=(f'cd {root} && env PYTHONPATH={root}/device_gateway:{root}/deps '
                     'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '
                     'DCW_SDK_ROOT=/root/installed-audit-20261002_2142/vendor/OrbbecSDK_v1.10.37/SDK '
                     'DCW_SDK_CONFIG=/root/remaining-issues-20261002_2226/config_v4l2.xml '
                     f'python3 -m e6_observer --parent-stdin --instance {instance} --media-hz {self.media_hz}')
            if self.wifi_low_latency:command+=' --wifi-low-latency'
            if self.wifi_reorder_ms is not None:command+=' --wifi-reorder-ms '+str(self.wifi_reorder_ms)
            if self.ssh_rto_min_ms is not None:command+=' --ssh-rto-min-ms '+str(self.ssh_rto_min_ms)
            if self.e6_rto_min_ms is not None:command+=' --e6-rto-min-ms '+str(self.e6_rto_min_ms)
            if self.control_stage:command+=f' --control-stage {self.control_stage} --control-role {self.control_role} --startup-config-stdin'
            stdin,stdout,stderr=self.client.exec_command(command)
            self.remote_stdin=stdin;self.remote_channel=stdout.channel
            if self.control_stage:
                stdin.write(json.dumps(dict(control_token=self.control_token,operator_verified=bool(self.acceptance_profile),control_limits=self.tested_limits(),kinematics_profile=remote_profile,feedback_framing=self.feedback_framing,recovery_policy=self.recovery_policy,gripper_command_profile=self.gripper_command_profile))+'\n');stdin.flush()
            # Drain remote logs, avoiding channel backpressure during long capture.
            self.logs=[]
            def drain():
                while True:
                    for ready,read in ((self.remote_channel.recv_ready,self.remote_channel.recv),(self.remote_channel.recv_stderr_ready,self.remote_channel.recv_stderr)):
                        if ready():self.logs.append(read(65536).decode('utf-8','replace'));self.logs=self.logs[-64:]
                    if self.remote_channel.exit_status_ready():break
                    time.sleep(.1)
            self.log_thread=threading.Thread(target=drain,daemon=True);self.log_thread.start()
            # Readiness and identity belong to the process we just launched.
            # Do not let the PC connect to an older listener or a startup race.
            time.sleep(.5)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                if self.remote_channel.exit_status_ready():raise RuntimeError('Owned X5 observer exited during startup')
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{self.port}/health',timeout=1) as response:health=json.load(response)
                    if health.get('instance')!=instance:raise RuntimeError('Another process owns the X5 gateway port')
                    break
                except (OSError,urllib.error.URLError):time.sleep(.1)
            else:raise RuntimeError('X5 observer startup timed out')
        if self.control_stage:self._start_control_channels(options)
        return self

    def _start_control_channels(self,options):
        # Open fresh sockets after gateway readiness: its peer route is now active.
        if self.control_stage:
            if not self.remote_root or not self.control_token:raise ValueError('Hardware control requires an owned gateway and session token')
            self.control_client=paramiko.SSHClient();self.control_client.load_system_host_keys();self.control_client.set_missing_host_key_policy(paramiko.RejectPolicy())
            # Control replies repeat limits, calibration and lease metadata.
            # SSH's negotiated streaming compression reduces packet count on
            # Wi-Fi without adding another input queue or acknowledgement wait.
            # Media is already compressed and uses a different transport.
            recovery=self.connect_realtime_ssh(self.control_client,dict(options,compress=True))
            transport_control=self.control_client.get_transport();transport_control.set_keepalive(5)
            self.control_transport_profile=dict(local_compression=transport_control.local_compression,
                                                remote_compression=transport_control.remote_compression,
                                                tcp_recovery=recovery)
            transport_control.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            # Request WMM/DSCP priority for this separate control transport;
            # the access point may ignore it. Never mark bulk video/archive.
            transport_control.sock.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,0xB8)
            from .socket_qos import ControlSocketQos
            self.control_qos=ControlSocketQos(transport_control.sock)
            self.control_transport_profile['socket_qos']=self.control_qos.report
            self.control_transport_profile['return_socket_qos']=self.configure_return_qos(self.control_client)
            self.control_server=ssh_loopback_forward(transport_control,self.control_port,8768)
            # Small ordered observations must not share TCP head-of-line
            # blocking with video. Native archives keep their own transport.
            self.telemetry_client=paramiko.SSHClient();self.telemetry_client.load_system_host_keys();self.telemetry_client.set_missing_host_key_policy(paramiko.RejectPolicy())
            recovery=self.connect_realtime_ssh(self.telemetry_client,dict(options,compress=True))
            telemetry=self.telemetry_client.get_transport();telemetry.set_keepalive(5)
            telemetry.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.telemetry_qos=ControlSocketQos(telemetry.sock)
            self.telemetry_transport_profile=dict(socket_qos=self.telemetry_qos.report,tcp_recovery=recovery,
                return_socket_qos=self.configure_return_qos(self.telemetry_client),separate_from_preview=True)
            self.telemetry_server=ssh_loopback_forward(telemetry,self.telemetry_port)

    def connect_realtime_ssh(self,client,options):
        if self.pc_rto_min_ms is None:
            client.connect(**options);return None
        from .socket_qos import linux_low_latency_recovery
        wire=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            recovery=linux_low_latency_recovery(wire)
            wire.settimeout(options['timeout'])
            wire.connect((options['hostname'],options['port']))
            client.connect(**dict(options,sock=wire))
            return recovery
        except BaseException:wire.close();raise

    def configure_return_qos(self,client):
        # SSH_CONNECTION belongs to this authenticated SSH session. A WSL/NAT
        # client's getsockname() is not the peer observed by the remote sshd.
        # Check the remote server against the connected destination before
        # allowing the X5 helper to verify that exact accepted socket by PID.
        local=list(client.get_transport().sock.getsockname())
        destination=list(client.get_transport().sock.getpeername())
        _,out,err=client.exec_command("python3 -c 'import os; print(os.environ.get(\"SSH_CONNECTION\",\"\"))'",timeout=5)
        connection=out.read().decode().strip()
        if out.channel.recv_exit_status():raise RuntimeError('Owned SSH endpoint inspection failed')
        from .ssh_endpoints import observed_peer
        peer=observed_peer(connection,destination)
        command='env PYTHONPATH='+shlex.quote(self.remote_root+'/device_gateway')+' python3 -m e6_observer.ssh_socket_qos '+shlex.quote(json.dumps(peer))
        _,out,err=client.exec_command(command,timeout=5)
        value=out.read();error=err.read().decode('utf-8','replace')
        if out.channel.recv_exit_status():raise RuntimeError('Owned SSH return QoS configuration failed: '+error)
        result=json.loads(value)
        if result.get('peer')!=peer or result.get('local')!=destination:raise ValueError('Owned SSH endpoint changed during QoS configuration')
        result.update(client_local_endpoint=local,nat_observed=local!=peer)
        return result

    def close(self):
        result=dict(remote_owned=bool(self.remote_stdin),remote_exited=None)
        if self.remote_stdin:
            try:self.remote_stdin.channel.shutdown_write()
            except (OSError,EOFError,paramiko.SSHException):pass
            deadline=time.monotonic()+8
            while not self.remote_channel.exit_status_ready() and time.monotonic()<deadline:time.sleep(.1)
            result['remote_exited']=self.remote_channel.exit_status_ready()
            if result['remote_exited']:result['exit_code']=self.remote_channel.recv_exit_status()
            else:print('Owned X5 observer exit was not confirmed; check resource status.',flush=True)
        if getattr(self,'log_thread',None):self.log_thread.join(1)
        if self.server:self.server.shutdown();self.server.server_close()
        if self.archive_server:self.archive_server.shutdown();self.archive_server.server_close()
        if self.control_server:self.control_server.shutdown();self.control_server.server_close()
        if self.telemetry_server:self.telemetry_server.shutdown();self.telemetry_server.server_close()
        if self.archive_client:self.archive_client.close()
        if self.client:self.client.close()
        if self.control_qos:self.control_qos.close()
        if self.control_client:self.control_client.close()
        if self.telemetry_qos:self.telemetry_qos.close()
        if self.telemetry_client:self.telemetry_client.close()
        self.remote_stdin=None;self.server=None;self.client=None;self.archive_server=None;self.archive_client=None
        return result
