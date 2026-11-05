"""Windows owns PC clocks/UI/recording; existing WSL owns only SSH TCP sockets."""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import uuid
from .model import ROOT
from .pc_tcp_latency import configuration


def linux_path(value):
    p=Path(value).resolve()
    if len(p.drive)!=2 or p.drive[1]!=':':raise ValueError('Windows workspace drive path required')
    return '/mnt/'+p.drive[0].lower()+p.as_posix()[2:]


def split_profile(report):
    if configuration(report)!=dict(platform='linux',requested_min_rto_ms=20,effective_min_rto_ms=20,interface='eth0',preview_renderer=None):
        raise ValueError('Owned WSL SSH peer route not verified')
    return dict(report,platform='win32-wsl-ssh',ssh_socket_platform='linux',
                pc_clock_platform='win32',preview_renderer=None,
                linux_transport_provenance=dict(report),scope='Linux SSH sockets only; Windows PC clocks and UI')


class WslTunnel:
    def __init__(self,host,user='root',remote_root=None,**options):
        if sys.platform!='win32':raise ValueError('WSL SSH runtime requires Windows PC service')
        if options.get('pc_rto_min_ms')!=20:raise ValueError('WSL SSH runtime requires explicit 20 ms peer route')
        self.options=dict(options,host=host,user=user,remote_root=remote_root);self.child=None;self.messages=queue.Queue();self.logs=[]
        self.port=options.get('port',8767);self.archive_port=self.port+2;self.control_port=8770;self.telemetry_port=8771
        self.pc_tcp_profile=None;self.control_transport_profile=None;self.telemetry_transport_profile=None
        self.acceptance_profile=options.get('acceptance_profile')
        self.route_evidence='/home/lixiaoyu/.cache/e6-teleop-wsl/peer-routes/'+uuid.uuid4().hex
        self.route_owned=False;self.route_report=None

    def configure_route(self,mode):
        command=['wsl.exe','-d','Ubuntu-24.04','-u','root','--',
                 '/home/lixiaoyu/.cache/e6-teleop-wsl/bin/python',
                 linux_path(ROOT/'scripts/configure_pc_peer_route.py'),mode,'--evidence',self.route_evidence]
        result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',timeout=15,
                              creationflags=subprocess.CREATE_NO_WINDOW)
        if result.returncode:raise RuntimeError('Owned WSL peer route '+mode+' failed: '+result.stderr[-1500:])
        return json.loads(result.stdout)

    def start(self):
        startup=dict(self.options)
        if startup.get('kinematics_profile'):startup['kinematics_profile']=linux_path(startup['kinematics_profile'])
        startup['password']=os.environ.get('E6_X5_PASSWORD')
        command=['wsl.exe','-d','Ubuntu-24.04','--',
                 '/home/lixiaoyu/.cache/e6-teleop-wsl/bin/python','-u',
                 linux_path(ROOT/'src/e6_teleop/ssh_runtime.py')]
        self.route_report=self.configure_route('create');self.route_owned=True
        try:
            self.child=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                        text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW)
        except BaseException:self.close();raise
        def read():
            for line in self.child.stdout:
                try:
                    value=json.loads(line)
                    if isinstance(value,dict) and value.get('event') in ('ready','closed','failure'):
                        self.messages.put(value);continue
                except ValueError:pass
                # Never forward credentials/capability tokens from child logs.
                for secret in (startup.get('password'),startup.get('control_token')):
                    if secret:line=line.replace(secret,'[secret]')
                self.logs=(self.logs+[line])[-64:]
            self.messages.put(dict(event='exited',code=self.child.poll()))
        self.reader=threading.Thread(target=read,daemon=True);self.reader.start()
        try:
            self.child.stdin.write(json.dumps(startup)+'\n');self.child.stdin.flush()
            event=self.messages.get(timeout=45)
            if event.get('event')!='ready':raise RuntimeError('Owned WSL SSH startup failed: '+str(event.get('error') or event.get('event')))
            self.pc_tcp_profile=split_profile(event['pc_tcp_profile'])
            self.pc_tcp_profile['route_ownership']=dict(evidence=self.route_evidence,created=self.route_report)
            if self.acceptance_profile and self.acceptance_profile.get('pc_transport')!=configuration(self.pc_tcp_profile):
                raise ValueError('Split PC transport differs from hardware acceptance')
            self.control_transport_profile=event['control_transport_profile']
            self.telemetry_transport_profile=event['telemetry_transport_profile']
            return self
        except BaseException:self.close();raise

    def close(self):
        if not self.child:
            result=dict(remote_owned=False,remote_exited=None)
            return self.restore_route(result)
        try:
            if not self.child.stdin.closed:self.child.stdin.close()
        except (OSError,BrokenPipeError):pass
        try:self.child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            # Do not kill other WSL processes or claim remote resources released.
            return dict(remote_owned=True,remote_exited=False,reason='Owned SSH runtime exit not confirmed')
        self.reader.join(timeout=1)
        events=[]
        while not self.messages.empty():events.append(self.messages.get_nowait())
        completed=next((e for e in events if e.get('event')=='closed'),None)
        result=completed['result'] if completed else dict(remote_owned=True,remote_exited=False,exit_code=self.child.returncode)
        return self.restore_route(result)

    def restore_route(self,result):
        if self.route_owned:
            try:
                result['pc_route_cleanup']=self.configure_route('remove');self.route_owned=False
            except Exception as exc:result['pc_route_cleanup']=dict(restored=False,error=str(exc))
        return result
