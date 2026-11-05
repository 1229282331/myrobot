"""Explicit session-owned camera restart IPC; never opens a robot connection."""
import json
from pathlib import Path
import re
import time
import uuid
from .camera_outputs import atomic


def atomic_json(path, value):
    # This module also runs in the stdlib-only resource supervisor. Do not
    # import the camera recorder or its compression/numerical dependencies.
    atomic(path,json.dumps(value,ensure_ascii=False).encode('utf8'))


class CameraRestart:
    def __init__(self, health, session, *, clock=time.perf_counter_ns, status_reader=None):
        self.health=Path(health);self.session=session;self.clock=clock
        self.status_reader=status_reader
        self.request_path=self.health.with_name('camera.restart.request.json')
        self.status_path=self.health.with_name('camera.restart.status.json')
        self.pending=None

    def status(self):
        default=dict(available=True,phase='requested' if self.pending else 'idle',id=self.pending)
        try:
            value=self.status_reader(self.status_path) if self.status_reader else json.loads(self.status_path.read_text(encoding='utf8'))
            if value['session']!=self.session:return default
            if self.pending and value['id']!=self.pending:return default
            return dict(value,available=True)
        except (OSError,ValueError,KeyError):return default

    def request(self):
        if self.status()['phase'] in ('requested','starting'):
            raise ValueError('相机正在重新连接')
        identity=uuid.uuid4().hex
        atomic_json(self.request_path,dict(op='restart_camera',session=self.session,id=identity,requested_ns=self.clock()))
        self.pending=identity
        return identity


class CameraRestartSupervisor:
    """The resource owner supplies exact-child restart and alive callbacks."""
    def __init__(self, health, session, restart, alive, event, *, clock=time.perf_counter_ns):
        self.channel=CameraRestart(health,session,clock=clock)
        self.restart=restart;self.alive=alive;self.event=event;self.clock=clock
        self.last=None;self.started=None;self.old_epoch=None

    def health(self):
        try:return json.loads(self.channel.health.read_text(encoding='utf8'))
        except (OSError,ValueError):return {}

    def publish(self,phase,**values):
        result=dict(session=self.channel.session,id=self.last,phase=phase,at_ns=self.clock(),**values)
        atomic_json(self.channel.status_path,result)
        # The lifecycle sink owns its own at_ns; preserve the status timestamp
        # under a distinct field instead of colliding with the sink contract.
        self.event('camera_restart_'+phase,session=self.channel.session,id=self.last,
                   phase=phase,status_at_ns=result['at_ns'],**values)

    def step(self):
        if self.started is not None:
            h=self.health();now=self.clock();received=h.get('rgb_received_ns',0)
            if not self.alive():
                self.publish('failed',error='相机采集进程启动后退出');self.started=None
            elif (h.get('serial')=='AY4A1520111' and h.get('capture_epoch')
                  and h['capture_epoch']!=self.old_epoch and type(received) is int and self.started<=received<=now
                  and now-received<=500_000_000 and not h.get('error')
                  and not h.get('closed_ns') and not h.get('capture_recovering')):
                self.publish('complete',capture_epoch=h['capture_epoch'],rgb_received_ns=received)
                self.started=None
            elif now-self.started>15_000_000_000:
                self.publish('failed',error='相机重连后15秒未取得新画面');self.started=None
            return
        path=self.channel.request_path
        if not path.exists():return
        try:
            if path.stat().st_size>4096:raise ValueError('Camera restart request too large')
            data=json.loads(path.read_text(encoding='utf8'))
            identity=data['id']
            if not isinstance(identity,str) or not re.fullmatch('[a-f0-9]{32}',identity):return
            if identity==self.last:return
            self.last=identity
            if data.get('op')!='restart_camera' or data.get('session')!=self.channel.session:
                raise ValueError('Camera restart session mismatch')
            if not 0<=self.clock()-data['requested_ns']<=5_000_000_000:
                raise ValueError('Camera restart request expired')
            self.old_epoch=self.health().get('capture_epoch')
            self.publish('starting')
            self.restart(identity)
            self.started=self.clock()
        except Exception as exc:
            if self.last is not None:self.publish('failed',error=str(exc))
