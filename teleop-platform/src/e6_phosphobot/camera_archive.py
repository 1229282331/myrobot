"""Camera-only original-media recording using the existing verified spool.

The owner writes a session-scoped request file; no device commands or media
payloads cross the control channel. File I/O/compression never run in capture.
"""
import hashlib
import errno
import json
import os
from pathlib import Path
import threading
import time
from collections import deque
from .media_archive import ArchiveStore


def read_atomic_json(path):
    """Read fresh metadata through a brief Windows atomic-rename conflict.

    The CRT can expose delete-pending as EACCES without a winerror. There is
    no cached fallback: persistent denial and invalid JSON still fail.
    """
    deadline=time.monotonic()+.05
    while True:
        try:return json.loads(path.read_text(encoding='utf-8'))
        except PermissionError as exc:
            if os.name!='nt' or exc.errno!=errno.EACCES or time.monotonic()>=deadline:raise
            time.sleep(.002)


def atomic_json(path,data):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8')
    deadline=time.monotonic()+.05
    while True:
        try:
            temp.replace(path)
            break
        except PermissionError as exc:
            # Windows readers briefly omit FILE_SHARE_DELETE. Retry only the
            # atomic rename of this already-written metadata, never a recording
            # request or device action. Persistent ACL/disk errors still abort.
            if os.name!='nt' or getattr(exc,'winerror',None) not in (5,32) or time.monotonic()>=deadline:
                raise
            time.sleep(.002)


class CameraArchive:
    def __init__(self,root,*,store_factory=ArchiveStore,status_path=None):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.store=store_factory(self.root/'media')
        self.request_path=self.root/'recording.request.json'
        # Session IPC can share the existing health tmpfs. Native media and
        # journals remain under root; their checksums/durability do not change.
        self.status_path=Path(status_path) if status_path is not None else self.root/'recording.status.json'
        self.status_path.parent.mkdir(parents=True,exist_ok=True)
        self.lock=threading.Lock();self.latest={};self.error=None
        self.last_request=None;self.sample_index=0;self.phase='idle'
        self.halt=threading.Event();self.thread=None
        self.previous={}
        self.timings={};self.slow_stages=deque(maxlen=16);self.last_loop_ns=None
        self.public_status=self.status()

    def measure(self,name,operation):
        started=time.perf_counter_ns()
        try:return operation()
        finally:self.timing(name,started,time.perf_counter_ns())

    def timing(self,name,started,finished):
        elapsed=finished-started
        stats=self.timings.setdefault(name,dict(count=0,total_ns=0,max_ns=0,over_100ms=0))
        stats['count']+=1;stats['total_ns']+=elapsed;stats['max_ns']=max(stats['max_ns'],elapsed)
        if elapsed>100_000_000:
            stats['over_100ms']+=1
            self.slow_stages.append(dict(stage=name,started_ns=started,finished_ns=finished,phase=self.phase))

    def read_request(self):
        if self.request_path.exists():return read_atomic_json(self.request_path)

    def offer(self,meta,data):
        name={'color':'rgb','depth':'depth'}.get(meta['stream'])
        if not name:return
        if not isinstance(data,bytes):data=bytes(data)
        with self.lock:self.latest[name]=(dict(meta),data)

    def request(self,data):
        request_id=data['id']
        if not isinstance(request_id,str) or len(request_id)>100:raise ValueError('Invalid request id')
        if self.last_request==request_id:return
        op=data['op'];session=data['session']
        if op=='start':
            if self.store.reserved:
                if self.store.active:raise ValueError('A recording is already active')
                self.store.release(self.store.session)
            self.store.begin(session)
            (self.store.directory/'camera_samples.jsonl').touch(exist_ok=False)
            self.error=None;self.phase='capturing';self.sample_index=0
            self.previous={}
            self.timings={};self.slow_stages.clear();self.last_loop_ns=None
        elif op=='finish':
            self.store.seal(session)
            self.phase='sealed' if not self.store.error else 'failed'
        else:raise ValueError('Unknown recording operation')
        self.last_request=request_id

    def sample(self):
        if not self.store.active:return
        started=time.perf_counter_ns()
        with self.lock:latest=dict(self.latest)
        now=time.perf_counter_ns();sources={}
        for name in ('rgb','depth'):
            source={};sources[name]=source
            if name not in latest:continue
            meta,data=latest[name]
            digest=hashlib.sha256(data).hexdigest()
            previous=self.previous.get(name)
            if previous and (meta['index']<previous[0] or meta['index']==previous[0] and digest!=previous[1]):
                raise RuntimeError('Camera source identity restarted or changed: '+name)
            self.previous[name]=(meta['index'],digest)
            key='jpeg' if name=='rgb' else 'uint16_raw'
            # Record timestamps and source fields verbatim; no exposure sync claim.
            source.update(device_frame_id=meta['index'],metadata=meta,
                          x5_receive_monotonic_ns=meta.get('source_received_perf_ns',meta['host_perf_counter_ns']),
                          original_sha256=digest,original_bytes=len(data))
            source[key]=data
        frame=dict(sources=sources,stats={},x5_sample_ns=now,sample_index=self.sample_index)
        self.timing('sample_prepare',started,time.perf_counter_ns())
        self.measure('native_enqueue',lambda:self.store.capture(frame))
        for name,source in sources.items():
            source.pop('jpeg',None);source.pop('uint16_raw',None)
            received=source.get('x5_receive_monotonic_ns')
            source['age_ms']=None if received is None else (now-received)/1e6
            source['fresh']=received is not None and 0<=now-received<=100_000_000
        line=self.measure('sample_encode',lambda:json.dumps(frame)+'\n')
        self.measure('sample_enqueue',lambda:self.store.enqueue_sample(self.sample_index,line))
        self.sample_index+=1
        if self.store.error:raise RuntimeError(self.store.error)

    def status(self):
        return dict(self.store.status(),phase=self.phase,last_request=self.last_request,
                    sample_index=self.sample_index,error=self.error or self.store.error,
                    timing=dict(stages={k:dict(v) for k,v in self.timings.items()},slow_stages=list(self.slow_stages)),
                    recorded_on='X5',clock='perf_counter_ns',exposure_synchronized=False)

    def _run(self):
        try:
            while not self.halt.is_set():
                started=time.monotonic()
                now=time.perf_counter_ns()
                if self.last_loop_ns is not None:self.timing('loop_interval',self.last_loop_ns,now)
                self.last_loop_ns=now
                try:
                    data=self.measure('request_read',self.read_request)
                    if data and data.get('id')!=self.last_request:self.measure('request_apply',lambda:self.request(data))
                    self.measure('sample',self.sample)
                    if self.store.error:raise RuntimeError(self.store.error)
                except Exception as exc:
                    self.error=str(exc);self.phase='failed'
                    with self.store.lock:self.store.active=False
                    # Do not retry a failed start or finish on every frame.
                    if 'data' in locals() and data:self.last_request=data.get('id')
                self.public_status=self.measure('status_snapshot',self.status)
                try:self.measure('status_write',lambda:atomic_json(self.status_path,self.public_status))
                except Exception as exc:self.error='Recording status write failed: '+str(exc)
                self.halt.wait(max(0,.05-(time.monotonic()-started)))
        finally:
            try:
                if self.store.session is not None and self.phase=='capturing':
                    self.store.seal(self.store.session)
                    self.error='Camera service exited during recording';self.phase='aborted'
            finally:
                self.store.close()
                self.public_status=self.status()
                try:atomic_json(self.status_path,self.status())
                except OSError:pass

    def start(self):
        self.thread=threading.Thread(target=self._run,name='camera-original-recording',daemon=True)
        self.thread.start()

    def close(self):
        self.halt.set()
        if self.thread:
            self.thread.join(7)
            if self.thread.is_alive():raise RuntimeError('Camera recording task did not exit')
        else:
            self.store.close()
