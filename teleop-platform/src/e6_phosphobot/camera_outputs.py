"""Coalesced health and diagnostic snapshots, independent of SDK acquisition.

The health file belongs on a session-scoped tmpfs path. Original media stays in
CameraArchive on disk. Nothing here invents a source timestamp or queues images
for later playback. Slow diagnostics replace their pending snapshot with latest.
"""
import json
from pathlib import Path
import threading
import time


def atomic(path,data):
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_bytes(data)
    temp.replace(path)


class CameraOutputs:
    def __init__(self,root,health_path,*,write=atomic,depth_valid=None,snapshot_root=None):
        self.root=Path(root);self.health_path=Path(health_path)
        self.snapshot_root=Path(snapshot_root) if snapshot_root is not None else self.root
        self.root.mkdir(parents=True,exist_ok=True)
        self.snapshot_root.mkdir(parents=True,exist_ok=True)
        self.health_path.parent.mkdir(parents=True,exist_ok=True)
        self.write=write;self.depth_valid=depth_valid
        self.lock=threading.Lock();self.latest={};self.health=None
        self.halt=threading.Event();self.threads=[]
        self.diagnostics=dict(snapshot_error=None,health_error=None,snapshot_write_max_ms=0,
                              health_write_max_ms=0,snapshot_frames={},snapshot_root=str(self.snapshot_root))

    def offer(self,meta,data,health):
        # No serialization, compression, decoding or file I/O under this lock.
        with self.lock:
            self.latest[meta['stream']]=(dict(meta),data)
            self.health=dict(health)

    def status(self,health):
        # A capture error may update diagnostics, never the last source time.
        with self.lock:self.health=dict(health)

    def _health_loop(self):
        while True:
            closing=self.halt.is_set()
            with self.lock:health=None if self.health is None else dict(self.health)
            if health is not None:
                health['outputs']=dict(self.diagnostics)
                begin=time.perf_counter_ns()
                try:
                    self.write(self.health_path,json.dumps(health).encode())
                    self.diagnostics['health_error']=None
                except Exception as exc:self.diagnostics['health_error']=str(exc)
                self.diagnostics['health_write_max_ms']=max(self.diagnostics['health_write_max_ms'],(time.perf_counter_ns()-begin)/1e6)
            if closing:return
            self.halt.wait(.05)

    def _snapshot_loop(self):
        previous={};last={}
        while not self.halt.is_set():
            with self.lock:latest=dict(self.latest)
            for name,(meta,data) in latest.items():
                now=time.perf_counter_ns()
                period=1_000_000_000 if name=='color' else 200_000_000
                if previous.get(name)==meta['index'] or now-last.get(name,0)<period:continue
                try:
                    begin=time.perf_counter_ns()
                    if name=='color':
                        self.write(self.snapshot_root/'color.jpg',data)
                        self.write(self.snapshot_root/'color.json',json.dumps(meta).encode())
                    elif name=='depth':
                        valid=self.depth_valid(meta,data) if self.depth_valid else None
                        self.write(self.snapshot_root/'depth.uint16',data)
                        self.write(self.snapshot_root/'depth.json',json.dumps(dict(meta,valid_fraction=valid)).encode())
                    else:continue
                    self.diagnostics['snapshot_write_max_ms']=max(self.diagnostics['snapshot_write_max_ms'],(time.perf_counter_ns()-begin)/1e6)
                    self.diagnostics['snapshot_frames']={**self.diagnostics['snapshot_frames'],name:meta}
                    previous[name]=meta['index'];last[name]=now
                except Exception as exc:
                    # A broken diagnostic disk is reported without killing SDK
                    # acquisition or retrying every frame. Original archive has
                    # its own failure handling and remains independently owned.
                    self.diagnostics['snapshot_error']=str(exc);return
            self.halt.wait(.02)

    def start(self):
        for name,target in [('camera-health',self._health_loop),('camera-snapshots',self._snapshot_loop)]:
            thread=threading.Thread(name=name,target=target,daemon=True)
            self.threads.append(thread);thread.start()

    def close(self,final):
        with self.lock:self.health=dict(final)
        self.halt.set()
        for thread in self.threads:thread.join(3)
        errors=[thread.name+' did not exit' for thread in self.threads if thread.is_alive()]
        # Only the latest diagnostic image is volatile. Once capture/outputs
        # stop, retain the exact last-written pair for session diagnosis. This
        # never touches CameraArchive's original recording or source clocks.
        if not errors and self.snapshot_root!=self.root:
            for name in ('color.jpg','color.json','depth.uint16','depth.json'):
                source=self.snapshot_root/name
                try:
                    if source.exists():self.write(self.root/name,source.read_bytes())
                except OSError as exc:errors.append('Final diagnostic snapshot: '+str(exc))
        return errors
