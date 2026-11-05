"""Session-owned JSON file polling outside the control/feedback threads.

Snapshots retain source timestamps. A blocked file read cannot extend freshness.
"""
import json
from pathlib import Path
import threading


class LatestJsonFiles:
    def __init__(self, paths, *, period=.02, load=None):
        self.paths=tuple(dict.fromkeys(Path(p) for p in paths))
        self.period=period;self.load=load or self._load
        self.halt=threading.Event();self.thread=None
        self.lock=threading.Lock();self.values={}

    @staticmethod
    def _load(path):
        with path.open('rb') as source:data=source.read(65537)
        if len(data)>65536:raise ValueError('Camera status JSON exceeds 64 KiB')
        value=json.loads(data)
        if not isinstance(value,dict):raise ValueError('Camera status must be an object')
        return value

    def start(self):
        if self.thread is not None:raise RuntimeError('JSON monitor already started')
        self.thread=threading.Thread(target=self._run,name='e6-camera-status',daemon=True)
        self.thread.start()

    def _run(self):
        while not self.halt.is_set():
            for path in self.paths:
                if self.halt.is_set():break
                try:result=(self.load(path),None)
                except Exception as exc:result=(None,str(exc))
                # File IO/decoding never holds the short snapshot lock.
                with self.lock:self.values[path]=result
            self.halt.wait(self.period)

    def read(self,path):
        with self.lock:value,error=self.values.get(Path(path),(None,'Camera status not yet read'))
        if error is not None:raise OSError(error)
        # Consumers read values or make their own projection; no mutable state
        # is shared with the producer after publication.
        return value

    def close(self):
        self.halt.set()
        if self.thread is not None:
            self.thread.join(1)
            if self.thread.is_alive():raise RuntimeError('Owned camera status reader did not exit')
