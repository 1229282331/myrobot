"""Bounded, session-owned native media spool; never touches device controls."""
import hashlib
import json
from pathlib import Path
import queue
import re
import threading
import time
import zstandard


class ArchiveStore:
    def __init__(self, root, max_bytes=2*1024**3, queue_size=128):
        self.root=Path(root)
        self.max_bytes=max_bytes
        self.queue=queue.Queue(maxsize=queue_size)
        self.lock=threading.Lock()
        self.session=None; self.directory=None; self.seen=set(); self.files={}
        self.bytes=0; self.error=None; self.active=False; self.closing=False
        self.reserved=False
        self.queue_peak=0;self.storage_bytes=0;self.write_max_ms=0
        self.stop=threading.Event()
        self.worker=threading.Thread(target=self._write,daemon=True)
        self.worker.start()

    def begin(self, session):
        if not isinstance(session,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',session):
            raise ValueError('Invalid archive session')
        with self.lock:
            if self.active or self.closing or self.reserved:raise ValueError('Archive already active or awaiting transfer')
            self.root.mkdir(parents=True,exist_ok=True)
            directory=self.root/session
            directory.mkdir()  # Never overwrite an earlier recording.
            self.directory=directory;self.session=session;self.seen=set();self.files={}
            self.bytes=0;self.error=None;self.active=True;self.reserved=True
            self.queue_peak=0;self.storage_bytes=0;self.write_max_ms=0
            (directory/'session.json').write_text(json.dumps(dict(session=session,complete=False)),encoding='utf-8')
        return self.status()

    def capture(self, frame):
        """Called on the telemetry snapshot. Metadata and archived IDs agree."""
        with self.lock:
            if not self.active:return
            frame['archive_session']=self.session
            for name,key in (('rgb','jpeg'),('depth','uint16_raw')):
                source=frame['sources'][name]
                if not source.get(key):continue
                identity=(name,source['device_frame_id'])
                if identity in self.seen:continue
                data=source[key]
                if not isinstance(data,bytes):raise ValueError('Native archive requires bytes')
                if self.bytes+len(data)>self.max_bytes:
                    self.error='Native archive disk budget exceeded';self.active=False;break
                try:self.queue.put_nowait((self.directory,identity,data,source['original_sha256']))
                except queue.Full:
                    self.error='Native archive writer queue overflow';self.active=False;break
                self.seen.add(identity);self.bytes+=len(data)
                self.queue_peak=max(self.queue_peak,self.queue.qsize())
            if self.error:frame['stats']['fatal']=self.error

    def _write(self):
        while not self.stop.is_set() or not self.queue.empty():
            try:directory,identity,data,expected=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                started=time.perf_counter_ns()
                digest=hashlib.sha256(data).hexdigest()
                if digest!=expected:raise ValueError('Native image changed before archive write')
                name,index=identity
                filename=f'{name}_{index:010d}.bin'
                stored=zstandard.ZstdCompressor(level=1,write_checksum=True).compress(data)
                (directory/filename).write_bytes(stored)
                entry=dict(source=name,id=index,file=filename,bytes=len(data),sha256=digest,
                           storage_codec='zstd',storage_bytes=len(stored),
                           x5_written_monotonic_ns=time.perf_counter_ns())
                # Filesystem I/O must never hold the capture/status lock.
                with (directory/'manifest.jsonl').open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(entry)+'\n')
                with self.lock:
                    self.files[identity]=entry
                    self.storage_bytes+=len(stored)
                    self.write_max_ms=max(self.write_max_ms,(time.perf_counter_ns()-started)/1e6)
            except Exception as exc:
                with self.lock:self.error='Native archive write failed: '+str(exc);self.active=False
            finally:self.queue.task_done()

    def seal(self, session):
        with self.lock:
            if session!=self.session:raise ValueError('Wrong archive session')
            self.active=False;self.closing=True
        self.queue.join()
        with self.lock:
            self.closing=False
            result=self._status()
            (self.directory/'session.json').write_text(json.dumps(result),encoding='utf-8')
        return result

    def _status(self):
        return dict(session=self.session,active=self.active,complete=not self.active and not self.error,
                    native_bytes=self.bytes,storage_bytes=self.storage_bytes,unique_files=len(self.files),queue_size=self.queue.qsize(),
                    queue_peak=self.queue_peak,write_max_ms=self.write_max_ms,error=self.error)

    def status(self):
        with self.lock:return self._status()

    def fetch(self, session, source, index):
        with self.lock:
            if session!=self.session or self.active or self.closing:raise ValueError('Archive must be sealed')
            if source not in ('rgb','depth') or type(index) is not int or index<0:raise ValueError('Invalid media identity')
            entry=dict(self.files[(source,index)])
            directory=self.directory
        stored=(directory/entry['file']).read_bytes()
        data=zstandard.ZstdDecompressor().decompress(stored,max_output_size=entry['bytes'])
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Archive checksum mismatch')
        return entry,data

    def release(self, session):
        with self.lock:
            if session!=self.session or self.active or self.closing:raise ValueError('Archive must be sealed')
            self.reserved=False
        return self.status()

    def close(self):
        with self.lock:self.active=False
        self.stop.set();self.worker.join(5)
