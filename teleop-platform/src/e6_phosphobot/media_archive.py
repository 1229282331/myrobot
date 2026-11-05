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
    def __init__(self, root, max_bytes=2*1024**3, queue_size=512,
                 max_buffer_bytes=128*1024**2):
        self.root=Path(root)
        self.max_bytes=max_bytes
        if queue_size<=0 or max_buffer_bytes<=0:
            raise ValueError('Archive buffer limits must be positive')
        self.queue=queue.Queue(maxsize=queue_size)
        self.write_queue=queue.Queue(maxsize=queue_size)
        # RGB, depth and sample rows share this FIFO: 128 entries at 20 Hz
        # cover only ~2.1 s, less than the measured 2.28 s SD write stall.
        # Bound both item overhead and retained payload, including the writer's
        # in-flight item. This absorbs finite bursts, not sustained overload.
        self.max_buffer_bytes=max_buffer_bytes
        self.pending_bytes=0;self.buffer_peak_bytes=0;self.pending_items=0
        self.lock=threading.Lock()
        self.session=None; self.directory=None; self.seen=set(); self.files={}
        self.bytes=0; self.error=None; self.active=False; self.closing=False
        self.reserved=False
        self.queue_peak=0;self.storage_bytes=0;self.write_max_ms=0
        self.compression_max_ms=0;self.compressed_frames=0
        self.sample_records=0;self.sample_write_max_ms=0
        self.stop=threading.Event()
        self.compression_done=threading.Event()
        self.compressor=threading.Thread(target=self._compress,name='native-media-compress',daemon=True)
        self.worker=threading.Thread(target=self._write,name='native-media-write',daemon=True)
        self.compressor.start()
        self.worker.start()

    def begin(self, session):
        if not isinstance(session,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',session):
            raise ValueError('Invalid archive session')
        with self.lock:
            if self.active or self.closing or self.reserved or self.pending_items:raise ValueError('Archive already active or awaiting transfer')
            self.root.mkdir(parents=True,exist_ok=True)
            directory=self.root/session
            directory.mkdir()  # Never overwrite an earlier recording.
            self.directory=directory;self.session=session;self.seen=set();self.files={}
            self.bytes=0;self.error=None;self.active=True;self.reserved=True
            self.queue_peak=0;self.storage_bytes=0;self.write_max_ms=0
            self.compression_max_ms=0;self.compressed_frames=0
            self.pending_bytes=0;self.buffer_peak_bytes=0
            self.sample_records=0;self.sample_write_max_ms=0
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
                    self.error='Native image payload budget exceeded';self.active=False;break
                metadata={k:v for k,v in source.items() if k not in ('jpeg','uint16_raw')}
                if not self._enqueue((self.directory,identity,data,source['original_sha256'],metadata),len(data)):
                    break
                self.seen.add(identity);self.bytes+=len(data)
            if self.error:frame['stats']['fatal']=self.error

    def _enqueue(self,item,size):
        """Caller holds lock; a failed admission never takes byte ownership."""
        # Keep the original total allowance: queue slots plus one in-flight
        # item. Splitting compression and writing must not double that bound.
        if self.pending_items>=self.queue.maxsize+1:
            self.error='Native archive writer queue overflow';self.active=False
            return False
        if self.pending_bytes+size>self.max_buffer_bytes:
            self.error='Native archive writer payload buffer exceeded';self.active=False
            return False
        try:self.queue.put_nowait((*item,size))
        except queue.Full:
            self.error='Native archive writer queue overflow';self.active=False
            return False
        self.pending_bytes+=size
        self.pending_items+=1
        self.buffer_peak_bytes=max(self.buffer_peak_bytes,self.pending_bytes)
        self.queue_peak=max(self.queue_peak,max(0,self.pending_items-1))
        return True

    def enqueue_sample(self,index,line):
        """Journal metadata shares the bounded FIFO with its preceding images."""
        if not isinstance(line,str) or len(line)>1024*1024:raise ValueError('Invalid camera sample line')
        with self.lock:
            if not self.reserved or self.closing:raise ValueError('No open camera sample journal')
            if not self._enqueue((self.directory,('sample',index),line,None,None),len(line.encode('utf-8'))):
                raise RuntimeError(self.error)

    @staticmethod
    def _write_sample(directory,line):
        with (directory/'camera_samples.jsonl').open('a',encoding='utf-8') as stream:stream.write(line)

    def _compress(self):
        """Keep lossless encoding progressing while the storage thread stalls.

        The 128 MiB budget counts queued/in-flight payload at both stages.
        As before, one codec's temporary output is not a process RSS bound.
        Original sample rows traverse the same FIFO, after their image jobs.
        """
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:directory,identity,data,expected,metadata,payload_bytes=self.queue.get(timeout=.1)
                except queue.Empty:continue
                transferred=False;stored=None
                try:
                    original_bytes=payload_bytes
                    if identity[0]!='sample':
                        started=time.perf_counter_ns()
                        if hashlib.sha256(data).hexdigest()!=expected:
                            raise ValueError('Native image changed before archive write')
                        stored=zstandard.ZstdCompressor(level=1,write_checksum=True).compress(data)
                        elapsed_ms=(time.perf_counter_ns()-started)/1e6
                        with self.lock:
                            new_total=self.pending_bytes-payload_bytes+len(stored)
                            if new_total>self.max_buffer_bytes:
                                raise RuntimeError('Native archive writer payload buffer exceeded')
                            self.pending_bytes=new_total
                            self.buffer_peak_bytes=max(self.buffer_peak_bytes,new_total)
                            self.compressed_frames+=1
                            self.compression_max_ms=max(self.compression_max_ms,elapsed_ms)
                        data=stored;payload_bytes=len(stored)
                    self.write_queue.put((directory,identity,data,expected,metadata,payload_bytes,original_bytes))
                    transferred=True
                except Exception as exc:
                    with self.lock:self.error='Native archive compression failed: '+str(exc);self.active=False
                finally:
                    if not transferred:
                        with self.lock:self.pending_bytes-=payload_bytes;self.pending_items-=1
                    del data
                    del stored
                    self.queue.task_done()
        finally:self.compression_done.set()

    def _write(self):
        while not self.compression_done.is_set() or not self.write_queue.empty():
            try:directory,identity,data,expected,metadata,payload_bytes,original_bytes=self.write_queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                started=time.perf_counter_ns()
                if identity[0]=='sample':
                    self._write_sample(directory,data)
                    with self.lock:
                        self.sample_records+=1
                        self.sample_write_max_ms=max(self.sample_write_max_ms,(time.perf_counter_ns()-started)/1e6)
                    continue
                name,index=identity
                filename=f'{name}_{index:010d}.bin'
                (directory/filename).write_bytes(data)
                entry=dict(source=name,id=index,file=filename,bytes=original_bytes,sha256=expected,
                           metadata=metadata,
                           storage_codec='zstd',storage_bytes=len(data),
                           x5_written_monotonic_ns=time.perf_counter_ns())
                # Filesystem I/O must never hold the capture/status lock.
                with (directory/'manifest.jsonl').open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(entry)+'\n')
                with self.lock:
                    self.files[identity]=entry
                    self.storage_bytes+=len(data)
                    self.write_max_ms=max(self.write_max_ms,(time.perf_counter_ns()-started)/1e6)
            except Exception as exc:
                with self.lock:self.error='Native archive write failed: '+str(exc);self.active=False
            finally:
                with self.lock:self.pending_bytes-=payload_bytes;self.pending_items-=1
                del data
                self.write_queue.task_done()

    def seal(self, session):
        with self.lock:
            if session!=self.session:raise ValueError('Wrong archive session')
            self.active=False;self.closing=True
        self.queue.join()
        self.write_queue.join()
        with self.lock:
            self.closing=False
            result=self._status()
            (self.directory/'session.json').write_text(json.dumps(result),encoding='utf-8')
        return result

    def _status(self):
        return dict(session=self.session,active=self.active,complete=not self.active and not self.error,
                    native_bytes=self.bytes,native_budget_bytes=self.max_bytes,
                    storage_bytes=self.storage_bytes,unique_files=len(self.files),queue_size=max(0,self.pending_items-1),
                    compression_queue_size=self.queue.qsize(),storage_queue_size=self.write_queue.qsize(),
                    queue_peak=self.queue_peak,write_max_ms=self.write_max_ms,
                    compressed_frames=self.compressed_frames,compression_max_ms=self.compression_max_ms,
                    write_timing_scope='storage_and_manifest_only',
                    queue_capacity=self.queue.maxsize,buffer_payload_limit_bytes=self.max_buffer_bytes,
                    buffer_payload_bytes=self.pending_bytes,buffer_payload_peak_bytes=self.buffer_peak_bytes,
                    sample_records=self.sample_records,sample_write_max_ms=self.sample_write_max_ms,error=self.error)

    def status(self):
        with self.lock:return self._status()

    def fetch(self, session, source, index):
        with self.lock:
            if session!=self.session or self.active or self.closing or self.pending_items:raise ValueError('Archive must be sealed')
            if source not in ('rgb','depth') or type(index) is not int or index<0:raise ValueError('Invalid media identity')
            entry=dict(self.files[(source,index)])
            directory=self.directory
        stored=(directory/entry['file']).read_bytes()
        data=zstandard.ZstdDecompressor().decompress(stored,max_output_size=entry['bytes'])
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Archive checksum mismatch')
        return entry,data

    def release(self, session):
        with self.lock:
            if session!=self.session or self.active or self.closing or self.pending_items:raise ValueError('Archive must be sealed')
            self.reserved=False
        return self.status()

    def close(self):
        with self.lock:self.active=False
        self.stop.set();deadline=time.monotonic()+5
        self.compressor.join(max(0,deadline-time.monotonic()))
        self.worker.join(max(0,deadline-time.monotonic()))
        if self.compressor.is_alive() or self.worker.is_alive():raise RuntimeError('Native media writer failed to exit')
