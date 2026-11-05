"""Bounded, independent evidence writer. Never called by the raw receive thread."""
import json
import pickle
import queue
import subprocess
import sys
import threading
import time


# 65.5 seconds at the SDK's 125 Hz, approximately 12 MiB of raw payload.
# Cold native startup coincided with stalls beyond the old 16 s buffer.
# This queue stores evidence only; the controller reads FeedbackReader.latest.
RAW_CAPACITY = 8192
EVENT_CAPACITY = 4096


class Audit:
    def __init__(self,path,reader):
        self.path=path
        path.mkdir(parents=True,exist_ok=False)
        self.reader=reader
        self.raw=reader.subscribe(RAW_CAPACITY)
        self.events=queue.Queue(EVENT_CAPACITY)
        self.halt=threading.Event()
        self.error=None
        self.counts=dict(raw=0,events=0)
        self.writer_receipt_received=False
        # A clean interpreter inherits only these private pipes. JSON/base64
        # and file writes no longer contend for the control process's GIL.
        # The bounded in-process queues remain the receiver/control boundary.
        self.process=None
        self.thread=threading.Thread(target=self._run,daemon=True,name='e6-audit-writer')
        self.thread.start()
    def put(self,event):
        if self.error:return
        try:self.events.put_nowait(event)
        except queue.Full:self.error='audit_event_overflow'
    def _run(self):
        try:
            self.process=subprocess.Popen([sys.executable,'-B','-m','e6_phosphobot.audit_writer',str(self.path)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,close_fds=True)
            with self.process.stdin as destination:
                while not self.halt.is_set() or not self.raw.queue.empty() or not self.events.empty():
                    if self.raw.dropped:raise RuntimeError('audit_raw_overflow')
                    if self.process.poll() is not None:raise RuntimeError('audit_writer_exited')
                    try:
                        generation,sequence,frame=self.raw.queue.get(timeout=.01)
                        destination.write(pickle.dumps(('raw',(generation,sequence,frame.received_ns,
                            frame.prefix,frame.raw)),protocol=5))
                    except queue.Empty:pass
                    # Fair bounded batches: a continuous event producer must
                    # not prevent original raw feedback from being drained.
                    for _ in range(8):
                        try:event=self.events.get_nowait()
                        except queue.Empty:break
                        destination.write(pickle.dumps(('events',event),protocol=5))
                    destination.flush()
                destination.write(pickle.dumps((None,None),protocol=5))
        except Exception as exc:
            if not self.error:self.error=str(exc)
        finally:
            if self.process:
                # Closing the private pipe also lets a failed run flush its
                # partial journal and report counts. Preserve the original
                # failure; do not kill a healthy child just because a queue
                # overflowed in the parent.
                try:
                    self.process.wait(timeout=3)
                    result=json.loads(self.process.stdout.read())
                    self.counts=result['counts']
                    self.writer_receipt_received=True
                    if result['error'] and not self.error:self.error=result['error']
                    if self.process.returncode and not self.error:
                        self.error='audit_writer_exit_'+str(self.process.returncode)
                except Exception as exc:
                    if not self.error:self.error=str(exc)
                if self.process.poll() is None:
                    self.process.terminate()
                    try:self.process.wait(timeout=1)
                    except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1)
                if self.process.stderr:
                    detail=self.process.stderr.read().decode('utf-8',errors='replace')[-4096:]
                    if detail:self.error=(self.error or 'audit_writer_error')+': '+detail
                    self.process.stderr.close()
                if self.process.stdout:self.process.stdout.close()
    def close(self):
        self.reader.unsubscribe(self.raw)
        self.halt.set();self.thread.join(5)
        if self.thread.is_alive():
            self.error='audit_writer_did_not_exit'
            if self.process and self.process.poll() is None:self.process.terminate()
            self.thread.join(2)
        (self.path/'summary.json').write_text(json.dumps(dict(counts=self.counts,error=self.error,
                                                writer_receipt_received=self.writer_receipt_received,
                                                raw_capacity=RAW_CAPACITY,event_capacity=EVENT_CAPACITY,
                                                raw_dropped=self.raw.dropped,
                                                closed_ns=time.perf_counter_ns())),encoding='utf-8')
