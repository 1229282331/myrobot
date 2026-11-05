"""No-device SD read / feedback residency comparison on the X5.

Uses the actual feedback parser, audit writer and idle gateway state. A separate
stdlib producer sends retained frames through socketpair; only TimeStamp is
replaced by same-kernel monotonic send time. No real socket or device start.
MCL_CURRENT is scoped to this process and released on exit; no system tuning.
"""
import argparse
import base64
import ctypes
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import socket
import struct
import subprocess
import sys
import time
import numpy as np
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.audit import Audit
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.runtime_timing import cooperative_thread_scheduling,frozen_startup_heap

OFFSET=MyType.fields['TimeStamp'][1]


def save(path,value):Path(path).write_text(json.dumps(value,indent=2),encoding='utf8')


def proc():
    status=Path('/proc/self/status').read_text()
    return dict(ns=time.perf_counter_ns(),status=status,
                meminfo=Path('/proc/meminfo').read_text(),vmstat=Path('/proc/vmstat').read_text(),
                usage=list(resource.getrusage(resource.RUSAGE_SELF)))


def stats(values):
    return dict(n=len(values),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
                p99=float(np.percentile(values,99)),max=max(values),over_50=sum(v>50 for v in values),
                over_100=sum(v>100 for v in values)) if values else None


PRODUCER='''import json,socket,struct,signal,sys,time
from pathlib import Path
sock=socket.socket(fileno=int(sys.argv[1]));raw=bytearray(Path(sys.argv[2]).read_bytes());offset=int(sys.argv[3]);rows=[];halt=False
def stop(*args):
 global halt
 halt=True
signal.signal(signal.SIGTERM,stop)
try:
 while not halt:
  started=time.perf_counter_ns();struct.pack_into('<Q',raw,offset,started)
  sock.sendall(raw);rows.append([started,time.perf_counter_ns()]);time.sleep(.008)
finally:
 sock.close();Path(sys.argv[4]).write_text(json.dumps(rows))
'''


READ_LOAD='''import hashlib,json,sys,time
from pathlib import Path
paths=json.loads(sys.argv[1]);rows=[]
for iteration in range(2):
 for name in paths:
  p=Path(name);started=time.perf_counter_ns();h=hashlib.sha256();size=0
  with p.open('rb') as f:
   for block in iter(lambda:f.read(1048576),b''):h.update(block);size+=len(block)
  rows.append(dict(iteration=iteration,path=str(p),start_ns=started,end_ns=time.perf_counter_ns(),bytes=size,sha256=h.hexdigest()))
Path(sys.argv[2]).write_text(json.dumps(rows))
'''


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['default','locked'],required=True)
    p.add_argument('--raw',type=Path,required=True);p.add_argument('--bundles',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    raw=a.raw.read_bytes();assert len(raw)==1440
    bundle_paths=json.loads(a.bundles.read_text());assert len(bundle_paths)==4
    assert all(Path(n).is_file() for n in bundle_paths)
    before=proc();save(a.output/'before.json',before)
    available=int(next(line.split()[1] for line in before['meminfo'].splitlines() if line.startswith('MemAvailable:')))*1024
    vmsize=int(next(line.split()[1] for line in before['status'].splitlines() if line.startswith('VmSize:')))*1024
    assert available>1024**3 and vmsize<350*1024**2,(available,vmsize)
    left,right=socket.socketpair();used=False
    def connect(address,timeout):
        nonlocal used
        if used or address!=('synthetic-only',30004):raise RuntimeError('Real device sockets prohibited')
        used=True;return left
    e=NativeExecutor('synthetic-only','shadow',None,servo_duration_s=.1)
    reader=FeedbackReader('synthetic-only',socket_factory=connect);e.reader=reader
    e.camera=lambda:dict(valid=True,age_ms=0,test_fixture=True)
    # No executor.start(): the injected socketpair reader is the sole source.
    subscription=reader.subscribe(8192)
    audit=Audit(a.output/'audit',reader)
    producer=subprocess.Popen([sys.executable,'-I','-S','-c',PRODUCER,str(right.fileno()),str(a.raw),str(OFFSET),str(a.output/'producer.json')],pass_fds=(right.fileno(),))
    right.close();reader.start()
    frames=[];ticks=[];errors=[];events=[];load=None;load_start=None;load_end=None
    libc=ctypes.CDLL(None,use_errno=True);locked=False;lock_result=None
    try:
        if a.mode=='locked':
            started=time.perf_counter_ns();result=libc.mlockall(1);err=ctypes.get_errno()
            lock_result=dict(start_ns=started,end_ns=time.perf_counter_ns(),returncode=result,errno=err)
            if result:raise OSError(err,os.strerror(err))
            locked=True
        save(a.output/'started.json',dict(mode=a.mode,memory_lock=lock_result,resources=proc(),pid=os.getpid(),hardware_io=False))
        begin=time.perf_counter_ns();usage=resource.getrusage(resource.RUSAGE_SELF)
        while time.perf_counter_ns()-begin<150e9:
            start=time.perf_counter_ns();e.tick();state=e.state();json.dumps(state)
            audit.put(dict(event='cycle',at_ns=start,state=state))
            ticks.append(dict(start_ns=start,end_ns=time.perf_counter_ns(),age_ms=state['feedback']['age_ms']))
            while not subscription.queue.empty():
                generation,seq,frame=subscription.queue.get_nowait()
                frames.append([seq,struct.unpack_from('<Q',frame.raw,OFFSET)[0],frame.received_ns])
            events.extend(reader.drain_events())
            if load_start is None and start-begin>3e9:
                load_start=time.perf_counter_ns()
                load=subprocess.Popen([sys.executable,'-I','-S','-c',READ_LOAD,json.dumps(bundle_paths),str(a.output/'read-load.json')])
            if load is not None and load_end is None and load.poll() is not None:
                load_end=time.perf_counter_ns()
                if load.returncode:raise RuntimeError('Read workload failed')
            if load_end and start-load_end>3e9:break
            if audit.error:raise RuntimeError(audit.error)
            time.sleep(max(0,.05-(time.perf_counter_ns()-start)/1e9))
        else:raise TimeoutError('Bounded comparison exceeded 150 seconds')
        after=resource.getrusage(resource.RUSAGE_SELF)
        result=dict(mode=a.mode,hardware_io=False,begin_ns=begin,load_start_ns=load_start,load_end_ns=load_end,
                    end_ns=time.perf_counter_ns(),major_fault_delta=after.ru_majflt-usage.ru_majflt,
                    minor_fault_delta=after.ru_minflt-usage.ru_minflt,metrics={})
        for name,low,high in [('before',begin,load_start),('during',load_start,load_end),('after',load_end,result['end_ns'])]:
            selected=[r for r in frames if low<=r[2]<high];t=[r for r in ticks if low<=r['start_ns']<high]
            result['metrics'][name]=dict(receive_delay_ms=stats([(r[2]-r[1])/1e6 for r in selected]),
              work_ms=stats([(r['end_ns']-r['start_ns'])/1e6 for r in t]),
              interval_ms=stats([(b['start_ns']-a['start_ns'])/1e6 for a,b in zip(t,t[1:])]))
        save(a.output/'summary.json',result);print(json.dumps(result),flush=True)
    except BaseException as exc:
        errors.append(repr(exc));raise
    finally:
        if load is not None and load.poll() is None:load.terminate();load.wait(5)
        producer.terminate();producer.wait(5);reader.close()
        audit.close();events.extend(reader.drain_events());reader.unsubscribe(subscription)
        if locked:libc.munlockall()
        for name,rows in [('frames',frames),('ticks',ticks)]:
            with gzip.open(a.output/(name+'.jsonl.gz'),'wt',encoding='utf8') as f:
                for row in rows:f.write(json.dumps(row)+'\n')
        save(a.output/'receiver-events.json',events)
        save(a.output/'after.json',proc())
        save(a.output/'exit.json',dict(errors=errors,audit_error=audit.error,subscription_dropped=subscription.dropped,
             producer_exit=producer.returncode,load_exit=load.returncode if load is not None else None,
             memory_unlocked=True,hardware_io=False))


if __name__=='__main__':
    with cooperative_thread_scheduling(),frozen_startup_heap():main()
