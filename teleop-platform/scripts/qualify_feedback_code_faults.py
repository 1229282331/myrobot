"""No-device, process-local code-page reclaim challenge for feedback residency.

Only this test process's read/execute NumPy mapping is advised DONTNEED, then
its clean file-cache range is advised DONTNEED. No file contents, global cache
settings, priorities, device connections or production configuration change.
"""
import argparse
import ctypes
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import socket
import struct
import subprocess
import sys
import time

import numpy as np
from qualify_feedback_residency import (Audit, FeedbackReader, NativeExecutor, OFFSET,
    PRODUCER, cooperative_thread_scheduling, frozen_startup_heap, proc, save, stats)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=['default','locked'],required=True)
    parser.add_argument('--raw',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    raw=args.raw.read_bytes();assert len(raw)==1440
    library=Path(np.core._multiarray_umath.__file__).resolve()
    mappings=[]
    for line in Path('/proc/self/maps').read_text().splitlines():
        fields=line.split()
        if fields[-1]==str(library) and fields[1]=='r-xp':
            start,end=[int(value,16) for value in fields[0].split('-')]
            mappings.append(dict(start=start,length=end-start,offset=int(fields[2],16)))
    assert mappings and sum(m['length'] for m in mappings)<32*1024**2
    libc=ctypes.CDLL(None,use_errno=True)
    libc.madvise.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_int]
    libc.madvise.restype=ctypes.c_int
    left,right=socket.socketpair();used=False
    def connect(address,timeout):
        nonlocal used
        assert address==('synthetic-only',30004) and not used
        used=True;return left
    executor=NativeExecutor('synthetic-only','shadow',None,servo_duration_s=.1)
    reader=FeedbackReader('synthetic-only',socket_factory=connect)
    executor.reader=reader;executor.camera=lambda:dict(valid=True,age_ms=0,test_fixture=True)
    subscription=reader.subscribe(8192)
    audit=None;producer=None;locked=False;lock_result=None;errors=[]
    before=proc();save(args.output/'before.json',before)
    vmsize=int(next(x.split()[1] for x in before['status'].splitlines() if x.startswith('VmSize:')))*1024
    available=int(next(x.split()[1] for x in before['meminfo'].splitlines() if x.startswith('MemAvailable:')))*1024
    assert vmsize<350*1024**2 and available>1024**3
    frames=[];ticks=[];challenges=[];events=[]
    try:
        if args.mode=='locked':
            started=time.perf_counter_ns();result=libc.mlockall(1);err=ctypes.get_errno()
            lock_result=dict(start_ns=started,end_ns=time.perf_counter_ns(),returncode=result,errno=err)
            if result:raise OSError(err,os.strerror(err))
            locked=True
        save(args.output/'started.json',dict(pid=os.getpid(),mode=args.mode,memory_lock=lock_result,
            resources=proc(),hardware_io=False,mappings=mappings,library=str(library),
            library_sha256=hashlib.sha256(library.read_bytes()).hexdigest()))
        audit=Audit(args.output/'audit',reader)
        producer=subprocess.Popen([sys.executable,'-I','-S','-c',PRODUCER,str(right.fileno()),
            str(args.raw),str(OFFSET),str(args.output/'producer.json')],pass_fds=(right.fileno(),))
        right.close();reader.start()
        begin=time.perf_counter_ns();usage=resource.getrusage(resource.RUSAGE_SELF)
        next_challenge=begin+2_000_000_000
        with library.open('rb') as libfile:
            while time.perf_counter_ns()-begin<15_000_000_000:
                start=time.perf_counter_ns()
                if start>=next_challenge and len(challenges)<6:
                    row=dict(start_ns=start,mappings=[])
                    for mapping in mappings:
                        ctypes.set_errno(0)
                        rc=libc.madvise(mapping['start'],mapping['length'],4)
                        item=dict(**mapping,returncode=rc,errno=ctypes.get_errno())
                        if rc==0:
                            os.posix_fadvise(libfile.fileno(),mapping['offset'],mapping['length'],os.POSIX_FADV_DONTNEED)
                        row['mappings'].append(item)
                    row['end_ns']=time.perf_counter_ns();challenges.append(row)
                    next_challenge+=2_000_000_000
                executor.tick();state=executor.state()
                audit.put(dict(event='cycle',at_ns=start,state=state))
                ticks.append(dict(start_ns=start,end_ns=time.perf_counter_ns(),age_ms=state['feedback']['age_ms']))
                while not subscription.queue.empty():
                    _,seq,frame=subscription.queue.get_nowait()
                    frames.append([seq,struct.unpack_from('<Q',frame.raw,OFFSET)[0],frame.received_ns])
                events.extend(reader.drain_events())
                if audit.error:raise RuntimeError(audit.error)
                time.sleep(max(0,.02-(time.perf_counter_ns()-start)/1e9))
        after=resource.getrusage(resource.RUSAGE_SELF)
        summary=dict(mode=args.mode,hardware_io=False,begin_ns=begin,end_ns=time.perf_counter_ns(),
            major_fault_delta=after.ru_majflt-usage.ru_majflt,minor_fault_delta=after.ru_minflt-usage.ru_minflt,
            receive_delay_ms=stats([(r[2]-r[1])/1e6 for r in frames]),
            challenges=challenges,windows=[])
        for c in challenges:
            selected=[r for r in frames if c['start_ns']<=r[2]<c['end_ns']+250_000_000]
            summary['windows'].append(dict(start_ns=c['start_ns'],challenge_ms=(c['end_ns']-c['start_ns'])/1e6,
                receive_delay_ms=stats([(r[2]-r[1])/1e6 for r in selected])))
        save(args.output/'summary.json',summary)
        print(json.dumps({k:v for k,v in summary.items() if k not in ('challenges','windows')}),flush=True)
    except BaseException as exc:
        errors.append(repr(exc));raise
    finally:
        if producer is not None:
            producer.terminate();producer.wait(5)
        reader.close();reader.unsubscribe(subscription)
        if audit is not None:audit.close()
        if locked:libc.munlockall()
        events.extend(reader.drain_events())
        for name,rows in [('frames',frames),('ticks',ticks)]:
            with gzip.open(args.output/(name+'.jsonl.gz'),'wt',encoding='utf8') as f:
                for row in rows:f.write(json.dumps(row)+'\n')
        save(args.output/'receiver-events.json',events);save(args.output/'after.json',proc())
        save(args.output/'exit.json',dict(errors=errors,audit_error=None if audit is None else audit.error,
            producer_exit=None if producer is None else producer.returncode,subscription_dropped=subscription.dropped,
            memory_unlocked=True,hardware_io=False))


if __name__=='__main__':
    with cooperative_thread_scheduling(),frozen_startup_heap():main()
