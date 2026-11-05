"""X5 no-device contention experiment using the unchanged production bundler.

Retained episode/media are hardlinked read-only into four distinct outputs.
A separate process produces retained SDK frames at 125 Hz over socketpair,
replacing only TimeStamp with that process's X5 monotonic send time. This is
synthetic feedback, not an E6 latency or motion acceptance result.
"""
import argparse
import base64
import gzip
import hashlib
import inspect
import json
import os
from pathlib import Path
import resource
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import numpy as np
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.episodes import Episodes
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.runtime_timing import cooperative_thread_scheduling, frozen_startup_heap

OFFSET=MyType.fields['TimeStamp'][1]
BASELINE_EPISODES_SHA256='ed50b0680483827b0da637a6539afa80cd2a0866bf16d1b0b03a741a41d8c115'


def stats(values):
    return dict(n=len(values),percentiles_ms=np.percentile(values,[50,95,99,100]).tolist(),
                over_50_ms=sum(v>50 for v in values),over_100_ms=sum(v>100 for v in values)) if values else None


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2),encoding='utf8')


def producer(args):
    data=bytearray(args.raw.read_bytes());sock=socket.socket(fileno=args.fd)
    halt=False;rows=[]
    def stop(*_):
        nonlocal halt
        halt=True
    signal.signal(signal.SIGTERM,stop)
    try:
        while not halt:
            begin=time.perf_counter_ns();struct.pack_into('<Q',data,OFFSET,begin)
            sock.sendall(data);end=time.perf_counter_ns()
            rows.append([begin,end]);time.sleep(.008)
    finally:
        sock.close()
        with gzip.open(args.output/'producer.jsonl.gz','wt',encoding='utf8') as f:
            for row in rows:f.write(json.dumps(row)+'\n')


def bundle(args):
    started=time.perf_counter_ns();usage=resource.getrusage(resource.RUSAGE_SELF)
    dummy=SimpleNamespace(root=args.output/'episodes',camera_dir=args.output/'camera',shutdown_deadline=float('inf'))
    digest=Episodes._bundle(dummy,args.episode)
    after=resource.getrusage(resource.RUSAGE_SELF)
    dump(args.output/'bundle-result.json',dict(sha256=digest,start_ns=started,end_ns=time.perf_counter_ns(),
        major_fault_delta=after.ru_majflt-usage.ru_majflt,minor_fault_delta=after.ru_minflt-usage.ru_minflt,
        pid=os.getpid(),path=str(args.output/'episodes'/args.episode/'bundle.zip')))


def link_sources(source,destination):
    for path in sorted(source.rglob('*')):
        if not path.is_file() or path.name in ('bundle.zip','bundle.zip.part'):continue
        target=destination/path.relative_to(source);target.parent.mkdir(parents=True,exist_ok=True)
        os.link(path,target)


def trial(args,mode,index):
    output=args.output/f'{index}-{mode}';output.mkdir()
    episode=output/'episodes'/args.episode;episode.mkdir(parents=True)
    media=output/'camera/media'/args.episode;media.mkdir(parents=True)
    link_sources(args.source/'episodes'/args.episode,episode)
    link_sources(args.source/'camera/media'/args.episode,media)
    left,right=socket.socketpair();used=False
    def connect(address,timeout):
        nonlocal used
        if used or address!=('synthetic-only',30004):raise RuntimeError('Device sockets prohibited')
        used=True;return left
    reader=FeedbackReader('synthetic-only',socket_factory=connect)
    sub=reader.subscribe(8192);frames=[];ticks=[];events=[];errors=[]
    cmd=[sys.executable,str(Path(__file__).resolve()),'--output',str(output),'--raw',str(args.raw)]
    child=subprocess.Popen(cmd+['--producer','--fd',str(right.fileno())],pass_fds=(right.fileno(),))
    right.close();reader.start()
    worker=None;worker_error=[]
    def run_bundle():
        try:bundle(SimpleNamespace(output=output,episode=args.episode))
        except BaseException as exc:worker_error.append(repr(exc))
    begin=time.perf_counter_ns();load_start=None;load_end=None
    usage=resource.getrusage(resource.RUSAGE_SELF)
    try:
        while (time.perf_counter_ns()-begin)<180e9:
            tick=time.perf_counter_ns();state=reader.snapshot(full=True)
            # Production snapshot/JSON path at 20 Hz; no IK or motion API.
            json.dumps(state)
            ticks.append(dict(start_ns=tick,end_ns=time.perf_counter_ns(),age_ms=state['age_ms']))
            while not sub.queue.empty():
                _,seq,frame=sub.queue.get_nowait()
                frames.append([seq,struct.unpack_from('<Q',frame.raw,OFFSET)[0],frame.received_ns])
            events.extend(reader.drain_events())
            if load_start is None and tick-begin>3e9:
                load_start=time.perf_counter_ns()
                if mode=='thread':
                    worker=threading.Thread(target=run_bundle,name='episode-bundle-fixture');worker.start()
                else:
                    worker=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--bundle',
                        '--output',str(output),'--episode',args.episode])
            if worker and load_end is None:
                done=not worker.is_alive() if mode=='thread' else worker.poll() is not None
                if done:load_end=time.perf_counter_ns()
            if load_end and tick-load_end>3e9:break
            time.sleep(max(0,.05-(time.perf_counter_ns()-tick)/1e9))
        else:errors.append('180s trial timeout')
    finally:
        if worker:
            if mode=='thread':worker.join(30)
            elif worker.poll() is None:worker.terminate();worker.wait(10)
        child.terminate();child.wait(10)
        reader.close()
        while not sub.queue.empty():
            _,seq,frame=sub.queue.get_nowait();frames.append([seq,struct.unpack_from('<Q',frame.raw,OFFSET)[0],frame.received_ns])
        events.extend(reader.drain_events());reader.unsubscribe(sub)
        after=resource.getrusage(resource.RUSAGE_SELF)
        errors.extend(worker_error)
        if mode=='process' and worker.returncode:errors.append('bundle child exit '+str(worker.returncode))
        if sub.dropped:errors.append('subscription overflow')
        with gzip.open(output/'receive.jsonl.gz','wt',encoding='utf8') as f:
            for row in frames:f.write(json.dumps(row)+'\n')
        with gzip.open(output/'ticks.jsonl.gz','wt',encoding='utf8') as f:
            for row in ticks:f.write(json.dumps(row)+'\n')
        dump(output/'receiver-events.json',events)
        result=dict(mode=mode,hardware_io=False,begin_ns=begin,load_start_ns=load_start,load_end_ns=load_end,
            end_ns=time.perf_counter_ns(),errors=errors,raw_frames=len(frames),subscription_dropped=sub.dropped,
            process_major_fault_delta=after.ru_majflt-usage.ru_majflt,
            process_minor_fault_delta=after.ru_minflt-usage.ru_minflt,metrics={})
        for label,low,high in [('before',begin,load_start),('during',load_start,load_end),('after',load_end,time.perf_counter_ns())]:
            if low is None or high is None:continue
            selected=[r for r in frames if low<=r[2]<high]
            selected_ticks=[r for r in ticks if low<=r['start_ns']<high]
            result['metrics'][label]=dict(receive_delay=stats([(r[2]-r[1])/1e6 for r in selected]),
                tick_work=stats([(r['end_ns']-r['start_ns'])/1e6 for r in selected_ticks]),
                snapshot_age=stats([r['age_ms'] for r in selected_ticks if r['age_ms'] is not None]),
                tick_interval=stats([(b['start_ns']-a['start_ns'])/1e6 for a,b in zip(selected_ticks,selected_ticks[1:])]))
        dump(output/'summary.json',result)
    return result


def main():
    # Otherwise a newer Episodes._bundle would spawn a process even in the
    # nominal "thread" arm and silently invalidate this historical comparison.
    source=Path(inspect.getfile(Episodes))
    if hashlib.sha256(source.read_bytes()).hexdigest()!=BASELINE_EPISODES_SHA256:
        raise RuntimeError('This comparison requires the archived v200 Episodes source; use its fixed release/PYTHONPATH')
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source',type=Path);p.add_argument('--episode');p.add_argument('--raw',type=Path)
    p.add_argument('--producer',action='store_true');p.add_argument('--fd',type=int);p.add_argument('--bundle',action='store_true')
    args=p.parse_args()
    if args.producer:return producer(args)
    if args.bundle:return bundle(args)
    args.output.mkdir(exist_ok=False,parents=True)
    if not args.raw:
        args.raw=args.output/'retained.raw'
        with (args.source/'episodes'/args.episode/'audit/raw.jsonl').open(encoding='utf8') as f:
            raw=base64.b64decode(json.loads(next(f))['raw_b64'])
        assert len(raw)==1440;args.raw.write_bytes(raw)
    result=dict(hardware_io=False,source=str(args.source),episode=args.episode,
        scope='Same-process production feedback parser/20Hz snapshot and bundler versus bundle subprocess. No full gateway, video, active recorder, robot or CAN.',
        source_raw_sha256=hashlib.sha256(args.raw.read_bytes()).hexdigest(),trials=[])
    with cooperative_thread_scheduling(),frozen_startup_heap():
        for index,mode in enumerate(['thread','process','process','thread'],1):
            row=trial(args,mode,index);result['trials'].append(row)
            dump(args.output/'summary.json',result)
            print(json.dumps(dict(trial=index,mode=mode,metrics=row['metrics']['during'],errors=row['errors'])),flush=True)
            if row['errors']:raise RuntimeError(str(row['errors']))


if __name__=='__main__':main()
