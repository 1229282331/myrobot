"""Camera-only minimal reproduction. No E6, CAN, preview or control process.

The existing Camera and PtsClock implementations remain authoritative. This
probe only observes call timing and rejected values; it never relaxes the PTS
validator or substitutes timestamps. Diagnostic rows are saved after capture.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


class ClockWitness:
    def __init__(self,clock):
        self.clock=clock;self.rows=deque(maxlen=10000)
    def add(self,pts):
        row=dict(at_ns=time.perf_counter_ns(),pts=pts,previous=self.clock.last,
                 hz=self.clock.hz,extended=self.clock.extended,wraps=self.clock.wraps)
        try:
            result=self.clock.add(pts);row['result']=result;return result
        except Exception as exc:
            row['error']=str(exc);raise
        finally:self.rows.append(row)


def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    import glob
    nodes=glob.glob('/dev/video*')
    if nodes and subprocess.run(['fuser',*nodes],capture_output=True).returncode==0:
        raise RuntimeError('Camera occupied; no takeover')
    sys.path.insert(0,str(args.camera_tools))
    import camera as source
    import uvc_clock
    stopped=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    rows=[];packets=deque(maxlen=10000);state=dict(recording=args.recording,status_io=args.status_io,error=None,
        duration_requested_s=args.duration_s,device_commands=0,preview=False,
        source_sha256={name:hashlib.sha256((args.camera_tools/name).read_bytes()).hexdigest()
                       for name in ('camera.py','uvc_clock.py','sdk.py') if (args.camera_tools/name).exists()})
    parser=uvc_clock.v4l2_pts
    def parse(raw,received):
        row=dict(at_ns=time.perf_counter_ns(),received_ns=received,metadata_hex=raw.hex())
        try:
            result=parser(raw,received);row.update(pts=result[0],kernel_ns=result[1]);return result
        except Exception as exc:row['error']=str(exc);raise
        finally:packets.append(row)
    uvc_clock.v4l2_pts=parse
    archive=None;camera=None;witness=None;began=None;last_end=None;outputs=None
    try:
        camera=source.Camera('both',serial='AY4A1520111',profiles={
            'color':(1280,720,5,30),'depth':(640,480,12,30)},align_mode=0,uvc_pts=True)
        witness=ClockWitness(camera.uvc_clock);camera.uvc_clock=witness
        state.update(identity=camera.info,profiles=camera.selected,clock=camera.uvc_clock_info)
        if args.async_status_io:
            from .camera_outputs import CameraOutputs
            outputs=CameraOutputs(args.output,args.health_file)
            outputs.start();state['async_status_io']=True;state['health_file']=str(args.health_file)
        if args.recording:
            from .camera_archive import CameraArchive,atomic_json
            archive=CameraArchive(args.output/'capture');archive.start()
        began=time.perf_counter_ns();last_end=began;record_started=False;record_finished=False
        rgb_at=depth_at=0;latest={}
        while not stopped.is_set() and (time.perf_counter_ns()-began)/1e9<args.duration_s:
            before=time.perf_counter_ns();cpu_before=time.thread_time_ns()
            row=dict(read_started_ns=before,consumer_gap_ns=before-last_end)
            try:
                frames=camera.read()
                after=time.perf_counter_ns()
                row.update(read_duration_ns=after-before,read_cpu_ns=time.thread_time_ns()-cpu_before,
                           frames=[{k:v for k,v in meta.items() if k!='_original_bytes'} for meta,_ in frames])
                if archive:
                    for meta,data in frames:archive.offer(meta,data)
                    elapsed=(after-began)/1e9
                    if elapsed>=5 and not record_started:
                        atomic_json(archive.request_path,dict(id='probe_start',op='start',session='camera_probe_recording'))
                        record_started=True
                    if elapsed>=args.duration_s-5 and not record_finished:
                        atomic_json(archive.request_path,dict(id='probe_finish',op='finish',session='camera_probe_recording'))
                        record_finished=True
                    row['archive_status']=archive.public_status
                if args.status_io:
                    from .camera_service import atomic
                    writes=[]
                    def write(name,data):
                        begin=time.perf_counter_ns();atomic(args.output/name,data)
                        writes.append(dict(name=name,begin_ns=begin,duration_ns=time.perf_counter_ns()-begin))
                    for meta,data in frames:
                        latest[meta['stream']]=meta
                        if meta['stream']=='color' and after-rgb_at>=1_000_000_000:
                            write('color.jpg',data);write('color.json',json.dumps(meta).encode());rgb_at=after
                        if meta['stream']=='depth' and after-depth_at>=200_000_000:
                            write('depth.uint16',data);depth_at=after
                        write('health.json',json.dumps(dict(latest=latest,recording=archive.public_status if archive else None)).encode())
                    row['status_writes']=writes
                if outputs:
                    for meta,data in frames:
                        latest[meta['stream']]=meta
                        outputs.offer(meta,data,dict(latest=dict(latest),recording=archive.public_status if archive else None))
                row['consumer_duration_ns']=time.perf_counter_ns()-after
            except Exception as exc:row['error']=str(exc);raise
            finally:
                rows.append(row);last_end=time.perf_counter_ns()
    except Exception as exc:state['error']=str(exc)
    finally:
        state['elapsed_s']=None if began is None else (time.perf_counter_ns()-began)/1e9
        state['interrupted']=stopped.is_set();state['cleanup_errors']=[]
        if camera:
            try:state['camera_close']=camera.close()
            except Exception as exc:state['cleanup_errors'].append(str(exc))
        if archive:
            try:archive.close()
            except Exception as exc:state['cleanup_errors'].append(str(exc))
            state['archive']=archive.status()
        if outputs:
            state['cleanup_errors'].extend(outputs.close(dict(closed_ns=time.perf_counter_ns())))
            state['outputs']=outputs.diagnostics
        uvc_clock.v4l2_pts=parser
        for name,data in [('calls',rows),('uvc_metadata',packets),('pts',[] if witness is None else witness.rows)]:
            with (args.output/(name+'.jsonl')).open('x') as stream:
                for row in data:stream.write(json.dumps(row)+'\n')
        (args.output/'summary.json').write_text(json.dumps(state,indent=2))
        print(json.dumps(state),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--camera-tools',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--duration-s',type=float,default=120)
    p.add_argument('--recording',action='store_true');p.add_argument('--status-io',action='store_true')
    p.add_argument('--async-status-io',action='store_true');p.add_argument('--health-file',type=Path);args=p.parse_args()
    if args.async_status_io and (args.status_io or not args.health_file):raise ValueError('Async output needs --health-file and excludes --status-io')
    if not 10<=args.duration_s<=120:raise ValueError('Probe duration must be 10–120 s')
    run(args)
