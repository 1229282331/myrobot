"""Isolated real worker/IPC/audit timing, with an explicitly synthetic plant.

Run inside a loopback-only namespace. No SDK, CAN, device socket, camera or
physical motion implementation is imported. This is not a dynamics simulator.
"""
import argparse,copy,gc,hashlib,importlib.util,json,os,select,shutil,signal,subprocess,sys,threading,time
from collections import deque
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.moveit_bridge import MoveItBridge
from e6_phosphobot.jazzy_host import CandidateInbox
from e6_phosphobot.latest_pipe import LatestJsonPipe
from e6_phosphobot.runtime_timing import frozen_startup_heap
from qualify_audit_load import Reader


def main():
 sys.setswitchinterval(.001)
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
 ap.add_argument('--variant',choices=['baseline','candidate'],required=True)
 ap.add_argument('--tag',required=True);ap.add_argument('--journals',type=int,choices=[0,2],default=2)
 ap.add_argument('--audit-variant',choices=['baseline','candidate'])
 ap.add_argument('--seconds',type=float,default=12)
 ap.add_argument('--speed-scale',type=float,default=.2)
 ap.add_argument('--orientation-assist',action='store_true')
 ap.add_argument('--tool-frame',action='store_true')
 ap.add_argument('--vary-speed',action='store_true')
 ap.add_argument('--production-runtime',action='store_true',help='Use the existing gateway startup heap policy before workers start; does not disable runtime GC')
 ap.add_argument('--linear-only',action='store_true',help='Use the same XYZ/release fixture with strict translation')
 a=ap.parse_args();root=a.root
 gc_events=[]
 def gc_event(phase,info):gc_events.append(dict(phase=phase,at_ns=time.perf_counter_ns(),**info))
 gc.callbacks.append(gc_event)
 if not 0<a.speed_scale<=1:raise ValueError('Invalid offline speed scale')
 out=root/('pipeline-'+a.tag);out.mkdir();bundle=out/'bundle';bundle.mkdir()
 for name in ('model','meshes','collision','servo_jazzy.yaml'):(bundle/name).symlink_to(Path('/qualification')/name)
 source=root/('baseline/servo_cpp_adapter' if a.variant=='baseline' else 'build/servo_cpp_adapter')
 binary=bundle/'servo_cpp_adapter';shutil.copy2(source,binary);binary.chmod(0o755)
 pin=json.loads(Path('/qualification/cpp-adapter.json').read_text());pin.update(binary=binary.name,sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
 if a.orientation_assist:
  if a.variant!='candidate':raise ValueError('Assistance requires the candidate binary')
  pin['orientation_assist_version']=1
 (bundle/'cpp-adapter.json').write_text(json.dumps(pin))
 audit_variant=a.audit_variant or a.variant
 if audit_variant=='baseline':
  spec=importlib.util.spec_from_file_location('baseline_audit',root/'baseline/audit.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);Audit=module.Audit
 else:from e6_phosphobot.audit import Audit
 fixture=json.loads((root/'fixtures.json').read_text())[1]['cycle']
 fixture['offline_test']=True
 reader=Reader();writers=[Audit(out/label,reader) for label in ('continuous','episode')[:a.journals]]
 bridge=MoveItBridge(bundle,reader,api='cpp',motion_profile='normal',tcp_profile='omnipicker-r4',collision_attachments=True)
 actual=copy.deepcopy(fixture['state']['feedback']);actual.update(generation='offline-plant',age_ms=0.,connected=True)
 grip=copy.deepcopy(fixture['state']['gripper']);grip.update(phase='ready',inflight_raw=None,arrived=True,reported_state=0,reported_velocity_raw=0)
 plant_lock=threading.Lock();queue=deque();halt=threading.Event();errors=[];counts={'raw':0,'events':0};accepted=[];poll_gaps=[];health=[];timing_gaps=[]
 def snapshot():
  with plant_lock:return copy.deepcopy(actual)
 def grip_snapshot():
  with plant_lock:return dict(grip)
 bridge.gripper_snapshot=grip_snapshot
 def raw_loop():
  target=time.perf_counter()
  previous=time.perf_counter_ns()
  try:
   while not halt.is_set():
    now=time.perf_counter_ns()
    if now-previous>20_000_000:timing_gaps.append(dict(loop='raw',start_ns=previous,end_ns=now,gap_ms=(now-previous)/1e6))
    previous=now
    with plant_lock:
     while queue and queue[0][0]<=now:
      _,row=queue.popleft();actual['status']['joint_state']['q_actual']=np.rad2deg(row['joints_rad']).tolist()
      angles=Rotation.from_quat(row['quaternion_xyzw']).as_euler('xyz',degrees=True).tolist()
      actual['status']['tool_vector_actual']=dict(zip(('x','y','z','rx','ry','rz'),[v*1000 for v in row['position_m']]+angles))
     actual['received_ns']=now;grip['received_ns']=now
    counts['raw']+=1
    reader.publish(('offline-plant',counts['raw'],SimpleNamespace(received_ns=now,prefix=b'',raw=b'x'*1440)))
    target+=.008;halt.wait(max(0,target-time.perf_counter()))
  except Exception as e:errors.append(repr(e));halt.set()
 log=(out/'child.log').open('w');proc=subprocess.Popen([sys.executable,'-B','-m','e6_phosphobot.jazzy_cpp_worker','--bundle',str(bundle),'--session',str(out/'worker'),'--device-period','.05','--motion-profile','normal','--tcp-profile','omnipicker-r4','--collision-attachments'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log)
 pipe=LatestJsonPipe(proc.stdout.fileno(),proc.stdin.fileno());inbox=CandidateInbox(bridge,pipe,out);bridge.refresh_candidate=inbox.receive
 def host_loop():
  previous=time.perf_counter_ns()
  try:
   while not halt.is_set():
    now=time.perf_counter_ns();poll_gaps.append((now-previous)/1e6)
    if now-previous>20_000_000:timing_gaps.append(dict(loop='host',start_ns=previous,end_ns=now,gap_ms=(now-previous)/1e6))
    previous=now
    if proc.poll() is not None:raise RuntimeError('worker exited')
    command,epoch=bridge.command_snapshot()
    pipe.offer(dict(command=command,epoch=epoch,actual=snapshot(),gripper=grip_snapshot(),at_ns=now));pipe.flush();inbox.receive()
    select.select([pipe.read_fd],[],[],.005)
  except Exception as e:errors.append(repr(e));halt.set()
 threads=[threading.Thread(target=raw_loop),threading.Thread(target=host_loop)]
 for t in threads:t.start()
 active_begin=None
 try:
  deadline=time.monotonic()+40
  while not bridge.snapshot().get('ready'):
   if halt.is_set() or time.monotonic()>deadline:raise RuntimeError('worker not ready: '+str(bridge.snapshot()))
   time.sleep(.02)
  start=time.perf_counter();active_begin=start;next_tick=start;sequence=0
  while time.perf_counter()-start<a.seconds and not halt.is_set():
   now=time.perf_counter()
   if now>=next_tick:
    sequence+=1;elapsed=now-start;axis=3 if elapsed<a.seconds/2 else 1
    # Repeat/reverse the retained Rx and Y directions in the synthetic plant.
    sign=1 if int(elapsed/1.5)%2==0 else -1
    if a.orientation_assist or a.linear_only:axis=int(elapsed/4)%3
    scale=a.speed_scale*(.25 if a.vary_speed and int(elapsed/3)%2 else 1)
    values=[0.]*6;values[axis]=sign*(np.deg2rad(10) if axis==3 else .03)*scale
    intent=dict(deadman=True,linear_m_s=values[:3],angular_rad_s=values[3:],joint_rad_s=[0.]*6,frame='tool' if a.tool_frame else 'base')
    if a.orientation_assist:
     intent.update(orientation_assist=True,assist_angular_rad_s=min(np.deg2rad(3),np.deg2rad(10)*scale))
    # Exercise a release and new actual-orientation anchor every four seconds.
    if (a.orientation_assist or a.linear_only) and elapsed%4>3.5:bridge.stop()
    else:bridge.submit(SimpleNamespace(model_dump=lambda:intent),sequence)
    row=copy.deepcopy(fixture);row['at_ns']=time.perf_counter_ns();row['state']['model']=bridge.snapshot();row['state']['feedback']=snapshot()
    for w in writers:w.put(row)
    counts['events']+=1;health.append(dict(at_ns=row['at_ns'],model=bridge.snapshot()))
    next_tick+=.05
   candidate=bridge.take()
   if candidate:
    accepted.append(dict(candidate=candidate,take=bridge.last_take))
    with plant_lock:queue.append((time.perf_counter_ns()+100_000_000,candidate))
   bridge.initial_candidate_event.wait(min(.005,max(0,next_tick-time.perf_counter())))
  bridge.stop();time.sleep(.3)
 finally:
  bridge.stop();halt.set()
  for t in threads:t.join(2)
  bridge.refresh_candidate=None;inbox.close()
  if proc.poll() is None:proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
  proc.stdin.close();proc.stdout.close();log.close()
  for w in writers:w.close()
  (out/'accepted.json').write_text(json.dumps(accepted));(out/'health.json').write_text(json.dumps(health))
  (out/'timing-gaps.json').write_text(json.dumps(timing_gaps))
  gc.callbacks.remove(gc_event);(out/'gc-events.json').write_text(json.dumps(gc_events))
  final=bridge.snapshot();worker_exit=json.loads((out/'worker/exit.json').read_text()) if (out/'worker/exit.json').exists() else None
  summary=dict(offline_test=True,synthetic_plant_delay_ms=100,variant=a.variant,audit_variant=audit_variant,journals=a.journals,seconds=a.seconds,speed_scale=a.speed_scale,orientation_assist=a.orientation_assist,tool_frame=a.tool_frame,vary_speed=a.vary_speed,production_startup_heap=a.production_runtime,interpreter_startup_frozen_objects=interpreter_startup_frozen_objects,binary=pin['sha256'],errors=errors,worker_exit=worker_exit,
   accepted=len(accepted),counts=counts,take_counts=bridge.take_counts,rejections=final.get('rejections',{}),
   host_poll_ms=np.percentile(poll_gaps,[50,95,100]).tolist() if poll_gaps else [],
   consume_age_ms=np.percentile([x['take']['sample_age_ms'] for x in accepted],[50,95,100]).tolist() if accepted else [],
   writers=[dict(counts=w.counts,error=w.error) for w in writers])
  (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
 return int(bool(errors) or not worker_exit or bool(worker_exit.get('error')) or any(w.error or w.counts!=counts for w in writers))


if __name__=='__main__':
 # This standalone, device-free harness has no session owner yet. Ubuntu's
 # interpreter has already frozen a small startup graph. Fold it into the
 # same post-import policy used by the host gateway; never do this on a live
 # gateway or alter the production helper's ownership guard.
 interpreter_startup_frozen_objects=gc.get_freeze_count()
 if '--production-runtime' in sys.argv:gc.unfreeze()
 with frozen_startup_heap() if '--production-runtime' in sys.argv else nullcontext():
  raise SystemExit(main())
