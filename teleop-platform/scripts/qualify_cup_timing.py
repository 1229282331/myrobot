"""Offline retained-pose Servo regression. No device client or motion endpoint.

The synthetic plant follows candidates after a bounded frame delay. This is a
solver/recording timing test, not a physical dynamics or cup-grasp simulation.
"""
import argparse,collections,hashlib,json,select,subprocess,time
from pathlib import Path
import numpy as np
import yaml
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.tool_frame import get_tool
from scipy.spatial.transform import Rotation


def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--variant',choices=['baseline','candidate'],required=True)
 ap.add_argument('--quick',action='store_true');a=ap.parse_args();root=a.root
 out=root/('timing-'+a.variant);out.mkdir(exist_ok=False)
 params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 binary=root/('baseline/servo_cpp_adapter' if a.variant=='baseline' else 'build/servo_cpp_adapter')
 fixtures=json.loads((root/'fixtures.json').read_text());tool=get_tool('omnipicker-r4');ramp=IntentRamp(.2)
 seq=epoch=0;last=0.;cases=[];frames=[]
 with (out/'solver.log').open('w') as log:
  proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  def step(q,v,active,raw,sent,label,frame=False):
   nonlocal seq,last
   time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
   intent=dict(linear_m_s=list(v[:3]),angular_rad_s=list(v[3:]),joint_rad_s=[0]*6,frame='tool' if frame else 'base')
   if active:
    shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s']
   else:ramp.reset();effective=[0]*6
   started=time.perf_counter_ns()
   proc.stdin.write(' '.join(map(str,[seq,epoch,int(active),1,int(frame),*q,*effective,'G3',raw,sent,-1,sent,1]))+'\n');proc.stdin.flush()
   if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('Offline solver response')
   reply=json.loads(proc.stdout.readline());ended=time.perf_counter_ns();assert reply['seq']==seq
   fk=reply['target_fk'];p,o=tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
   row=dict(case=label,active=active,q=list(q),requested=list(v),raw=raw,sent=sent,reply=reply,tcp=p.tolist(),orientation=o.as_quat().tolist(),roundtrip_ms=(ended-started)/1e6)
   frames.append(row);return row
  try:
   for fixture in fixtures:
    state=fixture['cycle']['state'];raw=round(state['gripper']['actual']*255);sent=state['gripper']['sent_raw']
    for delay in ((2,) if a.quick else (0,2,4)):
     for axis in ((1,3) if a.quick else range(6)):
      for frame in ((False,) if a.quick else (False,True)):
       q=np.array(state['actual']['joints_rad']);epoch+=1;label=f"{fixture['name']}-{delay}-{axis}-{frame}"
       for _ in range(4):start=step(q,np.zeros(6),False,raw,sent,label,frame)
       pending=collections.deque([q.copy() for _ in range(delay)]);active=[]
       for sign in (1.,-1.,1.):
        for _ in range(20):
         v=np.zeros(6);v[axis]=sign*(.006 if axis<3 else .034906585)
         row=step(q,v,True,raw,sent,label,frame);active.append(row)
         pending.append(np.array(row['reply']['joints_rad']));q=pending.popleft()
       release=step(q,np.zeros(6),False,raw,sent,label,frame)
       assert release['reply']['reset'] and np.max(abs(np.array(release['reply']['joints_rad'])-q))<1e-12
       replies=[x['reply'] for x in active];reasons=collections.Counter(x['collision_retreat']['reason'] for x in replies)
       case=dict(case=label,frames=len(replies),reasons=dict(reasons),budgets=sum(x['collision_retreat']['budget_exhausted'] or x['gripper_check']['budget_exhausted'] for x in replies),
        stops=sum(x['status'] in (-1,2,5,6) for x in replies),callback_ms=np.percentile([x['callback_us']/1000 for x in replies],[50,95,100]).tolist(),
        roundtrip_ms=np.percentile([x['roundtrip_ms'] for x in active],[50,95,100]).tolist())
       cases.append(case);print(json.dumps(case),flush=True)
  finally:
   proc.stdin.close();proc.wait(timeout=5);proc.stdout.close()
   (out/'frames.json').write_text(json.dumps(frames));(out/'cases.json').write_text(json.dumps(cases,indent=2))
   (out/'summary.json').write_text(json.dumps(dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),cases=len(cases),frames=len(frames),budgets=sum(x['budgets'] for x in cases),solver_exit=proc.returncode),indent=2))
 return 0


if __name__=='__main__':raise SystemExit(main())
