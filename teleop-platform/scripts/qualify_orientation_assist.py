"""20 Hz isolated official Servo tests with delayed synthetic joint feedback.

No robot, CAN, camera or network control client. Baseline and assisted policy
use the same model, original safety thresholds and final swept FCL checks.
"""
from pathlib import Path
import argparse,collections,hashlib,json,math,select,subprocess,time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.tool_frame import get_tool

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
 ap.add_argument('--smoke',action='store_true')
 ap.add_argument('--baseline',action='store_true')
 ap.add_argument('--reversal',action='store_true')
 ap.add_argument('--tag');ap.add_argument('--steps',type=int)
 ap.add_argument('--speed',type=float,default=.006)
 ap.add_argument('--vary-speed',action='store_true')
 ap.add_argument('--wrist-near',action='store_true')
 ap.add_argument('--tool-only',action='store_true')
 a=ap.parse_args();root=a.root
 if not 0<a.speed<=.03:raise ValueError('Offline speed must respect the normal profile')
 out=root/(a.tag or ('assist-smoke' if a.smoke else 'assist-regression'));out.mkdir(exist_ok=False)
 params,profile=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 state=json.loads((root/'current-state.json').read_text());seed=np.array(state['actual']['joints_rad'])
 raw=round(state['actual']['gripper']*255);counts=[raw,-1,-1,-1]
 tool=get_tool('omnipicker-r4');frames=[];cases=[];seq=epoch=0;last=0.;ramp=IntentRamp(.2)
 binary=root/('baseline/servo_cpp_adapter' if a.baseline else 'build/servo_cpp_adapter')
 with (out/'servo.log').open('w') as log:
  p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  def step(q,v,active,assisted,frame,case,rate=.03490658503988659):
   nonlocal seq,last
   time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
   intent=dict(linear_m_s=list(v),angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,frame='tool' if frame else 'base',orientation_assist=assisted)
   if active:shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+[0.]*3
   else:ramp.reset();effective=[0.]*6
   extra=['A1',rate] if assisted and active else []
   values=[seq,epoch,int(active),2 if assisted and active else 1,int(frame),*q,*effective,*extra,'G3',*counts,1]
   p.stdin.write(' '.join(map(str,values))+'\n');p.stdin.flush()
   if not select.select([p.stdout],[],[],20)[0]:raise TimeoutError('Isolated solver response')
   line=p.stdout.readline()
   if not line:raise RuntimeError('Solver exited: '+str(p.poll()))
   result=json.loads(line);assert result['seq']==seq
   fk=result['target_fk'];pos,rot=tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
   row=dict(case=case,q=list(q),linear=list(v),effective=effective,frame=frame,assisted=assisted,rate=rate,
            tcp=pos.tolist(),orientation=rot.as_quat().tolist(),reply=result)
   frames.append(row);return row
  def start(q,case):
   nonlocal epoch
   epoch+=1
   for _ in range(6):row=step(q,[0.]*3,False,False,False,case)
   return row
  try:
   poses=[('wrist-near' if a.wrist_near else 'current',seed+np.array([0,0,0,0,-.8,0]) if a.wrist_near else seed)]
   if not a.smoke:
    poses += [('wrist-near',seed+np.array([0,0,0,0,-.8,0])),
              ('elbow-near',seed+np.array([0,.15,-.35,0,0,0]))]
   for pose_name,initial in poses:
    for frame in ((True,) if a.tool_only else (False,) if a.smoke else (False,True)):
     for direction in ([0,1,0],[0,-1,0]) if a.smoke else ([1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1],[1,1,1]):
      for assisted in ((False,) if a.baseline else (False,True)):
       label=f'{pose_name}-{int(frame)}-{direction}-{int(assisted)}';q=initial.copy();first=start(q,label)
       p0=np.array(first['tcp']);r0=Rotation.from_quat(first['orientation'])
       unit=np.array(direction,dtype=float);unit/=np.linalg.norm(unit);expected=r0.apply(unit) if frame else unit
       plant=collections.deque([q.copy(),q.copy()]);active=[]
       steps=a.steps or (50 if a.smoke else 80)
       for i in range(steps):
        sign=-1 if a.reversal and (i//40)%2 else 1
        scale=.25 if a.vary_speed and (i//60)%2 else 1.
        rate=np.deg2rad(3)*scale if a.vary_speed else .03490658503988659
        row=step(q,unit*a.speed*sign*scale,True,assisted,frame,label,rate);active.append(row)
        plant.append(np.array(row['reply']['joints_rad']));q=plant.popleft()
       # Native status alone is not an execution permit: the real Python host
       # also rejects a reply crossing the callback deadline during encoding.
       accepted=[x for x in active if x['reply']['status'] in (0,1,3,4) and
                 x['reply']['callback_us']<42000 and x['reply']['gripper_check']['arm_safe']]
       max_transverse=0.;max_angle=0.
       for row in accepted:
        delta=np.array(row['tcp'])-p0;max_transverse=max(max_transverse,np.linalg.norm(delta-expected*np.dot(delta,expected)))
        angle=math.degrees((r0.inv()*Rotation.from_quat(row['orientation'])).magnitude());max_angle=max(max_angle,angle)
        if assisted:
         info=row['reply']['orientation_assist'];assert info['enabled'] and info['version']==1
         assert np.linalg.norm(info['twist_base'][3:])<=row['rate']+1e-9
         assert np.allclose(info['twist_base'][:3],r0.apply(row['effective'][:3]) if frame else row['effective'][:3],atol=1e-10)
         assert angle<=15+1e-6 and abs(angle-info['deviation_deg'])<1e-6
        else:assert angle<.12
        assert row['reply']['gripper_check']['arm_safe'] and row['reply']['callback_us']<42000
       delta=np.array(active[-1]['tcp'])-p0
       case=dict(case=label,assisted=assisted,frame=frame,pose=pose_name,requested_mm=steps*.05*a.speed*1000,reversal=a.reversal,
         achieved_mm=float(delta@expected*1000),cross_track_mm=float(max_transverse*1000),rotation_deg=max_angle,
         blocked=sum(x['reply']['status'] in (-1,2,5,6) for x in active),
         late_replies=sum(x['reply']['callback_us']>=42000 for x in active),
         statuses=dict(collections.Counter(str(x['reply']['status']) for x in active)),
         callback_ms=dict(zip(('p50','p95','max'),np.percentile([x['reply']['callback_us']/1000 for x in active],[50,95,100]))))
       # Safety checks hold even where the original model blocks a direction.
       assert max_transverse<.0005,(label,'cross-track',case)
       release=step(q,[0.]*3,False,assisted,frame,label)
       assert release['reply']['reset'] and np.allclose(release['reply']['joints_rad'],q,atol=1e-12)
       # Release preserves actual orientation; it cannot return to the anchor.
       again=step(q,[0.]*3,False,assisted,frame,label)
       assert np.allclose(again['tcp'],release['tcp'],atol=1e-12)
       cases.append(case);print(json.dumps(case),flush=True)
   summary=dict(passed=True,hardware_io=False,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
     assembly_id=profile['assembly_id'],cases=len(cases),frames=len(frames),
     active_frames=sum(x['reply']['active'] for x in frames),
     budget_stops=sum(x['reply']['active'] and x['reply']['gripper_check']['budget_exhausted'] for x in frames))
   (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
  finally:
   p.stdin.close()
   try:p.wait(10)
   except subprocess.TimeoutExpired:p.terminate();p.wait(5)
   (out/'frames.json').write_text(json.dumps(frames));(out/'cases.json').write_text(json.dumps(cases,indent=2))

if __name__=='__main__':main()
