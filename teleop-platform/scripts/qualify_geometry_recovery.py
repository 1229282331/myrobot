"""Offline 20 Hz replay of the v299 stuck pose with formal Servo and FCL.

Runs inside the existing isolated namespace. No device clients or CAN IO.
All rejected/expired targets are excluded from the delayed synthetic plant.
"""
from pathlib import Path
import argparse,collections,hashlib,json,select,subprocess,time
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.intent_ramp import IntentRamp

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
 ap.add_argument('--baseline',action='store_true');ap.add_argument('--smoke',action='store_true')
 ap.add_argument('--tag');a=ap.parse_args();root=a.root
 out=root/(a.tag or (('baseline' if a.baseline else 'candidate')+('-smoke' if a.smoke else '-regression')));out.mkdir(exist_ok=False)
 params,profile=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 seed=np.array(json.loads((root/'current-state.json').read_text())['actual']['joints_rad'])
 binary=root/('baseline/servo_cpp_adapter' if a.baseline else 'build/servo_cpp_adapter')
 frames=[];cases=[];seq=epoch=0;last=0.;ramp=IntentRamp(.2)
 with (out/'servo.log').open('w') as log:
  p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  def step(q,v,active,kind,tool,counts,hold,label):
   nonlocal seq,last
   time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
   intent=dict(linear_m_s=list(v[:3]) if kind else [0.]*3,angular_rad_s=list(v[3:]) if kind else [0.]*3,
               joint_rad_s=list(v) if not kind else [0.]*6,frame='tool' if tool else 'base',orientation_assist=kind==2)
   if active:
    shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s'] if kind else shaped['joint_rad_s']
   else:ramp.reset();effective=[0.]*6
   values=[seq,epoch,int(active),kind,int(tool),*q,*effective]
   if kind==2:values+=['A1',.010471975511965976]
   values+=['G3',*counts,hold]
   p.stdin.write(' '.join(map(str,values))+'\n');p.stdin.flush()
   if not select.select([p.stdout],[],[],20)[0]:raise TimeoutError('Isolated solver response')
   line=p.stdout.readline()
   if not line:raise RuntimeError('Solver exited: '+str(p.poll()))
   result=json.loads(line);assert result['seq']==seq
   accepted=result['status'] in (0,1,3,4) and result['callback_us']<42000 and result['gripper_check']['arm_safe']
   row=dict(case=label,q=list(q),requested=list(v),effective=effective,kind=kind,tool=tool,counts=counts,hold=hold,accepted=accepted,reply=result)
   frames.append(row);return row
  def run(label,kind,tool,axis,sign,raw=80,hold=1,steps=60,follow=True):
   nonlocal epoch
   epoch+=1;q=seed.copy();counts=[raw,raw,-1,raw];replies=[]
   for _ in range(6):initial=step(q,np.zeros(6),False,kind,tool,counts,hold,label)
   plant=collections.deque([q.copy(),q.copy()])
   for i in range(steps):
    v=np.zeros(6);v[axis]=sign*(.006 if kind and axis<3 else .02)*(-1 if i>=steps//2 else 1)
    counts=[max(0,raw-(i%2)),raw,raw if i%4==0 else -1,raw]
    row=step(q,v,True,kind,tool,counts,hold,label);replies.append(row)
    if follow:
     plant.append(np.array(row['reply']['joints_rad']) if row['accepted'] else q.copy());q=plant.popleft()
   released=step(q,np.zeros(6),False,kind,tool,counts,hold,label)
   assert released['reply']['reset'] and np.allclose(released['reply']['joints_rad'],q,atol=1e-12)
   # A new command after release is a new epoch with a fresh smoothing seed.
   epoch+=1;v=np.zeros(6);v[axis]=sign*(.003 if kind and axis<3 else .01)
   restarted=step(q,v,True,kind,tool,counts,hold,label)
   assert restarted['reply']['reset'] and np.allclose(restarted['reply']['seed_positions'],q,atol=1e-12)
   step(q,np.zeros(6),False,kind,tool,counts,hold,label)
   timeouts=sum(x['reply']['gripper_check'].get('arm_reason') in ('sweep_budget','callback_deadline') or x['reply']['callback_us']>=42000 for x in replies)
   case=dict(case=label,active_frames=len(replies),accepted=sum(x['accepted'] for x in replies),timeouts=timeouts,
    initial_geometry=initial['reply']['gripper_check'],
    statuses=dict(collections.Counter(str(x['reply']['status']) for x in replies)),
    retreat_reasons=dict(collections.Counter(x['reply']['collision_retreat']['reason'] for x in replies)),
    callback_ms=dict(zip(('p50','p95','max'),np.percentile([x['reply']['callback_us']/1000 for x in replies],[50,95,100]))),
    finger_ms=dict(zip(('p50','p95','max'),np.percentile([x['reply']['gripper_check']['elapsed_us']/1000 for x in replies],[50,95,100]))))
   cases.append(case);print(json.dumps(case),flush=True)
  try:
   if a.smoke:
    for axis in (0,3):run('stuck-fixed-'+str(axis),1,False,axis,1,steps=40,follow=False)
   else:
    for tool in (False,True):
     for axis in range(6):
      for sign in (-1,1):run(f'axis-{int(tool)}-{axis}-{sign}',1,tool,axis,sign)
    for tool in (False,True):
     for axis in range(3):run(f'assist-{int(tool)}-{axis}',2,tool,axis,1)
    for axis in range(6):run('joint-'+str(axis),0,False,axis,1,steps=40)
    for raw in (12,128,253):run('aperture-'+str(raw),1,False,0,1,raw=raw,steps=30,follow=False)
    for hold in (0,2):run('hold-'+str(hold),1,False,0,1,hold=hold,steps=30,follow=False)
  finally:
   p.stdin.close()
   try:p.wait(10)
   except subprocess.TimeoutExpired:p.terminate();p.wait(5)
   (out/'frames.json').write_text(json.dumps(frames));(out/'cases.json').write_text(json.dumps(cases,indent=2))
 summary=dict(completed=True,hardware_io=False,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),assembly_id=profile['assembly_id'],
  cases=len(cases),frames=len(frames),active_frames=sum(c['active_frames'] for c in cases),timeouts=sum(c['timeouts'] for c in cases),
  accepted=sum(c['accepted'] for c in cases),unresponsive_cases=[c['case'] for c in cases if c['accepted']==0])
 # Alternate aperture fixtures can already collide at this arm pose. A
 # correctly rejected colliding starting model must not be made executable.
 summary['initial_collision_cases']=[c['case'] for c in cases if c['case'].startswith('aperture-') and
  not c['initial_geometry']['arm_safe'] and c['initial_geometry'].get('arm_reason')=='collision']
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
 if not a.baseline:assert summary['timeouts']==0 and set(summary['unresponsive_cases'])<=set(summary['initial_collision_cases']),summary

if __name__=='__main__':main()
