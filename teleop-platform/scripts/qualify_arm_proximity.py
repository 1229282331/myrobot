"""Isolated official Servo + FCL regression. No actuator SDK or device IO."""
from pathlib import Path
import argparse,json,math,subprocess,select,time,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.tool_frame import get_tool

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root
 out=root/'qualification';out.mkdir(exist_ok=False);seed=np.array(json.loads((root/'state.json').read_text())['actual']['joints_rad'])
 new=root/'build/servo_cpp_adapter';old=Path('/qualification/dynamic-gripper-v267/build/servo_cpp_adapter')
 tool=get_tool('omnipicker-r4');results=[];all_rows=[]
 for version,binary in [('baseline',old),('fixed',new)]:
  seq=epoch=0;last=0.;ramp=IntentRamp(.2);label='startup'
  with (out/(version+'-solver.log')).open('w') as log:
   proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(root/'params.yaml')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
   def step(q,counts,active=False,v=None,frame=False,kind=1):
    nonlocal seq,last
    time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
    v=np.zeros(6) if v is None else np.array(v)
    intent=dict(linear_m_s=list(v[:3]) if kind else [0]*3,angular_rad_s=list(v[3:]) if kind else [0]*3,joint_rad_s=[0]*6 if kind else list(v),frame='tool' if frame else 'base')
    if active:
     shaped,trace=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s'] if kind else shaped['joint_rad_s']
    else:ramp.reset();effective=[0]*6
    values=[seq,epoch,int(active),kind,int(frame),*q,*effective,'G2',*counts]
    proc.stdin.write(' '.join(map(str,values))+'\n');proc.stdin.flush()
    if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('Isolated solver timeout')
    text=proc.stdout.readline()
    if not text:raise RuntimeError('Isolated solver exited '+str(proc.poll()))
    r=json.loads(text);assert r['seq']==seq
    all_rows.append(dict(version=version,case=label,q=list(q),counts=counts,input=list(v),effective=effective,reply=r));return r
   def start(q,raw=153):
    nonlocal epoch
    epoch+=1
    for _ in range(5):r=step(q,[raw,raw,-1,raw])
    return r
   def pose(r):
    fk=r['target_fk'];return tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
   try:
    for raw in ([153] if version=='baseline' else [39,89,153]):
     label=f'steady-24mm-raw{raw}';q=seed.copy();start(q,raw);positions=[];statuses=[]
     for i in range(45):
      r=step(q,[raw,raw,-1,raw],True,[0,-.024,0,0,0,0]);q=np.array(r['joints_rad']);positions.append(pose(r)[0]);statuses.append(r['status'])
     speed=float(np.linalg.norm(positions[-1]-positions[25])/(19*.05)*1000)
     passed=(1<speed<2) if version=='baseline' else (23<speed<24.2 and set(statuses)=={0})
     results.append(dict(version=version,case=label,steady_mm_s=speed,statuses=sorted(set(statuses)),passed=passed));assert passed,results[-1]
    if version=='baseline':continue
    # Translation, fixed-TCP rotation, both coordinate frames, all signs.
    for frame in (False,True):
     for axis in range(6):
      for sign in (-1,1):
       label=f'{"tool" if frame else "base"}-{axis}-{sign}';q=seed.copy();first=start(q);origin,orientation=pose(first)
       v=np.zeros(6);v[axis]=sign*(.004 if axis<3 else math.radians(2));statuses=[]
       for i in range(25):
        raw=152+i%2;r=step(q,[raw,153,-1,153],True,v,frame);q=np.array(r['joints_rad']);statuses.append(r['status'])
        assert r['gripper_check']['arm_safe'] and r['gripper_check']['arm_range_raw'][0]<=152 and r['gripper_check']['arm_range_raw'][1]>=153
       pos,rot=pose(r);delta=pos-origin if axis<3 else (rot*orientation.inv()).as_rotvec();expected=np.eye(3)[axis%3]*sign
       if frame:expected=orientation.apply(expected)
       cosine=float(delta@expected/max(np.linalg.norm(delta),1e-12));invariant=float(math.degrees((rot*orientation.inv()).magnitude())) if axis<3 else float(np.linalg.norm(pos-origin)*1000)
       released=step(q,[153,153,-1,153]);assert released['reset'] and np.max(abs(np.array(released['joints_rad'])-q))<1e-12
       restarted=step(q,[153,153,-1,153],True,v,frame);assert restarted['reset']
       passed=cosine>.995 and invariant<(.05 if axis<3 else .3) and set(statuses)=={0}
       results.append(dict(case=label,passed=passed,cosine=cosine,invariant=invariant,statuses=sorted(set(statuses))));assert passed,results[-1]
    for axis in range(6):
     for sign in (-1,1):
      label=f'joint-{axis}-{sign}';q=seed.copy();start(q);v=np.zeros(6);v[axis]=sign*math.radians(2)
      for _ in range(20):r=step(q,[153,153,-1,153],True,v,kind=0);q=np.array(r['joints_rad'])
      delta=q-seed;assert delta[axis]*sign>0 and np.max(abs(np.delete(delta,axis)))<1e-10
      results.append(dict(case=label,passed=True))
    label='multi-axis-reverse';q=seed.copy();first=start(q);origin,_=pose(first);v=np.array([.003,-.003,.003,0,0,0])
    for _ in range(25):r=step(q,[153,153,-1,153],True,v);q=np.array(r['joints_rad'])
    pos,_=pose(r);delta=pos-origin;cosine=float(delta@v[:3]/(np.linalg.norm(delta)*np.linalg.norm(v[:3])));assert cosine>.995
    for _ in range(40):r=step(q,[153,153,-1,153],True,-v);q=np.array(r['joints_rad'])
    results.append(dict(case=label,passed=True,cosine=cosine))
    # Keep the original opening-boundary retreat and stale feedback fallback.
    label='opening-boundary';q=np.array(json.loads((root/'fcl-verification.json').read_text())['aperture_dependent_cases'][0]['q']);raw=38;start(q,raw);blocked=None
    for _ in range(32):
     proposal=min(153,raw+4);r=step(q,[raw,raw,-1,proposal]);check=r['gripper_check']
     if not check['allowed']:blocked=proposal;break
     raw=proposal
    assert blocked is not None and check['arm_safe'] and not check['budget_exhausted']
    retreat=step(q,[raw,raw,-1,raw-4]);assert retreat['gripper_check']['allowed']
    unknown=step(q,[-1,raw,-1,raw]);assert unknown['gripper_check']['fallback'] and not unknown['gripper_check']['arm_safe']
    results.append(dict(case=label,passed=True,blocked=blocked,last_valid=raw))
    # All actual external and INTERNAL collision queries remain hard stops.
    fixtures=json.loads((root/'geometry-cases.json').read_text());external=next(c for c in fixtures if c['case']=='whole-range' and c['raw']>=0 and c['result']['collision'])
    label='actual-collision';q=np.array(external['q']);raw=external['raw'];start(q,raw)
    r=step(q,[raw,raw,-1,raw],True,[0,.004,0,0,0,0]);assert r['status']==5 and np.max(abs(np.array(r['joints_rad'])-q))<1e-12
    results.append(dict(case=label,passed=True,status=r['status']))
    nearby=next(c for c in fixtures if c['case']=='whole-range' and c['raw']>=0 and not c['result']['collision'] and 0<c['result']['distance']<.009)
    label='external-proximity';q=np.array(nearby['q']);raw=nearby['raw'];start(q,raw)
    r=step(q,[raw,raw,-1,raw],True,[0,.004,0,0,0,0])
    retreat=r.get('collision_retreat',{})
    if r['status'] in (0,1,3):
     # Directional retreat may now leave a near state at requested speed;
     # plain status 0 without a complete current-path proof is still a failure.
     assert retreat.get('boosted') and retreat.get('verified') and not retreat.get('budget_exhausted')
     assert retreat['minimum_after_m']>retreat['minimum_before_m']>0
     assert r['gripper_check']['arm_safe'] and not r['gripper_check']['budget_exhausted'] and r['callback_us']<42000
    else:assert r['status'] in (2,4,5,6)
    results.append(dict(case=label,passed=True,status=r['status'],distance=nearby['result']['distance'],retreat=retreat))
    label='abnormal-feedback-gap';q=seed.copy();start(q,89)
    step(q,[89,89,-1,89],True,[0,.004,0,0,0,0])
    q[0]+=math.radians(20)
    r=step(q,[89,89,-1,89],True,[0,.004,0,0,0,0])
    assert r['status']==5 and not r['gripper_check']['arm_safe'] and np.max(abs(np.array(r['joints_rad'])-q))<1e-12
    results.append(dict(case=label,passed=True,status=r['status']))
   finally:
    proc.stdin.close()
    try:proc.wait(10)
    except subprocess.TimeoutExpired:proc.terminate();proc.wait(5)
    (out/'frames.json').write_text(json.dumps(all_rows))
    (out/'cases.json').write_text(json.dumps(results,indent=2))
 callbacks=[x['reply']['callback_us']/1000 for x in all_rows if x['version']=='fixed']
 summary=dict(passed=all(r['passed'] for r in results),binary_sha256=hashlib.sha256(new.read_bytes()).hexdigest(),hardware_io=False,frames=len(all_rows),cases=results,callback_ms=dict(zip(('p50','p95','p99','max'),np.percentile(callbacks,[50,95,99,100]))))
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
