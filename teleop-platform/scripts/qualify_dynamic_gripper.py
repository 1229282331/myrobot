"""Offline Servo/FCL regression; synthetic state only, no actuator imports."""
from pathlib import Path
import argparse,hashlib,json,math,select,subprocess,time,yaml
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.tool_frame import get_tool

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--binary',type=Path,required=True);args=p.parse_args()
 root=args.root;out=root/'qualification';out.mkdir(exist_ok=False)
 params,profile=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 config=out/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 seed=np.array(json.loads((root/'states.json').read_text())[-1]['actual']['joints_rad']);tool=get_tool('omnipicker-r4')
 seq=0;epoch=0;rows=[];cases=[];last=0.
 with (out/'solver.log').open('w') as log:
  process=subprocess.Popen([str(args.binary),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  def step(q,counts,active=False,v=None,tool_frame=False):
   nonlocal seq,last
   time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
   vals=[seq,epoch,int(active),1,int(tool_frame),*q,*(v if v is not None else [0]*6),'G2',*counts]
   process.stdin.write(' '.join(map(str,vals))+'\n');process.stdin.flush()
   if not select.select([process.stdout],[],[],15)[0]:raise TimeoutError('Isolated solver timeout')
   row=json.loads(process.stdout.readline());assert row['seq']==seq
   rows.append(dict(case=label,actual_q=list(q),counts=counts,reply=row));return row
  def tcp(row):
   fk=row['target_fk'];return tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
  try:
   label='all-apertures'
   for raw in range(256):
    r=step(seed,[raw,raw,-1,raw]);assert r['gripper_check']['allowed'] and r['gripper_check']['arm_safe']
   cases.append(dict(case=label,passed=True,states=256))
   for frame in [False,True]:
    for axis in range(6):
     for sign in [-1,1]:
      label=f'{"tool" if frame else "base"}-{axis}-{sign}';epoch+=1;q=seed.copy()
      for _ in range(3):first=step(q,[89,89,-1,89])
      origin,ori=tcp(first);v=np.zeros(6);v[axis]=sign*(.004 if axis<3 else math.radians(2))
      for _ in range(20):
       r=step(q,[89,89,-1,89],True,v,frame);q=np.array(r['joints_rad'])
       assert r['gripper_check']['arm_safe'] and r['callback_us']<50_000
      pos,rot=tcp(r);delta=pos-origin if axis<3 else (rot*ori.inv()).as_rotvec()
      expected=np.eye(3)[axis%3]*sign
      if frame:expected=ori.apply(expected)
      cosine=float(delta@expected/max(np.linalg.norm(delta),1e-12))
      invariant=float(math.degrees((rot*ori.inv()).magnitude())) if axis<3 else float(np.linalg.norm(pos-origin)*1000)
      released=step(q,[89,89,-1,89]);assert released['reset'] and np.max(abs(np.array(released['joints_rad'])-q))<1e-12
      restarted=step(q,[89,89,-1,89],True,v,frame)
      cases.append(dict(case=label,passed=cosine>.995 and invariant<(.05 if axis<3 else .3),cosine=cosine,invariant=invariant,restart_status=restarted['status']))
   label='opening-boundary-retreat';epoch+=1
   q=np.array(json.loads((root/'fcl-verification.json').read_text())['aperture_dependent_cases'][0]['q']);raw=38;blocked=None
   for _ in range(32):
    proposal=min(153,raw+4);r=step(q,[raw,raw,-1,proposal]);check=r['gripper_check']
    if not check['allowed']:blocked=proposal;break
    raw=proposal
   assert blocked is not None and check['arm_safe'] and not check['budget_exhausted']
   r=step(q,[raw,raw,-1,raw-4]);assert r['gripper_check']['allowed']
   cases.append(dict(case=label,passed=True,last_valid=raw,blocked=blocked,retreat=raw-4))
   label='unknown-fallback';r=step(q,[-1,raw,-1,raw]);assert r['gripper_check']['fallback'] and not r['gripper_check']['arm_safe']
   cases.append(dict(case=label,passed=True))
  finally:
   process.stdin.close()
   try:process.wait(10)
   except subprocess.TimeoutExpired:process.terminate();process.wait(5)
   (out/'frames.json').write_text(json.dumps(rows))
 summary=dict(model_id=profile['assembly_id'],binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),hardware_io=False,cases=cases,passed=all(c['passed'] for c in cases))
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));assert summary['passed']

if __name__=='__main__':main()
