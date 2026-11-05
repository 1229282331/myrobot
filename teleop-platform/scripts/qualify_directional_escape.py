"""Old/new official Servo at recorded posture; virtual plant only, no SDK IO."""
import argparse,json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.tool_frame import get_tool

SEED=np.deg2rad([-26.435213088989258,-60.74069595336914,102.37481689453125,-78.12995910644531,20.279388427734375,30.848236083984375])

def run(args):
 args.output.mkdir(exist_ok=False);tool=get_tool('omnipicker-r2');results=[];healthy={}
 params,_=load_jazzy_bundle(args.bundle,.05,'normal','omnipicker-r2')
 params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
 config=args.output/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
 with (args.output/'frames.jsonl').open('w') as frames,(args.output/'solver.log').open('w') as log:
  for name,binary in [('old',args.old_binary),('directional',args.binary)]:
   proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
   seq=0;epoch=0
   def step(q,v,active,frame='base'):
    nonlocal seq
    seq+=1;proc.stdin.write(' '.join(map(str,[seq,epoch,int(active),1,int(frame=='tool'),*q,*v]))+'\n');proc.stdin.flush()
    line=proc.stdout.readline()
    if not line:raise RuntimeError('Solver exited; see solver.log')
    r=json.loads(line);frames.write(json.dumps(dict(binary=name,frame=frame,velocity=v.tolist(),reply=r))+'\n')
    assert r['seq']==seq
    return r
   def tcp(row,field):
    pose=row[field];return tool.apply(np.array(pose['position_m']),Rotation.from_quat(pose['quaternion_xyzw']))
   try:
    for frame in ('base','tool'):
     for axis in range(3):
      for sign in (-1,1):
       epoch+=1;q=SEED.copy();v=np.zeros(6);v[3+axis]=sign*math.radians(2)
       start=step(q,np.zeros(6),False,frame);origin,orientation=tcp(start,'actual_fk');conditions=[];statuses=[];max_drift=0.;max_joint_step=0
       for _ in range(30):
        r=step(q,v,True,frame);statuses.append(r['status']);nq=np.array(r['joints_rad'])
        max_joint_step=max(max_joint_step,float(np.max(abs(nq-q))));q=nq
        p,rot=tcp(r,'target_fk');max_drift=max(max_drift,float(np.linalg.norm(p-origin)))
        if r.get('singularity'):conditions.append(r['singularity'])
       delta=(rot*orientation.inv()).as_rotvec();direction=np.eye(3)[axis]*sign
       if frame=='tool':direction=orientation.apply(direction)
       release=step(q,np.zeros(6),False,frame)
       assert release['reset'] and np.allclose(release['joints_rad'],q,atol=1e-12,rtol=0)
       result=dict(binary=name,frame=frame,axis=axis,sign=sign,statuses=sorted(set(statuses)),conditions=conditions,
        orientation_delta_deg=np.rad2deg(delta).tolist(),angle_deg=math.degrees(np.linalg.norm(delta)),
        direction_cosine=float(delta@direction/max(np.linalg.norm(delta),1e-12)),tcp_drift_mm=max_drift*1000,max_joint_step_deg=math.degrees(max_joint_step))
       if name=='old':assert max_joint_step==0 and set(statuses)=={2}
       elif result['angle_deg']>1e-6:
        assert result['direction_cosine']>.995 and max_drift<.00025
        assert all(c['condition_target']<c['condition_before'] for c,s in zip(conditions,statuses) if c['condition_before']>=60 and s in (0,1,3,4))
       results.append(result)
    # Away from the singular band, the fork must preserve official output.
    for axis in range(3):
     epoch+=1;q=SEED.copy();q[4]=math.radians(80);step(q,np.zeros(6),False)
     v=np.zeros(6);v[3+axis]=math.radians(2);trace=[]
     for _ in range(20):
      r=step(q,v,True);q=np.array(r['joints_rad']);trace.append((r['status'],q.copy()))
     if name=='old':healthy[axis]=trace
     else:
      for (status,q),(old_status,old_q) in zip(trace,healthy[axis]):
       assert status==old_status and np.allclose(q,old_q,atol=1e-12,rtol=0)
    if name=='directional':
     # Reverse a moving escape without release: smoothing may not carry it
     # deeper while the condition is above the ordinary hard threshold.
     for case in [r for r in results if r['binary']==name and r.get('angle_deg',0)>1e-6]:
      epoch+=1;q=SEED.copy();v=np.zeros(6);v[3+case['axis']]=case['sign']*math.radians(2)
      step(q,np.zeros(6),False,case['frame'])
      for _ in range(2):q=np.array(step(q,v,True,case['frame'])['joints_rad'])
      before=q.copy();r=step(q,-v,True,case['frame']);c=r['singularity']
      assert r['status']==2 and np.allclose(r['joints_rad'],before,atol=1e-12,rtol=0)
    if name=='directional':
     # A truly rank-deficient/deeply singular posture remains blocked.
     for j5 in (0.,5.):
      q=SEED.copy();q[4]=math.radians(j5);epoch+=1;step(q,np.zeros(6),False)
      v=np.zeros(6);v[3]=math.radians(2);r=step(q,v,True)
      assert r['status'] in (-1,2) and np.allclose(r['joints_rad'],q,atol=1e-12,rtol=0)
      results.append(dict(binary=name,case='deep_singularity',j5=j5,status=r['status']))
   finally:
    proc.stdin.close();proc.wait(10);assert proc.returncode==0
 summary=dict(hardware_io=False,seed_deg=np.rad2deg(SEED).tolist(),cases=results,passed=True,
  moving_directions=[{k:r[k] for k in ('frame','axis','sign','angle_deg','tcp_drift_mm')} for r in results if r.get('binary')=='directional' and r.get('angle_deg',0)>1e-6])
 assert len(summary['moving_directions'])>=2
 (args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='cases'}))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--binary',type=Path,required=True)
 p.add_argument('--old-binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 run(p.parse_args())
