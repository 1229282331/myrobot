"""Formal Servo binary with an ideal feedback plant; no device SDK or IO."""
import argparse,json,math,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.tool_frame import get_tool
from e6_phosphobot.jazzy_cpp_worker import verified_binary

def run(args):
    args.output.mkdir(exist_ok=False);tool=get_tool('omnipicker-r4')
    seed=np.array(json.loads(args.seed.read_text()))
    params,profile=load_jazzy_bundle(args.bundle,.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
    config=args.output/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    binary,pin=verified_binary(args.bundle);cases=[];epoch=0;seq=0;last=0.
    with (args.output/'frames.jsonl').open('w') as frames,(args.output/'solver.log').open('w') as log:
        process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        def step(q,v,active,frame,kind=1):
            nonlocal seq,last
            time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
            process.stdin.write(' '.join(map(str,[seq,epoch,int(active),kind,int(frame=='tool'),*q,*v]))+'\n');process.stdin.flush()
            line=process.stdout.readline()
            if not line:raise RuntimeError('Solver closed')
            row=json.loads(line);assert row['seq']==seq
            frames.write(json.dumps(dict(case=label,actual_joints_rad=q.tolist(),velocity=v.tolist(),frame=frame,reply=row))+'\n')
            return row
        def pose(row,field):
            p=row[field];return tool.apply(np.array(p['position_m']),Rotation.from_quat(p['quaternion_xyzw']))
        def start(frame,q):
            nonlocal epoch
            epoch+=1
            # Let the existing asynchronous collision monitor see the new seed.
            for _ in range(6):r=step(q,np.zeros(6),False,frame)
            return r
        try:
            for frame in ('base','tool'):
                for kind in ('translation','rotation'):
                    for axis in range(3):
                        for sign in (-1,1):
                            label=f'{frame}-{kind}-{axis}-{sign}';q=seed.copy();first=start(frame,q)
                            origin,ori=pose(first,'actual_fk');v=np.zeros(6);v[axis+(3 if kind=='rotation' else 0)]=sign*(math.radians(2) if kind=='rotation' else .004)
                            max_drift=0.;max_angle=0.;statuses=[]
                            for _ in range(25):
                                r=step(q,v,True,frame);q=np.array(r['joints_rad']);statuses.append(r['status'])
                                p,rot=pose(r,'target_fk');max_drift=max(max_drift,float(np.linalg.norm(p-origin)));max_angle=max(max_angle,float((rot*ori.inv()).magnitude()))
                            delta=(rot*ori.inv()).as_rotvec() if kind=='rotation' else p-origin
                            direction=np.eye(3)[axis]*sign
                            if frame=='tool':direction=ori.apply(direction)
                            cosine=float(delta@direction/max(np.linalg.norm(delta),1e-12))
                            release=step(q,np.zeros(6),False,frame)
                            assert release['reset'] and np.max(abs(np.array(release['joints_rad'])-q))<1e-12
                            restart=step(q,v,True,frame)
                            invariant=max_drift*1000 if kind=='rotation' else math.degrees(max_angle)
                            cases.append(dict(case=label,statuses=sorted(set(statuses)),direction_cosine=cosine,invariant_error=invariant,
                                magnitude=float(np.linalg.norm(delta)),release_reset=True,restart_status=restart['status'],
                                passed=bool(cosine>.995 and invariant<(.3 if kind=='rotation' else .05) and np.linalg.norm(delta)>(.005 if kind=='rotation' else .001))))
            for axis,sign in [(0,-1),(0,1),(1,-1),(1,1),(2,-1),(2,1)]:
                label=f'continuous-{axis}-{sign}';q=seed.copy();first=start('base',q);origin,ori=pose(first,'actual_fk');v=np.zeros(6);v[axis]=sign*.03
                distance=0.;statuses=[];max_angle=0.
                for _ in range(100):
                    r=step(q,v,True,'base');q=np.array(r['joints_rad']);statuses.append(r['status']);p,rot=pose(r,'target_fk')
                    distance=max(distance,float(np.linalg.norm(p-origin)));max_angle=max(max_angle,math.degrees((rot*ori.inv()).magnitude()))
                    if distance>.065 or r['status'] in (-1,2,5,6):break
                step(q,np.zeros(6),False,'base')
                # Reverse at the boundary with the same control session.
                before=q.copy()
                for _ in range(20):q=np.array(step(q,-v,True,'base')['joints_rad'])
                cases.append(dict(case=label,max_distance_m=distance,statuses=sorted(set(statuses)),orientation_error_deg=max_angle,reverse_joint_delta_rad=float(np.linalg.norm(q-before))))
            label='multi-axis';q=seed.copy();first=start('base',q);origin,_=pose(first,'actual_fk');v=np.array([.003,-.003,.003,0,0,0])
            for _ in range(25):r=step(q,v,True,'base');q=np.array(r['joints_rad'])
            p,_=pose(r,'target_fk');delta=p-origin;cosine=float(delta@v[:3]/(np.linalg.norm(delta)*np.linalg.norm(v[:3])))
            cases.append(dict(case=label,direction_cosine=cosine,passed=cosine>.995))
        finally:
            process.stdin.close()
            try:process.wait(10)
            except subprocess.TimeoutExpired:process.terminate();process.wait(5)
    summary=dict(hardware_io=False,model_id=profile['assembly_id'],binary_sha256=pin['sha256'],cases=cases,
        passed=all(r.get('passed',True) for r in cases) and any(r.get('max_distance_m',0)>.05 for r in cases))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=Path,required=True);run(p.parse_args())
