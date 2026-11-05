"""Official C++ Servo FK checks with a virtual plant; no actuator sockets.

Run in the existing Jazzy root, using the same model/config/binary as the
formal adapter. All candidate/actual poses from C++ remain flange FK, so the
TCP invariant is checked independently from the configured solver tip.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.tool_frame import get_tool


def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    tool=get_tool('omnipicker-r2')
    results=[]
    seed=np.deg2rad([-75.66455,-56.63528,83.49904,-59.55208,98.90785,-3.1572])
    with (args.output/'solver.log').open('w') as log, (args.output/'frames.jsonl').open('w') as frames:
        for profile in ('flange','omnipicker-r2'):
            params,_=load_jazzy_bundle(args.bundle,.05,'normal',profile)
            params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2)
            params['update_period']=.05
            config=args.output/(profile+'.yaml')
            config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
            proc=subprocess.Popen([str(args.binary),'--ros-args','--params-file',str(config)],
                                  stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
            sequence=0;epoch=0
            def step(q,vel,active,frame):
                nonlocal sequence
                sequence+=1
                proc.stdin.write(' '.join(map(str,[sequence,epoch,int(active),1,int(frame=='tool'),*q,*vel]))+'\n')
                proc.stdin.flush()
                line=proc.stdout.readline()
                if not line:raise RuntimeError('Solver closed; inspect solver.log')
                row=json.loads(line)
                frames.write(json.dumps(dict(profile=profile,frame=frame,request_velocity=vel.tolist(),reply=row))+'\n')
                assert row['seq']==sequence and row['epoch']==epoch
                assert row['status'] in (0,1,3,4),row['status']
                return row
            try:
                cases=[('base','rotation',1,1)] if profile=='flange' else [
                    (frame,kind,axis,sign) for frame in ('base','tool')
                    for kind in ('rotation','translation') for axis in range(3) for sign in (-1,1)]
                for frame,kind,axis,sign in cases:
                    epoch+=1;q=seed.copy();zero=np.zeros(6)
                    first=step(q,zero,False,frame)
                    fp=np.array(first['actual_fk']['position_m']);fr=Rotation.from_quat(first['actual_fk']['quaternion_xyzw'])
                    origin,ori=tool.apply(fp,fr);maximum=0.;max_angle=0.;flange_shift=0.
                    velocity=zero.copy();velocity[axis+(3 if kind=='rotation' else 0)]=sign*(math.radians(2) if kind=='rotation' else .004)
                    for index in range(30):
                        reply=step(q,velocity,True,frame)
                        q=np.array(reply['joints_rad'])
                        p=np.array(reply['target_fk']['position_m']);r=Rotation.from_quat(reply['target_fk']['quaternion_xyzw'])
                        tcp,rot=tool.apply(p,r)
                        maximum=max(maximum,float(np.linalg.norm(tcp-origin)))
                        max_angle=max(max_angle,float((rot*ori.inv()).magnitude()))
                        flange_shift=max(flange_shift,float(np.linalg.norm(p-fp)))
                    moved=(rot*ori.inv()).as_rotvec() if kind=='rotation' else tcp-origin
                    direction=np.eye(3)[axis]*sign
                    if frame=='tool':direction=ori.apply(direction)
                    cosine=float(moved@direction/max(np.linalg.norm(moved),1e-12))
                    invariant=maximum*1000 if kind=='rotation' else math.degrees(max_angle)
                    result=dict(profile=profile,frame=frame,kind=kind,axis=axis,sign=sign,
                        tcp_drift_mm=maximum*1000,orientation_change_deg=math.degrees(max_angle),
                        flange_shift_mm=flange_shift*1000,direction_cosine=cosine,
                        invariant_error=invariant,actual_magnitude=float(np.linalg.norm(moved)))
                    if profile!='flange':
                        result['passed']=bool(invariant<(.25 if kind=='rotation' else .05) and cosine>.995 and np.linalg.norm(moved)>(.005 if kind=='rotation' else .001))
                    else:result['demonstrates_old_pivot']=maximum>.003
                    # Inactive reset must discard every old target.
                    release=step(q,zero,False,frame)
                    assert release['reset'] and np.max(np.abs(np.array(release['joints_rad'])-q))<1e-12
                    results.append(result)
            finally:
                proc.stdin.close()
                try:proc.wait(5)
                except subprocess.TimeoutExpired:proc.terminate();proc.wait(3)
                assert proc.returncode==0
    summary=dict(hardware_io=False,tool=tool.description(),cases=results,
        passed=bool(results and results[0].get('demonstrates_old_pivot') and all(r.get('passed',True) for r in results)))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary))
    return 0 if summary['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    raise SystemExit(run(parser.parse_args()))
