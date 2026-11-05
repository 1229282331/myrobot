"""Compare an explicitly identified URDF with native FK; never sends motion.

Uses a separately compiled Orocos KDL probe, not a replacement FK/IK solver.
Fresh actual joints select a sample region; hypothetical joints are queries only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
from dobot_sdk.api.robot_control import RobotControl
from .dobot_transport import FramedDobotConnection
from .feedback import FeedbackReader
from .native_probe import reply_vector


def sample_joints(seed, sample_set='local'):
    if sample_set=='independent-global':
        # Fresh held-out seed, deliberately not used to derive model parameters.
        # Hypothetical FK queries only: these are NOT executable trajectories.
        limits=np.array([350,130,149,155,168,350.])
        return [np.array(seed,dtype=float),*np.random.default_rng(710202621).uniform(-limits,limits,(64,6))]
    if sample_set=='frame-definitions':
        values=[np.zeros(6)]
        for axis in range(6):
            for sign in (-1,1):
                q=np.zeros(6);q[axis]=sign*45.;values.append(q)
        values.extend(np.random.default_rng(20261007).uniform(-60,60,(24,6)))
        return values
    values=[np.array(seed,dtype=float)]
    for size in (1., 10.):
        for axis in range(6):
            for sign in (-1,1):
                q=np.array(seed,dtype=float);q[axis]+=size*sign;values.append(q)
    # Fixed deterministic combined offsets: read-only FK, not motion targets.
    values.extend(np.array(seed)+row for row in np.random.default_rng(20261006).uniform(-10,10,(24,6)))
    return values


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='192.168.1.4')
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--sample-set',choices=['local','frame-definitions','independent-global'],default='local')
    p.add_argument('--base',default='base_link')
    p.add_argument('--tip',default='Link6')
    p.add_argument('--jacobian-count',type=int,default=0,help='Central differences on first N query poses, no motion')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    reader=FeedbackReader(a.host);connection=FramedDobotConnection(a.host)
    if not 0<=a.jacobian_count<=65:raise ValueError('Jacobian count outside audit bound')
    result=dict(motion_commands=0,can_sends=0,parameter_writes=0,base=a.base,tip=a.tip,
                urdf_sha256=hashlib.sha256(a.urdf.read_bytes()).hexdigest(),error=None,samples=[],sample_set=a.sample_set)
    result['jacobian_samples']=[]
    try:
        reader.start();deadline=time.monotonic()+3
        while reader.snapshot()['status'] is None and time.monotonic()<deadline:time.sleep(.02)
        before=reader.snapshot();s=before['status']
        if s is None or not before['connected'] or before['age_ms']>100:raise RuntimeError('Fresh feedback required')
        if any(abs(v)>1e-6 for v in s['user_value']+s['tool_value']) or s['tool_coordinate'] or s['user_coordinate']:
            raise RuntimeError('Compare only zero User0 / Tool0, without modifying them')
        result['before']=before
        qs=sample_joints(s['joint_state']['q_actual'],a.sample_set)
        kdl=subprocess.run([str(a.probe),str(a.urdf),a.base,a.tip],input='\n'.join(
            ' '.join(map(str,np.deg2rad(q))) for q in qs)+'\n',text=True,capture_output=True,check=True,timeout=15)
        models=[json.loads(line) for line in kdl.stdout.splitlines()]
        if len(models)!=len(qs):raise RuntimeError('Incomplete KDL output')
        connection.connect();robot=RobotControl(connection)
        for q,model in zip(qs,models):
            start=time.perf_counter_ns();reply=robot.PositiveKin(q,user=0,tool=0);pose=reply_vector(reply)
            error_mm=float(np.linalg.norm(np.array(model['position_m'])*1000-pose[:3]))
            error_deg=float(np.rad2deg((Rotation.from_quat(model['quaternion_xyzw']).inv()*
                Rotation.from_euler('xyz',pose[3:],degrees=True)).magnitude()))
            result['samples'].append(dict(q_deg=q.tolist(),controller_pose_mm_deg=pose.tolist(),model=model,
                position_error_mm=error_mm,orientation_error_deg=error_deg,raw_reply=reply,
                query_ms=(time.perf_counter_ns()-start)/1e6))
        result['max_position_error_mm']=max(r['position_error_mm'] for r in result['samples'])
        result['max_orientation_error_deg']=max(r['orientation_error_deg'] for r in result['samples'])
        result['aligned_under_1mm_1deg']=(result['max_position_error_mm']<=1 and result['max_orientation_error_deg']<=1)
        for index,(q,model) in enumerate(zip(qs,models)):
            if index>=a.jacobian_count:break
            numeric=np.zeros((6,6));queries=[];epsilon=.1
            for axis in range(6):
                poses=[]
                for sign in (-1,1):
                    offset=np.array(q);offset[axis]+=sign*epsilon
                    reply=robot.PositiveKin(offset,user=0,tool=0);pose=reply_vector(reply);poses.append(pose)
                    queries.append(dict(axis=axis,sign=sign,q_deg=offset.tolist(),raw_reply=reply,
                                        controller_pose_mm_deg=pose.tolist()))
                minus,plus=poses
                numeric[:3,axis]=(plus[:3]-minus[:3])/1000/(2*np.deg2rad(epsilon))
                # Spatial angular velocity, expressed in the controller base.
                numeric[3:,axis]=(Rotation.from_euler('xyz',plus[3:],degrees=True)*
                    Rotation.from_euler('xyz',minus[3:],degrees=True).inv()).as_rotvec()/(2*np.deg2rad(epsilon))
            delta=numeric-np.array(model['jacobian'])
            result['jacobian_samples'].append(dict(index=index,q_deg=q.tolist(),epsilon_deg=epsilon,
                controller_finite_difference=numeric.tolist(),queries=queries,
                max_linear_error_m_rad=float(np.max(np.linalg.norm(delta[:3],axis=0))),
                max_angular_error_rad_rad=float(np.max(np.linalg.norm(delta[3:],axis=0)))))
        if result['jacobian_samples']:
            result['max_jacobian_linear_error_m_rad']=max(s['max_linear_error_m_rad'] for s in result['jacobian_samples'])
            result['max_jacobian_angular_error_rad_rad']=max(s['max_angular_error_rad_rad'] for s in result['jacobian_samples'])
        result['after']=reader.snapshot()
    except Exception as exc:
        result['error']=str(exc)
    finally:
        connection.disconnect();reader.close()
        result['commands']=list(connection.events)
        result['reader_events']=list(reader.events)
        (a.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','jacobian_samples','before','after','commands')}))
    return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
