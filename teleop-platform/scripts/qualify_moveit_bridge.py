"""Exercise the production bridge with virtual feedback; no hardware imports/IO.

This is adapter qualification, explicitly not webpage or real-motion acceptance.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge


class VirtualReader:
    def __init__(self,bundle,seed):
        data=subprocess.run([str(bundle/'kdl_model_probe'),str(bundle/'model/e6.urdf'),
            'controller_base','controller_flange'],input=' '.join(map(str,np.deg2rad(seed)))+'\n',
            capture_output=True,text=True,check=True,timeout=5)
        self.set(dict(joints_rad=np.deg2rad(seed).tolist(),**json.loads(data.stdout)))

    def set(self,row):
        pose=[*(np.array(row['position_m'])*1000),*Rotation.from_quat(row['quaternion_xyzw']).as_euler('xyz',degrees=True)]
        self.s=dict(joint_state=dict(q_actual=np.rad2deg(row['joints_rad']).tolist()),
                    tool_vector_actual=dict(zip(('x','y','z','rx','ry','rz'),pose)))

    def snapshot(self):
        return dict(connected=True,age_ms=0,received_ns=time.perf_counter_ns(),status=self.s)


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--seed',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    reader=VirtualReader(a.bundle,json.loads(a.seed.read_text()));b=MoveItBridge(a.bundle,reader)
    result=dict(hardware_connected=False,motion_commands=0,can_sends=0,error=None,cases=[]);seq=0
    try:
        b.start();deadline=time.monotonic()+40
        while not b.snapshot()['ready']:
            if b.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(b.snapshot()))
            time.sleep(.05)
        maps=Path('/proc')/str(b.process.pid)/'maps'
        (a.output/'loaded-libraries.txt').write_text(maps.read_text())
        with (a.output/'candidates.jsonl').open('w') as log:
            for kind,frame in [('joint','base'),('cartesian','base'),('cartesian','tool')]:
                for axis in range(6):
                    for sign in (-1,1):
                        i=dict(linear_m_s=[0.]*3,angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,
                            gripper_rate_s=0,deadman=True,frame=frame)
                        key='joint_rad_s' if kind=='joint' else 'linear_m_s' if axis<3 else 'angular_rad_s'
                        i[key][axis if kind=='joint' else axis%3]=sign*(.001 if key=='linear_m_s' else math.radians(.5))
                        command=Intent(**i);count=0;statuses=set();sources=set();before=reader.snapshot()
                        start=time.monotonic();next_send=start
                        while time.monotonic()-start<.8:
                            if time.monotonic()>=next_send:
                                seq+=1;b.submit(command,seq);sources.add(seq);next_send=time.monotonic()+.05
                            candidate=b.take()
                            if candidate:
                                if candidate['input_sequence'] not in sources:raise RuntimeError('Cross-case input provenance')
                                reader.set(candidate);count+=1;statuses.add(candidate['status'])
                                log.write(json.dumps(dict(case=f'{kind}_{frame}_{axis}_{sign}',**candidate))+'\n')
                            if b.snapshot()['phase']=='failed':raise RuntimeError(str(b.snapshot()))
                            time.sleep(.003)
                        after=reader.snapshot();b.stop();time.sleep(.2)
                        if b.take() is not None:raise RuntimeError('Candidate survived release')
                        if kind=='joint':
                            delta=np.deg2rad(np.array(after['status']['joint_state']['q_actual'])-
                                            before['status']['joint_state']['q_actual'])
                            expected_axis=axis
                        else:
                            def pose(s):
                                v=s['status']['tool_vector_actual']
                                return np.array([v[k] for k in ('x','y','z')])/1000,Rotation.from_euler(
                                    'xyz',[v[k] for k in ('rx','ry','rz')],degrees=True)
                            p0,r0=pose(before);p1,r1=pose(after)
                            delta=p1-p0 if axis<3 else (r1*r0.inv()).as_rotvec()
                            if frame=='tool':delta=r0.inv().apply(delta)
                            expected_axis=axis%3
                        norm=float(np.linalg.norm(delta));cosine=float(sign*delta[expected_axis]/norm) if norm>1e-9 else 0.
                        result['cases'].append(dict(kind=kind,frame=frame,axis=axis,sign=sign,count=count,
                            before=before,after=after,statuses=sorted(statuses,key=str),delta_si=delta.tolist(),
                            direction_cosine=cosine,direction_pass=cosine>.97 and norm>1e-6))
        result['all_cases_generated_candidates']=all(c['count']>0 for c in result['cases'])
        result['all_directions_pass']=all(c['direction_pass'] for c in result['cases'])
    except Exception as exc:result['error']=str(exc)
    finally:
        b.close();result['bridge']=b.snapshot()
        result['servo_exit_code']=b.process.returncode if b.process else None
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='cases'}))
    return int(result['error'] is not None or not result.get('all_directions_pass'))


if __name__=='__main__':raise SystemExit(main())
