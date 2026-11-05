"""Minimal actual-state lag experiment using the genuine production Servo bridge.

No hardware sockets or CAN. The virtual plant's first-order lag is a declared
test fixture, not an identified model of E6. Compare identical input at tau=0
and tau>0 to detect a controller-interface assumption.
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


class LagReader:
    def __init__(self,bundle,seed,tau):
        self.q=np.deg2rad(seed);self.target=self.q.copy();self.tau=tau
        self.proc=subprocess.Popen([str(bundle/'kdl_model_probe'),str(bundle/'model/e6.urdf'),
            'controller_base','controller_flange'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.last=time.monotonic();self.update()

    def update(self):
        now=time.monotonic();dt=now-self.last;self.last=now
        self.q+=(self.target-self.q)*(1 if self.tau==0 else 1-math.exp(-dt/self.tau))
        self.proc.stdin.write(' '.join(map(str,self.q))+'\n');self.proc.stdin.flush()
        model=json.loads(self.proc.stdout.readline())
        pose=[*(np.array(model['position_m'])*1000),*Rotation.from_quat(model['quaternion_xyzw']).as_euler('xyz',degrees=True)]
        self.s=dict(connected=True,age_ms=0,received_ns=time.perf_counter_ns(),status=dict(
            joint_state=dict(q_actual=np.rad2deg(self.q).tolist()),
            tool_vector_actual=dict(zip(('x','y','z','rx','ry','rz'),pose))))

    def snapshot(self):
        result=dict(self.s);result['age_ms']=(time.perf_counter_ns()-self.s['received_ns'])/1e6
        return result

    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=3)


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--seed',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tau',type=float,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    reader=LagReader(a.bundle,json.loads(a.seed.read_text()),a.tau);b=MoveItBridge(a.bundle,reader)
    result=dict(hardware_connected=False,motion_commands=0,can_sends=0,tau_s=a.tau,error=None)
    try:
        b.start();deadline=time.monotonic()+40
        while not b.snapshot()['ready']:
            reader.update()
            if b.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(b.snapshot()))
            time.sleep(.01)
        intent=Intent(linear_m_s=[0]*3,angular_rad_s=[0]*3,joint_rad_s=[0]*5+[math.radians(.2)],
            gripper_rate_s=0,deadman=True,frame='base')
        start=time.monotonic();next_send=start;seq=0;rows=[]
        while time.monotonic()-start<6:
            now=time.monotonic();reader.update()
            if now>=next_send:
                seq+=1;b.submit(intent,seq);next_send=now+.05
            candidate=b.take()
            if candidate:reader.target=np.array(candidate['joints_rad'])
            rows.append(dict(t_s=now-start,q=reader.q.tolist(),target=reader.target.tolist(),candidate=candidate))
            if b.snapshot()['phase']=='failed':raise RuntimeError(str(b.snapshot()))
            time.sleep(.005)
        b.stop()
        steady=[r for r in rows if 2<r['t_s']<5.8]
        speed=np.polyfit([r['t_s'] for r in steady],[math.degrees(r['q'][5]) for r in steady],1)[0]
        result.update(actual_deg_s=float(speed),requested_deg_s=.2,tracking_ratio=float(speed/.2),
            samples=len(rows),candidates=sum(r['candidate'] is not None for r in rows))
        (a.output/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    except Exception as exc:result['error']=str(exc)
    finally:
        b.close();reader.close();result['bridge']=b.snapshot()
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result));return int(result['error'] is not None)


if __name__=='__main__':raise SystemExit(main())
