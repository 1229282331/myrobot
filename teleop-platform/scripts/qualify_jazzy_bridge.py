"""Exercise the SAME isolated production bridge with 20 Hz endpoint targets.

First-order virtual plant only; no E6/CAN connections or device writes.
"""
import argparse
import gc
import gzip
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge


class DeadlineCapture:
    """Use the production scheduler while keeping the virtual plant updating.

    This is only a wait substitute: production wait computes the same delay,
    while the outer 5 ms loop advances feedback until that deadline.
    """
    def wait(self, timeout):
        self.next_tick = time.monotonic() + timeout
        return False


class Plant:
    def __init__(self,bundle,tau):
        cfg=json.loads((bundle/'jazzy-runtime.json').read_text());self.q=np.deg2rad(json.loads((bundle/'seed.json').read_text()))
        self.target=self.q.copy();self.tau=tau;self.last=time.monotonic()
        self.proc=subprocess.Popen(['chroot',cfg['rootfs'],'/bin/bash','-c',
            'source /opt/ros/jazzy/setup.bash; exec "$1" "$2" controller_base controller_flange','bash',
            cfg['bundle']+'/jazzy-adapter-v35/build/kdl_model_probe',cfg['bundle']+'/model/e6.urdf'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.update()

    def update(self):
        now=time.monotonic();dt=now-self.last;self.last=now
        self.q+=(self.target-self.q)*(1 if self.tau==0 else 1-math.exp(-dt/self.tau))
        self.proc.stdin.write(' '.join(map(str,self.q))+'\n');self.proc.stdin.flush();model=json.loads(self.proc.stdout.readline())
        pose=[*(np.array(model['position_m'])*1000),*Rotation.from_quat(model['quaternion_xyzw']).as_euler('xyz',degrees=True)]
        self.s=dict(connected=True,age_ms=0,received_ns=time.perf_counter_ns(),generation=1,
            status=dict(joint_state=dict(q_actual=np.rad2deg(self.q).tolist()),
                        tool_vector_actual=dict(zip(('x','y','z','rx','ry','rz'),pose))))

    def snapshot(self):
        s=dict(self.s);s['age_ms']=(time.perf_counter_ns()-s['received_ns'])/1e6;return s

    def stop_fixture(self):
        # Explicit substitute for independent hardware Stop, NOT evidence of
        # actual E6 stop timing. The bridge must discard every prior target.
        self.target=self.q.copy()

    def close(self):self.proc.stdin.close();self.proc.wait(timeout=3)


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--tau',type=float,default=.15)
    p.add_argument('--suite',choices=['speed','directions'],default='speed')
    p.add_argument('--gateway-cadence',action='store_true')
    p.add_argument('--work-ms',type=float,default=0)
    p.add_argument('--motion-profile',default='commissioning')
    p.add_argument('--api',choices=['ros','cpp'],default='ros');a=p.parse_args()
    if a.gateway_cadence:
        from e6_phosphobot.gateway import wait_control_cycle
    a.output.mkdir(parents=True,exist_ok=False);plant=Plant(a.bundle,a.tau);bridge=MoveItBridge(a.bundle,plant,motion_profile=a.motion_profile,api=a.api)
    speed_profile=bridge.motion_profile
    result=dict(hardware_io=False,device_period_s=.05,virtual_tau_s=a.tau,suite=a.suite,
                motion_profile=speed_profile.description(),api=a.api,error=None,cases=[],
                gateway_cadence=a.gateway_cadence,synthetic_compute_ms=a.work_ms)
    schedule=DeadlineCapture()
    seq=0;log=gzip.open(a.output/'samples.jsonl.gz','wt');all_sources=set();gc_events=[];gc_start={}
    def gc_probe(phase,info):
        now=time.perf_counter_ns();generation=info['generation']
        if phase=='start':gc_start[generation]=now
        elif generation in gc_start:
            elapsed=(now-gc_start[generation])/1e6
            if elapsed>10:gc_events.append(dict(at_ns=now,duration_ms=elapsed,generation=generation))
    gc.callbacks.append(gc_probe)
    try:
        bridge.start();deadline=time.monotonic()+40
        while not bridge.snapshot()['ready']:
            plant.update()
            if bridge.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        info=bridge.snapshot()
        for name in ['servo','kdl']:
            if name+'_pid' not in info:continue
            (a.output/(name+'-maps.txt')).write_text(Path('/proc',str(info[name+'_pid']),'maps').read_text())
        cases=([(kind,frame,axis,sign,1.) for kind,frame in [('joint','base'),('cartesian','base'),('cartesian','tool')]
                for axis in range(6) for sign in (-1,1)] if a.suite=='directions' else
               [('joint','base',5,sign,speed) for speed in (1.,.25) for sign in (1,-1)])
        for kind,frame,axis,sign,speed in cases:
            values=dict(linear_m_s=[0.]*3,angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,gripper_rate_s=0,deadman=True,frame=frame)
            key='joint_rad_s' if kind=='joint' else 'linear_m_s' if axis<3 else 'angular_rad_s'
            maximum=(speed_profile.linear_m_s if key=='linear_m_s' else
                     math.radians(speed_profile.input_joint_deg_s if key=='joint_rad_s' else speed_profile.angular_deg_s))
            request=sign*speed*maximum
            values[key][axis if kind=='joint' else axis%3]=request;intent=Intent(**values)
            before=plant.snapshot();start=time.monotonic();next_tick=start;count=0;samples=[];sources=set();gaps=[];last_target=None;first_target=None;last_sent_ns=None
            duration=6 if a.suite=='speed' else 1.3
            while time.monotonic()-start<duration:
                now=time.monotonic();plant.update();candidate=None
                if now>=next_tick:
                    begin_ns=time.perf_counter_ns()
                    seq+=1;sources.add(seq);all_sources.add(seq);bridge.submit(intent,seq)
                    if a.work_ms:time.sleep(a.work_ms/1000)
                    candidate=bridge.take();next_tick=now+.05
                    if candidate:
                        if candidate['input_sequence'] not in sources:raise RuntimeError('Cross-epoch input selected')
                        for source in candidate['trajectory_sources']:
                            if source['input'] and source['input']['sequence'] not in sources:raise RuntimeError('Cross-epoch window source')
                        if candidate['pre_action_received_ns']>time.perf_counter_ns():raise RuntimeError('Observation after action')
                        plant.target=np.array(candidate['joints_rad']);count+=1
                        last_sent_ns=time.perf_counter_ns();dispatch=time.monotonic()
                        if first_target is None:first_target=dispatch-start
                        if last_target is not None:gaps.append(dispatch-last_target)
                        last_target=dispatch
                    if a.gateway_cadence:
                        wait_control_cycle(schedule,None,begin_ns,.05,last_sent_ns)
                        next_tick=schedule.next_tick
                sample=dict(case=len(result['cases']),t_s=now-start,q=plant.q.tolist(),target=plant.target.tolist(),candidate=candidate)
                # Retain only numeric regressors. Full provenance is streamed
                # to evidence, not held in a growing control-thread heap.
                samples.append(dict(t_s=now-start,q=plant.q.tolist()));log.write(json.dumps(sample)+'\n')
                if bridge.snapshot()['phase']=='failed':raise RuntimeError(str(bridge.snapshot()))
                time.sleep(.005)
            after=plant.snapshot();bridge.stop();plant.stop_fixture();hold=plant.q.copy()
            until=time.monotonic()+.3
            while time.monotonic()<until:
                plant.update()
                if bridge.take() is not None:raise RuntimeError('Candidate survived release')
                time.sleep(.005)
            def pose(s):
                v=s['status']['tool_vector_actual'];return np.array([v[k] for k in ('x','y','z')])/1000,Rotation.from_euler('xyz',[v[k] for k in ('rx','ry','rz')],degrees=True)
            if kind=='joint':delta=np.deg2rad(np.array(after['status']['joint_state']['q_actual'])-before['status']['joint_state']['q_actual']);index=axis
            else:
                p0,r0=pose(before);p1,r1=pose(after);delta=p1-p0 if axis<3 else (r1*r0.inv()).as_rotvec()
                if frame=='tool':delta=r0.inv().apply(delta)
                index=axis%3
            norm=float(np.linalg.norm(delta));cosine=sign*float(delta[index])/norm if norm>1e-9 else 0
            case=dict(kind=kind,frame=frame,axis=axis,sign=sign,requested_si=request,candidates=count,
                direction_cosine=cosine,max_target_gap_s=max(gaps,default=None),first_target_delay_s=first_target,
                dispatch_gap_percentiles_s=np.percentile(gaps,[0,50,95,99,100]).tolist() if gaps else [],
                hold_excursion_rad=float(np.max(abs(plant.q-hold))))
            if a.suite=='speed':
                steady=[r for r in samples if 2<r['t_s']<5.8]
                actual=float(np.polyfit([r['t_s'] for r in steady],[r['q'][axis] for r in steady],1)[0])
                case.update(actual_si=actual,ratio=actual/request)
            case['passed']=bool(count>0 and first_target<=.2 and cosine>.97 and case['hold_excursion_rad']<1e-12 and
                                max(gaps,default=0)<=.15 and (.95<=case.get('ratio',1)<=1.05))
            result['cases'].append(case)
            if not case['passed']:raise RuntimeError('Qualification case failed: '+str(case))
        result['passed']=all(c['passed'] for c in result['cases'])
    except Exception as exc:result['error']=str(exc)
    finally:
        bridge.close();plant.close();log.close();result['bridge']=bridge.snapshot()
        gc.callbacks.remove(gc_probe);result['gc_pauses_over_10ms']=gc_events
        result['child_exit']=bridge.process.returncode if bridge.process else None
        (a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    return int(result['error'] is not None or not result.get('passed'))


if __name__=='__main__':raise SystemExit(main())
