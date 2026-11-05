"""Requested SI speeds through the production Jazzy bridge and delayed plant.

No device IO. This qualifies a candidate profile, never its real stop distance
or workspace. Native Stop is explicitly substituted by the fixture after release.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from qualify_jazzy_bridge import Plant
from e6_phosphobot.intent import Intent
from e6_phosphobot.motion_profiles import get_profile
from e6_phosphobot.moveit_bridge import MoveItBridge


def pose(sample):
    v=sample['status']['tool_vector_actual']
    return np.array([v[k] for k in ('x','y','z')])/1000,Rotation.from_euler('xyz',[v[k] for k in ('rx','ry','rz')],degrees=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--motion-profile',choices=['commissioning','intermediate','normal'],default='normal')
    parser.add_argument('--cases',choices=['joint','cartesian','all'],default='all')
    parser.add_argument('--seed-deg',type=float,nargs=6,help='Explicit virtual pose only; never changes the production seed')
    parser.add_argument('--continue-on-failure',action='store_true',help='Complete independent cases; retain each failure and fail the suite')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    profile=get_profile(args.motion_profile)
    result=dict(hardware_io=False,profile=profile.description(),device_period_s=.05,virtual_tau_s=.15,
                virtual_stop='fixture target set to actual; NOT physical stop evidence',cases=[],error=None)
    plant=Plant(args.bundle,.15);bridge=MoveItBridge(args.bundle,plant,motion_profile=profile.name)
    if args.seed_deg is not None:
        plant.q=np.deg2rad(args.seed_deg);plant.target=plant.q.copy();plant.update()
    result['seed_deg']=np.rad2deg(plant.q).tolist()
    result['seed_source']='explicit_virtual_pose' if args.seed_deg is not None else 'archived_bundle_pose'
    log=gzip.open(args.output/'samples.jsonl.gz','wt');sequence=0
    groups=[]
    if args.cases in ('joint','all'):groups.append(('joint','base',[5]))
    if args.cases in ('cartesian','all'):
        groups.extend([('cartesian','base',range(6)),('cartesian','tool',range(6))])
    try:
        bridge.start();deadline=time.monotonic()+40
        while not bridge.snapshot()['ready']:
            plant.update()
            if bridge.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        for name in ('servo','kdl'):
            (args.output/(name+'-maps.txt')).write_text(Path('/proc',str(bridge.snapshot()[name+'_pid']),'maps').read_text())
        for kind,frame,axes in groups:
            for axis in axes:
                for scale in (1.,.25):
                    for sign in (1,-1):
                        values=dict(linear_m_s=[0.]*3,angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,
                                    gripper_rate_s=0,deadman=True,frame=frame)
                        key='joint_rad_s' if kind=='joint' else 'linear_m_s' if axis<3 else 'angular_rad_s'
                        magnitude=(math.radians(profile.input_joint_deg_s) if kind=='joint' else
                                   profile.linear_m_s if axis<3 else math.radians(profile.angular_deg_s))
                        index=axis if kind=='joint' else axis%3
                        request=sign*scale*magnitude;values[key][index]=request;intent=Intent(**values)
                        start=time.monotonic();next_tick=start;first=None;last=None;gaps=[];samples=[];sources=set();candidates=0
                        before=plant.snapshot();p0,r0=pose(before);q0=plant.q.copy()
                        statuses=set()
                        while time.monotonic()-start<2.5:
                            now=time.monotonic();plant.update();candidate=None
                            if now>=next_tick:
                                sequence+=1;sources.add(sequence);bridge.submit(intent,sequence)
                                candidate=bridge.take();next_tick=now+.05
                                if candidate:
                                    if candidate['input_sequence'] not in sources:raise RuntimeError('Cross-epoch target')
                                    for source in candidate['trajectory_sources']:
                                        if source['input'] and source['input']['sequence'] not in sources:raise RuntimeError('Cross-epoch window')
                                    plant.target=np.array(candidate['joints_rad']);candidates+=1
                                    if first is None:first=now-start
                                    if last is not None:gaps.append(now-last)
                                    last=now;statuses.add(candidate['status'])
                            p1,r1=pose(plant.snapshot())
                            delta=plant.q-q0 if kind=='joint' else p1-p0 if axis<3 else (r1*r0.inv()).as_rotvec()
                            if kind!='joint' and frame=='tool':delta=r0.inv().apply(delta)
                            samples.append((now-start,delta.tolist(),plant.q.tolist()))
                            log.write(json.dumps(dict(case=len(result['cases']),t_s=now-start,q=plant.q.tolist(),
                                                      target=plant.target.tolist(),candidate=candidate))+'\n')
                            if bridge.snapshot()['phase']=='failed':raise RuntimeError(str(bridge.snapshot()))
                            time.sleep(.005)
                        bridge.stop();plant.stop_fixture();hold=plant.q.copy();end=time.monotonic()+.35
                        while time.monotonic()<end:
                            plant.update()
                            if bridge.take() is not None:raise RuntimeError('Candidate survived release')
                            time.sleep(.005)
                        # The fit uses actual lagged FK samples, not the requested
                        # target or summed inputs. Every sample remains archived.
                        steady=[s for s in samples if 1.2<s[0]<2.4]
                        velocity=np.polyfit([s[0] for s in steady],[s[1] for s in steady],1)[0]
                        actual=float(velocity[index]);ratio=actual/request
                        cosine=float(sign*velocity[index]/np.linalg.norm(velocity)) if np.linalg.norm(velocity) else 0
                        case=dict(kind=kind,frame=frame,axis=axis,scale=scale,sign=sign,requested_si=request,
                                  actual_si=actual,ratio=ratio,direction_cosine=cosine,first_target_delay_s=first,
                                  max_target_gap_s=max(gaps,default=None),candidates=candidates,servo_statuses=sorted(statuses),
                                  hold_excursion_rad=float(np.max(abs(plant.q-hold))))
                        case['passed']=bool(first is not None and first<=.2 and max(gaps,default=0)<=.15
                            and .95<=ratio<=1.05 and cosine>.97 and case['hold_excursion_rad']<1e-12)
                        result['cases'].append(case)
                        print(json.dumps(case),flush=True)
                        if not case['passed'] and not args.continue_on_failure:
                            raise RuntimeError('Speed profile case failed; original evidence retained')
        result['passed']=all(c['passed'] for c in result['cases'])
    except Exception as exc:result['error']=str(exc)
    finally:
        bridge.close();plant.close();log.close();result['bridge']=bridge.snapshot()
        result['child_exit']=bridge.process.returncode if bridge.process else None
        (args.output/'summary.json').write_text(json.dumps(result,indent=2))
    return int(result['error'] is not None or not result.get('passed'))


if __name__=='__main__':raise SystemExit(main())
