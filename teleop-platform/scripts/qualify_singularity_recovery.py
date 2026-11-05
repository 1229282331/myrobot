"""Production MoveIt bridge: singularity stop then joint-mode retreat in a virtual plant."""
import argparse
import gzip
import json
import math
from pathlib import Path
import time
import numpy as np
from qualify_jazzy_bridge import Plant
from e6_phosphobot.intent import Intent
from e6_phosphobot.motion_profiles import get_profile
from e6_phosphobot.moveit_bridge import MoveItBridge


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--motion-profile',default='normal')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    profile=get_profile(a.motion_profile);plant=Plant(a.bundle,.15)
    plant.q[4]=0;plant.target=plant.q.copy();plant.update()
    bridge=MoveItBridge(a.bundle,plant,motion_profile=profile.name)
    result=dict(hardware_io=False,profile=profile.description(),seed_deg=np.rad2deg(plant.q).tolist(),
                fixture_stop_not_physical_evidence=True,cases=[],error=None,passed=False)
    log=gzip.open(a.output/'samples.jsonl.gz','wt');sequence=0
    def phase(name,kind,index,speed,duration):
        nonlocal sequence
        values=dict(linear_m_s=[0.]*3,angular_rad_s=[0.]*3,joint_rad_s=[0.]*6,
                    gripper_rate_s=0,deadman=True,frame='base')
        values[kind][index]=speed;intent=Intent(**values)
        before=plant.q.copy();start=time.monotonic();tick=start;count=0;statuses=set();sources=set();max_delta=0
        while time.monotonic()-start<duration:
            plant.update();now=time.monotonic();candidate=None
            # Upstream intentionally emits no trajectory at a hard singularity;
            # its independent status topic is the evidence in that state.
            status=bridge.snapshot().get('status')
            if status is not None:statuses.add(status)
            if now>=tick:
                sequence+=1;sources.add(sequence);bridge.submit(intent,sequence);candidate=bridge.take();tick=now+.05
                if candidate:
                    if candidate['input_sequence'] not in sources:raise RuntimeError('Cross-epoch target')
                    if any(s['input'] and s['input']['sequence'] not in sources for s in candidate['trajectory_sources']):
                        raise RuntimeError('Cross-epoch window')
                    plant.target=np.array(candidate['joints_rad']);count+=1;statuses.add(candidate['status'])
            max_delta=max(max_delta,float(np.max(abs(plant.q-before))))
            log.write(json.dumps(dict(phase=name,t_s=now-start,q=plant.q.tolist(),status=status,candidate=candidate))+'\n')
            if bridge.snapshot()['phase']=='failed':raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        bridge.stop();plant.stop_fixture();end=time.monotonic()+.35
        while time.monotonic()<end:
            plant.update()
            if bridge.take() is not None:raise RuntimeError('Candidate survived release')
            time.sleep(.005)
        item=dict(name=name,candidates=count,statuses=sorted(statuses),delta_deg=np.rad2deg(plant.q-before).tolist(),
                  max_excursion_rad=max_delta)
        result['cases'].append(item);print(json.dumps(item),flush=True)
        return item
    try:
        bridge.start();deadline=time.monotonic()+40
        while not bridge.snapshot()['ready']:
            plant.update()
            if bridge.snapshot()['phase']=='failed' or time.monotonic()>deadline:raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        for name in ('servo','kdl'):
            (a.output/(name+'-maps.txt')).write_text(Path('/proc',str(bridge.snapshot()[name+'_pid']),'maps').read_text())
        blocked=phase('singular-cartesian','linear_m_s',0,.03,1.5)
        if 2 not in blocked['statuses'] or blocked['max_excursion_rad']>1e-5:
            raise RuntimeError('Expected singularity stop did not hold')
        retreat=phase('joint-retreat','joint_rad_s',4,math.radians(10),4.5)
        if retreat['delta_deg'][4]<35:raise RuntimeError('Joint retreat did not leave singularity')
        recovered=phase('cartesian-resumed','linear_m_s',0,-.03,1.5)
        if recovered['candidates']<15 or recovered['max_excursion_rad']<.005 or 2 in recovered['statuses']:
            raise RuntimeError('Cartesian control did not resume')
        result['passed']=True
    except Exception as exc:result['error']=str(exc)
    finally:
        bridge.close();plant.close();log.close();result['bridge']=bridge.snapshot()
        result['child_exit']=bridge.process.returncode if bridge.process else None
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    return int(not result['passed'])


if __name__=='__main__':raise SystemExit(main())
