"""Replay recorded page intent phases through the production MoveIt bridge.

No actuator, SDK or CAN imports. Virtual first-order plant, not an identified
E6 model. Original X5 phase durations are used; 20Hz dispatch is newly measured,
not claimed to recreate the original executor's unrecorded tick start times.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from qualify_jazzy_bridge import Plant
from e6_phosphobot.intent import Intent
from e6_phosphobot.moveit_bridge import MoveItBridge


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',required=True,type=Path)
    p.add_argument('--fixture',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--tau',required=True,type=float)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    fixture=json.loads(a.fixture.read_text());group=fixture['groups'][0]
    assert len(fixture['groups'])==1
    phases=group['phases'];base=group['base']
    assert phases and all(p['kind']=='rotation' and p['intent']['frame']=='base' for p in phases)
    begin=group['start_ns'];ends=[(p['end_ns']-begin)/1e9 for p in phases]
    plant=Plant(a.bundle,a.tau);plant.q=np.array(base['joints_rad']);plant.target=plant.q.copy();plant.update()
    bridge=MoveItBridge(a.bundle,plant,motion_profile='normal')
    result=dict(hardware_io=False,device_commands=0,can_sends=0,virtual_tau_s=a.tau,
        fixture_sha256=hashlib.sha256(a.fixture.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        seed_joints_rad=plant.q.tolist(),profile=bridge.motion_profile.description(),
        source='v107 original received-intent phase durations; new production 20Hz bridge dispatch',
        stop='instant virtual target freeze, never actual E6 stopping evidence',error=None,completed=False)
    log=gzip.open(a.output/'samples.jsonl.gz','xt');samples=[];dispatch=[];candidates=[];sequence=0
    try:
        bridge.start();deadline=time.monotonic()+40
        while not bridge.snapshot()['ready']:
            plant.update()
            if bridge.snapshot()['phase']=='failed' or time.monotonic()>deadline:
                raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.005)
        info=bridge.snapshot()
        for name in ('servo','kdl'):
            (a.output/(name+'-maps.txt')).write_text(Path('/proc',str(info[name+'_pid']),'maps').read_text())
        start=time.perf_counter_ns();next_tick=start;phase=0
        while (time.perf_counter_ns()-start)/1e9<ends[-1]:
            plant.update();now=time.perf_counter_ns();elapsed=(now-start)/1e9
            while phase<len(phases)-1 and elapsed>=ends[phase]:phase+=1
            candidate=None
            if now>=next_tick:
                intent=Intent(**phases[phase]['intent']);sequence+=1
                bridge.submit(intent,sequence)
                dispatch.append(dict(at_s=elapsed,phase=phase,sequence=sequence,intent=intent.model_dump()))
                candidate=bridge.take();next_tick=now+50_000_000
                if candidate:
                    assert candidate['epoch']==1 and candidate['input_sequence']<=sequence
                    assert all(x['input'] is None or x['input']['epoch']==1 for x in candidate['trajectory_sources'])
                    plant.target=np.array(candidate['joints_rad'])
                    candidates.append(dict(at_s=elapsed,phase=phase,candidate=candidate))
            s=plant.snapshot();v=s['status']['tool_vector_actual']
            pose=Rotation.from_euler('xyz',[v[k] for k in ('rx','ry','rz')],degrees=True)
            angle=(pose*Rotation.from_quat(base['quaternion_xyzw']).inv()).as_rotvec(degrees=True)
            samples.append(dict(t_s=elapsed,angle_deg=angle.tolist(),q_rad=plant.q.tolist(),phase=phase))
            log.write(json.dumps(dict(at_s=elapsed,phase=phase,actual=s,target_rad=plant.target.tolist(),candidate=candidate))+'\n')
            if bridge.snapshot()['phase']=='failed':raise RuntimeError(str(bridge.snapshot()))
            time.sleep(.003)
        bridge.stop();plant.stop_fixture()
        until=time.monotonic()+.3
        while time.monotonic()<until:
            plant.update()
            if bridge.take() is not None:raise RuntimeError('Candidate survived release')
            time.sleep(.005)
        result.update(completed=True,duration_s=ends[-1],candidates=len(candidates),dispatch_count=len(dispatch),
            first_candidate_s=candidates[0]['at_s'] if candidates else None,
            dispatch_gap_ms_p50_p95_max=np.percentile(np.diff([x['at_s'] for x in dispatch])*1000,[50,95,100]).tolist())
    except Exception as exc:
        result['error']=str(exc)
    finally:
        bridge.close();plant.close();log.close();result['bridge']=bridge.snapshot()
        result['forbidden_imports']=[m for m in sys.modules if m.split('.')[0] in ('dobot_sdk','can','socketcan')]
        (a.output/'numeric.json').write_text(json.dumps(dict(samples=samples,dispatch=dispatch,candidates=candidates)))
        (a.output/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='bridge'}))
    return int(not result['completed'] or bool(result['forbidden_imports']))


if __name__=='__main__':raise SystemExit(main())
