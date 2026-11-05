"""Isolated official Servo regression; synthetic stdin feedback, no device IO."""
from pathlib import Path
import argparse,json,subprocess,select,time,math,hashlib
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.tool_frame import get_tool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--smoke',action='store_true');a=ap.parse_args();root=a.root
    out=root/('smoke' if a.smoke else 'regression');out.mkdir(exist_ok=False)
    params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
    config=root/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    seed=np.array(json.loads((root/'current-state.json').read_text())['actual']['joints_rad'])
    tool=get_tool('omnipicker-r4');frames=[];cases=[]
    for version,binary in [('baseline',root/'baseline/servo_cpp_adapter'),('candidate',root/'build/servo_cpp_adapter')]:
        seq=epoch=0;last=0.;ramp=IntentRamp(.2);label='startup'
        with (out/(version+'.log')).open('w') as log:
            proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
            def step(q,v=None,active=False,counts=(12,12,-1,12),frame=False,kind=1):
                nonlocal seq,last
                time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
                v=np.zeros(6) if v is None else np.array(v)
                intent=dict(linear_m_s=list(v[:3]) if kind else [0]*3,angular_rad_s=list(v[3:]) if kind else [0]*3,joint_rad_s=[0]*6 if kind else list(v),frame='tool' if frame else 'base')
                if active:
                    shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s'] if kind else shaped['joint_rad_s']
                else:ramp.reset();effective=[0]*6
                proc.stdin.write(' '.join(map(str,[seq,epoch,int(active),kind,int(frame),*q,*effective,'G2',*counts]))+'\n');proc.stdin.flush()
                if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('Solver response timeout')
                line=proc.stdout.readline()
                if not line:raise RuntimeError('Solver exited '+str(proc.poll()))
                r=json.loads(line);assert r['seq']==seq
                frames.append(dict(version=version,case=label,q=list(q),counts=list(counts),requested=list(v),effective=effective,reply=r));return r
            def start(q,counts=(12,12,-1,12)):
                nonlocal epoch
                epoch+=1
                for _ in range(6):r=step(q,counts=counts)
                return r
            def pose(r):
                f=r['target_fk'];return tool.apply(np.array(f['position_m']),Rotation.from_quat(f['quaternion_xyzw']))
            try:
                for speed,sign in [(v,s) for v in ((.015,) if a.smoke else (.015,.03)) for s in (1,-1)]:
                    label=f'z-{speed}-{sign}';q=seed.copy();start(q);positions=[];replies=[]
                    for _ in range(55):
                        r=step(q,[0,0,sign*speed,0,0,0],True);q=np.array(r['joints_rad']);positions.append(pose(r)[0]);replies.append(r)
                    measured=float(np.linalg.norm(positions[-1]-positions[30])/(24*.05)*1000)
                    passed=(.1<measured<2.) if version=='baseline' or sign<0 else .95*speed*1000<measured<1.02*speed*1000
                    case=dict(version=version,case=label,passed=passed,speed_mm_s=measured,statuses=sorted(set(r['status'] for r in replies)),boosted=sum(r.get('collision_retreat',{}).get('boosted',False) for r in replies))
                    cases.append(case);print(json.dumps(case),flush=True);assert passed,case
                if version=='baseline' or a.smoke:continue
                # Reinitialize each direction at the same pose; direction is never projected.
                for frame in (False,True):
                    for axis in range(6):
                        for sign in (-1,1):
                            label=f'axes-{frame}-{axis}-{sign}';q=seed.copy();first=start(q);p0,o0=pose(first)
                            v=np.zeros(6);v[axis]=sign*(.004 if axis<3 else math.radians(2));replies=[]
                            for _ in range(25):
                                r=step(q,v,True,frame=frame);q=np.array(r['joints_rad']);replies.append(r)
                            p,o=pose(r);delta=p-p0 if axis<3 else (o*o0.inv()).as_rotvec();expected=np.eye(3)[axis%3]*sign
                            if frame:expected=o0.apply(expected)
                            cosine=float(delta@expected/max(np.linalg.norm(delta),1e-12))
                            invariant=float(math.degrees((o*o0.inv()).magnitude())) if axis<3 else float(np.linalg.norm(p-p0)*1000)
                            assert cosine>.99 and invariant<(.1 if axis<3 else .4),(label,cosine,invariant)
                            assert not any(r['status'] in (-1,2,5,6) for r in replies),(label,replies[-1])
                            release=step(q);assert release['reset'] and np.max(abs(np.array(release['joints_rad'])-q))<1e-12
                            restart=step(q,v,True,frame=frame);assert restart['reset']
                            cases.append(dict(case=label,passed=True,cosine=cosine,invariant=invariant))
                for axis in range(6):
                    for sign in (-1,1):
                        label=f'joint-{axis}-{sign}';q=seed.copy();start(q);v=np.zeros(6);v[axis]=sign*math.radians(2)
                        for _ in range(25):r=step(q,v,True,kind=0);q=np.array(r['joints_rad'])
                        d=q-seed;assert d[axis]*sign>0 and np.max(abs(np.delete(d,axis)))<1e-10
                        cases.append(dict(case=label,passed=True))
                for label,counts,allowed in [('observed-idle',(12,-1,-1,12),True),('observed-no-target',(12,-1,-1,-1),True),('gripper-new-target',(12,-1,-1,24),False),('gripper-moving',(12,20,24,24),False),('gripper-unknown',(-1,12,-1,12),False)]:
                    q=seed.copy();start(q,counts);boosts=[]
                    for _ in range(25):
                        r=step(q,[0,0,.015,0,0,0],True,counts=counts);q=np.array(r['joints_rad']);boosts.append(r['collision_retreat']['boosted'])
                    assert bool(any(boosts))==allowed,(label,boosts);cases.append(dict(case=label,passed=True))
                label='reverse-without-release';q=seed.copy();start(q)
                for sign in (-1,1,-1,1):
                    positions=[];replies=[]
                    for _ in range(40):
                        r=step(q,[0,0,sign*.004,0,0,0],True);q=np.array(r['joints_rad']);positions.append(pose(r)[0]);replies.append(r)
                    delta=positions[-1]-positions[-11]
                    assert delta[2]*sign>0,(label,sign,delta)
                    if sign>0:
                        assert delta[2]/.5>.0038 and all(r['status'] in (0,1,3) for r in replies[-10:]),(label,replies[-1])
                    assert not any(r['collision_retreat']['budget_exhausted'] or r['gripper_check']['budget_exhausted'] for r in replies)
                cases.append(dict(case=label,passed=True))
                label='one-step-plant-lag';q=seed.copy();start(q);previous=q.copy();positions=[]
                monitor_transitions=0
                for _ in range(45):
                    r=step(q,[0,0,.015,0,0,0],True);q=previous;previous=np.array(r['joints_rad']);positions.append(pose(r)[0])
                    cr=r['collision_retreat']
                    monitor_transition=(r['status']==4 and cr['reason']=='no_current_proximity' and
                                        cr['minimum_before_m']>=.01 and not cr['budget_exhausted'])
                    monitor_transitions+=monitor_transition
                    assert (r['status'] in (0,1,3) or monitor_transition) and r['gripper_check']['arm_safe'],r
                assert (positions[-1][2]-positions[-11][2])/.5>.014
                cases.append(dict(case=label,passed=True,monitor_transition_frames=monitor_transitions))
                for wrist in (0.,5.,20.):
                    label=f'singularity-{wrist}';q=seed.copy();q[4]=math.radians(wrist);start(q)
                    for _ in range(12):
                        r=step(q,[0,0,0,math.radians(2),0,0],True)
                        cond=r['singularity']
                        if r['status'] in (0,1,3,4) and cond['condition_before'] is not None and cond['condition_before']>=60:
                            assert cond['condition_target']<cond['condition_before']
                        if r['status'] in (-1,2,5,6):assert np.max(abs(np.array(r['joints_rad'])-q))<1e-12
                        q=np.array(r['joints_rad'])
                    if wrist==0:assert r['status'] in (-1,2,5,6)
                    cases.append(dict(case=label,passed=True,status=r['status']))
                fixture=next(c for c in json.loads((root/'geometry-cases.json').read_text()) if c['raw']>=0 and c['result']['collision'])
                label='actual-collision';q=np.array(fixture['q']);raw=fixture['raw'];counts=(raw,raw,-1,raw);start(q,counts)
                r=step(q,[0,0,.015,0,0,0],True,counts=counts)
                assert r['status']==5 and np.max(abs(np.array(r['joints_rad'])-q))<1e-12 and not r['collision_retreat']['boosted']
                cases.append(dict(case=label,passed=True))
            finally:
                proc.stdin.close()
                try:proc.wait(10)
                except subprocess.TimeoutExpired:proc.terminate();proc.wait(5)
                (out/'frames.json').write_text(json.dumps(frames));(out/'cases.json').write_text(json.dumps(cases,indent=2))
    cb=[f['reply']['callback_us']/1000 for f in frames if f['version']=='candidate' and f['reply']['active']]
    summary=dict(passed=all(c['passed'] for c in cases),hardware_io=False,binary_sha256=hashlib.sha256((root/'build/servo_cpp_adapter').read_bytes()).hexdigest(),frames=len(frames),cases=cases,callback_ms=dict(zip(('p50','p95','p99','max'),np.percentile(cb,[50,95,99,100]))))
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
