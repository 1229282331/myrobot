"""Delayed-plant and recorded-pose Servo checks. Run in a network namespace.

Synthetic joint observations only: this program has no robot or CAN client.
"""
from pathlib import Path
import argparse, collections, hashlib, json, select, subprocess, time
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.intent_ramp import IntentRamp


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',type=Path,required=True)
    ap.add_argument('--version',choices=['baseline','candidate'],required=True)
    ap.add_argument('--label',default='continuity')
    args=ap.parse_args();root=args.root
    out=root/(args.label+'-'+args.version);out.mkdir(exist_ok=False)
    binary=root/('prior/servo_cpp_adapter' if args.version=='baseline' else 'build/servo_cpp_adapter')
    params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2)
    params['update_period']=.05
    cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    fixtures=json.loads((root/'continuity-fixtures.json').read_text())
    frames=[];cases=[];seq=epoch=0;last=0.;ramp=IntentRamp(.2)
    with (out/'solver.log').open('w') as log:
        proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        def step(q,v,active,raw,label):
            nonlocal seq,last
            time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
            intent=dict(linear_m_s=list(v[:3]),angular_rad_s=list(v[3:]),joint_rad_s=[0]*6,frame='base')
            if active:
                shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s']
            else:ramp.reset();effective=[0]*6
            proc.stdin.write(' '.join(map(str,[seq,epoch,int(active),1,0,*q,*effective,'G2',raw,raw,-1,raw]))+'\n');proc.stdin.flush()
            if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('Isolated solver timeout')
            reply=json.loads(proc.stdout.readline());assert reply['seq']==seq
            frames.append(dict(case=label,q=list(q),requested=list(v),effective=effective,reply=reply))
            return reply
        try:
            for fixture in fixtures:
                for speed in (.003,.006,.015):
                    for delay in (0,2,4,6):
                        label=f"{fixture['name']}-{speed}-{delay}"
                        q=np.array(fixture['q']);epoch+=1;raw=fixture['raw'];zero=np.zeros(6)
                        for _ in range(6):step(q,zero,False,raw,label)
                        pending=collections.deque([q.copy() for _ in range(delay)])
                        rows=[];direction=np.array(fixture['direction'],dtype=float);direction/=np.linalg.norm(direction)
                        for phase,sign in enumerate((1.,-1.,1.)):
                            for i in range(60):
                                v=np.r_[direction*speed,[0,0,0]]
                                r=step(q,v*sign,True,raw,label)
                                pending.append(np.array(r['joints_rad']));q=pending.popleft()
                                rows.append(r)
                        release=step(q,zero,False,raw,label)
                        assert release['reset'] and np.max(abs(np.array(release['joints_rad'])-q))<1e-12
                        active=[r for r in rows if r['active']]
                        transitions=sum(a['collision_retreat']['boosted']!=b['collision_retreat']['boosted'] for a,b in zip(active,active[1:]))
                        budgets=sum(r['collision_retreat']['budget_exhausted'] or r['gripper_check']['budget_exhausted'] for r in active)
                        stops=sum(r['status'] in (-1,2,5,6) for r in active)
                        acc=np.diff(np.array([r['velocities_rad_s'] for r in active]),axis=0)/.05
                        case=dict(case=label,frames=len(active),delay_cycles=delay,speed_mm_s=speed*1000,
                            statuses=dict(collections.Counter(r['status'] for r in active)),
                            reasons=dict(collections.Counter(r['collision_retreat']['reason'] for r in active)),
                            boost_transitions=transitions,budget_frames=budgets,stop_frames=stops,
                            peak_joint_acceleration_rad_s2=float(np.max(abs(acc))),
                            callback_ms=dict(zip(('p50','p95','max'),np.percentile([r['callback_us']/1000 for r in active],[50,95,100]))))
                        cases.append(case);print(json.dumps(case),flush=True)
                        # Low-speed, already noncolliding fixtures must not
                        # periodically stop from exhausted geometry checks.
                        if args.version=='candidate':
                            assert not budgets and not stops,case
        finally:
            proc.stdin.close()
            try:proc.wait(10)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(5)
            (out/'frames.json').write_text(json.dumps(frames))
            (out/'cases.json').write_text(json.dumps(cases,indent=2))
    (out/'summary.json').write_text(json.dumps(dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        hardware_io=False,cases=len(cases),frames=len(frames),budgets=sum(c['budget_frames'] for c in cases),
        stops=sum(c['stop_frames'] for c in cases)),indent=2))


if __name__=='__main__':main()
