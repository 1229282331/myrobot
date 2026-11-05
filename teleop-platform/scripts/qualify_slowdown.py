"""Recorded-pose CAN polling/contact regression in an isolated network namespace.

No robot/CAN/HTTP client. The plant is a delayed array of synthetic joint states.
Run both the original assembly and the refined assembly to expose interactions.
"""
from pathlib import Path
import argparse,collections,hashlib,json,select,subprocess,time
import numpy as np
import yaml
from e6_phosphobot import assembly_model
from e6_phosphobot.intent_ramp import IntentRamp
from e6_phosphobot.tool_frame import get_tool
from scipy.spatial.transform import Rotation

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
    ap.add_argument('--model',choices=['baseline','candidate'],required=True)
    ap.add_argument('--binary',choices=['baseline','candidate'],default='candidate')
    ap.add_argument('--smoke',action='store_true',help='Replay the retained contact/15 mm/s failure first')
    ap.add_argument('--smoke-index',type=int,default=-1)
    args=ap.parse_args();root=args.root
    assembly_model.ASSETS=root/('model-baseline' if args.model=='baseline' else 'python/e6_phosphobot/assets/workspace')
    from e6_phosphobot.moveit_config import load_jazzy_bundle
    params,_=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
    out=root/(('smoke-compound'+('' if args.smoke_index<0 else '-'+str(args.smoke_index))) if args.smoke else 'slowdown-'+args.model+'-'+args.binary);out.mkdir(exist_ok=False)
    cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    fixtures=json.loads((root/'slowdown-fixtures.json').read_text())
    binary=root/('production/servo_cpp_adapter' if args.binary=='baseline' else 'build/servo_cpp_adapter')
    seq=epoch=0;last=0.;ramp=IntentRamp(.2);frames=[];cases=[];tool=get_tool('omnipicker-r4')
    with (out/'solver.log').open('w') as log:
        proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        def step(q,v,active,counts,hold,label,frame=False):
            nonlocal seq,last
            time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
            intent=dict(linear_m_s=list(v[:3]),angular_rad_s=list(v[3:]),joint_rad_s=[0]*6,frame='tool' if frame else 'base')
            if active:
                shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s']
            else:ramp.reset();effective=[0]*6
            extra=['G2',*counts] if args.binary=='baseline' else ['G3',*counts,hold]
            proc.stdin.write(' '.join(map(str,[seq,epoch,int(active),1,int(frame),*q,*effective,*extra]))+'\n');proc.stdin.flush()
            if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('Isolated solver response')
            reply=json.loads(proc.stdout.readline());assert reply['seq']==seq
            fk=reply['target_fk'];p,o=tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
            frames.append(dict(case=label,q=list(q),requested=list(v),counts=counts,hold=hold,
                               tcp=p.tolist(),orientation=o.as_quat().tolist(),reply=reply))
            return reply
        try:
            for fixture in fixtures:
                if args.smoke and fixture['name']!='contact':continue
                for speed in (.003,.006,.015):
                    for delay in (0,2,4):
                        if args.smoke and (speed!=.015 or delay!=0):continue
                        if args.binary=='baseline' and (speed!=.006 or delay!=2):continue
                        q=np.array(fixture['q']);epoch+=1;zero=np.zeros(6)
                        label=f"{fixture['name']}-{speed}-{delay}";raw=fixture['raw'];sent=fixture['sent'];hold=fixture['hold']
                        counts=[raw,sent,-1,sent]
                        for _ in range(6):step(q,zero,False,counts,hold,label)
                        pending=collections.deque([q.copy() for _ in range(delay)]);replies=[]
                        for sign in (1.,-1.,1.):
                            for i in range(40):
                                # Repeat unchanged target transactions as in epoch149.
                                counts=[raw,sent,sent if i%4==0 else -1,sent]
                                v=np.r_[np.array(fixture['direction'])*speed*sign,[0.,0.,0.]]
                                reply=step(q,v,True,counts,hold,label);replies.append(reply)
                                pending.append(np.array(reply['joints_rad']));q=pending.popleft()
                        release=step(q,zero,False,counts,hold,label)
                        assert release['reset'] and np.max(abs(np.array(release['joints_rad'])-q))<1e-12
                        reasons=collections.Counter(x['collision_retreat']['reason'] for x in replies)
                        stops=sum(x['status'] in (-1,2,5,6) for x in replies)
                        budgets=sum(x['collision_retreat']['budget_exhausted'] or x['gripper_check']['budget_exhausted'] for x in replies)
                        case=dict(case=label,frames=len(replies),reasons=dict(reasons),stops=stops,budgets=budgets,
                            callback_ms=dict(zip(('p50','p95','max'),np.percentile([x['callback_us']/1000 for x in replies],[50,95,100]))))
                        cases.append(case);print(json.dumps(case),flush=True)
                        if args.binary=='candidate':
                            assert not reasons['gripper_not_stationary'],case
                            assert not reasons['no_current_proximity'],case
                            for row in replies:
                                cr=row['collision_retreat']
                                if cr['reason']=='clearance_restored':assert cr['boosted'] and cr['verified'] and cr['selected_scale']==1.
                            assert not stops and not budgets,case
            if args.binary=='candidate' and args.model=='candidate' and not args.smoke:
                def pose(reply):
                    fk=reply['target_fk'];return tool.apply(np.array(fk['position_m']),Rotation.from_quat(fk['quaternion_xyzw']))
                for fixture in fixtures:
                    for frame in (False,True):
                        for axis in range(6):
                            for sign in (-1,1):
                                label=f"axes-{fixture['name']}-{frame}-{axis}-{sign}";epoch+=1
                                q=np.array(fixture['q']);raw=fixture['raw'];sent=fixture['sent'];hold=fixture['hold']
                                counts=[raw,sent,-1,sent]
                                for _ in range(6):first=step(q,np.zeros(6),False,counts,hold,label,frame)
                                p0,o0=pose(first);replies=[];v=np.zeros(6);v[axis]=sign*(.003 if axis<3 else .02)
                                for i in range(20):
                                    counts=[raw,sent,sent if i%4==0 else -1,sent]
                                    reply=step(q,v,True,counts,hold,label,frame);replies.append(reply);q=np.array(reply['joints_rad'])
                                p,o=pose(reply);delta=p-p0 if axis<3 else (o*o0.inv()).as_rotvec()
                                expected=np.eye(3)[axis%3]*sign
                                if frame:expected=o0.apply(expected)
                                cosine=float(delta@expected/max(np.linalg.norm(delta),1e-12))
                                invariant=float((o*o0.inv()).magnitude()) if axis<3 else float(np.linalg.norm(p-p0))
                                stops=sum(x['status'] in (-1,2,5,6) for x in replies)
                                budgets=sum(x['collision_retreat']['budget_exhausted'] or x['gripper_check']['budget_exhausted'] for x in replies)
                                assert not stops and not budgets and cosine>.995 and invariant<(.001 if axis<3 else .0003),(label,cosine,invariant,stops,budgets)
                                released=step(q,np.zeros(6),False,counts,hold,label,frame)
                                assert released['reset'] and np.max(abs(np.array(released['joints_rad'])-q))<1e-12
                                case=dict(case=label,passed=True,frames=len(replies),stops=stops,budgets=budgets,cosine=cosine,invariant=invariant)
                                cases.append(case);print(json.dumps(case),flush=True)
            if args.binary=='candidate':
                fixture=fixtures[0];q=np.array(fixture['q']);epoch+=1;raw=fixture['raw'];sent=fixture['sent']
                for label,counts,hold in [('moving',[raw,sent,sent+4,sent+4],0),
                    ('unknown',[-1,sent,-1,sent],0),('stale',[raw,sent,-1,sent],0),
                    ('escape',[raw,sent,sent+4,sent+4],0)]:
                    for _ in range(5):step(q,np.zeros(6),False,counts,hold,label)
                    for _ in range(5):
                        reply=step(q,np.r_[np.array(fixture['direction'])*.006,[0.,0.,0.]],True,counts,hold,label)
                        assert not reply['collision_retreat']['boosted']
                    cases.append(dict(case=label,passed=True))
        finally:
            proc.stdin.close()
            try:proc.wait(10)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(5)
            (out/'frames.json').write_text(json.dumps(frames));(out/'cases.json').write_text(json.dumps(cases,indent=2))
    summary=dict(passed=True,hardware_io=False,model_id=assembly_model.manifest()['model_id'],
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),cases=len(cases),frames=len(frames),
        stops=sum(c.get('stops',0) for c in cases),budgets=sum(c.get('budgets',0) for c in cases))
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))

if __name__=='__main__':main()
