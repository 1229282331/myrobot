"""Isolated X5 replay of captured geometry timeouts. No hardware interfaces.

Run inside a network namespace with only loopback. Output retains every query,
including failures; synthetic feedback is never published to a live session.
"""
from pathlib import Path
import argparse, json, subprocess, select, time, math, hashlib
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.intent_ramp import IntentRamp


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);args=ap.parse_args();root=args.root
    params,profile=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
    config=root/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    cases=json.loads((root/'timeout-cases.json').read_text());rows=[]
    for _ in range(5):
        for c in cases:
            g=c['gripper_check'];rows.append(' '.join(map(str,[*g['arm_q_rad'],*g['arm_target_rad'],g['actual_raw'],g['sent_raw'],g['inflight_raw'],g['requested_raw']])))
    result={};checks=[]
    for name,binary in [('baseline',Path('/qualification/arm-proximity-v271/build/dynamic_gripper_probe')),('candidate',root/'build/dynamic_gripper_probe')]:
        p=subprocess.run([str(binary),'--ros-args','--params-file',str(config)],input='\n'.join(rows)+'\n',capture_output=True,text=True,timeout=120)
        (root/(name+'-replay.log')).write_text(p.stderr);assert p.returncode==0,p.stderr[-3000:]
        values=[json.loads(l) for l in p.stdout.splitlines() if l.startswith('{')];assert len(values)==len(rows)
        (root/(name+'-replay-rows.json')).write_text(json.dumps(values))
        result[name]=dict(queries=len(values),arm_denied=sum(not v['arm_safe'] for v in values),proposal_denied=sum(not v['allowed'] for v in values),budget_exhausted=sum(v['budget_exhausted'] for v in values),elapsed_ms=np.percentile([v['elapsed_us']/1000 for v in values],[50,95,99,100]).tolist())
        if name=='candidate':checks=values
    (root/'replay-summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    assert result['candidate']['arm_denied']==result['candidate']['proposal_denied']==result['candidate']['budget_exhausted']==0
    # Deterministic expiry: even a known safe stationary interval cannot issue
    # an arm or gripper permit after its caller's deadline has expired.
    g=cases[-1]['gripper_check'];q=g['arm_q_rad']
    expired=[' '.join(map(str,[*q,*q,raw,raw,-1,raw,0])) for raw in (38,39,89,153,-1)]
    p=subprocess.run([str(root/'build/dynamic_gripper_probe'),'--ros-args','--params-file',str(config)],input='\n'.join(expired)+'\n',capture_output=True,text=True,timeout=30)
    (root/'deadline-test.log').write_text(p.stderr);assert p.returncode==0,p.stderr[-3000:]
    values=[json.loads(l) for l in p.stdout.splitlines() if l.startswith('{')]
    (root/'deadline-results.json').write_text(json.dumps(values,indent=2))
    assert len(values)==5 and all(not r['arm_safe'] and not r['allowed'] and r['budget_exhausted'] and r['arm_reason']=='sweep_budget' for r in values)
    # Full official Servo cycle: both frames / all rotation axes / signs,
    # four actual apertures, then moving and in-flight gripper targets. All
    # feedback below is synthetic; joint targets are fed back only to stdin.
    seq=epoch=0;last=0.;frames=[];summaries=[];ramp=IntentRamp(.2);label='startup'
    with (root/'rotation-solver.log').open('w') as log:
        process=subprocess.Popen([str(root/'build/servo_cpp_adapter'),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        def step(q,counts,active,v,tool):
            nonlocal seq,last
            time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
            intent=dict(linear_m_s=list(v[:3]),angular_rad_s=list(v[3:]),joint_rad_s=[0]*6,frame='tool' if tool else 'base')
            if active:
                shaped,_=ramp.step(intent,epoch);effective=shaped['linear_m_s']+shaped['angular_rad_s']
            else:ramp.reset();effective=[0]*6
            process.stdin.write(' '.join(map(str,[seq,epoch,int(active),1,int(tool),*q,*effective,'G2',*counts]))+'\n');process.stdin.flush()
            assert select.select([process.stdout],[],[],10)[0],'Solver timed out'
            r=json.loads(process.stdout.readline());assert r['seq']==seq
            frames.append(dict(case=label,counts=counts,q=list(q),requested=list(v),reply=r));return r
        seed=np.array(cases[-1]['gripper_check']['arm_q_rad'])
        try:
            for raw in (38,39,89,153):
                for tool in (False,True):
                    for axis in (3,4,5):
                        for sign in (-1,1):
                            label=f'raw{raw}-{tool}-{axis}-{sign}';epoch+=1;q=seed.copy();counts=[raw,max(39,raw),-1,max(39,raw)]
                            for _ in range(3):step(q,counts,False,[0]*6,tool)
                            v=[0.]*6;v[axis]=sign*math.radians(10)
                            for _ in range(24):
                                r=step(q,counts,True,v,tool);q=np.array(r['joints_rad'])
                            released=step(q,counts,False,[0]*6,tool);assert released['reset'] and np.max(abs(np.array(released['joints_rad'])-q))<1e-12
                            step(q,counts,True,v,tool)
            for tool in (False,True):
                for axis in (0,3,4,5):
                    label=f'concurrent-{tool}-{axis}';epoch+=1;q=seed.copy()
                    for _ in range(3):step(q,[89,89,-1,89],False,[0]*6,tool)
                    v=[0.]*6;v[axis]=.03 if axis==0 else math.radians(10)
                    history=[]
                    for i in range(40):
                        raw=70+4*(i%12);counts=[raw,raw+4,raw+8,raw+8]
                        r=step(q,counts,True,v,tool);history.append(r['joints_rad'])
                        q=np.array(history[-2] if len(history)>1 else history[-1])
        finally:
            process.stdin.close();process.wait(15)
            (root/'rotation-frames.json').write_text(json.dumps(frames))
    for label in sorted(set(f['case'] for f in frames)):
        selected=[f['reply'] for f in frames if f['case']==label and f['reply']['active']]
        summaries.append(dict(case=label,frames=len(selected),timeouts=sum(r['gripper_check']['budget_exhausted'] for r in selected),geometry_denied=sum(not r['gripper_check']['arm_safe'] for r in selected),proposal_denied=sum(not r['gripper_check']['allowed'] for r in selected),callback_ms=np.percentile([r['callback_us']/1000 for r in selected],[50,95,99,100]).tolist(),statuses=sorted(set(r['status'] for r in selected))))
    summary=dict(model_id=profile['assembly_id'],hardware_io=False,binary_sha256=hashlib.sha256((root/'build/servo_cpp_adapter').read_bytes()).hexdigest(),cases=summaries,frames=len(frames),active_frames=sum(f['reply']['active'] for f in frames),timeouts=sum(r['timeouts'] for r in summaries),geometry_denied=sum(r['geometry_denied'] for r in summaries),proposal_denied=sum(r['proposal_denied'] for r in summaries),callback_ms=np.percentile([f['reply']['callback_us']/1000 for f in frames[3:]],[50,95,99,100]).tolist())
    summary['passed']=summary['timeouts']==summary['geometry_denied']==summary['proposal_denied']==0 and all(r['callback_ms'][-1]<50 for r in summaries)
    (root/'rotation-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='cases'}),flush=True)
    assert summary['passed'], 'Review saved per-case failures'

if __name__=='__main__':main()
