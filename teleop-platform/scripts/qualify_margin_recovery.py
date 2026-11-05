"""Real Servo/FCL isolated replay. No hardware IO or network routes."""
from pathlib import Path
import argparse,json,math,select,subprocess,time,hashlib
import numpy as np
import yaml
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.margin_recovery import validate_reply


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();root=a.root
    out=root/'recovery-qualification';out.mkdir(exist_ok=False)
    params,profile=load_jazzy_bundle(Path('/qualification'),.05,'normal','omnipicker-r4',True)
    params['moveit_servo'].update(publish_period=.05,max_expected_latency=.2);params['update_period']=.05
    config=out/'params.yaml';config.write_text(yaml.safe_dump({'/e6_cpp_adapter':{'ros__parameters':params}}))
    seed=np.array(json.loads((root/'current-state.json').read_text())['actual']['joints_rad'])
    binary=root/'build/servo_cpp_adapter';frames=[];seq=epoch=0;last=0.;failures=[]
    with (out/'servo.log').open('w') as log:
        proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        def step(q,v,active=True,kind=0,counts=(0,-1,-1,-1),hold=1,label='path'):
            nonlocal seq,last
            time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
            values=[seq,epoch,int(active),kind,0,*q,*v,'G3',*counts,hold]
            proc.stdin.write(' '.join(map(str,values))+'\n');proc.stdin.flush()
            assert select.select([proc.stdout],[],[],15)[0],'solver timeout'
            z=json.loads(proc.stdout.readline());assert z['seq']==seq
            pending=dict(kind='j' if kind==0 else 't',active=active,gripper_counts=list(counts),gripper_hold=hold,q=list(q),effective_intent=dict(joint_rad_s=list(v)))
            validate_reply(z,pending)
            accepted=z['status'] in (0,1,3,4) and z['callback_us']<42000 and z['gripper_check']['arm_safe']
            row=dict(case=label,actual=list(q),input=list(v),counts=list(counts),hold=hold,accepted=accepted,reply=z);frames.append(row)
            return row
        def neutral(q):
            nonlocal epoch
            epoch+=1
            for _ in range(4):step(q,np.zeros(6),False,label='neutral')
            epoch+=1
        try:
            # Two complete exits from the real recorded pose, with and without
            # lag. The only simulated plant update is from accepted targets.
            for follow in (1.,.5):
                q=seed.copy();neutral(q);accepted=0
                for k in range(240):
                    v=np.zeros(6);v[3]=math.radians(.5)
                    row=step(q,v,label='j4-exit-'+str(follow));z=row['reply']
                    if row['accepted']:
                        end=np.array(z['joints_rad']);assert end[3]>=q[3]-1e-10
                        assert np.max(np.abs(np.delete(end-q,3)))<1e-9
                        q+=follow*(end-q);accepted+=1
                    elif k>4:failures.append(dict(case=row['case'],step=k,reason=z['gripper_check']['arm_reason'],recovery=z.get('margin_recovery')))
                    if q[3]-seed[3]>=math.radians(2.5):break
                if q[3]-seed[3]<math.radians(2.5):failures.append(dict(case='insufficient_exit',follow=follow,delta_deg=math.degrees(q[3]-seed[3]),accepted=accepted))
                neutral(q)
            # None of these input/state changes may produce a recovery permit.
            for label,kind,v,counts,hold in [
                ('inward',0,[0,0,0,-.005,0,0],(0,-1,-1,-1),1),
                ('multi',0,[0,0,.005,.005,0,0],(0,-1,-1,-1),1),
                ('cartesian',1,[.001,0,0,0,0,0],(0,-1,-1,-1),1),
                ('unknown',0,[0,0,0,.005,0,0],(-1,-1,-1,-1),0),
                ('moving',0,[0,0,0,.005,0,0],(0,-1,-1,-1),0),
                ('contact',0,[0,0,0,.005,0,0],(0,0,-1,0),2),
                ('new_gripper_target',0,[0,0,0,.005,0,0],(0,0,4,4),1)]:
                neutral(seed)
                for _ in range(4):
                    row=step(seed,v,kind=kind,counts=counts,hold=hold,label=label)
                    if row['reply'].get('margin_recovery',{}).get('verified'):failures.append(dict(case=label,reason='unsafe_recovery_permit'))
            # Neutral/release must erase every old position/velocity.
            neutral(seed)
            for row in frames:
                if not row['reply']['active']:
                    assert row['reply']['reset'] and np.allclose(row['reply']['joints_rad'],row['actual'],atol=1e-12,rtol=0)
                    assert not any(row['reply']['velocities_rad_s'])
        finally:
            proc.stdin.close();proc.wait(10)
            (out/'frames.json').write_text(json.dumps(frames));(out/'failures.json').write_text(json.dumps(failures,indent=2))
    recoveries=[x for x in frames if x['reply'].get('margin_recovery',{}).get('verified')]
    summary=dict(passed=not failures,failures=failures,frames=len(frames),recoveries=len(recoveries),hardware_io=False,
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),assembly_id=profile['assembly_id'],
        recovery_callback_ms=np.percentile([x['reply']['callback_us']/1000 for x in recoveries],[50,95,100]).tolist() if recoveries else [],
        min_source_clearance_m=min((x['reply']['margin_recovery']['source_clearance_m'] for x in recoveries),default=None))
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
    assert summary['passed'] and len(recoveries)>30,summary


if __name__=='__main__':main()
