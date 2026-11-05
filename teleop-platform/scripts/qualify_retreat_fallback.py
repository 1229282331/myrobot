"""Compare rejected-trial smoothing with the deployed baseline, without device IO."""
from pathlib import Path
import argparse,json,select,subprocess,time
import numpy as np

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);args=ap.parse_args();root=args.root
    out=root/'fallback';out.mkdir(exist_ok=False)
    q0=np.array(json.loads((root/'continuity-fixtures.json').read_text())[0]['q'])
    trace=[];replies={};maximum=0.
    for version in ('baseline','candidate'):
        binary=root/('prior/servo_cpp_adapter' if version=='baseline' else 'build/servo_cpp_adapter')
        rows=[];seq=0;last=0.;q=q0.copy()
        with (out/(version+'.log')).open('w') as log:
            p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(root/'params.yaml')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
            try:
                for i in range(86):
                    time.sleep(max(0,.05-(time.monotonic()-last)));last=time.monotonic();seq+=1
                    active=i>=6
                    # Approach at several input magnitudes while staying in the
                    # original proximity region. Candidate sees identical actuals.
                    speed=(-.002,-.004,-.001,-.003)[max(0,i-6)//20] if active else 0.
                    if version=='candidate':q=np.array(trace[i])
                    else:trace.append(q.tolist())
                    p.stdin.write(' '.join(map(str,[seq,1,int(active),1,0,*q,0,0,speed,0,0,0,'G2',12,12,-1,12]))+'\n');p.stdin.flush()
                    if not select.select([p.stdout],[],[],15)[0]:raise TimeoutError('solver timeout')
                    row=json.loads(p.stdout.readline());rows.append(row);q=np.array(row['joints_rad'])
                    if active:
                        assert row['status']==4 and not row['collision_retreat']['boosted'],row
                        assert row['gripper_check']['arm_safe'],row
            finally:
                p.stdin.close();p.wait(10)
                (out/(version+'.json')).write_text(json.dumps(rows))
        replies[version]=rows
    for a,b in zip(replies['baseline'],replies['candidate']):
        for key in ('joints_rad','velocities_rad_s'):
            difference=float(np.max(abs(np.array(a[key])-np.array(b[key]))));maximum=max(maximum,difference)
            assert difference<1e-9,(key,difference,a,b)
    (out/'summary.json').write_text(json.dumps(dict(passed=True,hardware_io=False,frames=len(trace),maximum_numeric_difference=maximum),indent=2))
    print('Rejected-trial smoothing matches baseline',maximum,flush=True)

if __name__=='__main__':main()
