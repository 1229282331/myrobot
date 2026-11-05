"""Production mapper fixtures against the real MuJoCo simulation core, offline."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from e6_teleop.core import GatewayCore
from e6_teleop.model import E6Model
from e6_teleop.math3d import integrate

def verify(fixtures):
    class Clock:
        ns=10_000_000_000
        def __call__(self):return self.ns
    root=Path(__file__).resolve().parents[1]
    assert fixtures['production_input_sha256']==hashlib.sha256((root/'web/src/input.ts').read_bytes()).hexdigest()
    results=[]
    for row in fixtures['rows']+fixtures.get('multi',[]):
        clock=Clock();model=E6Model(render=False);core=GatewayCore(model,clock=clock);core.connect()
        try:
            n=dict(fixtures['neutral'][row['source']],mode=row['mode'],frame=row['frame'])
            def send(payload):core.command(dict(payload,session=core.owner,seq=core.seq+1,lease=core.lease))
            send(n);core.command(dict(operation='arm',session=core.owner))
            before_p,before_r=model.pose();before_g=model.gripper
            for _ in range(3):
                send(row['input']);clock.ns+=50_000_000;frame,_=core.tick()
                assert frame['effective_action']['active'],frame['status']['reason']
            p,r=model.pose();scale=.25 if row['fine'] else 1
            expected_p,expected_r=integrate(before_p,before_r,np.asarray(row['input']['twist'][:3])*scale,np.asarray(row['input']['twist'][3:])*scale,.15,row['frame'])
            # Request integration has tight equality; FK's tested IK tolerance
            # is separately enforced by the simulation model.
            np.testing.assert_allclose(core.target_position,expected_p,atol=1e-10)
            assert abs(core.target_quaternion@expected_r)>1-1e-10
            expected_g=np.clip(before_g+row['input']['gripper_rate']*.15*scale,0,1)
            assert abs(model.gripper-expected_g)<1e-10
            held_q=model.q.copy();held_g=model.gripper;send(n);clock.ns+=50_000_000;core.tick()
            np.testing.assert_array_equal(model.q,held_q);assert model.gripper==held_g
            results.append({k:row[k] for k in ('source','mode','frame','fine','axis','axes','sign') if k in row})
        finally:model.close()
    return dict(passed=True,cases=len(results),coverage=results,source_sha256=fixtures['production_input_sha256'],hardware_connections=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('fixtures',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=verify(json.loads(a.fixtures.read_text()));a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2),encoding='utf-8');print(f"{result['cases']} production-mapping simulation cases passed")
