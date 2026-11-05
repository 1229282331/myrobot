"""Reviewed fold-in parking path for the installed gripper and camera.

Default read-only. Execute only after physical clearance is established.
No enable, disable, recovery, gripper, shutdown or controller-setting writes.
The nominal tool sphere and URDF origins do not certify collision clearance.
"""
import argparse,hashlib,json,math
from datetime import datetime,timezone
from pathlib import Path
try:
    from .e6_zero import E6Error,validate_joints
    from .e6_calibration_view import validate_stage,rotation_angle,run
    from .e6_observation_move import ObservationDashboard
except ImportError:
    from e6_zero import E6Error,validate_joints
    from e6_calibration_view import validate_stage,rotation_angle,run
    from e6_observation_move import ObservationDashboard

def vector(value,n):
    return isinstance(value,list) and len(value)==n and all(isinstance(x,(int,float)) and math.isfinite(x) for x in value)

def validate_plan(plan):
    if plan.get('status')!='sampled_complete_tool_park' or not 1<=len(plan.get('stages',[]))<=24:
        raise E6Error('Require the sampled complete-tool park plan')
    oq=plan['origin_q'];op=plan['origin_pose'];a0=plan['origin_arm_origins_mm']
    validate_joints(oq)
    if not vector(op,6) or not isinstance(a0,list) or len(a0)!=6 or any(not vector(a,3) for a in a0):
        raise E6Error('Invalid original geometry')
    if max(abs(a-b) for a,b in zip(oq,[-66.5019,-74.9236,42.5551,-57.6311,90,9.1974]))>.03:
        raise E6Error('Plan is not from the reviewed installed-tool origin')
    expected=[oq[0],-60,105,-90,oq[4],oq[5]]
    if not vector(plan['target_q'],6) or max(abs(a-b) for a,b in zip(plan['target_q'],expected))>1e-5:
        raise E6Error('Unexpected parking target')
    if plan.get('nominal_tool_radius_mm')!=170 or plan.get('table_z_mm')!=-22:
        raise E6Error('Changed nominal geometry')
    previous=oq
    for i,stage in enumerate(plan['stages']):
        validate_stage(stage,stage['start_q'])
        if stage['index']!=i or max(abs(a-b) for a,b in zip(previous,stage['start_q']))>1e-5:
            raise E6Error('Disconnected parking path')
        if any(sign*(stage['target_q'][k]-stage['start_q'][k])<0 for k,sign in [(1,1),(2,1),(3,-1)]):
            raise E6Error('Fold-in path reversed direction')
        if len(stage['samples'])!=9:raise E6Error('Require nine samples per segment')
        for s in stage['samples']:
            q=s['q'];p=s['pose'];arms=s['arm_origins_mm']
            if any(abs(q[k]-oq[k])>1e-5 for k in [0,4,5]):raise E6Error('Base or wrist roll/yaw changed')
            if any(not min(a,b)-1e-5<=v<=max(a,b)+1e-5 for v,a,b in zip(q,oq,expected)):
                raise E6Error('Joint path left the reviewed fold-in interval')
            if not (-10<=p[0]<=76 and -374<=p[1]<=-186 and 278<=p[2]<=398):
                raise E6Error('Flange left the reviewed Cartesian region')
            if p[2]-170+22<130 or rotation_angle(p,op)>46:
                raise E6Error('Nominal table clearance or orientation exceeded')
            if len(arms)!=6 or any(not vector(a,3) for a in arms):raise E6Error('Invalid arm origins')
            if any(math.dist(a,b)>limit for a,b,limit in zip(arms,a0,[1,1,51,220,250,242])):
                raise E6Error('Model origin sweep exceeded')
        previous=stage['target_q']
    if max(abs(a-b) for a,b in zip(previous,expected))>1e-5:raise E6Error('Incomplete parking path')

def load_inputs(path,digest,scene_path,first):
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=digest:raise E6Error('Plan SHA mismatch')
    plan=json.loads(raw);validate_plan(plan)
    if not 0<=first<len(plan['stages']):raise E6Error('Invalid first stage')
    observation=json.loads(scene_path.read_text(encoding='utf-8'))
    if observation.get('status')!='static_scene' or observation['after']['mode']!=5:
        raise E6Error('Require a stationary enabled camera observation')
    camera=Path(observation['source_run'])/'report.json';camera_raw=camera.read_bytes()
    if hashlib.sha256(camera_raw).hexdigest()!=observation['camera_report_sha256']:
        raise E6Error('Camera report hash changed')
    scene=json.loads(camera_raw)
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:
        raise E6Error('Require a clean camera capture within 180 seconds')
    start=plan['stages'][first]['start_q']
    if max(abs(a-b) for a,b in zip(observation['after']['q'],start))>.15:
        raise E6Error('Camera observation does not match this stage origin')
    return plan,age

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--scene-observation',type=Path,required=True);p.add_argument('--first',type=int,default=0)
    p.add_argument('--count',type=int,default=1);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    plan,age=load_inputs(a.plan,a.sha256,a.scene_observation,a.first)
    if not 1<=a.count<=7 or a.first+a.count>len(plan['stages']):p.error('Group must contain 1 to 7 available stages')
    report=dict(started_utc=datetime.now(timezone.utc).isoformat(),execute=a.execute,first=a.first,count=a.count,
                plan_sha256=a.sha256,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                scene_age_s=age,status='starting',stages=[])
    with a.output.open('x',encoding='utf-8') as out:
        try:
            with ObservationDashboard('192.168.1.4',timeout=.5) as robot:
                for i in range(a.first,a.first+(a.count if a.execute else 1)):
                    stage=plan['stages'][i];one=dict(stage=i,status='starting');report['stages'].append(one)
                    try:
                        one['result']=run(robot,stage,stage['start_q'],one,a.execute);one['status']='completed'
                    except BaseException as exc:one.update(status='failed',error=repr(exc));raise
                    print(json.dumps({k:one[k] for k in ['stage','status','result','queue_id'] if k in one}),flush=True)
                report['status']='completed' if a.execute else 'preview'
        except BaseException as exc:report.update(status='aborted',error=repr(exc))
        report.update(ended_utc=datetime.now(timezone.utc).isoformat(),end_state='Keep enabled; no disable or power-off commands')
        json.dump(report,out,indent=2)
    print(json.dumps({k:report.get(k) for k in ['status','error','source_sha256']},indent=2))
    return int(report['status'] not in ('completed','preview'))

if __name__=='__main__':raise SystemExit(main())
