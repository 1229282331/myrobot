"""One reviewed noncontact approach waypoint, ending above the bottle.

Default read-only. Flange stays >=270mm; 170mm tool envelope is a
conservative nominal allowance, not a measured TCP or collision certificate.
"""
import argparse,hashlib,json,math
from datetime import datetime,timezone
from pathlib import Path
try:
    from .e6_zero import E6Error
    from .e6_calibration_view import validate_stage,rotation_angle,run
    from .e6_observation_move import ObservationDashboard
except ImportError:
    from e6_zero import E6Error
    from e6_calibration_view import validate_stage,rotation_angle,run
    from e6_observation_move import ObservationDashboard

def validate_plan(plan):
    if plan.get('status')!='sampled_noncontact_hover' or not plan.get('stages'):
        raise E6Error('Require a sampled noncontact hover plan')
    oq=plan['origin_q'];op=plan['origin_pose'];arms0=plan['origin_arm_origins_mm'];previous=oq
    if len(op)!=6 or not all(math.isfinite(v) for v in op) or len(arms0)!=6:
        raise E6Error('Invalid origin')
    for stage in plan['stages']:
        validate_stage(stage,stage['start_q'])
        if max(abs(a-b) for a,b in zip(previous,stage['start_q']))>1e-5:raise E6Error('Disconnected path')
        for s in stage['samples']:
            p=s['pose'];q=s['q'];arms=s['arm_origins_mm']
            if max(abs(a-b) for a,b in zip(q,oq))>80:raise E6Error('Joint window exceeded')
            if not (50<=p[0]<=115 and -390<=p[1]<=-180 and 270-0.01<=p[2]<=415):raise E6Error('Hover XYZ window exceeded')
            if rotation_angle(p,op)>36:raise E6Error('Hover orientation window exceeded')
            if len(arms)!=6 or any(len(a)!=3 or not all(math.isfinite(v) for v in a) for a in arms):raise E6Error('Invalid model origin samples')
            if any(math.dist(a,b)>limit for a,b,limit in zip(arms,arms0,[1,50,150,260,260,260])):raise E6Error('Model arm-origin sweep exceeded')
        previous=stage['target_q']

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--stage',type=int,required=True);p.add_argument('--scene-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    raw=a.plan.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=a.sha256:p.error('Plan hash mismatch')
    plan=json.loads(raw);validate_plan(plan)
    if not 0<=a.stage<len(plan['stages']):p.error('Stage absent')
    scene=json.loads(a.scene_report.read_text());age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:p.error('Require clean camera capture within 180 seconds')
    stage=plan['stages'][a.stage]
    with a.output.open('x',encoding='utf-8') as out:
        report=dict(started_utc=datetime.now(timezone.utc).isoformat(),execute=a.execute,stage=a.stage,plan_sha256=a.sha256,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scene_age_s=age,status='starting')
        try:
            with ObservationDashboard('192.168.1.4',timeout=.5) as robot:report['result']=run(robot,stage,stage['start_q'],report,a.execute)
            report['status']='completed'
        except BaseException as exc:report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat();json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ['states','command_log']},indent=2))
    return int(report['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
