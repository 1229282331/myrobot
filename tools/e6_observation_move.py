"""Explicit, bounded raised-flange camera observation from a reviewed plan.

Defaults to read-only. No gripper, recovery, enable or controller setting writes.
Model arm-origin sweeps are estimates, not collision certification.
"""
import argparse,hashlib,json,math
from datetime import datetime,timezone
from pathlib import Path
try:
    from .e6_zero import Dashboard,E6Error
    from .e6_calibration_view import validate_stage,rotation_angle,run
except ImportError:
    from e6_zero import Dashboard,E6Error
    from e6_calibration_view import validate_stage,rotation_angle,run

def validate_plan(plan):
    if plan.get('status')!='sampled_raised_observation' or not plan.get('stages'):
        raise E6Error('Require a sampled raised observation plan')
    origin=plan['origin_pose'];oq=plan['origin_q'];origins=plan['origin_arm_origins_mm']
    if len(origin)!=6 or not all(math.isfinite(x) for x in origin) or origin[2]<370:
        raise E6Error('Observation start must be at least 370 mm high')
    if len(origins)!=6 or any(len(p)!=3 or not all(math.isfinite(x) for x in p) for p in origins):
        raise E6Error('Missing model origin samples')
    previous=oq
    for stage in plan['stages']:
        validate_stage(stage,stage['start_q'])
        if max(abs(a-b) for a,b in zip(previous,stage['start_q']))>1e-5:
            raise E6Error('Disconnected path')
        for s in stage['samples']:
            if max(abs(a-b) for a,b in zip(s['q'],oq))>35:
                raise E6Error('Observation joint window exceeded')
            p=s['pose']
            if max(abs(a-b) for a,b in zip(p[:2],origin[:2]))>2 or not -1<=p[2]-origin[2]<=40:
                raise E6Error('Raised observation XYZ window exceeded')
            if rotation_angle(p,origin)>25:
                raise E6Error('Observation rotation exceeded')
            arms=s['arm_origins_mm']
            if len(arms)!=6 or any(len(v)!=3 or not all(math.isfinite(x) for x in v) for v in arms):
                raise E6Error('Invalid model samples')
            if max(math.dist(a,b) for a,b in zip(arms,origins))>150:
                raise E6Error('Model origin sweep exceeds 150 mm')
        previous=stage['target_q']

class ObservationDashboard(Dashboard):
    def command(self,command,*,timeout=None):
        return super().command(command,timeout=8 if command=='Stop()' else timeout)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--stage',type=int,required=True);p.add_argument('--scene-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    raw=a.plan.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=a.sha256:p.error('Plan hash mismatch')
    plan=json.loads(raw);validate_plan(plan)
    if not 0<=a.stage<len(plan['stages']):p.error('Waypoint absent')
    scene=json.loads(a.scene_report.read_text(encoding='utf-8'))
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','receiver_errors','processing_errors']) or not 0<=age<=180:
        p.error('Require a clean camera capture within 180 seconds')
    stage=plan['stages'][a.stage]
    with a.output.open('x',encoding='utf-8') as out:
        report=dict(started_utc=datetime.now(timezone.utc).isoformat(),execute=a.execute,stage=a.stage,plan_sha256=a.sha256,
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scene_age_s=age,status='starting')
        try:
            with ObservationDashboard('192.168.1.4',timeout=.5) as robot:report['result']=run(robot,stage,stage['start_q'],report,a.execute)
            report['status']='completed'
        except BaseException as exc:report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat();json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ['states','command_log']},indent=2))
    return int(report['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
