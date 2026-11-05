"""Execute one reviewed joint waypoint of the fixed look-down plan.

Default read-only. Caller must establish physical swept-volume clearance.
No automatic enable/disable, recovery, return, gripper command or parameter write.
"""
import argparse
from datetime import datetime,timezone
import hashlib,json,math,re,time
from pathlib import Path
try:
    from .e6_zero import Dashboard,E6Error,read_state,validate_joints
    from .e6_step import check_envelope,move_command
except ImportError:
    from e6_zero import Dashboard,E6Error,read_state,validate_joints
    from e6_step import check_envelope,move_command

PLAN_SHA256='7b7e27e966ec4563bd3d19ad11f135c2e8b9ae8b852e38c5e94dc82df58be935'
FULL_BOTTLE_PLAN_SHA256='cdf36f89cb50d45bbf9b565c54e49ac1f461c0425dd0d796e20c5d4abc41be72'
def validate_stage(stage):
    start,target=stage['start_q'],stage['target_q']
    validate_joints(start);validate_joints(target)
    if not 0<max(abs(a-b) for a,b in zip(start,target))<=13:
        raise E6Error('Waypoint must have a nonzero joint change <=13 deg')
    if not -15<=stage['pitch_delta_deg']<0 or not 0<stage['lift_mm']<=50:
        raise E6Error('Waypoint outside reviewed look-down plan')
    poses=[s['pose'] for s in stage['samples']]
    if len(poses)<9 or any(len(p)!=6 or not all(math.isfinite(v) for v in p) for p in poses):
        raise E6Error('Invalid FK sample envelope')
    lo=[min(p[i] for p in poses)-1.5 for i in range(3)]
    hi=[max(p[i] for p in poses)+1.5 for i in range(3)]
    return start,target,lo,hi

def run(robot,stage,report,execute=False,clock=time.monotonic,sleep=time.sleep):
    start,target,lo,hi=validate_stage(stage)
    report.update(expected_start=start,target=target,xyz_envelope_mm=[lo,hi],states=[],command_log=[],motion_command=move_command(target))
    def command(cmd):
        entry={'command':cmd,'monotonic_ns':time.monotonic_ns()};report['command_log'].append(entry)
        entry['payload']=robot.command(cmd);return entry['payload']
    def sample():
        t=clock();s=read_state(robot,clock)
        pose=[float(x) for x in command('GetPose(user=0,tool=0)').split(',')]
        if clock()-t>1:raise E6Error('Complete state/pose query exceeded 1 second')
        if len(pose)!=6 or not all(math.isfinite(x) for x in pose):raise E6Error('Invalid Cartesian readback')
        report['states'].append({'monotonic_ns':time.monotonic_ns(),'mode':s.mode,'joints_deg':list(s.joints),'pose':pose})
        return s,pose
    def require_start(s,p):
        if s.mode!=5 or max(abs(a-b) for a,b in zip(s.joints,start))>.15:
            raise E6Error('Requires enabled idle robot at reviewed start')
        if any(not a<=x<=b for a,x,b in zip(lo,p[:3],hi)):raise E6Error('Start Cartesian pose outside reviewed envelope')
    require_start(*sample())
    if not execute:return 'preview'
    require_start(*sample())
    try:
        qid=command(move_command(target))
        if not re.fullmatch(r'\d+',qid):raise E6Error('Invalid queue ID')
        report['queue_id']=int(qid);sent=clock();stable=0
        while True:
            s,p=sample()
            if s.mode not in (5,7):raise E6Error(f'Unexpected mode {s.mode}')
            check_envelope(start,target,s.joints)
            if any(not a<=x<=b for a,x,b in zip(lo,p[:3],hi)):raise E6Error('Cartesian path outside FK sample envelope')
            error=max(abs(a-b) for a,b in zip(s.joints,target))
            stable=stable+1 if s.mode==5 and error<=.05 else 0
            if stable>=5:
                report.update(motion_elapsed_s=clock()-sent,arrival_max_error_deg=error);return 'reached'
            if clock()-sent>30:raise E6Error('30 second arrival timeout')
            sleep(.1)
    except BaseException:
        try:command('Stop()');report['stop_acknowledged']=True
        except BaseException as exc:report['stop_error']=repr(exc)
        raise

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--stage',type=int,choices=range(1,7),required=True)
    p.add_argument('--scene-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    raw=a.plan.read_bytes()
    plan_hash=hashlib.sha256(raw).hexdigest()
    if plan_hash not in (PLAN_SHA256,FULL_BOTTLE_PLAN_SHA256):p.error('Plan differs from reviewed fixed paths')
    stages=json.loads(raw)['stages']
    if a.stage>=len(stages):p.error('Waypoint does not exist in this fixed path')
    stage=stages[a.stage];validate_stage(stage)
    scene=json.loads(a.scene_report.read_text(encoding='utf-8'))
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:
        p.error('Requires successful camera capture within 180 seconds')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf-8') as out:
        report={'started_utc':datetime.now(timezone.utc).isoformat(),'execute':a.execute,'stage':a.stage,'plan_sha256':plan_hash,'scene_age_s':age,'status':'starting','source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        try:
            with Dashboard('192.168.1.4',timeout=.5) as robot:report['result']=run(robot,stage,report,a.execute)
            report['status']='completed'
        except BaseException as exc:report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat();json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ['states','command_log']},indent=2))
    return int(report['status']!='completed')
if __name__=='__main__':raise SystemExit(main())
