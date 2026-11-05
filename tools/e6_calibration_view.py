"""One bounded waypoint for a reviewed calibration observation path.

Default read-only. No enable/disable, gripper commands, recovery or return.
Plan hash and camera freshness do not certify physical collision clearance.
"""
import argparse,hashlib,json,math,re,time
from datetime import datetime,timezone
from pathlib import Path
try:
    from .e6_zero import Dashboard,E6Error,read_state,validate_joints
    from .e6_step import check_envelope,move_command
except ImportError:
    from e6_zero import Dashboard,E6Error,read_state,validate_joints
    from e6_step import check_envelope,move_command


def validate_stage(stage,origin_q):
    start,target=stage['start_q'],stage['target_q']
    for q in [origin_q,start,target]:validate_joints(q)
    if not 0<max(abs(a-b) for a,b in zip(start,target))<=3.001:
        raise E6Error('Calibration waypoint joint travel must be nonzero and <=3 degrees')
    if max(abs(a-b) for q in [start,target] for a,b in zip(q,origin_q))>18:
        raise E6Error('Waypoint exceeds the original calibration joint window')
    samples=stage['samples']
    if len(samples)<9:raise E6Error('Require at least nine FK samples per segment')
    for j,s in enumerate(samples):
        pose=s['pose'];q=s['q']
        validate_joints(q)
        if len(pose)!=6 or not all(math.isfinite(v) for v in pose):raise E6Error('Invalid FK pose')
        expected=[a+(b-a)*j/(len(samples)-1) for a,b in zip(start,target)]
        if max(abs(a-b) for a,b in zip(q,expected))>1e-5:raise E6Error('FK samples do not cover this joint segment')
    poses=[s['pose'] for s in samples]
    lo=[min(p[i] for p in poses)-1.5 for i in range(3)]
    hi=[max(p[i] for p in poses)+1.5 for i in range(3)]
    return start,target,lo,hi


def rotation_angle(a,b):
    def matrix(p):
        x,y,z=map(math.radians,p[3:]);cx,sx=math.cos(x),math.sin(x);cy,sy=math.cos(y),math.sin(y);cz,sz=math.cos(z),math.sin(z)
        return [cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx]
    trace=sum(x*y for x,y in zip(matrix(a),matrix(b)))
    return math.degrees(math.acos(max(-1,min(1,(trace-1)/2))))


def validate_plan(plan):
    if plan.get('status')!='sampled_preview' or not plan.get('stages'):raise E6Error('Require a sampled calibration preview')
    origin=plan['origin_pose'];q=plan['origin_q']
    if len(origin)!=6 or not all(math.isfinite(v) for v in origin):raise E6Error('Invalid original pose')
    previous=q
    for stage in plan['stages']:
        validate_stage(stage,q)
        if max(abs(a-b) for a,b in zip(stage['start_q'],previous))>1e-5:raise E6Error('Path is not connected from original start')
        for s in stage['samples']:
            p=s['pose']
            if max(abs(a-b) for a,b in zip(p[:2],origin[:2]))>3 or not -3<=p[2]-origin[2]<=25:
                raise E6Error('Calibration path exceeds original XYZ window')
            if rotation_angle(p,origin)>8:raise E6Error('Calibration orientation exceeds 8 degrees')
        previous=stage['target_q']


def run(robot,stage,origin_q,report,execute=False,clock=time.monotonic,sleep=time.sleep,guard=None):
    start,target,lo,hi=validate_stage(stage,origin_q)
    report.update(expected_start=start,target=target,xyz_envelope_mm=[lo,hi],states=[],command_log=[],motion_command=move_command(target))
    def command(cmd):
        entry={'command':cmd,'monotonic_ns':time.monotonic_ns()};report['command_log'].append(entry)
        entry['payload']=robot.command(cmd);return entry['payload']
    def sample():
        if guard is not None:guard()
        t=clock();s=read_state(robot,clock)
        pose=[float(x) for x in command('GetPose(user=0,tool=0)').split(',')]
        if clock()-t>1 or len(pose)!=6 or not all(math.isfinite(v) for v in pose):raise E6Error('Invalid or stale complete state/pose')
        if guard is not None:guard()
        report['states'].append({'monotonic_ns':time.monotonic_ns(),'mode':s.mode,'joints_deg':list(s.joints),'pose':pose})
        return s,pose
    def require_start(s,p):
        if s.mode!=5 or max(abs(a-b) for a,b in zip(s.joints,start))>.15:raise E6Error('Require enabled idle robot at reviewed start')
        if any(not a<=x<=b for a,x,b in zip(lo,p[:3],hi)):raise E6Error('Start outside reviewed Cartesian envelope')
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
            if any(not a<=x<=b for a,x,b in zip(lo,p[:3],hi)):raise E6Error('Motion outside sampled FK Cartesian envelope')
            error=max(abs(a-b) for a,b in zip(s.joints,target))
            stable=stable+1 if s.mode==5 and error<=.05 else 0
            if stable>=5:
                report.update(motion_elapsed_s=clock()-sent,arrival_max_error_deg=error);return 'reached'
            if clock()-sent>30:raise E6Error('Arrival timeout')
            sleep(.1)
    except BaseException:
        try:command('Stop()');report['stop_acknowledged']=True
        except BaseException as exc:report['stop_error']=repr(exc)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--stage',type=int,required=True);p.add_argument('--scene-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
    a=p.parse_args();raw=a.plan.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=a.sha256:p.error('Plan hash mismatch')
    plan=json.loads(raw)
    validate_plan(plan)
    if not 0<=a.stage<len(plan['stages']):p.error('Invalid waypoint')
    stage=plan['stages'][a.stage];validate_stage(stage,plan['origin_q'])
    scene=json.loads(a.scene_report.read_text(encoding='utf-8'))
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:
        p.error('Require a successful camera capture within 180 seconds')
    with a.output.open('x',encoding='utf-8') as out:
        report={'started_utc':datetime.now(timezone.utc).isoformat(),'execute':a.execute,'stage':a.stage,'plan_sha256':a.sha256,
                'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scene_age_s':age,'status':'starting'}
        try:
            with Dashboard('192.168.1.4',timeout=.5) as robot:report['result']=run(robot,stage,plan['origin_q'],report,a.execute)
            report['status']='completed'
        except BaseException as exc:report.update(status='failed',error=repr(exc))
        report['ended_utc']=datetime.now(timezone.utc).isoformat();json.dump(report,out,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ['states','command_log']},indent=2))
    return int(report['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
