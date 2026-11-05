"""New-start, reviewed approach to the fixed grasp hover. No auto enabling.

This fixture-specific bridge is separate from the grasp cycles. Default is
offline validation. Explicit execution keeps the original waypoint guards.
"""
import argparse,json,math,signal
from datetime import datetime,timezone
from pathlib import Path
from .e6_calibration_view import validate_stage,rotation_angle,run
from .e6_zero import E6Error
from .e6_observation_move import ObservationDashboard
from .grasp_task import digest
from .grasp_task_plan import read_profile

def validate(plan):
    if plan.get('status')!='sampled_fixed_hover_approach' or not 1<=len(plan.get('stages',[]))<=24:raise E6Error('Require bounded new-start approach')
    oq=plan['origin_q'];op=plan['origin_pose'];goal=read_profile()['reviewed_hover_q'];previous=oq
    if max(abs(a-b) for a,b in zip(oq,[-66.271,-60.7104,105.7138,-103.8043,87.3056,10.6334]))>.05:raise E6Error('Unexpected approach fixture origin')
    if max(abs(a-b) for a,b in zip(goal,plan['target_q']))>1e-6:raise E6Error('Changed hover target')
    for index,stage in enumerate(plan['stages']):
        validate_stage(stage,stage['start_q'])
        if stage['index']!=index or max(abs(a-b) for a,b in zip(previous,stage['start_q']))>1e-5:raise E6Error('Disconnected approach')
        if len(stage['samples'])!=9:raise E6Error('Require nine samples')
        for sample in stage['samples']:
            q=sample['q'];p=sample['pose'];arms=sample['arm_origins_mm']
            if any(not min(a,b)-1e-5<=v<=max(a,b)+1e-5 for v,a,b in zip(q,oq,goal)):raise E6Error('Joint path left approach interval')
            if not (-10<=p[0]<=76 and -374<=p[1]<=-190 and 278<=p[2]<=370) or rotation_angle(p,op)>33:raise E6Error('Approach geometry left reviewed corridor')
            if len(arms)!=6 or any(len(a)!=3 or not all(math.isfinite(v) for v in a) for a in arms):raise E6Error('Invalid model origins')
            if any(math.dist(a,b)>limit for a,b,limit in zip(arms,plan['origin_arm_origins_mm'],[1,1,51,225,240,230])):raise E6Error('Arm-origin sweep exceeded')
        previous=stage['target_q']
    if max(abs(a-b) for a,b in zip(previous,goal))>1e-5:raise E6Error('Incomplete approach')

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--sha256',required=True)
    parser.add_argument('--scene-observation',type=Path);parser.add_argument('--first',type=int,default=0);parser.add_argument('--count',type=int,default=1)
    parser.add_argument('--output',type=Path);parser.add_argument('--execute',action='store_true');a=parser.parse_args()
    if digest(a.plan)!=a.sha256:raise E6Error('Approach plan SHA mismatch')
    plan=json.loads(a.plan.read_text(encoding='utf-8'));validate(plan)
    if not 0<=a.first<len(plan['stages']) or not 1<=a.count<=7 or a.first+a.count>len(plan['stages']):parser.error('Run 1..7 available stages')
    if not a.execute:
        print(json.dumps(dict(status='offline_validated',stages=len(plan['stages']),hardware_connections=0)));return 0
    if a.scene_observation is None or a.output is None:parser.error('Fresh scene observation and new report required')
    obs=json.loads(a.scene_observation.read_text(encoding='utf-8'));camera=Path(obs['source_run'])/'report.json'
    if obs.get('status')!='static_scene' or digest(camera)!=obs['camera_report_sha256']:raise E6Error('Require matching stationary observation')
    report=json.loads(camera.read_text(encoding='utf-8'));age=(datetime.now(timezone.utc)-datetime.fromisoformat(report['ended_at_utc'])).total_seconds()
    if not 0<=age<=180 or report['status']!='completed' or any(report.get(k) for k in ('cleanup_errors','processing_errors','receiver_errors')):raise E6Error('Stale or failed camera capture')
    start=plan['stages'][a.first]['start_q']
    if obs['after']['mode']!=5 or max(abs(a-b) for a,b in zip(start,obs['after']['q']))>.15:raise E6Error('Scene differs from this group start')
    cancelled=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:cancelled.__setitem__(0,True))
    def guard():
        if cancelled[0]:raise E6Error('Approach cancelled')
    result=dict(started_utc=datetime.now(timezone.utc).isoformat(),status='starting',plan_sha256=a.sha256,source_sha256=digest(__file__),first=a.first,count=a.count,stages=[])
    with a.output.open('x',encoding='utf-8') as out:
        try:
            with ObservationDashboard('192.168.1.4',timeout=.5) as robot:
                for i in range(a.first,a.first+a.count):
                    stage=plan['stages'][i];one=dict(stage=i);result['stages'].append(one)
                    one['result']=run(robot,stage,stage['start_q'],one,True,guard=guard)
                    print(json.dumps(dict(stage=i,queue_id=one['queue_id'],status=one['result'])),flush=True)
            result['status']='completed'
        except BaseException as exc:result.update(status='aborted',error=repr(exc))
        result['ended_utc']=datetime.now(timezone.utc).isoformat();json.dump(result,out,indent=2)
    print(json.dumps({k:result.get(k) for k in ('status','error')},indent=2));return int(result['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
