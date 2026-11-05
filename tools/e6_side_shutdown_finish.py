"""New actual stopped start to configured standby. Default read-only; no contact or recovery."""
import argparse,hashlib,json,math,signal,sys
from datetime import datetime,timezone
from pathlib import Path
from .e6_zero import E6Error,JOINT_BOUNDS_DEG
from .e6_calibration_view import validate_stage,run
from .e6_observation_move import ObservationDashboard
from .e6_step import move_command
from .grasp_task import Journal,source_manifest
from .grasp_task_hardware import HardwareBackend,RecordingRobot,RecordingChannel
from omnipicker.tools import grasp_can as grip

def validate_plan(plan):
    if plan.get('status')!='sampled_side_shutdown_standby' or len(plan.get('stages',[]))!=33:raise E6Error('Unexpected new-start standby path')
    origin=[-34.3597,-51.351,22.3261,-75.9322,46.4157,103.6757]
    goal=[-66.5019,-60,105,-90,90,9.1974]
    if max(abs(x-y) for x,y in zip(plan['origin_q'],origin))>.03:raise E6Error('Unexpected stopped origin')
    previous=plan['origin_q']
    for i,s in enumerate(plan['stages']):
        validate_stage(s,s['start_q'])
        if s['index']!=i or len(s['samples'])!=9 or max(abs(x-y) for x,y in zip(s['start_q'],previous))>1e-5:raise E6Error('Disconnected standby path')
        for v in s['samples']:
            q,p,arms=v['q'],v['pose'],v['arm_origins_mm']
            if not (-10<=p[0]<=186 and -286<=p[1]<=-186 and 346<=p[2]<=403):raise E6Error('Standby flange bounds exceeded')
            if p[2]+22-170<198 or any(abs(x)>lim-40 for x,lim in zip(q,JOINT_BOUNDS_DEG)):raise E6Error('Standby table or joint margin exceeded')
            if any(not min(a,b)-.03<=v<=max(a,b)+.03 for a,b,v in zip(origin,goal,q)):raise E6Error('Standby joint interval exceeded')
            if len(arms)!=6 or any(len(a)!=3 or not all(math.isfinite(x) for x in a) for a in arms):raise E6Error('Invalid model origins')
            if any(math.dist(a,b)>lim for a,b,lim in zip(arms,plan['origin_arm_origins_mm'],[1,28,94,222,260,240])):raise E6Error('Standby sweep exceeded')
        previous=s['target_q']
    if max(abs(x-y) for x,y in zip(previous,goal))>.03:raise E6Error('Incomplete standby path')

def load(plan_path,sha,scene_path,first):
    raw=plan_path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=sha:raise E6Error('Plan hash differs')
    plan=json.loads(raw);validate_plan(plan)
    if not 0<=first<len(plan['stages']):raise E6Error('Invalid stage')
    obs=json.loads(scene_path.read_text());rraw=(Path(obs['source_run'])/'report.json').read_bytes();r=json.loads(rraw)
    if hashlib.sha256(rraw).hexdigest()!=obs['camera_report_sha256']:raise E6Error('Scene report hash differs')
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(r['ended_at_utc'])).total_seconds()
    if obs.get('status')!='static_scene' or not 0<=age<=180 or r.get('status')!='completed' or any(r.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']):raise E6Error('Fresh stationary scene required')
    if r['device']['serial_number']!='AY4A1520111' or r['uvc_pts_availability']['missing_color_frames'] or r['color_decoded_frames']!=r['streams']['color']['frames'] or any(v['index_gaps'] or v['index_nonmonotonic_count'] for v in r['streams'].values()):raise E6Error('Camera integrity differs')
    for state in [obs['before'],obs['after']]:
        if state['mode']!=5 or max(abs(x-y) for x,y in zip(state['q'],plan['stages'][first]['start_q']))>.15:raise E6Error('Scene not at entry stage')
    return plan,age

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True);p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--first',type=int,required=True);p.add_argument('--count',type=int,required=True);p.add_argument('--source-pins',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    plan,age=load(a.plan,a.sha256,a.scene,a.first)
    if (a.first,a.count)!=(21,12):raise E6Error('Only the final twelve reviewed standby stages are allowed')
    pins=json.loads(a.source_pins.read_text())
    if source_manifest()!=pins['runtime'] or hashlib.sha256(Path(__file__).read_bytes()).hexdigest()!=pins['executor']:raise E6Error('Runtime source pins changed')
    if not a.execute:print(json.dumps(dict(status='preview',first=a.first,count=a.count,scene_age_s=age)));return 0
    if sys.platform!='linux':raise E6Error('X5 Linux execution only')
    import fcntl
    j=Journal(a.output,'side_shutdown_park');hw=HardwareBackend(j);hw.expected_q=plan['stages'][a.first]['start_q']
    result=dict(status='starting',started_utc=datetime.now(timezone.utc).isoformat(),plan_sha256=a.sha256,source_pins=pins,scene_age_s=age,first=a.first,count=a.count,stages=[])
    try:
        for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:
            hw.previous_handlers[sig]=signal.getsignal(sig);signal.signal(sig,lambda *_:setattr(hw,'cancelled',True))
        hw.raw_robot=ObservationDashboard('192.168.1.4',timeout=.5);hw.raw_robot.__enter__()
        hw.robot=RecordingRobot(hw.raw_robot,j,[move_command(s['target_q']) for s in plan['stages'][a.first:a.first+a.count]])
        result['before']=hw.snapshot()
        hw.lock=open('/tmp/omnipicker-can1.lock','a');fcntl.flock(hw.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ident=grip.can.validate_usb_interface('can1');before=grip.can.ip_snapshot('can1')
        if 'UP' in before['flags']:raise E6Error('CAN owner already present')
        hw.guard();hw.configured=True;grip.can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off');grip.can.configure('can1','up')
        state=grip.can.ip_snapshot('can1')
        if state['linkinfo']['info_data']['bittiming']['bitrate']!=1000000:raise E6Error('CAN bitrate mismatch')
        hw.raw_channel=grip.can.CanChannel('can1',1,.3,hw.transport_report,allow_stalled=True);hw.channel=RecordingChannel(hw.raw_channel,j)
        j.write('can_initialized',identity=ident,before=before,initialized=state)
        j.enter('opening_guard',1);hw.start_open_monitor();result['opening_verified']=hw.snapshot()
        for i in range(a.first,a.first+a.count):
            j.enter('standby_return',1);one=dict(stage=i,status='starting');result['stages'].append(one)
            one['result']=run(hw.robot,plan['stages'][i],plan['stages'][i]['start_q'],one,True,guard=hw.guard)
            hw.expected_q=plan['stages'][i]['target_q'];one['status']='completed';j.artifact(f'stages/{i:03d}.json',one)
            print(json.dumps(dict(stage=i,status=one['status'],queue_id=one['queue_id'])),flush=True)
        result['after']=hw.snapshot();result['status']='completed'
    except BaseException as exc:
        result.update(status='aborted',error=repr(exc))
        try:result['stop']=hw.stop_once()
        except BaseException as e:result['stop_error']=repr(e)
    finally:
        if hw.monitor is not None:result['opening_feedback']=hw.monitor.report
        try:result['cleanup']=hw.close()
        except BaseException as e:result.update(status='cleanup_unconfirmed',cleanup_error=repr(e))
        result['ended_utc']=datetime.now(timezone.utc).isoformat();j.artifact('summary.json',result);j.close()
    print(json.dumps({k:result.get(k) for k in ['status','error','after']},indent=2));return int(result['status']!='completed')

if __name__=='__main__':raise SystemExit(main())
