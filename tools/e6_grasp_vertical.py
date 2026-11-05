"""Bounded vertical arm stages with independent 20 Hz gripper guarding.

Default preview is read-only. Explicit execution is Linux-only, with one
Dashboard connection. No recovery, enable, controller settings or release.
The 170 mm tool envelope and table height are local nominal estimates.
"""
import argparse,hashlib,json,math,signal,sys
from datetime import datetime,timezone
from pathlib import Path
try:
    from .e6_zero import E6Error
    from .e6_calibration_view import validate_stage,rotation_angle,run
    from .e6_observation_move import ObservationDashboard
    from .grip_monitor import OpeningMonitor,HoldingMonitor,grip
except ImportError:
    from e6_zero import E6Error
    from e6_calibration_view import validate_stage,rotation_angle,run
    from e6_observation_move import ObservationDashboard
    from grip_monitor import OpeningMonitor,HoldingMonitor,grip

def validate_plan(plan,*,max_z_mm=285.01):
    if plan.get('status')!='sampled_vertical_grasp' or not 1<=len(plan.get('stages',[]))<=32:
        raise E6Error('Require a bounded sampled vertical plan')
    oq=plan['origin_q'];op=plan['origin_pose'];arms0=plan['origin_arm_origins_mm'];previous=oq
    if len(op)!=6 or not all(math.isfinite(v) for v in op) or len(arms0)!=6:
        raise E6Error('Invalid origin')
    if rotation_angle(op,[0,0,0,-180,0,op[5]])>.5:
        raise E6Error('Require downward flange')
    for stage in plan['stages']:
        validate_stage(stage,stage['start_q'])
        if max(abs(a-b) for a,b in zip(previous,stage['start_q']))>1e-5:
            raise E6Error('Disconnected path')
        for s in stage['samples']:
            p=s['pose'];q=s['q'];arms=s['arm_origins_mm']
            if max(abs(a-b) for a,b in zip(q,oq))>45:
                raise E6Error('Joint window exceeded')
            if not (50<=p[0]<=100 and -390<=p[1]<=-330 and 165-.01<=p[2]<=max_z_mm):
                raise E6Error('Vertical XYZ window exceeded')
            if max(abs(a-b) for a,b in zip(p[:2],op[:2]))>1 or abs(p[2]-op[2])>45.01 or rotation_angle(p,op)>.5:
                raise E6Error('Vertical path departed from reviewed axis or height')
            if len(arms)!=6 or any(len(a)!=3 or not all(math.isfinite(v) for v in a) for a in arms):
                raise E6Error('Invalid model samples')
            if any(math.dist(a,b)>limit for a,b,limit in zip(arms,arms0,[1,50,100,100,100,100])):
                raise E6Error('Model arm-origin sweep exceeded')
        previous=stage['target_q']

def load_inputs(path,digest,scene_path):
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=digest:raise E6Error('Plan hash mismatch')
    plan=json.loads(raw);validate_plan(plan)
    scene=json.loads(scene_path.read_text())
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(scene['ended_at_utc'])).total_seconds()
    if scene.get('status')!='completed' or any(scene.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=180:
        raise E6Error('Require clean camera capture within 180 seconds')
    return plan,age

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--scene-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['open','hold'],required=True);p.add_argument('--opening',type=float,default=60)
    p.add_argument('--contact',type=int);p.add_argument('--grip-target',type=float,default=0);p.add_argument('--force-raw',type=int,choices=[32,64],default=32);p.add_argument('--execute',action='store_true');a=p.parse_args()
    plan,age=load_inputs(a.plan,a.sha256,a.scene_report)
    if a.mode=='open' and (not math.isfinite(a.opening) or not 15<=a.opening<=60):p.error('Invalid opening')
    if a.mode=='hold':grip.HoldGate(a.contact)
    if not math.isfinite(a.grip_target) or not 0<=a.grip_target<=10:p.error('Invalid grip target')
    if a.execute and sys.platform!='linux':p.error('Execute on X5 only')
    r=dict(started_utc=datetime.now(timezone.utc).isoformat(),status='starting',execute=a.execute,mode=a.mode,opening=a.opening,contact=a.contact,grip_target=a.grip_target,force_raw=a.force_raw,scene_age_s=age,plan_sha256=a.sha256,events=[],stages=[],source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    cancel=[False];channel=None;configured=False;monitor=None
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,lambda *_:cancel.__setitem__(0,True))
    def stopped():return cancel[0] or Path('/root/omnipicker-can/STOP').exists()
    with a.output.open('x',encoding='utf-8') as out:
        try:
            if not a.execute:
                with ObservationDashboard('192.168.1.4',timeout=.5) as robot:
                    r['result']=run(robot,plan['stages'][0],plan['stages'][0]['start_q'],{},False)
                r['status']='preview'
            else:
                import fcntl
                with open('/tmp/omnipicker-can1.lock','a') as lock:
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    if stopped():raise E6Error('Stop requested')
                    r['identity']=grip.can.validate_usb_interface('can1');r['before']=grip.can.ip_snapshot('can1')
                    if 'UP' in r['before']['flags']:raise E6Error('CAN already in use')
                    configured=True
                    grip.can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off')
                    grip.can.configure('can1','up');r['initialized']=grip.can.ip_snapshot('can1')
                    if r['initialized']['linkinfo']['info_data']['bittiming']['bitrate']!=1000000:raise E6Error('Bitrate mismatch')
                    channel=grip.can.CanChannel('can1',1,.3,r,allow_stalled=a.mode=='hold')
                    monitor=OpeningMonitor(channel,a.opening,r,stopped,force_raw=a.force_raw) if a.mode=='open' else HoldingMonitor(channel,a.contact,r,stopped,target_percent=a.grip_target,force_raw=a.force_raw)
                    monitor.start()
                    with ObservationDashboard('192.168.1.4',timeout=.5) as robot:
                        for i,stage in enumerate(plan['stages']):
                            one=dict(stage=i,status='starting');r['stages'].append(one)
                            try:
                                one['result']=run(robot,stage,stage['start_q'],one,True,guard=monitor.guard)
                                one['status']='completed'
                            except BaseException as exc:one.update(status='failed',error=repr(exc));raise
                            print(json.dumps({'stage':i,'queue_id':one['queue_id'],'status':'completed'}),flush=True)
                        monitor.guard()
                    r['status']='completed'
        except BaseException as exc:r.update(status='aborted',error=repr(exc))
        finally:
            if monitor:
                try:monitor.stop()
                except BaseException as exc:r.update(status='cleanup_unconfirmed',monitor_cleanup_error=repr(exc))
            if channel:channel.close()
            if configured:
                try:r['before_down']=grip.can.ip_snapshot('can1');grip.can.configure('can1','down');r['after']=grip.can.ip_snapshot('can1')
                except BaseException as exc:r.update(status='cleanup_unconfirmed',can_cleanup_error=repr(exc))
            r['ended_utc']=datetime.now(timezone.utc).isoformat();r['motor_state']='24V remains powered; CAN DOWN is not motor disable'
            json.dump(r,out,indent=2)
    print(json.dumps({k:r.get(k) for k in ['status','error','source_sha256']},indent=2))
    return int(r['status'] not in ('completed','preview'))

if __name__=='__main__':raise SystemExit(main())
