"""X5 task backend. All motion is protected by existing reviewed executors."""
from __future__ import annotations
import hashlib,json,math,os,signal,subprocess,sys,time
from datetime import datetime,timezone
from pathlib import Path
from .e6_zero import E6Error,read_state
from .e6_observation_move import ObservationDashboard
from .e6_calibration_view import run
from .grip_monitor import OpeningMonitor,HoldingMonitor
from . import grasp_task_vision as vision
from .grasp_task import validate_bundle,validate_execution_review
from .e6_step import move_command
from omnipicker.tools import grasp_can as grip

class RecordingRobot:
    def __init__(self,robot,journal,allowed_moves=()):
        self.robot=robot;self.journal=journal;self.allowed_moves=set(allowed_moves)
        self.stop_attempted=False;self.stop_result=None;self.stop_error=None;self.cache={}
    def command(self,cmd,**kwargs):
        if cmd=='Stop()':
            if self.stop_attempted:
                if self.stop_error is not None:raise E6Error('Previous Stop acknowledgement unconfirmed: '+repr(self.stop_error))
                return self.stop_result
            self.stop_attempted=True
            try:self.stop_result=self.robot.command(cmd,**kwargs)
            except BaseException as exc:self.stop_error=exc;raise
            finally:
                try:self.journal.write('robot_stop',acknowledged=self.stop_result is not None)
                except BaseException:pass
            return self.stop_result
        if cmd not in {'RobotMode()','GetAngle()','GetErrorID()','GetPose(user=0,tool=0)'}|self.allowed_moves:
            raise E6Error('Command outside the reviewed task allowlist')
        self.journal.write('robot_command',command=cmd)
        started=time.monotonic_ns();payload=self.robot.command(cmd,**kwargs)
        self.journal.write('robot_reply',command=cmd,payload=payload,request_monotonic_ns=started)
        if cmd=='RobotMode()':self.cache={'mode':int(payload),'started_ns':started}
        elif cmd=='GetAngle()':self.cache['q']=[float(v) for v in payload.split(',')]
        elif cmd=='GetErrorID()':self.cache['errors']=json.loads(payload)
        elif cmd.startswith('GetPose') and all(k in self.cache for k in ['mode','q','errors']):
            self.journal.write('robot_state',robot_mode=self.cache['mode'],joints_deg=self.cache['q'],pose=[float(v) for v in payload.split(',')],alarms=self.cache['errors'],query_span_ns=time.monotonic_ns()-self.cache['started_ns'])
        return payload

class RecordingChannel:
    def __init__(self,channel,journal):self.channel=channel;self.journal=journal
    def exchange(self,command):
        self.journal.write('gripper_command',command_hex=command.hex())
        f=self.channel.exchange(command);self.journal.write('gripper_feedback',feedback=f);return f

class HardwareBackend:
    def __init__(self,journal):
        self.journal=journal;self.bundle=None;self.raw_robot=None;self.robot=None;self.raw_channel=None;self.channel=None;self.monitor=None
        self.lock=None;self.configured=False;self.cancelled=False;self.child=None;self.contact_position=None;self.expected_q=None;self.camera_number=0
        self.transport_report={'events':[]};self.previous_handlers={}
    def guard(self):
        if self.cancelled or Path('/root/omnipicker-can/STOP').exists():raise E6Error('Cancellation or STOP file')
        if self.monitor is not None:self.monitor.guard()
    def snapshot(self,target=None):
        self.guard();started=time.monotonic();s=read_state(self.robot);pose=[float(v) for v in self.robot.command('GetPose(user=0,tool=0)').split(',')]
        if time.monotonic()-started>1:raise E6Error('Stale complete task state')
        if len(pose)!=6 or not all(math.isfinite(x) for x in pose):raise E6Error('Invalid complete task pose')
        target=self.expected_q if target is None else target
        if s.mode!=5 or target is not None and max(abs(a-b) for a,b in zip(s.joints,target))>.15:raise E6Error('Robot is not enabled and idle at the reviewed task state')
        self.guard();return dict(mode=s.mode,q=list(s.joints),pose=pose)
    def preflight(self,bundle):
        if sys.platform!='linux':raise E6Error('Hardware execution is X5/Linux only')
        validate_bundle(bundle);validate_execution_review(bundle)
        self.bundle=bundle;self.expected_q=bundle['hover_q']
        for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:
            self.previous_handlers[sig]=signal.getsignal(sig);signal.signal(sig,lambda *_:setattr(self,'cancelled',True))
        camera=bundle['camera'];path=Path(camera['tools_dir'])/'rdk_healthcheck.py'
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=camera['healthcheck_sha256']:raise E6Error('Camera runtime hash differs from prepared preview')
        if hashlib.sha256(Path(camera['sdk_config']).read_bytes()).hexdigest()!=camera['sdk_config_sha256']:raise E6Error('Camera configuration changed after preview')
        self.raw_robot=ObservationDashboard('192.168.1.4',timeout=.5);self.raw_robot.__enter__()
        self.robot=RecordingRobot(self.raw_robot,self.journal,[move_command(s['target_q']) for p in bundle['plans'].values() for s in p['stages']])
        self.snapshot()
        import fcntl
        self.lock=open('/tmp/omnipicker-can1.lock','a');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        identity=grip.can.validate_usb_interface('can1');before=grip.can.ip_snapshot('can1')
        if 'UP' in before['flags']:raise E6Error('CAN already in use; no takeover')
        self.guard();self.configured=True
        grip.can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off');grip.can.configure('can1','up')
        state=grip.can.ip_snapshot('can1')
        if state['linkinfo']['info_data']['bittiming']['bitrate']!=1000000:raise E6Error('CAN bitrate mismatch')
        self.raw_channel=grip.can.CanChannel('can1',1,.3,self.transport_report,allow_stalled=True);self.channel=RecordingChannel(self.raw_channel,self.journal)
        self.journal.write('can_initialized',identity=identity,before=before,initialized=state)
        return dict(status='passed',mode=5,can_bitrate=1000000,enable_commands=0)
    def stop_monitor(self):
        if self.monitor is not None:
            self.monitor.stop();self.monitor=None
    def start_open_monitor(self):
        report={};self.monitor=OpeningMonitor(self.channel,60,report,lambda:self.cancelled,force_raw=64);self.monitor.start()
    def start_hold_monitor(self):
        report={};self.monitor=HoldingMonitor(self.channel,self.contact_position,report,lambda:self.cancelled,target_percent=10,force_raw=64);self.monitor.start()
    def gripper_stage(self,action,*,start=None):
        self.snapshot();self.stop_monitor();report={}
        last=[time.monotonic()]
        def stopped():
            self.guard()
            if time.monotonic()-last[0]>=.25:self.snapshot();last[0]=time.monotonic()
            return False
        try:
            result=grip.run_stage(self.channel,action,10 if action=='grasp' else 60,report,start_position=start,seconds=6,force_raw=64,stopped=stopped)
            return result
        except BaseException:self.stop_once();raise
        finally:self.journal.artifact(f'stages/c{self.journal.cycle}_{self.journal.phase}_gripper.json',report)
    def open(self):
        result=self.gripper_stage('open');self.start_open_monitor();return result
    def grasp(self):
        self.guard()
        if self.monitor is None or not isinstance(self.monitor,OpeningMonitor):raise E6Error('Verified opening required before grasp')
        start=self.monitor.gate.target;result=self.gripper_stage('grasp',start=start);self.contact_position=result['position_raw'];self.start_hold_monitor();return result
    def release(self):
        self.guard()
        if self.monitor is None or not isinstance(self.monitor,HoldingMonitor) or isinstance(self.monitor,OpeningMonitor):raise E6Error('Verified holding required before release')
        result=self.gripper_stage('release',start=self.contact_position);self.start_open_monitor();return result
    def move(self,name,plan):
        self.guard();reports=[]
        for i,stage in enumerate(plan['stages']):
            one=dict(stage=i,status='starting');reports.append(one)
            try:
                one['result']=run(self.robot,stage,stage['start_q'],one,True,guard=self.guard);one['status']='completed'
                self.expected_q=stage['target_q']
            except BaseException as exc:one.update(status='failed',error=repr(exc));self.stop_once();raise
            finally:self.journal.artifact(f'stages/c{self.journal.cycle}_{name}_{i:03d}.json',one)
            if name=='park' and (i+1)%7==0:self.observe('park')
        return dict(status='reached',stages=len(reports),last_queue_id=reports[-1]['queue_id'])
    def stop_capture(self):
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:self.child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.child.kill();self.child.wait(timeout=2);raise E6Error('Camera forced to exit; cleanup unconfirmed')
    def observe(self,kind):
        before=self.snapshot();self.camera_number+=1
        folder=self.journal.root/'camera'/f'{self.camera_number:03d}_c{self.journal.cycle}_{kind}';folder.parent.mkdir(exist_ok=True)
        camera=self.bundle['camera'];mode='d2c' if kind in ('hover','park') else 'both43'
        code="import sys;sys.path.insert(0,sys.argv[1]);import rdk_healthcheck as h;h.PROFILES['both43']={'color':(640,480,5,30),'depth':(640,480,12,30)};r=h.capture(sys.argv[2],sys.argv[3],8,decode_workers=2,uvc_pts=True);raise SystemExit(int(r['status']!='completed'))"
        env=dict(os.environ,DCW_SDK_ROOT=camera['sdk_root'],DCW_SDK_CONFIG=camera['sdk_config'])
        log=folder.parent/(folder.name+'.log')
        with log.open('x',encoding='utf-8') as out:
            self.child=subprocess.Popen([sys.executable,'-c',code,camera['tools_dir'],str(folder),mode],env=env,stdout=out,stderr=subprocess.STDOUT)
            start=time.monotonic();last=start
            try:
                while self.child.poll() is None:
                    self.guard()
                    if time.monotonic()-start>35:raise E6Error('Camera capture timed out')
                    if time.monotonic()-last>=.25:self.snapshot();last=time.monotonic()
                    time.sleep(.05)
                if self.child.returncode!=0:raise E6Error('Camera capture failed; inspect '+str(log))
            except BaseException:
                self.stop_once();self.stop_capture();raise
            finally:
                if self.child.poll() is not None:self.child=None
        after=self.snapshot()
        if max(abs(a-b) for a,b in zip(before['q'],after['q']))>.03:raise E6Error('Robot moved during stationary camera capture')
        report=json.loads((folder/'report.json').read_text(encoding='utf-8'))
        age=(datetime.now(timezone.utc)-datetime.fromisoformat(report['ended_at_utc'])).total_seconds()
        if report.get('status')!='completed' or any(report.get(k) for k in ['cleanup_errors','processing_errors','receiver_errors']) or not 0<=age<=30:raise E6Error('Incomplete or stale camera observation')
        if report['source_sha256']!=camera['healthcheck_sha256']:raise E6Error('Unexpected camera capture runtime')
        if report['uvc_pts_availability']['missing_color_frames'] or report['color_decoded_frames']!=report['streams']['color']['frames']:raise E6Error('Incomplete RGB decode/timestamp data')
        if report['device']['serial_number']!='AY4A1520111' or report['color_decoded_frames']<2 or any(s['index_gaps'] or s['index_nonmonotonic_count'] for s in report['streams'].values()):raise E6Error('Wrong camera, missing frames or empty capture')
        observation=dict(image=str(folder/'0008_color.png'),pose=after['pose'],calibration=report['calibration'],camera_report=str(folder/'report.json'),camera_report_sha256=hashlib.sha256((folder/'report.json').read_bytes()).hexdigest(),robot_before=before,robot_after=after)
        self.journal.artifact(f'camera/{folder.name}_observation.json',observation);self.journal.write('observation',observation=observation)
        return observation
    def verify_target(self,obs,bundle):return vision.target(obs,bundle)
    def verify_contact(self,obs):return vision.contact(obs)
    def verify_lift(self,before,after):return vision.lift(before,after)
    def verify_release(self,before,after,bundle):return vision.released(before,after,bundle)
    def verify_pose(self,target):
        s=self.snapshot(target)
        if max(abs(a-b) for a,b in zip(s['q'],target))>.05:raise E6Error('Task endpoint not reached')
        return dict(status='passed',mode=s['mode'],joints_deg=s['q'])
    def stop_once(self):
        if self.robot is None:return dict(attempted=False,reason='No robot connection was opened')
        if not self.robot.stop_attempted:self.robot.command('Stop()')
        return dict(attempted=self.robot.stop_attempted,acknowledged=self.robot.stop_result is not None,automatic_release=False)
    def close(self):
        errors=[];result=dict(status='closed',gripper_power='E6 tool 24V remains powered; CAN DOWN is not disable')
        def attempt(action):
            try:action()
            except BaseException as exc:
                errors.append(repr(exc))
                try:self.stop_once()
                except BaseException as stop_exc:errors.append('Stop: '+repr(stop_exc))
        for action in [self.stop_capture,self.stop_monitor]:attempt(action)
        if self.raw_channel is not None:attempt(self.raw_channel.close)
        if self.configured:
            try:grip.can.configure('can1','down');result['can_after']=grip.can.ip_snapshot('can1')
            except BaseException as exc:
                errors.append(repr(exc))
                try:self.stop_once()
                except BaseException as stop_exc:errors.append('Stop: '+repr(stop_exc))
        attempt(lambda:self.journal.artifact('can_transport.json',self.transport_report))
        if self.raw_robot is not None:attempt(lambda:self.raw_robot.__exit__(None,None,None))
        if self.lock is not None:attempt(self.lock.close)
        for sig,handler in self.previous_handlers.items():attempt(lambda sig=sig,handler=handler:signal.signal(sig,handler))
        if errors:raise E6Error('Cleanup not confirmed: '+repr(errors))
        return result
