"""Preview a single isolated gripper diagnostic; never sends E6 commands.

This is not a recovery or teleoperation acceptance. Execution is a separate
explicit diagnostic after a stopped fault, with the live control gateway
closed and a strictly read-only observation gateway active.
The first stalled/fault/dropped/missing reply terminates the trial, without
retry, initialization, force increase, flash writes or automatic return.
"""
import argparse
import base64
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'teleop/device_gateway'))
from omnipicker.tools import verify_can_control as can
from e6_observer.protocol import FeedbackParser,decode_wire

LIMITS={'velocity':5,'force':32,'acceleration':25,'deceleration':25}
TARGET=35.0


def plan(cycle=False,force=32,ramp=False,medium=False,arm_guard='strict',settle_tolerance_counts=1,start_open=False):
    if arm_guard not in ('strict','transient'):raise ValueError('Unknown arm guard policy')
    if type(settle_tolerance_counts) is not int or settle_tolerance_counts not in (1,5):raise ValueError('Endpoint tolerance must be 1 or 5 counts')
    if force not in (32,64) or force==64 and not cycle:raise ValueError('Force64 is only available to the independent fixed cycle diagnostic')
    if ramp and not cycle:raise ValueError('Slow ramps require the independent bounded cycle')
    if medium and (not cycle or ramp):
        raise ValueError('Medium speed requires the independent cycle without --ramp')
    if start_open and not cycle:raise ValueError('Open-start requires the independent bounded cycle')
    targets=[35.,15.,35.,15.] if start_open else [15.,35.,15.] if cycle else [TARGET]
    limits=dict(LIMITS,velocity=25 if medium else 10,force=force) if cycle else LIMITS
    ramp_rate=10. if medium else 2.5 if ramp else None
    return dict(diagnostic_only=True,target_percent=TARGET,targets_percent=targets,limits_raw=limits,
                command_hex=can.encode_command(TARGET,limits).hex(),commands_hex=[can.encode_command(t,limits).hex() for t in targets],hz=20,
                duration_s=6*len(targets)+(sum(abs(b-a) for a,b in zip(targets,targets[1:]))/ramp_rate if ramp_rate else 0),ramp_percent_s=ramp_rate,reply_timeout_ms=25,initialize=False,
                e6_writes=0,automatic_retry=False,automatic_return=False,
                requires_backend='hardware-observe',rgb_freshness_ms=500,
                arm_guard_policy=arm_guard,arm_guard_limits=dict(soft_deg=.05,hard_deg=.25 if arm_guard=='transient' else .05,persistence_ms=100 if arm_guard=='transient' else 0),
                settle_tolerance_counts=settle_tolerance_counts,settle_tolerance_percent=settle_tolerance_counts*100/255,
                stall_policy='abort on the first state=2, including the first reply',
                conclusion_scope='independent fixed targets only; does not unlock continuous teleoperation')


def validate_arm(state,baseline=None,check_tracking=True):
    d=state['decoded']
    if (d['robot_mode']!=5 or d['enable_status']!=1 or d['error_status']
            or d['collision_state'] or d['run_queued_command']):
        raise ValueError('Passive E6 state is not enabled, normal and idle')
    if (d['user_index']!=0 or d['tool_index']!=0
            or any(abs(v)>1e-8 for v in d['tool_configuration_raw_mm_deg'])
            or any(abs(v)>1e-8 for v in d['user_configuration_raw_mm_deg'])):
        raise ValueError('Passive E6 User0/Tool0 configuration changed')
    if check_tracking and max(abs(a-b) for a,b in zip(d['q_actual_deg'],d['q_target_deg']))>.05:
        raise ValueError('Passive E6 target is not stationary')
    if check_tracking and baseline is not None and max(abs(a-b) for a,b in zip(d['q_actual_deg'],baseline))>.05:
        raise ValueError('E6 changed during the isolated diagnostic')


class TransientArmGuard:
    """Observe every fresh packet; tolerate bounded brief deviations only."""
    def __init__(self,baseline=None):
        self.baseline=baseline;self.since={};self.last_ns=None
    def observe(self,state):
        validate_arm(state,check_tracking=False)
        now=state['x5_receive_monotonic_ns']
        if self.last_ns is not None and now<=self.last_ns:
            raise ValueError('Arm guard received a non-increasing receive timestamp')
        self.last_ns=now;d=state['decoded'];errors={}
        references={'target':d['q_target_deg']}
        if self.baseline is not None:references['baseline']=self.baseline
        warning=[];reason=None
        for kind,reference in references.items():
            for i,(actual,target) in enumerate(zip(d['q_actual_deg'],reference)):
                error=abs(actual-target);key=(kind,i);errors[f'{kind}_j{i+1}']=error
                if error>.25:
                    reason=f'J{i+1} {kind} deviation {error:.6f}deg exceeds .25deg hard bound'
                if error>.05:
                    self.since.setdefault(key,now)
                    elapsed=(now-self.since[key])/1e6
                    warning.append(dict(reference=kind,joint=i+1,error_deg=error,persistence_ms=elapsed))
                    if elapsed>=100 and reason is None:
                        reason=f'J{i+1} {kind} deviation exceeds .05deg for {elapsed:.3f}ms'
                else:self.since.pop(key,None)
        return dict(decision='stop' if reason else 'warning' if warning else 'normal',reason=reason,
                    warnings=warning,max_error_deg=max(errors.values()))


class PassiveArm:
    def __init__(self,guard_policy='strict'):
        self.latest=None;self.error=None;self.history=deque(maxlen=8192)
        self.guard=TransientArmGuard() if guard_policy=='transient' else None
        self.frames_received=0
        self.closed=threading.Event();self.lock=threading.Lock()
        # Port 30004 supplies passive feedback. There is no send method or
        # Dashboard connection in this diagnostic.
        self.sock=socket.create_connection(('192.168.1.4',30004),timeout=1)
        self.sock.settimeout(.1)
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()
    def receive(self):
        parser=FeedbackParser(extended=True)
        try:
            while not self.closed.is_set():
                try:data=self.sock.recv(65536)
                except socket.timeout:continue
                if not data:parser.eof();raise ConnectionError('E6 feedback disconnected')
                for raw,decoded in parser.feed(data):
                    state=dict(decoded=decoded,x5_receive_monotonic_ns=time.perf_counter_ns(),
                               raw_sha256=hashlib.sha256(raw).hexdigest(),
                               raw_base64=base64.b64encode(raw).decode())
                    # Preserve the triggering packet before validation raises;
                    # invalid feedback remains unusable because check() first
                    # rejects self.error. Never discard a protection's evidence.
                    with self.lock:
                        self.latest=state;self.history.append(state);self.frames_received=getattr(self,'frames_received',0)+1
                        if getattr(self,'guard',None) is not None:
                            state['tracking_guard']=self.guard.observe(state)
                            if state['tracking_guard']['decision']=='stop':
                                raise ValueError(state['tracking_guard']['reason'])
                        else:validate_arm(state)
        except Exception as exc:
            if not self.closed.is_set():self.error=str(exc)
    def evidence(self):
        with self.lock:
            return dict(error=self.error,latest=self.latest,frames=list(self.history),
                        total_frames_received=getattr(self,'frames_received',len(self.history)),
                        all_received_frames_retained=getattr(self,'frames_received',len(self.history))==len(self.history))
    def check(self,baseline=None):
        with self.lock:s=self.latest
        if self.error:raise RuntimeError(self.error)
        if s is None or time.perf_counter_ns()-s['x5_receive_monotonic_ns']>100_000_000:
            raise ValueError('Passive E6 feedback is missing or older than 100ms')
        validate_arm(s,baseline,check_tracking=getattr(self,'guard',None) is None);return s
    def stable(self,cancelled):
        started=time.monotonic()
        while time.monotonic()-started<5.5:
            if cancelled():raise InterruptedError('Diagnostic cancelled')
            if time.monotonic()-started>.2:self.check()
            time.sleep(.02)
        self.check()
        with self.lock:rows=list(self.history)
        newest=rows[-1]['x5_receive_monotonic_ns']
        recent=[r for r in rows if newest-r['x5_receive_monotonic_ns']<=5_000_000_000]
        if len(recent)<95 or newest-recent[0]['x5_receive_monotonic_ns']<4_900_000_000:
            raise ValueError('Five seconds of passive E6 feedback were not captured')
        q=[r['decoded']['q_actual_deg'] for r in recent]
        if max(max(v)-min(v) for v in zip(*q))>.03:
            raise ValueError('Passive E6 did not hold within .03 degrees')
        baseline=rows[-1]['decoded']['q_actual_deg']
        with self.lock:
            if self.guard is not None:self.guard.baseline=baseline
        return baseline
    def close(self):
        self.closed.set();self.sock.close();self.thread.join(1)


class PassiveRgb:
    """Read the existing thin camera gateway; no camera/device control calls."""
    def __init__(self):
        from websockets.sync.client import connect
        self.latest=None;self.error=None;self.closed=threading.Event()
        self.ws=connect('ws://127.0.0.1:8767/ws/observations',compression=None,max_size=8*1024*1024)
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()
    def receive(self):
        try:
            while not self.closed.is_set():
                try:raw=self.ws.recv(timeout=.1)
                except TimeoutError:continue
                frame=decode_wire(raw,{})
                if frame.get('clock_probe'):continue
                if frame.get('backend')!='hardware-observe' or frame.get('stats',{}).get('fatal'):
                    raise ValueError('A healthy strictly read-only observation gateway is required')
                self.latest=frame['sources']['rgb']
        except Exception as exc:
            if not self.closed.is_set():self.error=str(exc)
    def check(self):
        s=self.latest
        if self.error:raise RuntimeError(self.error)
        if (s is None or not s.get('valid') or not s.get('x5_receive_monotonic_ns')
                or not 0<=time.perf_counter_ns()-s['x5_receive_monotonic_ns']<=500_000_000):
            raise ValueError('RGB is missing, invalid or older than 500ms')
        return s
    def close(self):
        self.closed.set();self.ws.close();self.thread.join(1)


def trial(channel,report,check,cancelled=lambda:False,clock=time.monotonic,sleep=time.sleep,target=TARGET,limits=None,settle_tolerance_counts=1):
    if type(settle_tolerance_counts) is not int or settle_tolerance_counts not in (1,5):raise ValueError('Endpoint tolerance must be 1 or 5 counts')
    command=can.encode_command(target,limits or LIMITS)
    start=clock();deadline=start;stable=0;report['feedback']=[]
    while clock()-start<6:
        if cancelled():raise InterruptedError('Diagnostic cancelled; no later targets')
        check()
        f=channel.exchange(command)  # Strict production decoder; state=2 raises.
        report['feedback'].append(f)
        stable=stable+1 if f['state']==0 and abs(f['position_raw']-command[1])<=settle_tolerance_counts else 0
        deadline+=.05;sleep(max(0,deadline-clock()))
    if stable<3:raise ValueError('Fixed target did not end with three normal at-target replies')


def slow_ramp(channel,report,check,start,target,limits,cancelled=lambda:False,clock=time.monotonic,sleep=time.sleep,rate=2.5):
    """Bounded target ramp; every reply retains strict fault checks."""
    if not math.isfinite(rate) or not 0<rate<=10:
        raise ValueError('Diagnostic ramp rate must be in (0,10] percentage points/s')
    steps=math.ceil(abs(target-start)/(.05*rate))
    deadline=clock();report['ramp_feedback']=[]
    for index in range(1,steps+1):
        if cancelled():raise InterruptedError('Diagnostic cancelled; no later targets')
        check()
        goal=start+(target-start)*index/steps
        command=can.encode_command(goal,limits)
        feedback=channel.exchange(command)
        report['ramp_feedback'].append(dict(target_percent=goal,command_hex=command.hex(),feedback=feedback))
        deadline+=.05;sleep(max(0,deadline-clock()))


def execute(destination,cycle=False,force=32,ramp=False,medium=False,arm_guard='strict',settle_tolerance_counts=1,start_open=False):
    import fcntl
    report=dict(plan(cycle,force,ramp,medium,arm_guard,settle_tolerance_counts,start_open),started_at_utc=datetime.now(timezone.utc).isoformat(),
                status='preflight',events=[],source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    channel=arm=rgb=None;configured=False;cancel=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,lambda *_:cancel.set())
    # Reject concurrent use; never retire another process or clear its fault.
    with destination.open('x',encoding='utf-8') as evidence,open('/tmp/omnipicker-can1.lock','a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            report['identity']=can.validate_usb_interface('can1')
            report['before']=can.ip_snapshot('can1')
            if 'UP' in report['before']['flags']:raise ValueError('CAN already UP; gateway must be released explicitly')
            rgb=PassiveRgb();ready_until=time.monotonic()+1
            while rgb.latest is None:
                if cancel.is_set():raise InterruptedError('Diagnostic cancelled')
                if rgb.error or time.monotonic()>ready_until:rgb.check()
                time.sleep(.02)
            rgb.check();arm=PassiveArm(arm_guard)
            def preflight_cancelled():
                if arm.latest is not None:rgb.check()
                return cancel.is_set()
            baseline=arm.stable(preflight_cancelled);report['arm_baseline_deg']=baseline
            report['rgb_before']=rgb.check()
            configured=True
            can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off')
            can.configure('can1','up');report['configured']=can.ip_snapshot('can1')
            if report['configured']['linkinfo']['info_data']['bittiming']['bitrate']!=1000000:
                raise ValueError('CAN bitrate did not read back as 1Mbps')
            channel=can.CanChannel('can1',1,.025,report)
            def check():
                arm.check(baseline);rgb.check()
            report['stages']=[]
            for target in report['targets_percent']:
                stage=dict(target_percent=target,feedback=[])
                report['stages'].append(stage)
                if report['ramp_percent_s'] and len(report['stages'])>1:
                    slow_ramp(channel,stage,check,report['stages'][-2]['target_percent'],target,report['limits_raw'],cancel.is_set,rate=report['ramp_percent_s'])
                trial(channel,stage,check,cancel.is_set,target=target,limits=report['limits_raw'],settle_tolerance_counts=settle_tolerance_counts)
            report['feedback']=report['stages'][-1]['feedback']
            report['status']='completed';report['arm_final']=arm.check(baseline);report['rgb_after']=rgb.check()
        except BaseException as exc:report.update(status='aborted',error=str(exc))
        finally:
            if channel:channel.close()
            if configured:
                try:
                    report['before_down']=can.ip_snapshot('can1');can.configure('can1','down')
                    report['after']=can.ip_snapshot('can1')
                except Exception as exc:report.update(status='cleanup_unconfirmed',cleanup_error=str(exc))
            if arm:
                arm.close();report['arm_feedback']=arm.evidence()
            if rgb:rgb.close()
            report.update(ended_at_utc=datetime.now(timezone.utc).isoformat(),
                          gripper_power_state='not measured; CAN DOWN is not motor disable or 24V off')
            json.dump(report,evidence,ensure_ascii=False,indent=2)
    print(json.dumps({k:report.get(k) for k in ('status','error','cleanup_error')},ensure_ascii=False))
    return 0 if report['status']=='completed' else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--output',type=Path)
    parser.add_argument('--cycle',action='store_true',help='Independent empty-load 15→35→15 at verified Vel10/Force32, six seconds each; first anomaly aborts')
    parser.add_argument('--force',type=int,choices=(32,64),default=32,help='Force64 requires separately authorized independent cycle; production remains Force32')
    parser.add_argument('--ramp',action='store_true',help='Use 2.5 percentage points/s between the same independent 15/35/15 endpoints')
    parser.add_argument('--medium32',action='store_true',help='Independent Force32 / Vel25 cycle with 10 percentage points/s ramps; requires --cycle')
    parser.add_argument('--medium',action='store_true',help='Independent Vel25 cycle with 10 percentage points/s ramps and explicitly selected --force; requires --cycle')
    parser.add_argument('--start-open',action='store_true',help='Independent 35/15/35/15 cycle; first reply retains the same strict fault checks')
    parser.add_argument('--arm-guard',choices=('strict','transient'),default='strict',help='Diagnostic only: transient tolerates <.25deg deviations lasting <100ms, with every packet retained')
    parser.add_argument('--settle-tolerance-counts',type=int,choices=(1,5),default=1,help='Diagnostic endpoint tolerance: 1 count (.39 percentage points) or 5 counts (1.96 percentage points), still requires three state=0 replies')
    args=parser.parse_args()
    if args.medium32 and args.force!=32:parser.error('--medium32 is bound to Force32; use --medium for an explicitly selected force')
    medium=args.medium or args.medium32
    if not args.execute:print(json.dumps(plan(args.cycle,args.force,args.ramp,medium,args.arm_guard,args.settle_tolerance_counts,args.start_open),ensure_ascii=False,indent=2))
    else:
        if sys.platform!='linux' or args.output is None:parser.error('Execution requires Linux and a new --output file')
        raise SystemExit(execute(args.output,args.cycle,args.force,args.ramp,medium,args.arm_guard,args.settle_tolerance_counts,args.start_open))
