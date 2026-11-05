"""Fail-closed hardware/ shadow controller. Device IO is injected for tests.

All deadlines use the X5 monotonic clock. Acceptance is not physical completion.
"""
from collections import deque
from dataclasses import asdict,dataclass
import math
import secrets
import time
import numpy as np
from .control_math import angle,controller_pose,integrate,pose,slew
from .recovery import RecoveryGate,TRANSIENT

@dataclass(frozen=True)
class ControlLimits:
    period_s:float=.05
    lease_s:float=.25
    transaction_s:float=.075
    translation_m_s:float=.002
    rotation_rad_s:float=math.radians(1)
    gripper_s:float=.05
    translation_accel_m_s2:float=.002
    rotation_accel_rad_s2:float=math.radians(1)
    joint_velocity_rad_s:float=math.radians(2)
    joint_accel_rad_s2:float=math.radians(4)
    radius_m:float=.05
    orientation_rad:float=math.radians(10)
    joint_window_rad:float=math.radians(10)
    joint_margin_rad:float=math.radians(5)
    grip_min:float=.15
    grip_max:float=.60

BOUNDS=np.asarray([6.27,2.356,2.6878,2.7925,3.0194,6.27])

class InputReleased(InterruptedError):
    """Normal deadman release, before any new device write."""

class ControlCancelled(InterruptedError):
    """Preserve the originating watchdog/stop reason across device IO."""

def vector(v,n):
    if not isinstance(v,list) or len(v)!=n or any(type(x) not in (int,float) or not math.isfinite(x) for x in v):raise ValueError('Finite numeric vector required')
    return np.asarray(v,dtype=float)

class ControlCore:
    def __init__(self,device,stage='shadow',limits=None,clock=time.perf_counter_ns,recovery_policy='manual'):
        if stage not in ('shadow','live'):raise ValueError('Invalid control stage')
        self.device,self.stage,self.limits,self.clock=device,stage,limits or ControlLimits(),clock
        self.owner=None;self.seq=-1;self.input=None;self.armed=False;self.reason='回中后准备控制'
        self.lease=None;self.lease_ns=0;self.input_ns=0;self.last_tick=None;self.tick_id=0;self.motion_ready_ns=None
        self.base_q=None;self.base_p=None;self.base_r=None;self.target_p=None;self.target_r=None
        self.candidate_q=None;self.q_velocity=np.zeros(6);self.velocity=np.zeros(6)
        self.grip_target=None;self.pending=None;self.last_action=None;self.last_candidate=None
        self.stop_acknowledged=None;self.stop_requested_ns=None;self.stopped_feedback=False;self.stop_result=None
        self.enabled_once=False;self.gripper_initialized=False;self.events=[];self.history=deque(maxlen=160)
        self.commissioning=None;self.gripper_faulted=False
        self.resume_reanchor=False;self.faulted=False;self.first_fault=None
        self.resume_requires_stopped_feedback=False
        self.transaction_jitter_count=0
        self.recovery=RecoveryGate(recovery_policy)
        self._lease()

    def _lease(self):self.lease=secrets.token_urlsafe(24);self.lease_ns=self.clock()
    def status(self):
        return dict(backend='hardware-control',control_stage=self.stage,session=self.owner,lease=self.lease,seq=self.seq,
                    input_delivery='streaming',lease_duration_ms=250,
                    transaction_budget=dict(target_ms=50,hard_deadline_ms=round(self.limits.transaction_s*1000),
                        tolerated_jitter_count=self.transaction_jitter_count),
                    status_revision_ns=self.clock(),
                    motion_ready_monotonic_ns=self.motion_ready_ns,
                    armed=self.armed,reason=self.reason,limits=asdict(self.limits),stop_acknowledged=self.stop_acknowledged,
                    stop_requested_ns=self.stop_requested_ns,stopped_feedback=self.stopped_feedback,
                    stop_confirmation=self.stop_result,
                    pending_command_id=self.pending and self.pending['command_id'],
                    resume_requires_stopped_feedback=self.resume_requires_stopped_feedback,
                    gripper_initialized=self.gripper_initialized,enabled_once=self.enabled_once,faulted=self.faulted,
                    gripper_command_profile=getattr(self,'gripper_command_profile',None),
                    first_fault=None if self.first_fault is None else dict(self.first_fault),
                    recovery=self.recovery.status(),
                    kinematics=dict(getattr(self.device,'kinematics_status',{'kind':'controller-native'})),
                    servo_completion_policy=dict(kind='bounded target tracking, not controller idle',
                        joint_target_actual_tolerance_deg=.05,position_tolerance_mm=1,rotation_tolerance_deg=1,
                        earliest_feedback_ms=50,deadline_ms=150),
                    commissioning=None if self.commissioning is None else {k:v for k,v in self.commissioning.items() if k!='samples'},gripper_faulted=self.gripper_faulted,
                    baseline=None if self.base_q is None else dict(joints_rad=self.base_q.tolist(),position_m=self.base_p.tolist(),quaternion_xyzw=self.base_r.tolist()))

    def connect(self):
        if self.owner:raise ValueError('已有控制者')
        self.owner=secrets.token_urlsafe(24);self.seq=-1;self.input=None;self.armed=False;self._lease()
        return self.status()

    def disconnect(self,owner):
        if owner==self.owner:self.stop('控制连接断开',fault=True);self.owner=None;self.input=None

    def stop(self,reason,disarm=True,force=False,fault=False,explicit_request=False):
        was_active=self.armed or self.pending is not None
        unconfirmed=self.stop_requested_ns is not None and self.stop_acknowledged is False
        if disarm:self.armed=False
        self.reason=reason;self.faulted=self.faulted or fault;self.velocity[:]=0;self.q_velocity[:]=0
        if disarm:self._lease()
        # A failed stop revokes control. Watchdogs, pending-target checks and
        # disconnect handlers must not repeatedly resend it on new sockets.
        # A separately issued explicit Stop can request another attempt.
        confirmation_pending=bool(self.stop_result and not self.stop_result.get('confirmation_complete',True))
        if self.stage=='live' and (was_active or force) and (not unconfirmed or explicit_request and not confirmation_pending):
            self.resume_requires_stopped_feedback=True
            self.stop_requested_ns=self.clock();self.stop_acknowledged=False
            self.stopped_feedback=False;self.history.clear();self.stop_result=None
            try:
                if hasattr(self.device,'begin_stop'):
                    self.stop_result=self.device.begin_stop()
                else:
                    self.device.stop();self.stop_acknowledged=True;self.pending=None
            except Exception as exc:
                self.reason+='; Stop未确认: '+str(exc);self.faulted=True;self.armed=False
                # A normal release becomes a fault if Stop isn't confirmed.
                # Its old streaming lease must also be revoked immediately.
                self._lease()
        if self.faulted and self.first_fault is None:
            self.first_fault=dict(reason=self.reason,x5_monotonic_ns=self.clock())
        self.events.append(dict(kind='stop',reason=self.reason,x5_monotonic_ns=self.clock(),acknowledged=self.stop_acknowledged,faulted=self.faulted))
        if explicit_request:self.recovery.block('操作员停止，等待重新准备')
        elif fault:
            f=self.input or {}
            self.recovery.fault(reason,self.clock(),self.owner,(f.get('source'),f.get('identity')),self.seq,
                hard=self.gripper_faulted or not self.owner or bool(self.commissioning and self.commissioning.get('state')=='running')
                    or self.stop_requested_ns is not None and self.stop_acknowledged is False and self.stop_result is None
                    or self.stop_result is not None and self.stop_result.get('confirmation_complete') and not self.stop_acknowledged)

    def poll_stop(self):
        if self.stop_acknowledged is not False or not hasattr(self.device,'poll_stop'):return
        result=self.device.poll_stop()
        if result is None:return
        self.stop_result=result
        if not result.get('confirmation_complete'):return
        if result.get('acknowledged'):
            self.stop_acknowledged=True;self.pending=None
            self.events.append(dict(kind='stop_confirmation',**result))
        elif not self.faulted or not self.first_fault:
            self.stop('Stop未确认: '+result.get('error','未知错误'),fault=True)
        if not result.get('acknowledged'):self.recovery.block('Stop未确认，禁止自动恢复')

    def safety(self,state,rgb_ns,now):
        d=state['decoded']
        if now-state['x5_receive_monotonic_ns']>100_000_000:raise ValueError('E6反馈超过100ms')
        if not rgb_ns or now-rgb_ns>500_000_000:raise ValueError('RGB超过500ms')
        # Offset 1015 is the algorithm QUEUE pause flag, rather than RobotMode.
        # This E6 retains 1 in normal Mode4/Mode5 with an empty queue (including
        # archived completed MovJ holds). An owned Stop also briefly produces
        # Mode7/pause1/empty queue before its reply. Admit only that bounded
        # stopping transition (the just-stopped queue can still report 1),
        # with the exact last accepted ID and nearby
        # braking target. Stop adjusts QTarget while braking; this tolerance
        # does not change ServoJ's stricter endpoint-completion test. It does
        # not permit motion, rearming, Continue or a changed controller target.
        pause=d.get('pause_command_flag',0)
        if pause not in (0,1):raise ValueError('未知算法队列暂停标志')
        idle_queue=(d['robot_mode'] in (4,5) and not d.get('run_queued_command') and
                    np.max(np.abs(np.radians(vector(d.get('q_target_deg',[999]*6),6))-vector(d['joints_rad'],6)))<=math.radians(.05))
        # Stop can interrupt an acknowledged ServoJ before endpoint tracking
        # finishes and advance QTarget while braking. Bound both the actual
        # braking target by one permitted joint increment. The actual arm
        # may track that braking target within the existing .05deg tolerance,
        # even after passing the accepted endpoint (triangle bound: .15deg).
        # Preserve the existing accepted-target .1deg path for early braking;
        # endpoint completion and stationary confirmation keep their .05deg
        # checks. This is a software bound, not a controller collision limit.
        stop_tracking_bound=max(math.radians(.05),self.limits.joint_velocity_rad_s*self.limits.period_s)
        accepted_q=None if self.last_action is None else np.asarray(self.last_action['joints_rad'])
        actual_q=vector(d['joints_rad'],6)
        braking_q=np.radians(vector(d.get('q_target_deg',[999]*6),6))
        stop_transition=(self.stage=='live' and self.stop_requested_ns is not None
            and 0<=now-self.stop_requested_ns<=1_000_000_000 and self.stop_acknowledged is False
            and self.stop_result is not None and self.stop_result.get('send_confirmed') is True
            and not self.stop_result.get('confirmation_complete',True)
            and d['robot_mode']==7 and d.get('run_queued_command') in (0,1) and self.last_action is not None
            and d.get('current_command_id')==self.last_action['command_id']
            and np.max(np.abs(braking_q-accepted_q))<=stop_tracking_bound
            and (np.max(np.abs(actual_q-accepted_q))<=stop_tracking_bound
                 or np.max(np.abs(actual_q-braking_q))<=math.radians(.05)))
        if d['error_status'] or d.get('collision_state') or pause and not (idle_queue or stop_transition):raise ValueError('E6错误、碰撞或暂停队列')
        allowed=(4,5) if self.stage=='shadow' else (5,7)
        if d['robot_mode'] not in allowed or self.stage=='live' and d['enable_status']!=1:raise ValueError('E6状态不允许控制')
        if d['user_index']!=0 or d['tool_index']!=0 or any(abs(x)>1e-8 for x in d['tool_configuration_raw_mm_deg']):raise ValueError('需要User0与零偏移Tool0，不自动改参')
        if any(abs(x)>1e-8 for x in d.get('user_configuration_raw_mm_deg',[0]*6)):raise ValueError('User0偏移非零')
        q=vector(d['joints_rad'],6)
        if np.any(np.abs(q)>BOUNDS-self.limits.joint_margin_rad):raise ValueError('关节限位余量不足')
        if hasattr(self.device,'validate_feedback'):self.device.validate_feedback(d)
        if self.stage=='live' and self.base_q is not None:
            p,r=pose(d['controller_pose_raw_mm_deg'])
            if np.any(np.abs(q-self.base_q)>self.limits.joint_window_rad) or np.linalg.norm(p-self.base_p)>self.limits.radius_m or angle(r,self.base_r)>self.limits.orientation_rad:raise ValueError('实际状态离开本次工作区')
        return q

    def command(self,data,state,rgb_ns,*,received_ns=None):
        if data.get('session')!=self.owner or not self.owner:raise ValueError('没有控制权')
        operation=data.get('operation');now=self.clock();token_before=self.lease;succeeded=False
        # Only the guardian may supply this timestamp. Never read a client
        # field for lease renewal or input age; lock waiting cannot add life.
        received=now if received_ns is None else received_ns
        try:
            if operation=='stop':self.stop('操作员停止',force=self.stop_acknowledged is not True,explicit_request=True)
            elif operation=='arm':
                if self.commissioning and self.commissioning.get('state')=='running':raise ValueError('初始化任务尚未结束')
                f=self.input
                if not f or now-self.input_ns>250_000_000 or not f['neutral'] or f['deadman'] or not f['focused'] or not f['connected'] or f['stop'] or any(f['twist']) or f['gripper_rate']:raise ValueError('需要新鲜回中输入')
                q=self.safety(state,rgb_ns,now)
                if self.stage=='live':
                    if self.pending or self.stop_requested_ns is not None and (not self.stop_acknowledged or not self.stopped_feedback):raise ValueError('重新准备前必须确认停止及反馈静止')
                    samples=[qv for t,qv in self.history if now-t<=5_000_000_000]
                    if len(samples)<95 or now-self.history[0][0]<4_900_000_000 or np.max(np.ptp(samples,axis=0))>math.radians(.03):raise ValueError('需要至少5秒稳定保持')
                    if self.gripper_faulted:raise ValueError('夹爪存在故障，不允许重新准备')
                p,r=pose(state['decoded']['controller_pose_raw_mm_deg'])
                if hasattr(self.device,'prepare'):self.device.prepare(state['decoded'],now+round(self.limits.transaction_s*1e9))
                if self.base_q is None:self.base_q=q.copy();self.base_p=p.copy();self.base_r=r.copy()
                if np.any(np.abs(q-self.base_q)>self.limits.joint_window_rad):raise ValueError('已离开本次关节工作区')
                self.target_p=p;self.target_r=r;self.candidate_q=q;self.velocity[:]=0;self.q_velocity[:]=0
                if self.stage=='shadow' and self.grip_target is None:self.grip_target=.15
                self.last_tick=now;self.motion_ready_ns=now+round(self.limits.period_s*1e9)
                self.armed=True;self.reason='按住允许动作键随动';self.pending=None;self.faulted=False;self.first_fault=None
                self.resume_requires_stopped_feedback=False
                self.recovery.manual_prepared()
            elif operation=='input':
                if type(received) is not int or not 0<=now-received<=250_000_000:raise ValueError('输入接收后已过期')
                if type(data.get('seq')) is not int or data['seq']<=self.seq:raise ValueError('重复或乱序输入')
                if data.get('lease')!=self.lease or now-self.lease_ns>250_000_000:raise ValueError('租约过期或已消费')
                twist=vector(data.get('twist'),6);grip=data.get('gripper_rate')
                if type(grip) not in (int,float) or not math.isfinite(grip) or abs(grip)>.100001:raise ValueError('无效夹爪速率')
                if np.linalg.norm(twist[:3])>.010001 or np.linalg.norm(twist[3:])>math.radians(5)+1e-6:raise ValueError('输入速度越界')
                for k in ('connected','focused','deadman','fine','stop','neutral'):
                    if type(data.get(k)) is not bool:raise ValueError('输入状态必须为布尔')
                if data.get('frame') not in ('base','tool') or data.get('mode') not in ('translation','rotation') or data.get('source') not in ('gamepad','keyboard'):raise ValueError('未知输入映射')
                if not isinstance(data.get('identity'),str) or not 0<len(data['identity'])<=256 or not isinstance(data.get('raw',{}),dict):raise ValueError('输入身份无效')
                if data['mode']=='translation' and any(twist[3:]) or data['mode']=='rotation' and any(twist[:3]):raise ValueError('模式与速度不符')
                if data['neutral'] and (any(twist) or grip):raise ValueError('虚假回中状态')
                if self.input:
                    changed=[k for k in ('source','identity','frame','mode') if data[k]!=self.input[k]]
                    if changed and (data['deadman'] or not data['neutral']):raise ValueError('切换前必须松开并回中')
                    if any(k in changed for k in ('source','identity')):self.stop('输入设备改变，需要重新准备',fault=True)
                old=self.input;self.seq=data['seq'];self.input_ns=received
                self.input={k:data[k] for k in ('source','identity','frame','mode','connected','focused','deadman','fine','stop','neutral','twist','gripper_rate','seq')}
                self.input.update(raw=data.get('raw',{}),pc_input_monotonic_ns=data.get('pc_input_monotonic_ns'),
                                  gateway_received_ns=received,gateway_processed_ns=now)
                if data['stop'] or not data['focused'] or not data['connected']:self.stop('停止、失焦或输入断开',fault=not data['focused'] or not data['connected'],explicit_request=data['stop'])
                elif self.armed and old and old['deadman'] and not data['deadman']:
                    self.stop('允许动作键已释放，保持目标',disarm=False)
                    self.resume_reanchor=self.stage=='live'
            else:raise ValueError('控制后端拒绝此操作')
            succeeded=True
        except (ValueError,KeyError,TypeError) as exc:
            # Old revoked input authority is rejected, never executed. During
            # recovery it must not erase the original timeout or reset timers.
            if not (self.recovery.waiting and str(exc)=='租约过期或已消费'):
                self.stop('命令被拒绝，需要重新准备',fault=True)
            raise
        finally:
            # Valid sequenced input renews the SAME server lease. Rotating it
            # per input forces the sender to wait for a wireless round trip.
            # Preparation keeps this already validated neutral stream alive;
            # Fault/Stop/connect still invalidate all previous authority.
            if operation=='input' and succeeded:
                if self.lease==token_before:self.lease_ns=received
            elif operation=='arm' and succeeded:self.lease_ns=self.clock()
            elif operation!='input' and not (operation=='arm' and succeeded):self._lease()
        return self.status()

    def recover(self,state,media,now):
        if not self.recovery.waiting:return
        f=self.input or {};origin=self.recovery.origin
        healthy=bool(self.owner==origin['owner'] and (f.get('source'),f.get('identity'))==origin['source']
            and self.seq>origin['seq'] and 0<=now-self.input_ns<=250_000_000
            and f.get('neutral') and not f.get('deadman') and f.get('focused') and f.get('connected') and not f.get('stop')
            and not any(f.get('twist',[1])) and not f.get('gripper_rate') and not self.pending
            and not self.gripper_faulted and (not self.commissioning or self.commissioning.get('state')!='running')
            and (self.stage!='live' or self.stop_acknowledged and self.stopped_feedback))
        if media.get('parent_fatal'):
            self.recovery.block('观测/记录任务异常，不能自动续动');return
        try:
            self.safety(state,(media.get('rgb') or {}).get('x5_receive_monotonic_ns'),now)
            if state['decoded']['robot_mode']!=5 or state['decoded'].get('run_queued_command'):healthy=False
        except Exception as exc:
            if str(exc) not in TRANSIENT:self.recovery.block(str(exc));return
            healthy=False
        if not self.recovery.eligible(now,healthy):return
        original_fault=None if self.first_fault is None else dict(self.first_fault)
        original_origin=dict(origin);self.recovery.begin_attempt(now)
        try:
            if hasattr(self.device,'reconnect_for_arm'):self.device.reconnect_for_arm()
            self.command(dict(session=self.owner,operation='arm'),state,(media.get('rgb') or {}).get('x5_receive_monotonic_ns'))
            self.recovery.origin=original_origin;self.recovery.prepared()
            self.reason=self.recovery.reason
            self.events.append(dict(kind='automatic_prepared',x5_monotonic_ns=self.clock(),
                original_fault=original_fault,attempt=self.recovery.total_attempts,
                old_action_replayed=False,neutral_input_seq=self.seq))
        except Exception as exc:
            if str(exc) not in TRANSIENT:self.recovery.block(str(exc))
            self.events.append(dict(kind='automatic_prepare_failed',reason=str(exc),x5_monotonic_ns=self.clock(),
                attempt=self.recovery.total_attempts,old_action_replayed=False))

    def tick(self,state,media):
        now=self.clock();self.tick_id+=1
        self.poll_stop()
        dt=self.limits.period_s if self.last_tick is None else (now-self.last_tick)/1e9;self.last_tick=now
        first_period_pending=bool(self.armed and self.motion_ready_ns is not None and now<self.motion_ready_ns)
        d=state['decoded'];actual=vector(d['joints_rad'],6)
        received=state['x5_receive_monotonic_ns']
        if d['robot_mode']==5 and d['enable_status']==1:
            if not self.history or received>self.history[-1][0]:self.history.append((received,actual.tolist()))
        else:self.history.clear()
        recent=list(self.history)[-3:]
        self.stopped_feedback=bool(len(recent)>=3 and now-received<=100_000_000
            and (self.stop_requested_ns is None or recent[0][0]>self.stop_requested_ns)
            and np.max(np.ptp([v for _,v in recent],axis=0))<=math.radians(.01)
            and d['robot_mode']==5 and not d.get('run_queued_command')
            and np.max(np.abs(np.radians(d.get('q_target_deg',[999]*6))-actual))<=math.radians(.05))
        self.recover(state,media,now)
        candidate=None;action=None;ik=None;request_goal=None
        if self.pending and not (self.stop_requested_ns is not None and self.stop_acknowledged is False):
            p=self.pending
            # ServoJ's P/D tracker reports a changing internal trajectory in
            # QTarget, not an exact echo of the transmitted endpoint. Require
            # the exact executing ID, t elapsed, fresh feedback and bounded
            # joint AND Cartesian tracking. Release separately confirms Stop
            # and stationary feedback; this does not prove Mode5/idle.
            actual_p,actual_r=pose(d['controller_pose_raw_mm_deg'])
            tracked_pose=(np.linalg.norm(actual_p-np.asarray(p['position_m']))<=.001
                and angle(actual_r,np.asarray(p['quaternion_xyzw']))<=math.radians(1))
            completed=(d.get('current_command_id',-1)==p['command_id'] and d['robot_mode'] in (5,7)
                and 50_000_000<=received-p['accepted_ns']<=150_000_000
                and now-p['accepted_ns']<=150_000_000
                and now-received<=100_000_000
                and np.max(np.abs(actual-np.asarray(p['joints_rad'])))<=math.radians(.05)
                and np.max(np.abs(np.radians(d.get('q_target_deg',[999]*6))-np.asarray(p['joints_rad'])))<=math.radians(.05)
                and tracked_pose)
            if completed:
                self.events.append(dict(kind='servoj_endpoint_completed',command_id=p['command_id'],
                    accepted_ns=p['accepted_ns'],feedback_received_ns=received,robot_mode=d['robot_mode'],
                    run_queued_command=d.get('run_queued_command'),x5_monotonic_ns=now,
                    completion_semantics='target tracking within .05deg and 1mm/1deg, not controller idle'))
                self.pending=None
            elif now-p['accepted_ns']>150_000_000:self.stop('ServoJ目标未及时完成，禁止积压',fault=True)
        try:
            if self.armed:
                self.safety(state,media.get('rgb',{}).get('x5_receive_monotonic_ns'),now)
                if not first_period_pending and not .001<dt<=.075:raise ValueError('控制循环超过周期预算')
                if not self.input or now-self.input_ns>250_000_000:raise ValueError('输入看门狗超时')
            # A target can still be executing at the next 20 Hz tick. Skip
            # that integration/transaction without queuing or catching up;
            # the independent 150 ms completion deadline still stops faults.
            # arm() can arrive just before the next scheduled tick. Do not
            # turn its partial period into an IK/ServoJ/CAN transaction. Keep
            # checking fresh feedback and input; use only the latest input at
            # the first full period, with no buffered or replayed increment.
            active=bool(self.armed and self.input and self.input['deadman'] and not self.pending and not first_period_pending)
            if active:
                if self.recovery.state=='ready':
                    self.recovery.state='idle'
                    self.events.append(dict(kind='automatic_recovery_new_input',input_seq=self.seq,
                        x5_monotonic_ns=now,old_action_replayed=False))
                if self.stage=='live' and self.input['gripper_rate'] and not self.gripper_initialized:raise ValueError('夹爪尚未独立初始化，禁止开度输入')
                if self.stage=='live' and self.resume_requires_stopped_feedback and (not self.stop_acknowledged or not self.stopped_feedback):raise ValueError('停止后尚未确认静止')
                if self.resume_reanchor:
                    self.target_p,self.target_r=pose(d['controller_pose_raw_mm_deg']);self.candidate_q=actual.copy();self.resume_reanchor=False
                start=self.clock();deadline=start+round(self.limits.transaction_s*1e9);f=self.input;scale=.25 if f['fine'] else 1
                requested=np.asarray(f['twist'],float).copy()
                for span,cap in ((slice(0,3),self.limits.translation_m_s),(slice(3,6),self.limits.rotation_rad_s)):
                    requested[span]*=min(1,cap/max(np.linalg.norm(requested[span]),1e-12))*scale
                self.velocity[:3]=slew(self.velocity[:3],requested[:3],self.limits.translation_accel_m_s2*dt)
                self.velocity[3:]=slew(self.velocity[3:],requested[3:],self.limits.rotation_accel_rad_s2*dt)
                tp,tr=integrate(self.target_p,self.target_r,self.velocity[:3],self.velocity[3:],dt,f['frame'])
                if np.linalg.norm(tp-self.base_p)>self.limits.radius_m or angle(tr,self.base_r)>self.limits.orientation_rad:raise ValueError('局部末端工作区边界')
                raw=controller_pose(tp,tr,d['controller_pose_raw_mm_deg'])
                request_goal=dict(position_m=tp.tolist(),quaternion_xyzw=tr.tolist(),controller_pose_mm_deg=raw,input_seq=self.seq)
                seed=actual
                q=np.radians(vector([float(x) for x in self.device.ik(raw,np.degrees(seed),deadline)],6))
                fp,fr=pose(vector([float(x) for x in self.device.fk(np.degrees(q),deadline)],6))
                ik=dict(position_m=float(np.linalg.norm(tp-fp)),rotation_rad=angle(tr,fr),accepted=False)
                qv=(q-self.candidate_q)/dt
                ik.update(joint_velocity_rad_s=qv.tolist(),joint_acceleration_rad_s2=((qv-self.q_velocity)/dt).tolist(),query_duration_ms=(self.clock()-start)/1e6)
                grip=None if self.grip_target is None else float(np.clip(self.grip_target+np.clip(f['gripper_rate'],-self.limits.gripper_s,self.limits.gripper_s)*scale*dt,self.limits.grip_min,self.limits.grip_max))
                candidate=dict(joints_rad=q.tolist(),gripper=grip,requested_tcp=request_goal,ik_residual=ik,input_seq=self.seq)
                if ik['position_m']>.001 or ik['rotation_rad']>math.radians(1):raise ValueError('控制器IK/FK残差超限')
                if np.any(np.abs(q)>BOUNDS-self.limits.joint_margin_rad) or np.any(np.abs(q-self.base_q)>self.limits.joint_window_rad):raise ValueError('关节工作区或限位')
                if np.max(np.abs(qv))>self.limits.joint_velocity_rad_s+1e-6 or np.max(np.abs(qv-self.q_velocity))>self.limits.joint_accel_rad_s2*dt+1e-6:raise ValueError('关节速度或加速度约束')
                ik['accepted']=True
                if self.clock()>deadline:raise ValueError('IK/FK事务超过75ms')
                if getattr(self.device,'release_pending',False):raise InputReleased('允许动作键已释放，候选目标未发送')
                if self.stage=='live':
                    # Check release/fault/watchdog again immediately before every write.
                    # Expiry can occur during IK after the tick's entry check.
                    # Classify that same timeout consistently with watchdog()
                    # so bounded recovery applies after a confirmed stop.
                    if self.clock()-self.input_ns>250_000_000:raise ValueError('输入看门狗超时')
                    if not self.armed or not self.input['deadman']:raise ValueError('发送前控制资格已失效')
                    sent=self.clock();identity=self.device.servo(np.degrees(q),deadline)
                    accepted=self.clock()
                    self.pending=dict(command_id=identity,joints_rad=q.tolist(),accepted_ns=accepted,
                        position_m=tp.tolist(),quaternion_xyzw=tr.tolist())
                    # Once a new command has been accepted after a confirmed
                    # stop, subsequent moving feedback is expected. Preserve
                    # the last Stop evidence without treating it as a new gate.
                    self.resume_requires_stopped_feedback=False
                    action=dict(candidate,gripper=None,active=True,command_id=identity,transaction_started_ns=start,gateway_sent_ns=sent,gateway_accepted_ns=accepted,acceptance='controller_acknowledged_not_completion')
                    self.last_action=action
                    if accepted>deadline:raise ValueError('控制事务超过75ms，命令已接受后停止')
                    if grip is not None and getattr(self.device,'release_pending',False):
                        action['requested_gripper']=grip
                        action['gripper_acceptance']='suppressed_due_to_deadman_release'
                        grip=self.grip_target
                    elif grip is not None:
                        action['requested_gripper']=grip;action['gripper']=None
                        try:
                            feedback=self.device.grip(grip)
                            action['gripper']=(feedback or {}).get('command_opening',round(grip*255)/255)
                            action['gripper_acceptance']='request_associated_feedback'
                            action['gripper_feedback']=feedback
                        except InputReleased:
                            action['gripper_acceptance']='suppressed_due_to_deadman_release'
                            grip=self.grip_target
                        except ControlCancelled:
                            action['gripper_acceptance']='not_sent_due_to_control_cancellation'
                            raise
                        except Exception as exc:
                            self.gripper_faulted=True
                            action['gripper_acceptance']='uncertain';action['gripper_error']=str(exc)
                            raise
                    if self.clock()>deadline:raise ValueError('含CAN的控制事务超过75ms')
                    self.last_action=action
                elapsed=self.clock()-start
                if elapsed>50_000_000:
                    self.transaction_jitter_count+=1
                    self.events.append(dict(kind='control_transaction_jitter_tolerated',duration_ms=elapsed/1e6,
                        target_ms=50,hard_deadline_ms=75,x5_monotonic_ns=self.clock()))
                if action:action.update(transaction_duration_ms=elapsed/1e6,transaction_jitter_tolerated=elapsed>50_000_000)
                self.target_p,self.target_r=tp,tr;self.candidate_q=q;self.q_velocity=qv;self.grip_target=grip
                self.last_candidate=candidate
        except InputReleased as exc:
            # The input handler already raised the cancellation flag and is
            # waiting for this bounded transaction's lock. It issues Stop on
            # acquiring the lock. Do not turn a normal release into a fault.
            if candidate:candidate['rejected_reason']=str(exc)
            ik=dict(ik or {},accepted=False,reason=str(exc))
            self.velocity[:]=0;self.q_velocity[:]=0
        except Exception as exc:
            if candidate:candidate['rejected_reason']=str(exc)
            self.stop(str(exc),fault=True);ik=dict(ik or {},accepted=False,reason=str(exc))
            if action:action.update(stop_requested_after_acceptance=True,cycle_completed=False)
        result=dict(control_tick_id=self.tick_id,control_tick_monotonic_ns=now,input=self.input,
                    requested_target=request_goal,candidate_action=candidate,effective_action=action,ik_result=ik,status=self.status(),events=list(self.events),
                    last_accepted_action=self.last_action,pre_action_robot=state,selected_media=media,dt_s=dt)
        result['skipped_control_period_reason']=('first_period_after_prepare' if first_period_pending
            else 'awaiting_previous_completion' if self.armed and self.pending and action is None else None)
        self.events=[]
        return result
