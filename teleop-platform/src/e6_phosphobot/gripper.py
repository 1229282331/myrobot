"""OmniPicker adapter using the already verified SocketCAN protocol module.

The worker owns only can1. The gateway requests one feedback-only attachment
after health checks; import/start sends no frames. Other threads never wait
for a CAN response. FW335 observation never clears a fault or stops motion.
"""
from collections import deque
import math
import threading
import time

from . import omnipicker_can as can
from . import gripper_stroke as stroke

MINIMUM_UPDATE_COUNTS = 4
MINIMUM_OPENING = stroke.CLOSED_RAW/255
MAXIMUM_OPENING = 1.
# CAN velocity is normalized by the vendor, not a position-counts/s unit.
# Existing unloaded traces give 1.90..1.95 position counts/s per velocity byte.
# Keep this an explicit engineering estimate, qualified separately on hardware.
POSITION_COUNTS_PER_VELOCITY_SECOND = 1.92
RATE_LEAD_COUNTS = 4
MAX_RATE_LEAD_COUNTS = 8
RATE_ACCELERATION_S2 = .30
RATE_JERK_S3 = 2.
PROGRESS_COUNTS = 2 # One encoder count can be stationary quantization jitter.
CONTROL_LIMITS = can.FW335_STUDIO_LIMITS


def progress_timeout_ns(velocity):
    # FW335 state=2 can stay latched through a successful reverse. At speed
    # byte 1, two real counts take about 1.04 s, not 500 ms. This is a bounded
    # no-progress deadline, never permission to advance an unchecked target.
    seconds=max(.5,PROGRESS_COUNTS/(max(1,velocity or 1)*POSITION_COUNTS_PER_VELOCITY_SECOND)+.5)
    return int(seconds*1e9)


def opening_raw(target):
    """Quantize inside the configured envelope, including fractional endpoints."""
    return max(math.ceil(MINIMUM_OPENING*255),
               min(math.floor(MAXIMUM_OPENING*255),round(target*255)))


def geometry_proposal(command_raw,sent_raw):
    """One bounded four-count batch; keep the existing endpoint remainder rule.

    Geometry permits apply to the next CAN command, not an accumulating future
    target. At most seven counts are needed to avoid a stranded 1..3 remainder.
    """
    if command_raw is None or sent_raw is None:return command_raw
    delta=command_raw-sent_raw
    return sent_raw+(MINIMUM_UPDATE_COUNTS if delta>0 else -MINIMUM_UPDATE_COUNTS) if abs(delta)>=2*MINIMUM_UPDATE_COUNTS else command_raw

class CanDevice:
    def __init__(self,emit):
        import fcntl
        self.emit=emit
        class Sink:
            def append(self,event):emit(dict(event='can',data=event))
        self.lock=open('/tmp/omnipicker-can1.lock','a')
        self.channel=None;self.owned=False
        try:
            fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            identity=can.validate_usb_interface('can1')
            before=can.ip_snapshot('can1')
            if 'UP' in before['flags']:raise RuntimeError('can1 already owned; refusing takeover')
            emit(dict(event='can_identity',identity=identity,before=before))
            self.owned=True
            can.configure('can1','type','can','bitrate','1000000','sample-point','0.8','listen-only','off')
            can.configure('can1','up')
            configured=can.ip_snapshot('can1')
            if configured['linkinfo']['info_data']['bittiming']['bitrate']!=1_000_000:
                raise RuntimeError('CAN bitrate readback differs')
            emit(dict(event='can_configured',actual=configured))
            # State 2 is motion obstruction, distinct from nonzero fault codes.
            # The adapter classifies direction and freezes target accumulation.
            self.channel=can.CanChannel('can1',1,.025,dict(events=Sink()),allow_stalled=True,allow_dropped=True)
        except BaseException:
            self.close();raise
    def exchange(self,target,*,velocity=None):
        limits=dict(CONTROL_LIMITS)
        if velocity is not None:limits['velocity']=velocity
        command=can.encode_command(target*100,limits,bounds=(100*MINIMUM_OPENING,100*MAXIMUM_OPENING),profile='fw335_studio_3a')
        return self.channel.exchange(command)
    def observe(self):
        return self.channel.exchange(can.FW335_OBSERVATION)
    def close(self):
        try:
            if self.channel:self.channel.close()
        finally:
            try:
                if self.owned:
                    can.configure('can1','down')
                    self.emit(dict(event='can_closed',actual=can.ip_snapshot('can1')))
            finally:self.lock.close()


class Gripper:
    def __init__(self,*,device_factory=CanDevice,clock=time.perf_counter_ns,rate_s=.025,motion_guard=None,geometry_lock=None):
        self.motion_guard=motion_guard;self.collision_blocked=False
        self.geometry_lock=geometry_lock or threading.RLock()
        if rate_s not in (.025,.05,.1):raise ValueError('Unknown gripper rate profile')
        self.rate_s=rate_s
        self.factory=device_factory;self.clock=clock
        self.lock=threading.Lock();self.halt=threading.Event()
        self.thread=None;self.device=None;self.phase='uninitialized'
        self.reason=None;self.target=None;self.actual=None;self.received_ns=None
        self.events=deque(maxlen=4096);self.event_drops=0
        self.initialized=False;self.count=0;self.initializing_ns=None
        self.arrived_count=0;self.last_sent=None
        self.last_feedback=None;self.target_changed_ns=None
        self.command_target=None;self.inflight=None
        self.rate_active=False;self.rate_direction=0;self.velocity=None
        self.rate_received_ns=None
        self.braking_target=None
        self.attachment_raw=None
        self.ramped_rate=0.;self.rate_acceleration=0.
        self.contact=None;self.contact_requested=False;self.contact_escape_ns=None
        self.contact_escape_direction=0
        self.contact_escape_origin_raw=None;self.state2_motion_verified=False
        self.progress_direction=0;self.progress_raw=None;self.progress_ns=None;self.progress_velocity=1
    def emit(self,event):
        with self.lock:
            if len(self.events)==self.events.maxlen:self.event_drops+=1
            self.events.append(dict(at_ns=self.clock(),**event))
    def start(self):
        self.thread=threading.Thread(target=self._run,daemon=True,name='omnipicker-can')
        self.thread.start()
    def initialize(self):
        with self.lock:
            if self.phase!='uninitialized':raise RuntimeError('夹爪已初始化或故障；不自动重试')
            self.phase='initializing';self.target=.35;self.initializing_ns=self.clock()
            self.command_target=self.target
            self.target_changed_ns=self.initializing_ns
    def connect(self):
        """Acquire actual aperture once, without a 35% positioning command.

        A browser reconnect cannot repeat attachment or clear a CAN fault.
        The first real target still requires the normal geometry permit.
        """
        with self.lock:
            if self.phase!='uninitialized':return False
            self.phase='connecting';self.initializing_ns=self.clock()
        self.emit(dict(event='gripper_connecting',method='fw335_observation',stroke=stroke.metadata(),
                       native_limits=dict(CONTROL_LIMITS),configured_current_limit_a=CONTROL_LIMITS['force']*3/255,
                       current_limit_basis='pinned_fw335_D3_mapping_not_measured_force'))
        return True
    def set_target(self,target):
        with self.lock:
            if self.phase!='ready':return False
            if not math.isfinite(target) or not MINIMUM_OPENING<=target<=MAXIMUM_OPENING:raise ValueError('Native gripper target outside installed soft stroke')
            if self.contact is not None:
                direction=1 if target>self.actual else -1 if target<self.actual else 0
                return self._request_contact_escape(direction,self.rate_s,target)
            if self.target is None or opening_raw(target)!=opening_raw(self.target):
                self.target_changed_ns=self.clock()
            self.target=target
            # The device stalls short of position under single-count updates
            # despite state=0. Retained fixed/ramp/four-count comparisons prove
            # this command-size dependency. Accumulate operator intent; never
            # overshoot it, increase force, or retry a failed transaction.
            requested=opening_raw(target);previous=round(self.command_target*255)
            boundary=opening_raw(MAXIMUM_OPENING if requested>previous else MINIMUM_OPENING)
            # Avoid stranding a sub-batch at the endpoint. Wait for continued
            # input to reach it, then send that exact endpoint as one batch.
            # Release still discards the unsent remainder; no extrapolation.
            near_boundary=0<abs(boundary-requested)<MINIMUM_UPDATE_COUNTS
            if abs(requested-previous)>=MINIMUM_UPDATE_COUNTS and not near_boundary:
                # 15% rounds to raw 38 (14.90%). Keep the committed command
                # inward at raw 39 so the real encoder and FCL see the same
                # legal value; never relax the transport's range validation.
                self.command_target=target if round(target*255)==requested else requested/255
                self.arrived_count=0
            return True
    def set_stroke_rate(self,rate,dt):
        """Operator rate in usable stroke/s; raw CAN and geometry stay native."""
        if not math.isfinite(rate) or abs(rate)>self.rate_s+1e-9:raise ValueError('Invalid gripper stroke rate')
        return self.set_rate(rate*stroke.SPAN,dt)
    def set_rate(self,rate,dt):
        """Bounded position lead plus native CAN speed, driven by live intent.

        Hold a pending proposal stable until FCL admits it. No accumulated
        catch-up beyond eight counts of real feedback; reversals rebase there.
        Fine input no longer has to accumulate four counts before it can move.
        """
        if not math.isfinite(rate) or abs(rate)>self.rate_s+1e-9:raise ValueError('Invalid gripper rate')
        if not math.isfinite(dt) or not 0<=dt<=.1:raise ValueError('Invalid gripper interval')
        if not rate:self.hold();return True
        with self.lock:
            outside_closed=(self.actual is not None and self.actual<MINIMUM_OPENING and rate<0)
        if outside_closed:
            # Observation can legitimately report one count below the soft
            # endpoint. Closing must neither push farther nor command opening.
            self.hold();return True
        with self.lock:
            now=self.clock()
            if self.phase!='ready' or self.received_ns is None or not 0<=now-self.received_ns<=200_000_000:return False
            direction=1 if rate>0 else -1
            if self.contact is not None:
                self.rate_received_ns=now
                return self._request_contact_escape(direction,abs(rate))
            actual_raw=round(self.actual*255)
            if not self.rate_active or direction!=self.rate_direction:
                self.target=self.actual
                self.command_target=self.last_sent if self.last_sent is not None else self.actual
                self.ramped_rate=0.;self.rate_acceleration=0.
            self.rate_active=True;self.rate_direction=direction;self.rate_received_ns=now
            self.braking_target=None
            # Bound both speed changes and acceleration onset. Native CAN
            # still has a minimum positive speed byte; zero is observation,
            # never a motor stop. Release uses the existing one-shot hold.
            speed_goal=abs(rate)
            accel_goal=max(-RATE_ACCELERATION_S2,min(RATE_ACCELERATION_S2,(speed_goal-self.ramped_rate)/dt)) if dt else 0.
            self.rate_acceleration+=max(-RATE_JERK_S3*dt,min(RATE_JERK_S3*dt,accel_goal-self.rate_acceleration))
            advanced=self.ramped_rate+self.rate_acceleration*dt
            self.ramped_rate=min(speed_goal,max(self.ramped_rate,advanced)) if speed_goal>=self.ramped_rate else max(speed_goal,min(self.ramped_rate,advanced))
            self.velocity=max(1,min(CONTROL_LIMITS['velocity'],round(self.ramped_rate*255/POSITION_COUNTS_PER_VELOCITY_SECOND)))
            self.target=max(MINIMUM_OPENING,min(MAXIMUM_OPENING,self.target+rate*dt))
            # Limit integration against observed progress, including blocked
            # geometry and stalls. A fresh input never flushes old backlog.
            self.target=max(self.actual-MAX_RATE_LEAD_COUNTS/255,
                            min(self.actual+MAX_RATE_LEAD_COUNTS/255,self.target))
            requested=opening_raw(self.target+direction*RATE_LEAD_COUNTS/255)
            requested=opening_raw(max((actual_raw-MAX_RATE_LEAD_COUNTS)/255,
                                     min((actual_raw+MAX_RATE_LEAD_COUNTS)/255,requested/255)))
            sent=round((self.last_sent if self.last_sent is not None else self.actual)*255)
            pending=round(self.command_target*255)
            if pending==sent or direction*(pending-sent)<0:
                if requested!=pending:self.target_changed_ns=now;self.arrived_count=0
                self.command_target=requested/255
            return True
    def hold(self):
        with self.lock:
            if self.phase!='ready':return
            if self.contact is not None:
                if self.contact.get('kind')=='object_dropped':self.contact['neutral_seen']=True
                # Keep the already committed holding effort. Do not replace it
                # with measured position, increase force, or flush pending input.
                held=self.inflight if self.inflight is not None else self.last_sent
                self.target=self.command_target=self.actual if held is None else held
                self.contact_requested=False;self.rate_active=False;self.rate_direction=0
                return
            if self.braking_target is not None:return
            # An exchange already submitted can still complete. Discard only
            # unsent intent, including a pending complete batch; never flush a
            # residual after release, reconnect or a reversal from neutral.
            held=self.inflight if self.inflight is not None else self.last_sent
            if held is None:
                # Attachment has sent no target yet. Cancel pending first
                # intent without manufacturing a hold command.
                if self.attachment_raw is not None:self.target=self.command_target=self.attachment_raw/255
                self.rate_active=False;self.rate_direction=0
                return
            discarded=self.target
            # A low-speed position lead must not run on for several seconds.
            # The firmware ignores a zero velocity byte (retained v272 probe).
            # Select ONE near-feedback stopping target, within the already
            # submitted interval. The normal FCL proposal gate still applies.
            # Repeated neutral inputs must never chase the measured position.
            if (self.rate_active and self.received_ns is not None and
                    0<=self.clock()-self.received_ns<=200_000_000):
                raw=round(self.actual*255);sent=round(held*255)
                speed=max(self.velocity or 0,(self.last_feedback or {}).get('velocity_raw',0))
                lead=max(1,round(speed*POSITION_COUNTS_PER_VELOCITY_SECOND*.15))
                stopped=min(sent,raw+lead) if sent>=raw else max(sent,raw-lead)
                held=opening_raw(stopped/255)/255
                self.braking_target=held
            self.target=self.command_target=held
            self.rate_active=False;self.rate_direction=0
        if discarded!=held:self.emit(dict(event='gripper_hold',discarded_request=discarded,target=held))
    def fault(self,reason):
        with self.lock:
            self.phase='fault';self.reason=reason
        self.emit(dict(event='gripper_fault',reason=reason))
    def _request_contact_escape(self,direction,rate,target=None):
        """Called under lock. One bounded reverse proposal, never old-target steps."""
        if self.received_ns is None or not 0<=self.clock()-self.received_ns<=200_000_000:return False
        if self.contact.get('kind')=='object_dropped' and not self.contact.get('neutral_seen'):return False
        blocked=self.contact['direction']
        if not direction or direction==blocked or (blocked==0 and direction<0):
            if self.contact_escape_ns is None:
                self.contact_requested=False
                self.target=self.command_target=self.actual if self.last_sent is None else self.last_sent
                self.rate_active=False;self.rate_direction=0
            return True
        if self.contact_requested or self.contact_escape_ns is not None:return True
        actual_raw=round(self.actual*255)
        raw=opening_raw((actual_raw+direction*RATE_LEAD_COUNTS)/255)
        if target is not None:raw=min(raw,opening_raw(target)) if direction>0 else max(raw,opening_raw(target))
        if direction*(raw-actual_raw)<=0:return True
        self.target=self.command_target=raw/255
        self.target_changed_ns=self.clock();self.arrived_count=0
        self.contact_requested=True;self.rate_active=True;self.rate_direction=direction
        self.rate_received_ns=self.clock();self.braking_target=None
        self.ramped_rate=0.;self.rate_acceleration=0.
        # Restart gently. Subsequent live input resumes the existing ramp only
        # after the device confirms leaving the obstructed state.
        self.velocity=1
        return True
    def _contact_feedback(self,feedback,target,*,observed=False):
        """Update a latched obstruction from fresh feedback; no actuator writes."""
        event=None
        with self.lock:
            now=self.clock()
            previous_state=(self.last_feedback or {}).get('state')
            self.actual=feedback['position_raw']/255;self.received_ns=feedback['host_perf_counter_ns']
            self.last_feedback=dict(feedback);self.count+=1
            if feedback.get('fault',0):raise RuntimeError('Gripper device fault: '+str(feedback))
            dropped=feedback['state']==3
            existing_drop=self.contact is not None and self.contact.get('kind')=='object_dropped'
            if dropped and not existing_drop and (previous_state!=3 or not self.state2_motion_verified):
                # Task failure, not a motor fault. Discard pending motion and
                # observe only. Old held closing input can never chase a fall;
                # release/Stop then an explicit opening request is required.
                self.contact=dict(direction=-1,kind='object_dropped',neutral_seen=False,
                                  since_ns=now,actual_raw=feedback['position_raw'])
                self.contact_requested=False;self.contact_escape_ns=None
                self.state2_motion_verified=False;self.rate_active=False;self.rate_direction=0
                self.target=self.command_target=self.actual if target is None else target
                self.braking_target=None;self.arrived_count=0
                event=dict(event='gripper_object_dropped',**self.contact)
            elif self.contact is not None and self.contact_escape_ns is not None:
                progressed=self.contact_escape_direction*(feedback['position_raw']-self.contact_escape_origin_raw)>=PROGRESS_COUNTS
                if feedback['state'] not in (2,3) or progressed:
                    event=dict(event='gripper_contact_released',direction=self.contact_escape_direction,actual_raw=feedback['position_raw'],
                               reported_state=feedback['state'],basis='measured_reverse_progress' if feedback['state'] in (2,3) else 'device_state')
                    self.state2_motion_verified=feedback['state'] in (2,3)
                    self._reset_progress(self.contact_escape_direction,feedback['position_raw'],now)
                    self.contact=None;self.contact_escape_ns=None;self.contact_requested=False
                    self.target=self.command_target=self.last_sent
                    self.rate_active=False;self.rate_direction=0
                elif now-self.contact_escape_ns>progress_timeout_ns(1):
                    self.contact=dict(direction=self.contact_escape_direction,since_ns=now,actual_raw=feedback['position_raw'])
                    self.contact_escape_ns=None;self.contact_requested=False;self.rate_active=False
                    self.target=self.command_target=self.last_sent
                    event=dict(event='gripper_contact',**self.contact)
            elif feedback['state'] in (2,3) and self.contact is None:
                direction=0 if observed or target is None else (1 if target>self.actual else -1 if target<self.actual else 0)
                if self.state2_motion_verified and target is not None and not self._no_progress(feedback,round(target*255),now):
                    matches=self.command_target is not None and round(self.command_target*255)==round(target*255)
                    measured_arrival=abs(feedback['position_raw']-round(target*255))<=1 and feedback.get('velocity_raw',0)==0
                    self.arrived_count=self.arrived_count+1 if matches and measured_arrival else 0
                else:
                    self.state2_motion_verified=False
                    self.contact=dict(direction=direction,since_ns=now,actual_raw=feedback['position_raw'])
                    self.target=self.command_target=self.actual if target is None else target
                    self.contact_requested=False;self.rate_active=False;self.rate_direction=0
                    self.braking_target=None;self.arrived_count=0
                    event=dict(event='gripper_contact',**self.contact)
        if event:self.emit(event)
    def _reset_progress(self,direction,raw,now):
        # All progress helpers run under self.lock. New target packets in the
        # same direction must NOT reset the deadline for a physically stuck jaw.
        self.progress_direction=direction;self.progress_raw=raw;self.progress_ns=now
        self.progress_velocity=max(1,self.velocity or 1)
    def _no_progress(self,feedback,target_raw,now):
        raw=feedback['position_raw'];remaining=target_raw-raw
        direction=1 if remaining>0 else -1 if remaining<0 else 0
        if abs(remaining)<=1:
            self._reset_progress(0,raw,now)
            return False
        if self.progress_ns is None or direction!=self.progress_direction:
            self._reset_progress(direction,raw,now)
        elif direction*(raw-self.progress_raw)>=PROGRESS_COUNTS:
            self._reset_progress(direction,raw,now)
        # Allow the slowest speed used in this progress window to take effect;
        # a new faster command cannot retroactively shorten its startup time.
        self.progress_velocity=min(self.progress_velocity,max(1,self.velocity or 1))
        return now-self.progress_ns>progress_timeout_ns(self.progress_velocity)
    def cycle(self):
        with self.lock:
            expired=self.rate_active and self.rate_received_ns is not None and self.clock()-self.rate_received_ns>250_000_000
        if expired:self.hold()
        with self.lock:phase=self.phase
        if phase not in ('connecting','initializing','ready'):return
        try:
            # Heavy FCL computation runs in the separate model process. This
            # callback consumes only a bounded, fresh result. Keep polling the
            # already-authorized target while a new proposal waits/is blocked.
            if self.device is None:self.device=self.factory(self.emit)
            with self.lock:
                observing=phase=='connecting' or (phase=='ready' and self.last_sent is None and not self.rate_active and
                    self.command_target==self.target and round(self.command_target*255)==self.attachment_raw)
                contact_observing=self.contact is not None and (not self.contact_requested or self.contact_escape_ns is not None)
            if contact_observing:
                self._contact_feedback(self.device.observe(),self.last_sent,observed=True)
                return
            if observing:
                feedback=self.device.observe()
                now=self.clock();raw=feedback['position_raw']
                if feedback.get('fault',0):raise RuntimeError('Gripper attachment fault: '+str(feedback))
                with self.lock:
                    if self.phase not in ('connecting','ready'):return
                    stationary=feedback['state'] in (0,2,3) and feedback.get('velocity_raw',0)==0
                    consistent=self.attachment_raw is None or abs(raw-self.attachment_raw)<=1
                    self.arrived_count=self.arrived_count+1 if stationary and consistent else 0
                    if self.phase=='connecting':self.attachment_raw=raw
                    self.actual=raw/255;self.received_ns=feedback['host_perf_counter_ns']
                    self.last_feedback=dict(feedback);self.count+=1
                    connected=self.phase=='connecting' and self.arrived_count>=3
                    if connected:
                        self.initialized=True;self.phase='ready'
                        self.target=self.command_target=self.actual
                        self.target_changed_ns=now
                    expired=self.phase=='connecting' and now-self.initializing_ns>6_000_000_000
                self.emit(dict(event='gripper_observation',feedback=feedback))
                if connected:self.emit(dict(event='gripper_connected',actual_raw=raw,position_command_sent=False))
                if connected and feedback['state'] in (2,3):self._contact_feedback(feedback,None,observed=True)
                if expired:self.fault('Gripper did not provide stable idle feedback within 6 seconds')
                return
            # Explicit legacy 35% initialization is retained for compatibility;
            # normal gateway attachment uses only qualified FW335 observation.
            # Serialize only result consumption and command commitment with
            # the arm writer. CAN exchange and model computation never hold it.
            with self.geometry_lock:
                skipped_brake=None
                with self.lock:
                    command=self.command_target;previous=self.last_sent
                    escaping=self.contact is not None and self.contact_requested
                    if self.braking_target is not None and previous is not None and self.actual is not None:
                        remaining=round(previous*255)-round(command*255)
                        passed=round(self.actual*255)-round(command*255)
                        if remaining*passed>0:
                            # FCL may finish after the actuator has passed the
                            # one-shot brake point. Never reverse on release:
                            # finish only the already-submitted interval.
                            skipped_brake=command
                            command=self.target=self.command_target=self.braking_target=previous
                    proposed=command
                    if self.motion_guard is not None and previous is not None and not escaping:
                        proposed=geometry_proposal(round(command*255),round(previous*255))/255
                if skipped_brake is not None:
                    self.emit(dict(event='gripper_brake_obsolete',discarded_request=skipped_brake,target=previous))
                changing=previous is None or round(proposed*255)!=round(previous*255)
                admitted=phase=='initializing' or not changing or self.motion_guard is None or self.motion_guard(self.snapshot(),proposed)
                blocked=not admitted
                if blocked!=self.collision_blocked:
                    self.emit(dict(event='gripper_collision_blocked' if blocked else 'gripper_collision_resumed',target=proposed))
                self.collision_blocked=blocked
                with self.lock:
                    if self.halt.is_set() or self.phase not in ('initializing','ready'):return
                    if self.command_target!=command:return
                    target=proposed if admitted else previous
                    if escaping and not admitted:target=None
                    self.inflight=target
                    velocity=self.velocity
                    if escaping and admitted:
                        self.contact_escape_ns=self.clock();self.contact_escape_direction=self.rate_direction
                        self.contact_escape_origin_raw=round(self.actual*255)
                        self.contact_requested=False
            # Before the first accepted motion there is no prior target to
            # repeat. Keep actual feedback fresh with the no-op observation;
            # otherwise a denied first permit would deadlock after 200 ms.
            feedback=(self.device.observe() if target is None else
                self.device.exchange(target) if velocity is None else self.device.exchange(target,velocity=velocity))
            now=self.clock()
            if feedback.get('fault',0):raise RuntimeError('Gripper device fault: '+str(feedback))
            if feedback['state'] in (2,3) and phase=='initializing':raise RuntimeError('Gripper initialization obstructed: '+str(feedback))
            with self.lock:contact_feedback=self.contact is not None or feedback['state'] in (2,3)
            if contact_feedback:
                with self.lock:
                    if target is not None:self.last_sent=target
                    self.inflight=None
                self._contact_feedback(feedback,target,observed=target is None)
                self.emit(dict(event='gripper_feedback',target=target,feedback=feedback))
                return
            with self.lock:
                self.state2_motion_verified=False
                self.actual=feedback['position_percent']/100
                self.received_ns=feedback['host_perf_counter_ns']
                self.count+=1;self.last_sent=target
                self.inflight=None
                self.last_feedback=dict(feedback)
                matches_current=target is not None and self.command_target is not None and round(self.command_target*255)==round(target*255)
                self.arrived_count=self.arrived_count+1 if matches_current and can.arrived(feedback,round(target*255)) else 0
                if self.phase=='initializing' and self.arrived_count>=3:
                    self.initialized=True;self.phase='ready'
                expired=(self.phase=='initializing' and now-self.initializing_ns>6_000_000_000)
            self.emit(dict(event='gripper_feedback',target=target,feedback=feedback))
            if expired:self.fault('初始化35%未在6秒内到位')
        except Exception as exc:self.fault(str(exc))
    def _run(self):
        try:
            while not self.halt.is_set():
                begin=time.monotonic();self.cycle()
                self.halt.wait(max(0,.05-(time.monotonic()-begin)))
        finally:
            if self.device:
                try:self.device.close()
                except Exception as exc:self.fault('CAN释放失败: '+str(exc))
    def snapshot(self):
        with self.lock:
            age=None if self.received_ns is None else (self.clock()-self.received_ns)/1e6
            valid=self.phase=='ready' and age is not None and 0<=age<=200
            target_raw=None if self.target is None else opening_raw(self.target)
            command_raw=None if self.command_target is None else round(self.command_target*255)
            sent_raw=None if self.last_sent is None else round(self.last_sent*255)
            feedback=self.last_feedback if valid else None
            arrived=None if not valid else (command_raw==sent_raw or (sent_raw is None and command_raw==self.attachment_raw)) and self.arrived_count>=3
            if valid and self.contact is not None:arrived=False
            if not valid:position_status='unknown'
            elif self.contact is not None and self.contact.get('kind')=='object_dropped':position_status='dropped'
            elif self.contact is not None:position_status='contact' if self.contact['direction']==-1 else 'obstructed'
            elif self.braking_target is not None and arrived:position_status='held'
            elif sent_raw is None and arrived:position_status='observed_idle'
            elif command_raw!=sent_raw:position_status='pending'
            elif target_raw!=command_raw:position_status='accumulating'
            elif arrived:position_status='arrived'
            elif feedback['state']==1 or (self.state2_motion_verified and feedback.get('velocity_raw',0)>0):position_status='moving'
            else:position_status='not_arrived'
            return dict(phase=self.phase,reason=self.reason,valid=valid,
                actual=self.actual if valid else None,last_observed=self.actual,age_ms=age,
                opening=stroke.opening(self.actual) if valid else None,stroke=stroke.metadata(),
                collision_blocked=self.collision_blocked,inflight_raw=None if self.inflight is None else round(self.inflight*255),
                target=self.target,last_sent=self.last_sent,initialized=self.initialized,received_ns=self.received_ns,
                target_raw=target_raw,command_raw=command_raw,sent_raw=sent_raw,effective_target=None if sent_raw is None else sent_raw/255,
                arrived=arrived,position_status=position_status,reported_state=None if feedback is None else feedback['state'],
                reported_velocity_raw=None if feedback is None else feedback.get('velocity_raw'),
                contact=None if self.contact is None else dict(self.contact,escape_requested=self.contact_requested,escape_pending=self.contact_escape_ns is not None),
                state2_motion_verified=self.state2_motion_verified,
                position_error_counts=None if feedback is None or sent_raw is None else feedback['position_raw']-sent_raw,
                target_age_ms=None if self.target_changed_ns is None else (self.clock()-self.target_changed_ns)/1e6,
                exchanges=self.count,event_drops=self.event_drops,
                tracking_mode='continuous_rate' if self.rate_active else 'position_hold',
                velocity_command=self.velocity,rate_lead_counts=RATE_LEAD_COUNTS,
                ramped_rate_s=self.ramped_rate,rate_acceleration_s2=self.rate_acceleration,
                limits=dict(minimum=MINIMUM_OPENING,maximum=MAXIMUM_OPENING,rate_s=self.rate_s,
                            minimum_update_counts=MINIMUM_UPDATE_COUNTS,
                            position_minimum_update_counts=MINIMUM_UPDATE_COUNTS,rate_minimum_update_counts=1,
                            rate_maximum_lead_counts=MAX_RATE_LEAD_COUNTS,
                            rate_acceleration_s2=RATE_ACCELERATION_S2,rate_jerk_s3=RATE_JERK_S3,
                            native_minimum_rate_s_estimate=POSITION_COUNTS_PER_VELOCITY_SECOND/255,
                            configured_current_limit_a=CONTROL_LIMITS['force']*3/255,**CONTROL_LIMITS))
    def drain(self):
        with self.lock:
            items=list(self.events);self.events.clear();return items
    def close(self):
        with self.lock:
            if self.phase!='fault':self.phase='closed'
        self.halt.set()
        if self.thread:
            self.thread.join(2)
            if self.thread.is_alive():raise RuntimeError('Gripper worker did not exit')
