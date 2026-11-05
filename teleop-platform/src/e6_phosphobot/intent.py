"""One control session: latest intent, actual device state, execution receipts.

Only the device-side owner enforces freshness. Verified clock exchanges bound
the original input's age; a server-issued echo remains the unsynchronized
fallback. Unrelated host monotonic clocks are never directly subtracted.
"""
from dataclasses import dataclass
import math
import time
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from .clock_probe import InputAgeBounds


class Intent(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    linear_m_s: list[float] = Field(min_length=3, max_length=3)
    angular_rad_s: list[float] = Field(min_length=3, max_length=3)
    joint_rad_s: list[float] = Field(min_length=6, max_length=6)
    gripper_rate_s: float
    deadman: bool
    frame: Literal['base','tool']
    orientation_assist: bool = Field(default=False, strict=True)
    assist_angular_rad_s: float = Field(default=0., ge=0, le=math.radians(3))

    def centered(self):
        return not self.deadman and all(abs(x)<1e-9 for x in
            self.linear_m_s+self.angular_rad_s+self.joint_rad_s+[self.gripper_rate_s])


class ResearchContext(BaseModel):
    model_config = ConfigDict(extra='forbid',strict=True)
    scene: str = Field(default='',max_length=500)
    operator: str = Field(default='',max_length=80)


class Message(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    op: Literal['intent','prepare','stop','arm_enable','gripper_initialize','record_start','record_finish','camera_restart']
    seq: int = Field(ge=1)
    echo_ns: int = Field(ge=0)
    client_sample_ms: float = Field(ge=0)
    source: Literal['keyboard','gamepad']
    intent: Intent | None = None
    reason: str | None = Field(default=None,max_length=128)
    task: str | None = Field(default=None,max_length=500)
    result: Literal['success','failure','aborted'] | None = None
    research_context: ResearchContext | None = None


@dataclass(frozen=True)
class Limits:
    linear_m_s: float = .002
    angular_rad_s: float = math.radians(1)
    joint_rad_s: float = math.radians(1)
    gripper_rate_s: float = .025

    def accepts(self, intent):
        return (math.hypot(*intent.linear_m_s)<=self.linear_m_s+1e-9 and
                math.hypot(*intent.angular_rad_s)<=self.angular_rad_s+1e-9 and
                max(map(abs,intent.joint_rad_s))<=self.joint_rad_s+1e-9 and
                abs(intent.gripper_rate_s)<=self.gripper_rate_s+1e-9 and
                not (any(intent.joint_rad_s) and any(intent.linear_m_s+intent.angular_rad_s)) and
                (not intent.orientation_assist or (not any(intent.joint_rad_s+intent.angular_rad_s)
                  and 0<intent.assist_angular_rad_s<=self.angular_rad_s+1e-9)) and
                (intent.orientation_assist or intent.assist_angular_rad_s==0))


class IntentSession:
    def __init__(self, limits=Limits(), clock=time.perf_counter_ns):
        self.limits=limits
        self.clock=clock
        self.sequence=0
        self.intent_sequence=None
        self.intent=None
        self.received_ns=0
        self.source=None
        self.ready=False
        self.input_paused_ns=None
        self.paused_direction=None
        self.reason=None
        self.stop_requested=False
        self.dropped=0
        self.issued_ticks=set()
        self.input_age_bounds=InputAgeBounds()
        self.last_freshness=None

    def observe_clock(self, verified_sample):
        # Called only with a ClockProbes.complete result for this controller.
        # It does not update received_ns, readiness, input, or stop state.
        self.input_age_bounds.add(verified_sample)

    def clock_tick(self):
        now=self.clock()
        self.issued_ticks={x for x in self.issued_ticks if now-x<=250_000_000}
        self.issued_ticks.add(now)
        return now

    def suspend(self, reason):
        self.ready=False
        self.input_paused_ns=None
        self.paused_direction=None
        self.intent=None
        self.reason=reason
        self.stop_requested=True

    @staticmethod
    def direction(intent):
        if intent is None or not intent.deadman:return None
        values=intent.linear_m_s+intent.angular_rad_s+intent.joint_rad_s+[intent.gripper_rate_s]
        if not any(values):return None
        return (intent.frame,intent.orientation_assist,
                tuple(1 if x>0 else -1 if x<0 else 0 for x in values))

    def pause_input(self, reason, *, allow_direction_change=False):
        """Keep preparation; recoverable rejection also accepts a new direction.

        Input expiry and policy changes still require post-pause neutral.
        A changed speed alone cannot retry a held, refused direction.
        """
        if not self.ready:return
        if self.input_paused_ns is None:
            self.input_paused_ns=self.clock()
            self.paused_direction=self.direction(self.intent) if allow_direction_change else None
        elif not allow_direction_change:
            self.paused_direction=None
        self.intent=None
        self.intent_sequence=None
        self.reason=reason
        self.stop_requested=True

    def receive(self, data):
        message=Message.model_validate(data)
        if message.seq<=self.sequence:
            self.dropped+=1
            return False
        self.sequence=message.seq
        # Stops always take precedence, even when an old tick is echoed.
        if message.op=='stop':
            self.suspend(message.reason or 'operator_stop')
            return True
        if message.op=='record_finish':
            if message.result is None:raise ValueError('Recording result is required')
            # Ending an owned episode must remain possible after input expiry.
            return True
        # Enforce the old input deadline before a newly arrived packet can renew it.
        self.active()
        now=self.clock()
        age=self.input_age_bounds.bound(message.client_sample_ms,now)
        echo_fresh=(message.echo_ns in self.issued_ticks and 0<=now-message.echo_ns<=250_000_000)
        bounded_fresh=age is not None and age['upper_ms']<=250
        self.last_freshness=dict(source='clock_bound' if bounded_fresh else 'echo' if echo_fresh else 'expired',
                                 echo_age_ms=(now-message.echo_ns)/1e6, age_bound=age)
        if not bounded_fresh and not echo_fresh:
            # Discard a delayed packet without revoking a still-fresh accepted
            # intent. It cannot renew that intent's original 250 ms deadline.
            # A release is actionable even with a stale echo: clear work now,
            # but do not use the unverified packet to refresh readiness/age.
            if message.op=='intent' and message.intent is not None and not message.intent.deadman:
                if self.ready and self.intent is not None and self.intent.deadman:
                    self.stop_requested=True
                self.intent=None
                self.intent_sequence=None
            else:
                self.active()  # Expired accepted input still suspends immediately.
            self.dropped+=1
            return False
        if self.source is not None and message.source!=self.source:
            self.suspend('input_source_changed')
            return False
        self.source=message.source
        if self.input_paused_ns is not None and message.op!='intent':return False
        if message.op in ('prepare','arm_enable','gripper_initialize','record_start','camera_restart'):
            if self.intent is None or not self.intent.centered() or now-self.received_ns>250_000_000:
                self.reason='请释放允许动作并回中'
                return False
            if message.op=='prepare':self.ready=True
            self.reason=None
            return True
        if message.intent is None:
            raise ValueError('intent operation requires an intent')
        if not self.limits.accepts(message.intent):
            self.suspend('input_exceeds_capabilities')
            return False
        if self.input_paused_ns is not None:
            # A queued neutral sampled before the pause cannot acknowledge it.
            # An issued post-pause echo proves the browser has seen fresh state.
            post_pause_echo=echo_fresh and message.echo_ns>self.input_paused_ns
            changed_direction=(self.paused_direction is not None and
                self.direction(message.intent) is not None and
                self.direction(message.intent)!=self.paused_direction)
            if not (message.intent.centered() or changed_direction) or not post_pause_echo:
                self.dropped+=1
                return False
            self.input_paused_ns=None
            self.paused_direction=None
            self.reason=None
        previous=self.intent
        if (self.ready and previous is not None and previous.deadman and message.intent.deadman and
            (previous.orientation_assist,previous.frame)!=(message.intent.orientation_assist,message.intent.frame)):
            self.pause_input('control_policy_changed')
            return False
        self.intent=message.intent
        self.intent_sequence=message.seq
        self.received_ns=now
        if self.ready and previous is not None and previous.deadman and not self.intent.deadman:
            # Normal release preserves the prepared session, but must stop device motion.
            self.stop_requested=True
        return True

    def active(self):
        if self.input_paused_ns is not None:return None
        if self.ready and self.clock()-self.received_ns>250_000_000:
            # Stop on the original deadline. Keep ownership/preparation, but
            # require a newly sampled neutral before any later action. Fresh
            # held input never resumes, and socket/fault stops still disarm.
            if self.intent is not None and self.intent.deadman:
                self.pause_input('input_stale')
            return None
        return self.intent if self.ready and self.intent is not None and self.intent.deadman else None
