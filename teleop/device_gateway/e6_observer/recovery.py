"""Bounded readiness recovery, never a queue or replay of device commands."""
from collections import deque

TRANSIENT = {
    'Command reply deadline', 'Command deadline expired before send',
    'Local IK deadline', 'Local FK deadline', 'IK/FK事务超过50ms',
    '控制事务超过50ms，命令已接受后停止', '含CAN的控制事务超过50ms',
    'IK/FK事务超过75ms', '控制事务超过75ms，命令已接受后停止', '含CAN的控制事务超过75ms',
    '控制循环超过周期预算', '守护控制循环超过预算',
    '输入看门狗超时', 'E6反馈超过100ms', 'RGB超过500ms',
}


class RecoveryGate:
    def __init__(self,policy='manual'):
        if policy not in ('manual','bounded'):raise ValueError('Unknown recovery policy')
        self.policy=policy;self.state='idle';self.reason=None;self.origin=None
        self.fault_ns=None;self.healthy_since=None;self.attempts=deque()
        self.total_attempts=0;self.next_attempt_ns=0
        self.healthy_s=5;self.cooldown_s=2;self.window_s=60;self.max_attempts=3

    @property
    def waiting(self):return self.state=='waiting'

    def status(self):
        return dict(policy=self.policy,state=self.state,reason=self.reason,
            origin=None if self.origin is None else {k:v for k,v in self.origin.items() if k!='owner'},
            fault_ns=self.fault_ns,healthy_since_ns=self.healthy_since,
            attempts_in_window=len(self.attempts),total_attempts=self.total_attempts,
            max_attempts=self.max_attempts,window_s=self.window_s,healthy_s=self.healthy_s,
            cooldown_s=self.cooldown_s,next_attempt_ns=self.next_attempt_ns,
            replays_old_targets=False,requires_fresh_neutral=True,requires_new_deadman_press=True)

    def fault(self,reason,now,owner,source,seq,hard=False):
        if self.policy!='bounded':return
        if hard or reason not in TRANSIENT:
            self.block(reason);return
        if self.waiting:return  # Repeated detection cannot reset its expiry/backoff.
        if self.state=='blocked':return
        self.state='waiting';self.reason='等待Stop确认、静止及新回中输入'
        self.origin=dict(reason=reason,owner=owner,source=source,seq=seq)
        self.fault_ns=now;self.healthy_since=None;self.next_attempt_ns=now+2_000_000_000

    def block(self,reason):
        self.state='blocked';self.reason=reason;self.healthy_since=None

    def manual_prepared(self):
        self.state='idle';self.reason=None;self.origin=None;self.healthy_since=None

    def eligible(self,now,healthy):
        if not self.waiting:return False
        while self.attempts and now-self.attempts[0]>=60_000_000_000:self.attempts.popleft()
        if now-self.fault_ns>=30_000_000_000:
            self.block('自动恢复等待超过30秒，请检查后重新准备');return False
        if len(self.attempts)>=self.max_attempts:
            self.block('60秒内自动恢复已尝试3次，停止自动恢复');return False
        if not healthy:self.healthy_since=None;return False
        if self.healthy_since is None:self.healthy_since=now
        return now>=self.next_attempt_ns and now-self.healthy_since>=5_000_000_000

    def begin_attempt(self,now):
        self.attempts.append(now);self.total_attempts+=1;self.healthy_since=None
        self.next_attempt_ns=now+min(8,2**(len(self.attempts)+1))*1_000_000_000

    def prepared(self):
        self.state='ready';self.reason='已自动准备，重新按住允许动作键后使用最新输入'
