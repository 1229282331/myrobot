"""The isolated diagnostic is tested with packets only; no device IO."""
import importlib.util
from pathlib import Path
import pytest
import time
import base64
import struct
import threading
from collections import deque
from omnipicker.tools import verify_can_control as can

spec=importlib.util.spec_from_file_location('gripper_isolation',Path(__file__).resolve().parents[1]/'scripts/gripper_isolation_diagnostic.py')
diagnostic=importlib.util.module_from_spec(spec);spec.loader.exec_module(diagnostic)


class Clock:
    def __init__(self):self.now=0
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds


@pytest.mark.parametrize('fault',['stall','missing','dropped','arm-feedback','rgb-feedback'])
def test_first_anomaly_ends_trial_without_another_command(fault):
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command)
            if fault=='missing':raise TimeoutError('CAN reply missing')
            data=bytes.fromhex('0002460a17190000') if fault=='stall' else bytes.fromhex('0003460a17190000')
            return can.decode_feedback(can.FRAME.pack(1,8,data),0,1,command)
    def check():
        if fault=='arm-feedback':raise ValueError('E6 feedback stale')
        if fault=='rgb-feedback':raise ValueError('RGB stale')
    with pytest.raises((can.CheckFailed,TimeoutError,ValueError)):
        diagnostic.trial(Channel(),report,check,clock=clock,sleep=clock.sleep)
    assert len(calls)==(0 if fault in ('arm-feedback','rgb-feedback') else 1)
    assert not report['feedback']


def test_fixed_goal_does_not_increase_force_or_create_arm_commands():
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command);return dict(state=0,position_raw=89)
    diagnostic.trial(Channel(),report,lambda:None,clock=clock,sleep=clock.sleep)
    assert len(calls)>=120 and set(calls)=={bytes.fromhex('0059052019190000')}
    assert diagnostic.plan()['e6_writes']==0 and diagnostic.plan()['initialize'] is False


@pytest.mark.parametrize('age,valid,raises',[(100,True,False),(501,True,True),(100,False,True),(-1,True,True)])
def test_rgb_guard_preserves_the_500ms_limit(age,valid,raises):
    rgb=object.__new__(diagnostic.PassiveRgb);rgb.error=None
    rgb.latest=dict(valid=valid,x5_receive_monotonic_ns=time.perf_counter_ns()-round(age*1e6))
    if raises:
        with pytest.raises(ValueError,match='RGB'):rgb.check()
    else:assert rgb.check() is rgb.latest


def test_rejected_arm_packet_survives_without_becoming_valid_feedback():
    from e6_observer.protocol import SIZE
    raw=bytearray(SIZE)
    struct.pack_into('<Q',raw,0,SIZE)
    struct.pack_into('<Q',raw,24,5)
    struct.pack_into('<Q',raw,48,0x123456789ABCDEF)
    raw[1026]=1
    struct.pack_into('<d',raw,432,.06)
    arm=object.__new__(diagnostic.PassiveArm)
    arm.latest=None;arm.error=None;arm.history=deque(maxlen=2048)
    arm.closed=threading.Event();arm.lock=threading.Lock()
    class Socket:
        def recv(self,_):return bytes(raw)
    arm.sock=Socket();arm.receive()
    evidence=arm.evidence()
    assert 'target is not stationary' in evidence['error']
    assert len(evidence['frames'])==1
    assert base64.b64decode(evidence['latest']['raw_base64'])==raw
    assert evidence['latest']['decoded']['q_actual_deg'][0]==.06
    with pytest.raises(RuntimeError,match='stationary'):arm.check()


def test_owned_socket_shutdown_is_not_a_new_device_fault():
    arm=object.__new__(diagnostic.PassiveArm)
    arm.error=None;arm.closed=threading.Event()
    class Socket:
        def recv(self,_):
            arm.closed.set()
            raise OSError(9,'Bad file descriptor')
    arm.sock=Socket();arm.receive()
    assert arm.error is None


def test_fixed_cycle_covers_return_without_increasing_force():
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command);return dict(state=0,position_raw=command[1])
    p=diagnostic.plan(True)
    for target in p['targets_percent']:
        diagnostic.trial(Channel(),report,lambda:None,clock=clock,sleep=clock.sleep,target=target,limits=p['limits_raw'])
    assert [calls[i][1] for i in (0,121,242)]==[38,89,38]
    assert all(c[2:6]==bytes([10,32,25,25]) for c in calls)


def test_slow_diagnostic_ramp_limits_steps_and_aborts_at_first_fault():
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command)
            if len(calls)==5:raise can.CheckFailed('stalled')
            return dict(state=0,position_raw=command[1])
    limits=diagnostic.plan(True,64,True)['limits_raw']
    with pytest.raises(can.CheckFailed,match='stalled'):
        diagnostic.slow_ramp(Channel(),report,lambda:None,15,35,limits,clock=clock,sleep=clock.sleep)
    assert len(calls)==5 and len(report['ramp_feedback'])==4
    assert max(b[1]-a[1] for a,b in zip(calls,calls[1:]))<=1
    assert all(c[3]==64 for c in calls)


def test_medium_cycle_keeps_force32_and_bounds_target_ramp():
    p=diagnostic.plan(True,medium=True)
    assert p['limits_raw']==dict(velocity=25,force=32,acceleration=25,deceleration=25)
    assert p['ramp_percent_s']==10 and p['duration_s']==22
    assert p['e6_writes']==0 and p['automatic_retry'] is False
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command);return dict(state=0,position_raw=command[1])
    diagnostic.slow_ramp(Channel(),report,lambda:None,15,35,p['limits_raw'],clock=clock,sleep=clock.sleep,rate=10)
    assert len(calls)==40 and clock.now==pytest.approx(2)
    assert calls[-1][1]==89 and all(c[2:6]==bytes([25,32,25,25]) for c in calls)
    assert max(b[1]-a[1] for a,b in zip(calls,calls[1:]))<=2


@pytest.mark.parametrize('kwargs',[dict(medium=True),dict(cycle=True,ramp=True,medium=True)])
def test_medium_profile_rejects_ambiguous_or_higher_force_configuration(kwargs):
    with pytest.raises(ValueError,match='Medium speed'):
        diagnostic.plan(**kwargs)


@pytest.mark.parametrize('rate',[0,-1,10.01,float('nan'),float('inf')])
def test_ramp_rejects_invalid_rate_before_device_io(rate):
    class Channel:
        def exchange(self,_):raise AssertionError('No CAN exchange permitted')
    with pytest.raises(ValueError,match='ramp rate'):
        diagnostic.slow_ramp(Channel(),{},lambda:None,15,35,diagnostic.LIMITS,rate=rate)


def test_medium_ramp_fault_cancels_remaining_targets():
    calls=[];clock=Clock();report={}
    class Channel:
        def exchange(self,command):
            calls.append(command)
            if len(calls)==5:raise can.CheckFailed('stalled')
            return dict(state=0,position_raw=command[1])
    with pytest.raises(can.CheckFailed,match='stalled'):
        diagnostic.slow_ramp(Channel(),report,lambda:None,15,35,diagnostic.plan(True,medium=True)['limits_raw'],clock=clock,sleep=clock.sleep,rate=10)
    assert len(calls)==5 and len(report['ramp_feedback'])==4
    assert all(c[3]==32 for c in calls)


def arm_state(error=0,now=1_000_000_000,axis=5):
    q=[0.]*6;q[axis]=error
    return dict(x5_receive_monotonic_ns=now,decoded=dict(robot_mode=5,enable_status=1,error_status=0,
        collision_state=0,run_queued_command=0,user_index=0,tool_index=0,
        tool_configuration_raw_mm_deg=[0.]*6,user_configuration_raw_mm_deg=[0.]*6,
        q_actual_deg=q,q_target_deg=[0.]*6))


def test_transient_guard_records_a_short_spike_and_recovery_without_stop():
    guard=diagnostic.TransientArmGuard([0.]*6)
    assert guard.observe(arm_state(.102))['decision']=='warning'
    assert guard.observe(arm_state(.003,1_008_000_000))['decision']=='normal'
    assert guard.observe(arm_state(.102,1_100_000_000))['decision']=='warning'
    assert guard.observe(arm_state(.003,1_108_000_000))['decision']=='normal'


@pytest.mark.parametrize('metric',['target','baseline'])
def test_transient_guard_stops_persistent_deviation_at_100ms(metric):
    guard=diagnostic.TransientArmGuard([0.]*6)
    def state(now):
        s=arm_state(.06,now)
        if metric=='baseline':s['decoded']['q_target_deg']=s['decoded']['q_actual_deg'].copy()
        return s
    assert guard.observe(state(1_000_000_000))['decision']=='warning'
    assert guard.observe(state(1_099_000_000))['decision']=='warning'
    result=guard.observe(state(1_100_000_000))
    assert result['decision']=='stop' and metric in result['reason']


def test_transient_guard_hard_deviation_stops_on_first_packet():
    result=diagnostic.TransientArmGuard().observe(arm_state(.251))
    assert result['decision']=='stop' and 'hard bound' in result['reason']


def test_transient_guard_tracks_each_joint_separately():
    guard=diagnostic.TransientArmGuard()
    assert guard.observe(arm_state(.06,1_000_000_000,axis=0))['decision']=='warning'
    assert guard.observe(arm_state(.06,1_100_000_000,axis=5))['decision']=='warning'
    assert guard.observe(arm_state(.06,1_200_000_000,axis=5))['decision']=='stop'


@pytest.mark.parametrize('field,value',[('robot_mode',11),('enable_status',0),('error_status',1),('collision_state',1),('run_queued_command',1),('tool_index',1)])
def test_transient_policy_does_not_soften_device_faults_or_configuration_changes(field,value):
    s=arm_state();s['decoded'][field]=value
    with pytest.raises(ValueError):diagnostic.TransientArmGuard().observe(s)


def test_transient_guard_rejects_repeated_receive_timestamp():
    guard=diagnostic.TransientArmGuard();guard.observe(arm_state())
    with pytest.raises(ValueError,match='timestamp'):guard.observe(arm_state())


@pytest.mark.parametrize('offset,state,passes',[(3,0,True),(5,0,True),(6,0,False),(3,1,False)])
def test_relaxed_endpoint_still_requires_normal_settled_feedback_within_two_percent(offset,state,passes):
    clock=Clock();report={}
    class Channel:
        def exchange(self,command):return dict(state=state,position_raw=command[1]+offset)
    if passes:
        diagnostic.trial(Channel(),report,lambda:None,clock=clock,sleep=clock.sleep,settle_tolerance_counts=5)
    else:
        with pytest.raises(ValueError,match='three normal'):
            diagnostic.trial(Channel(),report,lambda:None,clock=clock,sleep=clock.sleep,settle_tolerance_counts=5)


def test_large_twenty_percent_error_cannot_be_accepted_as_a_settled_endpoint():
    clock=Clock();report={}
    class Channel:
        def exchange(self,_):return dict(state=0,position_raw=88)
    with pytest.raises(ValueError,match='three normal'):
        diagnostic.trial(Channel(),report,lambda:None,clock=clock,sleep=clock.sleep,target=15,settle_tolerance_counts=5)


def test_force64_comparison_changes_only_the_force_and_encoded_force_byte():
    before=diagnostic.plan(True,force=32,medium=True,arm_guard='transient',settle_tolerance_counts=5)
    after=diagnostic.plan(True,force=64,medium=True,arm_guard='transient',settle_tolerance_counts=5)
    for name in before:
        if name not in ('limits_raw','command_hex','commands_hex'):assert before[name]==after[name]
    assert dict(before['limits_raw'],force=64)==after['limits_raw']
    for old,new in zip(before['commands_hex'],after['commands_hex']):
        old=bytes.fromhex(old);new=bytes.fromhex(new)
        assert new==old[:3]+bytes([64])+old[4:]


def test_open_start_remains_bounded_and_does_not_ignore_first_stalled_reply():
    p=diagnostic.plan(True,force=64,medium=True,arm_guard='transient',settle_tolerance_counts=5,start_open=True)
    assert p['targets_percent']==[35,15,35,15] and p['duration_s']==30
    assert p['automatic_retry'] is False and 'first reply' in p['stall_policy']
    calls=[];clock=Clock()
    class Channel:
        def exchange(self,command):
            calls.append(command);raise can.CheckFailed('Gripper stalled')
    with pytest.raises(can.CheckFailed,match='stalled'):
        diagnostic.trial(Channel(),{},lambda:None,clock=clock,sleep=clock.sleep,target=35,limits=p['limits_raw'])
    assert len(calls)==1 and calls[0]==bytes.fromhex('0059194019190000')


def test_open_start_requires_a_declared_cycle():
    with pytest.raises(ValueError,match='Open-start'):diagnostic.plan(start_open=True)
