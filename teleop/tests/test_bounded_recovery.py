"""Recovery must restore readiness without replaying any accepted/uncertain action."""
import numpy as np
import pytest
from e6_observer.control_core import ControlCore
from e6_observer.recovery import RecoveryGate
from test_hardware_control import Clock,Device,arm,send,tick


def rig():
    c=Clock();d=Device(c);g=ControlCore(d,'live',clock=c,recovery_policy='bounded');g.connect();arm(g,d)
    # Recovery tests isolate arm readiness; no CAN exists in this substitute.
    g.gripper_initialized=False;g.grip_target=None
    return c,d,g


def healthy(c,d,g,seconds=5.3,**input_options):
    rows=[]
    for _ in range(round(seconds/.05)):
        c.step();send(g,d,**input_options)
        rows.append(g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()}}))
    return rows


def test_timeout_auto_prepares_but_never_replays_old_or_held_motion():
    c,d,g=rig();send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0]);tick(c,d,g)
    before=d.servo_count;anchor=g.base_q.copy();old_lease=g.lease
    g.stop('Command reply deadline',fault=True)
    assert g.recovery.waiting and not g.armed and d.stop_count==1
    # Continuing to hold the stick cannot rearm, even when feedback recovers.
    healthy(c,d,g,6,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    assert not g.armed and d.servo_count==before
    with pytest.raises(ValueError,match='租约'):send(g,d,lease=old_lease)
    assert g.recovery.waiting
    rows=healthy(c,d,g)
    assert g.armed and g.recovery.state=='ready' and d.servo_count==before
    assert any(e['kind']=='automatic_prepared' for r in rows for e in r['events'])
    np.testing.assert_array_equal(anchor,g.base_q)
    np.testing.assert_allclose(g.candidate_q,d.q)
    healthy(c,d,g,1)
    assert d.servo_count==before
    current=d.q.copy()
    send(g,d,deadman=True,neutral=False,twist=[-.001,0,0,0,0,0]);result=tick(c,d,g)
    assert d.servo_count==before+1 and result['effective_action']
    assert result['effective_action']['input_seq']==g.seq
    assert result['effective_action']['joints_rad'][0]<current[0]


def test_lease_expires_during_ik_stops_before_write_and_recovers_only_from_new_neutral():
    c,d,g=rig();send(g,d,deadman=True);c.step(.231)
    # Isolate an old input from a healthy, regularly scheduled control tick.
    g.last_tick=c()-50_000_000
    original=d.ik
    def crossing(*args):
        c.step(.02);return original(*args)
    d.ik=crossing
    result=g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert g.first_fault['reason']=='输入看门狗超时' and g.recovery.waiting
    assert not g.armed and result['effective_action'] is None
    assert d.servo_count==d.grip_count==0 and d.stop_count==1
    d.ik=original
    healthy(c,d,g,6,deadman=True)
    assert not g.armed and d.servo_count==0
    healthy(c,d,g)
    assert g.armed and g.recovery.state=='ready' and d.servo_count==0


def test_fresh_but_revoked_send_qualification_still_blocks_automatic_recovery():
    c,d,g=rig();send(g,d,deadman=True);original=d.ik
    def revoked(*args):
        g.armed=False;return original(*args)
    d.ik=revoked
    result=tick(c,d,g)
    assert g.first_fault['reason']=='发送前控制资格已失效'
    assert not g.armed and g.recovery.state=='blocked'
    assert result['effective_action'] is None and d.servo_count==d.grip_count==0


@pytest.mark.parametrize('reason',[
    'E6错误、碰撞或暂停队列','Passive E6 target is not stationary',
    '关节工作区或限位','控制器IK/FK残差超限','unreachable',
    '控制连接断开','控制证据队列溢出','父进程观测/记录异常',
])
def test_unclassified_or_physical_fault_cannot_auto_prepare(reason):
    c,d,g=rig();g.stop(reason,fault=True);healthy(c,d,g,7)
    assert not g.armed and g.recovery.state=='blocked' and d.servo_count==0


def test_gripper_fault_and_operator_stop_override_transient_recovery():
    c,d,g=rig();g.gripper_faulted=True;g.stop('Command reply deadline',fault=True)
    healthy(c,d,g,6);assert not g.armed and d.grip_count==0 and g.recovery.state=='blocked'
    c,d,g=rig();g.stop('Command reply deadline',fault=True)
    send(g,d,stop=True);healthy(c,d,g,6)
    assert not g.armed and g.recovery.state=='blocked' and d.servo_count==0


def test_stop_not_acknowledged_and_stale_rgb_never_count_as_healthy():
    c,d,g=rig();g.stop('Command reply deadline',fault=True);g.stop_acknowledged=False
    healthy(c,d,g,6);assert not g.armed and g.recovery.healthy_since is None
    g.stop_acknowledged=True
    for _ in range(110):
        c.step();send(g,d)
        g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()-501_000_000}})
    assert not g.armed and g.recovery.healthy_since is None and d.servo_count==0
    healthy(c,d,g)
    assert g.armed and d.servo_count==0


@pytest.mark.parametrize('changed',[
    {'focused':False},{'connected':False},{'source':'gamepad','identity':'different'},
])
def test_changed_input_or_focus_cancels_automatic_preparation(changed):
    c,d,g=rig();g.stop('Command reply deadline',fault=True);send(g,d,**changed)
    healthy(c,d,g,6,**changed)
    assert not g.armed and g.recovery.state=='blocked'


def test_safety_change_during_recovery_locks_before_any_write():
    c,d,g=rig();g.stop('Command reply deadline',fault=True)
    state=d.state();state['decoded']['collision_state']=1
    g.tick(state,{'rgb':{'x5_receive_monotonic_ns':c()}})
    healthy(c,d,g,7)
    assert not g.armed and g.recovery.state=='blocked' and d.servo_count==0


def test_repeated_successful_recovery_is_bounded_and_never_moves():
    c,d,g=rig()
    for attempt in range(3):
        g.stop('Command reply deadline',fault=True)
        healthy(c,d,g)
        assert g.armed and g.recovery.total_attempts==attempt+1
    g.stop('Command reply deadline',fault=True);healthy(c,d,g,6)
    assert not g.armed and g.recovery.state=='blocked' and d.servo_count==0


def test_detections_do_not_extend_30_second_wait_forever():
    gate=RecoveryGate('bounded');gate.fault('RGB超过500ms',0,'owner',('keyboard','id'),1)
    for ns in range(1_000_000_000,31_000_000_000,1_000_000_000):
        gate.fault('RGB超过500ms',ns,'owner',('keyboard','id'),1)
        assert not gate.eligible(ns,False)
    assert gate.state=='blocked' and gate.fault_ns==0


def test_manual_policy_keeps_existing_no_automatic_prepare_behavior():
    c=Clock();d=Device(c);g=ControlCore(d,'live',clock=c);g.connect();arm(g,d)
    g.stop('Command reply deadline',fault=True);healthy(c,d,g,8)
    assert not g.armed and g.recovery.policy=='manual' and d.servo_count==0


def test_failed_readonly_preparation_is_retried_with_a_bounded_budget():
    c,d,g=rig();calls=[]
    def prepare(*_):calls.append(c());raise TimeoutError('Command reply deadline')
    d.prepare=prepare
    g.stop('Command reply deadline',fault=True)
    healthy(c,d,g,23)
    assert len(calls)==3 and g.recovery.state=='blocked'
    assert not g.armed and d.servo_count==d.grip_count==0 and d.stop_count==1
    assert calls[1]-calls[0]>=5_000_000_000 and calls[2]-calls[1]>=8_000_000_000


def test_failed_stop_send_cannot_schedule_auto_preparation():
    c,d,g=rig()
    def failed_stop():raise TimeoutError('Stop send deadline')
    d.stop=failed_stop;g.stop('Command reply deadline',fault=True)
    healthy(c,d,g,7)
    assert not g.armed and g.recovery.state=='blocked' and d.servo_count==0


def test_recovery_configuration_is_bound_to_operator_acceptance():
    from e6_teleop.tunnel import Tunnel
    with pytest.raises(ValueError,match='Recovery policy differs'):
        Tunnel('unused',control_role='operator',acceptance_profile={'recovery_policy':'manual','media_hz':20},recovery_policy='bounded')
    with pytest.raises(ValueError,match='Unknown recovery'):
        RecoveryGate('always')


def test_pending_uncertain_target_blocks_recovery_until_confirmed_stop():
    c,d,g=rig();g.stop('Command reply deadline',fault=True)
    g.pending={'command_id':123};g.stop_acknowledged=False
    healthy(c,d,g,6)
    assert not g.armed and g.recovery.total_attempts==0
    # Acknowledgement drains/retires the uncertain target, not its action label.
    g.pending=None;g.stop_acknowledged=True;healthy(c,d,g)
    assert g.armed and d.servo_count==0


def test_cancel_before_can_send_preserves_arm_timeout_without_false_gripper_fault():
    from e6_observer.control_core import ControlCancelled
    c,d,g=rig();g.gripper_initialized=True;g.grip_target=.2
    def cancelled(_):raise ControlCancelled('E6反馈超过100ms')
    d.grip=cancelled
    send(g,d,deadman=True);result=tick(c,d,g)
    assert not g.gripper_faulted and g.recovery.waiting
    assert result['effective_action']['gripper_acceptance']=='not_sent_due_to_control_cancellation'
    assert result['effective_action']['gripper'] is None
    healthy(c,d,g)
    assert g.armed and d.grip_count==0 and d.servo_count==1


def test_recovery_status_does_not_archive_controller_capability():
    gate=RecoveryGate('bounded');gate.fault('Command reply deadline',1,'private-owner',('keyboard','id'),5)
    assert gate.origin['owner']=='private-owner'
    assert 'private-owner' not in str(gate.status())
