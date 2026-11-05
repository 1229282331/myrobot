"""A confirmed settled Stop cancels its endpoint, never a live motion target."""
import numpy as np
import pytest
from types import SimpleNamespace
from test_gateway import setup
from test_moveit_bridge import Bridge,row


def stopped_case():
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal')
    e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.015,0,0])
    e.joint_tracking.committed([.8,0,0,0,.7,0],now[0])
    e.moving=True
    # Stop legitimately cancels an endpoint still ahead of QTarget/QActual.
    send();step()
    return e,d,send,step,now


def test_confirmed_settled_stop_forgets_only_the_cancelled_endpoint():
    e,d,send,step,now=stopped_case()
    for _ in range(8):send();step()
    assert e.stop_acknowledged and e.stationary and e.session.ready
    assert e.joint_tracking.last_sent is None
    assert any(x['event']=='joint_tracking_rebased' for x in e.events)
    send(deadman=True,linear_m_s=[.015,0,0])
    e.moveit.row=row(now[0],e.session.sequence);step()
    assert [x[0] for x in d.calls]==['Stop','ServoJ']
    assert not any(x['event']=='joint_tracking_stopped' for x in e.events)


@pytest.mark.parametrize('condition',['moving','residual_target','alarm','stale'])
def test_stop_receipt_alone_cannot_reset_unsettled_or_unhealthy_feedback(condition):
    e,d,send,step,now=stopped_case()
    if condition=='moving':d.data['status']['robot_mode']=7
    if condition=='residual_target':d.data['status']['joint_state']['q_target'][0]=2.01
    if condition=='alarm':d.data['status']['collision_state']=1
    if condition=='stale':d.data['age_ms']=101
    for _ in range(8):send();step()
    assert e.joint_tracking.last_sent is not None
    assert not any(x['event']=='joint_tracking_rebased' for x in e.events)
    assert not any(x[0]=='ServoJ' for x in d.calls)


def test_unacknowledged_stop_does_not_rebase_or_allow_restart():
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal')
    e.moveit=Bridge();send(deadman=True,linear_m_s=[.015,0,0])
    e.joint_tracking.committed([.8,0,0,0,.7,0],now[0]);e.moving=True
    d.auto_ack=False;send()
    for _ in range(8):send();step()
    assert e.joint_tracking.last_sent is not None and e.stop_pending is not None
    assert not any(x['event']=='joint_tracking_rebased' for x in e.events)


def test_settled_stop_does_not_allow_an_initial_jump():
    e,d,send,step,now=stopped_case()
    for _ in range(8):send();step()
    send(deadman=True,linear_m_s=[.015,0,0])
    target=row(now[0],e.session.sequence);target['joints_rad'][0]=np.deg2rad(.5)
    e.moveit.row=target;step()
    assert not any(x[0]=='ServoJ' for x in d.calls)
    assert e.joint_tracking.last_detail['reason']=='joint_command_discontinuity'


@pytest.mark.parametrize('offset',[-2.,-1.473,-1.,-.46,.46,1.,1.473,2.])
def test_loaded_stopped_prepare_and_gripper_release_do_not_write_arm(offset):
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal')
    e.moveit=Bridge()
    e.session.ready=False
    d.data['status']['joint_state']['q_actual'][3]=offset
    for _ in range(105):send();step()
    send('prepare')
    assert e.session.ready
    assert np.rad2deg(e.target[2][3])==pytest.approx(offset)
    rates=[]
    e.gripper=SimpleNamespace(snapshot=lambda:dict(phase='ready'),
        hold=lambda:None,set_stroke_rate=lambda rate,dt:rates.append(rate))
    for _ in range(12):
        send(deadman=True,gripper_rate_s=.02);step()
    send();step()
    assert len(rates)==12 and all(rate==.02 for rate in rates)
    assert not d.calls
    assert e.session.ready
    # Opening releases the external load; a new stationary actual becomes the
    # first-write origin without an arm command or another manual Prepare.
    d.data['status']['joint_state']['q_actual'][3]=0.
    for _ in range(12):send();step()
    assert e.joint_tracking.rebased_actual[3]==0.
    assert not d.calls and e.session.ready


def test_prepare_rejection_names_joint_measured_offset_and_limit():
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal')
    e.moveit=Bridge();e.session.ready=False
    d.data['status']['joint_state']['q_target'][3]=2.2
    send('prepare')
    assert not e.session.ready
    assert 'J4' in e.session.reason and '2.20°' in e.session.reason and '2.0°' in e.session.reason
    assert not d.calls
