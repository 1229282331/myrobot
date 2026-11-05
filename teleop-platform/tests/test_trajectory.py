import math
import numpy as np
import pytest
from e6_phosphobot.trajectory import VelocityTrajectory
from test_gateway import setup


@pytest.mark.parametrize('kind,axis,speed',[('cartesian',0,.002),('cartesian',5,math.radians(1)),('joint',5,math.radians(1))])
def test_ruckig_velocity_stop_limits_and_no_position_jump(kind,axis,speed):
    trajectory=VelocityTrajectory(.05,kind)
    velocity=np.zeros(6);velocity[axis]=speed
    acceleration=[];deltas=[]
    for _ in range(20):
        deltas.append(trajectory.step(velocity))
        acceleration.append(trajectory.input.current_acceleration)
    assert trajectory.input.current_velocity[axis]==pytest.approx(speed)
    stops=[]
    for _ in range(10):
        stops.append(trajectory.step([0]*6))
        acceleration.append(trajectory.input.current_acceleration)
        if trajectory.stopped:break
    assert trajectory.stopped and len(stops)<=5
    assert np.min(np.asarray(stops)[:,axis])>=-1e-12
    assert sum(x[axis] for x in stops)<=speed*.25
    np.testing.assert_allclose(trajectory.step([0]*6),0,atol=1e-12)
    a=np.array(acceleration)
    assert np.all(np.abs(a)<=np.array(trajectory.input.max_acceleration)+1e-9)
    assert np.all(np.abs(np.diff(a,axis=0))/.05<=np.array(trajectory.input.max_jerk)+1e-9)


def smooth_setup():
    e,d,send,step,now=setup();e.release_mode='ruckig'
    for _ in range(20):send(deadman=True,linear_m_s=[.002,0,0]);step()
    return e,d,send,step,now


def test_normal_release_brakes_but_fault_bypasses_it():
    e,d,send,step,_=smooth_setup()
    send();assert e.braking and e.session.ready
    assert any(x['event']=='release_deceleration_requested' for x in e.events)
    send();step()
    assert d.calls[-1][0]=='ServoP' and e.candidate['braking']
    assert not e.candidate['source_input']['intent']['deadman']
    d.data['status']['collision_state']=1;step()
    assert d.calls[-1][0]=='Stop' and not e.session.ready and e.trajectory is None


def test_release_keeps_gripper_and_zero_hold_from_creating_arm_motion():
    e,d,send,step,_=setup();e.release_mode='ruckig'
    for _ in range(10):send(deadman=True,gripper_rate_s=.01);step()
    send();step()
    assert not d.calls


def test_finished_release_sends_stop_once_preserves_ready_and_accepts_new_input():
    e,d,send,step,_=smooth_setup()
    send()
    for _ in range(12):send();step()
    assert len([x for x in d.calls if x[0]=='Stop'])==1
    assert e.session.ready and e.trajectory is None
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert d.calls[-1][0]=='ServoP'


def test_release_with_pending_ack_never_queues_deceleration_and_loss_stops():
    e,d,send,step,_=smooth_setup();d.auto_ack=False
    send(deadman=True,linear_m_s=[.002,0,0]);step();before=len(d.calls)
    send();step(2)
    assert len(d.calls)==before
    step(6)
    assert d.calls[-1][0]=='Stop' and not e.session.ready


def test_stale_release_keeps_absent_input_explicit_during_optional_braking():
    e,d,send,step,_=smooth_setup()
    message=dict(op='intent',seq=e.session.sequence+1,echo_ns=0,source='keyboard',client_sample_ms=0,
                 intent=dict(linear_m_s=[0]*3,angular_rad_s=[0]*3,joint_rad_s=[0]*6,
                             gripper_rate_s=0,deadman=False,frame='base'))
    e.receive(message);step()
    assert e.braking and e.candidate['source_input']['intent'] is None
    step(6)
    assert not e.session.ready and d.calls[-1][0]=='Stop'


def test_mode_switch_during_deceleration_does_not_reset_velocity():
    e,d,send,step,_=smooth_setup();send()
    send(deadman=True,joint_rad_s=[0,0,0,0,0,.01]);step()
    assert e.braking and d.calls[-1][0]=='ServoP'


def test_expired_input_arriving_between_ticks_bypasses_experimental_release_ramp():
    e,d,send,step,now=setup();e.release_mode='ruckig'
    for _ in range(10):send(deadman=True,linear_m_s=[.002,0,0]);step()
    now[0]+=300_000_000
    send(deadman=True,linear_m_s=[.002,0,0])
    assert d.calls[-1][0]=='Stop' and not e.braking and e.trajectory is None
    assert e.session.ready and e.session.input_paused_ns is not None


def test_shadow_runs_same_release_trajectory_without_motion():
    e,d,send,step,_=setup('shadow');e.release_mode='ruckig'
    for _ in range(10):send(deadman=True,linear_m_s=[.002,0,0]);step()
    send();step();assert e.candidate['braking'] and not d.calls


def test_tool_frame_and_quaternion_boundary_use_base_twist():
    from scipy.spatial.transform import Rotation
    e,d,send,step,_=setup();e.release_mode='ruckig'
    e.target=(e.target[0],Rotation.from_euler('z',179,degrees=True),e.target[2])
    e.base=e.target
    for _ in range(30):
        send(deadman=True,angular_rad_s=[0,0,.01],linear_m_s=[.001,0,0],frame='tool');step()
    assert e.candidate['position_m'][0]<0
    assert abs(np.linalg.norm(e.candidate['quaternion_xyzw'])-1)<1e-12
    assert (e.target[1]*e.base[1].inv()).magnitude()<.02
