"""Active-input Ruckig qualification; no physical device connections."""
import json
import math
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.launcher import gateway_ready
from e6_phosphobot.motion_profiles import get_profile
from e6_phosphobot.session_runner import motion_engine_option
from e6_phosphobot.trajectory import VelocityTrajectory
from test_gateway import setup


@pytest.mark.parametrize('profile', ['commissioning', 'intermediate', 'normal'])
@pytest.mark.parametrize('kind', ['cartesian', 'joint'])
def test_profile_ramps_reversals_and_random_turns_stay_in_velocity_envelope(profile, kind):
    p = get_profile(profile)
    limits = np.array([p.linear_m_s]*3 + [math.radians(p.angular_deg_s)]*3
                      if kind == 'cartesian' else [math.radians(p.input_joint_deg_s)]*6)
    t = VelocityTrajectory(.05, kind, motion_profile=profile)
    previous = np.zeros(6)
    rng = np.random.default_rng(130)
    targets = [limits*np.array([1,0,0,0,0,0])]*10
    targets += [-targets[0]]*10
    targets += [limits*.25*np.array([0,1,0,0,0,0])]*10
    for _ in range(1000):
        v = rng.normal(size=6)
        if kind == 'cartesian':
            v[:3] /= np.linalg.norm(v[:3]); v[3:] /= np.linalg.norm(v[3:])
        else:
            v /= np.max(np.abs(v))
        targets.append(v*limits)
    for target in targets:
        delta = t.step(target)
        current = np.array(t.input.current_velocity)
        assert np.all(np.abs(current-previous)/.05 <= np.array(t.input.max_acceleration)+1e-9)
        # Time synchronization must preserve the convex speed envelope even
        # with changing acceleration. Check the full upstream trajectory,
        # not only period endpoints, and never clip individual channels.
        for at in np.linspace(0, .05, 11):
            _, v, _ = t.output.trajectory.at_time(at)
            normalized = np.asarray(v)/limits
            if kind == 'cartesian':
                assert np.linalg.norm(normalized[:3]) <= 1+1e-9
                assert np.linalg.norm(normalized[3:]) <= 1+1e-9
            else:
                assert max(abs(normalized)) <= 1+1e-9
        assert np.all(np.isfinite(delta))
        previous = current
    json.dumps(t.state(), allow_nan=False)
    assert t.state()['order'] == 'acceleration'
    assert t.state()['jerk_limits'] is None


@pytest.mark.parametrize('release', ['deadman', 'centered', 'fault', 'expired', 'disconnect'])
def test_release_and_faults_discard_trajectory_without_residual_motion(release):
    e,d,send,step,now = setup(native_smoothing='ruckig')
    for _ in range(6): send(deadman=True,linear_m_s=[.002,0,0]); step()
    before = len(d.calls)
    if release == 'deadman': send()
    elif release == 'centered': send(deadman=True); step()
    elif release == 'fault': d.data['status']['collision_state']=1; step()
    elif release == 'expired': now[0]+=251_000_000;step()
    else: e.suspend('input disconnected')
    assert d.calls[-1][0] == 'Stop'
    assert all(c[0] == 'Stop' for c in d.calls[before:])
    assert e.trajectory is None and e.target is None and not e.braking
    assert e.session.ready == (release in ('deadman','centered','expired'))
    if release=='expired':
        send(deadman=True,linear_m_s=[-.002,0,0]);step()
        assert d.calls[-1][0]=='Stop' and e.session.active() is None
        send();step()
    step()
    if e.session.ready:
        send(deadman=True,linear_m_s=[-.002,0,0]); step()
        assert d.calls[-1][0] == 'ServoP'
        assert -.0001 < e.target[0][0] < 0  # Fresh actual seed and ramp.
    else:
        count=len(d.calls)
        send(deadman=True,linear_m_s=[-.002,0,0]);step()
        assert len(d.calls)==count


def test_pending_ack_never_advances_smoothing_or_queues_old_input():
    e,d,send,step,_=setup(native_smoothing='ruckig');d.auto_ack=False
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    reference=deepcopy(e.trajectory.state()); target=e.target[0].copy()
    for _ in range(3):send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert e.trajectory.state()==reference and len(d.calls)==1
    np.testing.assert_array_equal(e.target[0],target)
    d.calls[0][3].event.update(accepted=True,acknowledged=True);d.calls[0][3].done.set()
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert len(d.calls)==2 and e.trajectory.input.current_velocity[0]<=0


def test_smoothing_tool_frame_quaternion_boundary_and_shadow_no_writes():
    e,d,send,step,_=setup('shadow',native_smoothing='ruckig')
    e.target=(e.target[0],Rotation.from_euler('z',179.9,degrees=True),e.target[2]);e.base=e.target
    for _ in range(30):
        send(deadman=True,linear_m_s=[.001,0,0],angular_rad_s=[0,0,.01],frame='tool');step()
    assert e.candidate['position_m'][0]<0
    assert np.linalg.norm(e.candidate['quaternion_xyzw'])==pytest.approx(1)
    assert (e.target[1]*e.base[1].inv()).magnitude()<.02
    assert e.candidate['source_input']['intent']['frame']=='tool' and not d.calls


def test_smoothing_mode_transition_stops_and_keeps_session_recoverable():
    e,d,send,step,_=setup(native_smoothing='ruckig')
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    send(deadman=True,joint_rad_s=[0,0,0,0,0,.01]);step()
    assert d.calls[-1][0]=='Stop' and e.session.ready and e.trajectory is None


def test_smoothing_configuration_is_explicit_and_read_back():
    with pytest.raises(ValueError): NativeExecutor('unused','shadow',None,native_smoothing='other')
    with pytest.raises(ValueError): NativeExecutor('unused','shadow',None,native_smoothing='ruckig',release_mode='ruckig')
    with pytest.raises(ValueError): motion_engine_option(dict(rootfs='/ros',native_smoothing='ruckig'))
    for profile in ('commissioning','intermediate','normal'):
        assert motion_engine_option(dict(motion_engine='native_servop',motion_profile=profile,native_smoothing='ruckig'))==[]
        e,d,_,_,_=setup('shadow',native_smoothing='ruckig',motion_profile=profile,servo_duration_s=.1)
        state=e.state()
        config=dict(motion_engine='native_servop',native_smoothing='ruckig')
        assert gateway_ready(state,'shadow',motion_profile=profile,servo_config=config)
        assert not gateway_ready(state,'shadow',servo_config=dict(motion_engine='native_servop'))
