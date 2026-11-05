"""Regression for v100: braking left J4 10.29 degrees from the test origin."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_gateway import setup
from test_moveit_bridge import Bridge, row
from e6_phosphobot.workspace import TestWorkspace


def outside_case(kind, direction):
    e, d, send, step, now = setup()
    e.moveit = Bridge()
    candidate = row(now[0])
    if kind == 'joint':
        d.data['status']['joint_state']['q_actual'][3] = 10.29
        d.data['status']['joint_state']['q_target'][3] = 10.29
        candidate['joints_rad'][3] = np.deg2rad(10.29 + direction * .01)
    elif kind == 'position':
        d.data['status']['tool_vector_actual']['x'] = 51
        candidate['position_m'][0] = (51 + direction * .1) / 1000
    else:
        d.data['status']['tool_vector_actual']['rx'] = 10.29
        candidate['quaternion_xyzw'] = Rotation.from_euler('x', 10.29 + direction * .05,
                                                          degrees=True).as_quat().tolist()
    send(deadman=True, linear_m_s=[-.002, 0, 0])
    candidate['input_sequence'] = e.session.sequence
    e.moveit.row = candidate
    step()
    return e, d, send, step


@pytest.mark.parametrize('kind', ['joint', 'position', 'rotation'])
def test_inward_candidate_after_braking_overshoot_keeps_original_origin(kind):
    e, d, send, step = outside_case(kind, -1)
    assert [c[0] for c in d.calls] == ['ServoJ']
    assert e.reason is None and e.session.ready
    np.testing.assert_allclose(e.base[0], [0, 0, .4])
    np.testing.assert_allclose(e.base[2], [0] * 6)
    send(); step()
    assert d.calls[-1][0] == 'Stop' and e.session.ready


@pytest.mark.parametrize('kind', ['joint', 'position', 'rotation'])
@pytest.mark.parametrize('direction', [0, 1])
def test_outside_hold_or_outward_candidate_stays_blocked(kind, direction):
    e, d, _, _ = outside_case(kind, direction)
    assert not d.calls and e.session.ready and '边界' in e.reason


def test_joint_retreat_cannot_add_a_cartesian_violation():
    e, d, send, step, now = setup()
    e.moveit = Bridge()
    d.data['status']['joint_state']['q_actual'][3] = 10.29
    candidate = row(now[0])
    candidate['joints_rad'][3] = np.deg2rad(10.24)
    candidate['position_m'][0] = .0501
    send(deadman=True, linear_m_s=[-.002, 0, 0])
    e.moveit.row = candidate; step()
    assert not d.calls and '边界' in e.reason


@pytest.mark.parametrize('value', [True, 0, -1, float('nan'), float('inf'), '20'])
@pytest.mark.parametrize('field', ['position_m', 'rotation_deg', 'joint_deg'])
def test_invalid_workspace_is_rejected_before_any_device_connection(field, value):
    from e6_phosphobot.gateway import NativeExecutor
    with pytest.raises(ValueError, match='workspace'):
        NativeExecutor('unused', 'shadow', None, workspace_config={field:value})


def test_unknown_workspace_fields_do_not_silently_change_units():
    with pytest.raises(ValueError, match='workspace'):
        TestWorkspace.from_config({'position_mm':50})
    with pytest.raises(ValueError, match='workspace'):
        TestWorkspace.from_config([.05, 10, 20])


@pytest.mark.parametrize('stage', ['shadow', 'live'])
def test_explicit_joint_range_does_not_expand_cartesian_or_rotation_bounds(stage):
    e, d, send, step, now = setup(stage, workspace_config={'joint_deg':20})
    e.moveit = Bridge()
    send(deadman=True, linear_m_s=[.002, 0, 0])
    candidate = row(now[0])
    candidate['joints_rad'][2] = np.deg2rad(10.64)
    d.data['status']['joint_state']['q_actual'][2]=10.63
    d.data['status']['joint_state']['q_target'][2]=10.63
    candidate['position_m'][2] += .02325
    e.moveit.row = candidate; step()
    assert e.candidate and e.reason is None
    assert len(d.calls) == (1 if stage=='live' else 0)
    assert e.state()['capabilities']['test_workspace'] == dict(position_m=.05,rotation_deg=10,joint_deg=20)
    send(); step()
    # The new physical segment starts only after the Stop and idle window.
    for _ in range(7):send();step()
    for pos, angle in ((.0501,0), (0,10.01)):
        send(deadman=True, linear_m_s=[.002,0,0])
        candidate = row(now[0]); candidate['position_m'][0] = pos
        candidate['quaternion_xyzw'] = Rotation.from_euler('x', angle, degrees=True).as_quat().tolist()
        count = len(d.calls)
        e.moveit.row = candidate; step()
        assert len(d.calls)==count and '边界' in e.reason
    np.testing.assert_allclose(e.base[0], [0,0,.4])


def test_workspace_roundtrip_and_launcher_readback():
    import json
    from e6_phosphobot.session_runner import workspace_option
    from e6_phosphobot.launcher import gateway_ready
    config={'test_workspace':TestWorkspace(joint_deg=20).description()}
    options=workspace_option(config)
    assert options[0]=='--test-workspace'
    assert json.loads(options[1])==config['test_workspace']
    state=dict(stage='shadow',feedback={'connected':True,'age_ms':8},camera={'valid':True},model={'ready':True},
        capabilities={'motion_engine':'moveit_servo','servo_gain':500,'servo_aheadtime':50,'servo_duration_s':.1,
                      'test_workspace':config['test_workspace']})
    assert gateway_ready(state,'shadow',servo_config=config)
    state['capabilities']['test_workspace']=TestWorkspace().description()
    assert not gateway_ready(state,'shadow',servo_config=config)
    del state['capabilities']['test_workspace']
    assert not gateway_ready(state,'shadow',servo_config=config)


@pytest.mark.parametrize('stage',['shadow','live'])
@pytest.mark.parametrize('recovery',['stop','reconnect','feedback_generation'])
def test_prepare_keeps_site_origin_but_reseeds_from_new_actual(stage,recovery):
    e,d,send,step,now=setup(stage)
    origin=e.workspace_origin()
    if recovery=='reconnect':
        e.release('page')
        assert e.claim('replacement')
    else:e.suspend('test recovery')
    if recovery=='feedback_generation':d.data['generation']='b'
    # The real robot may settle away from the original prepare point. Use a
    # point near the old boundary to reveal a mistaken moving origin.
    d.data['status']['tool_vector_actual']['x']=49
    d.data['status']['joint_state']['q_actual'][5]=9
    d.data['status']['joint_state']['q_target'][5]=9
    step(110)
    send();send('prepare')
    assert e.session.ready
    assert e.workspace_origin()==origin
    assert e.state()['workspace_origin']==origin
    np.testing.assert_allclose(e.target[0],[.049,0,.4])
    np.testing.assert_allclose(e.target[2],np.deg2rad([0,0,0,0,0,9]))
    prepared=[ev for ev in e.events if ev['event']=='prepared'][-1]
    assert prepared['base']==origin
    assert prepared['start']['position_m']==[.049,0,.4]
    assert e.generation==d.data['generation']
    # A target 2 mm beyond the fresh start is already outside the old envelope.
    e.moveit=Bridge();candidate=row(now[0]);candidate['position_m'][0]=.051
    e.moveit.row=candidate
    send(deadman=True,linear_m_s=[.002,0,0]);count=len(d.calls);step()
    assert len(d.calls)==count and '边界' in e.reason
    # Releasing and trying the inward direction remains normal recovery.
    send();step();candidate=row(now[0]);candidate['position_m'][0]=.048
    candidate['joints_rad'][5]=np.deg2rad(8.99);e.moveit.row=candidate
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert e.reason is None and e.session.ready
    assert e.workspace_origin()==origin
