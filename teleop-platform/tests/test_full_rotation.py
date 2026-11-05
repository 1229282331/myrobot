"""Full orientation range uses source-backed absolute joints, never joint clipping."""
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from e6_phosphobot.gateway import NativeExecutor, evaluate_workspace
from e6_phosphobot.joint_range import JOINT_MARGIN_DEG, JointRange
from e6_phosphobot.moveit_config import load_jazzy_bundle
from e6_phosphobot.workspace import TestWorkspace
from test_gateway import setup
from test_moveit_bridge import Bridge, row

ROOT = Path(__file__).resolve().parents[1]
FULL = dict(position_m=.05, rotation_deg=None, joint_deg=None)


@pytest.fixture
def bundle(tmp_path):
    shutil.copytree(ROOT/'config/e6-controller-model', tmp_path/'model')
    return tmp_path


def test_bounds_are_exactly_pinned_official_joint_ranges_minus_five_degrees(bundle):
    limits = JointRange(bundle)
    official = ET.parse(ROOT/'upstream/dobot-ros2/dobot_rviz/urdf/me6_robot.urdf')
    for i in range(6):
        source = official.find(f"joint[@name='joint{i+1}']/limit")
        assert limits.bounds[i].tolist() == [float(source.get('lower')), float(source.get('upper'))]
    np.testing.assert_allclose(limits.bounds[:, 1]-limits.upper, math.radians(5))
    np.testing.assert_allclose(limits.lower-limits.bounds[:, 0], math.radians(5))
    assert limits.description()['margin_deg'] == 5


@pytest.mark.parametrize('joint', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_margin_blocks_outward_and_allows_retreat_without_clipping(bundle, joint, sign):
    limits = JointRange(bundle)
    bound = limits.lower[joint] if sign < 0 else limits.upper[joint]
    actual = np.zeros(6); actual[joint] = bound-sign*.001
    target = actual.copy(); target[joint] = bound+sign*.001
    saved = target.copy()
    assert not limits.evaluate(actual, target)['allowed']
    np.testing.assert_array_equal(target, saved)
    assert limits.evaluate(target, actual)['allowed']
    assert limits.evaluate(target, actual)['retreat']  # Crossing back inside is also retreat.
    nearer = target.copy(); nearer[joint] -= sign*.0001
    assert limits.evaluate(target, nearer)['retreat']
    assert not limits.evaluate(target, target)['allowed']
    outward = target.copy(); outward[joint] += sign*.0001
    assert not limits.evaluate(target, outward)['allowed']


def test_absolute_retreat_cannot_add_another_joint_violation(bundle):
    limits = JointRange(bundle)
    actual = np.zeros(6); actual[0] = limits.upper[0]+.01
    target = actual.copy(); target[0] -= .001; target[1] = limits.upper[1]+.001
    assert not limits.evaluate(actual, target)['allowed']


@pytest.mark.parametrize('bad', [None, np.ones(5), [0, 0, 0, 0, 0, float('nan')]])
def test_invalid_absolute_input_fails_closed(bundle, bad):
    with pytest.raises(ValueError):JointRange(bundle).evaluate(np.zeros(6), bad)


def test_explicit_full_range_requires_verified_model_and_legacy_defaults_remain():
    assert TestWorkspace().description() == dict(position_m=.05, rotation_deg=10., joint_deg=10.)
    assert TestWorkspace.from_config(FULL).description() == FULL
    with pytest.raises(ValueError, match='requires MoveIt'):
        NativeExecutor('unused', 'live', None, workspace_config=FULL)
    with pytest.raises(ValueError):TestWorkspace(position_m=None)


def test_modified_model_rejected_before_connection(bundle):
    with (bundle/'model/e6.urdf').open('a') as f:f.write(' ')
    with pytest.raises(ValueError, match='reviewed profile'):
        NativeExecutor('unused', 'live', None, workspace_config=FULL, moveit_bundle=bundle)


def test_solver_and_gateway_share_five_degree_margin(monkeypatch, bundle):
    (bundle/'servo_jazzy.yaml').write_text('{}')
    monkeypatch.setattr('e6_phosphobot.moveit_config.load_model', lambda _: ({}, {}))
    config, _ = load_jazzy_bundle(bundle)
    assert config['moveit_servo']['joint_limit_margins'] == [math.radians(JOINT_MARGIN_DEG)]
    assert config['moveit_servo']['check_collisions'] is True


@pytest.mark.parametrize('angle', [45, 120, 179.9, -179.9])
@pytest.mark.parametrize('stage', ['shadow', 'live'])
def test_large_tcp_rotation_accepted_without_flange_sphere_or_relative_joint_cap(bundle, angle, stage):
    e, d, send, step, now = setup(stage, moveit_bundle=bundle,
        tcp_profile='omnipicker-r2', workspace_config=FULL)
    bridge=Bridge(); bridge.info['tcp']=e.tool.description(); e.moveit=bridge
    center=e.base[0].copy()
    rotation=Rotation.from_euler('y', angle, degrees=True)
    flange=center-rotation.apply([0, 0, .1504])
    send(deadman=True, angular_rad_s=[0, .01, 0])
    target=row(now[0], e.session.sequence)
    target.update(position_m=flange.tolist(), quaternion_xyzw=rotation.as_quat().tolist(),
                  joints_rad=[0, -.5, .7, -.5, 1., .6])
    # This is the last small step of a large rotation, not a one-frame jump
    # from zero. The original workspace center deliberately stays unchanged.
    observed=np.rad2deg(target['joints_rad']);observed[5]-=.005
    d.data['status']['joint_state'].update(q_actual=observed.tolist(),q_target=observed.tolist())
    bridge.row=target; step()
    assert e.candidate is not None and e.reason is None
    np.testing.assert_allclose(e.candidate['position_m'], center)
    assert e.candidate['workspace']['flange']['position_delta_m'] > .05
    assert e.state()['capabilities']['joint_range'] == JointRange(bundle).description()
    if stage == 'live':
        assert len(d.calls) == 1 and d.calls[0][0] == 'ServoJ'
        np.testing.assert_allclose(d.calls[0][1][0], np.rad2deg(target['joints_rad']))
        send(); step()
        assert d.calls[-1][0] == 'Stop' and e.session.ready
    else:assert not d.calls


def test_gateway_rejects_absolute_violation_despite_solver_success(bundle):
    e,d,send,step,now=setup(moveit_bundle=bundle,workspace_config=FULL)
    e.moveit=Bridge()
    send(deadman=True,angular_rad_s=[0,.01,0])
    target=row(now[0],e.session.sequence)
    target['joints_rad'][3]=e.joint_range.upper[3]+.001
    e.moveit.row=target;step()
    assert not d.calls and e.session.ready and '限位' in e.reason
    assert [v for v in e.events if v['event']=='workspace_blocked'][-1]['workspace']['absolute_joints']['exceeded']==['J4']


def test_full_rotation_does_not_remove_tcp_translation_bound():
    base=(np.zeros(3),Rotation.identity(),np.zeros(6))
    target=(np.array([.051,0,0]),Rotation.from_euler('z',170,degrees=True),np.ones(6))
    report=evaluate_workspace(base,base,target,TestWorkspace.from_config(FULL))
    assert not report['allowed'] and report['exceeded']==['position']


def test_launcher_full_range_roundtrip():
    from e6_phosphobot.session_runner import workspace_option
    config=json.loads((ROOT/'config/workstation.json').read_text())
    assert json.loads(workspace_option(config)[1])==dict(FULL,mode='model')
    # Legacy local configurations retain the fixed translation sphere and
    # unrestricted relative rotation; publishing model mode must not erase it.
    config['test_workspace']=FULL
    assert json.loads(workspace_option(config)[1])==FULL
