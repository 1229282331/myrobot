"""Read-only model experiments cannot be mistaken for hardware acceptance."""
import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'experiments'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'device_gateway'))
from local_kinematics import LocalKinematics, logarithm, inverse
from e6_observer.control_math import angle, controller_pose, multiply, pose, rotvec


def reference(q):
    p = np.array([.1, -.2, .4])+.3*q[:3]+np.array([.02*q[1]*q[2], 0, 0])
    r = multiply(rotvec(q[3:]), rotvec([0, 0, 3.1]))
    return controller_pose(p, r, [0]*6)


@pytest.fixture(scope='module')
def model():
    q = np.random.default_rng(71).uniform(-math.radians(10), math.radians(10), (400, 6))
    return LocalKinematics.fit(q, [reference(v) for v in q], np.zeros(6), reference(np.zeros(6)), math.radians(10))


def test_independent_holdout_and_exact_anchor(model):
    for q in np.random.default_rng(19).uniform(-.17, .17, (30, 6)):
        p, r = model.fk(q); pp, rr = pose(reference(q))
        assert np.linalg.norm(p-pp) < 1e-9 and angle(r, rr) < 1e-7
    p, r = model.fk(np.zeros(6)); pp, rr = pose(reference(np.zeros(6)))
    np.testing.assert_array_equal(p, pp)
    assert angle(r, rr) < 1e-7
    exported = model.serializable()
    assert exported['experiment_only'] and not exported['eligible_for_hardware_acceptance']
    assert not exported['physical_accuracy_verified']


def test_quaternion_solve_crosses_euler_wrap_and_matches_independent_reference(model):
    target = np.array([.002, -.003, .001, .001, -.002, .08])
    p, r = pose(reference(target))
    q, residual = model.solve(p, r, np.r_[target[:3], target[3:]-[0, 0, .02]])
    pp, rr = pose(reference(q))
    assert np.linalg.norm(p-pp) < 1e-5 and angle(r, rr) < 1e-4
    assert residual['rotation_rad'] < 1e-4
    assert reference(target)[5] < 0  # Crossed +180 to -180 degrees.


def test_refuses_extrapolation_nonfinite_and_unreachable_targets(model):
    with pytest.raises(ValueError, match='extrapolation'):
        model.fk([math.radians(10.1), 0, 0, 0, 0, 0])
    with pytest.raises(ValueError): model.fk([float('nan')]*6)
    with pytest.raises(ValueError): model.solve([10, 10, 10], [0, 0, 0, 1], np.zeros(6))
    with pytest.raises(ValueError): model.solve([0, 0, 0], [0, 0, 0, 0], np.zeros(6))


def test_singular_and_rank_deficient_fit_fail_closed():
    with pytest.raises(ValueError, match='identify'):
        LocalKinematics.fit(np.zeros((400, 6)), [reference(np.zeros(6))]*400, np.zeros(6), reference(np.zeros(6)), .1)
    rng = np.random.default_rng(91); q = rng.uniform(-.1, .1, (400, 6))
    singular = LocalKinematics.fit(q, [reference(np.zeros(6))]*400, np.zeros(6), reference(np.zeros(6)), .1)
    p, r = singular.fk(np.zeros(6))
    with pytest.raises(ValueError, match='singular'): singular.solve(p+[.001, 0, 0], r, np.zeros(6))


def test_jacobian_agrees_with_reference_and_has_base_frame_rotation(model):
    q = np.array([.03, -.02, .01, .04, -.01, .02]); jac = model.jacobian(q)
    for axis in range(6):
        dq = np.zeros(6); dq[axis] = 1e-5
        pp, rp = pose(reference(q+dq)); pm, rm = pose(reference(q-dq))
        expected = np.r_[(pp-pm)/2e-5, logarithm(multiply(rp, inverse(rm)))/2e-5]
        np.testing.assert_allclose(jac[:, axis], expected, atol=1e-7)


@pytest.mark.parametrize('command', ['EnableRobot()', 'Stop()', 'DisableRobot()', 'ClearError()',
                                   'Continue()', 'ServoJ(0,0,0,0,0,0)', 'Load(0.2)', 'Tool(1)', 'DO(1,1)'])
def test_native_experiment_rejects_every_write_before_network_access(command):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
    from sample_controller_kinematics import ReadOnlyLink
    link = ReadOnlyLink.__new__(ReadOnlyLink)  # No socket, constructor, or CAN.
    with pytest.raises(ValueError, match='refuses all device writes'):
        link._query(command)


@pytest.mark.parametrize('method', ['servo','stop','grip'])
def test_local_benchmark_device_has_no_write_path(method, model):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from benchmark_local_kinematics import ModelDevice
    with pytest.raises(AssertionError,match='refuses'):
        getattr(ModelDevice(model),method)([0]*6)


@pytest.mark.parametrize('marker', [{'experiment_only':True},{'eligible_for_hardware_acceptance':False}])
def test_experiment_markers_cannot_be_relabelled_as_passing_hardware_acceptance(tmp_path, marker):
    import json
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from control_acceptance import require_shadow_acceptance,source_hashes
    from e6_teleop.acceptance_profile import verify_profile
    # Otherwise valid empty coverage isolates the new explicit marker guard.
    result=dict(passed=True,stage='shadow',suite='matrix',duration_s=601,invalid=0,actions=0,
        production_input_sha256='fixture',source_sha256=source_hashes(),source_changed_during_run=False,
        cases=[],stats=dict(motion_tx=0,can_tx=0,stop_tx=0),test_fixture=False,
        hardware_sources_verified=True,**marker)
    path=tmp_path/'experiment.json';path.write_text(json.dumps(result))
    with pytest.raises(ValueError,match='failed'):require_shadow_acceptance(path,{'production_input_sha256':'fixture','rows':[]})
    path.write_text(json.dumps(dict(schema_version=1,complete=True,**marker)))
    with pytest.raises(ValueError,match='experiments'):verify_profile(path)
