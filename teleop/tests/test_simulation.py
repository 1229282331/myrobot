import base64
from io import BytesIO
import json
from pathlib import Path
import numpy as np
import pytest
from PIL import Image
from e6_teleop.core import GatewayCore, InputError
from e6_teleop.model import E6Model
from e6_teleop.math3d import integrate, from_rotvec, multiply, inverse, matrix, to_rotvec
from e6_teleop.recording import EpisodeRecorder, read_raw_episode
from e6_teleop.exporter import load_episode


class Clock:
    now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, seconds=.05): self.now += int(seconds*1e9)


@pytest.fixture
def rig():
    model, clock = E6Model(), Clock()
    core = GatewayCore(model, clock)
    core.connect()
    return core, clock, model


def send(core, **updates):
    data = dict(operation="input", session=core.owner, lease=core.lease, seq=core.seq+1,
                source="keyboard", identity="test", connected=True, focused=True, deadman=False,
                neutral=True, fine=False, stop=False, frame="base", mode="translation",
                twist=[0]*6, gripper_rate=0, raw={})
    data.update(updates)
    core.command(data)
    return data


def arm(core):
    send(core)
    core.command(dict(operation="arm", session=core.owner))


def test_fk_independently_matches_official_urdf():
    m = E6Model()
    rng = np.random.default_rng(42)
    for _ in range(12):
        q = m.q + rng.uniform(-.08, .08, 6)
        t = np.eye(4)
        for theta, joint in zip(q, [j for j in m.source.findall("joint") if j.get("type") == "revolute"]):
            origin = joint.find("origin")
            r, p, y = map(float, origin.get("rpy").split())
            local = np.eye(4)
            local[:3, :3] = matrix(from_rotvec([0,0,y])) @ matrix(from_rotvec([0,p,0])) @ matrix(from_rotvec([r,0,0]))
            local[:3, 3] = list(map(float, origin.get("xyz").split()))
            rotation = np.eye(4)
            rotation[:3, :3] = matrix(from_rotvec(np.asarray(list(map(float, joint.find("axis").get("xyz").split()))) * theta))
            t = t @ local @ rotation
        position, quaternion = m.pose(q)
        np.testing.assert_allclose(position, t[:3,3], atol=1e-9)
        np.testing.assert_allclose(matrix(quaternion), t[:3,:3], atol=1e-9)


def test_fk_ik_closed_loop_seeded_and_limit_checked():
    m = E6Model()
    for axis in range(6):
        q = m.q.copy(); q[axis] += .025
        position, rotation = m.pose(q)
        solution, residual = m.solve(position, rotation)
        np.testing.assert_allclose(m.pose(solution)[0], position, atol=1e-5)
        assert residual["rotation_rad"] < 1e-4
        assert np.all(solution >= m.limits[:,0]) and np.all(solution <= m.limits[:,1])
    with pytest.raises(ValueError, match="limits"):
        m.set_state(m.limits[:,1]+.01, .5)
    old = m.q.copy()
    with pytest.raises(ValueError, match="IK rejected"):
        m.solve([2, 2, 2], m.pose()[1])
    np.testing.assert_array_equal(m.q, old)


def test_quaternion_wrap_and_coordinate_frames():
    q = from_rotvec([0,0,np.radians(179)])
    position, after = integrate([0,0,0], q, [0,0,0], [0,0,np.radians(4)], 1, "base")
    np.testing.assert_allclose(to_rotvec(multiply(after, inverse(q))), [0,0,np.radians(4)], atol=1e-10)
    q = from_rotvec([0,0,np.pi/2])
    np.testing.assert_allclose(integrate([0,0,0], q, [1,0,0], [0,0,0], 1, "tool")[0], [0,1,0], atol=1e-10)
    np.testing.assert_allclose(integrate([0,0,0], q, [1,0,0], [0,0,0], 1, "base")[0], [1,0,0], atol=1e-10)
    base = integrate([0,0,0], q, [0,0,0], [1,0,0], .1, "base")[1]
    tool = integrate([0,0,0], q, [0,0,0], [1,0,0], .1, "tool")[1]
    assert np.linalg.norm(to_rotvec(multiply(base, inverse(tool)))) > .1


def test_observation_before_effective_action_and_release_hold(rig):
    core, clock, model = rig
    arm(core)
    before = model.q.copy()
    send(core, twist=[.005,0,0,0,0,0], gripper_rate=.1, deadman=True, neutral=False)
    clock.advance()
    row, _ = core.tick()
    np.testing.assert_array_equal(row["observation"]["joints_rad"], before)
    np.testing.assert_array_equal(row["effective_action"]["joints_rad"], model.q)
    assert np.max(np.abs(model.q-before)) > 1e-5
    assert model.gripper == pytest.approx(.605)
    send(core, deadman=False, neutral=False, twist=[.005,0,0,0,0,0], gripper_rate=.1)
    held = model.q.copy()
    clock.advance(); core.tick()
    np.testing.assert_array_equal(model.q, held)
    assert model.gripper == pytest.approx(.605)


def test_fine_speed_scaling(rig):
    core, clock, model = rig
    arm(core)
    position = model.pose()[0]
    send(core, twist=[.01,0,0,0,0,0], gripper_rate=.1, deadman=True, neutral=False, fine=True)
    clock.advance(); row, _ = core.tick()
    assert row["effective_action"]["requested_tcp"]["position_m"][0] - position[0] == pytest.approx(.000125)
    assert model.gripper == pytest.approx(.60125)


@pytest.mark.parametrize("fault", ["duplicate", "out_of_order", "old_lease", "expired_lease", "overspeed", "nan", "wrong_mode", "held_switch"])
def test_input_fault_requires_rearm_and_holds(rig, fault):
    core, clock, model = rig
    arm(core)
    data = send(core)
    bad = dict(data, seq=core.seq+1, lease=core.lease)
    if fault == "duplicate": bad["seq"] = core.seq
    if fault == "out_of_order": bad["seq"] = core.seq-1
    if fault == "old_lease": bad["lease"] = data["lease"]
    if fault == "expired_lease": clock.advance(.251)
    if fault == "overspeed": bad["twist"] = [.02,0,0,0,0,0]
    if fault == "nan": bad["twist"] = [float("nan"),0,0,0,0,0]
    if fault == "wrong_mode": bad["twist"] = [0,0,0,.01,0,0]
    if fault == "held_switch": bad.update(frame="tool", deadman=True, neutral=False)
    held = model.q.copy()
    with pytest.raises(InputError): core.command(bad)
    assert not core.armed
    clock.advance(); core.tick()
    np.testing.assert_array_equal(model.q, held)
    arm(core)
    assert core.armed


@pytest.mark.parametrize("updates", [dict(focused=False), dict(connected=False), dict(stop=True)])
def test_stop_focus_and_device_loss(rig, updates):
    core, clock, model = rig
    arm(core)
    send(core, deadman=True, neutral=False, twist=[.01,0,0,0,0,0], **updates)
    held = model.q.copy()
    clock.advance(); core.tick()
    assert not core.armed
    np.testing.assert_array_equal(model.q, held)


def test_watchdog_disconnect_second_owner_and_reconnect(rig):
    core, clock, model = rig
    arm(core)
    with pytest.raises(InputError, match="控制权"): core.connect()
    assert core.armed  # rejected second controller does not seize the first one
    clock.advance(.26); core.tick()
    assert not core.armed
    first = core.owner
    core.disconnect(first)
    core.connect()
    assert not core.armed and core.input is None and core.owner != first
    with pytest.raises(InputError): core.command(dict(operation="arm", session=core.owner))
    with pytest.raises(InputError): core.command(dict(operation="stop", session=first))
    send(core, deadman=True)
    with pytest.raises(InputError): core.command(dict(operation="arm", session=core.owner))
    send(core, neutral=False, gripper_rate=0)  # both-held inputs may sum to zero, still not neutral
    with pytest.raises(InputError): core.command(dict(operation="arm", session=core.owner))
    arm(core)
    assert core.armed


def image_frame(core, clock):
    clock.advance(); row, _ = core.tick()
    stream = BytesIO(); Image.new("RGB", (640,360)).save(stream, "PNG")
    row["images"] = dict(wrist_png=base64.b64encode(stream.getvalue()).decode())
    return row


def test_recording_outcomes_images_and_abort_export(rig, tmp_path):
    core, clock, model = rig
    recorder = EpisodeRecorder(tmp_path)
    for result in ("success", "failure", "aborted"):
        name = recorder.start("模拟任务", {})
        recorder.sample(image_frame(core, clock)); recorder.sample(image_frame(core, clock))
        recorder.finish(result)
        if result == "aborted":
            with pytest.raises(ValueError, match="complete"): load_episode(tmp_path / name)
        else:
            path, meta, summary, rows = load_episode(tmp_path / name)
            assert len(rows) == 2 and summary["complete"] and meta["dataset_kind"] == "simulation"
            (path / rows[0]["image"]).write_bytes(b"corrupt")
            with pytest.raises(ValueError, match="hash"): load_episode(path)


def test_missing_frame_marks_incomplete(rig, tmp_path):
    core, clock, model = rig
    recorder = EpisodeRecorder(tmp_path)
    name = recorder.start("test", {})
    recorder.sample(image_frame(core, clock))
    core.tick()
    with pytest.raises(ValueError, match="缺帧"): recorder.sample(image_frame(core, clock))
    recorder.finish("aborted", "missing frame")
    summary = json.loads((tmp_path / name / "summary.json").read_text())
    assert summary["complete"] is False and summary["samples"] == 1


def test_disk_error_does_not_create_fake_sample(rig, tmp_path, monkeypatch):
    core, clock, model = rig
    recorder = EpisodeRecorder(tmp_path)
    name = recorder.start("test", {})
    def fail(*args, **kwargs): raise OSError("injected disk failure")
    monkeypatch.setattr(Path, "write_bytes", fail)
    with pytest.raises(OSError): recorder.sample(image_frame(core, clock))
    assert recorder.samples == 0
    recorder.incomplete = True
    recorder.finish("aborted", "injected disk failure")
    assert not json.loads((tmp_path / name / "summary.json").read_text())["complete"]


def test_process_interrupted_raw_still_inspectable(rig, tmp_path):
    core, clock, model = rig
    recorder = EpisodeRecorder(tmp_path)
    name = recorder.start("test", {})
    recorder.sample(image_frame(core, clock))
    recorder.stream.write('{"truncated":')
    recorder.stream.close()
    summary, rows = read_raw_episode(tmp_path/name)
    assert len(rows)==1 and summary["result"]=="aborted" and summary["complete"] is False


def test_ik_rejection_keeps_joint_and_gripper_target(rig, monkeypatch):
    core, clock, model = rig
    arm(core)
    before=model.q.copy(); grip=model.gripper
    def reject(*args): raise ValueError("IK joint limit rejected")
    monkeypatch.setattr(model,"solve",reject)
    send(core,deadman=True,neutral=False,twist=[.01,0,0,0,0,0],gripper_rate=.1)
    clock.advance(); row,_=core.tick()
    np.testing.assert_array_equal(model.q,before)
    assert model.gripper==grip and not core.armed
    assert row["effective_action"]["ik_result"]["reason"]=="IK joint limit rejected"
