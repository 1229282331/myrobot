"""Observation rendering must not own the actuator's input/Stop mutex."""
from copy import deepcopy
import threading
from unittest.mock import patch

import pytest

from e6_phosphobot.state_observation import observe_executor, render_observation
from test_gateway import setup


def test_slow_render_does_not_block_explicit_stop_or_rewrite_captured_readiness():
    executor, device, send, _, _ = setup()
    mutex = threading.RLock()
    entered, resume = threading.Event(), threading.Event()
    result, failures = [], []
    captured_source = executor.last_snapshot["received_ns"]

    def slow(observation):
        entered.set()
        assert resume.wait(3)
        return render_observation(observation)

    def publish():
        try:
            result.append(observe_executor(executor, mutex, True))
        except Exception as exc:
            failures.append(exc)

    with patch("e6_phosphobot.state_observation.render_observation", slow):
        worker = threading.Thread(target=publish)
        worker.start()
        try:
            assert entered.wait(2)
            assert mutex.acquire(timeout=.2), "Renderer retained the control lock"
            try:
                send("stop")
                assert not executor.session.ready
            finally:
                mutex.release()
        finally:
            resume.set()
            worker.join(3)
    assert not worker.is_alive() and not failures
    # Publication remains an honest old observation. It cannot restore ready.
    assert result[0]["ready"] is True
    assert result[0]["feedback"]["received_ns"] == captured_source
    assert executor.session.ready is False
    assert not any(call[0] in ("ServoJ", "ServoP") for call in device.calls)


def test_capture_and_render_keep_feedback_pose_gripper_and_session_from_one_capture():
    executor, _, send, _, _ = setup()
    with threading.RLock():
        captured = executor.capture_state(True)
    original = deepcopy(captured.data)
    send("stop")
    executor.last_snapshot = dict(executor.last_snapshot, status=None, received_ns=999999)
    first = render_observation(captured)
    second = render_observation(captured)
    assert first == second
    assert captured.data == original
    assert first["ready"] and first["actual"]["position_m"] == [0., 0., .4]
    assert first["feedback"]["received_ns"] == original["feedback"]["received_ns"]
    assert first["echo_ns"] == original["echo_ns"]
    assert first["capabilities"]["test_workspace"] == executor.workspace.description()
    assert first["capabilities"]["tcp"] == executor.tool.description()
    assert not executor.session.ready


@pytest.mark.parametrize("error", [ValueError("geometry"), OSError("asset missing"), KeyError("table")])
def test_display_geometry_failure_remains_local_to_observation(error):
    executor, device, _, _, _ = setup()
    with patch("e6_phosphobot.gripper_geometry.pose", side_effect=error):
        state = observe_executor(executor, threading.RLock())
    assert state["gripper"]["geometry"] is None
    assert state["feedback"]["status"]["robot_mode"] == 5
    assert state["ready"] and executor.session.ready
    assert not device.calls
