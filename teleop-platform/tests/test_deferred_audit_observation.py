"""Journal rendering cannot occupy the actuator thread or alter old samples."""
from copy import deepcopy
import json
import logging
import threading
from types import SimpleNamespace
from unittest.mock import patch

from e6_phosphobot.audit import Audit
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.operator_logs import AsyncLog, audit_message
from e6_phosphobot.state_observation import (
    capture_audit_observation, observation_fields, render_audit_event,
    render_observation,
)
from test_gateway import setup


def test_capture_never_renders_and_freezes_in_place_feedback_changes():
    executor, device, send, _, _ = setup()
    with patch("e6_phosphobot.state_observation.render_observation",
               side_effect=AssertionError("Rendered on control thread")):
        captured = capture_audit_observation(executor, threading.RLock())
    expected = render_observation(captured)
    source = expected["feedback"]["received_ns"]
    executor.last_snapshot["status"]["joint_state"]["q_actual"][0] = 42.
    executor.last_snapshot["received_ns"] = source + 200_000_000
    send("stop")
    assert render_observation(captured) == expected
    assert expected["ready"] and not executor.session.ready
    assert expected["feedback"]["received_ns"] == source
    assert expected["actual"]["joints_rad"][0] == 0.
    assert not any(call[0] in ("ServoJ", "ServoP") for call in device.calls)


def test_two_independent_journals_render_the_same_frozen_state_and_order(tmp_path):
    executor, _, _, _, _ = setup()
    captured = capture_audit_observation(executor, threading.RLock())
    before = deepcopy(captured)
    rows = [
        dict(event="input", at_ns=1, data={"text": "原始输入\n原行"}),
        dict(event="cycle", at_ns=2, state=captured, gateway_timing={"sample": 2}),
        dict(event="motion_confirmed", at_ns=3, receipt={"sent_ns": 1, "ack_ns": 3}),
        dict(event="episode_finished", at_ns=4, final_state=captured, result="success"),
    ]
    expected = [render_audit_event(row) for row in rows]
    reader = FeedbackReader("unused")
    writers = [Audit(tmp_path/name, reader) for name in ("continuous", "episode")]
    for sequence, row in enumerate(rows):
        reader._publish(SimpleNamespace(raw=b"unaltered", prefix=b"", received_ns=sequence))
        for writer in writers:
            writer.put(row)
    # Deferred work is bound to the capture clock, even if live state changes.
    executor.last_snapshot["status"]["tool_vector_actual"]["z"] = 999.
    for writer in writers:
        writer.close()
        assert writer.error is None
        assert writer.counts == dict(raw=4, events=4)
        assert writer.process.poll() == 0 and not writer.thread.is_alive()
        actual = [json.loads(line) for line in (writer.path/"events.jsonl").read_text(encoding="utf8").splitlines()]
        assert actual == expected
    assert captured == before
    assert not reader._subscriptions


def test_human_logs_read_lifecycle_without_display_geometry(tmp_path):
    executor, _, _, _, _ = setup()
    captured = capture_audit_observation(executor, threading.RLock())
    logger = logging.getLogger("deferred-observation-test")
    logger.setLevel(logging.DEBUG)
    sink = AsyncLog(tmp_path/"human.log")
    logger.addHandler(sink)
    try:
        with patch("e6_phosphobot.state_observation.render_observation",
                   side_effect=AssertionError("Human log rendered geometry")):
            audit_message(logger, dict(event="cycle", at_ns=1, state=captured))
            sink.close()
        assert sink.write_errors == 0 and sink.dropped == 0
        assert "Control status" in (tmp_path/"human.log").read_text(encoding="utf8")
        fields = observation_fields(captured)
        assert fields["stationary"] == captured.fields["stationary"]
        assert fields["stopping"] == captured.fields["stopping"]
    finally:
        logger.removeHandler(sink)
