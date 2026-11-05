import pytest

from studio.scene import Rig


def feedback_state(**changes):
    return dict(dict(connected=True, connectionId="connection-a", simulated=False,
                     feedbackValid=True, feedbackPosition=15., feedbackAgeMs=8.), **changes)


def transforms(rig):
    return [(part.position.toTuple(), part.rotation.toVector4D().toTuple()) for part in rig.parts]


def test_obstructed_grip_uses_observation_not_command(qapp):
    rig = Rig()
    rig.updateDisplay(feedback_state(), 35.)
    first = transforms(rig)
    assert rig.opening == 15 and rig.displayMode == "feedback"
    assert rig.displayValue == "15 %"
    rig.updateDisplay(feedback_state(), 0.)
    assert transforms(rig) == first
    assert "0.0 %" in rig.displayDetail
    rig.updateDisplay(feedback_state(feedbackPosition=22.), 0.)
    assert rig.opening == 22 and transforms(rig) != first


@pytest.mark.parametrize("invalid", [
    {"feedbackValid": False, "feedbackPosition": None},
    {"feedbackAgeMs": 301},
    {"feedbackPosition": float("nan")},
    {"feedbackPosition": 500},
])
def test_invalid_feedback_freezes_last_observation_without_target_fallback(qapp, invalid):
    rig = Rig()
    rig.updateDisplay(feedback_state(), 35.)
    first = transforms(rig)
    rig.updateDisplay(feedback_state(**invalid), 100.)
    assert rig.opening == 15 and transforms(rig) == first
    assert rig.displayMode == "stale" and rig.displayValue == "—"
    assert rig.modelVisible and "非实时" in rig.displayDetail


def test_new_connection_never_reuses_old_pose_and_offline_preview_is_explicit(qapp):
    rig = Rig()
    rig.updateDisplay(feedback_state(), 35.)
    rig.updateDisplay(feedback_state(connectionId="connection-b", feedbackValid=False), 100.)
    assert rig.displayMode == "waiting" and not rig.modelVisible
    assert rig.displayValue == "—"
    rig.updateDisplay(feedback_state(connectionId="connection-b", simulated=True, feedbackPosition=28.), 100.)
    assert rig.displayMode == "simulated" and rig.opening == 28
    rig.updateDisplay({"connected": False}, 35.)
    assert rig.displayMode == "preview" and rig.opening == 35
    assert "未连接" in rig.displayTitle
    rig.updateDisplay(feedback_state(feedbackValid=False), 80.)
    assert rig.displayMode == "waiting" and not rig.modelVisible


@pytest.mark.parametrize("position,bounded", [(-1., 0.), (101., 100.)])
def test_geometry_bounds_do_not_change_reported_feedback(qapp, position, bounded):
    rig = Rig()
    rig.updateDisplay(feedback_state(feedbackPosition=position), 35.)
    assert rig.opening == bounded and rig.displayValue == f"{position:.0f} %"
