import pytest
from tools.verify_cdc_motion import Stream, reduced_limits, experiment
from studio.device import MockDevice


def test_chunk_boundaries_strict_rows_and_staleness():
    s = Stream()
    s.feed(b"partial\n0.10,-0.", 1)
    s.feed(b"02,0.15\n0.20,0.00,0.35\n", 1.01)
    assert len(s.rows) == 2
    assert s.rows[0]["values"] == [.1, -.02, .15]
    s.check(1.1)
    with pytest.raises(TimeoutError):
        s.check(1.4)
    s.feed(b"nan,0.00,0.00\n", 1.5)
    with pytest.raises(RuntimeError):
        s.check(1.5)


class Clock:
    t = 0
    def now(self): return self.t
    def sleep(self, seconds): self.t += seconds


class Feed:
    error = None
    invalid = []
    def check(self, now): pass


class FaultAfterTarget(MockDevice):
    def set_opening(self, target):
        result = super().set_opening(target)
        self.error = 7
        return result


@pytest.mark.parametrize("kind", ["ok", "fault", "target_timeout", "disable_timeout", "partial_config", "initial_reject"])
def test_sequence_aborts_without_replay_and_restores_only_when_disabled(tmp_path, kind):
    d = (FaultAfterTarget() if kind == "fault" else MockDevice())
    original = d.limits()
    clock, report = Clock(), {}
    if kind == "target_timeout":
        def fail_target(target):
            d.calls.append(("opening_uncertain", target))
            raise TimeoutError()
        d.set_opening = fail_target
    if kind in ("disable_timeout", "initial_reject"):
        state = d.request_state
        def fail_disable(enabled):
            if not enabled and (kind == "initial_reject" or any(c == ("state", True) for c in d.calls)):
                d.calls.append(("disable_uncertain",))
                raise TimeoutError()
            state(enabled)
        d.request_state = fail_disable
    if kind == "partial_config":
        set_limit = d.set_limit
        first = [True]
        def fail_once(name, value):
            set_limit(name, value)
            if first[0]:
                first[0] = False
                raise TimeoutError()
        d.set_limit = fail_once
    try:
        experiment(d, Feed(), report, tmp_path / "stop", dwell=.4, clock=clock.now, sleep=clock.sleep)
    except (TimeoutError, RuntimeError):
        assert kind != "ok"
    openings = [c for c in d.calls if c[0].startswith("opening")]
    assert len(openings) == (3 if kind in ("ok", "disable_timeout") else 1 if kind in ("fault", "target_timeout") else 0)
    if kind == "disable_timeout":
        assert report["disable_confirmed"] is False
        assert report["limits_restored"] is False
        assert d.values == reduced_limits(original)
    else:
        assert d.values == original
    if kind == "initial_reject":
        assert d.calls == [("disable_uncertain",)]
    if kind == "ok":
        assert report["status"] == "completed"


def test_stop_before_start_never_enables_or_moves(tmp_path):
    stop = tmp_path / "stop"
    stop.touch()
    d = MockDevice()
    with pytest.raises(RuntimeError):
        experiment(d, Feed(), {}, stop)
    assert d.calls == []
