import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import verify_can_control as base
import verify_can_stability as stress


class Clock:
    def __init__(self): self.value = 0.
    def now(self): return self.value
    def sleep(self, seconds): self.value += max(seconds, .001)


class Channel:
    def __init__(self, clock, positions=None, fail=False):
        self.clock, self.positions, self.fail = clock, positions, fail
        self.commands = []
    def exchange(self, command):
        self.commands.append(command)
        if self.fail: raise base.CheckFailed("timeout")
        pos = self.positions[min(len(self.commands)-1, len(self.positions)-1)] if self.positions else command[1]
        return {"state": 0, "position_raw": pos, "position_percent": pos*100/255,
                "round_trip_ms": .8, "fault": 0}


class TestFeedbackGates(unittest.TestCase):
    def runner(self, clock):
        args = SimpleNamespace(stop_file=str(TOOLS / "NO_SUCH_STOP_FILE"), full_stroke=False)
        report = {"stages": [], "events": []}
        return stress.Acceptance(args, report, clock=clock.now, sleep=clock.sleep)

    def test_target_advances_only_after_stable_matching_feedback(self):
        clock = Clock(); runner = self.runner(clock)
        runner.channel = Channel(clock, [50, 55, 60, 64])
        runner.step(25, deadline=2, hold=.1)
        self.assertTrue(runner.report["stages"][0]["completed"])
        self.assertGreater(runner.report["stages"][0]["first_stable_at_s"], .2)

    def test_mismatched_at_target_cannot_pass(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock, [10])
        with self.assertRaises(base.CheckFailed): runner.step(25, deadline=.5)
        self.assertNotIn("completed", runner.report["stages"][0])

    def test_hold_drift_after_arrival_aborts(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock, [64]*4+[80])
        with self.assertRaises(base.CheckFailed): runner.step(25, duration=2)

    def test_feedback_timeout_is_not_retried(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock, fail=True)
        with self.assertRaises(base.CheckFailed): runner.step(25)
        self.assertEqual(len(runner.channel.commands), 1)

    def test_cancelled_runner_never_sends(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock)
        runner.cancelled = True
        with self.assertRaises(base.CheckFailed): runner.step(35)
        self.assertEqual(runner.channel.commands, [])

    def test_duration_limit_is_bounded(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock)
        clock.value = 901
        with self.assertRaises(base.CheckFailed): runner.step(25)
        self.assertEqual(runner.channel.commands, [])

    def test_default_bounds_still_reject_full_open(self):
        clock = Clock(); runner = self.runner(clock); runner.channel = Channel(clock)
        with self.assertRaises(ValueError): runner.step(100)
        self.assertEqual(runner.channel.commands, [])


if __name__ == "__main__": unittest.main()
