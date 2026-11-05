"""Offline checks only: all attempts to open a real TCP connection are blocked."""

from collections import deque
import json
import logging
from pathlib import Path
import unittest
from unittest.mock import patch

from tools import e6_zero as zero


Q = (78.6759, 117.3672, -130.1950, 144.6762, -21.9205, 80.6561)
Z = (0.0,) * 6
READS = {"RobotMode()", "GetAngle()", "GetErrorID()"}


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakeRobot:
    def __init__(self, samples, fail_motion=None, interrupt_after_motion=False):
        self.samples = deque(samples)
        self.current = None
        self.commands = []
        self.fail_motion = fail_motion
        self.interrupt_after_motion = interrupt_after_motion
        self.motion_sent = False
        self.clock = None
        self.slow_after_motion = False

    def command(self, command, **kwargs):
        self.commands.append(command)
        if command == "RobotMode()":
            if self.motion_sent and self.interrupt_after_motion:
                raise KeyboardInterrupt()
            if self.samples:
                self.current = self.samples.popleft()
            return str(self.current[0])
        if command == "GetAngle()":
            if self.motion_sent and self.slow_after_motion:
                self.clock.sleep(1.1)
            return ",".join(str(v) for v in self.current[1])
        if command == "GetErrorID()":
            return json.dumps(self.current[2] if len(self.current) > 2 else [])
        if command.startswith("MovJ("):
            self.motion_sent = True
            if self.fail_motion:
                raise self.fail_motion
            return "42"
        if command in ("EnableRobot()", "Stop()"):
            return ""
        raise AssertionError(f"Unexpected command: {command}")


class ZeroTests(unittest.TestCase):
    def setUp(self):
        block = patch("socket.create_connection", side_effect=AssertionError("Real TCP forbidden in tests"))
        block.start()
        self.addCleanup(block.stop)
        self.clock = Clock()
        logger = patch.object(zero.LOG, "disabled", True)
        logger.start()
        self.addCleanup(logger.stop)

    def run_robot(self, robot, **kwargs):
        robot.clock = self.clock
        return zero.run_to_zero(robot, confirm=lambda _: "GO ZERO", clock=self.clock,
                                sleep=self.clock.sleep, **kwargs)

    def writes(self, robot):
        return [cmd for cmd in robot.commands if cmd not in READS]

    def test_default_preview_never_writes(self):
        robot = FakeRobot([(4, Q)])
        self.assertEqual(self.run_robot(robot), "preview")
        self.assertEqual(self.writes(robot), [])

    def test_cancel_never_enables_or_moves(self):
        robot = FakeRobot([(4, Q)])
        result = zero.run_to_zero(robot, execute=True, confirm=lambda _: "no", clock=self.clock)
        self.assertEqual(result, "cancelled")
        self.assertEqual(self.writes(robot), [])

    def test_already_zero_never_enables(self):
        robot = FakeRobot([(4, Z)])
        self.assertEqual(self.run_robot(robot, execute=True), "already_at_zero")
        self.assertEqual(self.writes(robot), [])

    def test_busy_state_rejected_without_stop(self):
        robot = FakeRobot([(7, Q)])
        with self.assertRaises(zero.E6Error):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [])

    def test_alarm_rejected_without_clear_error(self):
        robot = FakeRobot([(4, Q, [[], [1537], []])])
        with self.assertRaisesRegex(zero.E6Error, "1537"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [])

    def test_invalid_joints_rejected(self):
        for q in [(0,) * 5, (float("nan"),) + Z[1:], (0, 200, 0, 0, 0, 0)]:
            with self.subTest(q=q):
                robot = FakeRobot([(4, q)])
                with self.assertRaises(zero.E6Error):
                    self.run_robot(robot, execute=True)
                self.assertEqual(self.writes(robot), [])

    def test_large_turn_not_wrapped_or_executed(self):
        robot = FakeRobot([(4, (350.0,) + Z[1:])])
        with self.assertRaisesRegex(zero.E6Error, "180"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [])

    def test_changed_start_after_confirmation_rejected(self):
        changed = (Q[0] + 2.0,) + Q[1:]
        robot = FakeRobot([(4, Q), (4, changed)])
        with self.assertRaisesRegex(zero.E6Error, "变化"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [])

    def test_enable_and_wait_for_three_stable_idle_samples(self):
        moving = tuple(v / 2 for v in Q)
        # A target sample during motion is not completion; an out-of-tolerance
        # idle sample resets the stable counter.
        robot = FakeRobot([(4, Q), (4, Q), (5, Q), (5, Q), (7, moving),
                           (7, Z), (5, Z), (5, (0.2,) + Z[1:]), (5, Z), (5, Z), (5, Z)])
        self.assertEqual(self.run_robot(robot, execute=True), "reached_zero")
        self.assertFalse(robot.samples)
        self.assertEqual(self.writes(robot), ["EnableRobot()", zero.move_command(5, 5)])

    def test_enabled_arm_is_not_enabled_again(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (5, Z), (5, Z), (5, Z)])
        self.assertEqual(self.run_robot(robot, execute=True), "reached_zero")
        self.assertEqual(self.writes(robot), [zero.move_command(5, 5)])

    def test_motion_ack_failure_stops_without_retry(self):
        for error in (TimeoutError("lost acknowledgement"), zero.E6Error("controller rejected")):
            with self.subTest(error=error):
                robot = FakeRobot([(5, Q), (5, Q), (5, Q)], fail_motion=error)
                with self.assertRaises(type(error)):
                    self.run_robot(robot, execute=True)
                self.assertEqual(self.writes(robot), [zero.move_command(5, 5), "Stop()"])

    def test_interrupt_after_motion_attempts_stop(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q)], interrupt_after_motion=True)
        with self.assertRaises(KeyboardInterrupt):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [zero.move_command(5, 5), "Stop()"])

    def test_motion_timeout_stops(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (7, Q)])
        with self.assertRaisesRegex(zero.E6Error, "超时"):
            self.run_robot(robot, execute=True, motion_timeout=5)
        self.assertEqual(self.writes(robot), [zero.move_command(5, 5), "Stop()"])

    def test_alarm_during_motion_stops(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (9, Q, [[42]])])
        with self.assertRaisesRegex(zero.E6Error, "报警"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot), [zero.move_command(5, 5), "Stop()"])

    def test_stale_query_during_motion_stops(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (7, Q)])
        robot.slow_after_motion = True
        with self.assertRaisesRegex(zero.E6Error, "过期"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot)[-1], "Stop()")

    def test_motion_outside_expected_joint_interval_stops(self):
        wrong_way = (Q[0] + 3.0,) + Q[1:]
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (7, wrong_way)])
        with self.assertRaisesRegex(zero.E6Error, "偏离"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot)[-1], "Stop()")

    def test_disabled_during_motion_is_failure(self):
        robot = FakeRobot([(5, Q), (5, Q), (5, Q), (4, Z)])
        with self.assertRaisesRegex(zero.E6Error, "非预期状态"):
            self.run_robot(robot, execute=True)
        self.assertEqual(self.writes(robot)[-1], "Stop()")

    def test_execute_requires_terminal_before_connecting(self):
        with patch("sys.stdin.isatty", return_value=False), patch("sys.stderr"):
            with self.assertRaises(SystemExit) as error:
                zero.main(["--execute"])
        self.assertEqual(error.exception.code, 2)

    def test_captured_real_readonly_replies_parse(self):
        path = Path(__file__).resolve().parents[1] / "local-network/e6-readonly-2026-09-19-tcp-mode.json"
        report = json.loads(path.read_text(encoding="utf-8"))
        for item in report["dashboard"]["queries"]:
            self.assertEqual(zero.parse_reply(item["raw"], item["command"]), item["payload"])

    def test_parser_rejects_error_wrong_echo_and_non_tcp_mode(self):
        cases = [("-1,{},EnableRobot();", "EnableRobot()"),
                 ("0,{4},GetAngle();", "RobotMode()"),
                 ("Control Mode Is Not Tcp", "RobotMode()"),
                 ("0,{4},RobotMode()", "RobotMode()")]
        for raw, command in cases:
            with self.subTest(raw=raw), self.assertRaises(zero.E6Error):
                zero.parse_reply(raw, command)

    def test_motion_is_joint_target_and_echo_checked(self):
        command = zero.move_command(5, 5)
        self.assertIn("joint={0.000000,0.000000,0.000000,0.000000,0.000000,0.000000}", command)
        self.assertNotIn("pose=", command)
        self.assertEqual(zero.parse_reply(f"0,{{42}},{command};", command), "42")
        with self.assertRaises(zero.E6Error):
            zero.parse_reply(f"0,{{42}},{command.replace('v=5', 'v=100')};", command)
        with self.assertRaises(zero.E6Error):
            zero.move_command(100, 5)


if __name__ == "__main__":
    unittest.main()
