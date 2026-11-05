import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("can_check", Path(__file__).resolve().parents[1] / "tools/verify_can_control.py")
can = importlib.util.module_from_spec(spec)
spec.loader.exec_module(can)


class TestCanProtocol(unittest.TestCase):
    def setUp(self):
        self.command = can.encode_command(15)

    def packet(self, data, frame_id=1, dlc=8):
        return can.FRAME.pack(frame_id, dlc, data)

    def feedback(self, fault=0, state=0, pos=38):
        return bytes((fault, state, pos, 0, 5, 25, 0, 0))

    def test_bounded_bytes_and_reserved_fields(self):
        self.assertEqual(self.command.hex(), "0026194019190000")
        self.assertEqual(can.encode_command(35).hex(), "0059194019190000")
        for bad in (0, 100, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                can.encode_command(bad)
        with self.assertRaises(ValueError):
            can.encode_command(15, {**can.LIMITS, "velocity": 255})

    def test_full_stroke_requires_explicit_valid_bounds(self):
        self.assertEqual(can.encode_command(0, bounds=(0, 100))[1], 0)
        self.assertEqual(can.encode_command(100, bounds=(0, 100))[1], 255)
        for bounds in ((-1, 100), (0, 101), (35, 15), (0, float("inf"))):
            with self.assertRaises(ValueError):
                can.encode_command(25, bounds=bounds)

    def test_local_and_adapter_echo_never_observation(self):
        self.assertIsNone(can.decode_feedback(self.packet(self.feedback()), 4, 1, self.command))
        self.assertIsNone(can.decode_feedback(self.packet(self.command), 0, 1, self.command))

    def test_other_ids_and_non_data_frames_ignored(self):
        for frame_id in (2, 1 | can.CAN_EFF, 1 | can.CAN_RTR):
            self.assertIsNone(can.decode_feedback(self.packet(self.feedback(), frame_id), 0, 1, self.command))

    def test_fault_stall_drop_error_and_malformed_abort(self):
        cases = [(self.packet(self.feedback(fault=1)), 0),
                 (self.packet(self.feedback(state=2)), 0),
                 (self.packet(self.feedback(state=3)), 0),
                 (self.packet(self.feedback(state=4)), 0),
                 (self.packet(self.feedback(), can.CAN_ERR | 64), 0),
                 (self.packet(self.feedback(), dlc=7), 0), (b"bad", 0)]
        for packet, flags in cases:
            with self.assertRaises(can.CheckFailed):
                can.decode_feedback(packet, flags, 1, self.command)

    def test_stall_opt_in_still_rejects_fault_and_drop(self):
        self.assertEqual(can.decode_feedback(self.packet(self.feedback(state=2)), 0, 1,
                                             self.command, allow_stalled=True)['state'], 2)
        for data in [self.feedback(fault=1,state=2),self.feedback(state=3)]:
            with self.assertRaises(can.CheckFailed):
                can.decode_feedback(self.packet(data),0,1,self.command,allow_stalled=True)

    def test_echo_tail_checked(self):
        with self.assertRaises(can.CheckFailed):
            can.decode_feedback(self.packet(bytes((0, 0, 38, 0, 0, 0, 0, 0))), 0, 1, self.command)

    def test_at_target_also_requires_matching_position(self):
        feedback = can.decode_feedback(self.packet(self.feedback()), 0, 1, self.command)
        self.assertTrue(can.arrived(feedback, 38))
        self.assertFalse(can.arrived(feedback, 89))
        feedback["state"] = 1
        self.assertFalse(can.arrived(feedback, 38))


class Clock:
    value = 0.0
    def now(self): return self.value
    def sleep(self, duration): self.value += max(duration, .001)


class FakeChannel:
    def __init__(self, fail=False): self.commands = []; self.fail = fail
    def exchange(self, command):
        self.commands.append(command)
        if self.fail: raise can.CheckFailed("timeout")
        return {"state": 0, "position_raw": command[1], "position_percent": command[1] * 100 / 255}


class TestSequenceGates(unittest.TestCase):
    def test_timeout_does_not_retry_or_issue_next_target(self):
        channel, clock, report = FakeChannel(True), Clock(), {}
        with self.assertRaises(can.CheckFailed):
            can.run_sequence(channel, [15, 35, 15], report, dwell=1, clock=clock.now, sleep=clock.sleep)
        self.assertEqual(len(channel.commands), 1)
        self.assertEqual(len(report["stages"]), 1)

    def test_stop_prevents_all_motion(self):
        channel, report = FakeChannel(), {}
        with self.assertRaises(can.CheckFailed):
            can.run_sequence(channel, [15], report, stopped=lambda: True)
        self.assertEqual(channel.commands, [])

    def test_sequence_requires_three_at_target_reports(self):
        channel, clock, report = FakeChannel(), Clock(), {}
        can.run_sequence(channel, [15, 35, 15], report, dwell=1, clock=clock.now, sleep=clock.sleep)
        self.assertTrue(report["sequence_completed"])
        self.assertEqual([stage["last_feedback"]["position_raw"] for stage in report["stages"]], [38, 89, 38])
        self.assertTrue(all(stage["sample_count"] >= 3 for stage in report["stages"]))


if __name__ == "__main__":
    unittest.main()
