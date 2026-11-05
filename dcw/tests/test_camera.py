import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from camera import FrameStats, decode_color, decode_depth


class DataSemantics(unittest.TestCase):
    def test_high_resolution_host_clock_and_wrong_device_time_rate(self):
        stats = FrameStats()
        for i in range(3):
            stats.add(dict(host_monotonic_ns=i*31_000_000, host_perf_counter_ns=i*33_000_000,
                           index=i, device_timestamp_us=i*495_000_000))
        result = stats.summary()
        self.assertAlmostEqual(result['host_fps_between_first_last'], 1/.033)
        self.assertEqual(result['device_timestamp_elapsed_over_host_elapsed'], 15000)
        self.assertFalse(result['device_timestamp_rate_plausible_as_us'])

    def test_depth_units_and_invalid_zero(self):
        raw, mm = decode_depth(np.array([0, 10000], dtype="<u2").tobytes(), 2, 1, 12, .1)
        np.testing.assert_array_equal(raw, [[0, 10000]])
        np.testing.assert_allclose(mm, [[0, 1000]])

    def test_packed_or_truncated_depth_rejected(self):
        with self.assertRaises(ValueError):
            decode_depth(b"\0\0\0", 2, 1, 12, 1)
        with self.assertRaises(ValueError):
            decode_depth(b"\0\0", 1, 1, 12, float("nan"))

    def test_rgb_is_converted_to_opencv_bgr(self):
        bgr = decode_color(bytes([255, 0, 0]), 1, 1, 22)
        np.testing.assert_array_equal(bgr, [[[0, 0, 255]]])

    def test_mjpeg_and_bad_dimensions(self):
        _, data = cv2.imencode(".jpg", np.full((6, 8, 3), 127, dtype=np.uint8))
        self.assertEqual(decode_color(data.tobytes(), 8, 6, 5).shape, (6, 8, 3))
        with self.assertRaises(ValueError):
            decode_color(data.tobytes(), 7, 6, 5)

    def test_gaps_duplicates_and_sdk_timestamp_reset(self):
        stats = FrameStats()
        for ns, index, stamp in ((0, 10, 100), (100_000_000, 12, 200), (200_000_000, 12, 0)):
            stats.add(dict(host_monotonic_ns=ns, index=index, device_timestamp_us=stamp))
        summary = stats.summary()
        self.assertEqual(summary["index_gaps"], 1)
        self.assertEqual(summary["index_nonmonotonic_count"], 1)
        self.assertEqual(summary["device_timestamp_nonmonotonic_count"], 1)
        self.assertEqual(summary["device_timestamp_zero_count"], 1)
        self.assertEqual(summary["host_fps_between_first_last"], 10)


if __name__ == "__main__":
    unittest.main()
