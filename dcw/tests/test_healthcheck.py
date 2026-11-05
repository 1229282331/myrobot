import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from healthcheck import PixelNoise, depth_metrics, validate_calibration
from analyze_health import timing


class HealthMetrics(unittest.TestCase):
    def test_monotonic_but_wrong_timestamp_unit_is_rejected(self):
        rows = [dict(stream='color', host_monotonic_ns=str(n*33000000), host_perf_counter_ns=str(n*33333333),
                     device_timestamp_us=str(n*500000000), sdk_system_timestamp_us=str(n*33333)) for n in range(10)]
        result = timing(rows)['color']
        self.assertFalse(result['device_timestamp_us']['rate_plausible_as_microseconds'])
        self.assertTrue(result['sdk_system_timestamp_us']['rate_plausible_as_microseconds'])
        self.assertEqual(result['host_clock'], 'host_perf_counter_ns')

    def test_invalid_depth_does_not_become_zero_distance_or_noise(self):
        noise = PixelNoise()
        for i in range(32):
            a = np.full((100, 100), 1000 + (i % 2) * 2, dtype=np.float32)
            a[50, 50] = 0
            noise.add(a)
        result = noise.summary()
        self.assertEqual(result['pixels_with_30_samples'], 6399)
        self.assertAlmostEqual(result['median_pixel_std_mm'], np.std([1000, 1002]*16, ddof=1))
        self.assertIsNone(depth_metrics(np.zeros((100, 100)))['center_median_mm'])

    def test_empty_calibration_rejected_and_identity_rotation_valid(self):
        intrinsic = dict(fx=400, fy=400, cx=320, cy=240, width=640, height=480)
        param = dict(depthIntrinsic=intrinsic, rgbIntrinsic=intrinsic,
                     transform=dict(rot=np.eye(3).ravel().tolist(), trans=[12, 0, 0]))
        result = validate_calibration(param)
        self.assertTrue(result['depthIntrinsic'])
        self.assertEqual(result['rotation_determinant'], 1)
        self.assertEqual(result['translation_norm_mm'], 12)
        param['depthIntrinsic'] = {k: 0 for k in intrinsic}
        self.assertFalse(validate_calibration(param)['depthIntrinsic'])


if __name__ == '__main__':
    unittest.main()
