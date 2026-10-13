"""Recheck window on sensor capture times (OPS-108, 2026-10-13).

Samples are timed by their capture stamp (camera / LiDAR driver host, TB-6 Pi), not by when this
host received them. The goal is accepted on this host's clock, so a sample counts only when its
capture stamp is later than the acceptance by both clocks' error bounds (C-11 rows 8·11: nothing
captured before the instruction). The observation time is CAPTURE with this host's measured bound
plus the sensor host's (sensor_host_clock_error_bound_ns, same key and value as the detection launch);
another clock domain is never SYNCED.
"""
import unittest

from rescue_perception.recheck import RecheckPolicy, RecheckWindow
from rescue_perception.recheck_server import SensorClock, capture_clock, window_margin_ns

POLICY = RecheckPolicy(presence_radius_m=0.5, lidar_radius_m=0.3, min_frames=3, min_scans=3, min_lidar_rays=3,
                       min_lidar_hits=2, image_margin_px=10, min_depth_m=0.2, max_depth_m=5.0,
                       occlusion_tolerance_m=0.3, depth_window_px=2)
NOW = 1_760_000_000_000_000_000
RECORD = dict(state='SYNCED', measured_at_ns=NOW - 10_000_000_000, sampled_at_ns=NOW, valid_until_ns=NOW + 10**12,
              error_bound_ns=300_000_000, drift_ppm=100.0)
PI = SensorClock(error_bound_ns=150_000_000, clock_domain_id='rokey7-utc-field')


class CaptureWindowTests(unittest.TestCase):
    def test_only_samples_captured_after_acceptance_plus_both_bounds_count(self):
        window = RecheckWindow((2.0, 0.0, 0.3), POLICY, start_ns=NOW, start_margin_ns=450_000_000)
        self.assertFalse(window.add_frame(stamp_ns=NOW + 449_999_999, coverage='VALID', confirmed_points=[],
                                          unconfirmed_at_point=False))
        self.assertTrue(window.add_frame(stamp_ns=NOW + 450_000_000, coverage='VALID', confirmed_points=[],
                                         unconfirmed_at_point=False))
        self.assertFalse(window.add_scan(stamp_ns=NOW, state='CLEAR'))

    def test_margin_is_host_bound_plus_sensor_bound(self):
        self.assertEqual(300_000_000 + 150_000_000, window_margin_ns(RECORD, NOW, PI))
        self.assertEqual(150_000_000, window_margin_ns(dict(state='UNKNOWN'), NOW, PI))   # no host bound claimed

    def test_observation_clock_adds_the_sensor_host(self):
        self.assertEqual(('SYNCED', 450_000_000), capture_clock(RECORD, NOW, PI, 'rokey7-utc-field'))
        other = SensorClock(error_bound_ns=0, clock_domain_id='tb6-pi-unsynced')
        self.assertEqual(('UNKNOWN', None), capture_clock(RECORD, NOW, other, 'rokey7-utc-field'))
        self.assertEqual(('UNKNOWN', None), capture_clock(dict(state='UNKNOWN'), NOW, PI, 'rokey7-utc-field'))


if __name__ == '__main__':
    unittest.main()
