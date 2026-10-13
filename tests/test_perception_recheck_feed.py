"""amr_target frame/scan -> recheck window inputs (2026-10-13). Pure; no ROS, no model.

The recheck server (#165) counts what amr_target sees while a window is open: whether the watched
map point is in view with depth behind it, where confirmed detections are in the map, and whether a
detection that is not confirmed sits on the point (then the camera cannot say ABSENT).
"""
import math
import unittest

from rescue_mapping import RigidTransform
from rescue_perception.recheck import CameraModel, RecheckPolicy
from rescue_perception.recheck_feed import Candidate, depth_median, frame_evidence, rigid_from_tf, scan_fields

POLICY = RecheckPolicy(presence_radius_m=0.5, lidar_radius_m=0.3, min_frames=3, min_scans=3, min_lidar_rays=3,
                       min_lidar_hits=2, image_margin_px=10, min_depth_m=0.2, max_depth_m=5.0,
                       occlusion_tolerance_m=0.3, depth_window_px=2)
CAMERA = CameraModel(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
MAP_FROM_OPTICAL = RigidTransform((0.0, 0.0, 0.3), (-0.5, 0.5, -0.5, 0.5))   # looking along map +x
POINT = (2.0, 0.0, 0.3)   # projects to the image centre (320, 240)


def flat(value):
    return lambda u, v, k: value


class FrameEvidenceTests(unittest.TestCase):
    def test_confirmed_detection_goes_to_the_map(self):
        candidates = [Candidate('CONFIRMED', (0.0, 0.0, 2.1), (300, 220, 340, 260))]
        coverage, points, unconfirmed = frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(2.1), candidates, POLICY)
        self.assertEqual('VALID', coverage)
        self.assertAlmostEqual(2.1, points[0][0])
        self.assertFalse(unconfirmed)

    def test_unconfirmed_box_on_the_point_is_flagged(self):
        on_point = [Candidate('UNCONFIRMED', None, (300, 220, 340, 260))]
        self.assertTrue(frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), on_point, POLICY)[2])
        elsewhere = [Candidate('UNCONFIRMED', None, (10, 10, 60, 60))]
        self.assertFalse(frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), elsewhere, POLICY)[2])

    def test_unconfirmed_point_near_the_target_is_flagged(self):
        near = [Candidate('UNCONFIRMED', (0.0, 0.0, 2.2), (10, 10, 60, 60))]
        self.assertTrue(frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), near, POLICY)[2])

    def test_empty_view_of_the_point(self):
        self.assertEqual(('VALID', [], False), frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), [], POLICY))


    def test_unrectified_box_is_compared_after_undistortion(self):
        # #200 review REV-200-01: TB-6 RGB is not rectified, so the detector's box is in raw pixels while the
        # point is projected with P (the undistorted grid). The box is moved to that grid first, as core.py does.
        shift = lambda points: [(u + 30.0, v) for u, v in points]   # stand-in distortion: raw -> undistorted +30 px
        raw_box = [Candidate('UNCONFIRMED', None, (270, 220, 310, 260))]   # raw; undistorted (300..340) holds 320
        self.assertTrue(frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), raw_box, POLICY,
                                       undistort=shift)[2])
        self.assertFalse(frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), raw_box, POLICY)[2])  # rectified

    def test_undistortion_uses_edges_and_pixel_centres_like_core(self):
        seen = []
        def record(points):
            seen.extend(points)
            return list(points)
        frame_evidence(POINT, MAP_FROM_OPTICAL, CAMERA, flat(4.0), [Candidate('UNCONFIRMED', None, (300, 220, 340, 260))],
                       POLICY, undistort=record)
        self.assertEqual({(299.5, 219.5), (319.5, 219.5), (339.5, 219.5), (339.5, 239.5), (339.5, 259.5),
                          (319.5, 259.5), (299.5, 259.5), (299.5, 239.5)}, set(seen))

class HelperTests(unittest.TestCase):
    def test_depth_median_ignores_invalid_pixels(self):
        rows = [[0.0, math.nan, 2.0], [2.2, 0.0, 2.1], [math.inf, -1.0, 0.0]]
        self.assertAlmostEqual(2.1, depth_median(rows, 1, 1, 1))
        self.assertIsNone(depth_median([[0.0, 0.0], [0.0, 0.0]], 0, 0, 1))
        self.assertAlmostEqual(2.0, depth_median(rows, 2, 0, 0))   # window clipped at the edge

    def test_tf_and_scan_conversions(self):
        class V:
            def __init__(self, **kw):
                self.__dict__.update(kw)
        tf = V(transform=V(translation=V(x=1.0, y=2.0, z=0.3), rotation=V(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.assertEqual((2.0, 2.0, 0.3), rigid_from_tf(tf).apply((1.0, 0.0, 0.0)))
        scan = V(ranges=[1.0, 2.0], angle_min=-0.1, angle_increment=0.2, range_min=0.1, range_max=8.0)
        self.assertEqual(dict(ranges=[1.0, 2.0], angle_min=-0.1, angle_increment=0.2, range_min=0.1, range_max=8.0),
                         scan_fields(scan))


class SettingsTests(unittest.TestCase):
    def values(self, **changes):
        from rescue_perception.recheck_feed import RECHECK_PARAMS
        v = {name: {'STRING': 'x', 'INTEGER': 3, 'DOUBLE': 0.5}[kind] for name, kind in RECHECK_PARAMS.items()}
        v.update(recheck_max_window_ns=10_000_000_000, recheck_validity_ns=5_000_000_000, recheck_min_depth_m=0.2,
                 recheck_max_depth_m=5.0, recheck_image_margin_px=10, recheck_depth_window_px=2)
        v.update(changes)
        return v

    def test_policy_comes_from_the_parameters(self):
        from rescue_perception.recheck_feed import recheck_values
        policy, other = recheck_values(self.values())
        self.assertEqual((0.5, 3, 5.0), (policy.presence_radius_m, policy.min_frames, policy.max_depth_m))
        self.assertEqual(10_000_000_000, other['recheck_max_window_ns'])

    def test_missing_or_bad_values_stop_the_node(self):
        from rescue_perception.recheck_feed import recheck_values
        for bad in (dict(recheck_scan_topic=None), dict(recheck_map_frame=''), dict(recheck_max_window_ns=0),
                    dict(recheck_min_frames=0), dict(recheck_presence_radius_m=-1.0)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                recheck_values(self.values(**bad))

    def test_depth_rows_use_the_calibration_and_drop_invalid(self):
        try:
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('NumPy not installed')
        from rescue_perception.recheck_feed import depth_rows_m
        rows = depth_rows_m([[0, 1000, 2000]], '16UC1', 0.5, 0.1)
        self.assertTrue(math.isnan(rows[0][0]))
        self.assertAlmostEqual(0.6, rows[0][1])
        self.assertAlmostEqual(1.1, depth_median(rows, 2, 0, 0))   # 2000 mm * 0.5 + 0.1 m


if __name__ == '__main__':
    unittest.main()
