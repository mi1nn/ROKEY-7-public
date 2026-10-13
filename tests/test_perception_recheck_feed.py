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


if __name__ == '__main__':
    unittest.main()
