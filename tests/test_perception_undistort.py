"""TB-6 RGB is not rectified (measured 2026-10-13): detection points are undistorted, not refused.

Measured on TB-6 (read only, 2026-10-13 12:38-12:44): RGB and aligned Depth CameraInfo share
frame_id oakd_rgb_camera_optical_frame, 704x704, the same K and P; the RGB D is rational_polynomial
(below), the Depth D is all zero, no rect topic. So the depth grid is the undistorted RGB grid (P).
The box corners and centre are undistorted with K·D onto P (cv2.undistortPoints); the whole image is
not. Unsupported models are still ALIGNMENT_UNVERIFIED. Depth arrives about 0.78 s after RGB (RGB
29.7 Hz), so the pairing queue and the CameraInfo age are field values, not code defaults.
"""
from dataclasses import replace
from types import SimpleNamespace as NS
import unittest

from rescue_perception import (AlignmentEvidence, CameraIntrinsics, DepthFrame, DepthPolicy, Detection, FrameMeta,
                               SourceBinding, camera_intrinsics, locate_detection)
from rescue_perception.camera_utils import field_values
from rescue_perception.target_utils import FIELD_PARAMS, SETTINGS

FRAME = 'oakd_rgb_camera_optical_frame'
TB6_D = (8.75, -106.59, -0.0027, -0.0016, 506.56, 8.48, -104.17, 495.67)   # measured RGB D (rounded)


def shift_right(points):
    """Synthetic undistortion: every point moves 2 px right."""
    return [(u + 2.0, v) for u, v in points]


class CoreUndistortTests(unittest.TestCase):
    def setUp(self):
        self.rgb = FrameMeta(FRAME, 100_000_000_000, 'clock', 'CAPTURE', 8, 8, 'cal')
        rows = [[1000, 1000] + [3000] * 6 for _ in range(8)]   # columns 0-1 at 1 m, the rest at 3 m
        self.depth = DepthFrame(self.rgb, '16UC1', 'mm', rows)
        self.alignment = AlignmentEvidence('align', 'cal', FRAME, FRAME, True)
        self.policy = DepthPolicy(20_000_000, 0.5, 5.0, 2, 0.5, 0.5)
        self.detection = Detection(0, 'survivor', 0.9, (0, 0, 2, 8), None, 'DETECTION_ONLY')
        self.raw = CameraIntrinsics(FRAME, 'cal', 8, 8, 4.0, 4.0, 3.5, 3.5, False)

    def locate(self, intrinsics, detection=None):
        return locate_detection(self.rgb, self.depth, intrinsics, self.alignment, detection or self.detection,
                                self.policy)

    def test_unrectified_without_a_model_is_still_refused(self):
        self.assertEqual('ALIGNMENT_UNVERIFIED', self.locate(self.raw).reason)

    def test_undistorted_box_reads_the_aligned_depth_where_the_point_really_is(self):
        undistorting = replace(self.raw, undistort=shift_right, undistortion_id='test:shift2')
        result = self.locate(undistorting)
        self.assertEqual('LOCATED', result.reason)
        x, y, z = result.point_optical
        self.assertAlmostEqual(3.0, z)                        # the 1 m columns would be read without undistortion
        self.assertAlmostEqual((2.5 - 3.5) * 3.0 / 4.0, x)    # centre 0.5 -> 2.5 px after undistortion
        self.assertEqual('test:shift2', result.undistortion_id)
        self.assertIsNone(self.locate(replace(self.raw, rectified=True)).undistortion_id)

    def test_box_undistorted_out_of_the_image_has_no_depth(self):
        away = replace(self.raw, undistort=lambda pts: [(u + 50.0, v) for u, v in pts], undistortion_id='t')
        self.assertEqual('DEPTH_INSUFFICIENT', self.locate(away).reason)


def tb6_info(model='rational_polynomial', d=TB6_D):
    k = (500.0, 0.0, 352.0, 0.0, 500.0, 352.0, 0.0, 0.0, 1.0)
    p = (500.0, 0.0, 352.0, 0.0, 0.0, 500.0, 352.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    return NS(header=NS(frame_id=FRAME, stamp=NS(sec=1, nanosec=0)), width=704, height=704, k=k, p=p, d=d,
              distortion_model=model, binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0))


class CameraInfoUndistortTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('OpenCV/NumPy not installed: undistortion NOT_RUN')
        self.binding = SourceBinding('robot_a', 'amr_rgbd', '00000000-0000-4000-8000-000000000100', 'clock',
                                     'CAPTURE', 'CAPTURE', 'cal', 'UNKNOWN', None)

    def test_supported_model_gives_a_point_undistorter_equal_to_opencv(self):
        import cv2
        import numpy as np
        intrinsics = camera_intrinsics(tb6_info(), self.binding, rectified=False)
        self.assertIsNotNone(intrinsics.undistort)
        self.assertRegex(intrinsics.undistortion_id, r'^rational_polynomial:[0-9a-f]{12}$')
        self.assertEqual((500.0, 352.0), (intrinsics.fx, intrinsics.cx))    # P: the depth grid
        point = (600.0, 120.0)
        k = np.array(tb6_info().k).reshape(3, 3)
        p = np.array(tb6_info().p).reshape(3, 4)
        want = cv2.undistortPoints(np.array([[point]]), k, np.array(TB6_D), P=p).reshape(2)
        got = intrinsics.undistort([point])[0]
        self.assertAlmostEqual(float(want[0]), got[0], places=6)
        self.assertAlmostEqual(float(want[1]), got[1], places=6)

    def test_unsupported_model_or_wrong_length_has_no_undistorter(self):
        for info in (tb6_info(model='equidistant', d=(0.1, 0.0, 0.0, 0.0)), tb6_info(d=(0.1, 0.2, 0.3))):
            with self.subTest(model=info.distortion_model, n=len(info.d)):
                self.assertIsNone(camera_intrinsics(info, self.binding, rectified=False).undistort)

    def test_rectified_stream_never_undistorts(self):
        self.assertIsNone(camera_intrinsics(tb6_info(), self.binding, rectified=True).undistort)


class PairingFieldValueTests(unittest.TestCase):
    def test_pairing_and_camera_info_age_are_field_values(self):
        for name in ('sync_queue_size', 'sync_slop_ns', 'camera_info_max_age_ns'):
            self.assertNotIn(name, SETTINGS)
            self.assertEqual('INTEGER', FIELD_PARAMS[name])

    def test_integer_values_are_checked(self):
        supplied = {name: 'x' for name, kind in FIELD_PARAMS.items() if kind == 'STRING'}
        supplied.update({name: False for name, kind in FIELD_PARAMS.items() if kind == 'BOOL'})
        supplied.update(rgb_stamp_semantics='CAPTURE', depth_stamp_semantics='CAPTURE', image_type='raw',
                        depth_transport='raw',
                        sync_queue_size=40, sync_slop_ns=20_000_000, camera_info_max_age_ns=1_500_000_000)
        self.assertEqual(40, field_values(SETTINGS, supplied, FIELD_PARAMS)['sync_queue_size'])
        for bad in (dict(sync_queue_size=0), dict(sync_queue_size=True), dict(sync_slop_ns=-1),
                    dict(camera_info_max_age_ns=0), dict(sync_queue_size=4.0)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                field_values(SETTINGS, supplied | bad, FIELD_PARAMS)


if __name__ == '__main__':
    unittest.main()
