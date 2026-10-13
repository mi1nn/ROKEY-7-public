"""Exit QR on TB-6 RGB-D (OPS-136, owner decision 2026-10-13 evening).

An exit is a QR whose content equals the field value exactly, seen in N consecutive frames, each
with a depth-backed position, all within a tolerance. Anything else is no exit: a misread must never
create one.
"""
import math
import unittest

from rescue_perception.exit_qr import ExitQrConfirmer, ExitQrPolicy, detect_qr, qr_point

POLICY = ExitQrPolicy(payload='ROKEY7-EXIT', required_frames=3, position_tolerance_m=0.2, min_valid_depth_ratio=0.5)
P = (1.0, 0.0, 2.0)


class PolicyTests(unittest.TestCase):
    def test_rejects_unusable_values(self):
        for bad in (dict(payload=''), dict(payload='  '), dict(required_frames=0), dict(required_frames=2.0),
                    dict(position_tolerance_m=0.0), dict(position_tolerance_m=math.nan),
                    dict(min_valid_depth_ratio=0.0), dict(min_valid_depth_ratio=1.5)):
            values = dict(payload='ROKEY7-EXIT', required_frames=3, position_tolerance_m=0.2, min_valid_depth_ratio=0.5)
            values.update(bad)
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ExitQrPolicy(**values)


class ConfirmerTests(unittest.TestCase):
    def test_confirmed_only_on_the_nth_consecutive_matching_frame(self):
        c = ExitQrConfirmer(POLICY)
        self.assertEqual([False, False, True], [c.add('ROKEY7-EXIT', P) for _ in range(3)])

    def test_other_content_never_counts_and_breaks_the_run(self):
        c = ExitQrConfirmer(POLICY)
        frames = [('ROKEY7-EXIT', P), ('ROKEY7-EXIT', P), ('ROKEY7-EXIT2', P), ('ROKEY7-EXIT', P), ('ROKEY7-EXIT', P)]
        self.assertEqual([False] * 5, [c.add(*f) for f in frames])
        self.assertFalse(any(c.add('rokey7-exit', P) for _ in range(5)))   # exact match only

    def test_a_frame_without_qr_or_position_breaks_the_run(self):
        for gap in ((None, None), ('ROKEY7-EXIT', None)):
            c = ExitQrConfirmer(POLICY)
            results = [c.add('ROKEY7-EXIT', P), c.add('ROKEY7-EXIT', P), c.add(*gap), c.add('ROKEY7-EXIT', P)]
            with self.subTest(gap=gap):
                self.assertEqual([False] * 4, results)

    def test_scattered_positions_are_not_one_exit(self):
        c = ExitQrConfirmer(POLICY)
        far = (1.5, 0.0, 2.0)
        self.assertEqual([False, False, False, False, True],
                         [c.add('ROKEY7-EXIT', p) for p in (P, P, far, far, far)])   # restarts at the far point

    def test_runs_again_after_a_confirmation(self):
        c = ExitQrConfirmer(POLICY)
        self.assertEqual([False, False, True, False, False, True], [c.add('ROKEY7-EXIT', P) for _ in range(6)])


class DetectTests(unittest.TestCase):
    def test_reads_content_and_four_corners_from_an_image(self):
        import cv2
        import numpy as np
        code = cv2.resize(cv2.QRCodeEncoder.create().encode('ROKEY7-EXIT'), None, fx=8, fy=8,
                          interpolation=cv2.INTER_NEAREST)
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        h, w = code.shape
        image[100:100 + h, 200:200 + w] = cv2.cvtColor(code, cv2.COLOR_GRAY2BGR)
        found = detect_qr(image)
        self.assertEqual('ROKEY7-EXIT', found[0])
        self.assertEqual(4, len(found[1]))
        us = [u for u, _ in found[1]]
        self.assertTrue(190 <= min(us) and max(us) <= 210 + w)

    def test_no_qr_is_none(self):
        import numpy as np
        self.assertIsNone(detect_qr(np.full((480, 640, 3), 255, dtype=np.uint8)))


class PointTests(unittest.TestCase):
    CORNERS = ((300.0, 220.0), (340.0, 220.0), (340.0, 260.0), (300.0, 260.0))   # centre (320, 240)

    def rows(self, value):
        return [[value] * 640 for _ in range(480)]

    def test_back_projects_the_centre_at_the_median_depth(self):
        point = qr_point(self.CORNERS, self.rows(2.0), fx=500.0, fy=500.0, cx=320.0, cy=240.0, undistort=None,
                         min_valid_ratio=0.5)
        self.assertEqual((0.0, 0.0, 2.0), tuple(round(c, 6) for c in point))

    def test_corners_are_undistorted_before_depth_and_projection(self):
        shifted = lambda points: [(u + 50.0, v) for u, v in points]
        point = qr_point(self.CORNERS, self.rows(2.0), fx=500.0, fy=500.0, cx=320.0, cy=240.0, undistort=shifted,
                         min_valid_ratio=0.5)
        self.assertAlmostEqual(0.2, point[0])   # 50 px * 2.0 m / 500 px

    def test_no_or_too_little_depth_is_none(self):
        self.assertIsNone(qr_point(self.CORNERS, self.rows(math.nan), fx=500.0, fy=500.0, cx=320.0, cy=240.0,
                                   undistort=None, min_valid_ratio=0.5))
        rows = self.rows(2.0)
        for v in range(232, 480):   # only the top part of the inner square has depth
            rows[v] = [0.0] * 640
        self.assertIsNone(qr_point(self.CORNERS, rows, fx=500.0, fy=500.0, cx=320.0, cy=240.0, undistort=None,
                                   min_valid_ratio=0.5))


if __name__ == '__main__':
    unittest.main()
