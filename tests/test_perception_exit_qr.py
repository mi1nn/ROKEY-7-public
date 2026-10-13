"""Exit QR on TB-6 RGB-D (OPS-136, owner decision 2026-10-13 evening).

An exit is a QR whose content equals the field value exactly, seen in N consecutive frames, each
with a depth-backed position, all within a tolerance. Anything else is no exit: a misread must never
create one.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_perception.adapter import SourceBinding
from rescue_perception.exit_qr import (EXIT_PARAMS, ExitQrConfirmer, ExitQrPolicy, ExitSender, detect_qr, exit_values,
                                       fill_exit_observation, qr_point)

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
    def setUp(self):
        try:
            import cv2  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('OpenCV/NumPy not installed: QR detection NOT_RUN')

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



def message():
    return NS(time=NS(), sensor_position=NS(), evidence=NS())


BINDING = SourceBinding('robot_a', 'amr_rgbd', '00000000-0000-4000-8000-000000000100', 'rokey7-utc-field',
                        'CAPTURE', 'CAPTURE', 'cal', 'SYNCED', 7_000_000)
FIELD = dict(exit_qr_payload='ROKEY7-EXIT', exit_qr_required_frames=5, exit_qr_position_tolerance_m=0.2,
             exit_qr_min_valid_depth_ratio=0.5, exit_map_state_topic='/rescue/robots/robot_a/map_state')


class ValuesTests(unittest.TestCase):
    def test_every_value_is_required(self):
        self.assertEqual(set(FIELD), set(EXIT_PARAMS))
        for name in FIELD:
            with self.subTest(name=name), self.assertRaises(ValueError):
                exit_values({**FIELD, name: None})
        with self.assertRaises(ValueError):
            exit_values({**FIELD, 'exit_map_state_topic': ' '})

    def test_policy_and_map_state_topic(self):
        policy, topic = exit_values(FIELD)
        self.assertEqual((policy.payload, policy.required_frames), ('ROKEY7-EXIT', 5))
        self.assertEqual('/rescue/robots/robot_a/map_state', topic)


class ObservationTests(unittest.TestCase):
    def test_exit_observation_fields(self):
        corners = ((300.0, 220.0), (340.0, 221.0), (341.0, 260.0), (299.0, 259.0))
        m = fill_exit_observation(message(), observation_id='00000000-0000-4000-8000-000000000001', binding=BINDING,
                                  frame_id='oakd_rgb_camera_optical_frame', observed_at_ns=1_000, validity_ns=500,
                                  corners=corners, point=(0.1, -0.2, 2.0), covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                                  model_id='opencv.QRCodeDetector/4.6.0', model_sha256='a' * 64,
                                  evidence_id='frame-1', evidence_sha256='b' * 64)
        self.assertEqual(('exit_qr', 1.0, 'DETECTION_ONLY', False, ''),
                         (m.class_label, m.confidence, m.tracking_mode, m.has_track_id, m.track_id))
        self.assertEqual(('opencv.QRCodeDetector/4.6.0', 'a' * 64), (m.model_id, m.model_sha256))
        self.assertEqual([299.0, 220.0, 341.0, 260.0], m.bbox_xyxy_pixels)
        self.assertEqual(('VALID', 'LOCATED', 'oakd_rgb_camera_optical_frame'),
                         (m.localization_status, m.depth_status, m.sensor_frame))
        self.assertEqual((0.1, -0.2, 2.0), (m.sensor_position.x, m.sensor_position.y, m.sensor_position.z))
        self.assertEqual((1_000, 1_500, 'CAPTURE', 'rokey7-utc-field', 'SYNCED', True, 7_000_000),
                         (m.time.observed_at_ns, m.time.valid_until_ns, m.time.stamp_semantics,
                          m.time.clock_domain_id, m.time.clock_state, m.time.has_clock_error_bound,
                          m.time.clock_error_bound_ns))
        self.assertEqual(('robot_a', 'amr_rgbd'), (m.source_robot, m.source_camera))
        self.assertEqual(('frame-1', 'b' * 64, 1_000), (m.evidence.evidence_id, m.evidence.sha256,
                                                        m.evidence.observed_at_ns))

    def test_unmeasured_clock_is_carried_as_unknown(self):
        from dataclasses import replace
        m = fill_exit_observation(message(), observation_id='00000000-0000-4000-8000-000000000001',
                                  binding=replace(BINDING, clock_state='UNKNOWN', clock_error_bound_ns=None),
                                  frame_id='f', observed_at_ns=1_000, validity_ns=500,
                                  corners=((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)), point=(0.0, 0.0, 1.0),
                                  covariance=[0.0] * 9, model_id='m', model_sha256='a' * 64, evidence_id='e',
                                  evidence_sha256='b' * 64)
        self.assertEqual(('UNKNOWN', False, 0), (m.time.clock_state, m.time.has_clock_error_bound,
                                                 m.time.clock_error_bound_ns))


class SenderTests(unittest.TestCase):
    def test_once_per_map_revision(self):
        s = ExitSender()
        self.assertEqual([True, False], [s.send(('s1', 3)), s.send(('s1', 3))])
        self.assertEqual([True, False], [s.send(('s1', 4)), s.send(('s1', 4))])   # same QR, new revision

    def test_sent_without_a_map_is_sent_again_once_the_map_is_known(self):
        s = ExitSender()   # target_locator refuses it without a VALID MapState
        self.assertEqual([True, False, True], [s.send(None), s.send(None), s.send(('s1', 0))])


if __name__ == '__main__':
    unittest.main()
