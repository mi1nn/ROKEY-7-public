"""Exit QR on TB-6 RGB-D (OPS-136, owner decision 2026-10-13 evening; several exits OPS-154/155).

An exit is a QR whose content equals one of the field values exactly, seen in N consecutive checked
frames, each with a depth-backed position, all within a tolerance; each exit counts on its own.
Anything else is no exit: a misread must never create one.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_perception.adapter import SourceBinding
from rescue_perception.exit_qr import (EXIT_PARAMS, ExitQrConfirmer, ExitQrPolicy, ExitSender, detect_qrs, exit_values,
                                       fill_exit_observation, payloads_argument, qr_point, split_qrs)

POLICY = ExitQrPolicy(payloads=('EXIT-1', 'EXIT-2'), required_frames=3, position_tolerance_m=0.2,
                      min_valid_depth_ratio=0.5, every_n_frames=1)
P = (1.0, 0.0, 2.0)
Q = (-1.0, 0.0, 3.0)


class PolicyTests(unittest.TestCase):
    def test_rejects_unusable_values(self):
        for bad in (dict(payloads=()), dict(payloads=('',)), dict(payloads=('  ',)), dict(payloads=('EXIT-1', 'EXIT-1')),
                    dict(payloads='EXIT-1'), dict(payloads=['EXIT-1']), dict(payloads=('EXIT-1', 2)),
                    dict(required_frames=0), dict(required_frames=2.0),
                    dict(position_tolerance_m=0.0), dict(position_tolerance_m=math.nan),
                    dict(min_valid_depth_ratio=0.0), dict(min_valid_depth_ratio=1.5),
                    dict(every_n_frames=0), dict(every_n_frames=6.0)):
            values = dict(payloads=('EXIT-1',), required_frames=3, position_tolerance_m=0.2, min_valid_depth_ratio=0.5,
                          every_n_frames=1)
            values.update(bad)
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ExitQrPolicy(**values)


class ConfirmerTests(unittest.TestCase):
    def test_confirmed_only_on_the_nth_consecutive_matching_frame(self):
        c = ExitQrConfirmer(POLICY)
        self.assertEqual([(), (), ('EXIT-1',)], [c.add({'EXIT-1': P}) for _ in range(3)])

    def test_each_exit_counts_on_its_own(self):
        c = ExitQrConfirmer(POLICY)
        frames = [{'EXIT-1': P}, {'EXIT-1': P, 'EXIT-2': Q}, {'EXIT-1': P, 'EXIT-2': Q}, {'EXIT-2': Q}]
        self.assertEqual([(), (), ('EXIT-1',), ('EXIT-2',)], [c.add(f) for f in frames])
        both = ExitQrConfirmer(POLICY)
        self.assertEqual([(), (), ('EXIT-1', 'EXIT-2')], [both.add({'EXIT-2': Q, 'EXIT-1': P}) for _ in range(3)])

    def test_a_checked_frame_without_that_exit_or_its_position_breaks_only_its_run(self):
        for gap in ({}, {'EXIT-2': Q}, {'EXIT-1': None, 'EXIT-2': Q}):
            c = ExitQrConfirmer(POLICY)
            results = [c.add({'EXIT-1': P, 'EXIT-2': Q}), c.add({'EXIT-1': P, 'EXIT-2': Q}), c.add(gap),
                       c.add({'EXIT-1': P, 'EXIT-2': Q})]
            with self.subTest(gap=gap):
                self.assertEqual([(), ()], results[:2])
                self.assertNotIn('EXIT-1', results[2] + results[3])
                self.assertEqual(('EXIT-2',) if 'EXIT-2' in gap and gap['EXIT-2'] else (), results[2])

    def test_content_outside_the_list_is_refused(self):
        c = ExitQrConfirmer(POLICY)
        for other in ('EXIT-3', 'exit-1'):   # exact match only: split_qrs leaves these out before
            with self.subTest(other=other), self.assertRaises(ValueError):
                c.add({other: P})

    def test_scattered_positions_are_not_one_exit(self):
        c = ExitQrConfirmer(POLICY)
        far = (1.5, 0.0, 2.0)
        self.assertEqual([(), (), (), (), ('EXIT-1',)],
                         [c.add({'EXIT-1': p}) for p in (P, P, far, far, far)])   # restarts at the far point

    def test_every_nth_frame_is_checked_and_skipped_frames_do_not_break_the_run(self):
        from dataclasses import replace
        c = ExitQrConfirmer(replace(POLICY, every_n_frames=3))   # OPS-141: about 35 ms per frame at 704x704
        self.assertEqual([True, False, False, True, False, False, True], [c.due() for _ in range(7)])
        self.assertEqual([(), (), ('EXIT-1',)], [c.add({'EXIT-1': P}) for _ in range(3)])   # checked frames only

    def test_runs_again_after_a_confirmation(self):
        c = ExitQrConfirmer(POLICY)
        self.assertEqual([(), (), ('EXIT-1',), (), (), ('EXIT-1',)], [c.add({'EXIT-1': P}) for _ in range(6)])


class SplitTests(unittest.TestCase):
    C1, C2, C3 = (((1.0, 1.0),) * 4), (((2.0, 2.0),) * 4), (((3.0, 3.0),) * 4)

    def test_listed_exits_by_exact_content_others_and_duplicates_apart(self):
        found = (('EXIT-1', self.C1), ('exit-1', self.C2), ('EXIT-2', self.C2), ('EXIT-2', self.C3), ('door', self.C3))
        allowed, unknown, duplicated = split_qrs(found, POLICY.payloads)
        self.assertEqual({'EXIT-1': self.C1}, allowed)   # an exit read twice in one frame is not placed
        self.assertEqual(('exit-1', 'door'), unknown)
        self.assertEqual(('EXIT-2',), duplicated)

    def test_nothing_found(self):
        self.assertEqual(({}, (), ()), split_qrs((), POLICY.payloads))


class ArgumentTests(unittest.TestCase):
    def test_comma_separated_launch_argument(self):
        self.assertEqual(('EXIT-1', 'EXIT-2'), payloads_argument('EXIT-1,EXIT-2'))
        self.assertEqual(('EXIT 1',), payloads_argument('EXIT 1'))   # exact text, nothing stripped
        for bad in ('', 'EXIT-1,', ',EXIT-1', 'EXIT-1,,EXIT-2', 'EXIT-1,EXIT-1', ' '):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                payloads_argument(bad)


class DetectTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('OpenCV/NumPy not installed: QR detection NOT_RUN')

    def test_reads_every_code_with_four_corners_from_one_image(self):
        import cv2
        import numpy as np
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        for text, left in (('EXIT-1', 40), ('EXIT-2', 360)):
            code = cv2.resize(cv2.QRCodeEncoder.create().encode(text), None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
            h, w = code.shape
            image[100:100 + h, left:left + w] = cv2.cvtColor(code, cv2.COLOR_GRAY2BGR)
        found = dict(detect_qrs(image))
        self.assertEqual({'EXIT-1', 'EXIT-2'}, set(found))
        for text, left in (('EXIT-1', 40), ('EXIT-2', 360)):
            us = [u for u, _ in found[text]]
            self.assertEqual(4, len(found[text]))
            self.assertTrue(left - 10 <= min(us) and max(us) <= left + w + 10)

    def test_no_qr_is_empty(self):
        import numpy as np
        self.assertEqual((), detect_qrs(np.full((480, 640, 3), 255, dtype=np.uint8)))


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
FIELD = dict(exit_qr_payloads=['EXIT-1', 'EXIT-2'], exit_qr_required_frames=5, exit_qr_position_tolerance_m=0.2,
             exit_qr_min_valid_depth_ratio=0.5, exit_qr_every_n_frames=6,
             exit_map_state_topic='/rescue/robots/robot_a/map_state')


class ValuesTests(unittest.TestCase):
    def test_every_value_is_required(self):
        self.assertEqual(set(FIELD), set(EXIT_PARAMS))
        self.assertEqual('STRING_ARRAY', EXIT_PARAMS['exit_qr_payloads'])   # the single-exit name is gone
        for name in FIELD:
            with self.subTest(name=name), self.assertRaises(ValueError):
                exit_values({**FIELD, name: None})
        with self.assertRaises(ValueError):
            exit_values({**FIELD, 'exit_map_state_topic': ' '})

    def test_policy_and_map_state_topic(self):
        policy, topic = exit_values(FIELD)
        self.assertEqual((policy.payloads, policy.required_frames, policy.every_n_frames), (('EXIT-1', 'EXIT-2'), 5, 6))
        self.assertEqual('/rescue/robots/robot_a/map_state', topic)


class ObservationTests(unittest.TestCase):
    def test_exit_observation_fields(self):
        corners = ((300.0, 220.0), (340.0, 221.0), (341.0, 260.0), (299.0, 259.0))
        m = fill_exit_observation(message(), observation_id='00000000-0000-4000-8000-000000000001', binding=BINDING,
                                  frame_id='oakd_rgb_camera_optical_frame', observed_at_ns=1_000, validity_ns=500,
                                  payload='EXIT-2', corners=corners, point=(0.1, -0.2, 2.0), covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                                  model_id='opencv.QRCodeDetector/4.6.0', model_sha256='a' * 64,
                                  evidence_id='frame-1', evidence_sha256='b' * 64)
        # which exit (OPS-157): its content as the track id, the same name over frames and map revisions
        self.assertEqual(('exit_qr', 1.0, 'TRACKING', True, 'EXIT-2'),
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
                                  frame_id='f', observed_at_ns=1_000, validity_ns=500, payload='EXIT-1',
                                  corners=((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)), point=(0.0, 0.0, 1.0),
                                  covariance=[0.0] * 9, model_id='m', model_sha256='a' * 64, evidence_id='e',
                                  evidence_sha256='b' * 64)
        self.assertEqual(('UNKNOWN', False, 0), (m.time.clock_state, m.time.has_clock_error_bound,
                                                 m.time.clock_error_bound_ns))


class SenderTests(unittest.TestCase):
    def test_once_per_exit_and_map_revision(self):
        s = ExitSender()
        self.assertEqual([True, False], [s.send('EXIT-1', ('s1', 3)), s.send('EXIT-1', ('s1', 3))])
        self.assertEqual([True, False], [s.send('EXIT-2', ('s1', 3)), s.send('EXIT-2', ('s1', 3))])   # other exit
        self.assertEqual([True, True], [s.send('EXIT-1', ('s1', 4)), s.send('EXIT-2', ('s1', 4))])   # new revision

    def test_sent_without_a_map_is_sent_again_once_the_map_is_known(self):
        s = ExitSender()   # target_locator refuses it without a VALID MapState
        self.assertEqual([True, False, True], [s.send('EXIT-1', None), s.send('EXIT-1', None),
                                               s.send('EXIT-1', ('s1', 0))])


if __name__ == '__main__':
    unittest.main()
