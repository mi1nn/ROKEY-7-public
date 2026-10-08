"""E2 tests for fixed camera B exit passes on SYNTHETIC detection sequences.

No camera, bag, model, ROS or storage is run (2026-10-08: no recording exists).
Geometry: 40x40 image, exit line y=20, exit side is below the line (larger y).
All numbers are fixtures, never deployment values.
"""
from dataclasses import replace
import hashlib
from types import SimpleNamespace as NS
import unittest
from uuid import UUID, uuid5

from rescue_contracts import SiteEvent, canonical_code

from rescue_perception import Detection, PerceptionError, SourceBinding
from rescue_perception.exit_pass import EVENT_NAMESPACE, ExitLinePolicy, ExitPassDetector

T0_SEC, T0_NSEC = 1700000000, 123456789  # frame k is stamped T0 + k ns (k < 1e9 - T0_NSEC)
PIXELS = bytes([5]) * (40 * 40 * 3)
SHA = hashlib.sha256(PIXELS).hexdigest()


def uid(n):
    return str(UUID(int=n))


def header(k, frame_id="cam_b/rgb_optical"):
    return NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id=frame_id)


def rgb(k):
    return NS(header=header(k), width=40, height=40, encoding="rgb8", is_bigendian=0, step=120, data=PIXELS)


def person(bottom, x=10.0, track=None, mode=None, confidence=0.9, label="person"):
    mode = mode or ("TRACKING" if track is not None else "DETECTION_ONLY")
    return Detection(0 if label == "person" else 1, label, confidence, (x - 2, bottom - 8, x + 2, bottom),
                     track, mode)


class ExitPassTests(unittest.TestCase):
    def setUp(self):
        self.policy = ExitLinePolicy(((0.0, 20.0), (40.0, 20.0)), 1, 2.0, ("person",), 0.5, 12.0, 300)
        self.binding = SourceBinding(None, "cam_b", uid(200), "site-clock", "CAPTURE", "CAPTURE",
                                     "cam-b-cal", "SYNCED", 5000000)
        self.detector = ExitPassDetector(self.binding, self.policy)

    def new(self, **policy):
        return ExitPassDetector(self.binding, replace(self.policy, **policy))

    def update(self, k, detections, detector=None):
        return (detector or self.detector).update(rgb(k), detections, header(k))

    def feed(self, sequence, detector=None):
        """sequence: list of (k, [detections]); returns all events in order."""
        events = []
        for k, detections in sequence:
            events += self.update(k, detections, detector).events
        return events

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))

    def test_one_tracked_pass_is_one_event_with_exact_frame_stamp(self):
        events = self.feed([(0, [person(10, track="7")]), (100, [person(15, track="7")]),
                            (200, [person(25, track="7")]), (300, [person(30, track="7")])])
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertIsInstance(event, SiteEvent)
        self.assertEqual((event.kind, event.camera_id), ("EXIT_PASS", "cam_b"))
        self.assertEqual(event.observed_at_ns, 1700000000123456989)
        self.assertEqual(event.evidence_id, str(uuid5(EVENT_NAMESPACE, "cam_b/1700000000123456989/frame")))
        self.assertEqual(event.evidence_sha256, SHA)
        self.assertEqual(event.confidence, 0.9)
        self.assertEqual(event.detail, "method=TRACK track_id=7")
        self.assertEqual((event.clock_domain_id, event.stamp_semantics, event.clock_state,
                          event.clock_error_bound_ns), ("site-clock", "CAPTURE", "SYNCED", 5000000))
        self.assertTrue(event.time_basis_declared)

    def test_unknown_time_basis_is_recorded_not_rejected(self):
        unknown = replace(self.binding, rgb_stamp_semantics="UNKNOWN", clock_state="UNKNOWN",
                          clock_error_bound_ns=None)
        events = self.feed([(0, [person(15, track="7")]), (1, [person(25, track="7")])],
                           ExitPassDetector(unknown, self.policy))
        self.assertEqual(len(events), 1)
        self.assertEqual((events[0].stamp_semantics, events[0].clock_state), ("UNKNOWN", "UNKNOWN"))
        self.assertFalse(events[0].time_basis_declared)

    def test_detection_from_another_frame_is_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: self.detector.update(rgb(5), [], header(6)))
        self.assertCode("INVALID_INPUT", lambda: self.detector.update(rgb(5), [], header(5, "other")))

    def test_round_trip_standing_and_jitter_add_nothing(self):
        round_trip = [(0, [person(15, track="1")]), (1, [person(25, track="1")]),
                      (2, [person(15, track="1")]), (3, [person(25, track="1")])]
        self.assertEqual(len(self.feed(round_trip)), 1)
        standing = [(10, [person(19, track="2")]), (11, [person(21, track="2")]),
                    (12, [person(19.5, track="2")]), (13, [person(21.9, track="2")])]
        self.assertEqual(self.feed(standing), [])
        through_band = [(20, [person(17, track="3")]), (21, [person(21, track="3")]),
                        (22, [person(19, track="3")]), (23, [person(23, track="3")])]
        self.assertEqual(len(self.feed(through_band)), 1)

    def test_first_seen_on_exit_side_is_not_a_pass(self):
        self.assertEqual(self.feed([(0, [person(30, track="9")]), (1, [person(35, track="9")])]), [])

    def test_exit_side_follows_policy_sign(self):
        upward = self.new(exit_side=-1)
        self.assertEqual(self.feed([(0, [person(25, track="4")]), (1, [person(15, track="4")])], upward)[0].kind,
                         "EXIT_PASS")
        self.assertEqual(self.feed([(0, [person(15, track="4")]), (1, [person(25, track="4")])],
                                   self.new(exit_side=-1)), [])

    def test_duplicate_and_reordered_frames_change_nothing(self):
        self.assertEqual(self.update(10, [person(15, track="5")]).status, "PROCESSED")
        again = self.update(10, [person(25, track="5")])
        self.assertEqual((again.status, again.events), ("DUPLICATE", ()))
        older = self.update(9, [person(25, track="5")])
        self.assertEqual((older.status, older.events), ("REORDERED", ()))
        # The ignored frames did not move the subject: the real crossing still counts once.
        self.assertEqual(len(self.update(11, [person(25, track="5")]).events), 1)

    def test_untracked_hysteresis_survives_flicker_within_gap(self):
        events = self.feed([(0, [person(15)]), (100, []), (200, [person(18)]), (300, [person(25)]),
                            (400, []), (500, [person(28)]), (600, [person(18)]), (700, [person(25)])])
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].observed_at_ns, 1700000000123457089)
        self.assertTrue(events[0].detail.startswith("method=HYSTERESIS mode=DETECTION_ONLY subject=0"))

    def test_untracked_after_long_gap_reappearing_outside_is_not_counted(self):
        self.assertEqual(self.feed([(0, [person(15)]), (1000, [person(25)])]), [])

    def test_two_untracked_people_in_one_frame_are_separate(self):
        events = self.feed([(0, [person(15, x=5), person(15, x=30)]), (1, [person(25, x=5), person(25, x=30)])])
        self.assertEqual(len(events), 2)
        self.assertNotEqual(events[0].event_id, events[1].event_id)

    def test_close_untracked_people_never_share_one_subject_in_a_frame(self):
        events = self.feed([(0, [person(15, x=10), person(15, x=16)]), (1, [person(25, x=10), person(25, x=16)])])
        self.assertEqual(len(events), 2)

    def test_fallback_mode_uses_hysteresis_and_is_reported(self):
        events = self.feed([(0, [person(15, mode="TRACKING_FALLBACK")]),
                            (1, [person(25, mode="TRACKING_FALLBACK")])])
        self.assertIn("mode=TRACKING_FALLBACK", events[0].detail)

    def test_low_confidence_and_other_labels_are_ignored(self):
        self.assertEqual(self.feed([(0, [person(15, track="6", confidence=0.4)]),
                                    (1, [person(25, track="6", confidence=0.4)])]), [])
        self.assertEqual(self.feed([(2, [person(15, track="8", label="dummy")]),
                                    (3, [person(25, track="8", label="dummy")])]), [])

    def test_replay_yields_identical_event_ids(self):
        sequence = [(0, [person(15, track="7")]), (1, [person(25, track="7")])]
        first = self.feed(sequence)
        second = self.feed(sequence, ExitPassDetector(self.binding, self.policy))
        self.assertEqual(first, second)

    def test_invalid_inputs_are_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: ExitPassDetector(replace(self.binding, source_robot="robot_a"),
                                                                  self.policy))
        self.assertCode("INVALID_INPUT", lambda: ExitPassDetector(replace(self.binding, source_camera="front"),
                                                                  self.policy))
        self.assertCode("INVALID_INPUT", lambda: self.update(0, [person(15, track="1"), person(25, track="1", x=30)]))
        self.assertCode("INVALID_INPUT", lambda: self.update(0, [person(45, track="1")]))
        for bad in (dict(line=((1.0, 1.0), (1.0, 1.0))), dict(exit_side=0), dict(exit_side=True),
                    dict(band_px=0.0), dict(max_gap_ns=0), dict(max_gap_ns=1.0), dict(min_confidence=1.5),
                    dict(count_labels=())):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.policy, **bad))


if __name__ == "__main__":
    unittest.main()
