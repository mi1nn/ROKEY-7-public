"""E2 tests for the robot camera person pipeline on SYNTHETIC frames and recorded detections.

No model, weights, bag, ROS or robot is run (Q-18 open; 2026-10-08 no recording).
All numbers are fixtures, never Q-21 values.
"""
from dataclasses import replace
import struct
from types import SimpleNamespace as NS
import unittest
from uuid import UUID, uuid5

from rescue_contracts import canonical_code

from rescue_perception import (AlignmentEvidence, DepthPolicy, ModelManifest, ObservationPolicy,
                               ObservationRegistry, PerceptionError, SourceBinding, TimePolicy)
from rescue_perception.person_pipeline import (NAMESPACE, ConfirmationPolicy, DetectorOutput, Inference,
                                               PersonPipeline, RecordedDetections)

T0_SEC, T0_NSEC = 1700000000, 123456789
T0 = 1700000000123456789
FRAME = "robot_a/rgb_optical"


def uid(n):
    return str(UUID(int=n))


def header(k):
    return NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id=FRAME)


def rgb(k):
    return NS(header=header(k), width=4, height=4, encoding="rgb8", is_bigendian=0, step=12, data=bytes(48))


def depth(k, mm=2000):
    data = b"".join(struct.pack("<4H", *[mm] * 4) for _ in range(4))
    return NS(header=header(k), width=4, height=4, encoding="16UC1", is_bigendian=0, step=8, data=data)


INFO = NS(header=header(0), width=4, height=4, k=(2.0, 0, 1.5, 0, 2.0, 1.5, 0, 0, 1),
          p=(2.0, 0, 1.5, 0, 0, 2.0, 1.5, 0, 0, 0, 1, 0), d=(0.0,) * 5, distortion_model="plumb_bob",
          binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0))
MODEL = ModelManifest("test-model", "a" * 64, ("person", "dummy"))


def out(confidence=0.9, track=None, label="person"):
    return DetectorOutput(label, confidence, (0, 0, 4, 4), track)


class PersonPipelineTests(unittest.TestCase):
    def setUp(self):
        self.binding = SourceBinding("robot_a", "front", uid(100), "test-clock", "CAPTURE", "CAPTURE",
                                     "test-cal", "SYNCED", 10000000)
        self.kwargs = dict(binding=self.binding,
                           alignment=AlignmentEvidence("test-alignment", "test-cal", FRAME, FRAME, True),
                           rectified=True, depth_policy=DepthPolicy(20000000, 0.5, 5.0, 2, 0.5, 0.5),
                           observation_policy=ObservationPolicy(2000000000),
                           confirmation_policy=ConfirmationPolicy(("person",), 0.5))

    def pipeline(self, by_stamp, mode="TRACKING", **kw):
        args = dict(self.kwargs, detector=RecordedDetections(MODEL, mode, by_stamp))
        args.update(kw)
        return PersonPipeline(**args)

    def frame(self, p, k, mm=2000):
        return p.process(rgb(k), depth(k, mm), INFO)

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))

    def test_confident_located_person_is_confirmed_observation(self):
        result = self.frame(self.pipeline({T0: [out(track="7")]}), 0)
        self.assertEqual(result.status, "PROCESSED")
        (candidate,) = result.candidates
        self.assertEqual((candidate.status, candidate.reasons, candidate.new_candidate), ("CONFIRMED", (), True))
        obs = candidate.observation
        self.assertEqual(obs.observed_at_ns, 1700000000123456789)
        self.assertEqual(obs.localization.point_optical, (0.0, 0.0, 2.0))
        self.assertEqual((obs.detection.track_id, obs.detection.tracking_mode), ("7", "TRACKING"))
        self.assertEqual(obs.observation_id,
                         str(uuid5(NAMESPACE, "00000000-0000-0000-0000-000000000064/1700000000123456789/0")))

    def test_low_confidence_or_no_position_stays_unconfirmed_but_is_kept(self):
        low = self.frame(self.pipeline({T0: [out(0.4, track="1")]}), 0).candidates[0]
        self.assertEqual((low.status, low.reasons), ("UNCONFIRMED", ("LOW_CONFIDENCE",)))
        self.assertIsNotNone(low.observation.localization.point_optical)
        holes = self.frame(self.pipeline({T0: [out(track="2")]}), 0, mm=0).candidates[0]
        self.assertEqual((holes.status, holes.reasons), ("UNCONFIRMED", ("DEPTH_INSUFFICIENT",)))
        self.assertIsNone(holes.observation.localization.point_optical)
        both = self.frame(self.pipeline({T0: [out(0.4, track="3")]}), 0, mm=0).candidates[0]
        self.assertEqual(both.reasons, ("LOW_CONFIDENCE", "DEPTH_INSUFFICIENT"))

    def test_threshold_is_inclusive(self):
        self.assertEqual(self.frame(self.pipeline({T0: [out(0.5, track="1")]}), 0).candidates[0].status,
                         "CONFIRMED")

    def test_same_track_is_a_new_candidate_once_and_again_only_on_upgrade(self):
        p = self.pipeline({T0: [out(0.4, track="5")], T0 + 1: [out(0.4, track="5")],
                           T0 + 2: [out(0.9, track="5")], T0 + 3: [out(0.9, track="5")],
                           T0 + 4: [out(0.4, track="5")]})
        flags = [self.frame(p, k).candidates[0].new_candidate for k in range(5)]
        self.assertEqual(flags, [True, False, True, False, False])

    def test_untracked_detections_are_each_new(self):
        p = self.pipeline({T0: [out()], T0 + 1: [out()]}, mode="DETECTION_ONLY")
        self.assertEqual([self.frame(p, k).candidates[0].new_candidate for k in range(2)], [True, True])

    def test_model_not_run_is_not_an_empty_result(self):
        p = self.pipeline({T0 + 1: []})
        self.assertEqual(self.frame(p, 0), self.frame(p, 0))  # not run twice: still NOT_INFERRED
        self.assertEqual(self.frame(p, 0).status, "NOT_INFERRED")
        empty = self.frame(p, 1)
        self.assertEqual((empty.status, empty.candidates), ("PROCESSED", ()))

    def test_non_person_labels_skipped_and_unknown_labels_rejected(self):
        self.assertEqual(self.frame(self.pipeline({T0: [out(label="dummy")]}, mode="DETECTION_ONLY"), 0).candidates, ())
        p = self.pipeline({T0: [out(label="doll")]}, mode="DETECTION_ONLY",
                          confirmation_policy=ConfirmationPolicy(("person",), 0.5))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.frame(p, 0))

    def test_person_label_missing_from_manifest_is_not_an_empty_frame(self):
        # A policy label the model cannot output would turn every frame into
        # "ran, no person" and leak into recheck ABSENT evidence.
        p = self.pipeline({T0: [out(label="dummy")]}, mode="DETECTION_ONLY",
                          confirmation_policy=ConfirmationPolicy(("human",), 0.5))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.frame(p, 0))
        empty = self.pipeline({T0: []}, confirmation_policy=ConfirmationPolicy(("person", "doll"), 0.5))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.frame(empty, 0))

    def test_output_label_outside_manifest_is_rejected_even_if_not_a_person(self):
        p = self.pipeline({T0: [out(label="cat")]}, mode="DETECTION_ONLY")
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.frame(p, 0))

    def test_duplicate_and_reordered_frames_are_ignored(self):
        p = self.pipeline({T0: [out(track="1")], T0 + 1: [out(track="1")]})
        self.frame(p, 1)
        self.assertEqual(self.frame(p, 1).status, "DUPLICATE")
        self.assertEqual(self.frame(p, 0).status, "REORDERED")

    def test_every_frame_feeds_registry_freshness(self):
        p = self.pipeline({T0: [out(track="1")], T0 + 1000: [out(track="1")]})
        registry, policy = ObservationRegistry(), TimePolicy(2000000000, 20000000, 40000000, 10000000)
        statuses = []
        for k, steady in ((0, 1.0), (1000, 2.0)):
            obs = self.frame(p, k).candidates[0].observation
            statuses.append(registry.ingest(obs, now_ns=T0 + 500000000, received_steady=steady,
                                            receiver_clock_domain="test-clock",
                                            receiver_clock_error_bound_ns=10000000, policy=policy).status)
        self.assertEqual(statuses, ["ACCEPTED", "ACCEPTED"])
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 2.0)

    def test_rejected_frame_leaves_no_state(self):
        # Same class as #30 CDX-30-01: a later bad output must not consume the frame
        # stamp or mark an earlier track as already reported.
        bad = DetectorOutput("person", 0.9, (0, 0, 5, 4), "2")  # x1 beyond the 4-px image
        p = self.pipeline({T0: [out(track="1"), bad]})
        self.assertCode("INVALID_INPUT", lambda: self.frame(p, 0))
        p.detector = RecordedDetections(MODEL, "TRACKING", {T0: [out(track="1")]})
        retry = self.frame(p, 0)
        self.assertEqual(retry.status, "PROCESSED")
        self.assertTrue(retry.candidates[0].new_candidate)

    def test_rejected_label_then_clean_retry(self):
        p = self.pipeline({T0: [out(track="1"), out(track="3", label="cat")]})
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.frame(p, 0))
        p.detector = RecordedDetections(MODEL, "TRACKING", {T0: [out(track="1")]})
        self.assertEqual([c.new_candidate for c in self.frame(p, 0).candidates], [True])

    def test_invalid_inputs_are_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: self.pipeline({}, binding=replace(self.binding, source_robot=None)))
        bad_mode = self.pipeline({T0: [out(track="9")]}, mode="DETECTION_ONLY")
        self.assertCode("INVALID_INPUT", lambda: self.frame(bad_mode, 0))
        wrong = self.pipeline({}, detector=NS(infer=lambda rgb, stamp: [out()]))
        self.assertCode("INVALID_INPUT", lambda: self.frame(wrong, 0))
        self.assertCode("INVALID_INPUT", lambda: RecordedDetections(MODEL, "TRACKING", {1.5: []}))
        self.assertCode("INVALID_INPUT", lambda: Inference(MODEL, "AUTO", ()))
        for bad in (dict(person_labels=()), dict(confirm_confidence_min=1.5), dict(confirm_confidence_min=True)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(ConfirmationPolicy(("person",), 0.5), **bad))


if __name__ == "__main__":
    unittest.main()
