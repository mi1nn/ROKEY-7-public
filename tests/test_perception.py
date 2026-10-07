"""E2 synthetic records: no model, ROS, TF, or physical robot is run."""
from dataclasses import FrozenInstanceError, replace
import math
import unittest
from uuid import UUID

from rescue_perception import (
    Admission, AlignmentEvidence, CameraIntrinsics, ConservativeEventRegistry,
    DepthFrame, DepthPolicy, Detection, FrameMeta, LocalizationResult, ModelManifest,
    ObservationRegistry, PerceptionError, Publication, TimePolicy, locate_detection,
    make_observation, validate_freshness,
)


def uid(n):
    return str(UUID(int=n))


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        # All numbers are synthetic test fixtures, never hardware defaults.
        self.rgb = FrameMeta("a/rgb_optical", 100.0, "test-clock", "CAPTURE", 4, 4, "test-cal")
        self.depth = DepthFrame(self.rgb, "16UC1", "mm", [[2000] * 4 for _ in range(4)])
        self.intrinsics = CameraIntrinsics("a/rgb_optical", "test-cal", 4, 4, 2.0, 2.0, 1.5, 1.5, True)
        self.alignment = AlignmentEvidence("test-alignment", "test-cal", "a/rgb_optical", "a/rgb_optical", True)
        self.depth_policy = DepthPolicy(0.02, 0.5, 5.0, 2, 0.5, 0.5)
        self.detection = Detection(0, "person", 0.9, (0, 0, 4, 4), "7")
        self.model = ModelManifest("test-model", "a" * 64, ("person", "dummy"))
        self.time_policy = TimePolicy(2.0, 0.02, 0.04, 0.01)

    def locate(self, **kw):
        args = dict(rgb=self.rgb, depth=self.depth, intrinsics=self.intrinsics,
                    alignment=self.alignment, detection=self.detection, policy=self.depth_policy)
        args.update(kw)
        return locate_detection(**args)

    def observation(self, n=1, **kw):
        args = dict(observation_id=uid(n), source_robot="robot_a", source_camera="front",
                    source_boot_id=uid(100), frame=self.rgb, model=self.model,
                    detection=self.detection, localization=self.locate(), valid_until=102.0,
                    clock_state="SYNCED", clock_error_bound=0.01,
                    evidence_id=f"image-{n}", evidence_sha256="b" * 64)
        args.update(kw)
        return make_observation(**args)

    def fresh_args(self, now=100.5):
        return dict(now=now, receiver_clock_domain="test-clock",
                    receiver_clock_error_bound=0.01, policy=self.time_policy)

    def ingest(self, registry, observation, steady=10.0, now=100.5):
        return registry.ingest(observation, received_steady=steady, **self.fresh_args(now))

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, code)

    def test_registered_millimetres_deproject_to_optical_metres(self):
        result = self.locate()
        self.assertEqual(result.point_optical, (0.0, 0.0, 2.0))
        self.assertEqual(result.valid_pixels, 16)
        offcentre = self.locate(detection=replace(self.detection, bbox=(2, 0, 4, 2)))
        self.assertEqual(offcentre.point_optical, (1.0, -1.0, 2.0))

    def test_float_metres_invalid_samples_and_median(self):
        rows = [[math.nan, 0, 1.9, 2.0], [math.inf, -1, 2.1, 2.0], [2.0] * 4, [2.0] * 4]
        result = self.locate(depth=DepthFrame(self.rgb, "32FC1", "m", rows))
        self.assertEqual(result.point_optical[2], 2.0)
        self.assertEqual(result.valid_pixels, 12)

    def test_depth_failure_keeps_discovery_without_position(self):
        result = self.locate(depth=DepthFrame(self.rgb, "16UC1", "mm", [[0] * 4 for _ in range(4)]))
        observation = self.observation(localization=result)
        self.assertIsNone(observation.localization.point_optical)
        self.assertEqual(observation.localization.reason, "DEPTH_INSUFFICIENT")

    def test_background_discontinuity_rejects_median_position(self):
        result = self.locate(depth=DepthFrame(self.rgb, "16UC1", "mm", [[1000, 1000, 4000, 4000]] * 4))
        self.assertEqual(result.reason, "DEPTH_DISCONTINUITY")

    def test_sync_semantics_and_clock_domain_rejected(self):
        for meta, reason in [(replace(self.rgb, observed_at=100.03), "UNSYNCHRONIZED"),
                             (replace(self.rgb, clock_domain_id="other"), "CLOCK_UNCERTAIN"),
                             (replace(self.rgb, stamp_semantics="UNKNOWN"), "CLOCK_UNCERTAIN"),
                             (replace(self.rgb, stamp_semantics="DRIVER_RECEIVE"), "CLOCK_UNCERTAIN")]:
            with self.subTest(meta=meta):
                self.assertEqual(self.locate(depth=replace(self.depth, meta=meta)).reason, reason)

    def test_intrinsics_alignment_and_resolution_are_not_inferred(self):
        cases = [dict(alignment=replace(self.alignment, verified=False)),
                 dict(intrinsics=replace(self.intrinsics, rectified=False)),
                 dict(alignment=replace(self.alignment, calibration_id="other")),
                 dict(intrinsics=replace(self.intrinsics, frame_id="other")),
                 dict(intrinsics=replace(self.intrinsics, width=8))]
        for args in cases:
            with self.subTest(args=args):
                self.assertIsNone(self.locate(**args).point_optical)

    def test_depth_encoding_units_and_shape(self):
        for encoding, unit in [("16UC1", "m"), ("32FC1", "mm"), ("mono16", "mm"), ("RGB8", "m")]:
            self.assertCode("UNSUPPORTED_DEPTH", lambda: DepthFrame(self.rgb, encoding, unit, [[1] * 4] * 4))
        for rows in ([[1] * 4] * 3, [[1] * 3] * 4, [[True] * 4] * 4, [[-1] * 4] * 4, [[1.0] * 4] * 4):
            self.assertCode("INVALID_INPUT", lambda: DepthFrame(self.rgb, "16UC1", "mm", rows))

        self.assertCode("INVALID_INPUT", lambda: DepthFrame(self.rgb, "32FC1", "m", [[10**400] * 4] * 4))

    def test_bbox_no_silent_clip_or_scale(self):
        self.assertCode("INVALID_INPUT", lambda: self.locate(detection=replace(self.detection, bbox=(0, 0, 5, 4))))
        for bbox in [(0, 0, 0, 1), (-1, 0, 1, 1), (0, 0, math.nan, 4), (False, 0, 1, 1)]:
            self.assertCode("INVALID_INPUT", lambda: replace(self.detection, bbox=bbox))

    def test_no_implicit_thresholds_and_invalid_scalar_validation(self):
        with self.assertRaises(TypeError):
            DepthPolicy()
        for value in (True, math.nan, math.inf, -1, 0):
            self.assertCode("INVALID_INPUT", lambda: replace(self.intrinsics, fx=value))
            self.assertCode("INVALID_INPUT", lambda: replace(self.rgb, observed_at=value))
        for value in (True, math.nan, math.inf, -1):
            self.assertCode("INVALID_INPUT", lambda: replace(self.time_policy, max_future_s=value))
        self.assertCode("INVALID_INPUT", lambda: replace(self.depth_policy, min_valid_fraction=1.1))
        self.assertCode("INVALID_INPUT", lambda: replace(self.depth_policy, min_valid_pixels=True))

    def test_model_manifest_and_class_mapping(self):
        for changes in (dict(model_id=""), dict(sha256="not-sha"), dict(class_labels=("person", "person"))):
            self.assertCode("INVALID_INPUT", lambda: replace(self.model, **changes))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.observation(detection=replace(self.detection, class_id=1)))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.observation(detection=replace(self.detection, class_id=4)))
        self.assertIsNone(self.observation(detection=replace(self.detection, track_id=None)).detection.track_id)

    def test_observation_ids_hashes_validity_and_freezing(self):
        for changes in (dict(observation_id="bad"), dict(source_boot_id=uid(0)), dict(evidence_sha256="x"),
                        dict(valid_until=100), dict(valid_until=True), dict(valid_until=math.inf),
                        dict(clock_error_bound=True)):
            self.assertCode("INVALID_INPUT", lambda: self.observation(**changes))
        observation = self.observation()
        with self.assertRaises(FrozenInstanceError):
            observation.valid_until = 1000
        self.assertEqual(observation.observed_at, 100)
        self.assertEqual(len(observation.payload_hash), 64)

    def test_unknown_clock_and_stamp_can_be_recorded_but_not_used(self):
        for observation in (self.observation(clock_state="UNKNOWN"), self.observation(clock_error_bound=None),
                            self.observation(frame=replace(self.rgb, stamp_semantics="UNKNOWN"))):
            self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(observation, **self.fresh_args()))
        self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(self.observation(), **{
            **self.fresh_args(), "receiver_clock_domain": "other"}))
        self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(self.observation(clock_error_bound=0.1), **self.fresh_args()))

    def test_future_and_original_expiry_consider_uncertainty(self):
        self.assertCode("FUTURE_OBSERVATION", lambda: validate_freshness(self.observation(), **self.fresh_args(99.9)))
        self.assertCode("STALE_OBSERVATION", lambda: validate_freshness(self.observation(), **self.fresh_args(101.98)))
        self.assertCode("STALE_OBSERVATION", lambda: validate_freshness(self.observation(valid_until=110), **self.fresh_args(102.1)))
        for now in (True, math.nan, 0):
            self.assertCode("INVALID_INPUT", lambda: validate_freshness(self.observation(), **self.fresh_args(now)))

    def test_republication_changes_only_envelope_not_source_hash_or_timer(self):
        registry = ObservationRegistry()
        observation = self.observation()
        one = Publication(observation, "perception", uid(200), 1, 100.5, "1.0")
        two = replace(one, publisher_id="relay", publisher_boot_id=uid(201), sequence=7, published_at=101.0)
        self.assertEqual(one.observation.payload_hash, two.observation.payload_hash)
        self.assertEqual(observation.payload_hash,
                         replace(observation, frame=replace(self.rgb, observed_at=100)).payload_hash)
        self.assertEqual(self.ingest(registry, one.observation).status, "ACCEPTED")
        self.assertEqual(self.ingest(registry, two.observation, 11, 101).status, "DUPLICATE")
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.assertCode("STALE_OBSERVATION", lambda: self.ingest(registry, two.observation, 12, 102))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)

    def test_collision_cannot_extend_lifetime_or_replace_evidence(self):
        registry = ObservationRegistry()
        observation = self.observation()
        self.ingest(registry, observation)
        for changed in (replace(observation, valid_until=110), replace(observation, evidence_sha256="c" * 64),
                        replace(observation, frame=replace(self.rgb, observed_at=100.1))):
            self.assertCode("ID_CONTENT_CONFLICT", lambda: self.ingest(registry, changed, 11))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)

    def test_reordered_and_same_frame_do_not_refresh_gap_timer(self):
        registry = ObservationRegistry()
        first = self.ingest(registry, self.observation())
        other_person = self.ingest(registry, self.observation(2, detection=replace(self.detection, track_id="8")), 11)
        older = self.ingest(registry, self.observation(3, frame=replace(self.rgb, observed_at=99.9)), 12)
        self.assertEqual((first.status, other_person.status, older.status), ("ACCEPTED", "ACCEPTED", "REORDERED"))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.ingest(registry, self.observation(4, frame=replace(self.rgb, observed_at=100.2)), 13)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 13)

    def test_boot_identity_and_receiver_time_regression(self):
        registry = ObservationRegistry()
        self.ingest(registry, self.observation())
        self.assertCode("INVALID_INPUT", lambda: self.ingest(registry, self.observation(2), 9))
        self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(registry, self.observation(2), 11, 100.4))
        self.ingest(registry, self.observation(3, source_boot_id=uid(101)), 11)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(101)), 11)

    def test_two_robots_same_track_id_remain_separate_candidates(self):
        registry, events = ObservationRegistry(), ConservativeEventRegistry()
        a = events.register(self.ingest(registry, self.observation()), event_id=uid(1000))
        b = events.register(self.ingest(registry, self.observation(2, source_robot="robot_b"), 11),
                            event_id=uid(1001), candidate_event_ids=(a.event_id,))
        self.assertNotEqual(a.event_id, b.event_id)
        self.assertEqual(b.candidate_event_ids, (a.event_id,))
        self.assertEqual((a.status, b.status), ("CANDIDATE", "CANDIDATE"))

    def test_same_frame_people_both_get_events_without_identity_claim(self):
        registry, events = ObservationRegistry(), ConservativeEventRegistry()
        a = events.register(self.ingest(registry, self.observation()), event_id=uid(1000))
        b = events.register(self.ingest(registry, self.observation(2, detection=replace(self.detection, track_id="8")), 11), event_id=uid(1001))
        self.assertNotEqual(a.event_id, b.event_id)

    def test_events_dedup_conflicts_and_no_reordered_identity(self):
        registry, events = ObservationRegistry(), ConservativeEventRegistry()
        observation = self.observation()
        a = events.register(self.ingest(registry, observation), event_id=uid(1000))
        self.assertEqual(events.register(self.ingest(registry, observation, 11), event_id=uid(1000)), a)
        self.assertCode("ID_CONTENT_CONFLICT", lambda: events.register(self.ingest(registry, observation, 12), event_id=uid(1001)))
        reordered = self.ingest(registry, self.observation(2, frame=replace(self.rgb, observed_at=99.9)), 13)
        self.assertCode("STALE_OBSERVATION", lambda: events.register(reordered, event_id=uid(1002)))
        fresh = self.ingest(registry, self.observation(3, frame=replace(self.rgb, observed_at=100.1)), 14)
        self.assertCode("ID_CONTENT_CONFLICT", lambda: events.register(fresh, event_id=uid(1000)))
        self.assertCode("INVALID_INPUT", lambda: events.register(fresh, event_id=uid(1003), candidate_event_ids=(uid(999),)))

    def test_publication_schema_and_sequence_validation(self):
        observation = self.observation()
        for sequence in (True, -1, 2**64):
            self.assertCode("INVALID_INPUT", lambda: Publication(observation, "sender", uid(200), sequence, 100.5, "1.0"))
        self.assertCode("INCOMPATIBLE_SCHEMA", lambda: Publication(observation, "sender", uid(200), 1, 100.5, "2.0"))


if __name__ == "__main__":
    unittest.main()
