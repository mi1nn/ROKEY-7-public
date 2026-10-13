"""E2 synthetic records: no model, ROS, TF, or physical robot is run."""
from dataclasses import FrozenInstanceError, replace
import math
import unittest
from uuid import UUID
from rescue_contracts import RescueError, canonical_code, ns_to_ros_time, ros_time_to_ns

from rescue_perception import (
    AlignmentEvidence, CameraIntrinsics, ConservativeEventRegistry,
    DepthFrame, DepthPolicy, Detection, FrameMeta, ModelManifest,
    ObservationRegistry, PerceptionError, Publication, TimePolicy, locate_detection,
    make_observation, validate_freshness,
)


def uid(n):
    return str(UUID(int=n))


class BoundaryContractRegressionTests(unittest.TestCase):
    def test_float_epoch_capture_time_must_be_rejected_before_precision_is_lost(self):
        with self.assertRaises(RescueError):
            FrameMeta("rgb", 1700000000.1234567, "clock", "CAPTURE", 1, 1, "cal")

    def test_detection_without_explicit_tracking_mode_must_not_hide_fallback(self):
        with self.assertRaises(TypeError):
            Detection(0, "person", 0.9, (0, 0, 1, 1), None)


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        # All numbers are synthetic test fixtures, never hardware defaults.
        self.rgb = FrameMeta("a/rgb_optical", 100000000000, "test-clock", "CAPTURE", 4, 4, "test-cal")
        self.depth = DepthFrame(self.rgb, "16UC1", "mm", [[2000] * 4 for _ in range(4)])
        self.intrinsics = CameraIntrinsics("a/rgb_optical", "test-cal", 4, 4, 2.0, 2.0, 1.5, 1.5, True)
        self.alignment = AlignmentEvidence("test-alignment", "test-cal", "a/rgb_optical", "a/rgb_optical", True)
        self.depth_policy = DepthPolicy(20000000, 0.5, 5.0, 2, 0.5, 0.5)
        self.detection = Detection(0, "person", 0.9, (0, 0, 4, 4), "7", "TRACKING")
        self.model = ModelManifest("test-model", "a" * 64, ("person", "dummy"))
        self.time_policy = TimePolicy(2000000000, 20000000, 40000000, 10000000)

    def locate(self, **kw):
        args = dict(rgb=self.rgb, depth=self.depth, intrinsics=self.intrinsics,
                    alignment=self.alignment, detection=self.detection, policy=self.depth_policy)
        args.update(kw)
        return locate_detection(**args)

    def observation(self, n=1, **kw):
        args = dict(observation_id=uid(n), source_robot="robot_a", source_camera="front",
                    source_boot_id=uid(100), frame=self.rgb, model=self.model,
                    detection=self.detection, localization=self.locate(), valid_until_ns=102000000000,
                    clock_state="SYNCED", clock_error_bound_ns=10000000,
                    evidence_id=f"image-{n}", evidence_sha256="b" * 64)
        args.update(kw)
        return make_observation(**args)

    def fresh_args(self, now_ns=100500000000):
        return dict(now_ns=now_ns, receiver_clock_domain="test-clock",
                    receiver_clock_error_bound_ns=10000000, policy=self.time_policy)

    def ingest(self, registry, observation, steady=10.0, now_ns=100500000000):
        return registry.ingest(observation, received_steady=steady, **self.fresh_args(now_ns))

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))
        self.assertEqual(ctx.exception.detail_code, "rescue_perception." + code)
        self.assertEqual(ctx.exception.component, "rescue_perception")

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
        for meta, reason in [(replace(self.rgb, observed_at_ns=100030000000), "UNSYNCHRONIZED"),
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
            self.assertCode("INVALID_INPUT", lambda: replace(self.rgb, observed_at_ns=value))
        for value in (True, math.nan, math.inf, -1):
            self.assertCode("INVALID_INPUT", lambda: replace(self.time_policy, max_future_ns=value))
        self.assertCode("INVALID_INPUT", lambda: replace(self.depth_policy, min_valid_fraction=1.1))
        self.assertCode("INVALID_INPUT", lambda: replace(self.depth_policy, min_valid_pixels=True))

    def test_model_manifest_and_class_mapping(self):
        for changes in (dict(model_id=""), dict(sha256="not-sha"), dict(class_labels=("person", "person"))):
            self.assertCode("INVALID_INPUT", lambda: replace(self.model, **changes))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.observation(detection=replace(self.detection, class_id=1)))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: self.observation(detection=replace(self.detection, class_id=4)))
        self.assertIsNone(self.observation(detection=replace(self.detection, track_id=None, tracking_mode="DETECTION_ONLY")).detection.track_id)

    def test_observation_ids_hashes_validity_and_freezing(self):
        for changes in (dict(observation_id="bad"), dict(source_boot_id=uid(0)), dict(evidence_sha256="x"),
                        dict(valid_until_ns=100000000000), dict(valid_until_ns=True), dict(valid_until_ns=math.inf),
                        dict(clock_error_bound_ns=True)):
            self.assertCode("INVALID_INPUT", lambda: self.observation(**changes))
        observation = self.observation()
        with self.assertRaises(FrozenInstanceError):
            observation.valid_until_ns = 1000
        self.assertEqual(observation.observed_at_ns, 100000000000)
        self.assertEqual(len(observation.payload_hash), 64)

    def test_unknown_clock_and_stamp_can_be_recorded_but_not_used(self):
        for observation in (self.observation(clock_state="UNKNOWN"), self.observation(clock_error_bound_ns=None),
                            self.observation(frame=replace(self.rgb, stamp_semantics="UNKNOWN"))):
            self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(observation, **self.fresh_args()))
        self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(self.observation(), **{
            **self.fresh_args(), "receiver_clock_domain": "other"}))
        self.assertCode("CLOCK_UNCERTAIN", lambda: validate_freshness(self.observation(clock_error_bound_ns=100000000), **self.fresh_args()))

    def test_future_and_original_expiry_consider_uncertainty(self):
        self.assertCode("FUTURE_OBSERVATION", lambda: validate_freshness(self.observation(), **self.fresh_args(99900000000)))
        self.assertCode("STALE_OBSERVATION", lambda: validate_freshness(self.observation(), **self.fresh_args(101980000000)))
        self.assertCode("STALE_OBSERVATION", lambda: validate_freshness(self.observation(valid_until_ns=110000000000), **self.fresh_args(102100000000)))
        for now_ns in (True, math.nan, 0):
            self.assertCode("INVALID_INPUT", lambda: validate_freshness(self.observation(), **self.fresh_args(now_ns)))

    def test_republication_changes_only_envelope_not_source_hash_or_timer(self):
        registry = ObservationRegistry()
        observation = self.observation()
        one = Publication(observation, "perception", uid(200), 1, 100500000000, "1.0")
        two = replace(one, publisher_id="relay", publisher_boot_id=uid(201), sequence=7, published_at_ns=101000000000)
        self.assertEqual(one.observation.payload_hash, two.observation.payload_hash)
        self.assertEqual(observation.payload_hash,
                         replace(observation, frame=replace(self.rgb, observed_at_ns=100000000000)).payload_hash)
        self.assertEqual(self.ingest(registry, one.observation).status, "ACCEPTED")
        self.assertEqual(self.ingest(registry, two.observation, 11, 101000000000).status, "DUPLICATE")
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.assertCode("STALE_OBSERVATION", lambda: self.ingest(registry, two.observation, 12, 102000000000))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)

    def test_collision_cannot_extend_lifetime_or_replace_evidence(self):
        registry = ObservationRegistry()
        observation = self.observation()
        self.ingest(registry, observation)
        for changed in (replace(observation, valid_until_ns=110000000000), replace(observation, evidence_sha256="c" * 64),
                        replace(observation, frame=replace(self.rgb, observed_at_ns=100100000000))):
            self.assertCode("ID_CONTENT_CONFLICT", lambda: self.ingest(registry, changed, 11))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)

    def test_reordered_and_same_frame_do_not_refresh_gap_timer(self):
        registry = ObservationRegistry()
        first = self.ingest(registry, self.observation())
        other_person = self.ingest(registry, self.observation(2, detection=replace(self.detection, track_id="8")), 11)
        older = self.ingest(registry, self.observation(3, frame=replace(self.rgb, observed_at_ns=99900000000)), 12)
        self.assertEqual((first.status, other_person.status, older.status), ("ACCEPTED", "ACCEPTED", "REORDERED"))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.ingest(registry, self.observation(4, frame=replace(self.rgb, observed_at_ns=100200000000)), 13)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 13)

    def test_source_boots_keep_separate_receive_history(self):
        registry = ObservationRegistry()
        self.ingest(registry, self.observation())
        self.ingest(registry, self.observation(3, source_boot_id=uid(101)), 11)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(101)), 11)

    def test_receiver_time_regression_latches_until_new_receiver_session(self):
        for steady, wall in ((9, 100600000000), (11, 100400000000)):
            with self.subTest(steady=steady, wall=wall):
                registry = ObservationRegistry()
                self.ingest(registry, self.observation())
                self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(
                    registry, self.observation(2), steady, wall))
                # Neither a later timestamp nor a source process restart proves
                # that this receiver's wall/monotonic clock basis recovered.
                for boot in (uid(100), uid(101)):
                    fresh = self.observation(3, frame=replace(self.rgb, observed_at_ns=100600000000),
                                             source_boot_id=boot)
                    self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(registry, fresh, 12, 100700000000))
                self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
                self.assertIsNone(registry.last_new_receive("robot_a", "front", uid(101)))
                # A caller must recover/revalidate clocks before creating this
                # new session; construction itself is not clock verification.
                recovered = ObservationRegistry()
                self.assertEqual("ACCEPTED", self.ingest(recovered, fresh, 12, 100700000000).status)

    def test_stale_input_still_records_receiver_clock_and_detects_rollback(self):
        registry = ObservationRegistry()
        self.ingest(registry, self.observation())
        self.assertCode("STALE_OBSERVATION", lambda: self.ingest(registry, self.observation(), 11, 200000000000))
        self.assertEqual(registry.last_new_receive("robot_a", "front", uid(100)), 10)
        self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(registry, self.observation(), 12, 100600000000))
        fresh = self.observation(2, frame=replace(self.rgb, observed_at_ns=200500000000), valid_until_ns=202000000000)
        self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(registry, fresh, 13, 201000000000))

    def test_rejected_collision_still_records_receiver_clock(self):
        registry = ObservationRegistry()
        self.ingest(registry, self.observation())
        collision = self.observation(evidence_sha256="c" * 64)
        self.assertCode("ID_CONTENT_CONFLICT", lambda: self.ingest(registry, collision, 11, 101000000000))
        fresh = self.observation(2, frame=replace(self.rgb, observed_at_ns=100700000000))
        self.assertCode("CLOCK_UNCERTAIN", lambda: self.ingest(registry, fresh, 12, 100800000000))

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
        reordered = self.ingest(registry, self.observation(2, frame=replace(self.rgb, observed_at_ns=99900000000)), 13)
        self.assertCode("STALE_OBSERVATION", lambda: events.register(reordered, event_id=uid(1002)))
        fresh = self.ingest(registry, self.observation(3, frame=replace(self.rgb, observed_at_ns=100100000000)), 14)
        self.assertCode("ID_CONTENT_CONFLICT", lambda: events.register(fresh, event_id=uid(1000)))
        self.assertCode("INVALID_INPUT", lambda: events.register(fresh, event_id=uid(1003), candidate_event_ids=(uid(999),)))

    def test_publication_schema_and_sequence_validation(self):
        observation = self.observation()
        for sequence in (True, -1, 2**64):
            self.assertCode("INVALID_INPUT", lambda: Publication(observation, "sender", uid(200), sequence, 100500000000, "1.0"))
        self.assertCode("INCOMPATIBLE_SCHEMA", lambda: Publication(observation, "sender", uid(200), 1, 100500000000, "2.0"))

    def test_real_epoch_ros_stamp_preserved_exactly_through_observation_and_tf_input(self):
        stamp = 1700000000123456789
        self.assertEqual(stamp, ros_time_to_ns(1700000000, 123456789))
        self.assertEqual((1700000000, 123456789), ns_to_ros_time(stamp))
        rgb = replace(self.rgb, observed_at_ns=stamp)
        point = self.locate(rgb=rgb, depth=replace(self.depth, meta=rgb))
        obs = self.observation(frame=rgb, localization=point, valid_until_ns=stamp + 2000000000)
        publication = Publication(obs, "perception", uid(200), 1, stamp + 1, "1.0")
        self.assertEqual(stamp, publication.observation.observed_at_ns)
        # TF query input is the original integer, not a rounded float-seconds copy.
        tf_query_ns = ros_time_to_ns(*ns_to_ros_time(publication.observation.observed_at_ns))
        self.assertEqual(obs.observed_at_ns, tf_query_ns)
        self.assertNotEqual(stamp, int((stamp / 1000000000) * 1000000000))

    def test_timestamp_and_duration_boundaries_reject_float_bool_and_overflow(self):
        for value in (100.0, True, 0, -1, 2**63):
            self.assertCode("INVALID_INPUT", lambda: replace(self.rgb, observed_at_ns=value))
            self.assertCode("INVALID_INPUT", lambda: self.observation(valid_until_ns=value))
            self.assertCode("INVALID_INPUT", lambda: Publication(self.observation(), "sender", uid(200), 1, value, "1.0"))
            self.assertCode("INVALID_INPUT", lambda: validate_freshness(self.observation(), **self.fresh_args(value)))
        for value in (1.0, True, -1, 2**63):
            self.assertCode("INVALID_INPUT", lambda: replace(self.time_policy, max_future_ns=value))
            self.assertCode("INVALID_INPUT", lambda: replace(self.depth_policy, max_stamp_skew_ns=value))
            self.assertCode("INVALID_INPUT", lambda: self.observation(clock_error_bound_ns=value))
        self.assertEqual(0, replace(self.time_policy, max_future_ns=0).max_future_ns)

    def test_depth_pair_one_ns_boundary_at_real_epoch(self):
        stamp = 1700000000123456789
        rgb = replace(self.rgb, observed_at_ns=stamp)
        policy = replace(self.depth_policy, max_stamp_skew_ns=1)
        for delta, expected in ((1, "LOCATED"), (2, "UNSYNCHRONIZED")):
            depth = replace(self.depth, meta=replace(rgb, observed_at_ns=stamp + delta))
            self.assertEqual(expected, self.locate(rgb=rgb, depth=depth, policy=policy).reason)

    def test_tracking_mode_required_and_consistent_with_track_id(self):
        with self.assertRaises(TypeError):
            Detection(0, "person", 0.9, (0, 0, 4, 4), None)
        for mode in ("DETECTION_ONLY", "TRACKING_FALLBACK"):
            value = replace(self.detection, tracking_mode=mode, track_id=None)
            self.assertIsNone(value.track_id)
            self.assertEqual(mode, value.tracking_mode)
            self.assertCode("INVALID_INPUT", lambda: replace(value, track_id="7"))
        self.assertCode("INVALID_INPUT", lambda: replace(self.detection, track_id=None))
        self.assertCode("INVALID_INPUT", lambda: replace(self.detection, tracking_mode="UNKNOWN"))
        self.assertEqual("0", replace(self.detection, track_id="0").track_id)

    def test_tracking_mode_is_part_of_immutable_source_payload_hash(self):
        only = self.observation(detection=replace(self.detection, tracking_mode="DETECTION_ONLY", track_id=None))
        fallback = replace(only, detection=replace(only.detection, tracking_mode="TRACKING_FALLBACK"))
        self.assertNotEqual(only.payload_hash, fallback.payload_hash)
        registry = ObservationRegistry()
        self.ingest(registry, only)
        self.assertCode("ID_CONTENT_CONFLICT", lambda: self.ingest(registry, fallback, 11))

    def test_common_error_and_localization_detail_keep_distinct_meanings(self):
        try:
            DepthFrame(self.rgb, "mono16", "mm", [[1] * 4] * 4)
        except PerceptionError as exc:
            self.assertIsInstance(exc, RescueError)
            self.assertEqual(canonical_code("rescue_perception", "UNSUPPORTED_DEPTH"), exc.code)
            self.assertEqual("rescue_perception.UNSUPPORTED_DEPTH", exc.detail_code)
            self.assertEqual("rescue_perception", exc.component)
        else:
            self.fail("unsupported encoding must fail")
        bad = self.locate(depth=DepthFrame(self.rgb, "16UC1", "mm", [[0] * 4] * 4))
        self.assertEqual("DEPTH_INSUFFICIENT", bad.reason)
        self.assertEqual(canonical_code("rescue_perception", bad.reason), bad.error_code)
        self.assertEqual("rescue_perception.DEPTH_INSUFFICIENT", bad.detail_code)
        self.assertIsNone(self.locate().error_code)
        self.assertIsNone(self.locate().detail_code)

    def test_reordered_discovery_survives_in_immutable_history_without_event_admission(self):
        registry, events = ObservationRegistry(), ConservativeEventRegistry()
        self.ingest(registry, self.observation())
        late = self.observation(2, frame=replace(self.rgb, observed_at_ns=99900000000))
        admission = self.ingest(registry, late, 11)
        snapshot = registry.history()
        self.assertEqual(("ACCEPTED", "REORDERED"), tuple(item.status for item in snapshot))
        self.assertIs(late, snapshot[-1].observation)
        self.assertCode("STALE_OBSERVATION", lambda: events.register(admission, event_id=uid(1001)))
        self.ingest(registry, late, 12)
        self.assertEqual(snapshot, registry.history())
        with self.assertRaises(FrozenInstanceError):
            snapshot[-1].observation.valid_until_ns = 999
        self.assertEqual(10, registry.last_new_receive("robot_a", "front", uid(100)))

    def test_robot_identity_uses_shared_profile_format(self):
        self.assertEqual("custom_amr_3", self.observation(source_robot="custom_amr_3").source_robot)
        for value in ("/robot_a", "Robot A", "robot-a", "", "robot_a\x00"):
            self.assertCode("INVALID_INPUT", lambda: self.observation(source_robot=value))


if __name__ == "__main__":
    unittest.main()
