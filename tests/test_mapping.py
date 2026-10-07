"""Synthetic E2 coordinate tests. They do not exercise TF, SLAM, ROS or a robot."""

from dataclasses import FrozenInstanceError, replace
import math
import unittest
from uuid import NAMESPACE_URL, uuid5

from rescue_mapping import (FrameAlignment, MappingError, MappingRegistry,
                            RigidTransform, SourceObservation, TimedTransform)


def uid(label):
    return str(uuid5(NAMESPACE_URL, "rescue-mapping-tests:" + label))


def observation(robot="a", oid=None, **changes):
    value = SourceObservation(uid(oid or robot + "-obs"), robot, "oakd", uid("camera-boot-1"),
                              100, "camera_optical", (0, 0, 2), 1000, "test-clock",
                              uid(robot + "-session"), 0)
    return replace(value, **changes)


def transform(robot="a", **changes):
    value = TimedTransform(robot, "camera_optical", "map", uid(robot + "-session"), 0,
                           "test-clock", 100,
                           RigidTransform((1, 2, 3), (-.5, .5, -.5, .5)),
                           "EXACT", (100,), robot + "-tf")
    return replace(value, **changes)


def alignment(robot="a", **changes):
    value = FrameAlignment(robot, "map", "mission", uid(robot + "-session"), 0, 1,
                           "mapping", "test-clock", RigidTransform.identity(),
                           1, 2000, 0.1, 0.05)
    return replace(value, **changes)


class MappingTests(unittest.TestCase):
    def setUp(self):
        # Synthetic limits/time units; these are not proposed field thresholds.
        self.registry = MappingRegistry(clock_domain_id="test-clock", alignment_authority="mapping",
                                        max_position_error_m=.2, max_rotation_error_rad=.1)
        for robot in ("a", "b"):
            self.registry.start_session(robot, "map", uid(robot + "-session"))
            self.registry.set_alignment(alignment(robot))

    def assertPoint(self, expected, actual):
        for left, right in zip(expected, actual):
            self.assertAlmostEqual(left, right, places=10)

    def project(self, robot="a", **changes):
        return self.registry.project(observation(robot, **changes), transform(robot), now_ns=100)

    def goal(self, ids=(uid("a-obs"),), destination="b", point=(3, 2, 3)):
        return self.registry.capture_goal_snapshot(ids, destination, point, now_ns=100)

    def assertCode(self, code, operation):
        with self.assertRaises(MappingError) as caught:
            operation()
        self.assertEqual(code, caught.exception.code)

    def test_optical_depth_point_uses_full_3d_rotation_then_translation(self):
        result = self.project()
        self.assertPoint((3, 2, 3), result.map_point_xyz)
        self.assertPoint((3, 2, 3), result.mission_point_xyz)
        self.assertEqual((0, 0, 2), result.original.point_xyz)
        self.assertEqual(transform(), result.sensor_to_map)
        self.assertEqual(alignment(), result.map_to_mission)

    def test_nonplanar_map_alignment_preserves_vertical_component(self):
        self.registry.set_alignment(alignment(transform_revision=2, transform=RigidTransform(
            (4, 5, 6), (math.sqrt(.5), 0, 0, math.sqrt(.5)))))
        result = self.project()
        self.assertPoint((7, 2, 8), result.mission_point_xyz)

    def test_planar_alignment_and_destination_inverse_share_a_physical_point(self):
        self.registry.set_alignment(alignment("a", transform_revision=2,
                                             transform=RigidTransform.planar(10, 20, math.pi / 2)))
        self.registry.set_alignment(alignment("b", transform_revision=2,
                                             transform=RigidTransform.planar(2, 3, math.pi)))
        result = self.project()
        self.assertPoint((8, 23, 3), result.mission_point_xyz)
        goal = self.goal(point=result.mission_point_xyz)
        self.assertPoint((-6, -20, 3), goal.destination_point_xyz)
        self.assertTrue(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)

    def test_general_rotation_inverse_roundtrip_and_scaled_quaternion(self):
        rigid = RigidTransform((1, -3, 4), (.5, 1, 1.5, 2))
        point = (3, 5, -7)
        self.assertPoint(point, rigid.inverse().apply(rigid.apply(point)))
        self.assertAlmostEqual(1, math.hypot(*rigid.rotation_xyzw))

    def test_extreme_finite_quaternion_scales_preserve_lengths_and_inverse(self):
        for rotation in ((5e-324, 5e-324, 0, 0), (1e308, 1e308, 1e308, 1e308)):
            with self.subTest(rotation=rotation):
                rigid = RigidTransform((0, 0, 0), rotation)
                self.assertAlmostEqual(1, math.hypot(*rigid.rotation_xyzw))
                self.assertAlmostEqual(1, math.hypot(*rigid.apply((1, 0, 0))))
                self.assertPoint((1, 0, 0), rigid.inverse().apply(rigid.apply((1, 0, 0))))

    def test_defensive_copy_and_frozen_observation_preserve_original(self):
        point = [0, 0, 2]
        raw = observation(point_xyz=point)
        point[2] = 99
        result = self.registry.project(raw, transform(), now_ns=100)
        self.assertEqual((0, 0, 2), raw.point_xyz)
        with self.assertRaises(FrozenInstanceError):
            raw.observed_at_ns = 101
        self.assertIs(raw, self.registry.original(raw.observation_id))
        self.assertIs(raw, result.original)

    def test_duplicate_observation_id_must_keep_every_semantic_field(self):
        raw = observation()
        self.registry.retain_observation(raw)
        self.assertIs(raw, self.registry.retain_observation(raw))
        for field, changed in (("point_xyz", (1, 0, 2)), ("source_boot_id", uid("other-boot")),
                               ("valid_until_ns", 1100), ("observed_at_ns", 101),
                               ("map_revision", 1), ("source_camera", "other-camera")):
            with self.subTest(field=field):
                self.assertCode("ID_CONTENT_CONFLICT", lambda: self.registry.retain_observation(
                    replace(raw, **{field: changed})))

    def test_missing_tf_retains_original_but_produces_no_location(self):
        raw = observation()
        self.assertCode("TF_UNAVAILABLE", lambda: self.registry.project(raw, None, now_ns=100))
        self.assertEqual(raw, self.registry.original(raw.observation_id))
        self.assertCode("PROJECTION_UNAVAILABLE", lambda: self.registry.projection(uid("a-obs"), now_ns=100))

    def test_latest_tf_is_rejected_even_if_very_close_to_observation_time(self):
        latest = transform(query_time_ns=101, sample_times_ns=(101,))
        self.assertCode("TF_TIME_MISMATCH", lambda: self.registry.project(observation(), latest, now_ns=102))

    def test_validated_interpolation_requires_bracketing_original_stamp(self):
        tf = transform(mode="INTERPOLATED_VALIDATED", sample_times_ns=(90, 110))
        result = self.registry.project(observation(), tf, now_ns=100)
        self.assertEqual((90, 110), result.tf_sample_times_ns)
        self.assertEqual("INTERPOLATED_VALIDATED", result.tf_mode)
        for samples in ((101, 110), (90,), (110, 90), (100, 100)):
            with self.subTest(samples=samples):
                self.assertCode("INVALID_TF_TIME_EVIDENCE", lambda: transform(
                    mode="INTERPOLATED_VALIDATED", sample_times_ns=samples))
        self.assertCode("INVALID_TF_TIME_EVIDENCE", lambda: transform(mode="LATEST"))

    def test_native_map_and_camera_names_are_isolated_by_robot_identity(self):
        left = self.project("a")
        right = self.registry.project(observation("b"), transform("b", transform=RigidTransform.identity()), now_ns=100)
        self.assertNotEqual(left.mission_point_xyz, right.mission_point_xyz)
        self.assertCode("TF_ROBOT_MISMATCH", lambda: self.registry.project(
            observation("a", oid="new-a"), transform("b"), now_ns=100))

    def test_wrong_sensor_map_clock_or_map_version_cannot_be_substituted(self):
        cases = [("TF_FRAME_MISMATCH", {"source_frame": "other_camera"}),
                 ("TF_FRAME_MISMATCH", {"target_frame": "odom"}),
                 ("CLOCK_DOMAIN_MISMATCH", {"clock_domain_id": "other-clock"}),
                 ("TF_MAP_VERSION_MISMATCH", {"map_revision": 1}),
                 ("TF_MAP_VERSION_MISMATCH", {"map_session_id": uid("other-session")})]
        for code, changes in cases:
            with self.subTest(code=code, changes=changes):
                self.assertCode(code, lambda: self.registry.project(observation(), transform(**changes), now_ns=100))
        self.assertCode("CLOCK_DOMAIN_MISMATCH", lambda: self.registry.retain_observation(
            observation(clock_domain_id="other-clock")))

    def test_zero_future_expired_observation_and_clock_regression_fail_closed(self):
        self.assertCode("INVALID_INPUT", lambda: observation(observed_at_ns=0))
        self.assertCode("FUTURE_OBSERVATION", lambda: self.registry.project(
            observation(), transform(), now_ns=99))
        self.assertCode("OBSERVATION_EXPIRED", lambda: self.registry.project(
            observation(), transform(), now_ns=1000))
        self.assertCode("CLOCK_RESET_REQUIRED", lambda: self.registry.project(
            observation(), transform(), now_ns=100))
        self.assertCode("CLOCK_RESET_REQUIRED", lambda: self.registry.project(
            observation(), transform(), now_ns=1001))

    def test_alignment_authority_frame_and_clock_are_explicit(self):
        for code, changes in (("WRONG_ALIGNMENT_AUTHORITY", {"authority_id": "other"}),
                              ("FRAME_MISMATCH", {"map_frame": "robot_b/map"}),
                              ("FRAME_MISMATCH", {"mission_frame": "other-mission"}),
                              ("CLOCK_DOMAIN_MISMATCH", {"clock_domain_id": "other"}),
                              ("MAP_VERSION_MISMATCH", {"map_session_id": uid("wrong")})):
            with self.subTest(code=code):
                self.assertCode(code, lambda: self.registry.set_alignment(
                    alignment(transform_revision=2, **changes)))

    def test_alignment_degraded_invalid_uncertain_and_time_window_are_rejected(self):
        cases = [("ALIGNMENT_DEGRADED", {"status": "DEGRADED"}),
                 ("ALIGNMENT_INVALID", {"status": "INVALID"}),
                 ("ALIGNMENT_UNCERTAIN", {"position_error_m": .3}),
                 ("ALIGNMENT_UNCERTAIN", {"rotation_error_rad": .2}),
                 ("ALIGNMENT_EXPIRED_OR_NOT_YET_VALID", {"valid_from_ns": 101}),
                 ("ALIGNMENT_EXPIRED_OR_NOT_YET_VALID", {"valid_until_ns": 100})]
        for revision, (code, changes) in enumerate(cases, 2):
            with self.subTest(code=code, changes=changes):
                self.registry.set_alignment(alignment(transform_revision=revision, **changes))
                self.assertCode(code, lambda: self.project())

    def test_a_map_correction_invalidates_mixed_goal_but_preserves_b_observation(self):
        a, b = self.project("a"), self.project("b")
        goal = self.goal((uid("a-obs"), uid("b-obs")))
        event = self.registry.advance_map_revision("a", uid("a-session"), 1)
        self.assertEqual((uid("a-obs"),), event.observation_ids)
        self.assertEqual((goal.snapshot_id,), event.goal_snapshot_ids)
        self.assertEqual("NEEDS_REOBSERVATION", self.registry.projection(uid("a-obs"), now_ns=101).status)
        self.assertEqual(b, self.registry.projection(uid("b-obs"), now_ns=101))
        self.assertEqual(a.original, self.registry.original(uid("a-obs")))
        self.assertFalse(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)
        self.assertEqual(("a", "b"), tuple(ref.version.source_robot for ref in goal.observations))

    def test_corrected_map_needs_new_observation_not_current_tf_added_to_old_point(self):
        self.project()
        self.registry.advance_map_revision("a", uid("a-session"), 1)
        self.registry.set_alignment(alignment(map_revision=1, transform_revision=2))
        self.assertCode("NEEDS_REOBSERVATION", lambda: self.registry.project(
            observation(), transform(map_revision=1), now_ns=101))
        self.assertCode("ID_CONTENT_CONFLICT", lambda: self.registry.project(
            observation(map_revision=1), transform(map_revision=1), now_ns=101))
        fresh = self.registry.project(observation(oid="a-new", map_revision=1),
                                      transform(map_revision=1), now_ns=101)
        self.assertEqual(1, fresh.version.map_revision)

    def test_alignment_only_change_can_reproject_same_sensor_time_transform(self):
        previous = self.project()
        goal = self.goal()
        event = self.registry.set_alignment(alignment(transform_revision=2,
                                                      transform=RigidTransform.planar(10, 0, 0)))
        self.assertEqual((uid("a-obs"),), event.observation_ids)
        current = self.registry.project(observation(), transform(), now_ns=101)
        self.assertPoint((13, 2, 3), current.mission_point_xyz)
        self.assertPoint((3, 2, 3), previous.mission_point_xyz)
        self.assertEqual(previous, self.registry.projection_history(uid("a-obs"))[0])
        self.assertFalse(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)

    def test_same_observation_cannot_silently_replace_its_bound_tf(self):
        self.project()
        self.assertCode("TF_CONTENT_CONFLICT", lambda: self.registry.project(
            observation(), transform(transform=RigidTransform.identity()), now_ns=101))

    def test_destination_restart_invalidates_goal_from_other_robot(self):
        self.project()
        goal = self.goal()
        event = self.registry.start_session("b", "map", uid("b-session-new"))
        self.assertEqual((), event.observation_ids)
        self.assertEqual((goal.snapshot_id,), event.goal_snapshot_ids)
        self.assertFalse(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)
        self.assertEqual("VALID", self.registry.projection(uid("a-obs"), now_ns=101).status)
        self.assertCode("RETIRED_SESSION", lambda: self.registry.start_session("b", "map", uid("b-session")))

    def test_map_content_update_does_not_fabricate_coordinate_revision(self):
        self.project()
        goal = self.goal()
        state = self.registry.record_map_update("a", uid("a-session"), 7)
        self.assertEqual(0, state.map_revision)
        self.assertTrue(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)
        self.assertCode("STALE_MAP_UPDATE", lambda: self.registry.record_map_update("a", uid("a-session"), 6))

    def test_revisions_cannot_go_backwards_or_change_content_under_same_number(self):
        self.assertEqual("UNCHANGED", self.registry.set_alignment(alignment()).reason)
        self.assertCode("ALIGNMENT_REVISION_CONFLICT", lambda: self.registry.set_alignment(
            alignment(transform=RigidTransform.planar(1, 0, 0))))
        self.assertCode("STALE_ALIGNMENT_REVISION", lambda: self.registry.set_alignment(alignment(transform_revision=0)))
        self.assertCode("NONINCREASING_MAP_REVISION", lambda: self.registry.advance_map_revision("a", uid("a-session"), 0))
        self.assertCode("MAP_SESSION_MISMATCH", lambda: self.registry.advance_map_revision("a", uid("other"), 1))
        self.assertCode("SESSION_CONTENT_CONFLICT", lambda: self.registry.start_session("a", "other-map", uid("a-session")))

    def test_goal_recheck_closes_revision_change_after_acceptance_in_pure_model(self):
        self.project()
        goal = self.goal()
        self.assertTrue(self.registry.validate_goal_snapshot(goal, now_ns=100).coordinates_valid)
        self.registry.set_alignment(alignment("b", transform_revision=2))
        self.assertFalse(self.registry.validate_goal_snapshot(goal, now_ns=101).coordinates_valid)

    def test_goal_snapshot_cannot_be_modified_or_used_in_another_registry(self):
        self.project()
        goal = self.goal()
        altered = replace(goal, destination_point_xyz=(999, 0, 0))
        self.assertFalse(self.registry.validate_goal_snapshot(altered, now_ns=100).coordinates_valid)
        other = MappingRegistry(clock_domain_id="test-clock", alignment_authority="mapping",
                                max_position_error_m=.2, max_rotation_error_rad=.1)
        self.assertFalse(other.validate_goal_snapshot(goal, now_ns=100).coordinates_valid)

    def test_expiry_uses_original_deadline_and_alignment_deadline(self):
        self.registry.set_alignment(alignment("a", transform_revision=2, valid_until_ns=150))
        result = self.project()
        goal = self.goal()
        self.assertEqual(150, result.valid_until_ns)
        self.assertEqual(150, goal.valid_until_ns)
        self.assertEqual("EXPIRED", self.registry.projection(uid("a-obs"), now_ns=150).status)
        self.assertFalse(self.registry.validate_goal_snapshot(goal, now_ns=150).coordinates_valid)

    def test_explicit_alignment_failure_invalidates_affected_goals(self):
        self.project()
        goal = self.goal()
        event = self.registry.invalidate_alignment("a", "ANCHOR_LOST")
        self.assertEqual((goal.snapshot_id,), event.goal_snapshot_ids)
        self.assertEqual("ANCHOR_LOST", self.registry.projection(uid("a-obs"), now_ns=100).reason)
        self.assertCode("ALIGNMENT_REVISION_CONFLICT", lambda: self.registry.set_alignment(alignment()))

    def test_geometry_valid_never_means_backend_or_automatic_approach_valid(self):
        result = self.project()
        goal = self.goal()
        validation = self.registry.validate_goal_snapshot(goal, now_ns=100)
        self.assertTrue(validation.coordinates_valid)
        for value in (result, goal, validation):
            self.assertIs(False, value.auto_approach_eligible)
        self.assertIn("BACKEND_NOT_INTEGRATED", validation.reasons)

    def test_invalid_numeric_values_and_unknown_robot_are_not_coordinates(self):
        for point in ((math.nan, 0, 1), (0, math.inf, 1), (True, 0, 1), (0, 1), ("0", 0, 1)):
            with self.subTest(point=point):
                self.assertCode("INVALID_INPUT", lambda: observation(point_xyz=point))
        self.assertCode("INVALID_QUATERNION", lambda: RigidTransform((0, 0, 0), (0, 0, 0, 0)))
        self.assertCode("INVALID_INPUT", lambda: observation(map_revision=True))
        self.assertCode("INVALID_INPUT", lambda: alignment(position_error_m=-1))
        self.assertCode("UNKNOWN_ROBOT", lambda: self.registry.retain_observation(observation("c")))

    def test_uuid_fields_require_canonical_nonzero_uuid(self):
        for value in ("not-a-uuid", "00000000-0000-0000-0000-000000000000",
                      uid("x").upper(), uid("x").replace("-", ""), None, []):
            for field in ("observation_id", "source_boot_id", "map_session_id"):
                with self.subTest(value=value, field=field):
                    self.assertCode("INVALID_INPUT", lambda: observation(**{field: value}))
            self.assertCode("INVALID_INPUT", lambda: transform(map_session_id=value))
            self.assertCode("INVALID_INPUT", lambda: alignment(map_session_id=value))
            self.assertCode("INVALID_INPUT", lambda: self.registry.start_session("c", "map", value))

    def test_uint64_revision_bounds_apply_to_inputs_and_registry_updates(self):
        self.assertEqual(2**64 - 1, observation(map_revision=2**64 - 1).map_revision)
        for value in (-1, 2**64, True, 1.0):
            self.assertCode("INVALID_INPUT", lambda: observation(map_revision=value))
            self.assertCode("INVALID_INPUT", lambda: transform(map_revision=value))
            self.assertCode("INVALID_INPUT", lambda: alignment(map_revision=value))
            self.assertCode("INVALID_INPUT", lambda: alignment(transform_revision=value))
            self.assertCode("INVALID_INPUT", lambda: self.registry.advance_map_revision("a", uid("a-session"), value))
            self.assertCode("INVALID_INPUT", lambda: self.registry.record_map_update("a", uid("a-session"), value))


if __name__ == "__main__":
    unittest.main()
