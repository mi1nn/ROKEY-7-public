"""Bundle 1 contract additions: robot roles, fixed-camera site events, modes, rescue states."""

import unittest
from uuid import UUID

from rescue_contracts import (RescueError, ROBOT_ROLES, SITE_EVENT_KINDS, SYSTEM_MODES, RESCUE_STATES,
                              TASK_KINDS, TARGET_TASK_KINDS, SiteEvent, camera_id, robot_id, robot_role)


def uid(n):
    return str(UUID(int=n))


class SiteContractTests(unittest.TestCase):
    def event(self, **changes):
        args = dict(event_id=uid(1), camera_id="cam_a", kind="DISASTER_SIGN", observed_at_ns=1700000000123456789,
                    evidence_id=uid(2), evidence_sha256="a" * 64, confidence=0.9,
                    clock_domain_id="site-clock", stamp_semantics="CAPTURE", clock_state="SYNCED",
                    clock_error_bound_ns=5_000_000, detail="edge shake")
        args.update(changes)
        return SiteEvent(**args)

    def test_vocabularies_are_closed_and_disjoint_from_error_codes(self):
        self.assertEqual({"SCOUT", "SUPPORT"}, set(ROBOT_ROLES))
        self.assertEqual({"DISASTER_SIGN", "EXIT_PASS"}, set(SITE_EVENT_KINDS))
        self.assertEqual({"NORMAL", "DISASTER", "CLEARING"}, set(SYSTEM_MODES))
        self.assertEqual({"NOT_CHECKED", "RECHECK_NEEDED", "RESCUE_CONFIRMED"}, set(RESCUE_STATES))
        self.assertEqual({"SEARCH_AREA", "REOBSERVE_TARGET", "APPROACH_TARGET", "HOLD"}, set(TASK_KINDS))
        self.assertTrue(TARGET_TASK_KINDS < TASK_KINDS)
        from rescue_contracts import CANONICAL_CODES
        for vocabulary in (ROBOT_ROLES, SITE_EVENT_KINDS, SYSTEM_MODES, RESCUE_STATES, TASK_KINDS):
            self.assertFalse(vocabulary & CANONICAL_CODES)

    def test_role_and_camera_id_validation(self):
        self.assertEqual("SCOUT", robot_role("SCOUT"))
        for bad in ("scout", "STANDBY", None, True):
            with self.assertRaises(RescueError):
                robot_role(bad)
        self.assertEqual("cam_exit_b", camera_id("cam_exit_b"))
        for bad in ("robot_a", "cam_", "cam-a", "CAM_A", "", None):
            with self.assertRaises(RescueError):
                camera_id(bad)
        # A camera ID is never a valid robot ID namespace collision by syntax alone:
        # robot_id() accepts the same characters, so consumers must keep the sets apart.
        self.assertEqual("cam_a", robot_id("cam_a"))

    def test_site_event_round_trip_and_rejections(self):
        event = self.event()
        self.assertEqual(1700000000123456789, event.observed_at_ns)
        self.assertEqual("DISASTER_SIGN", event.kind)
        for changes in (dict(kind="PERSON"), dict(camera_id="robot_a"), dict(observed_at_ns=1.0),
                        dict(observed_at_ns=0), dict(event_id=uid(0)), dict(evidence_sha256="A" * 64),
                        dict(confidence=1), dict(confidence=1.5), dict(confidence=True), dict(detail=None),
                        dict(clock_domain_id=""), dict(stamp_semantics="RECEIVE"), dict(clock_state="synced"),
                        dict(clock_error_bound_ns=-1), dict(clock_error_bound_ns=1.0)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                self.event(**changes)
        self.assertEqual("EXIT_PASS", self.event(kind="EXIT_PASS", camera_id="cam_b").kind)

    def test_unknown_time_basis_is_recorded_but_not_declared_usable(self):
        self.assertTrue(self.event().time_basis_declared)
        for changes in (dict(stamp_semantics="UNKNOWN"), dict(clock_state="UNKNOWN"), dict(clock_error_bound_ns=None)):
            with self.subTest(changes=changes):
                self.assertFalse(self.event(**changes).time_basis_declared)
        # Declared is a precondition only: a foreign clock domain or a huge bound still
        # declares a basis; comparing with another observation is the consumer's check.
        self.assertTrue(self.event(clock_domain_id="unrelated-clock", clock_error_bound_ns=2**63 - 1).time_basis_declared)

    def test_non_string_enum_inputs_are_contract_errors_not_type_errors(self):
        for changes in (dict(kind=[]), dict(stamp_semantics=[]), dict(clock_state=[]), dict(kind=None),
                        dict(stamp_semantics=1), dict(clock_state=True)):
            with self.subTest(changes=changes), self.assertRaises(RescueError) as caught:
                self.event(**changes)
            self.assertEqual("INVALID_INPUT", caught.exception.code)


if __name__ == "__main__":
    unittest.main()
