"""A TargetLocation becomes a target only with a usable observation time (OPS-103, #142; E2).

The coordinator checks ``observation.time``: a declared basis (stamp semantics not UNKNOWN, SYNCED, an
error bound), the coordinator's ``target_clock_domain_id``, not later than now by more than
``target_max_future_s``, not older than ``target_max_age_s`` and before its own ``valid_until_ns``.
Anything else makes no target, no report, and SystemState.reason says why until a location is accepted.
"""
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from test_coordination_ros_core import (TARGET_AGE, TARGET_FUTURE, location, make_core, obs_time, site_msg)
from test_coordination_search import NOW, uid

MS = 1_000_000
from types import SimpleNamespace as NS


def timed(n, **time_changes):
    return location(n, observation=NS(time=obs_time(**time_changes), class_label='survivor'))


class TargetTimeGateTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core()
        self.core.on_site_event(site_msg())

    def refused(self, msg, code, reason):
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(msg, now_ns=NOW)
        self.assertEqual(code, caught.exception.code)
        self.assertEqual(({}, []), (self.core.mission.targets, self.core.reports_due(NOW)))
        self.assertEqual(reason, self.core.system_state_change(NOW)['reason'])

    def test_a_usable_time_makes_a_target(self):
        self.assertIsNotNone(self.core.on_target_location(timed(1), now_ns=NOW))

    def test_undeclared_time_basis_is_refused(self):
        for n, changes in enumerate((dict(stamp_semantics='UNKNOWN'), dict(clock_state='UNKNOWN'),
                                     dict(has_clock_error_bound=False)), 1):
            with self.subTest(changes=changes):
                self.setUp()
                self.refused(timed(n, **changes), 'CLOCK_UNCERTAIN', 'TARGET_TIME_BASIS_UNUSABLE')

    def test_empty_and_invalid_stamp_semantics_never_make_a_target(self):
        for stamp in ('', 'INVALID'):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.refused(timed(1, stamp_semantics=stamp), 'CLOCK_UNCERTAIN', 'TARGET_TIME_BASIS_UNUSABLE')

    def test_driver_receive_is_a_declared_stamp(self):
        self.assertIsNotNone(self.core.on_target_location(timed(1, stamp_semantics='DRIVER_RECEIVE'), now_ns=NOW))

    def test_another_clock_domain_is_refused(self):
        self.refused(timed(1, clock_domain_id='robot-clock'), 'CLOCK_UNCERTAIN', 'TARGET_CLOCK_DOMAIN_MISMATCH')

    def test_a_future_stamp_beyond_the_allowance_plus_its_own_error_bound_is_refused(self):
        # PCs disagree by hundreds of ms (timesyncd, measured 2026-10-13): the observation's declared bound
        # is allowed on top of target_max_future_s, or every fresh observation from a fast clock is refused.
        bound = 400 * MS
        ahead = NOW + TARGET_FUTURE + bound
        self.assertIsNotNone(self.core.on_target_location(
            timed(1, observed_at_ns=ahead, clock_error_bound_ns=bound), now_ns=NOW))
        self.setUp()
        self.refused(timed(2, observed_at_ns=ahead + 1, clock_error_bound_ns=bound), 'STALE_OBSERVATION',
                     'TARGET_OBSERVATION_FUTURE')

    def test_too_old_or_past_its_own_expiry_is_refused(self):
        self.assertIsNotNone(self.core.on_target_location(timed(1, observed_at_ns=NOW - TARGET_AGE), now_ns=NOW))
        self.setUp()
        self.refused(timed(2, observed_at_ns=NOW - TARGET_AGE - 1), 'STALE_OBSERVATION', 'TARGET_OBSERVATION_STALE')
        self.setUp()
        self.refused(timed(3, valid_until_ns=NOW), 'STALE_OBSERVATION', 'TARGET_OBSERVATION_STALE')

    def test_the_reason_clears_when_a_location_is_accepted(self):
        with self.assertRaises(RescueError):
            self.core.on_target_location(timed(1, clock_state='UNKNOWN'), now_ns=NOW)
        self.core.on_target_location(timed(2), now_ns=NOW)
        self.assertEqual('', self.core.system_state_change(NOW)['reason'])

    def test_a_replay_of_an_accepted_location_is_still_the_same_target(self):
        target = self.core.on_target_location(timed(1), now_ns=NOW)
        self.assertEqual(target, self.core.on_target_location(timed(1), now_ns=NOW + 10 * TARGET_AGE))

    def test_policy_values_are_typed_and_required(self):
        self.assertEqual(('STRING', 'DOUBLE', 'DOUBLE'), tuple(ros_node.PARAMETERS[n] for n in (
            'target_clock_domain_id', 'target_max_age_s', 'target_max_future_s')))
        for changes in (dict(target_clock_domain_id=''), dict(target_max_age_ns=0), dict(target_max_future_ns=-1)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                make_core(**changes)


if __name__ == '__main__':
    unittest.main()
