from dataclasses import replace
import unittest
from uuid import UUID
from rescue_coordination.planning import (ApproachPolicy, PathCandidate, Reservation, Allocator,
    ApproachTracker, Corridor, CoordinationError, select_candidate)


def uid(n):
    return str(UUID(int=n))


class CoordinationTests(unittest.TestCase):
    def setUp(self):
        self.policy = ApproachPolicy('fixture-only', .6, 1.4, .3, .2, 1, .5)
        self.candidate = PathCandidate('robot_b', (1., 0.), (0., 0.), 4, .4, .1, 110, uid(1), 0, 0,
                                       True, True, True)
        self.item = Reservation(uid(2), uid(3), 0, 'robot_b', 1, uid(4), self.candidate)

    def choose(self, candidates):
        return select_candidate(candidates, now=100, policy=self.policy, available_robots=('robot_b',))

    def test_candidate_uses_path_and_protects_person(self):
        near_but_long_path = replace(self.candidate, path_length=9)
        self.assertEqual(self.choose([near_but_long_path, self.candidate]), self.candidate)
        for change in (dict(goal_xy=(0., 0.)), dict(reachable=False), dict(footprint_free=False),
                       dict(path_collision_free=False), dict(footprint_clearance=.1), dict(uncertainty=.3),
                       dict(goal_xy=(.65, 0.)), dict(goal_xy=(1.35, 0.)), dict(valid_until=100)):
            self.assertIsNone(self.choose([replace(self.candidate, **change)]))

    def test_unavailable_observer_is_not_reassigned(self):
        self.assertIsNone(self.choose([replace(self.candidate, robot_id='robot_a')]))

    def test_allocator_single_event_single_robot_idempotency(self):
        allocator = Allocator(uid(4))
        self.assertTrue(allocator.reserve(self.item))
        self.assertTrue(allocator.reserve(self.item))
        with self.assertRaises(CoordinationError):
            allocator.reserve(replace(self.item, target_revision=1))
        self.assertFalse(allocator.reserve(replace(self.item, task_id=uid(5), generation=2)))
        self.assertFalse(allocator.release(uid(2), robot_id='robot_b', generation=1,
                                           authority_epoch=uid(4), stop_confirmed=False))
        self.assertTrue(allocator.release(uid(2), robot_id='robot_b', generation=1,
                                          authority_epoch=uid(4), stop_confirmed=True))
        self.assertFalse(allocator.reserve(self.item))
        self.assertFalse(allocator.reserve(replace(self.item, task_id=uid(5))))

    def tracker(self):
        tracker = ApproachTracker(uid(3), 0, self.policy)
        self.assertTrue(tracker.observe(uid(9), observed_at=100, valid_until=102, now_wall=100,
                                       received_steady=1, target_id=uid(3), target_revision=0,
                                       same_identity=True, freshness_verified=True))
        return tracker

    def test_duplicate_does_not_refresh_gap(self):
        tracker = self.tracker()
        self.assertFalse(tracker.observe(uid(9), observed_at=100, valid_until=102, now_wall=101,
                                         received_steady=1.9, target_id=uid(3), target_revision=0,
                                         same_identity=True, freshness_verified=True))
        self.assertEqual(tracker.guard(now_wall=101.1, now_steady=2.1, local_guards_valid=True), 'CANCEL')

    def test_original_expiry_and_guard_loss_cancel_irrevocably(self):
        for wall, steady, guards in ((102, 1.1, True), (100.1, 1.1, False)):
            tracker = self.tracker()
            self.assertEqual(tracker.guard(now_wall=wall, now_steady=steady, local_guards_valid=guards), 'CANCEL')
            self.assertFalse(tracker.observe(uid(10), observed_at=103, valid_until=105, now_wall=103,
                                             received_steady=3, target_id=uid(3), target_revision=0,
                                             same_identity=True, freshness_verified=True))

    def test_success_requires_fresh_reobservation_stop_and_distance(self):
        tracker = self.tracker()
        tracker.observe(uid(10), observed_at=100.15, valid_until=102, now_wall=100.15,
                        received_steady=1.15, target_id=uid(3), target_revision=0,
                        same_identity=True, freshness_verified=True)
        common = dict(now_wall=100.2, now_steady=1.2, distance=1, uncertainty=.1,
                      nav_succeeded=True, stop_confirmed=True, local_guards_valid=True, navigation_finished_at=100.1)
        self.assertTrue(tracker.complete(**common))
        for change in (dict(stop_confirmed=False), dict(nav_succeeded=False), dict(distance=.6),
                       dict(now_wall=100.7, now_steady=1.7)):
            self.assertFalse(tracker.complete(**(common | change)))

    def test_no_target_no_permission(self):
        tracker = ApproachTracker(uid(3), 0, self.policy)
        self.assertEqual(tracker.guard(now_wall=100, now_steady=1, local_guards_valid=True), 'CANCEL')

    def test_corridor_expiry_and_stale_pose_stay_unknown(self):
        for expired in (True, False):
            c = Corridor()
            self.assertFalse(c.acquire(self.item, now=0, duration=1))
            self.assertTrue(c.confirm_vacant(expected_revision=0, verified_exit=True, observation_fresh=True, now=0))
            self.assertTrue(c.acquire(self.item, now=0, duration=1))
            self.assertEqual(c.check(now=1 if expired else .5, holder_pose_fresh=expired), 'OCCUPANCY_UNKNOWN')
            self.assertFalse(c.acquire(self.item, now=1, duration=1))
            self.assertFalse(c.renew(self.item, expected_revision=c.revision, sequence=1,
                                      now=1, duration=1, holder_pose_fresh=True))
            self.assertFalse(c.confirm_vacant(expected_revision=c.revision-1, verified_exit=True,
                                              observation_fresh=True, now=1))
            self.assertTrue(c.confirm_vacant(expected_revision=c.revision, verified_exit=True,
                                             observation_fresh=True, now=1))

    def test_corridor_duplicate_renewal_cannot_extend(self):
        c = Corridor()
        c.confirm_vacant(expected_revision=0, verified_exit=True, observation_fresh=True, now=0)
        c.acquire(self.item, now=0, duration=1)
        self.assertTrue(c.renew(self.item, expected_revision=c.revision, sequence=1,
                                now=.2, duration=1, holder_pose_fresh=True))
        self.assertFalse(c.renew(self.item, expected_revision=c.revision, sequence=1,
                                 now=.3, duration=2, holder_pose_fresh=True))
        self.assertEqual(c.check(now=1.2, holder_pose_fresh=True), 'OCCUPANCY_UNKNOWN')

    def test_new_observation_cannot_erase_gap_or_original_expiry(self):
        for wall, steady in ((103, 4), (101, 3)):
            tracker = self.tracker()
            self.assertFalse(tracker.observe(uid(11), observed_at=wall, valid_until=wall+2,
                now_wall=wall, received_steady=steady, target_id=uid(3), target_revision=0,
                same_identity=True, freshness_verified=True))
            self.assertEqual(tracker.guard(now_wall=wall, now_steady=steady,
                                          local_guards_valid=True), 'CANCEL')

    def test_observation_at_arrival_is_not_postarrival_confirmation(self):
        tracker = self.tracker()
        self.assertFalse(tracker.complete(now_wall=100, now_steady=1, distance=1,
            uncertainty=.1, nav_succeeded=True, stop_confirmed=True, local_guards_valid=True,
            navigation_finished_at=100))

    def test_overflowing_corridor_lease_rejected_before_mutation(self):
        c = Corridor()
        c.confirm_vacant(expected_revision=0, verified_exit=True, observation_fresh=True, now=0)
        with self.assertRaises(CoordinationError):
            c.acquire(self.item, now=1e308, duration=1e308)
        self.assertEqual(c.state, 'FREE')
        with self.assertRaises(CoordinationError):
            c.acquire(self.item, now=1e308, duration=1)
        self.assertEqual(c.state, 'FREE')
        for changes in (dict(path_length=10**1000), dict(map_session_id=uid(0))):
            with self.assertRaises(CoordinationError):
                replace(self.candidate, **changes)

    def test_method_inputs_cannot_use_boolean_revisions(self):
        allocator = Allocator(uid(4))
        allocator.reserve(self.item)
        with self.assertRaises(CoordinationError):
            allocator.release(self.item.task_id, robot_id='robot_b', generation=True,
                              authority_epoch=uid(4), stop_confirmed=True)
        self.assertIn(self.item.task_id, allocator.active)
        tracker = ApproachTracker(uid(3), 1, self.policy)
        with self.assertRaises(CoordinationError):
            tracker.observe(uid(9), observed_at=100, valid_until=102, now_wall=100,
                            received_steady=1, target_id=uid(3), target_revision=True,
                            same_identity=True, freshness_verified=True)
        with self.assertRaises(CoordinationError):
            ApproachTracker(uid(3), 1, None)

    def test_invalid_input_and_clock(self):
        for change in (dict(path_length=True), dict(goal_xy=(float('nan'), 0.)), dict(map_revision=True),
                       dict(reachable=1)):
            with self.assertRaises(CoordinationError):
                replace(self.candidate, **change)
        tracker = self.tracker()
        with self.assertRaises(CoordinationError):
            tracker.guard(now_wall=100, now_steady=.5, local_guards_valid=True)


if __name__ == '__main__':
    unittest.main()
