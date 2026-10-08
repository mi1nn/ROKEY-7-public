"""Review regressions: admission authority, holder identity and nanosecond boundaries."""
from dataclasses import replace
import unittest
from uuid import UUID

from rescue_coordination.planning import (ApproachPolicy, PathCandidate, Reservation,
    Allocator, ApproachTracker, Corridor, CoordinationError, select_candidate)


def uid(n):
    return str(UUID(int=n))


class CoordinationReviewTests(unittest.TestCase):
    def policy(self):
        return ApproachPolicy('test-only', .6, 1.4, .3, .2, 1., 500_000_000)

    def candidate(self, robot='unit_2'):
        return PathCandidate(robot, (1., 0.), (0., 0.), 4., .4, .1,
                             1_800_000_000_000_000_100, uid(1), 0, 0, True, True, True)

    def allocator(self):
        return Allocator(uid(4), allowed_robot_ids=('unit_1', 'unit_2'))

    def item(self, allocator=None, robot='unit_2', n=0):
        generation = allocator.issue_generation(robot) if allocator is not None else n
        return Reservation(uid(20+n), uid(30+n), 0, robot, generation, uid(4),
                           self.candidate(robot), command_id=uid(40+n), robot_boot_id=uid(50))

    def receipt(self, item, accepted=True, **changes):
        from rescue_contracts import TaskAdmission
        return TaskAdmission(**(dict(command_id=item.command_id, task_id=item.task_id,
            robot_id=item.robot_id, robot_boot_id=item.robot_boot_id, authority_epoch=item.authority_epoch,
            generation=item.generation, accepted=accepted, reason='ACCEPTED' if accepted else 'NAV_NOT_READY') | changes))

    def test_dto_accepts_profile_robot_names_not_a_b_constants(self):
        self.assertEqual('unit_2', self.candidate().robot_id)
        for invalid in ('2unit', 'unit/2', '', True):
            with self.assertRaises(CoordinationError):
                self.candidate(invalid)

    def test_reserve_is_pending_until_matching_executor_admission(self):
        allocator = self.allocator()
        item = self.item(allocator)
        self.assertTrue(allocator.reserve(item))
        self.assertEqual({}, allocator.active)
        self.assertEqual({item.task_id: item}, allocator.pending)
        self.assertEqual(-1, allocator.accepted_generations[item.robot_id])
        self.assertEqual(item.generation, allocator.issued_generations[item.robot_id])
        self.assertTrue(allocator.record_admission(self.receipt(item)))
        self.assertEqual({item.task_id: item}, allocator.active)
        self.assertEqual({}, allocator.pending)
        self.assertEqual(item.generation, allocator.accepted_generations[item.robot_id])

    def test_unissued_generation_or_robot_cannot_be_reserved(self):
        allocator = self.allocator()
        self.assertFalse(allocator.reserve(self.item()))
        with self.assertRaises(CoordinationError):
            allocator.issue_generation('unit_3')
        rogue = self.item(robot='unit_3')
        self.assertFalse(allocator.reserve(rogue))
        item = self.item(allocator)
        self.assertFalse(allocator.reserve(replace(item, generation=item.generation+1)))
        self.assertTrue(allocator.reserve(item))

    def test_receipt_identity_mismatch_cannot_activate_or_raise_counter(self):
        allocator = self.allocator()
        item = self.item(allocator)
        allocator.reserve(item)
        for change in (dict(command_id=uid(90)), dict(task_id=uid(91)), dict(robot_id='unit_1'),
                       dict(robot_boot_id=uid(92)), dict(authority_epoch=uid(93)), dict(generation=1)):
            self.assertFalse(allocator.record_admission(self.receipt(item, **change)))
            self.assertEqual(-1, allocator.accepted_generations[item.robot_id])
            self.assertIn(item.task_id, allocator.pending)
        self.assertTrue(allocator.record_admission(self.receipt(item)))

    def test_rejection_keeps_accepted_counter_and_cannot_later_flip_to_acceptance(self):
        allocator = self.allocator()
        item = self.item(allocator)
        allocator.reserve(item)
        rejected = self.receipt(item, False)
        self.assertFalse(allocator.record_admission(rejected))
        self.assertFalse(allocator.record_admission(rejected))
        self.assertEqual(-1, allocator.accepted_generations[item.robot_id])
        self.assertNotIn(item.task_id, allocator.pending)
        self.assertEqual({}, allocator.active)
        with self.assertRaises(CoordinationError) as caught:
            allocator.record_admission(self.receipt(item))
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)
        self.assertFalse(allocator.reserve(item))
        next_item = self.item(allocator, n=1)
        self.assertGreater(next_item.generation, item.generation)
        self.assertTrue(allocator.reserve(next_item))

    def test_pending_holds_robot_target_until_rejection_or_verified_stop(self):
        allocator = self.allocator()
        item = self.item(allocator)
        allocator.reserve(item)
        same_robot = self.item(allocator, n=1)
        self.assertFalse(allocator.reserve(same_robot))
        other = self.item(allocator, robot='unit_1', n=2)
        self.assertFalse(allocator.reserve(replace(other, target_id=item.target_id)))
        self.assertFalse(allocator.release(item.task_id, robot_id=item.robot_id,
            generation=item.generation, authority_epoch=item.authority_epoch, stop_confirmed=False))
        self.assertTrue(allocator.release(item.task_id, robot_id=item.robot_id,
            generation=item.generation, authority_epoch=item.authority_epoch, stop_confirmed=True))
        self.assertFalse(allocator.record_admission(self.receipt(item)))
        self.assertEqual({}, allocator.active)
        self.assertTrue(allocator.reserve(same_robot))

    def test_accepted_receipt_replay_after_release_does_not_reactivate(self):
        allocator = self.allocator()
        item = self.item(allocator)
        allocator.reserve(item)
        receipt = self.receipt(item)
        self.assertTrue(allocator.record_admission(receipt))
        self.assertTrue(allocator.record_admission(receipt))
        allocator.release(item.task_id, robot_id=item.robot_id, generation=item.generation,
                          authority_epoch=item.authority_epoch, stop_confirmed=True)
        self.assertFalse(allocator.record_admission(receipt))
        self.assertEqual({}, allocator.active)
        self.assertEqual(item.generation, allocator.accepted_generations[item.robot_id])

    def test_command_or_generation_cannot_be_reused_for_another_task(self):
        allocator = self.allocator()
        item = self.item(allocator)
        allocator.reserve(item)
        allocator.record_admission(self.receipt(item, False))
        with self.assertRaises(CoordinationError):
            allocator.reserve(replace(item, task_id=uid(98)))
        self.assertFalse(allocator.reserve(replace(item, task_id=uid(98), command_id=uid(99))))

    def test_corridor_requires_exact_holder_even_after_expiry(self):
        item = self.item()
        other = replace(item, command_id=uid(90))
        for expire in (False, True):
            c = Corridor()
            self.assertFalse(c.confirm_vacant(holder=item, expected_revision=0,
                verified_exit=True, observation_fresh=True, now=0.))
            self.assertTrue(c.confirm_vacant(holder=None, expected_revision=0,
                verified_exit=True, observation_fresh=True, now=0.))
            c.acquire(item, now=0., duration=1.)
            if expire:
                c.check(now=1., holder_pose_fresh=True)
            for wrong in (None, other):
                self.assertFalse(c.confirm_vacant(holder=wrong, expected_revision=c.revision,
                    verified_exit=True, observation_fresh=True, now=1.))
                self.assertEqual(item, c.holder)
            self.assertTrue(c.confirm_vacant(holder=item, expected_revision=c.revision,
                verified_exit=True, observation_fresh=True, now=1.))
            self.assertEqual('FREE', c.state)

    def test_nanosecond_expiry_and_confirmation_at_large_epoch(self):
        c = self.candidate()
        self.assertIs(c, select_candidate([c], now_ns=c.valid_until_ns-1,
            policy=self.policy(), available_robots=('unit_2',)))
        self.assertIsNone(select_candidate([c], now_ns=c.valid_until_ns,
            policy=self.policy(), available_robots=('unit_2',)))
        t = ApproachTracker(uid(3), 0, self.policy())
        observed = c.valid_until_ns-10
        self.assertTrue(t.observe(uid(4), observed_at_ns=observed, valid_until_ns=c.valid_until_ns,
            now_wall_ns=observed, received_steady=1., target_id=uid(3), target_revision=0,
            same_identity=True, freshness_verified=True))
        kw = dict(now_wall_ns=observed, now_steady=1., distance=1., uncertainty=.1,
                  nav_succeeded=True, stop_confirmed=True, local_guards_valid=True)
        self.assertTrue(t.complete(**kw, navigation_finished_at_ns=observed-1))
        self.assertFalse(t.complete(**kw, navigation_finished_at_ns=observed))
        self.assertEqual('CANCEL', t.guard(now_wall_ns=c.valid_until_ns,
                                         now_steady=1., local_guards_valid=True))

    def test_nanosecond_fields_reject_float_bool_zero_and_overflow(self):
        for value in (1., True, 0, -1, 2**63):
            with self.assertRaises(CoordinationError):
                replace(self.candidate(), valid_until_ns=value)
        for value in (1., True, -1, 2**63):
            with self.assertRaises(CoordinationError):
                replace(self.policy(), confirmation_max_age_ns=value)

    def test_deployment_ids_and_generation_overflow_are_explicit(self):
        for ids in (None, ('unit_1',), ('unit_1', 'unit_1'), ('unit_1', 'unit_2', 'unit_3')):
            with self.assertRaises(CoordinationError):
                Allocator(uid(4), allowed_robot_ids=ids)
        allocator = self.allocator()
        allocator.issued_generations['unit_2'] = 2**64-1
        with self.assertRaises(CoordinationError):
            allocator.issue_generation('unit_2')
        self.assertEqual(2**64-1, allocator.issued_generations['unit_2'])
        self.assertEqual(-1, allocator.accepted_generations['unit_2'])

    def test_wall_rollback_latches_cancellation(self):
        t = ApproachTracker(uid(3), 0, self.policy())
        wall = self.candidate().valid_until_ns-10
        t.observe(uid(4), observed_at_ns=wall, valid_until_ns=wall+10,
            now_wall_ns=wall+2, received_steady=1., target_id=uid(3), target_revision=0,
            same_identity=True, freshness_verified=True)
        with self.assertRaises(CoordinationError) as caught:
            t.guard(now_wall_ns=wall+1, now_steady=1.1, local_guards_valid=True)
        self.assertEqual('CLOCK_UNCERTAIN', caught.exception.code)
        self.assertEqual('CANCEL', t.guard(now_wall_ns=wall+3, now_steady=1.2,
                                         local_guards_valid=True))

    def test_error_is_common_machine_readable_contract(self):
        from rescue_contracts import RescueError
        with self.assertRaises(RescueError) as caught:
            self.candidate('bad/name')
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        self.assertEqual('rescue_coordination', caught.exception.component)
        self.assertTrue(caught.exception.detail)
        self.assertTrue(caught.exception.detail_code.startswith('rescue_coordination.'))


if __name__ == '__main__':
    unittest.main()
