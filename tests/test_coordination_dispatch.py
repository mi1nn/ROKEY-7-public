"""Which confirmed target the SUPPORT robot goes to next (Part I 3.4.3/3.4.5, Q-19) (E2)."""
from dataclasses import replace
import itertools
import unittest
from uuid import UUID

from rescue_coordination.planning import Allocator, CoordinationError, PathCandidate
from rescue_coordination import dispatch
from test_coordination_search import BOOT, EPOCH, MAP, NOW, policy, uid

SUPPORT = 'unit_2'


def candidate(target_xy, path_length=4.0, robot=SUPPORT, **changes):
    goal = (target_xy[0] + 1.0, target_xy[1])
    values = dict(robot_id=robot, goal_xy=goal, target_xy=target_xy, path_length=path_length,
                  footprint_clearance=.4, uncertainty=.1, valid_until_ns=NOW + 10**9, map_session_id=MAP,
                  map_revision=3, transform_revision=1, footprint_free=True, reachable=True,
                  path_collision_free=True)
    return PathCandidate(**(values | changes))


def target(n, target_xy=(0., 0.), path_length=4.0, **changes):
    values = dict(event_id=uid(500 + n), target_revision=0, detection_confirmed=True,
                  projection_status='VALID', rescue_state='NOT_CHECKED',
                  candidate=candidate(target_xy, path_length))
    return dispatch.DispatchTarget(**(values | changes))


def origin(xy=(10., 0.), **changes):
    values = dict(robot_id=SUPPORT, xy=xy, map_session_id=MAP, map_revision=3, transform_revision=1,
                  stamp_ns=NOW - 10**8)
    return dispatch.SupportOrigin(**(values | changes))


def rule(order='STRAIGHT_LINE', **changes):
    values = dict(version='test-only', order=order, max_origin_age_ns=10**9)
    return dispatch.DispatchPolicy(**(values | changes))


def select(targets, *, order='STRAIGHT_LINE', assigned=frozenset(), at=None, **changes):
    return dispatch.select_dispatch_target(
        targets, policy=rule(order), support_robot_id=SUPPORT, assigned_events=assigned,
        origin=changes.get('origin', origin()), now_ns=NOW if at is None else at)


class DispatchSelectionTests(unittest.TestCase):
    def test_straight_line_picks_the_nearest_target_to_the_support_robot(self):
        near, far = target(1, (8., 0.), path_length=50.), target(2, (0., 0.), path_length=1.)
        choice = select((far, near))
        self.assertEqual(('SELECTED', near.event_id), (choice.reason, choice.target.event_id))
        self.assertEqual((near.event_id, far.event_id), choice.ranked)

    def test_path_length_order_uses_the_verified_path_cost(self):
        near, far = target(1, (8., 0.), path_length=50.), target(2, (0., 0.), path_length=1.)
        choice = select((near, far), order='PATH_LENGTH', origin=None)
        self.assertEqual(far.event_id, choice.target.event_id)

    def test_tie_breaks_by_event_id_not_input_order(self):
        a, b = target(1, (8., 2.)), target(2, (8., -2.))
        self.assertEqual(a.event_id, select((b, a)).target.event_id)
        self.assertEqual(a.event_id, select((a, b)).target.event_id)

    def test_ineligible_targets_are_skipped_with_closed_reasons(self):
        cases = {
            'NOT_CONFIRMED': target(1, detection_confirmed=False),
            'RESCUED': target(2, rescue_state='RESCUE_CONFIRMED'),
            'EVENT_ALREADY_HAS_APPROACH': target(3),
            'PROJECTION_NOT_VALID': target(4, projection_status='STALE'),
            'NO_SUPPORT_CANDIDATE': target(5, candidate=None),
            'CANDIDATE_EXPIRED': target(6, candidate=candidate((0., 0.), valid_until_ns=NOW)),
            'BASIS_MISMATCH': target(7, candidate=candidate((0., 0.), map_revision=4)),
        }
        wrong_robot = target(8, candidate=candidate((0., 0.), robot='unit_1'))
        choice = select(tuple(cases.values()) + (wrong_robot,), assigned=frozenset({cases[
            'EVENT_ALREADY_HAS_APPROACH'].event_id}))
        self.assertEqual(('NO_ELIGIBLE_TARGET', None, ()), (choice.reason, choice.target, choice.ranked))
        skipped = dict(choice.skipped)
        for reason, item in cases.items():
            self.assertEqual(reason, skipped[item.event_id], reason)
        self.assertEqual('NO_SUPPORT_CANDIDATE', skipped[wrong_robot.event_id])
        self.assertLessEqual({r for _, r in choice.skipped}, dispatch.DISPATCH_SKIP_REASONS)

    def test_recheck_needed_target_is_still_eligible(self):
        choice = select((target(1, rescue_state='RECHECK_NEEDED'),))
        self.assertEqual('SELECTED', choice.reason)

    def test_straight_line_without_a_usable_origin_selects_nothing(self):
        for bad in (None, origin(robot_id='unit_1'), origin(stamp_ns=NOW - 2 * 10**9),
                    origin(stamp_ns=NOW + 1)):
            with self.subTest(origin=bad):
                choice = select((target(1),), origin=bad)
                self.assertEqual(('ORIGIN_UNUSABLE', None), (choice.reason, choice.target))

    def test_origin_on_another_map_basis_is_not_compared(self):
        choice = select((target(1),), origin=origin(transform_revision=2))
        self.assertEqual('NO_ELIGIBLE_TARGET', choice.reason)
        self.assertEqual('BASIS_MISMATCH', dict(choice.skipped)[target(1).event_id])

    def test_policy_has_no_default_order_and_rejects_unknown(self):
        with self.assertRaises(TypeError):
            dispatch.DispatchPolicy(version='v', max_origin_age_ns=1)
        for bad in ('NEAREST', '', None):
            with self.subTest(order=bad), self.assertRaises(CoordinationError):
                rule(bad)

    def test_duplicate_event_ids_rejected(self):
        with self.assertRaises(CoordinationError) as caught:
            select((target(1), target(1, (5., 0.))))
        self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_choice_reasons_are_closed(self):
        with self.assertRaises(CoordinationError):
            dispatch.DispatchChoice('PICKED', None, (), ())


class DispatchHandoffTests(unittest.TestCase):
    """The choice feeds SearchCoordinator.request_approach; a reserved event is not chosen again."""

    def setUp(self):
        self.ids = itertools.count(1000)

    def test_selected_target_is_reserved_once(self):
        from rescue_coordination.search import SearchCoordinator
        c = SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=('unit_1', 'unit_2')), policy(),
                              new_id=lambda: uid(next(self.ids)))
        targets = (target(1, (8., 0.)), target(2, (0., 0.)))
        first = c.select_approach_target(targets, policy=rule(), origin=origin(), now_ns=NOW)
        decision = c.request_approach(first.target.event_id, first.target.target_revision,
                                      projection_status=first.target.projection_status,
                                      candidate=first.target.candidate, robot_boot_id=BOOT[SUPPORT], now_ns=NOW)
        self.assertEqual('RESERVED', decision.action)
        second = c.select_approach_target(targets, policy=rule(), origin=origin(), now_ns=NOW)
        self.assertEqual(targets[1].event_id, second.target.event_id)
        self.assertEqual('EVENT_ALREADY_HAS_APPROACH', dict(second.skipped)[targets[0].event_id])


if __name__ == '__main__':
    unittest.main()
