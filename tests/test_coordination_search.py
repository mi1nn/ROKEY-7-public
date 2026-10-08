"""Role-driven SEARCH_AREA/HOLD/APPROACH proposals over the existing Allocator (E2)."""
from dataclasses import replace
import itertools
import unittest
from uuid import UUID

from rescue_contracts import TaskAdmission
from rescue_coordination.planning import (Allocator, CoordinationError, PathCandidate, Reservation)
from rescue_coordination.search import (DECISION_REASONS, EXCLUSION_REASONS, SEARCH_REJECT_REASONS,
    MissionSite, SearchCoordinator, SearchPolicy)
from rescue_robot.executor import Executor, GoalPose, Policy, Readiness, Task

NOW = 1_800_000_000_000_000_000
BOOT = {'unit_1': str(UUID(int=901)), 'unit_2': str(UUID(int=902))}
EPOCH = str(UUID(int=4))
MAP = str(UUID(int=5))


def uid(n):
    return str(UUID(int=n))


def area(n, basis='MAP_REVISION_BOUND', bound=True):
    refs = dict(map_session_id=MAP, map_revision=3, transform_revision=1) if bound else {}
    return MissionSite(uid(100+n), (float(n), 0.), basis, **refs)


def policy(roles=(('unit_1', 'SCOUT'), ('unit_2', 'SUPPORT')), areas=None, **changes):
    values = dict(version='test-only', roles=roles,
                  areas=areas if areas is not None else (area(1), area(2), area(3)),
                  executable_bases=frozenset({'MAP_REVISION_BOUND'}), task_duration_ns=60_000_000_000,
                  lease_duration=10., executor_policy_version='exec-test',
                  mission_frame_id='mission')
    return SearchPolicy(**(values | changes))


def candidate(robot='unit_2'):
    return PathCandidate(robot, (1., 0.), (0., 0.), 4., .4, .1, NOW + 10**9, MAP, 3, 1, True, True, True)


def task_of(fields):
    """What the robot lane does with task_fields: goal_pose arrives as a plain dict."""
    pose = fields.get('goal_pose')
    return Task(**(fields | {'goal_pose': GoalPose(**pose) if pose is not None else None}))


def receipt(item, accepted=True):
    return TaskAdmission(item.command_id, item.task_id, item.robot_id, item.robot_boot_id,
                         item.authority_epoch, item.generation, accepted,
                         'ACCEPTED' if accepted else 'NAV_NOT_READY')


class SearchCoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.ids = itertools.count(1000)

    def make(self, **changes):
        allocator = Allocator(EPOCH, allowed_robot_ids=('unit_1', 'unit_2'))
        return SearchCoordinator(allocator, policy(**changes), new_id=lambda: uid(next(self.ids)))

    def plan(self, c, robot, ready=True, boot=None):
        return c.plan(robot, robot_boot_id=boot or BOOT[robot], ready=ready, now_ns=NOW)

    def approach(self, c, event, robot='unit_2', status='VALID', cand='default'):
        return c.request_approach(event, 0, projection_status=status,
                                  candidate=candidate(robot) if cand == 'default' else cand,
                                  robot_boot_id=BOOT[robot], now_ns=NOW)

    def test_scout_gets_first_area_and_support_gets_hold_as_executor_tasks(self):
        c = self.make()
        scout, support = self.plan(c, 'unit_1'), self.plan(c, 'unit_2')
        self.assertEqual(('RESERVED', 'SEARCH_AREA', area(1).site_id),
                         (scout.action, scout.reservation.task_kind, scout.reservation.target_id))
        self.assertEqual(('RESERVED', 'HOLD'), (support.action, support.reservation.task_kind))
        self.assertIsNone(scout.reservation.candidate)
        for decision in (scout, support):
            task = task_of(decision.task_fields)  # the robot lane's real DTO accepts the fields
            self.assertEqual(decision.reservation.generation, task.generation)
            self.assertIsNone(task.target_id)

    def test_all_three_kinds_build_real_executor_tasks_with_goal_fields(self):
        c = self.make()
        search, hold = self.plan(c, 'unit_1'), self.plan(c, 'unit_2')
        c.finish(hold.reservation, outcome='CANCELED', stop_confirmed=True)
        approach = self.approach(c, uid(500))
        tasks = {d.reservation.task_kind: (d, task_of(d.task_fields)) for d in (search, hold, approach)}
        self.assertEqual({'SEARCH_AREA', 'HOLD', 'APPROACH_TARGET'}, set(tasks))
        decision, task = tasks['SEARCH_AREA']
        self.assertEqual(area(1).site_id, task.search_area_ref)
        self.assertEqual(GoalPose('mission', 1., 0., 0., MAP, 3), task.goal_pose)
        self.assertEqual((None, None), tuple(getattr(tasks['HOLD'][1], k) for k in ('search_area_ref', 'goal_pose')))
        decision, task = tasks['APPROACH_TARGET']
        self.assertIsNone(task.search_area_ref)
        self.assertEqual(GoalPose('mission', 1., 0., 0., MAP, 3), task.goal_pose)  # candidate goal_xy, not target
        self.assertEqual(uid(500), task.target_id)
        # Plain data only: coordination does not import rescue_robot types.
        self.assertIs(dict, type(search.task_fields['goal_pose']))

    def test_reservation_is_pending_until_executor_admission(self):
        c = self.make()
        item = self.plan(c, 'unit_1').reservation
        self.assertEqual(('PENDING', item), c.assignment('unit_1'))
        self.assertEqual({}, c.allocator.active)
        self.assertEqual('PENDING', self.plan(c, 'unit_1').reason)
        self.assertTrue(c.record_admission(receipt(item)))
        self.assertEqual(('ACTIVE', item), c.assignment('unit_1'))

    def test_rejected_admission_does_not_activate_and_next_proposal_uses_new_generation(self):
        c = self.make()
        item = self.plan(c, 'unit_1').reservation
        self.assertFalse(c.record_admission(receipt(item, accepted=False)))
        self.assertIsNone(c.assignment('unit_1'))
        again = self.plan(c, 'unit_1').reservation
        self.assertEqual(item.target_id, again.target_id)
        self.assertGreater(again.generation, item.generation)
        self.assertNotEqual((item.task_id, item.command_id), (again.task_id, again.command_id))

    def test_same_area_is_never_reserved_twice(self):
        c = self.make()
        item = self.plan(c, 'unit_1').reservation
        self.assertEqual('KEEP', self.plan(c, 'unit_1').action)
        # Even a hand-made reservation for the other robot cannot take the area.
        rogue = Reservation(uid(1), item.target_id, 3, 'unit_2', c.allocator.issue_generation('unit_2'),
                            EPOCH, None, uid(2), BOOT['unit_2'], task_kind='SEARCH_AREA')
        self.assertFalse(c.allocator.reserve(rogue))
        self.assertEqual(1, sum(r.target_id == item.target_id for r in c.allocator.pending.values()))

    def test_areas_without_executable_map_basis_are_excluded(self):
        c = self.make(areas=(area(1, basis='OPERATOR_SKETCH'), area(2, bound=False), area(3)))
        self.assertEqual([(area(1).site_id, 'BASIS_NOT_EXECUTABLE'), (area(2).site_id, 'NO_MAP_REVISION')],
                         c.excluded_areas())
        self.assertEqual(area(3).site_id, self.plan(c, 'unit_1').reservation.target_id)
        self.assertTrue({r for _, r in c.excluded_areas()} <= EXCLUSION_REASONS)
        only_bad = self.make(areas=(area(1, basis='OPERATOR_SKETCH'),))
        self.assertEqual(('WAIT', 'NO_EXECUTABLE_AREA'), (lambda d: (d.action, d.reason))(self.plan(only_bad, 'unit_1')))

    def test_switch_needs_confirmed_stop_then_new_generation_and_ids(self):
        c = self.make()
        first = self.plan(c, 'unit_1').reservation
        c.record_admission(receipt(first))
        self.assertFalse(c.finish(first, outcome='COMPLETED', stop_confirmed=False))
        self.assertEqual(('ACTIVE', first), c.assignment('unit_1'))
        self.assertTrue(c.finish(first, outcome='COMPLETED', stop_confirmed=True))
        second = self.plan(c, 'unit_1').reservation
        self.assertEqual(area(2).site_id, second.target_id)
        self.assertGreater(second.generation, first.generation)
        self.assertTrue({second.task_id, second.command_id}.isdisjoint({first.task_id, first.command_id}))

    def test_unfinished_area_is_proposed_again_but_not_counted_as_searched(self):
        c = self.make()
        first = self.plan(c, 'unit_1').reservation
        c.record_admission(receipt(first))
        self.assertTrue(c.finish(first, outcome='UNREACHABLE', stop_confirmed=True))
        self.assertNotIn(first.target_id, c.completed_areas)
        self.assertEqual(first.target_id, self.plan(c, 'unit_1').reservation.target_id)

    def test_late_admission_or_result_cannot_revive_a_released_task(self):
        c = self.make()
        old = self.plan(c, 'unit_1').reservation
        self.assertTrue(c.finish(old, outcome='CANCELED', stop_confirmed=True))
        new = self.plan(c, 'unit_1').reservation
        self.assertFalse(c.record_admission(receipt(old)))
        self.assertNotIn(old.task_id, c.allocator.active)
        self.assertEqual(('PENDING', new), c.assignment('unit_1'))
        # A duplicated stale result neither releases the new task nor marks areas.
        self.assertFalse(c.finish(old, outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual(set(), c.completed_areas)
        self.assertEqual(('PENDING', new), c.assignment('unit_1'))

    def test_not_ready_robot_gets_nothing_new(self):
        c = self.make()
        self.assertEqual(('WAIT', 'NOT_READY'), (lambda d: (d.action, d.reason))(self.plan(c, 'unit_1', ready=False)))
        self.assertEqual({}, c.allocator.pending)
        with self.assertRaises(CoordinationError):
            self.plan(c, 'unit_1', ready=1)

    def test_withdrawn_area_requires_release_before_reassignment(self):
        c = self.make()
        item = self.plan(c, 'unit_1').reservation
        c.withdraw_area(item.target_id)
        decision = self.plan(c, 'unit_1')
        self.assertEqual(('RELEASE_REQUIRED', 'AREA_WITHDRAWN', item),
                         (decision.action, decision.reason, decision.reservation))
        self.assertTrue(c.finish(item, outcome='CANCELED', stop_confirmed=True))
        self.assertEqual(area(2).site_id, self.plan(c, 'unit_1').reservation.target_id)
        self.assertIn((item.target_id, 'WITHDRAWN'), c.excluded_areas())

    def test_hold_is_not_reissued_after_confirmed_hold_for_the_same_boot(self):
        c = self.make()
        hold = self.plan(c, 'unit_2').reservation
        c.record_admission(receipt(hold))
        self.assertTrue(c.finish(hold, outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual(('WAIT', 'HOLDING'), (lambda d: (d.action, d.reason))(self.plan(c, 'unit_2')))
        rebooted = self.plan(c, 'unit_2', boot=uid(777))
        self.assertEqual('HOLD', rebooted.reservation.task_kind)

    def test_discovery_reserves_one_support_approach_per_event_after_hold_release(self):
        c = self.make()
        hold = self.plan(c, 'unit_2').reservation
        event_a, event_b = uid(500), uid(501)
        blocked = self.approach(c, event_a)
        self.assertEqual(('RELEASE_REQUIRED', 'SUPPORT_HOLDING', hold),
                         (blocked.action, blocked.reason, blocked.reservation))
        c.finish(hold, outcome='CANCELED', stop_confirmed=True)
        for status, cand, reason in (('NEEDS_REOBSERVATION', 'default', 'PROJECTION_NOT_VALID'),
                                     ('VALID', None, 'NO_SUPPORT_CANDIDATE'),
                                     ('VALID', candidate('unit_1'), 'NO_SUPPORT_CANDIDATE'),
                                     ('VALID', replace(candidate(), valid_until_ns=NOW), 'CANDIDATE_EXPIRED')):
            self.assertEqual(reason, self.approach(c, event_a, status=status, cand=cand).reason)
        reserved = self.approach(c, event_a)
        self.assertEqual(('RESERVED', 'APPROACH_TARGET', event_a),
                         (reserved.action, reserved.reservation.task_kind, reserved.reservation.target_id))
        self.assertEqual(event_a, task_of(reserved.task_fields).target_id)
        self.assertEqual('EVENT_ALREADY_HAS_APPROACH', self.approach(c, event_a).reason)
        self.assertEqual('SUPPORT_BUSY', self.approach(c, event_b).reason)
        self.assertEqual(1, len(c.approach_tasks))

    def test_approach_reservation_without_path_candidate_is_impossible(self):
        for kind in ('APPROACH_TARGET', 'REOBSERVE_TARGET'):
            with self.assertRaises(CoordinationError):
                Reservation(uid(1), uid(2), 0, 'unit_2', 0, EPOCH, None, uid(3), BOOT['unit_2'], task_kind=kind)
        with self.assertRaises(CoordinationError):  # default kind is a target task
            Reservation(uid(1), uid(2), 0, 'unit_2', 0, EPOCH, None, uid(3), BOOT['unit_2'])
        with self.assertRaises(CoordinationError):
            Reservation(uid(1), uid(2), 0, 'unit_2', 0, EPOCH, None, uid(3), BOOT['unit_2'], task_kind='DOCK')

    def test_scout_keeps_searching_during_approach_unless_policy_pauses(self):
        for continues, expected in ((True, 'RESERVED'), (False, 'PAUSED_FOR_APPROACH')):
            c = self.make(scout_continues_during_approach=continues)
            first = self.plan(c, 'unit_1').reservation
            self.assertEqual('RESERVED', self.approach(c, uid(500)).action)
            c.finish(first, outcome='COMPLETED', stop_confirmed=True)
            decision = self.plan(c, 'unit_1')
            self.assertIn(expected, (decision.action, decision.reason))

    def test_swapping_role_input_swaps_robots(self):
        c = self.make(roles=(('unit_1', 'SUPPORT'), ('unit_2', 'SCOUT')))
        self.assertEqual('SEARCH_AREA', self.plan(c, 'unit_2').reservation.task_kind)
        self.assertEqual('HOLD', self.plan(c, 'unit_1').reservation.task_kind)
        self.assertEqual('RELEASE_REQUIRED', self.approach(c, uid(500), robot='unit_1').action)
        self.assertEqual('NO_SUPPORT_CANDIDATE', self.approach(c, uid(500), robot='unit_2').reason)

    def test_policy_rejects_bad_roles_and_foreign_robots(self):
        for roles in ((('unit_1', 'SCOUT'), ('unit_2', 'SCOUT')), (('unit_1', 'SCOUT'),),
                      (('unit_1', 'SCOUT'), ('unit_1', 'SUPPORT')), (('unit_1', 'LEADER'), ('unit_2', 'SUPPORT'))):
            with self.assertRaises(CoordinationError):
                policy(roles=roles)
        with self.assertRaises(CoordinationError):
            policy(areas=(area(1), area(1)))
        with self.assertRaises(CoordinationError):
            policy(mission_frame_id='')
        with self.assertRaises(CoordinationError):
            SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=('unit_1', 'unit_3')), policy())

    def test_every_returned_reason_is_from_the_closed_set(self):
        c = self.make(scout_continues_during_approach=False)
        decisions = [self.plan(c, 'unit_1'), self.plan(c, 'unit_1'), self.plan(c, 'unit_2', ready=False),
                     self.plan(c, 'unit_2'), self.approach(c, uid(500)), self.approach(c, uid(500), status='STALE')]
        for d in decisions:
            self.assertIn(d.reason, DECISION_REASONS)
            if d.action == 'WAIT':
                self.assertIn(d.reason, SEARCH_REJECT_REASONS)
        with self.assertRaises(CoordinationError) as caught:  # C-09 error, not a bare ValueError
            type(decisions[0])('WAIT', 'some free text')
        self.assertEqual('INVALID_INPUT', caught.exception.code)


class ExecutorHandoffTests(unittest.TestCase):
    """The proposals drive the real executor: admission, cancel, stop, new generation."""

    def test_search_cancel_stop_then_next_area_is_admitted_with_new_generation(self):
        ids = itertools.count(2000)
        allocator = Allocator(EPOCH, allowed_robot_ids=('unit_1', 'unit_2'))
        c = SearchCoordinator(allocator, policy(), new_id=lambda: uid(next(ids)))
        exec_policy = Policy('exec-test', 1., 3., .2, .11, .5, .01, .01)
        executor = Executor('unit_1', BOOT['unit_1'], EPOCH, exec_policy, allowed_robot_ids=('unit_1', 'unit_2'))
        for t in (0., .1, .2):
            executor.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
        ready = Readiness(.2, True, True, True, True, True, True)
        self.assertTrue(executor.arm(EPOCH, now=.2, readiness=ready))

        first = c.plan('unit_1', robot_boot_id=BOOT['unit_1'], ready=True, now_ns=NOW)
        admission = executor.admit(task_of(first.task_fields), now_wall_ns=NOW, now_steady=.2, readiness=ready)
        self.assertTrue(c.record_admission(admission))
        self.assertTrue(executor.dispatch(now_wall_ns=NOW, now_steady=.2, readiness=ready, guards_valid=True))
        executor.goal_response(first.reservation.command_id, accepted=True, now=.3)
        c.withdraw_area(first.reservation.target_id)
        self.assertEqual('RELEASE_REQUIRED', c.plan('unit_1', robot_boot_id=BOOT['unit_1'],
                                                    ready=True, now_ns=NOW).action)
        executor.request_stop('ACTION_CANCELED', now=.5)
        executor.goal_terminal(first.reservation.command_id, terminal_status='CANCELED', now=.6)
        self.assertNotIn(first.reservation.command_id, executor.results)  # no result before stop samples
        for t in (.7, .8, .9, 1.):
            executor.observe_motion(sample_steady=t, received_steady=t, linear=0., angular=0.)
        result = executor.results[first.reservation.command_id]
        self.assertTrue(c.finish(first.reservation, outcome=result.outcome, stop_confirmed=result.stop_confirmed))

        ready = Readiness(1., True, True, True, True, True, True)
        self.assertTrue(executor.arm(EPOCH, now=1., readiness=ready))
        second = c.plan('unit_1', robot_boot_id=BOOT['unit_1'], ready=True, now_ns=NOW + 10**9)
        self.assertEqual(area(2).site_id, second.reservation.target_id)
        stale = executor.admit(task_of(first.task_fields), now_wall_ns=NOW + 10**9, now_steady=1., readiness=ready)
        self.assertFalse(c.record_admission(stale))  # replayed old receipt
        admission = executor.admit(task_of(second.task_fields), now_wall_ns=NOW + 10**9, now_steady=1., readiness=ready)
        self.assertTrue(admission.accepted)
        self.assertTrue(c.record_admission(admission))
        self.assertGreater(second.reservation.generation, first.reservation.generation)


if __name__ == '__main__':
    unittest.main()
