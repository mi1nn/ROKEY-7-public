"""Exit count increase -> SCOUT re-observation task (BRD Part I 3.5.3) (E2)."""
import itertools
import unittest

from rescue_coordination.planning import Allocator, CoordinationError
from rescue_coordination.search import DECISION_REASONS, RECHECK_REASONS, SearchCoordinator
from test_coordination_search import BOOT, EPOCH, NOW, candidate, policy, receipt, task_of, uid

SCOUT, SUPPORT = 'unit_1', 'unit_2'
EVENT, OTHER = uid(500), uid(501)


class RecheckTests(unittest.TestCase):
    def setUp(self):
        self.ids = itertools.count(1000)
        self.c = SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=(SCOUT, SUPPORT)), policy(),
                                   new_id=lambda: uid(next(self.ids)))

    def run_task(self, decision, outcome='COMPLETED'):
        item = decision.reservation
        self.assertTrue(self.c.record_admission(receipt(item)))
        self.assertTrue(self.c.finish(item, outcome=outcome, stop_confirmed=True))

    def approach_done(self, event=EVENT, outcome='COMPLETED'):
        support = self.c.plan(SUPPORT, robot_boot_id=BOOT[SUPPORT], ready=True, now_ns=NOW)
        self.run_task(support)  # HOLD completes, so the approach can be reserved
        decision = self.c.request_approach(event, 0, projection_status='VALID', candidate=candidate(SUPPORT),
                                           robot_boot_id=BOOT[SUPPORT], now_ns=NOW)
        self.assertEqual('RESERVED', decision.action)
        self.run_task(decision, outcome)

    def recheck(self, count, event=EVENT, status='VALID', cand='default'):
        return self.c.request_recheck(event, 0, exit_pass_count=count, projection_status=status,
                                      candidate=candidate(SCOUT) if cand == 'default' else cand,
                                      robot_boot_id=BOOT[SCOUT], now_ns=NOW)

    def test_exit_after_completed_approach_reserves_a_scout_reobservation(self):
        self.approach_done()
        decision = self.recheck(1)
        self.assertEqual(('RESERVED', 'REOBSERVE_TARGET', SCOUT, EVENT),
                         (decision.action, decision.reservation.task_kind, decision.reservation.robot_id,
                          decision.reservation.target_id))
        task = task_of(decision.task_fields)
        self.assertEqual(('REOBSERVE_TARGET', EVENT), (task.kind, task.target_id))
        self.assertIsNotNone(task.goal_pose)

    def test_no_exit_yet_or_no_completed_approach_waits(self):
        self.assertEqual(('WAIT', 'APPROACH_NOT_COMPLETED'), self.reason(self.recheck(0)))
        self.approach_done(OTHER)
        self.assertEqual(('WAIT', 'NO_NEW_EXIT_PASS'), self.reason(self.recheck(0, event=OTHER)))
        self.approach_done(outcome='UNREACHABLE')
        self.assertEqual(('WAIT', 'APPROACH_NOT_COMPLETED'), self.reason(self.recheck(1)))

    def test_one_completed_recheck_per_exit_increase(self):
        self.approach_done()
        first = self.recheck(1)
        self.assertEqual(('KEEP', 'EVENT_ALREADY_HAS_RECHECK'), self.reason(self.recheck(1)))
        self.run_task(first)
        self.assertEqual(('WAIT', 'NO_NEW_EXIT_PASS'), self.reason(self.recheck(1)))
        self.assertEqual('RESERVED', self.recheck(2).action)

    def test_failed_recheck_can_be_retried_without_a_new_exit(self):
        self.approach_done()
        self.run_task(self.recheck(1), outcome='CANCELED')
        self.assertEqual('RESERVED', self.recheck(1).action)

    def test_searching_scout_must_be_released_first(self):
        search = self.c.plan(SCOUT, robot_boot_id=BOOT[SCOUT], ready=True, now_ns=NOW)
        self.assertEqual('SEARCH_AREA', search.reservation.task_kind)
        self.approach_done()
        decision = self.recheck(1)
        self.assertEqual(('RELEASE_REQUIRED', 'SCOUT_SEARCHING'), self.reason(decision))
        self.assertIs(search.reservation, decision.reservation)

    def test_scout_rechecking_another_event_is_busy(self):
        self.approach_done()
        self.approach_done(OTHER)
        self.assertEqual('RESERVED', self.recheck(1).action)
        self.assertEqual(('WAIT', 'SCOUT_BUSY'), self.reason(self.recheck(1, event=OTHER)))

    def test_candidate_and_projection_gates(self):
        self.approach_done()
        self.assertEqual(('WAIT', 'PROJECTION_NOT_VALID'), self.reason(self.recheck(1, status='STALE')))
        for bad in (None, candidate(SUPPORT)):
            with self.subTest(candidate=bad):
                self.assertEqual(('WAIT', 'NO_SCOUT_CANDIDATE'), self.reason(self.recheck(1, cand=bad)))

    def test_exit_count_must_not_regress_or_be_invalid(self):
        self.approach_done()
        self.recheck(2)
        with self.assertRaises(CoordinationError) as caught:
            self.recheck(1, event=OTHER)
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        for bad in (-1, 1.0, None, True):
            with self.subTest(count=bad), self.assertRaises(CoordinationError):
                self.recheck(bad, event=OTHER)

    def test_recheck_reasons_are_closed_and_distinct_from_search_reasons(self):
        self.assertFalse(RECHECK_REASONS & DECISION_REASONS)
        self.approach_done()
        for decision in (self.recheck(0), self.recheck(1, status='STALE')):
            self.assertIn(decision.reason, RECHECK_REASONS | DECISION_REASONS)

    @staticmethod
    def reason(decision):
        return decision.action, decision.reason


if __name__ == '__main__':
    unittest.main()
