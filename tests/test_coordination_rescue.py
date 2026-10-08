"""Rescue-completion verdicts: three positive evidences, unknown never confirms (E2)."""
from dataclasses import replace
import itertools
import unittest
from uuid import UUID

from rescue_contracts import RESCUE_STATES, SiteEvent
from rescue_coordination.planning import CoordinationError
from rescue_coordination.rescue import (RESCUE_HOLD_REASONS, SINGLE_TARGET_SCENARIO, ExitLink, LidarCheck,
    RecheckEvidence, RescueBoard, RescuePolicy, TargetBasis)

S = 1_000_000_000
T0 = 1_800_000_000 * S  # approach completed
EXIT_AT = T0 + 5 * S
SEEN_AT = T0 + 20 * S
NOW = T0 + 25 * S
TARGET = str(UUID(int=500))
OTHER = str(UUID(int=501))
MAP = str(UUID(int=5))


def uid(n):
    return str(UUID(int=n))


def policy(**changes):
    values = dict(version='test-only', max_sensor_skew_ns=1 * S, max_recheck_age_ns=30 * S,
                  max_clock_error_sum_ns=100_000_000,
                  min_exit_confidence=0.5, accepted_link_bases=frozenset({'MARKER_ID_MATCH'}),
                  lidar_detects_target_measured=True)
    return RescuePolicy(**(values | changes))


def recheck(**changes):
    values = dict(target_id=TARGET, observation_id=uid(600), robot_id='unit_1', task_id=uid(601), generation=4,
                  task_kind='REOBSERVE_TARGET', task_outcome='COMPLETED', stop_confirmed=True,
                  observed_at_ns=SEEN_AT, map_session_id=MAP, map_revision=3, target_in_view=True,
                  depth_valid_at_target=True, person_detected_at_target=False,
                  clock_domain_id='site-clock', clock_error_bound_ns=10_000_000)
    return RecheckEvidence(**(values | changes))


def lidar(**changes):
    values = dict(target_id=TARGET, robot_id='unit_1', scan_at_ns=SEEN_AT, map_session_id=MAP, map_revision=3,
                  target_in_scan_range=True, obstacle_at_target=False,
                  clock_domain_id='site-clock', clock_error_bound_ns=10_000_000)
    return LidarCheck(**(values | changes))


def site_event(event=700, kind='EXIT_PASS', at=EXIT_AT, confidence=0.9, **time_basis):
    values = dict(clock_domain_id='site-clock', stamp_semantics='CAPTURE', clock_state='SYNCED',
                  clock_error_bound_ns=10_000_000)
    return SiteEvent(uid(event), 'cam_exit', kind, at, uid(event + 1), 'a' * 64, confidence,
                     **(values | time_basis))


def exit_link(target=TARGET, basis='MARKER_ID_MATCH', at=EXIT_AT, confidence=0.9, event=700, **time_basis):
    return ExitLink(site_event(event, at=at, confidence=confidence, **time_basis), target, basis)


def basis(**changes):
    values = dict(target_id=TARGET, scout_robot_id='unit_1', map_session_id=MAP, map_revision=3,
                  projection_valid=True, approach_completed_at_ns=T0, open_target_count=1,
                  approach_clock_domain_id='site-clock', approach_clock_error_bound_ns=10_000_000,
                  exit_pass_count=1)
    return TargetBasis(**(values | changes))


class RescueBoardTests(unittest.TestCase):
    def setUp(self):
        self.ids = itertools.count(9000)

    def board(self, **changes):
        return RescueBoard(policy(**changes), new_id=lambda: uid(next(self.ids)))

    def full(self, board, check=None, scan=None, link=None):
        board.record_recheck(check or recheck())
        board.record_lidar(scan or lidar())
        board.link_exit(link or exit_link())
        return board

    def test_all_three_positive_evidences_confirm(self):
        verdict = self.full(self.board()).evaluate(basis(), now_ns=NOW)
        self.assertEqual(('RESCUE_CONFIRMED', frozenset(), 'MARKER_ID_MATCH'),
                         (verdict.state, verdict.reasons, verdict.link_basis))
        self.assertIn(verdict.state, RESCUE_STATES)

    def test_no_recheck_is_not_checked_even_with_exit_pass(self):
        b = self.board()
        b.link_exit(exit_link())
        b.record_lidar(lidar())
        self.assertEqual(('NOT_CHECKED', frozenset()), (lambda v: (v.state, v.reasons))(b.evaluate(basis(), now_ns=NOW)))

    def test_each_missing_or_negative_input_holds_with_its_reason(self):
        cases = {
            'TASK_NOT_COMPLETED': [dict(check=recheck(task_outcome='FAILED')), dict(check=recheck(stop_confirmed=False)),
                                   dict(check=recheck(task_kind='SEARCH_AREA'))],
            'RECHECK_ROBOT_NOT_SCOUT': [dict(check=recheck(robot_id='unit_2')), dict(scan=lidar(robot_id='unit_2'))],
            'TARGET_NOT_OBSERVED': [dict(check=recheck(target_in_view=False)),
                                    dict(check=recheck(depth_valid_at_target=False))],
            'PERSON_PRESENT_OR_UNKNOWN': [dict(check=recheck(person_detected_at_target=True))],
            'OBSTACLE_PRESENT_OR_UNKNOWN': [dict(scan=lidar(obstacle_at_target=True)),
                                            dict(scan=lidar(target_in_scan_range=False))],
            'EVIDENCE_STALE': [dict(scan=lidar(scan_at_ns=SEEN_AT + 2 * S))],
            'MAP_BASIS_CHANGED': [dict(check=recheck(map_revision=4)), dict(scan=lidar(map_session_id=uid(6)))],
            'EXIT_NOT_LINKED': [dict(link=exit_link(confidence=0.4)), dict(link=exit_link(basis='GUESS'))],
            'ORDER_VIOLATION': [dict(link=exit_link(at=SEEN_AT)), dict(check=recheck(observed_at_ns=T0),
                                                                      scan=lidar(scan_at_ns=T0))],
        }
        for reason, variants in cases.items():
            for variant in variants:
                with self.subTest(reason=reason, variant=variant):
                    verdict = self.full(self.board(), **variant).evaluate(basis(), now_ns=NOW)
                    self.assertEqual('RECHECK_NEEDED', verdict.state)
                    self.assertIn(reason, verdict.reasons)
                    self.assertTrue(verdict.reasons <= RESCUE_HOLD_REASONS)

    def test_unknown_values_never_confirm(self):
        for check, scan, reason in (
                (recheck(target_in_view=None), lidar(), 'TARGET_NOT_OBSERVED'),
                (recheck(depth_valid_at_target=None), lidar(), 'TARGET_NOT_OBSERVED'),
                (recheck(person_detected_at_target=None), lidar(), 'PERSON_PRESENT_OR_UNKNOWN'),
                (recheck(), lidar(obstacle_at_target=None), 'OBSTACLE_PRESENT_OR_UNKNOWN'),
                (recheck(), lidar(target_in_scan_range=None), 'OBSTACLE_PRESENT_OR_UNKNOWN')):
            verdict = self.full(self.board(), check=check, scan=scan).evaluate(basis(), now_ns=NOW)
            self.assertEqual(('RECHECK_NEEDED', True), (verdict.state, reason in verdict.reasons))
        with self.assertRaises(CoordinationError):
            recheck(person_detected_at_target=0)

    def test_absence_alone_never_confirms_kpi06(self):
        # "No person, no obstacle" but the point was never in view: not a rescue.
        verdict = self.full(self.board(), check=recheck(target_in_view=False, depth_valid_at_target=None)
                            ).evaluate(basis(), now_ns=NOW)
        self.assertEqual('RECHECK_NEEDED', verdict.state)
        self.assertIn('TARGET_NOT_OBSERVED', verdict.reasons)
        # No LiDAR judgement at all, or LiDAR never shown to see the target object (TR-12).
        b = self.board()
        b.record_recheck(recheck())
        b.link_exit(exit_link())
        self.assertIn('OBSTACLE_PRESENT_OR_UNKNOWN', b.evaluate(basis(), now_ns=NOW).reasons)
        unmeasured = self.full(self.board(lidar_detects_target_measured=False)).evaluate(basis(), now_ns=NOW)
        self.assertEqual(frozenset({'OBSTACLE_PRESENT_OR_UNKNOWN'}), unmeasured.reasons)
        # No exit pass linked.
        b = self.board()
        b.record_recheck(recheck())
        b.record_lidar(lidar())
        self.assertEqual(frozenset({'EXIT_NOT_LINKED'}), b.evaluate(basis(), now_ns=NOW).reasons)

    def test_stale_recheck_and_unfinished_approach_hold(self):
        b = self.full(self.board())
        self.assertIn('EVIDENCE_STALE', b.evaluate(basis(), now_ns=SEEN_AT + 31 * S).reasons)
        b = self.full(self.board())
        self.assertIn('ORDER_VIOLATION', b.evaluate(basis(approach_completed_at_ns=None), now_ns=NOW).reasons)
        b = self.full(self.board())
        self.assertIn('MAP_BASIS_CHANGED', b.evaluate(basis(projection_valid=False), now_ns=NOW).reasons)

    def test_future_evidence_or_regressed_clock_is_rejected(self):
        b = self.full(self.board())
        with self.assertRaises(CoordinationError) as caught:
            b.evaluate(basis(), now_ns=SEEN_AT - 1)
        self.assertEqual('CLOCK_UNCERTAIN', caught.exception.code)
        b.evaluate(basis(), now_ns=NOW)
        with self.assertRaises(CoordinationError):
            b.evaluate(basis(), now_ns=NOW - 1)

    def test_contrary_evidence_demotes_with_history_and_misjudgement_mark(self):
        b = self.full(self.board())
        confirmed = b.evaluate(basis(), now_ns=NOW)
        # Ageing or a map revision alone does not take a confirmed rescue back.
        self.assertIs(confirmed, b.evaluate(basis(map_revision=9), now_ns=NOW + 60 * S))
        self.assertTrue(b.record_recheck(recheck(observation_id=uid(610), observed_at_ns=NOW + 61 * S,
                                                 person_detected_at_target=True)))
        demoted = b.evaluate(basis(), now_ns=NOW + 62 * S)
        self.assertEqual(('RECHECK_NEEDED', True, confirmed.verdict_id, confirmed.evaluated_at_ns),
                         (demoted.state, demoted.demoted_from_confirmed, demoted.previous_verdict_id,
                          demoted.previous_confirmed_at_ns))
        self.assertIn('PERSON_PRESENT_OR_UNKNOWN', demoted.reasons)
        self.assertEqual([confirmed, demoted], b.history[TARGET])  # nothing overwritten
        self.assertFalse(confirmed.demoted_from_confirmed)

    def test_older_evidence_cannot_replace_newer(self):
        b = self.board()
        self.assertTrue(b.record_recheck(recheck()))
        self.assertFalse(b.record_recheck(recheck(observation_id=uid(611), observed_at_ns=SEEN_AT - 1,
                                                  person_detected_at_target=True)))
        self.assertFalse(b.record_recheck(recheck()))
        self.assertTrue(b.record_lidar(lidar()))
        self.assertFalse(b.record_lidar(lidar(scan_at_ns=SEEN_AT - 1, obstacle_at_target=True)))
        self.assertEqual(False, b.rechecks[TARGET].person_detected_at_target)

    def test_one_exit_pass_links_to_one_event(self):
        b = self.board()
        self.assertTrue(b.link_exit(exit_link()))
        self.assertFalse(b.link_exit(exit_link()))  # identical resend
        for link in (exit_link(target=OTHER), exit_link(event=710)):
            with self.assertRaises(CoordinationError) as caught:
                b.link_exit(link)
            self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)
        with self.assertRaises(CoordinationError):
            ExitLink(site_event(720, kind='DISASTER_SIGN'), TARGET, 'X')

    def test_single_target_link_needs_policy_one_open_event_and_post_approach_exit(self):
        single = exit_link(basis=SINGLE_TARGET_SCENARIO)
        self.assertIn('EXIT_NOT_LINKED', self.full(self.board(), link=single).evaluate(basis(), now_ns=NOW).reasons)
        allowed = dict(accepted_link_bases=frozenset({SINGLE_TARGET_SCENARIO}))
        ok = self.full(self.board(**allowed), link=single).evaluate(basis(), now_ns=NOW)
        self.assertEqual(('RESCUE_CONFIRMED', SINGLE_TARGET_SCENARIO), (ok.state, ok.link_basis))
        two_open = self.full(self.board(**allowed), link=single).evaluate(basis(open_target_count=2), now_ns=NOW)
        self.assertIn('EXIT_NOT_LINKED', two_open.reasons)
        early = exit_link(basis=SINGLE_TARGET_SCENARIO, at=T0)
        self.assertIn('ORDER_VIOLATION', self.full(self.board(**allowed), link=early).evaluate(basis(), now_ns=NOW).reasons)

    def test_exit_without_usable_time_basis_is_withheld_not_ordered(self):
        for time_basis in (dict(stamp_semantics='UNKNOWN'), dict(clock_state='UNKNOWN'),
                           dict(clock_error_bound_ns=None)):
            with self.subTest(time_basis=time_basis):
                # Even an exit stamp that would look "after" the recheck is not used to order.
                link = exit_link(at=SEEN_AT + S, **time_basis)
                self.assertFalse(link.site_event.time_basis_declared)
                verdict = self.full(self.board(), link=link).evaluate(basis(), now_ns=NOW)
                self.assertEqual(('RECHECK_NEEDED', frozenset({'EXIT_TIME_BASIS_UNUSABLE'})),
                                 (verdict.state, verdict.reasons))
        # Ordering that does not involve the exit stamp is still checked.
        late = self.full(self.board(), check=recheck(observed_at_ns=T0), scan=lidar(scan_at_ns=T0),
                         link=exit_link(stamp_semantics='UNKNOWN')).evaluate(basis(), now_ns=NOW)
        self.assertTrue({'EXIT_TIME_BASIS_UNUSABLE', 'ORDER_VIOLATION'} <= late.reasons)

    def test_declared_exit_still_needs_same_domain_and_bounded_error_sum(self):
        both = frozenset({'EXIT_TIME_BASIS_UNUSABLE', 'TIME_BASIS_INCOMPARABLE'})
        for name, kwargs, expected in (
                # The recheck stamp is then also not comparable with the approach time.
                ('other domain', dict(check=recheck(clock_domain_id='robot-clock'),
                                      scan=lidar(clock_domain_id='robot-clock')), both),
                ('recheck bound unknown', dict(check=recheck(clock_error_bound_ns=None)), both),
                ('error sum over cap', dict(link=exit_link(clock_error_bound_ns=95_000_000)),
                 frozenset({'EXIT_TIME_BASIS_UNUSABLE'}))):
            with self.subTest(name):
                verdict = self.full(self.board(), **kwargs).evaluate(basis(), now_ns=NOW)
                self.assertEqual(expected, verdict.reasons)
        # SINGLE also orders the exit against the approach time: same gate there.
        allowed = dict(accepted_link_bases=frozenset({SINGLE_TARGET_SCENARIO}))
        verdict = self.full(self.board(**allowed), link=exit_link(basis=SINGLE_TARGET_SCENARIO)).evaluate(
            basis(approach_clock_domain_id='support-clock'), now_ns=NOW)
        self.assertEqual(both, verdict.reasons)

    def test_robot_side_stamps_need_a_comparable_time_basis(self):
        for name, kwargs, b in (
                ('recheck vs approach domain', {}, basis(approach_clock_domain_id='support-clock')),
                ('approach bound unknown', {}, basis(approach_clock_error_bound_ns=None)),
                ('lidar domain differs', dict(scan=lidar(clock_domain_id='lidar-clock')), basis()),
                ('lidar bound unknown', dict(scan=lidar(clock_error_bound_ns=None)), basis())):
            with self.subTest(name):
                verdict = self.full(self.board(), **kwargs).evaluate(b, now_ns=NOW)
                self.assertEqual('RECHECK_NEEDED', verdict.state)
                self.assertIn('TIME_BASIS_INCOMPARABLE', verdict.reasons)
                self.assertNotIn('ORDER_VIOLATION', verdict.reasons)  # not comparable != wrong order
        # Comparable but within the summed error: a real order problem.
        close = self.full(self.board(), check=recheck(observed_at_ns=T0 + 15_000_000),
                          scan=lidar(scan_at_ns=T0 + 15_000_000), link=exit_link(at=T0 + 5_000_000))
        self.assertIn('ORDER_VIOLATION', close.evaluate(basis(), now_ns=NOW).reasons)
        # Only the recheck is within the error of the approach time; scan and exit are clear.
        only_seen = self.full(self.board(), check=recheck(observed_at_ns=T0 + 15_000_000),
                              scan=lidar(scan_at_ns=T0 + 500_000_000), link=exit_link(at=T0 - 100_000_000))
        self.assertEqual(frozenset({'ORDER_VIOLATION'}), only_seen.evaluate(basis(), now_ns=NOW).reasons)

    def test_after_must_exceed_the_summed_clock_error(self):
        # 20 ms of combined uncertainty: 15 ms "later" is not provably later.
        verdict = self.full(self.board(), link=exit_link(at=SEEN_AT - 15_000_000)).evaluate(basis(), now_ns=NOW)
        self.assertEqual(frozenset({'ORDER_VIOLATION'}), verdict.reasons)
        ok = self.full(self.board(), link=exit_link(at=SEEN_AT - 25_000_000)).evaluate(basis(), now_ns=NOW)
        self.assertEqual('RESCUE_CONFIRMED', ok.state)

    def test_swapped_roles_accept_only_the_scout_recheck(self):
        b = self.full(self.board(), check=recheck(robot_id='unit_2'), scan=lidar(robot_id='unit_2'))
        self.assertEqual('RESCUE_CONFIRMED', b.evaluate(basis(scout_robot_id='unit_2'), now_ns=NOW).state)
        b = self.full(self.board())
        self.assertIn('RECHECK_ROBOT_NOT_SCOUT', b.evaluate(basis(scout_robot_id='unit_2'), now_ns=NOW).reasons)

    def test_policy_and_verdict_closed_sets(self):
        for change in (dict(min_exit_confidence=1), dict(accepted_link_bases={'MARKER_ID_MATCH'}),
                       dict(max_clock_error_sum_ns=-1),
                       dict(lidar_detects_target_measured=None), dict(max_sensor_skew_ns=-1), dict(version='')):
            with self.assertRaises(CoordinationError):
                policy(**change)
        verdict = self.full(self.board()).evaluate(basis(), now_ns=NOW)
        for change in (dict(state='DONE'), dict(reasons=frozenset({'free text'})),
                       dict(state='RECHECK_NEEDED', reasons=frozenset()),
                       dict(state='NOT_CHECKED', reasons=frozenset({'EXIT_NOT_LINKED'}))):
            with self.assertRaises(CoordinationError):
                replace(verdict, **change)


class ExitCountCapTests(unittest.TestCase):
    """Part I 3.5.5: confirmed rescues never exceed the cumulative exit count."""

    def setUp(self):
        self.ids = itertools.count(9000)

    def board(self):
        return RescueBoard(policy(), new_id=lambda: uid(next(self.ids)))

    def two_linked(self):
        b = self.board()
        for target, event, obs in ((TARGET, 700, 600), (OTHER, 710, 610)):
            b.record_recheck(recheck(target_id=target, observation_id=uid(obs)))
            b.record_lidar(lidar(target_id=target))
            b.link_exit(exit_link(target=target, event=event))
        return b

    def test_second_confirmation_waits_for_the_exit_count(self):
        b = self.two_linked()
        self.assertEqual('RESCUE_CONFIRMED', b.evaluate(basis(exit_pass_count=1), now_ns=NOW).state)
        held = b.evaluate(basis(target_id=OTHER, exit_pass_count=1), now_ns=NOW)
        self.assertEqual(('RECHECK_NEEDED', frozenset({'EXIT_COUNT_EXCEEDED'})), (held.state, held.reasons))
        ok = b.evaluate(basis(target_id=OTHER, exit_pass_count=2), now_ns=NOW + 1)
        self.assertEqual('RESCUE_CONFIRMED', ok.state)

    def test_link_without_a_counted_exit_does_not_confirm(self):
        b = self.two_linked()
        verdict = b.evaluate(basis(exit_pass_count=0), now_ns=NOW)
        self.assertEqual(frozenset({'EXIT_COUNT_EXCEEDED'}), verdict.reasons)

    def test_confirmed_never_exceeds_exit_count_over_a_sequence(self):
        b = self.two_linked()
        for step, (target, count) in enumerate(((TARGET, 0), (OTHER, 1), (TARGET, 1), (OTHER, 1), (TARGET, 2))):
            b.evaluate(basis(target_id=target, exit_pass_count=count), now_ns=NOW + step)
            confirmed = sum(b.latest(t) is not None and b.latest(t).state == 'RESCUE_CONFIRMED'
                            for t in (TARGET, OTHER))
            self.assertLessEqual(confirmed, count, step)
        self.assertEqual(2, confirmed)

    def test_rejudging_a_confirmed_event_does_not_count_itself(self):
        b = self.two_linked()
        self.assertEqual('RESCUE_CONFIRMED', b.evaluate(basis(exit_pass_count=1), now_ns=NOW).state)
        b.record_recheck(recheck(observation_id=uid(620), observed_at_ns=SEEN_AT + 1))
        b.record_lidar(lidar(scan_at_ns=SEEN_AT + 1))
        again = b.evaluate(basis(exit_pass_count=1), now_ns=NOW + 1)
        self.assertEqual(('RESCUE_CONFIRMED', False), (again.state, again.demoted_from_confirmed))

    def test_exit_count_must_not_regress_or_be_invalid(self):
        b = self.two_linked()
        b.evaluate(basis(exit_pass_count=2), now_ns=NOW)
        with self.assertRaises(CoordinationError) as caught:
            b.evaluate(basis(target_id=OTHER, exit_pass_count=1), now_ns=NOW)
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        for bad in (-1, 1.0, None, True):
            with self.subTest(bad=bad), self.assertRaises(CoordinationError):
                basis(exit_pass_count=bad)


if __name__ == '__main__':
    unittest.main()
