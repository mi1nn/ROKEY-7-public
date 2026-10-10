"""Mission core: mode gate, operator command gate and displayed phase (scope Q-12, 2026-10-10) (E2)."""
from dataclasses import replace
import itertools
import unittest

from rescue_contracts import SiteEvent
from rescue_coordination.planning import Allocator, CoordinationError
from rescue_coordination.search import SearchCoordinator
from rescue_coordination import mission
from test_coordination_search import BOOT, EPOCH, NOW, policy, receipt, uid

SCOUT, SUPPORT = 'unit_1', 'unit_2'
TARGET, OTHER = uid(500), uid(501)


def sign(n=1, **changes):
    values = dict(event_id=uid(300 + n), camera_id='cam_a', kind='DISASTER_SIGN', observed_at_ns=NOW - 10**9,
                  evidence_id=uid(400 + n), evidence_sha256='a' * 64, confidence=.9,
                  clock_domain_id='site-clock', stamp_semantics='CAPTURE', clock_state='SYNCED',
                  clock_error_bound_ns=5_000_000)
    return SiteEvent(**(values | changes))


class MissionTestCase(unittest.TestCase):
    def setUp(self):
        self.ids = itertools.count(1000)
        self.search = SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=(SCOUT, SUPPORT)), policy(),
                                        new_id=lambda: uid(next(self.ids)))
        self.m = mission.MissionCoordinator(self.search)

    def command(self, kind, **fields):
        return mission.OperatorCommand(command_id=uid(next(self.ids)), kind=kind, operator_id='admin',
                                       requested_at_ns=NOW, **fields)

    def send(self, kind, **fields):
        return self.m.handle_command(self.command(kind, **fields))

    def reason(self, kind, **fields):
        result = self.send(kind, **fields)
        self.assertEqual(result.reason == 'ACCEPTED', result.accepted)
        return result.reason

    def disaster(self):
        self.assertEqual('ENTERED_DISASTER', self.m.record_site_event(sign()).reason)

    def found(self, target=TARGET, rev=0, segments=3, route=7):
        self.m.record_target(target, rev, detection_confirmed=True, report_persisted=True)
        self.m.record_route(target, route_revision=route, segment_count=segments)

    def dispatched(self, segments=3, route=7):
        self.disaster()
        self.found(segments=segments, route=route)
        self.assertEqual('ACCEPTED', self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=0))

    def segment(self, index, route=7, completed=True):
        self.assertTrue(self.m.segment_started(TARGET, route_revision=route, segment_index=index))
        self.assertTrue(self.m.segment_finished(TARGET, route_revision=route, segment_index=index,
                                                completed=completed, stop_confirmed=True))

    def arrived(self):
        self.dispatched(segments=2)
        self.reach_end()

    def reach_end(self):
        self.segment(0)
        self.assertEqual('ACCEPTED', self.reason('CONTINUE_SEGMENT', target_id=TARGET, route_revision=7,
                                                 segment_index=1))
        self.segment(1)


class ModeGateTests(MissionTestCase):
    def test_scout_gets_no_search_area_before_disaster(self):
        gate = self.m.plan(SCOUT, robot_boot_id=BOOT[SCOUT], ready=True, now_ns=NOW)
        self.assertEqual(('MODE_NOT_DISASTER', None), (gate.reason, gate.decision))
        self.assertEqual({}, self.search.allocator.pending)
        self.assertEqual(('NORMAL', 'IDLE'), (self.m.mode, self.m.phase()))

    def test_valid_disaster_sign_enters_disaster_and_releases_the_scout(self):
        event = sign()
        result = self.m.record_site_event(event)
        self.assertEqual((event.event_id, 'ENTERED_DISASTER', 'DISASTER'), (result.event_id, result.reason, result.mode))
        self.assertEqual((event.event_id, event.observed_at_ns),
                         (self.m.trigger_event_id, self.m.trigger_observed_at_ns))
        gate = self.m.plan(SCOUT, robot_boot_id=BOOT[SCOUT], ready=True, now_ns=NOW)
        self.assertEqual(('PASSED', 'RESERVED', 'SEARCH_AREA'),
                         (gate.reason, gate.decision.action, gate.decision.reservation.task_kind))

    def test_support_plan_is_not_gated_by_mode(self):
        gate = self.m.plan(SUPPORT, robot_boot_id=BOOT[SUPPORT], ready=True, now_ns=NOW)
        self.assertEqual(('PASSED', 'HOLD'), (gate.reason, gate.decision.reservation.task_kind))

    def test_same_event_id_is_idempotent_and_conflicting_content_is_rejected(self):
        first = self.m.record_site_event(sign())
        self.assertEqual(first, self.m.record_site_event(sign()))
        self.assertEqual('DISASTER', self.m.mode)
        with self.assertRaises(CoordinationError) as caught:
            self.m.record_site_event(sign(confidence=.5))
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)

    def test_other_kinds_and_unusable_time_basis_are_rejected_with_reasons(self):
        cases = {'NOT_DISASTER_SIGN': [sign(2, kind='EXIT_PASS', camera_id='cam_b')],
                 'TIME_BASIS_UNUSABLE': [sign(3, stamp_semantics='UNKNOWN'), sign(4, clock_state='UNKNOWN'),
                                         sign(5, clock_error_bound_ns=None)]}
        for reason, events in cases.items():
            for event in events:
                with self.subTest(event=event):
                    result = self.m.record_site_event(event)
                    self.assertEqual((reason, 'NORMAL'), (result.reason, result.mode))
                    self.assertIn(reason, mission.MODE_REJECT_REASONS)
        self.assertIsNone(self.m.trigger_event_id)
        # A rejected event_id is remembered too: its replay gives the same answer.
        self.assertEqual('TIME_BASIS_UNUSABLE', self.m.record_site_event(sign(3, stamp_semantics='UNKNOWN')).reason)

    def test_later_sign_keeps_the_first_trigger(self):
        self.disaster()
        self.assertEqual('ALREADY_DISASTER', self.m.record_site_event(sign(2)).reason)
        self.assertEqual(sign().event_id, self.m.trigger_event_id)

    def test_non_site_event_is_invalid_input(self):
        with self.assertRaises(CoordinationError) as caught:
            self.m.record_site_event(dict(kind='DISASTER_SIGN'))
        self.assertEqual('INVALID_INPUT', caught.exception.code)


class OperatorCommandTests(MissionTestCase):
    def test_every_command_needs_disaster_mode(self):
        self.found()
        for kind, fields in (('DISPATCH_SUPPORT', dict(target_id=TARGET, target_revision=0)),
                             ('CONTINUE_SEGMENT', dict(target_id=TARGET, route_revision=7, segment_index=1)),
                             ('REQUEST_RECHECK', dict(target_id=TARGET)), ('RETURN_HOME', {})):
            with self.subTest(kind=kind):
                self.assertEqual('MODE_NOT_DISASTER', self.reason(kind, **fields))

    def test_dispatch_needs_a_confirmed_persisted_current_target_with_a_route(self):
        self.disaster()
        self.assertEqual('UNKNOWN_TARGET', self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=0))
        self.m.record_target(TARGET, 0, detection_confirmed=True, report_persisted=False)
        self.assertEqual('TARGET_NOT_CONFIRMED', self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=0))
        self.m.record_target(TARGET, 1, detection_confirmed=True, report_persisted=True)
        self.assertEqual('TARGET_REVISION_STALE',
                         self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=0))
        self.assertEqual('NO_ROUTE', self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=1))
        self.m.record_route(TARGET, route_revision=7, segment_count=3)
        self.assertEqual('ACCEPTED', self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=1))
        self.assertEqual((TARGET, 7, 0, 3, 'APPROVED'),
                         (self.m.dispatch_target, self.m.route_revision, self.m.segment_index,
                          self.m.segment_count, self.m.segment_phase))

    def test_only_one_dispatch(self):
        self.dispatched()
        self.found(OTHER)
        self.assertEqual('ALREADY_DISPATCHED', self.reason('DISPATCH_SUPPORT', target_id=OTHER, target_revision=0))

    def test_continue_only_when_awaiting_approval_and_never_automatic(self):
        self.dispatched()
        cont = dict(target_id=TARGET, route_revision=7, segment_index=1)
        self.assertEqual('NOT_AWAITING_APPROVAL', self.reason('CONTINUE_SEGMENT', **cont))  # APPROVED
        self.assertTrue(self.m.segment_started(TARGET, route_revision=7, segment_index=0))
        self.assertEqual('NOT_AWAITING_APPROVAL', self.reason('CONTINUE_SEGMENT', **cont))  # MOVING
        self.assertTrue(self.m.segment_finished(TARGET, route_revision=7, segment_index=0, completed=True,
                                                stop_confirmed=True))
        self.assertEqual(('AWAITING_APPROVAL', 1), (self.m.segment_phase, self.m.segment_index))
        self.assertEqual('SEGMENT_INDEX_MISMATCH', self.reason('CONTINUE_SEGMENT', **(cont | dict(segment_index=2))))
        self.assertEqual('ROUTE_REVISION_STALE', self.reason('CONTINUE_SEGMENT', **(cont | dict(route_revision=6))))
        self.assertEqual('NOT_DISPATCHED_TARGET', self.reason('CONTINUE_SEGMENT', **(cont | dict(target_id=OTHER))))
        self.assertEqual('AWAITING_APPROVAL', self.m.segment_phase)  # rejections change nothing
        self.assertEqual('ACCEPTED', self.reason('CONTINUE_SEGMENT', **cont))
        self.assertEqual(('APPROVED', 1), (self.m.segment_phase, self.m.segment_index))

    def test_segment_end_needs_a_confirmed_stop_and_failure_waits_for_approval(self):
        self.dispatched()
        self.assertFalse(self.m.segment_started(TARGET, route_revision=7, segment_index=1))  # not the approved one
        self.assertTrue(self.m.segment_started(TARGET, route_revision=7, segment_index=0))
        self.assertFalse(self.m.segment_finished(TARGET, route_revision=7, segment_index=0, completed=True,
                                                 stop_confirmed=False))
        self.assertEqual('MOVING', self.m.segment_phase)
        self.assertTrue(self.m.segment_finished(TARGET, route_revision=7, segment_index=0, completed=False,
                                                stop_confirmed=True))
        self.assertEqual(('AWAITING_APPROVAL', 0), (self.m.segment_phase, self.m.segment_index))
        self.assertEqual('ACCEPTED', self.reason('CONTINUE_SEGMENT', target_id=TARGET, route_revision=7,
                                                 segment_index=0))

    def test_last_segment_arrives(self):
        self.arrived()
        self.assertEqual(('ARRIVED', 2), (self.m.segment_phase, self.m.segment_index))
        self.assertEqual('NOT_AWAITING_APPROVAL', self.reason('CONTINUE_SEGMENT', target_id=TARGET,
                                                              route_revision=7, segment_index=2))

    def test_route_replacement_only_while_stopped_and_restarts_the_count(self):
        self.dispatched()
        self.assertTrue(self.m.segment_started(TARGET, route_revision=7, segment_index=0))
        with self.assertRaises(CoordinationError):
            self.m.record_route(TARGET, route_revision=8, segment_count=2)
        self.assertTrue(self.m.segment_finished(TARGET, route_revision=7, segment_index=0, completed=True,
                                                stop_confirmed=True))
        self.assertTrue(self.m.record_route(TARGET, route_revision=8, segment_count=2))
        self.assertEqual((8, 0, 2, 'AWAITING_APPROVAL'),
                         (self.m.route_revision, self.m.segment_index, self.m.segment_count, self.m.segment_phase))
        self.assertEqual('ROUTE_REVISION_STALE', self.reason('CONTINUE_SEGMENT', target_id=TARGET,
                                                             route_revision=7, segment_index=0))
        self.assertEqual('ACCEPTED', self.reason('CONTINUE_SEGMENT', target_id=TARGET, route_revision=8,
                                                 segment_index=0))

    def test_route_and_target_revisions_do_not_go_back(self):
        self.found()
        self.assertFalse(self.m.record_route(TARGET, route_revision=7, segment_count=3))  # same content
        for rev, count, code in ((6, 3, 'INVALID_INPUT'), (7, 4, 'ID_CONTENT_CONFLICT')):
            with self.subTest(rev=rev), self.assertRaises(CoordinationError) as caught:
                self.m.record_route(TARGET, route_revision=rev, segment_count=count)
            self.assertEqual(code, caught.exception.code)
        self.assertFalse(self.m.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True))
        with self.assertRaises(CoordinationError) as caught:
            self.m.record_target(TARGET, 0, detection_confirmed=False, report_persisted=True)
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)
        self.assertTrue(self.m.record_target(TARGET, 2, detection_confirmed=True, report_persisted=True))
        self.assertFalse(self.m.record_target(TARGET, 1, detection_confirmed=True, report_persisted=True))
        for count in (0, -1, 1.0, True):
            with self.subTest(count=count), self.assertRaises(CoordinationError):
                self.m.record_route(OTHER, route_revision=1, segment_count=count)

    def test_recheck_only_after_arrival_and_once_at_a_time(self):
        self.dispatched(segments=2)
        self.assertEqual('NOT_ARRIVED', self.reason('REQUEST_RECHECK', target_id=TARGET))
        self.reach_end()
        self.assertEqual('NOT_DISPATCHED_TARGET', self.reason('REQUEST_RECHECK', target_id=OTHER))
        self.assertEqual('ACCEPTED', self.reason('REQUEST_RECHECK', target_id=TARGET))
        self.assertEqual('RECHECK_ALREADY_REQUESTED', self.reason('REQUEST_RECHECK', target_id=TARGET))
        self.m.record_rescue_verdict(TARGET, 'RECHECK_NEEDED')
        self.assertEqual('ACCEPTED', self.reason('REQUEST_RECHECK', target_id=TARGET))

    def test_return_home_only_after_a_verdict(self):
        self.arrived()
        self.assertEqual('NO_VERDICT', self.reason('RETURN_HOME'))
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        self.assertEqual('NO_VERDICT', self.reason('RETURN_HOME'))
        self.m.record_rescue_verdict(TARGET, 'RESCUE_CONFIRMED')
        self.assertEqual('ACCEPTED', self.reason('RETURN_HOME'))
        self.assertEqual('ALREADY_RETURNING', self.reason('RETURN_HOME'))

    def test_verdict_needs_a_pending_recheck_and_a_rescue_state(self):
        self.arrived()
        with self.assertRaises(CoordinationError):
            self.m.record_rescue_verdict(TARGET, 'RESCUE_CONFIRMED')
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        for bad in ('NOT_CHECKED', 'DONE', None):
            with self.subTest(state=bad), self.assertRaises(CoordinationError):
                self.m.record_rescue_verdict(TARGET, bad)

    def test_command_id_replay_is_idempotent_and_conflict_is_rejected(self):
        self.dispatched()
        command = self.command('CONTINUE_SEGMENT', target_id=TARGET, route_revision=7, segment_index=1)
        first = self.m.handle_command(command)
        self.assertEqual('NOT_AWAITING_APPROVAL', first.reason)
        self.segment(0)
        self.assertEqual(first, self.m.handle_command(command))  # replay, not re-judged
        self.assertEqual('AWAITING_APPROVAL', self.m.segment_phase)
        with self.assertRaises(CoordinationError) as caught:
            self.m.handle_command(replace(command, segment_index=2))
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)

    def test_malformed_commands_are_invalid_input(self):
        for kind, fields in (('PAUSE', {}), ('DISPATCH_SUPPORT', dict(target_id=TARGET)),
                             ('RETURN_HOME', dict(target_id=TARGET)),
                             ('CONTINUE_SEGMENT', dict(target_id=TARGET, route_revision=7, segment_index=-1)),
                             ('REQUEST_RECHECK', dict(target_id='not-a-uuid'))):
            with self.subTest(kind=kind, fields=fields), self.assertRaises(CoordinationError) as caught:
                self.command(kind, **fields)
            self.assertEqual('INVALID_INPUT', caught.exception.code)
        with self.assertRaises(CoordinationError):
            mission.OperatorCommand(command_id=uid(1), kind='RETURN_HOME', operator_id='', requested_at_ns=NOW)

    def test_reasons_are_closed(self):
        self.assertLessEqual(mission.MODE_REJECT_REASONS, mission.MODE_EVENT_REASONS)
        with self.assertRaises(CoordinationError):
            mission.CommandResult(uid(1), True, 'NOT_AWAITING_APPROVAL')
        with self.assertRaises(CoordinationError):
            mission.CommandResult(uid(1), False, 'because')


class MissionPhaseTests(MissionTestCase):
    def test_phase_follows_the_scenario(self):
        seen = [self.m.phase()]
        self.disaster()
        seen.append(self.m.phase())
        gate = self.m.plan(SCOUT, robot_boot_id=BOOT[SCOUT], ready=True, now_ns=NOW)
        self.assertTrue(self.search.record_admission(receipt(gate.decision.reservation)))
        seen.append(self.m.phase())
        self.found(segments=2)
        seen.append(self.m.phase())
        self.reason('DISPATCH_SUPPORT', target_id=TARGET, target_revision=0)
        seen.append(self.m.phase())
        self.m.segment_started(TARGET, route_revision=7, segment_index=0)
        seen.append(self.m.phase())
        self.m.segment_finished(TARGET, route_revision=7, segment_index=0, completed=True, stop_confirmed=True)
        seen.append(self.m.phase())
        self.reason('CONTINUE_SEGMENT', target_id=TARGET, route_revision=7, segment_index=1)
        seen.append(self.m.phase())
        self.segment(1)
        seen.append(self.m.phase())
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        seen.append(self.m.phase())
        self.m.record_rescue_verdict(TARGET, 'RESCUE_CONFIRMED')
        seen.append(self.m.phase())
        self.reason('RETURN_HOME')
        seen.append(self.m.phase())
        self.m.record_returned(SCOUT)
        seen.append(self.m.phase())
        self.m.record_returned(SUPPORT)
        seen.append(self.m.phase())
        self.assertEqual(['IDLE', 'DISASTER', 'SEARCHING', 'TARGET_FOUND', 'DISPATCH_APPROVED', 'SEGMENT_MOVING',
                          'AWAITING_CONTINUE', 'SEGMENT_MOVING', 'ARRIVED', 'RECHECKING', 'RESCUE_CONFIRMED',
                          'RETURNING', 'RETURNING', 'FINISHED'], seen)
        self.assertLessEqual(set(seen), mission.MISSION_PHASES)

    def test_recheck_needed_is_shown_until_the_next_recheck(self):
        self.arrived()
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        self.m.record_rescue_verdict(TARGET, 'RECHECK_NEEDED')
        self.assertEqual('RECHECK_NEEDED', self.m.phase())
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        self.assertEqual('RECHECKING', self.m.phase())

    def test_unconfirmed_target_is_not_found_yet(self):
        self.disaster()
        self.m.record_target(TARGET, 0, detection_confirmed=True, report_persisted=False)
        self.assertEqual('DISASTER', self.m.phase())

    def test_returned_needs_a_policy_robot_and_a_return(self):
        with self.assertRaises(CoordinationError):
            self.m.record_returned(SCOUT)  # no RETURN_HOME accepted yet
        self.arrived()
        self.reason('REQUEST_RECHECK', target_id=TARGET)
        self.m.record_rescue_verdict(TARGET, 'RESCUE_CONFIRMED')
        self.reason('RETURN_HOME')
        with self.assertRaises(CoordinationError):
            self.m.record_returned('unit_9')

    def test_view_carries_what_the_screen_shows(self):
        self.dispatched()
        view = self.m.view()
        self.assertEqual(('DISASTER', 'DISPATCH_APPROVED', sign().event_id, TARGET, 7, 0, 3, 'APPROVED'),
                         (view.mode, view.phase, view.trigger_event_id, view.target_id, view.route_revision,
                          view.segment_index, view.segment_count, view.segment_phase))


if __name__ == '__main__':
    unittest.main()
