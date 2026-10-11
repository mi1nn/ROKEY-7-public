"""Coordinator core: passage-free segments (AST88-01) and the lease message TTL (AST88-02) (E2).

작전 2026-10-11 (OPS-60): a MOVE_SEGMENT is issued only for a segment the deployment marks
passage-free (field-checked value, no default); otherwise "passage assessment required, hold"
(Q-25 default). CommandLease.valid_until_ns = issued_at_ns + lease_message_ttl.
"""
import unittest

from rescue_contracts import RescueError
from test_coordination_ros_core import (ACCEPT, SEGMENTS, SUPPORT, TTL, admission_msg, frame, location, make_core,
                                        receipt, request, site_msg, status_msg)
from test_coordination_search import NOW, uid
from types import SimpleNamespace as NS


class PassageTests(unittest.TestCase):
    def ready(self, passage):
        self.core = make_core(segment_passage_free=passage)
        self.core.on_site_event(site_msg())
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.core.on_status(status_msg(SUPPORT))
        return self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=self.target),
                                    now_ns=NOW)

    def held(self, reply):
        self.assertEqual((True, ''), (reply['accepted'], reply['created_task_id']))
        self.assertEqual([], self.core.tick(NOW))
        self.assertEqual('PASSAGE_ASSESSMENT_REQUIRED', self.core.issue_block)
        self.assertEqual('PASSAGE_ASSESSMENT_REQUIRED', self.core.system_state_change(NOW)['reason'])

    def test_segment_not_marked_passage_free_is_held(self):
        self.held(self.ready((False, True)))

    def test_no_passage_values_hold_every_segment(self):
        self.held(self.ready(()))

    def test_length_mismatch_holds_every_segment(self):
        self.held(self.ready((True,) * (len(SEGMENTS) + 1)))
        self.held(self.ready((True,) * (len(SEGMENTS) - 1)))

    def test_only_the_marked_segments_are_issued(self):
        reply = self.ready((True, False))
        (first,) = self.core.tick(NOW)
        self.assertEqual(reply['created_task_id'], first['task_id'])
        self.assertIsNone(self.core.issue_block)
        self.assertEqual('', self.core.system_state_change(NOW)['reason'])
        self.core.on_admission(admission_msg(first))
        self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        reply = self.core.on_command(request('CONTINUE_SEGMENT', command_id=uid(702), target_id=self.target,
                                             route_revision=7, segment_index=1), now_ns=NOW)
        self.held(reply)

    def test_passage_values_must_be_booleans(self):
        for passage in ((1, True), ('true', True), (None, True)):
            with self.subTest(passage=passage), self.assertRaises(RescueError):
                make_core(segment_passage_free=passage)


class LeaseTtlTests(unittest.TestCase):
    def test_valid_until_is_issued_plus_ttl(self):
        core = make_core()
        core.on_site_event(site_msg())
        core.on_status(status_msg())
        self.assertEqual('ACCEPTED', core.on_command(request('START_SEARCH', command_id=uid(690)),
                                                     now_ns=NOW)['request_state'])
        (fields,) = core.tick(NOW)
        (lease,) = core.leases_due(NOW)
        self.assertEqual((NOW, NOW + TTL), (lease['issued_at_ns'], lease['valid_until_ns']))

    def test_ttl_must_be_positive(self):
        for ttl in (0, -1, 1.5):
            with self.subTest(ttl=ttl), self.assertRaises(RescueError):
                make_core(lease_message_ttl_ns=ttl)


class StaleBlockTests(unittest.TestCase):
    """작전 OPS-70-01: SystemState.reason shows only a hold that still stands, not the last one ever seen."""

    def test_admission_timeout_clears_once_the_task_is_released_and_reissued(self):
        core = make_core()
        core.on_site_event(site_msg())
        core.on_status(status_msg())
        core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        (fields,) = core.tick(NOW)
        late = NOW + ACCEPT + 1
        core.on_status(status_msg(observed_at_ns=late, motion_state='MOVING', active_task_id=fields['task_id']))
        self.assertEqual([], core.tick(late))
        self.assertEqual('ADMISSION_TIMEOUT', core.system_state_change(late)['reason'])
        core.on_status(status_msg(observed_at_ns=late + 1))  # stopped, no active task: released
        (again,) = core.tick(late + 1)
        self.assertEqual('', core.system_state_change(late + 1)['reason'])

    def test_alignment_block_clears_once_the_recheck_is_issued(self):
        core = make_core()
        core.on_site_event(site_msg())
        target = core.on_target_location(location(1, frame_ref=frame(alignment_status='DEGRADED')), now_ns=NOW)
        core.on_command(request('REQUEST_RECHECK', command_id=uid(701), target_id=target), now_ns=NOW)
        self.assertEqual([], core.tick(NOW))
        self.assertEqual('TARGET_ALIGNMENT_NOT_VALID', core.system_state_change(NOW)['reason'])
        self.assertEqual(target, core.on_target_location(location(2, (4.2, 4.)), now_ns=NOW))  # aligned again
        core.on_status(status_msg())
        (fields,) = core.tick(NOW)
        self.assertEqual(('REOBSERVE_TARGET', ''), (fields['kind'], core.system_state_change(NOW)['reason']))


if __name__ == '__main__':
    unittest.main()
