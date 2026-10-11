"""Coordinator core: passage-free segments (AST88-01) and the lease message TTL (AST88-02) (E2).

작전 2026-10-11 (OPS-60): a MOVE_SEGMENT is issued only for a segment the deployment marks
passage-free (field-checked value, no default); otherwise "passage assessment required, hold"
(Q-25 default). CommandLease.valid_until_ns = issued_at_ns + lease_message_ttl.
"""
import unittest

from rescue_contracts import RescueError
from test_coordination_ros_core import (SEGMENTS, SUPPORT, TTL, admission_msg, location, make_core, receipt, request,
                                        site_msg, status_msg)
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


if __name__ == '__main__':
    unittest.main()
