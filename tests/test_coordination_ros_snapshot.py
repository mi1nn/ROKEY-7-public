"""Coordinator core: TargetSnapshot (IF-070) for the operations screen and DB adapter (E2).

작전 2026-10-11: published on change, one TargetEstimate per target, TRANSIENT_LOCAL in the node.
"""
from types import SimpleNamespace as NS
import unittest

from test_coordination_ros_core import (SUPPORT, admission_msg, location, make_core, receipt, request, site_msg,
                                        status_msg)
from test_coordination_search import NOW, uid


class TargetSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core()

    def estimate(self):
        snapshot = self.core.target_snapshot_change()
        (estimate,) = snapshot['targets']
        return estimate

    def test_published_on_change_only(self):
        first = self.core.target_snapshot_change()
        self.assertEqual([], first['targets'])
        self.assertIsNone(self.core.target_snapshot_change())
        self.core.on_target_location(location(1), now_ns=NOW)
        second = self.core.target_snapshot_change()
        self.assertNotEqual(first['snapshot_id'], second['snapshot_id'])
        self.assertIsNone(self.core.target_snapshot_change())

    def test_new_target_estimate(self):
        loc = location(1)
        target = self.core.on_target_location(loc, now_ns=NOW)
        e = self.estimate()
        self.assertEqual((target, 0, [loc], NOW - 10**8 + 1, NOW + 10**9 + 1, (4., 4.), [.01] * 9),
                         (e['target_id'], e['estimate_revision'], e['locations'], e['latest_observed_at_ns'],
                          e['valid_until_ns'], e['mission_position'], e['covariance']))
        self.assertEqual(('CONFIRMED', 'UNASSIGNED', 'IDLE', 'NOT_CHECKED', ''),
                         (e['confirmation_state'], e['approach_state'], e['segment_phase'], e['rescue_state'],
                          e['persisted_report_id']))

    def test_persisted_report_id_follows_the_current_revision(self):
        self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.assertEqual(report['report_id'], self.estimate()['persisted_report_id'])
        self.core.on_target_location(location(2, (4.5, 4.)), now_ns=NOW)
        e = self.estimate()
        self.assertEqual((1, '', 2, (4.5, 4.), NOW - 10**8 + 2), (e['estimate_revision'], e['persisted_report_id'],
                                                                 len(e['locations']), e['mission_position'],
                                                                 e['latest_observed_at_ns']))

    def test_approach_segment_and_rescue_states(self):
        self.core.on_site_event(site_msg())
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.core.on_status(status_msg(SUPPORT))
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=target), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('ASSIGNED', 'EXECUTING'), self.states())
        self.core.on_admission(admission_msg(fields))
        self.assertEqual(('IN_PROGRESS', 'EXECUTING'), self.states())
        self.core.on_result(fields['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual(('IN_PROGRESS', 'AWAITING_APPROVAL'), self.states())
        self.core.on_command(request('CONTINUE_SEGMENT', command_id=uid(702), target_id=target, route_revision=7,
                                     segment_index=1), now_ns=NOW)
        (second,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(second))
        self.core.on_result(second['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual(('APPROACH_COMPLETE', 'STOPPED'), self.states())
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(703), target_id=target), now_ns=NOW)
        self.core.mission.record_rescue_verdict(target, 'RESCUE_CONFIRMED')
        self.assertEqual('RESCUE_CONFIRMED', self.core.target_snapshot_change()['targets'][0]['rescue_state'])

    def states(self):
        snapshot = self.core.target_snapshot_change() or self.last
        self.last = snapshot
        e = snapshot['targets'][0]
        return e['approach_state'], e['segment_phase']

    def test_targets_are_ordered_by_id(self):
        self.core.on_target_location(location(1), now_ns=NOW)
        self.core.on_target_location(location(2, (9., 9.)), now_ns=NOW)
        ids = [e['target_id'] for e in self.core.target_snapshot_change()['targets']]
        self.assertEqual(sorted(ids), ids)


if __name__ == '__main__':
    unittest.main()
