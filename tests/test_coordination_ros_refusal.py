"""A task the executor refuses before moving is issued again, after a hold (E2; found on review 2026-10-13).

#90 meant "refused before moving: issue again on the next tick", but the Allocator returns False for a
refusal, so that branch never ran: a refused MOVE_SEGMENT or REOBSERVE_TARGET was never sent again.
Fixed: a refusal (a receipt the Allocator recorded, accepted=false) releases the task; the same approval
or recheck waits ``admission_timeout_ns`` with SystemState.reason TASK_REFUSED, then goes as a new task.
The hold keeps a robot that keeps refusing from getting the same task every tick.
"""
import unittest

from test_coordination_ros_core import (ACCEPT, SCOUT, SUPPORT, admission_msg, location, make_core, receipt, request,
                                        site_msg, status_msg)
from test_coordination_search import NOW, uid


class RefusalTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core()
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SUPPORT))
        self.core.on_status(status_msg(SCOUT))
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))

    def kinds(self, kind, now):
        for robot in (SUPPORT, SCOUT):  # fresh status each tick: the hold is longer than the status max age
            self.core.on_status(status_msg(robot, observed_at_ns=now - 10**8))
        return [f for f in self.core.tick(now) if f['kind'] == kind]

    def test_refused_segment_waits_then_goes_again_as_a_new_task(self):
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=self.target), now_ns=NOW)
        (first,) = self.kinds('MOVE_SEGMENT', NOW)
        self.assertFalse(self.core.on_admission(admission_msg(first, accepted=False)))  # not ACTIVE
        self.assertEqual([], self.kinds('MOVE_SEGMENT', NOW + 1))
        self.assertEqual('TASK_REFUSED', self.core.system_state_change(NOW + 1)['reason'])
        self.assertEqual([], self.kinds('MOVE_SEGMENT', NOW + 1 + ACCEPT))
        (again,) = self.kinds('MOVE_SEGMENT', NOW + 2 + ACCEPT)
        self.assertNotEqual(first['task_id'], again['task_id'])
        self.assertEqual(first['approval']['approval_id'], again['approval']['approval_id'])  # same operator approval

    def test_refused_recheck_waits_then_goes_again(self):
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(702), target_id=self.target), now_ns=NOW)
        (first,) = self.kinds('REOBSERVE_TARGET', NOW)
        self.core.on_admission(admission_msg(first, accepted=False))
        self.assertEqual([], self.kinds('REOBSERVE_TARGET', NOW + 1))
        (again,) = self.kinds('REOBSERVE_TARGET', NOW + 2 + ACCEPT)
        self.assertNotEqual(first['task_id'], again['task_id'])

    def test_a_refusal_that_does_not_match_the_task_changes_nothing(self):
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=self.target), now_ns=NOW)
        (first,) = self.kinds('MOVE_SEGMENT', NOW)
        wrong = replace_boot(admission_msg(first, accepted=False))
        self.assertFalse(self.core.on_admission(wrong))
        self.assertIn(first['task_id'], self.core.task_meta)  # still waiting for the real receipt
        self.assertEqual([], self.kinds('MOVE_SEGMENT', NOW + 2 + ACCEPT))


def replace_boot(msg):
    msg.robot_boot_id = uid(4242)
    return msg


if __name__ == '__main__':
    unittest.main()
