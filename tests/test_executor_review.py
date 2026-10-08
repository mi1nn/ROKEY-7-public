"""CLA-18/19/20 regression coverage, committed before implementation."""
import unittest
from dataclasses import replace
from uuid import UUID
from rescue_robot.executor import Executor, GoalPose, Policy, Readiness, Task


def uid(n): return str(UUID(int=n))


class ExecutorReviewTests(unittest.TestCase):
    def setup_session(self):
        policy = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)
        self.e = Executor('alpha', uid(1), uid(2), policy, allowed_robot_ids=('alpha', 'bravo'))
        self.ready = Readiness(.2, True, True, True, True, True, True)
        for t in (0, .1, .2):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertTrue(self.e.arm(uid(2), now=.2, readiness=self.ready))
        self.task = Task(uid(3), uid(4), 'alpha', uid(1), uid(2), 0, 'SEARCH_AREA',
                         1700000000123456789, 1700000002123456789, 2, 'test-only',
                         search_area_ref=uid(5), goal_pose=GoalPose('map', 1., 2., 0., uid(6), 0))

    def send(self):
        receipt = self.e.admit(self.task, now_wall_ns=self.task.issued_at_ns,
                              now_steady=.2, readiness=self.ready)
        self.assertTrue(receipt.accepted)
        self.assertEqual((receipt.command_id, receipt.task_id, receipt.robot_boot_id, receipt.generation),
                         (self.task.command_id, self.task.task_id, self.task.robot_boot_id, 0))
        self.e.dispatch(now_wall_ns=self.task.issued_at_ns, now_steady=.2,
                        readiness=self.ready, guards_valid=True)

    def test_profile_ids_and_receipt_identity(self):
        self.setup_session(); self.send()

    def test_bounded_velocity_before_fence_is_not_stop_requested(self):
        self.setup_session(); self.send()
        self.e.observe_motion(sample_steady=.3, received_steady=.3, linear=0, angular=0)
        self.assertEqual(self.e.motion, 'MOVING')

    def test_admission_reasons_are_canonical(self):
        self.setup_session()
        for change, expected in ((dict(robot_id='bravo'), 'WRONG_ROBOT'),
                                 (dict(authority_epoch=uid(20)), 'NOT_AUTHORIZED'),
                                 (dict(policy_version='other'), 'POLICY_MISMATCH')):
            receipt = self.e.admit(replace(self.task, **change), now_wall_ns=self.task.issued_at_ns,
                                   now_steady=.2, readiness=self.ready)
            self.assertEqual(receipt.reason, expected)
            self.assertTrue(receipt.detail_code.startswith('rescue_robot.'))
            self.e.seen.clear()

    def test_external_cancel_and_abort_preserve_action_status(self):
        for status, outcome, reason in (('CANCELED', 'CANCELED', 'ACTION_CANCELED'),
                                         ('ABORTED', 'FAILED', 'ACTION_ABORTED'),
                                         ('SUCCEEDED', 'COMPLETED', 'NAVIGATION_SUCCEEDED')):
            self.setup_session(); self.send()
            self.e.goal_terminal(uid(3), terminal_status=status, now=.3)
            for t in (.4, .5, .6, .7):
                self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
            result = self.e.results[uid(3)]
            self.assertEqual((result.action_status, result.outcome, result.reason), (status, outcome, reason))

    def test_stop_cause_not_overwritten_by_action_cancellation(self):
        self.setup_session(); self.send()
        self.e.request_stop('TARGET_STALE', now=.3)
        self.e.goal_terminal(uid(3), terminal_status='CANCELED', now=.4)
        for t in (.5, .6, .7, .8):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        result = self.e.results[uid(3)]
        self.assertEqual((result.original_cause, result.reason, result.action_status),
                         ('TARGET_STALE', 'STALE_OBSERVATION', 'CANCELED'))

    def test_float_epoch_and_conflicting_terminal_rejected(self):
        self.setup_session()
        with self.assertRaises(ValueError): replace(self.task, issued_at_ns=float(self.task.issued_at_ns))
        self.send()
        self.e.goal_terminal(uid(3), terminal_status='SUCCEEDED', now=.3)
        with self.assertRaises(ValueError):
            self.e.goal_terminal(uid(3), terminal_status='ABORTED', now=.4)
        self.assertEqual(self.e.action_status, 'SUCCEEDED')

    def test_conflicting_goal_response_cannot_invent_execution_fence(self):
        for stopping in (False, True):
            self.setup_session(); self.send()
            self.e.goal_response(uid(3), accepted=True, now=.3)
            if stopping:
                self.e.request_stop('TARGET_STALE', now=.4)
            with self.assertRaises(ValueError):
                self.e.goal_response(uid(3), accepted=False, now=.5)
            self.assertFalse(self.e.fenced)
            if not stopping:
                self.e.request_stop('TARGET_STALE', now=.5)
            self.assertIn(('CANCEL', uid(3)), self.e.drain_intents())
            for t in (.6, .7, .8, .9):
                self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
            self.assertNotIn(uid(3), self.e.results)

    def test_unknown_stop_reason_has_common_error_and_is_atomic(self):
        from rescue_contracts import RescueError
        self.setup_session(); self.send()
        with self.assertRaises(RescueError) as caught:
            self.e.request_stop('bogus', now=.3)
        self.assertEqual(caught.exception.code, 'INVALID_INPUT')
        self.assertEqual(self.e.state, 'AWAITING_ACCEPTANCE')
        self.assertFalse(self.e.fenced)
