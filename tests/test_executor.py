import unittest
from dataclasses import replace
from uuid import UUID
from rescue_robot.executor import Executor, GoalPose, Task, Policy, Readiness, ContractError, future_deadline


def uid(n):
    return str(UUID(int=n))


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.policy = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)
        self.e = Executor('robot_a', uid(1), uid(2), self.policy, allowed_robot_ids=('robot_a', 'robot_b'))
        self.ready = Readiness(.2, True, True, True, True, True, True)
        for t in (0, .1, .2):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertTrue(self.e.arm(uid(2), now=.2, readiness=self.ready))
        self.task = Task(uid(3), uid(4), 'robot_a', uid(1), uid(2), 1,
                         'SEARCH_AREA', 100, 200, 2, 'test-only', search_area_ref=uid(5),
                         goal_pose=GoalPose('map', 1., 2., 0., uid(6), 0))

    def admit(self):
        return self.e.admit(self.task, now_wall_ns=100, now_steady=.2, readiness=self.ready)

    def send(self):
        self.assertTrue(self.admit().accepted)
        self.assertTrue(self.e.dispatch(now_wall_ns=100, now_steady=.2, readiness=self.ready, guards_valid=True))
        self.assertEqual(self.e.drain_intents(), (('SEND', uid(3)),))

    def tick(self, t, **kwargs):
        self.e.tick(now_wall_ns=100+int(t), now_steady=t, readiness=replace(self.ready, received_steady=t),
                    guards_valid=kwargs.get('guards_valid', True))

    def test_duplicate_is_idempotent_conflict_rejected(self):
        self.assertTrue(self.admit().accepted)
        self.assertEqual(self.admit(), self.admit())
        conflict = replace(self.task, deadline_ns=300)
        self.assertEqual(self.e.admit(conflict, now_wall_ns=100, now_steady=.2,
                                    readiness=self.ready).reason, 'ID_CONTENT_CONFLICT')

    def test_wrong_robot_boot_authority_generation(self):
        for changed in (dict(robot_id='robot_b'), dict(robot_boot_id=uid(9)), dict(authority_epoch=uid(9))):
            other = replace(self.task, **changed)
            self.assertFalse(self.e.admit(other, now_wall_ns=100, now_steady=.2, readiness=self.ready).accepted)
            self.e.seen.clear()
        self.assertTrue(self.admit().accepted)
        self.assertFalse(self.e.admit(replace(self.task, command_id=uid(9)), now_wall_ns=100,
                                      now_steady=.2, readiness=self.ready).accepted)

    def test_readiness_requires_undock_and_watchdog(self):
        for change in (dict(undocked=False), dict(watchdog=False), dict(received_steady=2), dict(driver=1)):
            self.assertFalse(self.e.admit(self.task, now_wall_ns=100, now_steady=.2,
                                          readiness=replace(self.ready, **change)).accepted)
            self.e.seen.clear()

    def test_dispatch_revalidates_guards(self):
        self.admit()
        self.assertFalse(self.e.dispatch(now_wall_ns=100, now_steady=.2, readiness=self.ready, guards_valid=False))
        self.assertEqual(self.e.drain_intents(), ())
        self.assertEqual(self.e.state, 'STOPPING')

    def test_acceptance_timeout_cancels_late_accept(self):
        self.send()
        self.tick(1.2)
        self.assertEqual(self.e.state, 'STOPPING')
        self.e.drain_intents()
        self.e.goal_response(uid(3), accepted=True, now=self.e.clock)
        self.assertEqual(self.e.drain_intents(), (('CANCEL', uid(3)),))
        self.assertFalse(self.e.fenced)

    def test_stop_requires_terminal_and_continuous_fresh_motion(self):
        self.send()
        self.e.goal_response(uid(3), accepted=True, now=self.e.clock)
        self.e.request_stop('CANCEL_USER', now=.3)
        for t in (.3, .4, .5):
            self.assertFalse(self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0))
        self.assertNotIn(uid(3), self.e.results)
        self.e.goal_terminal(uid(3), terminal_status='CANCELED', now=.5)
        for t in (.6, .7, .8, .9):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        result = self.e.results[uid(3)]
        self.assertTrue(result.stop_confirmed)
        self.assertEqual(result.outcome, 'CANCELED')
        self.assertEqual(self.e.latch, 'STOP_LATCHED')

    def test_cancel_timeout_failure_latches_and_keeps_original_cause(self):
        self.send()
        self.e.request_stop('TARGET_STALE', now=.3)
        self.tick(3.3)
        result = self.e.results[uid(3)]
        self.assertEqual((result.reason, result.original_cause, result.stop_confirmed),
                         ('STOP_UNCONFIRMED', 'TARGET_STALE', False))
        self.assertFalse(self.e.arm(uid(2), now=3.3, readiness=replace(self.ready, received_steady=3.3)))
        self.e.goal_response(uid(3), accepted=True, now=self.e.clock)
        self.assertIn(('CANCEL', uid(3)), self.e.drain_intents())

    def test_lease_sequence_no_revival(self):
        self.send()
        self.assertTrue(self.e.renew(uid(3), uid(2), 1, 1, now=.3))
        self.assertFalse(self.e.renew(uid(3), uid(2), 1, 1, now=.4))
        self.assertFalse(self.e.renew(uid(3), uid(2), 1, 2, now=2.3))
        self.tick(2.3)
        self.assertEqual(self.e.stop_outcome, 'EXPIRED')

    def test_late_old_result_cannot_touch_current_task(self):
        self.send()
        self.assertFalse(self.e.goal_terminal(uid(99), terminal_status='SUCCEEDED', now=.3))
        self.assertEqual(self.e.state, 'AWAITING_ACCEPTANCE')

    def test_motion_duplicate_gap_moving_and_stale(self):
        self.assertFalse(self.e.observe_motion(sample_steady=.2, received_steady=.2, linear=0, angular=0))
        self.assertFalse(self.e.observe_motion(sample_steady=1, received_steady=1, linear=0, angular=0))
        self.assertFalse(self.e.observe_motion(sample_steady=1.1, received_steady=1.1, linear=.1, angular=0))
        self.assertFalse(self.e.observe_motion(sample_steady=1.2, received_steady=3, linear=0, angular=0))
        self.assertEqual(self.e.motion, 'STOP_UNCONFIRMED')

    def test_input_rejection(self):
        for change in (dict(generation=True), dict(command_id='x'), dict(issued_at_ns=float('nan')),
                       dict(lease_duration=0), dict(kind='UNDOCK'), dict(command_id=uid(0)), dict(lease_duration=10**1000)):
            with self.assertRaises(ContractError):
                replace(self.task, **change)

    def test_dispatch_requires_current_stopped_motion(self):
        self.admit()
        self.e.observe_motion(sample_steady=.3, received_steady=.3, linear=.5, angular=0)
        self.assertFalse(self.e.dispatch(now_wall_ns=100, now_steady=.3,
                         readiness=replace(self.ready, received_steady=.3), guards_valid=True))
        self.assertNotIn(('SEND', uid(3)), self.e.drain_intents())

    def test_delayed_prefence_samples_cannot_confirm_stop(self):
        self.send()
        self.e.goal_terminal(uid(3), terminal_status='SUCCEEDED', now=.8)
        for stamp in (.4, .5, .6, .7):
            self.assertFalse(self.e.observe_motion(sample_steady=stamp, received_steady=.8,
                                                   linear=0, angular=0))
        self.assertNotIn(uid(3), self.e.results)

    def test_lease_generation_rejects_boolean(self):
        self.send()
        with self.assertRaises(ContractError):
            self.e.renew(uid(3), uid(2), True, 1, now=.3)

    def test_duplicate_terminal_does_not_reset_stop_window(self):
        self.send()
        self.e.goal_terminal(uid(3), terminal_status='SUCCEEDED', now=.3)
        self.e.observe_motion(sample_steady=.4, received_steady=.4, linear=0, angular=0)
        self.e.goal_terminal(uid(3), terminal_status='SUCCEEDED', now=.5)
        self.assertEqual(self.e.fenced_since, .3)
        self.assertEqual(self.e.stop_start, .4)

    def test_local_deadlines_cannot_overflow_or_round_to_present(self):
        for now, duration in ((1e308, 1e308), (1e308, 1)):
            with self.assertRaises(ContractError):
                future_deadline(now, duration)
        with self.assertRaises(ContractError):
            self.e.observe_motion(sample_steady=.3, received_steady=.3, linear=10**1000, angular=0)

    def test_rejected_renewal_preserves_sequence_and_deadline(self):
        self.task = replace(self.task, lease_duration=1e308)
        self.admit()
        before = self.e.lease_sequence, self.e.lease_until
        with self.assertRaises(ContractError):
            self.e.renew(uid(3), uid(2), 1, 5, now=9e307)
        self.assertEqual((self.e.lease_sequence, self.e.lease_until), before)

    def test_invalid_stop_timer_still_cancels_and_latches_failure(self):
        self.e.policy = replace(self.policy, stop_timeout=5e-324)
        self.send()
        self.e.request_stop('TARGET_STALE', now=.3)
        self.assertIn(('CANCEL', uid(3)), self.e.drain_intents())
        self.assertEqual(self.e.latch, 'FAULT_LATCHED')
        result = self.e.results[uid(3)]
        self.assertEqual((result.reason, result.original_cause, result.stop_confirmed),
                         ('STOP_UNCONFIRMED', 'TARGET_STALE', False))

    def test_regressed_clock_latches(self):
        self.send()
        with self.assertRaises(ContractError):
            self.tick(.1)
        self.assertEqual(self.e.latch, 'FAULT_LATCHED')
        self.tick(.4)
        self.e.goal_terminal(uid(3), terminal_status='CANCELED', now=.5)
        for t in (.6, .7, .8, .9):
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertFalse(self.e.arm(uid(2), now=.9, readiness=replace(self.ready, received_steady=.9)))


if __name__ == '__main__':
    unittest.main()
