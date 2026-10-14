"""OPS-231: re-arm by itself after a COMPLETED task with a confirmed stop, no ROS.

Owner approval 2026-10-14 (재범, grade 4): TB-6 searches point after point without waiting, TB-5
leaves 15 s after it stops in front of the person. Both were blocked because every finished task
left the executor STOP_LATCHED until an operator task_arm (#233 item 3). Only COMPLETED + stop
confirmed + no session fault re-arms, on the task_arm conditions; every other end stays latched.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import unittest

import test_executor as core
import test_robot_task_site_runtime as site
from rescue_robot.executor import ContractError
from rescue_robot.task_site_runtime import SiteRuntime, InputStore
from rescue_robot.task_status import fill_robot_status


class ExecutorRearm(unittest.TestCase):
    """The executor alone: which ends may re-arm, and only on arm()'s conditions."""
    def setUp(self):
        self.t = core.ExecutorTests(); self.t.setUp()
        self.e = self.t.e

    def still(self, *times):
        for t in times:
            self.e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)

    def complete(self):
        self.t.send()
        self.e.goal_response(core.uid(3), accepted=True, now=.2)
        self.e.goal_terminal(core.uid(3), terminal_status='SUCCEEDED', now=.3)
        self.still(.3, .4, .5, .6)
        self.assertEqual(self.e.results[core.uid(3)].outcome, 'COMPLETED')

    def test_completed_with_confirmed_stop_re_arms_on_the_arm_conditions(self):
        self.complete()
        self.assertTrue(self.e.rearm_due)
        self.assertIsNone(self.e.rearm_after_completion(core.uid(2), now=.6,
                                                        readiness=replace(self.t.ready, received_steady=.6)))
        self.assertEqual((self.e.latch, self.e.state, self.e.rearm_due), ('ARMED', 'IDLE', False))
        task = replace(self.t.task, command_id=core.uid(30), task_id=core.uid(40), generation=2)
        self.assertTrue(self.e.admit(task, now_wall_ns=100, now_steady=.6,
                                     readiness=replace(self.t.ready, received_steady=.6)).accepted)

    def test_invalid_readiness_keeps_the_latch_and_names_the_reason(self):
        self.complete()
        reason = self.e.rearm_after_completion(core.uid(2), now=.6,
                                               readiness=replace(self.t.ready, received_steady=.6, nav2=False))
        self.assertEqual(reason, 'NAV_NOT_READY')
        self.assertEqual((self.e.latch, self.e.rearm_refused, self.e.rearm_due),
                         ('STOP_LATCHED', 'NAV_NOT_READY', False))

    def test_wrong_authority_epoch_is_not_re_armed(self):
        self.complete()
        self.assertEqual(self.e.rearm_after_completion(core.uid(9), now=.6,
                                                       readiness=replace(self.t.ready, received_steady=.6)),
                         'NOT_AUTHORIZED')
        self.assertEqual(self.e.latch, 'STOP_LATCHED')

    def test_only_once_per_completed_task(self):
        self.complete()
        self.e.rearm_after_completion(core.uid(2), now=.6, readiness=replace(self.t.ready, received_steady=.6))
        with self.assertRaises(ContractError):
            self.e.rearm_after_completion(core.uid(2), now=.6, readiness=replace(self.t.ready, received_steady=.6))

    def assert_stays_latched(self, outcome, reason):
        result = self.e.results[core.uid(3)]
        self.assertEqual((result.outcome, result.reason), (outcome, reason))
        self.assertFalse(self.e.rearm_due)
        with self.assertRaises(ContractError):
            self.e.rearm_after_completion(core.uid(2), now=self.e.clock,
                                          readiness=replace(self.t.ready, received_steady=self.e.clock))
        task = replace(self.t.task, command_id=core.uid(30), task_id=core.uid(40), generation=2)
        self.assertEqual(self.e.admit(task, now_wall_ns=100, now_steady=self.e.clock,
                                      readiness=replace(self.t.ready, received_steady=self.e.clock)).reason,
                         'STOP_LATCHED')

    def test_failed_stays_latched(self):
        self.t.send()
        self.e.goal_response(core.uid(3), accepted=True, now=.2)
        self.e.goal_terminal(core.uid(3), terminal_status='ABORTED', now=.3)
        self.still(.3, .4, .5, .6)
        self.assert_stays_latched('FAILED', 'ACTION_ABORTED')

    def test_task_stop_stays_latched(self):
        self.t.send()
        self.e.goal_response(core.uid(3), accepted=True, now=.2)
        self.e.request_stop('CANCEL_USER', now=.3)
        self.e.goal_terminal(core.uid(3), terminal_status='CANCELED', now=.3)
        self.still(.3, .4, .5, .6)
        self.assert_stays_latched('CANCELED', 'ACTION_CANCELED')  # CANCEL_USER's registered code

    def test_expired_lease_stays_latched(self):
        self.t.send()
        self.e.goal_response(core.uid(3), accepted=True, now=.2)
        self.t.tick(2.3)  # lease_duration 2 s from .2
        self.e.goal_terminal(core.uid(3), terminal_status='CANCELED', now=2.3)
        self.still(2.3, 2.4, 2.5, 2.6)
        self.assert_stays_latched('EXPIRED', 'LEASE_EXPIRED')

    def test_stop_unconfirmed_stays_fault_latched(self):
        self.t.send()
        self.e.goal_response(core.uid(3), accepted=True, now=.2)
        self.e.request_stop('CANCEL_USER', now=.3)
        self.t.tick(3.4)  # stop_timeout 3 s without stop evidence
        self.assertEqual(self.e.latch, 'FAULT_LATCHED')
        self.assert_stays_latched('FAILED', 'STOP_UNCONFIRMED')


class RuntimeRearm(unittest.TestCase):
    """The installed site runtime: the adapter calls it once after the completed task."""
    def setUp(self, auto_rearm=True):
        s = site.SiteRuntimeTests(); s.f = site.phase.Phase1RouteTests(); s.f.setUp()
        self.f = s.f
        self.data = site.records(self.f); self.cfg = site.config(self.f)
        self.cfg['auto_rearm'] = auto_rearm
        self.adapter = self.f.r.adapter; self.adapter.executor.latch = 'DISARMED'
        self.node = site.transport.Node()
        bindings = NS(**vars(self.f.r.server.bindings))
        def response():
            return NS(envelope=NS(), result=NS(final_pose=NS(envelope=NS(), time=NS())))
        bindings.ExecuteTask = NS(Result=response, Feedback=self.f.r.server.bindings.ExecuteTask.Feedback)
        self.store = InputStore(self.cfg, wall_clock_ns=self.adapter.wall_clock_ns, read=lambda: deepcopy(self.data))
        self.runtime = SiteRuntime(self.node, self.adapter, self.cfg, self.store, bindings=bindings)
        self.assertTrue(self.runtime.arm())
        self.g = self.f.g

    def start(self):
        self.assertEqual(self.runtime.server.on_goal(self.g), 'accept', self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)

    def stop(self, nav2=True):
        mapping = self.f.r.f
        for _ in range(4):
            mapping.clock.now = round(mapping.clock.now + .1, 6)
            self.adapter.observe_status(driver=True, sensors=True, slam=True, nav2=nav2, watchdog=True)
            self.adapter.on_dock_status(False)
            sec, nanosec = divmod(mapping.clock.wall(), 10**9)
            self.adapter.on_odom(stamp_sec=sec, stamp_nanosec=nanosec, linear=(0, 0, 0), angular=(0, 0, 0))

    def status(self):
        return fill_robot_status(NS(), self.adapter, self.runtime.server.ingress, state_sequence=1)

    def next_task(self):
        task = self.adapter.executor.seen[self.g.command_id][0]
        return replace(task, command_id=site.phase.runtime.mapping.uid(901),
                       task_id=site.phase.runtime.mapping.uid(902), generation=task.generation + 1)

    def test_completed_and_stopped_takes_the_next_task(self):
        self.start()
        self.adapter.on_terminal(self.g.command_id, 4)
        self.stop()
        result = self.adapter.result(self.g.command_id)
        self.assertEqual((result.outcome, result.stop_confirmed), ('COMPLETED', True))
        self.assertEqual(self.adapter.executor.latch, 'ARMED')
        self.assertEqual(self.status().readiness, 'READY')
        self.assertTrue(self.adapter.submit(self.next_task()).accepted)

    def test_failed_or_stopped_by_hand_waits_for_task_arm(self):
        for name, end in (('FAILED', lambda: self.adapter.on_terminal(self.g.command_id, 6)),
                          ('CANCELED', lambda: (self.adapter.request_stop('CANCEL_USER'),
                                                self.adapter.on_terminal(self.g.command_id, 5)))):
            with self.subTest(outcome=name):
                self.setUp()
                self.start()
                end()
                self.stop()
                self.assertEqual(self.adapter.result(self.g.command_id).outcome, name)
                self.assertEqual(self.status().readiness, 'NOT_READY')
                self.assertEqual(self.adapter.submit(self.next_task()).reason, 'STOP_LATCHED')

    def test_readiness_invalid_at_completion_keeps_the_latch_with_a_reason(self):
        self.start()
        self.adapter.on_terminal(self.g.command_id, 4)
        self.stop(nav2=False)
        self.assertEqual(self.adapter.result(self.g.command_id).outcome, 'COMPLETED')
        self.assertEqual(self.adapter.executor.latch, 'STOP_LATCHED')
        status = self.status()
        self.assertEqual((status.readiness, status.stop_reason), ('NOT_READY', 'REARM_REFUSED_NAV_NOT_READY'))
        self.assertIn('NAV_NOT_READY', '\n'.join(self.node.errors))
        self.stop()  # readiness back: still no re-arm, the operator task_arm decides
        self.assertEqual(self.adapter.executor.latch, 'STOP_LATCHED')
        self.assertEqual(self.adapter.submit(self.next_task()).reason, 'STOP_LATCHED')

    def test_off_keeps_the_old_latch(self):
        self.setUp(auto_rearm=False)
        self.start()
        self.adapter.on_terminal(self.g.command_id, 4)
        self.stop()
        self.assertEqual(self.adapter.executor.latch, 'STOP_LATCHED')
        self.assertEqual(self.adapter.submit(self.next_task()).reason, 'STOP_LATCHED')

    def test_setting_must_be_a_bool(self):
        self.adapter.executor.latch = 'DISARMED'
        self.cfg['auto_rearm'] = 'true'
        with self.assertRaisesRegex(ContractError, 'auto_rearm'):
            SiteRuntime(self.node, self.adapter, self.cfg, self.store, bindings=self.runtime.server.bindings)


if __name__ == '__main__':
    unittest.main()
