"""Nav2 READY = NavigateToPose server on the graph AND the Nav2 lifecycle manager says active (no ROS).

Field run1015b (2026-10-15 17:54): TB-6 Nav2 bringup aborted ("Failed to bring up all requested nodes"), bt_navigator
stayed inactive, yet nav2_state was READY and task_arm ARMED: ActionClient.server_is_ready() only sees the action
server, which bt_navigator creates while configuring. The first goal was rejected ("Action server is inactive").
Now <ns>/lifecycle_manager_navigation/is_active (std_srvs/Trigger) must answer success, recently; no answer,
an old answer or an unavailable service is NOT_READY (nothing made up).
"""
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

from test_nav2_adapter import CLOCK_ERROR_BOUND, POLICY, FakeClock, FakeNav2, uid

MAX_AGE = 3.0


class FakeFuture:
    def __init__(self):
        self.callbacks, self.answer, self.error = [], None, None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        if self.error:
            raise self.error
        return self.answer

    def finish(self, success=None, error=None):
        self.answer, self.error = NS(success=success, message=''), error
        for callback in self.callbacks:
            callback(self)


class FakeActionClient:
    server = True

    def __init__(self, node, action_type, name):
        self.name = name

    def server_is_ready(self):
        return FakeActionClient.server


class FakeService:
    def __init__(self, name):
        self.name, self.available, self.calls, self.removed = name, True, [], []

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        self.calls.append(FakeFuture())
        return self.calls[-1]

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeNode:
    def __init__(self):
        self.services, self.logs = {}, []

    def create_client(self, srv_type, name):
        self.services[name] = FakeService(name)
        return self.services[name]

    def get_logger(self):
        return NS(info=self.logs.append, warn=self.logs.append, error=self.logs.append)


def fake_ros():
    action, nav2, srv = ModuleType('rclpy.action'), ModuleType('nav2_msgs.action'), ModuleType('std_srvs.srv')
    action.ActionClient = FakeActionClient
    nav2.NavigateToPose = nav2.Spin = object   # Spin: SPIN tasks (C29)
    srv.Trigger = NS(Request=lambda: NS())
    return {'rclpy': ModuleType('rclpy'), 'rclpy.action': action, 'nav2_msgs': ModuleType('nav2_msgs'),
            'nav2_msgs.action': nav2, 'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': srv}


class Nav2Ready(unittest.TestCase):
    SERVICE = '/robot6/lifecycle_manager_navigation/is_active'

    def setUp(self):
        FakeActionClient.server = True
        self.now = 100.0
        self.node = FakeNode()
        patcher = patch.dict(sys.modules, fake_ros())
        patcher.start()
        self.addCleanup(patcher.stop)
        from rescue_robot.ros_node import Nav2Port
        self.port = Nav2Port(self.node, '/robot6/navigate_to_pose', self.SERVICE, clock=lambda: self.now,
                             max_age_s=MAX_AGE)
        self.service = self.node.services[self.SERVICE]

    def answer(self, success):
        self.service.calls[-1].finish(success)

    def test_action_server_up_but_lifecycle_inactive_is_not_ready(self):
        """run1015b: bt_navigator inactive, its action server on the graph."""
        self.assertFalse(self.port.server_ready())                   # asked, no answer yet
        self.answer(False)
        self.assertFalse(self.port.server_ready())

    def test_ready_only_with_both(self):
        self.port.server_ready()
        self.answer(True)
        self.assertTrue(self.port.server_ready())
        FakeActionClient.server = False
        self.assertFalse(self.port.server_ready())

    def test_no_service_is_not_ready_and_asks_nothing(self):
        self.service.available = False
        self.assertFalse(self.port.server_ready())
        self.assertEqual([], self.service.calls)

    def test_an_old_answer_is_not_ready(self):
        """A manager that stops answering (Wi-Fi, Discovery) is not taken as still active."""
        self.port.server_ready()
        self.answer(True)
        self.now += MAX_AGE + 0.1
        self.assertFalse(self.port.server_ready())

    def test_asks_again_and_drops_an_unanswered_call(self):
        self.port.server_ready()
        first = self.service.calls[-1]
        self.now += 0.5
        self.port.server_ready()
        self.assertEqual(1, len(self.service.calls))               # not every step
        self.now += MAX_AGE / 3
        self.port.server_ready()
        self.assertEqual(2, len(self.service.calls))
        self.assertEqual([first], self.service.removed)
        self.answer(True)
        self.assertTrue(self.port.server_ready())

    def test_an_answer_that_turns_inactive_is_not_ready_and_logged(self):
        self.port.server_ready()
        self.answer(True)
        self.now += MAX_AGE / 3
        self.port.server_ready()
        self.answer(False)
        self.assertFalse(self.port.server_ready())
        self.assertTrue(any('NAV2_LIFECYCLE' in line and 'inactive' in line for line in self.node.logs),
                        self.node.logs)

    def logged(self, text):
        return [line for line in self.node.logs if 'NAV2_LIFECYCLE' in line and text in line]

    def test_an_answer_that_goes_old_is_logged_and_a_new_one_too(self):
        """A manager that stops answering turns nav2 NOT_READY: the reason is logged once, and READY again too."""
        self.port.server_ready()
        self.answer(True)
        self.assertTrue(self.port.server_ready())
        self.service.available = False                             # no more questions, the answer ages
        self.now += MAX_AGE + 0.1
        self.assertFalse(self.port.server_ready())
        self.port.server_ready()
        self.assertEqual(1, len(self.logged('no answer')), self.node.logs)
        self.service.available = True
        self.port.server_ready()
        self.answer(True)
        self.assertTrue(self.port.server_ready())
        self.assertEqual(2, len(self.logged(' active')), self.node.logs)

    def test_each_not_ready_reason_is_logged_on_change(self):
        self.service.available = False
        self.port.server_ready()
        self.assertEqual(1, len(self.logged('no service')), self.node.logs)
        self.service.available = True
        self.port.server_ready()
        self.answer(True)
        FakeActionClient.server = False
        self.port.server_ready()
        self.port.server_ready()
        self.assertEqual(1, len(self.logged('no NavigateToPose server')), self.node.logs)

    def test_a_failed_call_is_no_answer(self):
        self.port.server_ready()
        self.service.calls[-1].finish(error=RuntimeError('service gone'))
        self.assertFalse(self.port.server_ready())


class InactiveNav2RefusesArm(unittest.TestCase):
    """The port's answer feeds observe_status(nav2=...): an inactive Nav2 is NOT_READY and task_arm refuses."""

    def test_task_arm_refused_while_nav2_lifecycle_inactive(self):
        from rescue_robot.executor import Executor
        from rescue_robot.nav2_adapter import CommandAdapter
        clock = FakeClock()
        executor = Executor('robot_a', uid(1), uid(2), POLICY, allowed_robot_ids=('robot_a', 'robot_b'))
        adapter = CommandAdapter(executor, FakeNav2(), guards=lambda task: True, clock_error_bound_s=CLOCK_ERROR_BOUND,
                                 clock=clock, wall_clock_ns=clock.wall)
        node = FakeNode()
        with patch.dict(sys.modules, fake_ros()):
            from rescue_robot.ros_node import Nav2Port
            port = Nav2Port(node, '/robot6/navigate_to_pose', Nav2Ready.SERVICE, clock=clock, max_age_s=MAX_AGE)
            port.server_ready()
            node.services[Nav2Ready.SERVICE].calls[-1].finish(False)
            for _ in range(4):                                     # stopped, undocked, every other flag ready
                clock.now = round(clock.now + .1, 6)
                adapter.observe_status(driver=True, sensors=True, slam=True, nav2=port.server_ready(), watchdog=True)
                adapter.on_dock_status(False)
                sec, nanosec = divmod(clock.wall(), 10**9)
                adapter.on_odom(stamp_sec=sec, stamp_nanosec=nanosec, linear=(0., 0., 0.), angular=(0., 0., 0.))
        readiness = adapter.readiness(clock())
        self.assertFalse(readiness.nav2)
        self.assertTrue(all((readiness.driver, readiness.sensors, readiness.slam, readiness.undocked,
                             readiness.watchdog)))
        self.assertFalse(executor.arm(uid(2), now=clock(), readiness=readiness))

if __name__ == '__main__':
    unittest.main()
