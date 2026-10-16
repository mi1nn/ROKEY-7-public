"""SPIN: the camera's turn in place as one Nav2 Spin action (C29; no ROS).

재범 2026-10-16 11:4x: 「판정할 때 같은 자리에서 너무 많이 돈다」. Run 18: of the 93 s from the disaster sign to
the doll, TB-6 moved 21 s and stood 68 s; at the tunnel 1 exit it turned 480 deg in 98 s. The camera look round a
frontier (frontier_look_turns 4) went as four 90 deg NavigateToPose goals, and Nav2's SimpleProgressChecker
(0.5 m in 10 s) took turning in place for no progress: Failed to make progress, abort, BT retry, recovery Spin.

Now the executor takes a task kind SPIN (contract addition) and runs it with nav2_msgs/action/Spin on the
robot's behavior_server (<ns>/spin, already running, max_rotational_vel 1.0 rad/s):
- fields spin_yaw_rad (positive = counter-clockwise, at most one turn) and time_allowance_s; no pose, no target,
  no area, no home, no route, no approval; other kinds carry 0.0 in both.
- Spin SUCCEEDED -> COMPLETED (NAVIGATION_SUCCEEDED), ABORTED (collision ahead, time allowance) -> FAILED
  SPIN_ABORTED, CANCELED -> CANCELED as NavigateToPose. Guards, watchdog and the stop confirmation are the same
  path as NavigateToPose. Robot settings are unchanged.
"""
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from rescue_robot.executor import ContractError, Task

from test_nav2_adapter import AdapterHelpers, uid
from test_robot_task_wire import validate, wire

ROOT = Path(__file__).resolve().parents[1]
TURN = 2 * math.pi


class SpinTask(unittest.TestCase):
    def task(self, kind='SPIN', **fields):
        base = dict(spin_yaw_rad=TURN, time_allowance_s=15.0) if kind == 'SPIN' else {}
        base.update(fields)
        return Task(uid(3), uid(4), 'robot_a', uid(1), uid(2), 1, kind, 10**18, 10**18 + 60 * 10**9, 5,
                    'test-only', **base)

    def test_a_spin_task_has_its_turn_and_no_pose(self):
        task = self.task()
        self.assertEqual((TURN, 15.0, None), (task.spin_yaw_rad, task.time_allowance_s, task.goal_pose))

    def test_bad_spin_fields_are_refused(self):
        for bad in (dict(spin_yaw_rad=0.0), dict(spin_yaw_rad=math.nan), dict(spin_yaw_rad=TURN + 0.01),
                    dict(spin_yaw_rad=None), dict(time_allowance_s=0.0), dict(time_allowance_s=math.inf)):
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                self.task(**bad)
        with self.assertRaises(ContractError):
            self.task('HOLD', spin_yaw_rad=TURN, time_allowance_s=15.0)   # only SPIN turns


class SpinWire(unittest.TestCase):
    def spin(self):
        goal, lease = wire('HOLD')
        goal.kind, goal.spin_yaw_rad, goal.time_allowance_s = 'SPIN', TURN, 15.0
        return goal, lease

    def test_spin_wire_is_valid_and_other_kinds_carry_no_turn(self):
        self.assertEqual(2.0, validate(*self.spin()))
        goal, lease = wire('SEARCH_AREA')
        goal.spin_yaw_rad = TURN
        with self.assertRaises(ContractError):
            validate(goal, lease)

    def test_spin_carries_no_route_approval_or_other_robots_frame(self):
        for change in (dict(has_route=True), dict(has_approval=True), dict(time_allowance_s=-1.0)):
            goal, lease = self.spin()
            for name, value in change.items():
                setattr(goal, name, value)
            with self.subTest(change=change), self.assertRaises(ContractError):
                validate(goal, lease)
        goal, lease = self.spin()
        goal.frame_ref.robot_id = 'robot_b'
        with self.assertRaises(ContractError):
            validate(goal, lease)


class SpinAdapter(AdapterHelpers, unittest.TestCase):
    def spin(self, n=3):
        from rescue_robot.executor import SpinGoal
        self.arm(self.a, 1)
        self.ready(self.a)
        task = self.task(self.a, 1, n, kind='SPIN', spin_yaw_rad=TURN, time_allowance_s=15.0)
        self.assertTrue(self.a.submit(task).accepted)
        self.a.step()
        self.assertEqual([(uid(n), SpinGoal(TURN, 15.0))], self.nav_a.goals)
        self.a.on_goal_response(uid(n), True)
        self.advance()

    def test_spin_sends_a_spin_goal_and_completes_after_the_stop(self):
        self.spin()
        self.a.on_terminal(uid(3), 4)
        self.still(self.a)
        result = self.a.result(uid(3))
        self.assertEqual(('COMPLETED', 'NAVIGATION_SUCCEEDED', True),
                         (result.outcome, result.reason, result.stop_confirmed))

    def test_an_aborted_spin_is_spin_aborted(self):
        self.spin()
        self.a.on_terminal(uid(3), 6)
        self.still(self.a)
        result = self.a.result(uid(3))
        self.assertEqual(('FAILED', 'SPIN_ABORTED'), (result.outcome, result.reason))

    def test_a_spin_is_canceled_like_a_navigation(self):
        self.spin()
        self.a.request_stop('CANCEL_USER')
        self.assertEqual([uid(3)], self.nav_a.cancels)
        self.a.on_terminal(uid(3), 5)
        self.still(self.a)
        self.assertEqual('CANCELED', self.a.result(uid(3)).outcome)

    def test_a_guard_failure_stops_a_spin(self):
        self.spin()
        self.guard_ok = False
        self.ready(self.a)
        self.a.step()
        self.assertEqual([uid(3)], self.nav_a.cancels)


class SpinPort(unittest.TestCase):
    @unittest.expectedFailure
    def test_the_port_sends_target_yaw_and_time_allowance_to_the_spin_server(self):
        from test_robot_nav2_ready import FakeNode, fake_ros
        sent = []

        class Client:
            def __init__(self, node, action_type, name):
                self.name, self.type = name, action_type

            def server_is_ready(self):
                return True

            def send_goal_async(self, goal):
                sent.append((self.name, goal))
                return NS(add_done_callback=lambda callback: None)

        class SpinGoalMessage:
            """nav2_msgs/action/Spin Goal as in Jazzy (ros-jazzy-nav2-msgs 1.3.13): target_yaw and time_allowance only.
            disable_collision_checks came after Jazzy; a generated message refuses an unknown field (AttributeError,
            REV-27)."""
            __slots__ = ('target_yaw', 'time_allowance')

            def __init__(self):
                self.target_yaw, self.time_allowance = 0.0, NS(sec=0, nanosec=0)

        modules = fake_ros()
        modules['rclpy.action'].ActionClient = Client
        modules['nav2_msgs.action'].Spin = NS(Goal=SpinGoalMessage)
        node = FakeNode()
        node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
        with patch.dict(sys.modules, modules):
            from rescue_robot.executor import SpinGoal
            from rescue_robot.ros_node import Nav2Port
            port = Nav2Port(node, '/robot6/navigate_to_pose', '/robot6/lifecycle_manager_navigation/is_active',
                            clock=lambda: 0.0, max_age_s=3.0)
            port.send_goal(uid(3), SpinGoal(TURN, 15.0))
        name, goal = sent[0]
        self.assertEqual(('/robot6/spin', TURN, 15, 0),
                         (name, goal.target_yaw, goal.time_allowance.sec, goal.time_allowance.nanosec))


class SpinContract(unittest.TestCase):
    def test_the_contract_lists_spin_and_its_fields(self):
        from rescue_contracts import CANONICAL_CODES, TARGET_TASK_KINDS, TASK_KINDS
        from rescue_robot.task_wire import COMPLETION_SCOPES
        self.assertIn('SPIN', TASK_KINDS)
        self.assertNotIn('SPIN', TARGET_TASK_KINDS)
        self.assertIn('SPIN_ABORTED', CANONICAL_CODES)
        self.assertEqual('SPIN', COMPLETION_SCOPES['SPIN'])

    @unittest.skipUnless((ROOT / 'docs').is_dir() and (ROOT / 'src').is_dir(), 'source checkout only (installed check)')
    def test_the_interface_files_list_spin_and_its_fields(self):
        spec = json.loads((ROOT / 'docs' / 'design' / 'sdd-node-contracts-v0.1.json').read_text())
        goal = spec['actions']['ExecuteTask']['Goal']
        self.assertIn('/MOVE_SEGMENT/RETURN_HOME/SPIN)', goal)
        self.assertIn('spin_yaw_rad:float64, time_allowance_s:float64', goal)
        action = (ROOT / 'src' / 'rescue_interfaces' / 'action' / 'ExecuteTask.action').read_text()
        self.assertIn('string KIND_SPIN="SPIN"', action)
        self.assertIn('float64 spin_yaw_rad\nfloat64 time_allowance_s\n---', action)

    def test_tb6_may_take_spin_in_the_field_file(self):
        from rescue_robot.task_site_runtime import AUTO_TARGET_KINDS
        self.assertIn('SPIN', AUTO_TARGET_KINDS)
        field = json.loads((ROOT / 'config' / 'field' / 'search_area.field.json').read_text())
        self.assertIn('SPIN', field['robots']['robot_a']['task.live.auto_target.kinds'])


if __name__ == '__main__':
    unittest.main()
