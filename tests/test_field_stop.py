"""Stopping a robot in the field, and not reusing an old SLAM map (OPS-183/184), no ROS.

Field report 2026-10-13 21:3x: Ctrl+C did not stop a robot driving to a search goal, and after
docking and a restart the SLAM map was the old one (an earlier slam_toolbox still running
[estimate]).

- scripts/field/stop_robot.sh <ns>: executor task_stop, cancel every Nav2 goal (empty
  CancelGoal on navigate_to_pose, navigate_through_poses, follow_waypoints = cancel all), a few
  zero cmd_vel, one line per step (a missing service says so), then the leftover nodes of <ns>
  and SLAM/Nav2/explore processes on this PC with their PIDs (kill by PID, never pkill -f).
- The command watchdog (the only writer of <ns>/cmd_vel under robot.launch.py) publishes zero
  commands when it is stopped (Ctrl+C / launch shutdown), before it goes away.
- tb6_slam.launch.py and mapping_navigation.launch.py refuse to start when <ns>/map already
  has a publisher, and name it: never continue an old map silently.
"""
import os
from pathlib import Path
import stat
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'stop_robot.sh'

FAKE_ROS2 = r'''#!/usr/bin/env python3
import os, sys
log = os.environ['FAKE_ROS2_LOG']
with open(log, 'a') as f:
    f.write(' '.join(sys.argv[1:]) + '\n')
args = sys.argv[1:]
services = os.environ.get('FAKE_SERVICES', '').split()
if args[:2] == ['service', 'list']:
    print('\n'.join(services))
elif args[:2] == ['service', 'call']:
    print('response:\n  success: True' if args[2] in services else 'service not available')
    sys.exit(0 if args[2] in services else 1)
elif args[:2] == ['node', 'list']:
    print('/robot6/slam_toolbox\n/robot6/bt_navigator\n/robot5/amcl')
'''


class StopScript(unittest.TestCase):
    def run_script(self, *args, services=()):
        if not SCRIPT.exists() and not (REPO / 'src').exists():
            self.skipTest('installed-wheel checkout: no source scripts')
        self.assertTrue(SCRIPT.exists(), 'scripts/field/stop_robot.sh missing')
        tmp = Path(tempfile.mkdtemp())
        fake = tmp / 'ros2'
        fake.write_text(FAKE_ROS2)
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
        env = dict(os.environ, PATH=f'{tmp}:{os.environ["PATH"]}', FAKE_ROS2_LOG=str(tmp / 'calls'),
                   FAKE_SERVICES=' '.join(services))
        done = subprocess.run(['bash', str(SCRIPT), *args], capture_output=True, text=True, env=env, timeout=60)
        calls = (tmp / 'calls').read_text().splitlines() if (tmp / 'calls').exists() else []
        return done, calls

    def test_stops_cancels_all_goals_and_zeroes(self):
        services = ['/robot6/rescue_command_adapter/task_stop', '/robot6/navigate_to_pose/_action/cancel_goal']
        done, calls = self.run_script('/robot6', services=services)
        self.assertEqual(0, done.returncode, done.stderr)
        out = done.stdout
        self.assertIn('service call /robot6/rescue_command_adapter/task_stop std_srvs/srv/Trigger', '\n'.join(calls))
        self.assertIn('service call /robot6/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal',
                      '\n'.join(calls))
        for missing in ('navigate_through_poses', 'follow_waypoints'):
            self.assertIn(f'/robot6/{missing}/_action/cancel_goal', out)
            self.assertNotIn(f'service call /robot6/{missing}', '\n'.join(calls))   # absent: said, not called
        pub = [c for c in calls if c.startswith('topic pub')]
        self.assertTrue(pub and '/robot6/cmd_vel geometry_msgs/msg/TwistStamped' in pub[0] and '--times' in pub[0], pub)
        self.assertIn('/robot6/slam_toolbox', out)
        self.assertNotIn('/robot5/amcl', out)                 # only this robot's nodes
        self.assertIn('kill -INT', out)
        self.assertNotIn('pkill', SCRIPT.read_text())

    def test_every_step_says_what_happened(self):
        done, _ = self.run_script('/robot6')
        lines = done.stdout.splitlines()
        for step in ('task_stop', 'navigate_to_pose', 'navigate_through_poses', 'follow_waypoints', 'cmd_vel'):
            self.assertTrue(any(step in line for line in lines), (step, done.stdout))

    def test_namespace_is_required_and_absolute(self):
        for args in ((), ('robot6',)):
            with self.subTest(args=args):
                done, calls = self.run_script(*args)
                self.assertNotEqual(0, done.returncode)
                self.assertEqual([], [c for c in calls if c.startswith('service call')])


class WatchdogZeroOnExit(unittest.TestCase):
    def test_zero_commands(self):
        from rescue_robot.watchdog_node import zero_commands

        def vec():
            return NS(x=9.0, y=9.0, z=9.0)

        def stamped():
            return NS(header=NS(stamp=None, frame_id='x'), twist=NS(linear=vec(), angular=vec()))
        out = zero_commands(stamped, stamped=True, stamp='now', count=3)
        self.assertEqual(3, len(out))
        for m in out:
            self.assertEqual('now', m.header.stamp)
            self.assertEqual([0.0] * 6, [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
                                         m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
        plain = zero_commands(lambda: NS(linear=vec(), angular=vec()), stamped=False, stamp=None, count=1)
        self.assertEqual(0.0, plain[0].linear.x)


class OldSlamRefused(unittest.TestCase):
    def test_a_map_publisher_already_there_is_named_and_refused(self):
        from rescue_bringup.map_guard import MapGuardError, refuse_existing_map_publishers
        refuse_existing_map_publishers('/robot6/map', [])
        info = [NS(node_namespace='/robot6', node_name='slam_toolbox')]
        with self.assertRaisesRegex(MapGuardError, '/robot6/slam_toolbox'):
            refuse_existing_map_publishers('/robot6/map', info)


if __name__ == '__main__':
    unittest.main()
