"""LIVE_ROS RETURN_HOME (C1-c, user decision 5A 2026-10-13), no ROS.

TB-6 returns to its docking approach point, TB-5 to the exit. The destination is this robot's
own home from the profile (home_positions), passed by the robot launch as task.live.home_xy_yaw
and held as configured_destinations['home/<robot_id>'] on the declared map basis. The robot
takes RETURN_HOME only when its own policy kinds include it (least privilege per robot). No
automatic docking: docking stays the supervised procedure (the Dock action moves the base
outside the Nav2/watchdog velocity path).
"""
import unittest

import test_robot_live_auto_approach as approach
from rescue_robot.executor import ContractError
from rescue_robot.live_parameters import home_destinations

POLICY = dict(approach.POLICY, kinds=['RETURN_HOME'])


class LiveReturnHome(unittest.TestCase):
    def setUp(self):
        a = self.a = approach.LiveAutoApproach('test_policy_on_accepts_and_drives_to_the_approach_goal_in_the_native_frame')
        a.setUp()
        self.f, self.adapter, self.node, g = a.f, a.adapter, a.node, a.g
        g.kind, g.target_id, g.target_estimate_revision = 'RETURN_HOME', '', 0
        g.has_route = g.has_approval = False
        g.home_ref = 'home/' + g.robot_id
        self.g = g
        self.home = home_destinations(g.robot_id, [4.0, 0.0, 0.0], frame_id='mission',
                                      map_session_id=g.frame_ref.map_session_id, map_revision=0, transform_revision=1)

    def build(self, policy, homes=True):
        self.a.build(policy)
        if homes:
            self.a.runtime.inputs.config['configured_destinations'] = dict(self.home)

    def test_home_in_policy_and_configured_drives_home(self):
        self.build(POLICY)
        self.assertEqual('accept', self.a.submit(), self.node.errors)
        self.adapter.step()
        self.assertEqual(2., self.f.r.f.port.goals[0][1].x)  # mission 4 -> native 2 (declared x+2)

    def test_not_in_this_robots_kinds_is_refused(self):
        self.build(dict(POLICY, kinds=['APPROACH_TARGET']))
        self.assertEqual('reject', self.a.submit())

    def test_no_home_for_this_robot_is_refused(self):
        self.build(POLICY, homes=False)
        self.assertEqual('reject', self.a.submit())

    def test_other_home_ref_is_refused(self):
        self.build(POLICY)
        self.g.home_ref = 'home/robot_b'
        self.assertEqual('reject', self.a.submit())

    def test_home_destination_record(self):
        self.assertEqual({'home/robot_x': dict(frame_id='mission', robot_id='robot_x', map_session_id='s',
                                              map_revision=0, transform_revision=1, xy_yaw=[1.0, 2.0, 0.5])},
                         home_destinations('robot_x', [1.0, 2.0, 0.5], frame_id='mission', map_session_id='s',
                                           map_revision=0, transform_revision=1))
        for bad in ([1.0, 2.0], [1.0, 2.0, float('inf')], None):
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                home_destinations('robot_x', bad, frame_id='mission', map_session_id='s', map_revision=0,
                                  transform_revision=1)


if __name__ == '__main__':
    unittest.main()
