"""Common-frame RViz markers without merging native TF trees; synthetic E2 inputs."""
import unittest
from types import SimpleNamespace as NS

from rescue_robot.shared_map_view import pose_for_view


def pose(**changes):
    values = dict(robot_id='robot_b', mission_pose_status='VALID',
                  frame_ref=NS(mission_frame='map', map_session_id='run', map_revision=0,
                               transform_revision=1, alignment_status='VALID'),
                  time=NS(observed_at_ns=100, valid_until_ns=300, clock_state='SYNCED'),
                  mission_pose=NS(header=NS(frame_id='map'), pose=NS(pose=NS(
                      position=NS(x=1., y=2., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))))
    return NS(**(values | changes))


class ViewTests(unittest.TestCase):
    def check(self, message, now=200):
        return pose_for_view(message, robot_id='robot_b', frame='map', map_session_id='run',
                             map_revision=0, transform_revision=1, now_ns=now)

    def test_valid_pose_is_already_in_the_shared_map(self):
        self.assertEqual((1., 2., 0., 0., 0., 0., 1.), self.check(pose()))

    def test_unknown_stale_future_and_wrong_robot_are_not_drawn(self):
        self.assertIsNone(self.check(pose(mission_pose_status='UNKNOWN')))
        self.assertIsNone(self.check(pose(), now=300))
        self.assertIsNone(self.check(pose(), now=99))
        self.assertIsNone(self.check(pose(robot_id='robot_a')))

    def test_wrong_map_basis_and_nonfinite_pose_are_not_drawn(self):
        p = pose()
        p.frame_ref.map_session_id = 'other'
        self.assertIsNone(self.check(p))

    def test_unsynced_pose_and_nonunit_quaternion_are_not_drawn(self):
        p = pose()
        p.time.clock_state = 'UNKNOWN'
        self.assertIsNone(self.check(p))
        p = pose()
        p.mission_pose.pose.pose.orientation.w = 0.
        self.assertIsNone(self.check(p))
        p = pose()
        p.mission_pose.pose.pose.position.x = float('nan')
        self.assertIsNone(self.check(p))


if __name__ == '__main__':
    unittest.main()
