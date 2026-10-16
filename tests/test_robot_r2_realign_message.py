"""TB-5 re-aligning on R2 does not take its executor down (C26, runs 16 and 17; no ROS).

At the second freeze TB-5 re-aligns: once its map topic carries R2, AMCL gets TB-5's last valid pose on
<ns>/initialpose (SavedMapSources._load -> reinitialize). That pose has come from TF since #430 (tick_tf_pose,
slam_tf_pose.pose_message without a message factory: plain look-alike objects), and reinitialize put its
``pose.pose`` into the PoseWithCovarianceStamped as it was. rclpy's C conversion then found no
geometry_msgs Pose there and aborted the process (exit code -6; Python cannot catch it):
``geometry_msgs__msg__pose__convert_from_py: Assertion ... failed`` right after
``robot_b map is not the frozen map; not loaded`` (R1's grid dropped at the adoption, then R2 arrived). TB-5 then
had no pose and no executor for the rest of the run (SUPPORT_NOT_LOCALIZED).

Now the message is built field by field from numbers (initial_pose_message), so whatever the last pose was made
of, only floats reach rclpy. If the re-initialization still fails, TB-5's map is blocked (MAP_REALIGN_FAILED):
its pose is not VALID, so it takes no command (AGENTS 6), and the executor keeps running.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_robot.slam_tf_pose import pose_message as tf_pose_message

from test_robot_live_map_lifecycle import COV
from test_robot_live_map_twice import G2, R2, TwiceHelpers


class Point:
    __slots__ = ('x', 'y', 'z')


class Quaternion:
    __slots__ = ('x', 'y', 'z', 'w')


class Pose:
    def __init__(self):
        self.position, self.orientation = Point(), Quaternion()


class PoseWithCovarianceStamped:
    """Stands in for the generated message; ``publish`` checks what rclpy's C conversion asserts."""

    def __init__(self):
        self.header = NS(frame_id='', stamp=None)
        self.pose = NS(pose=Pose(), covariance=[0.0] * 36)


def publish(message):
    pose = message.pose.pose
    if type(pose) is not Pose or type(pose.position) is not Point or type(pose.orientation) is not Quaternion:
        raise AssertionError('geometry_msgs__msg__pose__convert_from_py: abort (process killed)')
    values = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w, *message.pose.covariance]
    if not all(type(v) is float for v in values):
        raise AssertionError('not a float: abort')


class RealignFromATfPose(TwiceHelpers, unittest.TestCase):
    def tf_pose(self, x):
        """TB-5's pose as tick_tf_pose gives it: a look-alike from slam_tf_pose.pose_message."""
        self.r.now += 1_000_000
        yaw = 0.3
        message = tf_pose_message(dict(stamp_ns=self.r.now - 500_000, x=x, y=0.4, yaw=yaw, covariance=COV), 'map')
        self.s.receive_pose(self.robot, message, source='amcl_tf', covariance_fresh=True)

    def test_the_initial_pose_from_a_tf_pose_is_a_real_message(self):
        from rescue_robot.live_sources import initial_pose_message
        s = self.tb5_on_r1()
        self.tf_pose(2.5)
        self.assertEqual('VALID', self.state()[0])
        sent = []

        def reinitialize(robot, last):
            message = initial_pose_message(last, stamp='now', factory=PoseWithCovarianceStamped)
            publish(message)
            sent.append(message)
        s.reinitialize = reinitialize
        self.freeze(G2, R2)
        s.receive_map('robot_b', G2)
        message = sent[0]
        q = message.pose.pose.orientation
        self.assertEqual(('map', 'now', 2.5, 0.4, 0.3, COV),
                         (message.header.frame_id, message.header.stamp, message.pose.pose.position.x,
                          message.pose.pose.position.y, round(2 * math.atan2(q.z, q.w), 9),
                          message.pose.covariance))

    def test_a_failed_re_initialization_blocks_the_map_and_keeps_the_executor(self):
        s = self.tb5_on_r1()
        self.pose(x=2.5)

        def reinitialize(robot, last):
            raise AttributeError("'types.SimpleNamespace' object has no attribute 'position'")
        s.reinitialize = reinitialize
        self.freeze(G2, R2)
        s.receive_map('robot_b', G2)               # before: the exception left the map callback
        self.assertTrue(any('MAP_REALIGN_FAILED robot_b' in line for line in self.r.errors))
        self.pose()
        self.assertEqual(('UNKNOWN', R2), self.state())   # no command is taken without a valid pose
        self.assertNotIn('robot_b', s.maps)


if __name__ == '__main__':
    unittest.main()
