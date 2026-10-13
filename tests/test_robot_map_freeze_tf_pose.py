"""map_freeze takes TB-6's pose from TF too (OPS-179), no ROS.

slam_toolbox publishes /robot6/pose only for the scans it processes (after enough movement), so
a stopped TB-6 has no fresh SLAM pose and the freeze (it needs a fresh pose: the start pose,
and the last freeze's AMCL start, frozen_pose) failed with 'SLAM pose is not fresh' exactly when
TB-6 stopped for it. Like the executor (#253, OPS-177), map_freeze composes TF map -> base_link
(slam_tf_pose.SlamTfPose) on a timer; the covariance comes from the last /robot6/pose and must be
younger than task.saved_map.slam_covariance_max_age_ns, else the pose does not count (no start
pose, no freeze). The frozen pose of the last freeze is that TF pose.
"""
import math
from pathlib import Path
import tempfile
import unittest

from rescue_robot.map_freeze import FreezeError, MapFreeze, tf_pose_fields

from test_robot_map_freeze import NOW, SESSION, grid

COV = [0.0] * 36
COV[0] = COV[7] = COV[35] = 0.01


def tf_pose(x=1.0, y=2.0, yaw=0.5, stamp_ns=NOW - 100_000_000):
    """slam_tf_pose.SlamTfPose.pose() output."""
    return dict(stamp_ns=stamp_ns, x=x, y=y, yaw=yaw, covariance=list(COV), covariance_fresh=True)


class TfPose(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.f = MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=0, pose_max_age_ns=2_000_000_000,
                           output_dir=Path(self.dir.name), wall_clock_ns=lambda: NOW, git_sha='abc')

    def test_fields(self):
        p = tf_pose_fields(tf_pose(yaw=math.pi / 2), 'map')
        self.assertEqual(('map', NOW - 100_000_000, (1.0, 2.0, 0.0)), (p['frame'], p['stamp_ns'], p['position']))
        self.assertAlmostEqual(math.sin(math.pi / 4), p['orientation'][2])
        self.assertEqual(COV, p['covariance'])

    def test_a_stopped_tb6_is_frozen_from_its_tf_pose(self):
        self.f.on_tf_pose(tf_pose_fields(tf_pose(x=0.5), 'map'), covariance_fresh=True)
        self.assertEqual(0.5, self.f.start_pose['position'][0])
        self.f.on_map(grid())
        self.f.freeze()
        self.f.on_tf_pose(tf_pose_fields(tf_pose(x=3.0), 'map'), covariance_fresh=True)
        last = self.f.freeze()
        self.assertEqual(3.0, last['pose']['position'][0])      # frozen_pose = the TF pose

    def test_an_old_slam_covariance_does_not_count(self):
        self.f.on_tf_pose(tf_pose_fields(tf_pose(), 'map'), covariance_fresh=False)
        self.assertIsNone(self.f.start_pose)                   # no start from it
        self.f.on_tf_pose(tf_pose_fields(tf_pose(x=0.5), 'map'), covariance_fresh=True)
        self.f.on_map(grid())
        self.f.freeze()
        self.f.on_tf_pose(tf_pose_fields(tf_pose(x=3.0), 'map'), covariance_fresh=False)
        with self.assertRaisesRegex(FreezeError, 'covariance'):
            self.f.freeze()


if __name__ == '__main__':
    unittest.main()
