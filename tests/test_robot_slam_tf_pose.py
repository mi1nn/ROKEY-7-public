"""TB-6 pose while searching = TF map -> base_link (OPS-177), no ROS.

slam_toolbox publishes <ns>/pose only for the scans it processes, and it processes a scan only
after the robot moved far enough: a stopped TB-6 gets no new SLAM pose, so its RobotPose (valid
2 s) went stale exactly when the coordinator needs it (the map freeze asks for a stopped TB-6
with a fresh pose). Decision OPS-177: in the SLAM phase the pose comes from TF, map -> odom
(slam_toolbox, published periodically) composed with odom -> base_link (Create3, continuous),
read on the executor's poll period. This is the source of this phase, not a fallback. Its time
is the older of the two edges. The covariance still comes from the last <ns>/pose; older than
task.saved_map.slam_covariance_max_age_ns (30 s proposed) makes the pose not valid. No TF edge
or no SLAM pose yet: no pose at all (nothing is made up).
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_robot.executor import ContractError
from rescue_robot.live_sources import SavedMapSources
from rescue_robot.slam_tf_pose import SlamTfPose

from test_robot_live_map_lifecycle import COV, R0, Runtime, grid_message, settings

S = 1_000_000_000
AGE = 30 * S


def tf_message(*edges):
    """TFMessage-like: edges (parent, child, stamp_ns, x, y, yaw)."""
    transforms = []
    for parent, child, stamp_ns, x, y, yaw in edges:
        sec, nanosec = divmod(stamp_ns, S)
        transforms.append(NS(header=NS(frame_id=parent, stamp=NS(sec=sec, nanosec=nanosec)), child_frame_id=child,
                             transform=NS(translation=NS(x=x, y=y, z=0.0),
                                          rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
    return NS(transforms=transforms)


def spatial_tf_message(*edges):
    """TFMessage-like: (parent, child, stamp_ns, xyz, xyzw), without flattening."""
    message = tf_message(*((parent, child, stamp, 0, 0, 0)
                           for parent, child, stamp, xyz, xyzw in edges))
    for transform, (_, _, _, xyz, xyzw) in zip(message.transforms, edges):
        transform.transform.translation = NS(**dict(zip(('x', 'y', 'z'), xyz)))
        transform.transform.rotation = NS(**dict(zip(('x', 'y', 'z', 'w'), xyzw)))
    return message


def measured_tb6_tf(stamp_ns):
    """[측정, 2026-10-14] TB-6 map-freeze failure, stationary after undocking.

    Native TF values are unchanged; timestamps are rebased for deterministic E2 tests.
    This fixture supplies no field covariance or readiness evidence.
    """
    return spatial_tf_message(
        ('map', 'odom', stamp_ns,
         (0.02000981357266791, 1.4404437134362097e-05, 0.0014779624437087533),
         (-0.004797844888163447, 0.0025875895498232732, -1.2195844091517151e-05, 0.9999851423476311)),
        ('odom', 'base_link', stamp_ns - S // 10,
         (-0.14913737773895264, -0.0034079807810485363, -0.002282583387568593),
         (-0.0023667090572416782, -0.00456047710031271, 0.9999651908874512, 0.006571151316165924)))


class Compose(unittest.TestCase):
    def setUp(self):
        self.p = SlamTfPose(map_frame='map', odom_frame='odom', base_frame='base_link', covariance_max_age_ns=AGE)

    def test_map_to_base_is_map_to_odom_then_odom_to_base(self):
        self.p.on_tf(tf_message(('map', 'odom', 10 * S, 1.0, 0.0, math.pi / 2)))
        self.p.on_tf(tf_message(('odom', 'base_link', 11 * S, 1.0, 0.0, 0.0), ('base_link', 'laser', 11 * S, 0, 0, 0)))
        self.p.on_covariance(9 * S, COV)
        pose = self.p.pose(now_ns=12 * S)
        self.assertAlmostEqual(1.0, pose['x']); self.assertAlmostEqual(1.0, pose['y'])
        self.assertAlmostEqual(math.pi / 2, pose['yaw'])
        self.assertEqual((10 * S, COV, True), (pose['stamp_ns'], pose['covariance'], pose['covariance_fresh']))

    def test_measured_tb6_spatial_edges_produce_a_map_pose(self):
        self.assertEqual(0, self.p.on_tf(measured_tb6_tf(10 * S)))
        self.p.on_covariance(9 * S, COV)  # synthetic covariance; not an E4 observation
        pose = self.p.pose(now_ns=11 * S)
        self.assertIsNotNone(pose)
        self.assertEqual(10 * S - S // 10, pose['stamp_ns'])
        self.assertTrue(pose['covariance_fresh'])

    def test_compose_spatial_edges_before_projecting_to_the_map_plane(self):
        # Ry(30 deg) and Ry(-30 deg) cancel, but the second edge's height contributes
        # to map x. Flattening each edge first loses that contribution.
        s, c = math.sin(math.pi / 12), math.cos(math.pi / 12)
        self.p.on_tf(spatial_tf_message(
            ('map', 'odom', 10 * S, (1., 2., 0.3), (0., s, 0., c)),
            ('odom', 'base_link', 10 * S, (2., 1., 0.4), (0., -s, 0., c))))
        self.p.on_covariance(9 * S, COV)
        pose = self.p.pose(now_ns=11 * S)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(1 + 2 * math.cos(math.pi / 6) + 0.4 * math.sin(math.pi / 6), pose['x'])
        self.assertAlmostEqual(3., pose['y'])
        self.assertAlmostEqual(0., pose['yaw'])

    def test_the_older_edge_dates_the_pose(self):
        self.p.on_tf(tf_message(('map', 'odom', 20 * S, 0, 0, 0), ('odom', 'base_link', 15 * S, 0, 0, 0)))
        self.p.on_covariance(14 * S, COV)
        self.assertEqual(15 * S, self.p.pose(now_ns=21 * S)['stamp_ns'])

    def test_an_older_edge_never_replaces_a_newer_one(self):
        self.p.on_tf(tf_message(('map', 'odom', 20 * S, 2.0, 0, 0), ('odom', 'base_link', 20 * S, 0, 0, 0)))
        self.p.on_tf(tf_message(('map', 'odom', 19 * S, 9.0, 0, 0)))
        self.p.on_covariance(19 * S, COV)
        self.assertAlmostEqual(2.0, self.p.pose(now_ns=21 * S)['x'])

    def test_no_pose_without_both_edges_or_any_slam_pose(self):
        self.p.on_tf(tf_message(('map', 'odom', 10 * S, 0, 0, 0)))
        self.p.on_covariance(9 * S, COV)
        self.assertIsNone(self.p.pose(now_ns=11 * S))                 # no odom -> base_link
        p = SlamTfPose(map_frame='map', odom_frame='odom', base_frame='base_link', covariance_max_age_ns=AGE)
        p.on_tf(tf_message(('map', 'odom', 10 * S, 0, 0, 0), ('odom', 'base_link', 10 * S, 0, 0, 0)))
        self.assertIsNone(p.pose(now_ns=11 * S))                      # no SLAM pose: no covariance to give

    def test_an_old_slam_covariance_is_kept_but_not_fresh(self):
        self.p.on_tf(tf_message(('map', 'odom', 100 * S, 0, 0, 0), ('odom', 'base_link', 100 * S, 0, 0, 0)))
        self.p.on_covariance(60 * S, COV)
        self.assertTrue(self.p.pose(now_ns=90 * S)['covariance_fresh'])
        self.assertFalse(self.p.pose(now_ns=91 * S)['covariance_fresh'])

    def test_an_invalid_edge_is_not_used(self):
        for name, rotation in (('not a unit rotation', NS(x=0.3, y=0.0, z=0.0, w=0.3)),
                               ('not finite', NS(x=0.0, y=0.0, z=float('nan'), w=1.0))):
            with self.subTest(name):
                p = SlamTfPose(map_frame='map', odom_frame='odom', base_frame='base_link', covariance_max_age_ns=AGE)
                message = tf_message(('map', 'odom', 10 * S, 0, 0, 0))
                message.transforms[0].transform.rotation = rotation
                self.assertEqual(1, p.on_tf(message))                 # refused edges, counted for the log
                p.on_tf(tf_message(('odom', 'base_link', 10 * S, 0, 0, 0)))
                p.on_covariance(10 * S, COV)
                self.assertIsNone(p.pose(now_ns=11 * S))

    def test_a_tilted_robot_has_no_pose(self):
        # map -> base_link rolled 35 deg: the map-plane heading means nothing, no pose is made up
        message = tf_message(('map', 'odom', 10 * S, 0, 0, 0), ('odom', 'base_link', 10 * S, 0, 0, 0))
        message.transforms[0].transform.rotation = NS(x=0.3, y=0.0, z=0.0, w=math.sqrt(1 - 0.09))
        self.assertEqual(0, self.p.on_tf(message))
        self.p.on_covariance(10 * S, COV)
        self.assertIsNone(self.p.pose(now_ns=11 * S))
        self.assertAlmostEqual(2 * math.asin(0.3), self.p.tilt_rad)  # for the log

    def test_a_small_measured_tilt_is_kept(self):
        self.p.on_tf(measured_tb6_tf(10 * S))
        self.p.on_covariance(10 * S, COV)
        self.assertIsNotNone(self.p.pose(now_ns=11 * S))
        self.assertLess(self.p.tilt_rad, math.radians(1))           # [측정, 2026-10-14] about 0.5 deg

    def test_settings_are_checked(self):
        for bad in (dict(map_frame=''), dict(base_frame=None), dict(covariance_max_age_ns=0),
                    dict(covariance_max_age_ns=1.5)):
            values = dict(map_frame='map', odom_frame='odom', base_frame='base_link', covariance_max_age_ns=AGE)
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                SlamTfPose(**(values | bad))


class Searching(unittest.TestCase):
    """SavedMapSources takes the TF pose as TB-6's SLAM-phase pose; a stale covariance is not valid."""

    def setUp(self):
        self.r = Runtime()
        self.s = SavedMapSources(self.r, settings('SLAM_THEN_FROZEN'))
        self.s.receive_map('robot_a', grid_message())

    def tf_pose(self, covariance_fresh):
        from rescue_robot.slam_tf_pose import pose_message
        self.r.now += S
        pose = dict(stamp_ns=self.r.now - S // 10, x=1.0, y=2.0, yaw=0.5, covariance=COV,
                    covariance_fresh=covariance_fresh)
        self.s.receive_pose('robot_a', pose_message(pose, 'map'), source='slam',
                            covariance_fresh=pose['covariance_fresh'])

    def state(self):
        record = self.s.records()['robot_a']
        return record['localization_status'], record['map_revision']

    def test_a_tf_pose_with_a_recent_slam_covariance_is_valid_at_r0(self):
        self.tf_pose(True)
        self.assertEqual(('VALID', R0), self.state())

    def test_a_tf_pose_with_an_old_slam_covariance_is_not_valid(self):
        self.tf_pose(False)
        self.assertEqual(('UNKNOWN', R0), self.state())

    def test_settings_need_the_tf_source(self):
        for change in (dict(slam_tf_topic='tf'), dict(slam_tf_topic=None), dict(slam_odom_frame=''),
                       dict(slam_base_frame=''), dict(slam_covariance_max_age_ns=0)):
            with self.subTest(change=change), self.assertRaises(ContractError):
                SavedMapSources(Runtime(), settings('SLAM_THEN_FROZEN', **change))


if __name__ == '__main__':
    unittest.main()
