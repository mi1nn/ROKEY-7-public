"""A standing robot on the frozen map keeps a current pose (simulation 2026-10-16 run 3, no ROS).

After the freeze both robots localize with AMCL. Nav2 Jazzy AMCL publishes amcl_pose only when it updates
its filter: the robot moved update_min_d / update_min_a (TurtleBot4 localization.yaml 0.25 m / 0.2 rad),
a no-motion update was asked, or an initial pose was set. RobotPose went out only on amcl_pose, so a
standing TB-6 (after runbook 7.5) and a standing TB-5 had no pose after 2 s: the coordinator showed
TRAFFIC_SCOUT_POSE_UNKNOWN, and TB-5 waiting for a support dispatch would be refused the same way
(support_pose_max_age_s 2 s).

As in the SLAM phase (OPS-177) the pose of the frozen phase is read from TF on the poll period: map -> odom
(AMCL, sent again with every scan) composed with odom -> base_link (Create3). Its time is the older edge.
TF has no covariance: it is the last amcl_pose's, which is AMCL's current state as long as the robot has
moved less than AMCL's update thresholds since then (AMCL has not updated, so nothing newer exists). Past
them without a new amcl_pose it is not valid. An amcl_pose from before a map switch never counts. Nothing
else stands in for a missing value.

Fixed here: SavedMapSources.receive_tf / tick_tf_pose with slam_tf_pose.AmclTfPose.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_robot.live_sources import SavedMapSources

from test_robot_live_map_lifecycle import COV, R0, Runtime, grid_message, pose_message, settings
from test_robot_live_map_twice import G1, R1, R2, revision_message
from test_robot_slam_tf_pose import S, tf_message

AMCL_KEYS = dict(amcl_tf_topic='/robot5/tf', amcl_odom_frame='odom', amcl_base_frame='base_link',
                 amcl_update_min_d=0.25, amcl_update_min_a=0.2)


class FrozenRobotB:
    """TB-5 on the frozen map with its start aligned (helpers only)."""

    def setUp(self):
        self.r = Runtime()
        self.s = SavedMapSources(self.r, settings('FROZEN_MAP_AMCL', 'robot_b', **AMCL_KEYS))
        self.s.receive_start_align('robot_b', NS(data='{"status": "ALIGNED", "reason": ""}'))
        self.s.receive_frozen(G1)
        self.s.receive_frozen_revision(revision_message(G1, R1))
        self.s.receive_map('robot_b', G1)

    def tf(self, odom_x=0.0, odom_yaw=0.0):
        """AMCL map -> odom and Create3 odom -> base_link, both stamped now."""
        self.s.receive_tf('robot_b', tf_message(('map', 'odom', self.r.now, 1.0, 2.0, 0.0),
                                                ('odom', 'base_link', self.r.now, odom_x, 0.0, odom_yaw)))

    def amcl(self):
        """An amcl_pose stamped by its scan a little before it arrives (after the map switch)."""
        self.r.now += S // 5
        self.s.receive_pose('robot_b', pose_message(self.r.now - S // 20))

    def wait(self, seconds):
        self.r.now += int(seconds * S)

    def record(self):
        return self.s.records()['robot_b']


class StandingOnTheFrozenMap(FrozenRobotB, unittest.TestCase):
    def test_a_standing_robot_keeps_a_fresh_valid_pose(self):
        self.tf()
        self.amcl()
        self.assertEqual('VALID', self.record()['localization_status'])
        self.wait(5)                                   # no amcl_pose: AMCL does not update a standing robot
        self.tf()                                      # TF keeps coming
        self.s.tick_tf_pose('robot_b')
        record = self.record()
        self.assertEqual('VALID', record['localization_status'])
        self.assertEqual(self.r.now, record['observed_at_ns'])

    def test_the_tf_pose_is_map_odom_then_odom_base(self):
        self.tf()
        self.amcl()
        self.wait(1)
        self.tf(odom_x=0.1)
        poses = []
        self.r.adapter.executor.robot_boot_id = 'boot'
        self.s.publisher = NS(robot='robot_b', on_pose=lambda m, observed, valid: poses.append((m, valid)),
                              publish_map_state=lambda **_: None)
        self.s.tick_tf_pose('robot_b')
        message, valid = poses[-1]
        self.assertEqual((1.1, 2.0), (round(message.pose.pose.position.x, 6), message.pose.pose.position.y))
        self.assertTrue(valid)

    def test_moved_past_the_amcl_threshold_without_a_new_amcl_pose_is_not_valid(self):
        self.tf()
        self.amcl()
        self.wait(1)
        self.tf(odom_x=0.24)                           # under 0.25 m: AMCL has not updated, still its state
        self.s.tick_tf_pose('robot_b')
        self.assertEqual('VALID', self.record()['localization_status'])
        self.wait(1)
        self.tf(odom_x=0.30)                           # past 0.25 m and no new amcl_pose
        self.s.tick_tf_pose('robot_b')
        self.assertEqual('UNKNOWN', self.record()['localization_status'])
        self.wait(1)
        self.tf(odom_yaw=0.25)                         # turned past 0.2 rad
        self.amcl()                                    # AMCL updates: valid again
        self.assertEqual('VALID', self.record()['localization_status'])

    def test_no_amcl_pose_yet_gives_no_tf_pose(self):
        self.tf()
        self.s.tick_tf_pose('robot_b')
        self.assertEqual(('UNKNOWN', 0), (self.record()['localization_status'], self.record()['observed_at_ns']))

    def test_an_amcl_pose_from_before_a_map_switch_does_not_count(self):
        self.tf()
        self.amcl()
        self.s.receive_frozen(G1)
        self.s.receive_frozen_revision(revision_message(G1, R2))   # the second freeze: TB-5 takes R2
        self.wait(1)
        self.tf()
        self.s.tick_tf_pose('robot_b')
        self.assertEqual(('UNKNOWN', R2), (self.record()['localization_status'], self.record()['map_revision']))

    def test_nothing_while_tb6_is_still_searching(self):
        """SLAM_THEN_FROZEN before its last freeze: the SLAM TF pose is the source (OPS-177), not this one."""
        r = Runtime()
        s = SavedMapSources(r, settings('SLAM_THEN_FROZEN', 'robot_a', **dict(AMCL_KEYS, amcl_tf_topic='/robot6/tf')))
        s.receive_map('robot_a', grid_message())
        s.receive_tf('robot_a', tf_message(('map', 'odom', r.now, 0.0, 0.0, 0.0),
                                           ('odom', 'base_link', r.now, 0.0, 0.0, 0.0)))
        s.tick_tf_pose('robot_a')
        self.assertEqual((0, R0), (s.records()['robot_a']['observed_at_ns'], s.records()['robot_a']['map_revision']))


class AmclGetsItsNextScan(FrozenRobotB, unittest.TestCase):
    """Runs 14 and 15 (C24): TB-6's RETURN_HOME after the handover was canceled twice in each run by the task guard,
    'localization uncertain', for a pose UNKNOWN 0.02-0.09 s. TB-6 was turning in place; each time the poll saw it
    past update_min_a before AMCL had taken its next scan and sent the new amcl_pose. AMCL updates on the first
    scan after a threshold, so for AMCL_UPDATE_GRACE_S [제안] after the robot is first seen past one the last
    covariance is still AMCL's state. Without a new amcl_pose by then the pose is not valid, as before."""

    def grace(self):
        from rescue_robot.slam_tf_pose import AMCL_UPDATE_GRACE_S
        return AMCL_UPDATE_GRACE_S

    @unittest.expectedFailure
    def test_turning_in_place_with_amcl_a_scan_behind_stays_valid(self):
        self.tf()
        self.amcl()
        states, yaw, anchor = [], 0.0, 0.0
        for _ in range(200):                            # 10 s, polls every 0.05 s, turning 0.07 rad/s (run 15)
            self.wait(0.05)
            yaw += 0.0035
            self.tf(odom_yaw=yaw)
            self.s.tick_tf_pose('robot_b')
            states.append(self.record()['localization_status'])
            if yaw - anchor >= 0.2 + 0.0035:            # AMCL's next scan after the threshold, 0.07 s later
                self.r.now += S // 20
                self.s.receive_pose('robot_b', pose_message(self.r.now - S // 50))
                anchor = yaw
        self.assertEqual({'VALID'}, set(states))

    @unittest.expectedFailure
    def test_just_past_a_threshold_is_still_valid(self):
        self.tf()
        self.amcl()
        self.wait(1)
        self.tf(odom_yaw=0.25)
        self.s.tick_tf_pose('robot_b')
        self.assertEqual('VALID', self.record()['localization_status'])

    @unittest.expectedFailure
    def test_no_amcl_pose_within_the_grace_is_not_valid(self):
        self.tf()
        self.amcl()
        self.wait(1)
        self.tf(odom_x=0.30)                           # first seen past 0.25 m
        self.s.tick_tf_pose('robot_b')
        self.wait(self.grace() + 0.05)
        self.tf(odom_x=0.35)
        self.s.tick_tf_pose('robot_b')
        self.assertEqual('UNKNOWN', self.record()['localization_status'])


class AskAmclForAPose(unittest.TestCase):
    """Run 10 (C15): after the map switch the frozen phase waits for an amcl_pose taken after it; a standing robot gets
    none (Nav2 AMCL updates on motion), so the pose stayed UNKNOWN for 375 s until a person called
    request_nomotion_update. The executor now asks AMCL itself, as start_align does for TB-5: 2 s after the switch
    with no amcl_pose yet, then every 5 s until one comes [제안]. The robot is not moved."""

    def setUp(self):
        self.r = Runtime()
        self.s = SavedMapSources(self.r, settings('FROZEN_MAP_AMCL', 'robot_b', **AMCL_KEYS))
        self.asked = []
        self.s.request_nomotion = self.asked.append
        self.s.receive_start_align('robot_b', NS(data='{"status": "ALIGNED", "reason": ""}'))
        self.s.receive_frozen(G1)
        self.s.receive_frozen_revision(revision_message(G1, R1))
        self.s.receive_map('robot_b', G1)

    def tick(self, seconds):
        self.r.now += int(seconds * S)
        self.s.tick_tf_pose('robot_b')

    def test_asks_after_2_s_then_every_5_s_until_an_amcl_pose_comes(self):
        self.tick(1.0)
        self.assertEqual([], self.asked)
        self.tick(1.2)
        self.assertEqual(['robot_b'], self.asked)
        self.tick(2.0)
        self.assertEqual(1, len(self.asked))
        self.tick(3.1)
        self.assertEqual(2, len(self.asked))
        self.s.receive_pose('robot_b', pose_message(self.r.now - S // 20))   # AMCL answered
        self.tick(10.0)
        self.assertEqual(2, len(self.asked))

    def test_nothing_is_asked_before_the_frozen_grid_is_loaded(self):
        r = Runtime()
        s = SavedMapSources(r, settings('FROZEN_MAP_AMCL', 'robot_b', **AMCL_KEYS))
        s.request_nomotion = self.asked.append
        r.now += 10 * S
        s.tick_tf_pose('robot_b')
        self.assertEqual([], self.asked)


class Values(unittest.TestCase):
    def test_thresholds_must_be_positive_numbers(self):
        from rescue_robot.executor import ContractError
        from rescue_robot.slam_tf_pose import AmclTfPose
        for d, a in ((0.0, 0.2), (0.25, -1.0), (None, 0.2), (True, 0.2), (float('nan'), 0.2)):
            with self.subTest(d=d, a=a), self.assertRaises(ContractError):
                AmclTfPose(map_frame='map', odom_frame='odom', base_frame='base_link', update_min_d=d, update_min_a=a)

    def test_launch_takes_amcls_own_thresholds(self):
        from rescue_bringup.field_launch import FieldLaunchError, amcl_update_thresholds
        from test_bringup_no_backup import REPO, load_yaml
        vendor = REPO / 'src' / 'vendor' / 'turtlebot4' / 'turtlebot4_navigation' / 'config' / 'localization.yaml'
        if vendor.exists():   # the TurtleBot4 file frozen_localization.launch.py uses by default
            self.assertEqual({'task.saved_map.amcl_update_min_d': 0.25, 'task.saved_map.amcl_update_min_a': 0.2},
                             amcl_update_thresholds(load_yaml(vendor)))
        for bad in ({}, {'amcl': {'ros__parameters': {'update_min_d': 0.25}}},
                    {'amcl': {'ros__parameters': {'update_min_d': 0.0, 'update_min_a': 0.2}}}):
            with self.subTest(bad=bad), self.assertRaises(FieldLaunchError):
                amcl_update_thresholds(bad)
        launch = REPO / 'src' / 'rescue_bringup' / 'launch' / 'robot.launch.py'
        if launch.exists():
            text = launch.read_text()
            self.assertIn('amcl_update_thresholds(yaml.safe_load(Path(localization_params).read_text()))', text)
            self.assertIn("DeclareLaunchArgument('localization_params_file'", text)


if __name__ == '__main__':
    unittest.main()
