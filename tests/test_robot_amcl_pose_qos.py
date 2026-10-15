"""The executor receives AMCL's latched amcl_pose even if it was published before the two found each other
(simulation run 10, 2026-10-16, C15; no ROS).

Runbook 7.5 starts TB-6's AMCL (frozen_localization) after SLAM stops. AMCL published one amcl_pose at 04:36:33.79
(its first filter update after the initial pose) and none after, because TB-6 stood still. A topic echo got it
(AMCL's publisher is RELIABLE + TRANSIENT_LOCAL, depth 1), but the executor subscribed amcl_pose BEST_EFFORT +
VOLATILE (sensor data): a volatile reader gets only what is published after it has matched the new publisher, and
discovery of the just-started AMCL took longer than its first publish [추정, 부하 걸린 시뮬레이션 PC]. So the frozen
phase never had an amcl_pose: no covariance for the TF pose (#392), MapState UNKNOWN, no RobotPose, and the
coordinator held TB-6's RETURN_HOME (TRAFFIC_SCOUT_POSE_UNKNOWN). The executor now reads amcl_pose RELIABLE +
TRANSIENT_LOCAL depth 1, as AMCL publishes it, and gets the last one on matching. A pose from before a map switch
is still not counted (valid_after).

The test marked expectedFailure fails on main.
"""
import sys
import threading
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

from test_robot_live_map_lifecycle import settings

AM = dict(amcl_tf_topic='/robot6/tf', amcl_odom_frame='odom', amcl_base_frame='base_link', amcl_update_min_d=0.25,
          amcl_update_min_a=0.2)


def fake_ros():
    qos = ModuleType('rclpy.qos')

    class QoSProfile:
        def __init__(self, *, depth, reliability, durability):
            self.depth, self.reliability, self.durability = depth, reliability, durability
    qos.QoSProfile = QoSProfile
    qos.ReliabilityPolicy = NS(RELIABLE='RELIABLE', BEST_EFFORT='BEST_EFFORT')
    qos.DurabilityPolicy = NS(TRANSIENT_LOCAL='TRANSIENT_LOCAL', VOLATILE='VOLATILE')
    qos.qos_profile_sensor_data = QoSProfile(depth=5, reliability='BEST_EFFORT', durability='VOLATILE')
    modules = {'rclpy': ModuleType('rclpy'), 'rclpy.qos': qos}
    for package, names in (('nav_msgs', ('OccupancyGrid',)), ('geometry_msgs', ('PoseWithCovarianceStamped',)),
                           ('std_msgs', ('String',)), ('tf2_msgs', ('TFMessage',))):
        msg = ModuleType(package + '.msg')
        for name in names:
            setattr(msg, name, type(name, (), {}))
        modules[package] = ModuleType(package)
        modules[package + '.msg'] = msg
    return modules


class AmclPoseQos(unittest.TestCase):
    def subscriptions(self, mode, robot):
        from rescue_robot.live_sources import SavedMapSources, attach_sources
        subs = {}
        node = NS(create_subscription=lambda kind, topic, cb, qos: subs.setdefault(topic, qos) or object(),
                  create_timer=lambda period, cb: object(), create_publisher=lambda kind, topic, qos: object(),
                  get_logger=lambda: NS(warning=print, error=print, info=print))
        runtime = NS(adapter=NS(lock=threading.RLock(), wall_clock_ns=lambda: 1, executor=NS(active=None)),
                     node=node, config={'poll_period_s': 0.2}, input_handles=[])
        runtime.saved_maps = SavedMapSources(runtime, settings(mode, robot, **AM))
        with patch.dict(sys.modules, fake_ros()):
            attach_sources(runtime)
        return subs

    @unittest.expectedFailure
    def test_amcl_pose_is_read_as_amcl_publishes_it(self):
        for mode, robot, ns in (('SLAM_THEN_FROZEN', 'robot_a', '/robot6'), ('FROZEN_MAP_AMCL', 'robot_b', '/robot5')):
            with self.subTest(robot=robot):
                qos = self.subscriptions(mode, robot)[ns + '/amcl_pose']
                self.assertEqual(('RELIABLE', 'TRANSIENT_LOCAL', 1), (qos.reliability, qos.durability, qos.depth))


if __name__ == '__main__':
    unittest.main()
