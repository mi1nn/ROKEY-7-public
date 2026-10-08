"""E3 (partial): real rclpy readiness sources (odom, LaserScan, map->odom TF) per namespace.

Not collected by scripts/run_tests.py. Run on an isolated graph, never a robot's domain:

    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_robot:$PYTHONPATH \
      python3 tests/ros/e3_readiness_probe.py   # not -I: ROS needs PYTHONPATH

Only /robot_a gets odom, scan, a compressed camera frame and its map->odom TF; /robot_b gets a TF edge with
other frame names. Expected: robot_a driver/sensors/slam True, robot_b all False,
and no watchdog flag on either (no source). Exit code 0 means all checks held.
"""
import os
import sys
import time
from uuid import UUID

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from tf2_msgs.msg import TFMessage

from rescue_robot.executor import Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.readiness import ReadinessMonitor
from rescue_robot.ros_node import bind_odometry, bind_readiness, readiness_policy

ROBOTS = (('robot_a', '/robot_a'), ('robot_b', '/robot_b'))  # = config/deployment.example.json
POLICY = Policy('e3-probe', 1, 3, .2, .11, .5, .01, .01)    # probe-only thresholds
CAMERAS = ('oakd/rgb/preview/image_raw/compressed',)  # TB4 OAK-D default kind (P4 DEC-004)
AGES = {'odom': .5, 'scan': .5, 'camera': 1., 'wheel_status': .5, 'map_to_odom_tf': 1.}


class FakePort:
    def send_goal(self, command_id, pose):
        raise AssertionError('probe sends no goal')

    def cancel_goal(self, command_id):
        raise AssertionError('probe cancels nothing')


def main():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if domain < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        sys.exit('refuse: need ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    spin = SingleThreadedExecutor()
    nodes, monitors, adapters = [], {}, {}
    for n, (robot_id, namespace) in enumerate(ROBOTS):
        executor = Executor(robot_id, str(UUID(int=n + 1)), str(UUID(int=n + 11)), POLICY,
                            allowed_robot_ids=[r for r, _ in ROBOTS])
        node = Node('rescue_command_adapter', namespace=namespace)
        adapter = CommandAdapter(executor, FakePort(), guards=lambda task: False, clock_error_bound_s=.5)
        monitor = ReadinessMonitor(readiness_policy('e3-probe', AGES, CAMERAS, ''))
        bind_odometry(node, adapter, 'odom', monitor)
        bind_readiness(node, monitor, adapter, scan_topic='scan', camera_topics=CAMERAS, wheel_status_topic='',
                       tf_topic='tf', map_frame='map', odom_frame='odom')
        nodes.append(node)
        spin.add_node(node)
        monitors[robot_id], adapters[robot_id] = monitor, adapter
    sim = Node('fake_robot_sources')
    spin.add_node(sim)
    odom_a = sim.create_publisher(Odometry, '/robot_a/odom', qos_profile_sensor_data)
    scan_a = sim.create_publisher(LaserScan, '/robot_a/scan', qos_profile_sensor_data)
    cam_a = sim.create_publisher(CompressedImage, '/robot_a/' + CAMERAS[0], qos_profile_sensor_data)
    tf_a = sim.create_publisher(TFMessage, '/robot_a/tf', 100)
    tf_b = sim.create_publisher(TFMessage, '/robot_b/tf', 100)

    def tf(parent, child):
        edge = TransformStamped()
        edge.header.stamp = sim.get_clock().now().to_msg()
        edge.header.frame_id, edge.child_frame_id = parent, child
        edge.transform.rotation.w = 1.0
        return TFMessage(transforms=[edge])

    deadline = time.monotonic() + 5.0
    publishers = (odom_a, scan_a, cam_a, tf_a, tf_b)
    while time.monotonic() < deadline and any(p.get_subscription_count() == 0 for p in publishers):
        spin.spin_once(timeout_sec=.05)
    for _ in range(6):
        odom = Odometry()
        odom.header.stamp = sim.get_clock().now().to_msg()
        odom_a.publish(odom)
        scan_a.publish(LaserScan())
        cam_a.publish(CompressedImage())
        tf_a.publish(tf('map', 'odom'))
        tf_b.publish(tf('odom', 'base_link'))   # not the slam edge
        end = time.monotonic() + .05
        while time.monotonic() < end:
            spin.spin_once(timeout_sec=.01)
    flags = {robot: monitors[robot].flags(adapters[robot].clock()) for robot in monitors}
    print({'domain': domain, 'flags': flags})
    ok = (flags['robot_a'] == {'driver': True, 'sensors': True, 'slam': True}
          and flags['robot_b'] == {'driver': False, 'sensors': False, 'slam': False})
    for node in nodes + [sim]:
        node.destroy_node()
    rclpy.try_shutdown()
    print('E3 readiness probe:', 'PASSED' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
