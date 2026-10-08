"""E3 (partial): real rclpy + nav_msgs/Odometry topics -> two CommandAdapters.

Not collected by scripts/run_tests.py (needs a sourced ROS 2 environment).
Run with an isolated graph, never a robot's domain:

    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_robot:$PYTHONPATH \
      python3 tests/ros/e3_odom_probe.py   # not -I: ROS needs PYTHONPATH

Checks: /robot_a/odom only reaches robot_a's executor and /robot_b/odom only
robot_b's; zero-velocity samples confirm STOPPED, a moving sample does not.
Nav2 actions are not exercised here (FakePort); exit code 0 means all checks held.
"""
import os
import sys
import time
from uuid import UUID

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rescue_robot.executor import Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.ros_node import bind_odometry

ROBOTS = (('robot_a', '/robot_a'), ('robot_b', '/robot_b'))  # = config/deployment.example.json
POLICY = Policy('e3-probe', 1, 3, .2, .11, .5, .01, .01)  # probe-only thresholds


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
    adapters, pubs, nodes = {}, {}, []
    for n, (robot_id, namespace) in enumerate(ROBOTS):
        executor = Executor(robot_id, str(UUID(int=n + 1)), str(UUID(int=n + 11)), POLICY,
                            allowed_robot_ids=[r for r, _ in ROBOTS])
        node = Node('rescue_command_adapter', namespace=namespace)
        adapter = CommandAdapter(executor, FakePort(), guards=lambda task: False,
                                 clock_error_bound=.5)  # probe-only bound
        bind_odometry(node, adapter, 'odom')
        sim = Node('fake_odom', namespace=namespace)
        pubs[robot_id] = sim.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        adapters[robot_id] = adapter
        nodes += [node, sim]
        spin.add_node(node)
        spin.add_node(sim)

    def publish(robot_id, linear):
        msg = Odometry()
        msg.header.stamp = nodes[0].get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.twist.linear.x = linear
        pubs[robot_id].publish(msg)

    deadline = time.monotonic() + 5.0  # discovery on localhost
    while time.monotonic() < deadline and any(p.get_subscription_count() == 0 for p in pubs.values()):
        spin.spin_once(timeout_sec=.05)
    # robot_a stands still, robot_b moves.
    for _ in range(8):
        publish('robot_a', 0.0)
        publish('robot_b', 0.3)
        end = time.monotonic() + .05
        while time.monotonic() < end:
            spin.spin_once(timeout_sec=.01)
    for _ in range(10):
        spin.spin_once(timeout_sec=.01)
    a, b = adapters['robot_a'].executor, adapters['robot_b'].executor
    report = {'domain': domain, 'robot_a.motion': a.motion, 'robot_b.motion': b.motion,
              'robot_a.last_sample': a.last_sample is not None, 'robot_b.last_sample': b.last_sample is not None,
              'unusable': {r: sum(e[0] == 'ODOM_UNUSABLE' for e in ad.events) for r, ad in adapters.items()}}
    print(report)
    ok = a.motion == 'STOPPED' and b.motion == 'MOVING' and report['unusable'] == {'robot_a': 0, 'robot_b': 0}
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()
    print('E3 odom probe:', 'PASSED' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
