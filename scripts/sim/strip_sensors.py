#!/usr/bin/env python3
"""Republish this namespace's robot_description without the Gazebo Sensors system (run_sim.sh, 2026-10-15).

The Create3 model (irobot_create_description create3.urdf.xacro) loads gz::sim::systems::Sensors in every robot.
With a second robot in the same Gazebo the second Sensors system builds a second render scene and the server
aborts ("A scene node with the name sun already exists", measured on the simulation PC 2026-10-15 in every two-robot run).
One Sensors system renders every sensor in the world, so the extra robots are spawned from this copy:
    robot_description  ->  robot_description_sim  (same text, that one <plugin> removed)
Nothing else in the description changes.
"""
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SENSORS_PLUGIN = re.compile(r'<plugin[^>]*gz-sim-sensors-system[^>]*>.*?</plugin>', re.DOTALL)


def strip(urdf):
    """(text without the Sensors plugin, how many were removed)."""
    return SENSORS_PLUGIN.subn('', urdf)


class StripSensors(Node):
    def __init__(self):
        super().__init__('strip_sensors')
        self.pub = self.create_publisher(String, 'robot_description_sim', LATCHED)
        self.sub = self.create_subscription(String, 'robot_description', self.on_description, LATCHED)

    def on_description(self, msg):
        text, removed = strip(msg.data)
        if removed != 1:
            self.get_logger().error(f'STRIP_SENSORS expected 1 Sensors plugin, found {removed}; not publishing')
            return
        self.get_logger().info('STRIP_SENSORS removed the Sensors system from robot_description')
        self.pub.publish(String(data=text))


def main():
    rclpy.init(args=sys.argv)
    try:
        rclpy.spin(StripSensors())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
