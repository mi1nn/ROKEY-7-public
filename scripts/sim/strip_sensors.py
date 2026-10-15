#!/usr/bin/env python3
"""Republish this namespace's robot_description as robot_description_sim, the copy Gazebo spawns (2026-10-15, -16).

Two changes, nothing else:
1. Without the Gazebo Sensors system (second and later robots; --keep-sensors-system for the first one):

The Create3 model (irobot_create_description create3.urdf.xacro) loads gz::sim::systems::Sensors in every robot.
With a second robot in the same Gazebo the second Sensors system builds a second render scene and the server
aborts ("A scene node with the name sun already exists", measured on the simulation PC 2026-10-15 in every two-robot run).
One Sensors system renders every sensor in the world, so the extra robots are spawned without it.
2. --wheel-sphere: the two drive wheels collide as spheres of the wheel radius instead of 1.5 cm wide cylinders.
   With the cylinders the simulated odom counted every turn 6.4 % too large (undock: odom 191 deg, ground truth
   179.5 deg; a commanded 360 deg turn: odom 361, truth 339; straight lines exact), which tilted the TB-6 SLAM map
   11.5 deg after undocking. The cylinder touches the floor along its width, so the robot turns about wheels that are
   effectively 0.248 m apart while odom uses 0.233 m [추정]. With spheres odom and truth agree to 0.01 deg
   [측정 2026-10-16, 3 undock + 3 dock + two 360 deg turns]. Whether the real Create3 odom has such an error is
   not recorded [미확인]; the real SLAM map has walls along its axes.
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



WHEEL_COLLISION = re.compile(r'(<link name="(?:left|right)_wheel">.*?<collision>\s*<geometry>\s*)<cylinder length="[^"]*" '
                             r'radius="([^"]*)"/>', re.DOTALL)


def wheel_spheres(urdf):
    """(text whose two drive wheels collide as spheres of the wheel radius, how many were changed).
    A cylinder of 1.5 cm width touches the floor along its width; a sphere touches at the wheel's mid-plane."""
    return WHEEL_COLLISION.subn(lambda m: f'{m.group(1)}<sphere radius="{m.group(2)}"/>', urdf)


def patch(urdf, *, keep_sensors, wheel_sphere):
    """The simulation copy, or ValueError when the description is not the one expected (nothing is guessed)."""
    report = {'sensors_removed': 0, 'wheels_sphere': 0}
    if not keep_sensors:
        urdf, report['sensors_removed'] = strip(urdf)
        if report['sensors_removed'] != 1:
            raise ValueError(f'expected 1 Sensors plugin, found {report["sensors_removed"]}')
    if wheel_sphere:
        urdf, report['wheels_sphere'] = wheel_spheres(urdf)
        if report['wheels_sphere'] != 2:
            raise ValueError(f'expected 2 drive wheel cylinders, found {report["wheels_sphere"]}')
    return urdf, report


class StripSensors(Node):
    def __init__(self, keep_sensors, wheel_sphere):
        super().__init__('strip_sensors')
        self.keep_sensors, self.wheel_sphere = keep_sensors, wheel_sphere
        self.pub = self.create_publisher(String, 'robot_description_sim', LATCHED)
        self.sub = self.create_subscription(String, 'robot_description', self.on_description, LATCHED)

    def on_description(self, msg):
        try:
            text, report = patch(msg.data, keep_sensors=self.keep_sensors, wheel_sphere=self.wheel_sphere)
        except ValueError as exc:
            self.get_logger().error(f'STRIP_SENSORS {exc}; not publishing')
            return
        self.get_logger().info(f'STRIP_SENSORS robot_description_sim: Sensors system removed {report["sensors_removed"]}, '
                               f'drive wheel spheres {report["wheels_sphere"]}')
        self.pub.publish(String(data=text))


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    keep_sensors = '--keep-sensors-system' in args
    wheel_sphere = '--wheel-sphere' in args
    rclpy.init(args=sys.argv)
    try:
        rclpy.spin(StripSensors(keep_sensors, wheel_sphere))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
