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
3. --depth-near M: the OAK-D rgbd_camera's depth gets its own near clip (<camera><depth_camera><clip>); the RGB clip
   stays. Nearer than M the depth is -inf (no return), as the real stereo gives 0. Real stereo cannot measure nearer than
   about 0.40 m: fx 512 px x baseline 0.075 m / 96 disparities [계산; 704 px aligned depth and DISPARITY_96 from
   docs/evidence/field-2026-10-13-oakd-robot6.log, OAK-D Pro baseline 7.5 cm]. Depth noise is added by
   wall_clock_relay.py: Gazebo's <camera><noise> does not reach the rgbd depth image (measured std 0.0000 m).
4. --lidar-rate HZ --lidar-min M --lidar-noise S: the rplidar publishes at HZ (real /robot6/scan 9 Hz [측정 2026-10-15];
   the description says 62 Hz), adds gaussian noise S [추정], and measures from M. With the description's 0.164 m the
   GPU lidar let rays whose true distance is under about 0.23 m through walls (0.164 / cos 45 deg on the cube face)
   [측정 2026-10-16, lidar_probe.py]; from 0.10 m it does not, and wall_clock_relay.py drops returns under the real
   0.164 m so the field sees what the real lidar gives.
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


RGBD_CAMERA = re.compile(r'(<sensor name="rgbd_camera" type="rgbd_camera">\s*<camera>)(.*?)(</camera>)', re.DOTALL)


def real_depth(urdf, near_m):
    """(text whose rgbd_camera depth has its own near clip, how many cameras were changed)."""
    def change(m):
        far = re.search(r'<far>([^<]*)</far>', m.group(2))
        extra = f'<depth_camera><clip><near>{near_m:g}</near><far>{far.group(1) if far else 100}</far></clip></depth_camera>'
        return m.group(1) + m.group(2) + extra + m.group(3)
    return RGBD_CAMERA.subn(change, urdf)


RPLIDAR = re.compile(r'(<sensor name="rplidar" type="gpu_lidar">)(.*?)(</sensor>)', re.DOTALL)


def real_lidar(urdf, rate_hz, min_m, noise_m):
    """(text whose rplidar has the given update rate, range floor and gaussian noise, how many lidars were changed)."""
    def change(m):
        body = re.sub(r'<update_rate>[^<]*</update_rate>', f'<update_rate>{rate_hz:g}</update_rate>', m.group(2), count=1)
        body = re.sub(r'(<range>\s*<min>)[^<]*(</min>)', rf'\g<1>{min_m:g}\g<2>', body, count=1)
        body = body.replace('</lidar>', f'<noise><type>gaussian</type><mean>0</mean><stddev>{noise_m:g}</stddev></noise>'
                                        '</lidar>', 1)
        return m.group(1) + body + m.group(3)
    return RPLIDAR.subn(change, urdf)


def patch(urdf, *, keep_sensors, wheel_sphere, depth_near_m=None, lidar_rate_hz=None, lidar_min_m=None,
          lidar_noise_m=None):
    """The simulation copy, or ValueError when the description is not the one expected (nothing is guessed)."""
    report = {'sensors_removed': 0, 'wheels_sphere': 0, 'depth_camera': 0, 'lidar': 0}
    if not keep_sensors:
        urdf, report['sensors_removed'] = strip(urdf)
        if report['sensors_removed'] != 1:
            raise ValueError(f'expected 1 Sensors plugin, found {report["sensors_removed"]}')
    if wheel_sphere:
        urdf, report['wheels_sphere'] = wheel_spheres(urdf)
        if report['wheels_sphere'] != 2:
            raise ValueError(f'expected 2 drive wheel cylinders, found {report["wheels_sphere"]}')
    if depth_near_m is not None:
        urdf, report['depth_camera'] = real_depth(urdf, depth_near_m)
        if report['depth_camera'] != 1:
            raise ValueError(f'expected 1 rgbd_camera, found {report["depth_camera"]}')
    lidar = (lidar_rate_hz, lidar_min_m, lidar_noise_m)
    if any(v is not None for v in lidar):
        if any(v is None for v in lidar):
            raise ValueError('--lidar-rate, --lidar-min and --lidar-noise go together')
        urdf, report['lidar'] = real_lidar(urdf, *lidar)
        if report['lidar'] != 1:
            raise ValueError(f'expected 1 rplidar, found {report["lidar"]}')
    return urdf, report


class StripSensors(Node):
    def __init__(self, keep_sensors, wheel_sphere, depth_near_m=None, lidar=(None, None, None)):
        super().__init__('strip_sensors')
        self.keep_sensors, self.wheel_sphere, self.depth_near_m, self.lidar = keep_sensors, wheel_sphere, depth_near_m, lidar
        self.pub = self.create_publisher(String, 'robot_description_sim', LATCHED)
        self.sub = self.create_subscription(String, 'robot_description', self.on_description, LATCHED)

    def on_description(self, msg):
        try:
            text, report = patch(msg.data, keep_sensors=self.keep_sensors, wheel_sphere=self.wheel_sphere,
                                 depth_near_m=self.depth_near_m, lidar_rate_hz=self.lidar[0],
                                 lidar_min_m=self.lidar[1], lidar_noise_m=self.lidar[2])
        except ValueError as exc:
            self.get_logger().error(f'STRIP_SENSORS {exc}; not publishing')
            return
        self.get_logger().info(f'STRIP_SENSORS robot_description_sim: Sensors system removed {report["sensors_removed"]}, '
                               f'drive wheel spheres {report["wheels_sphere"]}, depth camera {report["depth_camera"]} '
                               f'(near {self.depth_near_m}), lidar {report["lidar"]} (rate, min, noise {self.lidar})')
        self.pub.publish(String(data=text))


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    keep_sensors = '--keep-sensors-system' in args
    wheel_sphere = '--wheel-sphere' in args
    value = lambda flag: float(args[args.index(flag) + 1]) if flag in args else None   # noqa: E731
    rclpy.init(args=sys.argv)
    try:
        rclpy.spin(StripSensors(keep_sensors, wheel_sphere, value('--depth-near'),
                                (value('--lidar-rate'), value('--lidar-min'), value('--lidar-noise'))))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
