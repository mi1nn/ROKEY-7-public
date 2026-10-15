#!/usr/bin/env python3
"""Drive the virtual robot along the walls so SLAM maps the world (simulation track stage 1, 2026-10-15).

A stand-in for hand teleop: no key press is needed, so a run can be repeated. It reads only the robot's own
LaserScan and publishes velocity, like a person at the keyboard would. It never reads the real map or the
simulator's ground truth. Right-hand wall following: keep the right wall about --wall m away, turn left in place
when something is ahead, and go forward-right when no wall is in reach.

    drive_wall.py --namespace robot6 --seconds 600      # seconds of simulation time (use_sim_time)

The robot starts docked (the TurtleBot4 spawn launch puts it on its dock), and a docked Create3 ignores velocity
commands, so the script sends the Undock action first (--no-undock to skip). The lidar frame is turned +90 deg
from the robot's base_link (TurtleBot4 URDF, rplidar_link rpy 0 0 pi/2): robot-forward is scan angle -90 deg.

Run it in the simulation domain (run_sim.sh prints the environment lines), e.g. domain 77.
Stops the robot (zero velocity) on exit. Needs rclpy (ROS 2 Jazzy).
"""
import argparse
import math

from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Undock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FRONT_STOP_M = 0.45   # something this close ahead: turn left in place [설정 2026-10-15]
SPEED = 0.18          # m/s forward [설정]; slam_toolbox needs to see movement (minimum_travel 0.1 m)
TURN = 0.6            # rad/s turning in place [설정]
GAIN = 1.5            # rad/s per m of error from the wall [설정]
MAX_W = 0.6           # rad/s limit while following [설정]


LIDAR_YAW_DEG = 90.0  # rplidar_link is turned this much from base_link [측정 2026-10-15, tf2_echo in the simulation]


def sector_min(scan, lo_deg, hi_deg):
    """Nearest valid range in the angle window [lo, hi] degrees in the ROBOT frame (0 = ahead, positive = left)."""
    best = math.inf
    for i, r in enumerate(scan.ranges):
        a = math.degrees(scan.angle_min + i * scan.angle_increment) + LIDAR_YAW_DEG
        a = (a + 180.0) % 360.0 - 180.0
        if lo_deg <= a <= hi_deg and scan.range_min <= r <= scan.range_max:
            best = min(best, r)
    return best


class WallDriver(Node):
    def __init__(self, namespace, wall, seconds, speed):
        super().__init__('drive_wall', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.wall, self.seconds, self.speed = wall, seconds, speed
        self.pub = self.create_publisher(TwistStamped, f'/{namespace}/cmd_vel', 10)
        self.create_subscription(LaserScan, f'/{namespace}/scan', self.on_scan, qos_profile_sensor_data)
        self.start = None
        self.done = False
        self.last_log = 0.0

    def send(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def on_scan(self, scan):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start is None:
            self.start = now
        if now - self.start >= self.seconds:
            self.send(0.0, 0.0)
            self.done = True
            return
        front = sector_min(scan, -25, 25)
        front_right = sector_min(scan, -70, -25)
        right = sector_min(scan, -100, -70)
        if front < FRONT_STOP_M or front_right < 0.3:
            v, w, why = 0.0, TURN, 'turn left'
        elif right > 1.0 and front_right > 1.0:
            v, w, why = self.speed, -0.4, 'look for wall'
        else:
            dist = min(right, front_right * math.cos(math.radians(45)))
            w = max(-MAX_W, min(MAX_W, -GAIN * (dist - self.wall)))
            v, why = self.speed, 'follow'
        self.send(v, w)
        if now - self.last_log > 10.0:
            self.last_log = now
            self.get_logger().info(f'{now - self.start:5.0f}/{self.seconds:.0f} s  front {front:.2f} '
                                   f'front_right {front_right:.2f} right {right:.2f}  {why}')


def undock(node, namespace, timeout_s=60.0):
    """Send Undock and wait for the result. False when refused (the robot is not docked) or timed out."""
    client = ActionClient(node, Undock, f'/{namespace}/undock')
    if not client.wait_for_server(timeout_sec=20.0):
        return False
    goal = client.send_goal_async(Undock.Goal())
    rclpy.spin_until_future_complete(node, goal, timeout_sec=timeout_s)
    handle = goal.result() if goal.done() else None
    if handle is None or not handle.accepted:
        return False
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout_s)
    return result.done()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--namespace', default='robot6', help='robot name without a slash')
    parser.add_argument('--seconds', type=float, required=True, help='how long to drive, in simulation seconds')
    parser.add_argument('--wall', type=float, default=0.45, help='distance to keep from the right wall, m')
    parser.add_argument('--speed', type=float, default=SPEED, help='forward speed, m/s')
    parser.add_argument('--no-undock', action='store_true', help='do not send the Undock action first')
    args = parser.parse_args()
    rclpy.init()
    node = WallDriver(args.namespace, args.wall, args.seconds, args.speed)
    if not args.no_undock and not undock(node, args.namespace):
        node.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit('drive_wall: Undock was not accepted or did not finish (already undocked? use --no-undock)')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
