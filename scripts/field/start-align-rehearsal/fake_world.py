"""E3 fake TB-5 world for start_align: frozen map, LaserScan from the true pose, TF, odom, undocked (no robot).

usage: fake_world.py <true x> <true y> <true yaw>. A 6 x 4 m room with a box, a wall stub and a shelf (not
symmetric), 0.05 m cells; laser rplidar_link at base_link (-0.04, 0, 0.2) turned 90 deg (exercises the mount);
360 beams at 5 Hz, ray-cast once from the true pose. odom -> base_link identity, zero twist.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RES, W, H, OX, OY = 0.05, 120, 80, -3.0, -2.0
LASER = (-0.04, 0.0, math.pi / 2)        # base_link -> rplidar_link (x, y, yaw), z 0.2


def occupied(x, y):
    if x <= 0 or y <= 0 or x >= W - 1 or y >= H - 1:
        return True
    px, py = OX + (x + 0.5) * RES, OY + (y + 0.5) * RES
    return (1.0 <= px <= 1.5 and 0.5 <= py <= 1.2) or (-1.55 <= px <= -1.45 and py <= -0.5) \
        or (-0.6 <= px <= 0.2 and 1.15 <= py <= 1.25)


GRID = [[occupied(x, y) for x in range(W)] for y in range(H)]


def cast(x, y, a):
    c, s = math.cos(a), math.sin(a)
    r = 0.0
    while r < 12.0:
        r += 0.01
        gx, gy = math.floor((x + r * c - OX) / RES), math.floor((y + r * s - OY) / RES)
        if gx < 0 or gy < 0 or gx >= W or gy >= H or GRID[gy][gx]:
            return r
    return float('inf')


def main():
    truth = [float(v) for v in sys.argv[1:4]]
    rclpy.init()
    node = rclpy.create_node('e3_fake_world', namespace='/robot5')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution, grid.info.width, grid.info.height = RES, W, H
    grid.info.origin.position.x, grid.info.origin.position.y = OX, OY
    grid.info.origin.orientation.w = 1.0
    grid.data = [100 if GRID[y][x] else 0 for y in range(H) for x in range(W)]
    pub_map = node.create_publisher(OccupancyGrid, '/robot5/map', latched)
    pub_scan = node.create_publisher(LaserScan, '/robot5/scan', qos_profile_sensor_data)
    pub_tf = node.create_publisher(TFMessage, '/robot5/tf', 10)
    pub_static = node.create_publisher(TFMessage, '/robot5/tf_static', latched)
    pub_odom = node.create_publisher(Odometry, '/robot5/odom', qos_profile_sensor_data)
    pub_dock = node.create_publisher(DockStatus, '/robot5/dock_status', qos_profile_sensor_data)
    grid.header.stamp = node.get_clock().now().to_msg()
    pub_map.publish(grid)
    st = TransformStamped()
    st.header.stamp, st.header.frame_id, st.child_frame_id = node.get_clock().now().to_msg(), 'base_link', 'rplidar_link'
    st.transform.translation.x, st.transform.translation.z = LASER[0], 0.2
    st.transform.rotation.z, st.transform.rotation.w = math.sin(LASER[2] / 2), math.cos(LASER[2] / 2)
    pub_static.publish(TFMessage(transforms=[st]))
    lx = truth[0] + math.cos(truth[2]) * LASER[0]
    ly = truth[1] + math.sin(truth[2]) * LASER[0]
    lyaw = truth[2] + LASER[2]
    beams = 360
    ranges = [cast(lx, ly, lyaw - math.pi + 2 * math.pi * i / beams) for i in range(beams)]

    def fast():
        now = node.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.rotation.w = 1.0
        pub_tf.publish(TFMessage(transforms=[t]))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.pose.pose.orientation.w = 1.0
        pub_odom.publish(o)

    def scan():
        m = LaserScan()
        m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), 'rplidar_link'
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / beams
        m.angle_max = m.angle_min + m.angle_increment * (beams - 1)
        m.range_min, m.range_max = 0.15, 12.0
        m.ranges = ranges
        pub_scan.publish(m)
        pub_dock.publish(DockStatus(is_docked=False))
        pub_static.publish(TFMessage(transforms=[st]))

    node.create_timer(0.05, fast)
    node.create_timer(0.2, scan)
    node.create_timer(2.0, lambda: pub_map.publish(grid))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
