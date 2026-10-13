#!/usr/bin/env python3
"""Print a robot's AMCL pose as the field-file line for its AMCL start (map lifecycle, OPS-113).

    python3 scripts/field/capture_initial_pose.py /robot5/amcl_pose map

Trial run: put the robot on its floor mark, give AMCL the start on the TB-6 map in RViz
(2D Pose Estimate), let AMCL settle, then run this. It reads one PoseWithCovarianceStamped
and prints on stdout the line to add to robots.<id> in config/field/search_area.field.json:

    "initial_pose_xy_yaw": [x, y, yaw]

The pose and its x/y/yaw standard deviations go to stderr so the person can judge whether
AMCL had settled. The same works for the exit and the TB-6 docking approach point (profile
home_positions). Nothing is written to disk; exit 2 when no valid pose arrives in time.
"""
import argparse
import json
import math
import sys


class CaptureError(Exception):
    pass


def _planar(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(v) for v in values) or p.z != 0 or q.x != 0 or q.y != 0
            or not math.isclose(q.z * q.z + q.w * q.w, 1., abs_tol=1e-6)):
        raise CaptureError('pose is not a finite planar pose')
    return float(p.x), float(p.y), 2 * math.atan2(q.z, q.w)


def snippet(message, *, map_frame):
    """The field-file line for this AMCL pose; CaptureError if it is not on the map frame or not planar."""
    if message.header.frame_id != map_frame:
        raise CaptureError(f'pose frame {message.header.frame_id!r} is not {map_frame!r}')
    x, y, yaw = _planar(message.pose.pose)
    return '"initial_pose_xy_yaw": ' + json.dumps([round(x, 4), round(y, 4), round(yaw, 4)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pose_topic', help='e.g. /robot5/amcl_pose')
    parser.add_argument('map_frame', help='expected header.frame_id (the profile map_basis frame)')
    parser.add_argument('--timeout-s', type=float, default=15.0)
    args = parser.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node('capture_initial_pose')
    got = []
    # AMCL publishes its pose RELIABLE, TRANSIENT_LOCAL: the last pose arrives even if AMCL is idle.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped, args.pose_topic, got.append, qos)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout_s * 1e9)
        while not got and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not got:
            print(f'no pose on {args.pose_topic} within {args.timeout_s} s', file=sys.stderr)
            return 2
        message = got[-1]
        try:
            line = snippet(message, map_frame=args.map_frame)
        except CaptureError as exc:
            print(f'{args.pose_topic}: {exc}; nothing printed', file=sys.stderr)
            return 2
        c = message.pose.covariance
        print(f'{args.pose_topic} stamp={message.header.stamp.sec}.{message.header.stamp.nanosec:09d} '
              f'sd x={math.sqrt(max(c[0], 0)):.3f} m y={math.sqrt(max(c[7], 0)):.3f} m '
              f'yaw={math.sqrt(max(c[35], 0)):.3f} rad', file=sys.stderr)
        print(line)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
