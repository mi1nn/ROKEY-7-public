#!/usr/bin/env python3
"""Print the occupancy hash of one real map message, for robot.launch.py occupancy_sha256.

    python3 scripts/field/map_occupancy_sha256.py /robot6/map map

Reads exactly one nav_msgs/OccupancyGrid from the map topic (RELIABLE, TRANSIENT_LOCAL, as the
localization map server publishes it) and prints rescue_robot.live_sources.Grid(...).sha256,
the hash task_input compares against. Same check as #132 start_robot.sh: the map frame must be
the expected frame (identity alignment is declared). Nothing is printed on stdout and the exit
code is 2 when no valid map arrives within the timeout; nothing is written to disk.
"""
import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('map_topic')
    parser.add_argument('map_frame', help='expected header.frame_id (the profile map_basis frame)')
    parser.add_argument('--timeout-s', type=float, default=15.0)
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_robot.executor import ContractError
    from rescue_robot.live_sources import Grid

    rclpy.init()
    node = rclpy.create_node('map_occupancy_sha256')
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, args.map_topic, got.append, qos)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout_s * 1e9)
        while not got and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not got:
            print(f'no map on {args.map_topic} within {args.timeout_s} s; no hash', file=sys.stderr)
            return 2
        try:
            grid = Grid(got[0])
        except ContractError as exc:
            print(f'invalid map on {args.map_topic}: {exc}; no hash', file=sys.stderr)
            return 2
        if grid.frame != args.map_frame:
            print(f'map frame {grid.frame!r} is not {args.map_frame!r}; refusing identity alignment', file=sys.stderr)
            return 2
        print(f'{args.map_topic} frame={grid.frame} {grid.width}x{grid.height} res={grid.resolution}',
              file=sys.stderr)
        print(grid.sha256)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
