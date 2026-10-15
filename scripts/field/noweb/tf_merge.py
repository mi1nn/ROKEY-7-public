#!/usr/bin/env python3
"""Both robots' TF on one /tf, and their scans with matching frames, for one RViz (재범 「제일 중요」, 2026-10-16).

    python3 scripts/field/noweb/tf_merge.py --robots robot6 robot5
    rviz2 -d scripts/field/noweb/two_robots.rviz

Each robot publishes TF on /<ns>/tf and /<ns>/tf_static with the same frame names (odom, base_link,
rplidar_link, ...). This prefixes every frame of a robot with its namespace (robot6/odom), keeps the shared
frames (--shared, default map: both robots are on the TB-6 map) and a frame that already has the prefix, and
publishes on /tf and /tf_static. Static transforms are kept per robot and frame and published together, because
/tf_static keeps only the last message of each publisher. Each /<ns>/scan is republished on /rviz/<ns>/scan with
its frame prefixed, so RViz can place it. two_robots.rviz draws the robot models with TF Prefix <ns>.

Run it where RViz runs, with the same Discovery settings. Nothing goes to the robots: it only republishes for
RViz. Do not run it next to a node that reads /tf for anything else (our nodes read <ns>/tf).
"""
import argparse
import copy


def prefixed(frame, ns, *, shared):
    """``frame`` of robot ``ns`` in the merged tree: shared frames as they are, others under ``ns/``."""
    name = frame.lstrip('/')
    if name in shared or name.startswith(ns + '/'):
        return name
    return f'{ns}/{name}'


def renamed(transforms, ns, *, shared):
    """Copies of TransformStamped-like ``transforms`` with both frames renamed (the input is not changed)."""
    out = []
    for t in transforms:
        c = copy.deepcopy(t)
        c.header.frame_id = prefixed(t.header.frame_id, ns, shared=shared)
        c.child_frame_id = prefixed(t.child_frame_id, ns, shared=shared)
        out.append(c)
    return out


class StaticSet:
    """Every static transform seen, the newest per (parent, child): what /tf_static must carry at once."""

    def __init__(self):
        self.edges = {}

    def add(self, transforms):
        changed = False
        for t in transforms:
            key = (t.header.frame_id, t.child_frame_id)
            changed = changed or key not in self.edges
            self.edges[key] = t
        return changed

    def all(self):
        return [self.edges[key] for key in sorted(self.edges)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robots', nargs='+', default=['robot6', 'robot5'], help='robot namespaces, no slash')
    parser.add_argument('--shared', nargs='+', default=['map'], help='frames both robots share (not prefixed)')
    args = parser.parse_args()
    shared = tuple(args.shared)

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('noweb_tf_merge')
    log = node.get_logger()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    tf_pub = node.create_publisher(TFMessage, '/tf', dynamic)
    static_pub = node.create_publisher(TFMessage, '/tf_static', latched)
    statics = StaticSet()

    def on_tf(ns):
        def receive(message):
            tf_pub.publish(TFMessage(transforms=renamed(message.transforms, ns, shared=shared)))
        return receive

    def on_static(ns):
        def receive(message):
            if statics.add(renamed(message.transforms, ns, shared=shared)):
                static_pub.publish(TFMessage(transforms=statics.all()))
                log.info(f'TF_MERGE static frames: {len(statics.all())}')
        return receive

    def on_scan(ns, publisher):
        def receive(message):
            message.header.frame_id = prefixed(message.header.frame_id, ns, shared=shared)
            publisher.publish(message)
        return receive

    for ns in args.robots:
        node.create_subscription(TFMessage, '/' + ns + '/tf', on_tf(ns), dynamic)
        node.create_subscription(TFMessage, '/' + ns + '/tf_static', on_static(ns), latched)
        scan_pub = node.create_publisher(LaserScan, '/rviz/' + ns + '/scan', qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/' + ns + '/scan', on_scan(ns, scan_pub), qos_profile_sensor_data)
    log.info(f'TF_MERGE robots={args.robots} shared={list(shared)} -> /tf, /tf_static, /rviz/<ns>/scan')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
