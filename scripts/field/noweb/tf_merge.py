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

TB-5 has no map -> odom until AMCL runs on the frozen map, so RViz could not draw it while TB-6 explores
(재범 2026-10-16 08:00). Until TB-5's own map -> odom comes, a provisional one goes to /tf: the field file's
start value (robots.<id>.initial_pose_xy_yaw, relative to the TB-6 start pose, the value AMCL starts from) on the
TB-6 start pose (/rescue/map/start_pose/<SLAM robot>), less TB-5's odometry at that time, so TB-5's body is drawn
on its start spot. It needs both; until then TB-5 is not drawn. The first map -> odom from the robot ends it for
good (log TF_PROVISIONAL_OFF; after a reset restart this tool). RViz cannot change a model's look from TF, so the
log is the only sign that the spot is provisional.

Run it where RViz runs, with the same Discovery settings. Nothing goes to the robots: it only republishes for
RViz. Do not run it next to a node that reads /tf for anything else (our nodes read <ns>/tf).
"""
import argparse
import copy
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[3]
try:
    import rescue_contracts  # noqa: F401  (installed or on PYTHONPATH)
except ImportError:  # run straight from the checkout
    sys.path.insert(0, str(REPO / 'src' / 'rescue_contracts'))

from rescue_contracts.occupancy import compose_planar, relative_planar  # noqa: E402

FIELD_FILE = REPO / 'config' / 'field' / 'search_area.field.json'
START_KEY = 'initial_pose_xy_yaw'              # field_launch FROZEN_ROBOT_OPTIONAL_KEYS (capture_initial_pose.py)
START_POSE_TOPIC = '/rescue/map/start_pose/'   # + SLAM robot id (field_launch.START_POSE_TOPIC)


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


def provisional_source(field_path, robot_id):
    """(start value (x, y, yaw), start pose topic) of ``robot_id`` from the field file. Raises ValueError."""
    robots = json.loads(Path(field_path).read_text(encoding='utf-8'))['robots']
    value = robots.get(robot_id, {}).get(START_KEY)
    if not (isinstance(value, list) and len(value) == 3):
        raise ValueError(f'{field_path}: robots.{robot_id}.{START_KEY} is not measured')
    slam = [rid for rid, section in robots.items() if section.get('task.saved_map.mode') == 'SLAM_THEN_FROZEN']
    if len(slam) != 1:
        raise ValueError(f'{field_path}: one SLAM_THEN_FROZEN robot is needed for the start pose')
    return tuple(float(v) for v in value), START_POSE_TOPIC + slam[0]


def _xy_yaw(t):
    q = t.transform.rotation
    return (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class ProvisionalOdom:
    """A stand-in map -> <ns>/odom for RViz until the robot's own comes (frames already renamed)."""

    def __init__(self, ns, start_value, *, map_frame='map', base='base_link'):
        self.start_value = start_value
        self.map_frame, self.odom, self.base = map_frame, f'{ns}/odom', f'{ns}/{base}'
        self.start_pose = self.odom_base = None
        self.ended = False

    def on_start_pose(self, xy_yaw):
        self.start_pose = tuple(xy_yaw)

    def on_robot_tf(self, transforms):
        """Take the robot's renamed transforms. True only the first time its own map -> odom shows up."""
        first_real = False
        for t in transforms:
            edge = (t.header.frame_id, t.child_frame_id)
            if edge == (self.map_frame, self.odom) and not self.ended:
                self.ended = first_real = True
            elif edge == (self.odom, self.base) and self.odom_base is None:
                self.odom_base = _xy_yaw(t)   # kept: the robot does not move before its map, and later moves show
        return first_real

    def transform(self):
        """(x, y, yaw) of map -> odom, or None (ended, or start pose or odometry not seen yet)."""
        if self.ended or self.start_pose is None or self.odom_base is None:
            return None
        body = compose_planar(self.start_pose, self.start_value)
        return compose_planar(body, relative_planar(self.odom_base, (0.0, 0.0, 0.0)))

    def stamped(self, stamp, out):
        """Fill TransformStamped ``out`` with transform() at ``stamp``."""
        x, y, yaw = self.transform()
        out.header.frame_id, out.header.stamp, out.child_frame_id = self.map_frame, stamp, self.odom
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = x, y, 0.0
        r = out.transform.rotation
        r.x, r.y, r.z, r.w = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robots', nargs='+', default=['robot6', 'robot5'], help='robot namespaces, no slash')
    parser.add_argument('--shared', nargs='+', default=['map'], help='frames both robots share (not prefixed)')
    parser.add_argument('--field', type=Path, default=FIELD_FILE, help='field run file (TB-5 start value)')
    parser.add_argument('--provisional', default='robot5=robot_b', metavar='NS=ROBOT_ID',
                        help="robot drawn at its start spot until its own map -> odom comes ('' = none)")
    args = parser.parse_args()
    shared = tuple(args.shared)
    provisional = None
    if args.provisional:
        prov_ns, _, prov_id = args.provisional.partition('=')
        try:
            start_value, start_topic = provisional_source(args.field, prov_id)
        except (OSError, ValueError, KeyError) as exc:
            parser.error(f'--provisional {args.provisional}: {exc}')
        provisional = ProvisionalOdom(prov_ns, start_value, map_frame=shared[0])

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
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
            transforms = renamed(message.transforms, ns, shared=shared)
            if provisional is not None and ns == prov_ns and transforms:
                if provisional.on_robot_tf(transforms):
                    log.info(f'TF_PROVISIONAL_OFF {ns}: its own map -> odom came; start spot no longer drawn')
                if provisional.transform() is not None:
                    transforms.append(provisional.stamped(transforms[0].header.stamp, TransformStamped()))
            tf_pub.publish(TFMessage(transforms=transforms))
        return receive

    def on_start_pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        provisional.on_start_pose((p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        log.info(f'TF_PROVISIONAL {prov_ns}: start pose {provisional.start_pose}, start value '
                 f'{provisional.start_value} -> map -> {provisional.odom} {provisional.transform()}')

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
    if provisional is not None:
        node.create_subscription(PoseWithCovarianceStamped, start_topic, on_start_pose, latched)
        log.info(f'TF_PROVISIONAL {prov_ns}: start value {start_value} from {args.field}, waiting for {start_topic} '
                 f'and {prov_ns}/odom -> base_link')
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
