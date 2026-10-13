"""Overhead structures (tunnels) ahead of a robot, marked on the map (OPS-185/186, 2026-10-13).

The 46 cm passage stop (Nav2 collision_monitor on <ns>/passage_points) only stops the robot; where
a structure is and how high it is was kept nowhere. This node reads the same depth points and
reports each overhead structure on the map:

* Points in the stop box ahead of base_link (passage.front_m, passage.half_width_m) between the
  floor noise and ``top_m``. The lowest height is the ``min_points``-th lowest point, the count
  collision_monitor needs to stop, so a few noisy points neither stop nor lower it.
* It floats only when that height is above the LiDAR plane (OPS-186 a): lower structures the LiDAR
  already sees (a wall, a box) are no tunnel.
* PASSABLE at or above the clearance (passage.max_height_m, owner decision 46 cm), BLOCKED below.
  UNKNOWN when the floor under it (same x span) was not seen: no height is claimed (OPS-186 b).
* Reported after ``confirm_frames`` agreeing processed frames at one place; a frame with another
  answer, nothing, or no usable input restarts the count. Position = the structure's floor centre
  in the map frame (TF at the cloud stamp); the map revision from the SCOUT's MapState.
* One item per place and map revision (``merge_radius_m``); UNKNOWN never replaces a known height.
  The whole list goes out as one latched JSON text (rescue_contracts.overhead, C-04b).

    ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=<robot>
"""
from collections import namedtuple
from dataclasses import dataclass
import math
import sys
import time
from statistics import median
from uuid import uuid4

Reading = namedtuple('Reading', 'state lowest_m x y reason')   # x, y in the frame of the points it came from

POLICY_PARAMS = {
    'clearance_m': 'DOUBLE', 'floor_noise_m': 'DOUBLE', 'front_near_m': 'DOUBLE', 'front_far_m': 'DOUBLE',
    'half_width_m': 'DOUBLE', 'lidar_plane_m': 'DOUBLE', 'top_m': 'DOUBLE', 'min_points': 'INTEGER',
    'floor_min_points': 'INTEGER', 'confirm_frames': 'INTEGER', 'position_tolerance_m': 'DOUBLE',
    'merge_radius_m': 'DOUBLE'}
NODE_PARAMS = {
    'points_topic': 'STRING', 'base_frame': 'STRING', 'map_frame': 'STRING', 'robot_id': 'STRING',
    'map_robot_id': 'STRING', 'map_state_topic': 'STRING', 'clock_error_bound_ns': 'INTEGER',
    'process_period_s': 'DOUBLE', 'max_cloud_age_s': 'DOUBLE', 'stride': 'INTEGER'}
OVERHEAD_PARAMS = {**NODE_PARAMS, **POLICY_PARAMS}   # every one required, none has a default


def _positive(value, name, *, zero=False):
    if type(value) is not float or not math.isfinite(value) or value < 0 or (value == 0 and not zero):
        raise ValueError(f'{name} must be a finite float {">=" if zero else ">"} 0')


@dataclass(frozen=True)
class OverheadPolicy:
    clearance_m: float            # passage.max_height_m: owner decision 46 cm
    floor_noise_m: float          # passage.min_height_m: at or below is floor
    front_near_m: float           # passage.front_m: the stop box ahead of base_link
    front_far_m: float
    half_width_m: float           # passage.half_width_m
    lidar_plane_m: float          # LiDAR scan height above the floor [estimate until measured]
    top_m: float                  # higher points are not part of a passage
    min_points: int               # passage.min_points, as collision_monitor
    floor_min_points: int         # floor points under the structure before "empty below" is claimed
    confirm_frames: int           # agreeing processed frames before reporting
    position_tolerance_m: float   # spread allowed within one run (map frame)
    merge_radius_m: float         # same place as an item already reported (same map revision)

    def __post_init__(self):
        _positive(self.floor_noise_m, 'floor_noise_m', zero=True)
        for name in ('clearance_m', 'front_near_m', 'front_far_m', 'half_width_m', 'lidar_plane_m', 'top_m',
                     'position_tolerance_m', 'merge_radius_m'):
            _positive(getattr(self, name), name, zero=name == 'front_near_m')
        if not self.floor_noise_m < self.lidar_plane_m < self.clearance_m < self.top_m:
            raise ValueError('need floor_noise_m < lidar_plane_m < clearance_m < top_m')
        if self.front_far_m <= self.front_near_m:
            raise ValueError('front_far_m must be beyond front_near_m')
        for name in ('min_points', 'floor_min_points', 'confirm_frames'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f'{name} must be an integer >= 1')


def overhead_values(values):
    """Field values (no defaults) -> (OverheadPolicy, node settings) or ValueError."""
    missing = sorted(name for name in OVERHEAD_PARAMS if values.get(name) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    node = {name: values[name] for name in NODE_PARAMS}
    for name, kind in NODE_PARAMS.items():
        if kind == 'STRING' and (type(node[name]) is not str or not node[name].strip()):
            raise ValueError(f'{name} must be a non-empty STRING')
    if type(node['clock_error_bound_ns']) is not int or node['clock_error_bound_ns'] < 0:
        raise ValueError('clock_error_bound_ns must be an integer >= 0')
    if type(node['stride']) is not int or node['stride'] < 1:
        raise ValueError('stride must be an integer >= 1')
    _positive(node['process_period_s'], 'process_period_s')
    _positive(node['max_cloud_age_s'], 'max_cloud_age_s')
    return OverheadPolicy(**{name: values[name] for name in POLICY_PARAMS}), node


def cloud_xyz(msg, stride):
    """(N, 3) finite x, y, z of a PointCloud2 (little-endian float32 x/y/z), every ``stride`` row and column."""
    import numpy as np
    fields = {f.name: f for f in msg.fields}
    if msg.is_bigendian or any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in 'xyz'):
        raise ValueError('PointCloud2 needs little-endian FLOAT32 x, y, z')
    if msg.row_step != msg.width * msg.point_step or len(msg.data) != msg.height * msg.row_step:
        raise ValueError('PointCloud2 rows must be packed (row_step = width * point_step)')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [fields[n].offset for n in 'xyz'], 'itemsize': msg.point_step})
    # no copy: rclpy gives the data as array.array, which shares its buffer
    grid = np.frombuffer(msg.data, dtype=dtype, count=msg.height * msg.width).reshape(
        msg.height, msg.width)[::stride, ::stride]
    points = np.stack([grid['x'].ravel(), grid['y'].ravel(), grid['z'].ravel()], axis=1).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def to_frame(points, translation, quaternion):
    """Points expressed in the parent frame of a transform (translation, quaternion x/y/z/w)."""
    import numpy as np
    x, y, z, w = quaternion
    rotation = np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def judge(points, policy):
    """Reading for the overhead structure in the box ahead (points in base_link), or None when there is none."""
    import numpy as np
    if len(points) == 0:
        return None
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    box = (x >= policy.front_near_m) & (x <= policy.front_far_m) & (np.abs(y) <= policy.half_width_m)
    above = box & (z > policy.floor_noise_m) & (z <= policy.top_m)
    if np.count_nonzero(above) < policy.min_points:
        return None
    lowest = round(float(np.partition(z[above], policy.min_points - 1)[policy.min_points - 1]), 3)
    if lowest <= policy.lidar_plane_m:
        return None   # the LiDAR sees it: an ordinary obstacle
    xs = x[above]
    floor = box & (z <= policy.floor_noise_m) & (x >= xs.min()) & (x <= xs.max())
    cx, cy = float(xs.mean()), float(y[above].mean())
    if np.count_nonzero(floor) < policy.floor_min_points:
        return Reading('UNKNOWN', None, cx, cy, 'NO_FLOOR_SEEN')
    return Reading('PASSABLE' if lowest >= policy.clearance_m else 'BLOCKED', lowest, cx, cy, '')


class OverheadConfirmer:
    """Counts consecutive processed frames with the same answer at one place (map frame)."""

    def __init__(self, policy):
        self.policy = policy
        self.run = []
        self.frames_of_last = 0

    def add(self, reading):
        """One processed frame (Reading in the map frame, or None); the confirmed Reading on the Nth agreeing one."""
        if reading is None:
            self.run = []
            return None
        if self.run and (reading.state != self.run[0].state or math.dist((reading.x, reading.y), (
                self.run[0].x, self.run[0].y)) > self.policy.position_tolerance_m):
            self.run = []
        self.run.append(reading)
        if len(self.run) < self.policy.confirm_frames:
            return None
        run, self.run = self.run, []
        self.frames_of_last = len(run)
        lowest = None if run[0].state == 'UNKNOWN' else round(median(r.lowest_m for r in run), 3)
        return Reading(run[0].state, lowest, sum(r.x for r in run) / len(run), sum(r.y for r in run) / len(run),
                       run[-1].reason)


class PassageRegistry:
    """Reported places per map revision; ``update`` says whether the published list changes."""

    def __init__(self, policy, new_id=lambda: str(uuid4())):
        self.policy, self.new_id = policy, new_id
        self._items = []

    def update(self, reading, *, map_session_id, map_revision, observed_at_ns, frames):
        same = [i for i in self._items if (i['map_session_id'], i['map_revision']) == (map_session_id, map_revision)
                and math.dist((i['x'], i['y']), (reading.x, reading.y)) <= self.policy.merge_radius_m]
        if not same:
            self._items.append(dict(id=self.new_id(), state=reading.state, lowest_m=reading.lowest_m, x=reading.x,
                                    y=reading.y, map_session_id=map_session_id, map_revision=map_revision,
                                    frames=frames, first_seen_ns=observed_at_ns, last_seen_ns=observed_at_ns,
                                    reason=reading.reason))
            return True
        item = min(same, key=lambda i: math.dist((i['x'], i['y']), (reading.x, reading.y)))
        if reading.state == 'UNKNOWN' and item['state'] != 'UNKNOWN':
            return False   # another distance did not see the floor: the height measured before stands
        changed = (item['state'], item['lowest_m']) != (reading.state, reading.lowest_m)
        item.update(state=reading.state, lowest_m=reading.lowest_m, x=reading.x, y=reading.y, reason=reading.reason,
                    frames=item['frames'] + frames, last_seen_ns=max(item['last_seen_ns'], observed_at_ns))
        return changed

    def items(self):
        return [dict(i) for i in sorted(self._items, key=lambda i: (i['first_seen_ns'], i['id']))]


def main(args=None):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rclpy._rclpy_pybind11 import RCLError
    from rescue_contracts.overhead import overhead_passages_text, overhead_passages_topic
    from rescue_interfaces.msg import MapState
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from .camera_utils import git_sha, read_field_parameters
    from .shutdown import spin_until_shutdown
    from .target_location import MapRevisionTracker

    rclpy.init(args=args)
    node = rclpy.create_node('overhead_passage')
    log = node.get_logger()
    try:
        values = read_field_parameters(node, OVERHEAD_PARAMS)
        try:
            policy, v = overhead_values(values)
            topic = overhead_passages_topic(v['robot_id'])
        except ValueError as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        tracker = MapRevisionTracker(robot_id=v['map_robot_id'], map_frame=v['map_frame'])
        confirmer, registry = OverheadConfirmer(policy), PassageRegistry(policy)
        buffer = Buffer()
        node.tf_listener = TransformListener(buffer, node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(String, topic, latched)
        state = {'cloud': None, 'done': None, 'frames': 0, 'busy_s': 0.0}

        def publish():
            publisher.publish(String(data=overhead_passages_text(
                robot_id=v['robot_id'], map_frame=v['map_frame'], generated_at_ns=node.get_clock().now().nanoseconds,
                clearance_m=policy.clearance_m, floor_noise_m=policy.floor_noise_m,
                lidar_plane_m=policy.lidar_plane_m, items=registry.items())))

        def on_map_state(msg):
            if msg.robot_id == v['map_robot_id']:
                tracker.update(msg)

        def lookup(target, source, stamp):
            tf = buffer.lookup_transform(target, source, Time(nanoseconds=stamp))
            t, q = tf.transform.translation, tf.transform.rotation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

        def process():
            msg = state['cloud']
            if msg is None:
                return
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if stamp == state['done']:
                return
            state['done'] = stamp
            started = time.monotonic()
            try:
                age = (node.get_clock().now().nanoseconds - stamp) / 1e9
                if age > v['max_cloud_age_s']:
                    raise ValueError(f'STALE_CLOUD age={age:.3f}s')
                session, revision = tracker.basis_for(stamp, v['clock_error_bound_ns'])
                import numpy as np
                points = to_frame(cloud_xyz(msg, v['stride']), *lookup(v['base_frame'], msg.header.frame_id, stamp))
                found = judge(points, policy)
                if found is not None:
                    (mx, my, _), = to_frame(np.array([[found.x, found.y, 0.0]]),
                                            *lookup(v['map_frame'], v['base_frame'], stamp))
                    found = found._replace(x=float(mx), y=float(my))
                confirmed = confirmer.add(found)
                if confirmed is not None:
                    changed = registry.update(confirmed, map_session_id=session, map_revision=revision,
                                              observed_at_ns=stamp, frames=confirmer.frames_of_last)
                    if changed:   # the same answer again only refreshes the item, no log line
                        log.info(f'overhead {confirmed.state} lowest={confirmed.lowest_m} map=({confirmed.x:.2f}, '
                                 f'{confirmed.y:.2f}) revision={session}/{revision} {confirmed.reason} published')
                        publish()
            except Exception as exc:   # stale, no map revision (LocationRefused), no TF, bad cloud: says nothing
                confirmer.add(None)
                log.warning(f'cloud {stamp} not judged: {type(exc).__name__}: {exc}', throttle_duration_sec=5.0)
            state['frames'] += 1
            state['busy_s'] += time.monotonic() - started
            if state['frames'] % 20 == 0:
                log.info(f'processed={state["frames"]} mean_ms={1000 * state["busy_s"] / state["frames"]:.1f}')

        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(PointCloud2, v['points_topic'], lambda m: state.update(cloud=m), sensor)
        node.create_subscription(MapState, v['map_state_topic'], on_map_state, latched)
        node.create_timer(v['process_period_s'], process)
        publish()   # an empty list: no structure reported yet (latched for late listeners)
        log.info(f'overhead_passage git={git_sha()} topic={topic} parameters={dict(sorted(values.items()))}')
        spin_until_shutdown(lambda: rclpy.spin(node), node.context.ok, RCLError)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
