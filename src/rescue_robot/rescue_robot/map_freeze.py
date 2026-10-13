"""Freeze the TB-6 SLAM map once per run (map lifecycle, OPS-113/117; compute_a).

On /rescue/map/freeze (std_srvs/Trigger) the latest TB-6 SLAM map and TB-6 SLAM pose are
captured, hashed (live_sources.Grid, the robots' hash) and written to ``output_dir``:

- ``frozen-<sha12>-<ns>.json``: the exact grid (origin quaternion as received) and pose, the
  basis (session, revision R0 + 1) and the git SHA. A restarted node reloads it
  (``record_path``) and republishes the identical grid, so the robots see the same hash.
- ``.pgm``/``.yaml``: the same map for people and RViz (trinary 0.25/0.65). Loading it into
  map_server does not give the same hash; robots use the republished grid, not this image.

TB-6 start pose (OPS-141/142): the first valid TB-6 SLAM pose after this node starts is saved once
per run (output_dir/start_pose.json) and published latched on /rescue/map/start_pose/robot_a.
Coordinates measured in advance (patrol points, TB-6 docking approach, TB-5 AMCL start) are
relative to it. A restarted node republishes the saved one; a new run needs a new output_dir.
Start this node before TB-6 moves.

Then /rescue/map/frozen (nav_msgs/OccupancyGrid) and /rescue/map/frozen_pose/robot_a
(PoseWithCovarianceStamped, TB-6 AMCL start) are published TRANSIENT_LOCAL. Once per run:
a second request fails. Without a valid map in the map frame and a fresh SLAM pose on it
nothing is frozen, written or published. The response's success is the result; the message
is for people. The robot must be stopped first (the coordinator checks before asking).
"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

from rescue_robot.executor import ContractError
from rescue_robot.live_sources import Grid, planar

FORMAT = 'rescue-map-freeze/1'
FROZEN_TOPIC = '/rescue/map/frozen'
FROZEN_POSE_TOPIC = '/rescue/map/frozen_pose/robot_a'
START_POSE_TOPIC = '/rescue/map/start_pose/robot_a'   # TB-6 start pose: origin of coordinates measured in advance
START_FORMAT = 'rescue-start-pose/1'
START_FILE = 'start_pose.json'
SERVICE = '/rescue/map/freeze'


class FreezeError(Exception):
    pass


def _grid_fields(g):
    return dict(frame=g['frame'], width=g['width'], height=g['height'], resolution=g['resolution'],
                position=tuple(g['position']), orientation=tuple(g['orientation']), data=list(g['data']))


def _pose_fields(p):
    return dict(frame=p['frame'], stamp_ns=p['stamp_ns'], position=tuple(p['position']),
                orientation=tuple(p['orientation']), covariance=list(p['covariance']))


def _ns_pose(position, orientation):
    (px, py, pz), (qx, qy, qz, qw) = position, orientation
    return NS(position=NS(x=px, y=py, z=pz), orientation=NS(x=qx, y=qy, z=qz, w=qw))


def grid_hash(g, map_frame):
    """The robots' hash of this grid (live_sources.Grid); FreezeError if invalid or not in the map frame."""
    if len(g['position']) != 3 or len(g['orientation']) != 4:
        raise FreezeError('invalid map origin')
    message = NS(header=NS(frame_id=g['frame']), info=NS(width=g['width'], height=g['height'],
                 resolution=g['resolution'], origin=_ns_pose(g['position'], g['orientation'])), data=g['data'])
    try:
        grid = Grid(message)
    except (ContractError, ValueError, TypeError) as exc:
        raise FreezeError(f'invalid map: {exc}') from None
    if grid.frame != map_frame:
        raise FreezeError(f'map frame {grid.frame!r} is not {map_frame!r}')
    return grid


def _check_pose(p, *, map_frame, now_ns, max_age_ns):
    if p is None:
        raise FreezeError('no SLAM pose received')
    if p['frame'] != map_frame:
        raise FreezeError(f'SLAM pose frame {p["frame"]!r} is not {map_frame!r}')
    if type(p['stamp_ns']) is not int or abs(now_ns - p['stamp_ns']) > max_age_ns:
        raise FreezeError('SLAM pose is not fresh')
    if len(p['position']) != 3 or len(p['orientation']) != 4 or len(p['covariance']) != 36 or not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in p['covariance']):
        raise FreezeError('invalid SLAM pose')
    try:
        planar(_ns_pose(p['position'], p['orientation']))
    except ContractError as exc:
        raise FreezeError(f'invalid SLAM pose: {exc}') from None


def _pgm(g):
    """Trinary image, top row = largest y (map_server convention)."""
    def pixel(v):
        return 205 if v < 0 else 0 if v >= 65 else 254 if v <= 25 else 205
    rows = (bytes(pixel(g['data'][iy * g['width'] + ix]) for ix in range(g['width']))
            for iy in reversed(range(g['height'])))
    return f'P5\n{g["width"]} {g["height"]}\n255\n'.encode() + b''.join(rows)


def _write(path, payload):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(payload)
    os.replace(tmp, path)


def load_record(path, *, map_frame, map_session_id, map_revision):
    """A saved freeze of this run (same session, R0 + 1), with its hash re-checked."""
    try:
        record = json.loads(Path(path).read_text())
        if record.get('format') != FORMAT:
            raise FreezeError('not a map freeze record')
        record['grid'], record['pose'] = _grid_fields(record['grid']), _pose_fields(record['pose'])
        if record.get('start_pose') is not None:
            record['start_pose'] = _pose_fields(record['start_pose'])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise FreezeError(f'freeze record unreadable: {exc}') from None
    if (record['map_frame'], record['map_session_id'], record['map_revision']) != (
            map_frame, map_session_id, map_revision + 1):
        raise FreezeError('freeze record is from another run (frame, session or revision differ)')
    if grid_hash(record['grid'], map_frame).sha256 != record['occupancy_sha256']:
        raise FreezeError('freeze record grid does not match its hash')
    record['path'] = str(path)
    return record


class MapFreeze:
    def __init__(self, *, map_frame, map_session_id, map_revision, pose_max_age_ns, output_dir, wall_clock_ns,
                 git_sha):
        if type(pose_max_age_ns) is not int or pose_max_age_ns <= 0 or type(map_revision) is not int or map_revision < 0:
            raise FreezeError('positive pose_max_age_ns and map_revision >= 0 required')
        self.map_frame, self.map_session_id, self.map_revision = map_frame, map_session_id, map_revision
        self.pose_max_age_ns, self.output_dir = pose_max_age_ns, Path(output_dir)
        self.wall_clock_ns, self.git_sha = wall_clock_ns, git_sha
        self.latest_map = self.latest_pose = self.frozen = None
        self.start_pose = self._load_start()

    def _load_start(self):
        """The start pose this run already recorded (a restarted node keeps it, never re-captures)."""
        path = self.output_dir / START_FILE
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text())
            if (data['format'], data['map_frame'], data['map_session_id'], data['map_revision']) != (
                    START_FORMAT, self.map_frame, self.map_session_id, self.map_revision):
                raise FreezeError(f'{path} is the start pose of another run; use a new output_dir for a new run')
            return _pose_fields(data['pose'])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise FreezeError(f'start pose unreadable: {exc}') from None

    def on_map(self, g):
        self.latest_map = _grid_fields(g)

    def on_pose(self, p):
        """Latest SLAM pose; the first valid one of the run becomes the start pose (returned once, saved)."""
        self.latest_pose = _pose_fields(p)
        if self.start_pose is not None:
            return None
        try:
            _check_pose(self.latest_pose, map_frame=self.map_frame, now_ns=self.wall_clock_ns(),
                        max_age_ns=self.pose_max_age_ns)
            self.output_dir.mkdir(parents=True, exist_ok=True)
            _write(self.output_dir / START_FILE, json.dumps(dict(
                format=START_FORMAT, map_frame=self.map_frame, map_session_id=self.map_session_id,
                map_revision=self.map_revision, captured_at_ns=self.wall_clock_ns(), git_sha=self.git_sha,
                pose=self.latest_pose), allow_nan=False).encode())
        except (FreezeError, OSError, ValueError):
            return None
        self.start_pose = self.latest_pose
        return self.start_pose

    def restore(self, record):
        self.frozen = record

    def freeze(self):
        if self.frozen is not None:
            raise FreezeError(f'already frozen ({self.frozen["occupancy_sha256"]}); one freeze per run')
        if self.latest_map is None:
            raise FreezeError('no SLAM map received')
        if self.start_pose is None:
            raise FreezeError('no TB-6 start pose recorded: coordinates measured from the start cannot be placed')
        g, p, now = self.latest_map, self.latest_pose, self.wall_clock_ns()
        grid = grid_hash(g, self.map_frame)
        _check_pose(p, map_frame=self.map_frame, now_ns=now, max_age_ns=self.pose_max_age_ns)
        record = dict(format=FORMAT, map_frame=self.map_frame, map_session_id=self.map_session_id,
                      map_revision=self.map_revision + 1, occupancy_sha256=grid.sha256, frozen_at_ns=now,
                      git_sha=self.git_sha, grid=g, pose=p, start_pose=self.start_pose)
        base = self.output_dir / f'frozen-{grid.sha256[:12]}-{now}'
        x, y, yaw = grid.origin
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            _write(base.with_suffix('.pgm'), _pgm(g))
            _write(base.with_suffix('.yaml'), (f'image: {base.name}.pgm\nmode: trinary\nresolution: {grid.resolution!r}\n'
                                               f'origin: [{x!r}, {y!r}, {yaw!r}]\nnegate: 0\n'
                                               'occupied_thresh: 0.65\nfree_thresh: 0.25\n').encode())
            _write(base.with_suffix('.json'), json.dumps(record, allow_nan=False).encode())
        except (OSError, ValueError) as exc:
            for suffix in ('.pgm', '.yaml', '.json'):
                base.with_suffix(suffix).unlink(missing_ok=True)
            raise FreezeError(f'freeze not saved: {exc}') from None
        record['path'] = str(base.with_suffix('.json'))
        self.frozen = record
        return record


# ---- ROS boundary ---------------------------------------------------------------------------

def _grid_message(record, stamp):
    from nav_msgs.msg import OccupancyGrid
    g, msg = record['grid'], OccupancyGrid()
    msg.header.frame_id, msg.header.stamp = g['frame'], stamp
    msg.info.width, msg.info.height, msg.info.resolution = g['width'], g['height'], g['resolution']
    p, q = msg.info.origin.position, msg.info.origin.orientation
    p.x, p.y, p.z = g['position']
    q.x, q.y, q.z, q.w = g['orientation']
    msg.data = g['data']
    return msg


def _pose_message(record, key='pose'):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    p, msg = record[key], PoseWithCovarianceStamped()
    msg.header.frame_id = p['frame']
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(p['stamp_ns'], 1_000_000_000)
    position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
    position.x, position.y, position.z = p['position']
    orientation.x, orientation.y, orientation.z, orientation.w = p['orientation']
    msg.pose.covariance = [float(v) for v in p['covariance']]
    return msg


def main(args=None):
    import time

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_srvs.srv import Trigger
    from rescue_contracts import ros_time_to_ns
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_map_freeze')
    log = node.get_logger()
    S, I = T.PARAMETER_STRING, T.PARAMETER_INTEGER
    values, missing = declare_required(node, {'map_topic': S, 'pose_topic': S, 'map_frame': S, 'map_session_id': S,
                                              'map_revision': I, 'pose_max_age_ns': I, 'output_dir': S})
    if missing:
        log.error('map_freeze missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    node.declare_parameter('record_path', Parameter.Type.STRING)   # optional: restart from a saved freeze
    try:
        record_path = node.get_parameter('record_path').value
    except ParameterUninitializedException:
        record_path = None
    sha = git_sha()
    try:
        core = MapFreeze(map_frame=values['map_frame'], map_session_id=values['map_session_id'],
                         map_revision=values['map_revision'], pose_max_age_ns=values['pose_max_age_ns'],
                         output_dir=values['output_dir'], wall_clock_ns=time.time_ns, git_sha=sha)
    except FreezeError as exc:
        log.error(f'START_POSE_RESTORE_FAILED {exc}')
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    frozen_pub = node.create_publisher(OccupancyGrid, FROZEN_TOPIC, latched)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, FROZEN_POSE_TOPIC, latched)
    start_pub = node.create_publisher(PoseWithCovarianceStamped, START_POSE_TOPIC, latched)

    def publish_start(start, how):
        start_pub.publish(_pose_message({'pose': start}))
        x, y, yaw = planar(_ns_pose(start['position'], start['orientation']))
        log.info(f'START_POSE {how} robot_a map=({x:.3f}, {y:.3f}, {yaw:.3f}) session={values["map_session_id"]} '
                 f'rev={values["map_revision"]} -> {START_POSE_TOPIC} (origin of coordinates measured in advance)')

    def publish(record):
        frozen_pub.publish(_grid_message(record, node.get_clock().now().to_msg()))
        pose_pub.publish(_pose_message(record))
        x, y, yaw = planar(_ns_pose(record['pose']['position'], record['pose']['orientation']))
        log.info(f'MAP_FROZEN sha256={record["occupancy_sha256"]} session={record["map_session_id"]} '
                 f'rev={record["map_revision"]} tb6_start=({x:.3f}, {y:.3f}, {yaw:.3f}) record={record["path"]}')

    def on_map(msg):
        o = msg.info.origin
        core.on_map(dict(frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                         resolution=msg.info.resolution, position=(o.position.x, o.position.y, o.position.z),
                         orientation=(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                         data=list(msg.data)))

    def on_pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        start = core.on_pose(dict(frame=msg.header.frame_id, stamp_ns=ros_time_to_ns(msg.header.stamp.sec,
                                                                                     msg.header.stamp.nanosec),
                                  position=(p.x, p.y, p.z), orientation=(q.x, q.y, q.z, q.w),
                                  covariance=[float(v) for v in msg.pose.covariance]))
        if start is not None:
            publish_start(start, 'captured')

    def freeze(_request, response):
        try:
            record = core.freeze()
        except FreezeError as exc:
            log.error(f'MAP_FREEZE_FAILED {exc}')
            response.success, response.message = False, str(exc)
            return response
        publish(record)
        response.success, response.message = True, f'frozen sha256={record["occupancy_sha256"]}'
        return response

    log.info(f'git_sha={sha} parameters={values} record_path={record_path}')
    if core.start_pose is not None:   # same run restarted: the saved start, never a new one
        publish_start(core.start_pose, 'restored')
    if record_path is not None:
        try:
            record = load_record(record_path, map_frame=values['map_frame'], map_session_id=values['map_session_id'],
                                 map_revision=values['map_revision'])
        except FreezeError as exc:
            log.error(f'MAP_FREEZE_RESTORE_FAILED {exc}')
            raise SystemExit(2)
        core.restore(record)
        if core.start_pose is None and record.get('start_pose') is not None:
            core.start_pose = record['start_pose']
            publish_start(core.start_pose, 'restored from the freeze record')
        publish(record)
    else:
        node.create_subscription(OccupancyGrid, values['map_topic'], on_map, latched)
        node.create_subscription(PoseWithCovarianceStamped, values['pose_topic'], on_pose, qos_profile_sensor_data)
    node.create_service(Trigger, SERVICE, freeze)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
