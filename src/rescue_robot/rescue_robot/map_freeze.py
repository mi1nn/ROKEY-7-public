"""Freeze the TB-6 SLAM map twice per run (map lifecycle, OPS-113/117/154; compute_a).

On /rescue/map/freeze (std_srvs/Trigger) the latest TB-6 SLAM map is captured, hashed
(live_sources.Grid, the robots' hash) and written to ``output_dir``. Twice per run:

1. R0 + 1, when the doll is found: TB-5 goes on this map; TB-6 keeps searching with SLAM, so no
   TB-6 pose is captured (TB-6 may be moving).
2. R0 + 2, when the search is done: also the fresh TB-6 SLAM pose (TB-6 stopped; the
   coordinator checks before asking), the AMCL start for TB-6 on /rescue/map/frozen_pose/robot_a.

A third request fails. TB-6's pose (start pose, last freeze) is TF map -> base_link read on a
timer (slam_tf_pose, OPS-179): slam_toolbox publishes /robot6/pose only after enough movement, so
a stopped TB-6 has no fresh one. The covariance is the last /robot6/pose's and must be younger than
``covariance_max_age_ns``, else the pose does not count. Files per freeze:

- ``frozen-r<revision>-<sha12>-<ns>.json``: the exact grid (origin quaternion as received), the pose (second
  only), the basis (session, revision) and the git SHA. A restarted node reloads every record of
  this run from ``output_dir`` and republishes them in order, so the robots see the same hashes.
- ``.pgm``/``.yaml``: the same map for people and RViz (trinary 0.25/0.65). Loading it into
  map_server does not give the same hash; robots use the republished grid, not this image.

TB-6 start pose (OPS-141/142): the first valid TB-6 SLAM pose after this node starts is saved once
per run (output_dir/start_pose.json) and published latched on /rescue/map/start_pose/robot_a.
Coordinates measured in advance (patrol points, TB-6 docking approach, TB-5 AMCL start) are
relative to it. A restarted node republishes the saved one; a new run needs a new output_dir.
Start this node before TB-6 moves.

Then /rescue/map/frozen (nav_msgs/OccupancyGrid) and /rescue/map/frozen_revision (std_msgs/String,
rescue_contracts.occupancy.frozen_revision_text: which revision that grid is; the grid has no
field for it) are published TRANSIENT_LOCAL, depth 1 (the latest freeze). Without a valid map in
the map frame (and, for the second, a fresh SLAM pose on it) nothing is frozen, written or
published. The response's success is the result; the message is for people.

TB-5's body (C25, run 15): TB-5 stands at its start while TB-6 maps, so TB-6's lidar marks TB-5 as occupied cells
and TB-5 had no free start cell on the frozen map. Each freeze turns the occupied cells within
``SUPPORT_BODY_CLEAR_M`` of TB-5's start (TB-6 start pose + robots.<TB-5>.initial_pose_xy_yaw, where AMCL starts
TB-5) into free cells before hashing; unknown cells and anything farther stay. Without a measured TB-5 start
nothing is cleared and the record says so (``support_body_cleared`` None).
"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

from rescue_contracts.occupancy import compose_planar, frozen_revision_text
from rescue_robot.executor import ContractError
from rescue_robot.live_sources import Grid, planar
from rescue_robot.slam_tf_pose import MAX_TILT_RAD, SlamTfPose

FORMAT = 'rescue-map-freeze/1'
FROZEN_TOPIC = '/rescue/map/frozen'
FROZEN_REVISION_TOPIC = '/rescue/map/frozen_revision'   # which revision /rescue/map/frozen is (OPS-159)
TF_POSE_PERIOD_S = 0.2   # TB-6 TF pose read period: well inside pose_max_age_ns (2 s)
FREEZES = 2   # per run: R0 + 1 (doll found), R0 + 2 (search done, TB-6 changes to AMCL)
FROZEN_POSE_TOPIC = '/rescue/map/frozen_pose/robot_a'
START_POSE_TOPIC = '/rescue/map/start_pose/robot_a'   # TB-6 start pose: origin of coordinates measured in advance
START_FORMAT = 'rescue-start-pose/1'
START_FILE = 'start_pose.json'
SERVICE = '/rescue/map/freeze'
# TB-5 body cleared at each freeze [제안]: footprint radius 0.189 m (Nav2 footprint) + one map noise cell 0.05 m +
# a little start pose error; no larger, so walls next to TB-5's start stay (C25)
SUPPORT_BODY_CLEAR_M = 0.25
OCCUPIED = 65   # occupancy value counted as occupied (Nav2 map_server occupied_thresh 0.65; slam_toolbox writes 100)


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


def clear_disc(g, xy, radius_m):
    """A copy of grid fields ``g`` with occupied cells whose centres are within ``radius_m`` of ``xy`` (map frame)
    made free, and how many were changed. Unknown cells are left unknown."""
    grid = Grid(NS(header=NS(frame_id=g['frame']), info=NS(width=g['width'], height=g['height'],
                   resolution=g['resolution'], origin=_ns_pose(g['position'], g['orientation'])), data=g['data']))
    ox, oy, oyaw = grid.origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    gx, gy = c * (xy[0] - ox) + s * (xy[1] - oy), -s * (xy[0] - ox) + c * (xy[1] - oy)   # map -> grid frame
    res, width, height = float(g['resolution']), g['width'], g['height']
    reach = math.ceil(radius_m / res) + 1
    cx, cy = math.floor(gx / res), math.floor(gy / res)
    data, cleared = list(g['data']), 0
    for y in range(max(cy - reach, 0), min(cy + reach, height - 1) + 1):
        for x in range(max(cx - reach, 0), min(cx + reach, width - 1) + 1):
            if data[y * width + x] >= OCCUPIED and math.hypot((x + 0.5) * res - gx, (y + 0.5) * res - gy) <= radius_m:
                data[y * width + x] = 0
                cleared += 1
    return dict(g, data=data), cleared


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


def tf_pose_fields(pose, frame):
    """A slam_tf_pose.SlamTfPose.pose() result as this node's pose fields."""
    return dict(frame=frame, stamp_ns=pose['stamp_ns'], position=(float(pose['x']), float(pose['y']), 0.0),
                orientation=(0.0, 0.0, math.sin(pose['yaw'] / 2), math.cos(pose['yaw'] / 2)),
                covariance=[float(v) for v in pose['covariance']])


def _write(path, payload):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(payload)
    os.replace(tmp, path)


def revision_text(record):
    """/rescue/map/frozen_revision for this freeze record."""
    return frozen_revision_text(map_session_id=record['map_session_id'], map_revision=record['map_revision'],
                                occupancy_sha256=record['occupancy_sha256'])


def load_record(path, *, map_frame, map_session_id, map_revision):
    """A saved freeze of this run (same session, R0 + 1 or R0 + 2), with its hash re-checked."""
    try:
        record = json.loads(Path(path).read_text())
        if record.get('format') != FORMAT:
            raise FreezeError('not a map freeze record')
        record['grid'] = _grid_fields(record['grid'])
        record['pose'] = None if record['pose'] is None else _pose_fields(record['pose'])
        if record.get('start_pose') is not None:
            record['start_pose'] = _pose_fields(record['start_pose'])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise FreezeError(f'freeze record unreadable: {exc}') from None
    if (record['map_frame'], record['map_session_id']) != (map_frame, map_session_id) or record[
            'map_revision'] not in range(map_revision + 1, map_revision + FREEZES + 1):
        raise FreezeError('freeze record is from another run (frame, session or revision differ)')
    if (record['map_revision'] == map_revision + FREEZES) != (record['pose'] is not None):
        raise FreezeError('only the last freeze carries the TB-6 pose')
    if grid_hash(record['grid'], map_frame).sha256 != record['occupancy_sha256']:
        raise FreezeError('freeze record grid does not match its hash')
    record['final'] = record['map_revision'] == map_revision + FREEZES
    record['path'] = str(path)
    return record


def load_records(output_dir, *, map_frame, map_session_id, map_revision):
    """Every freeze of this run in output_dir, in revision order: [], [R0+1] or [R0+1, R0+2]."""
    records = sorted((load_record(path, map_frame=map_frame, map_session_id=map_session_id, map_revision=map_revision)
                      for path in sorted(Path(output_dir).glob('frozen-*.json'))), key=lambda r: r['map_revision'])
    if [r['map_revision'] for r in records] != list(range(map_revision + 1, map_revision + 1 + len(records))):
        raise FreezeError(f'{output_dir} freeze records are not one run (revisions '
                          f'{[r["map_revision"] for r in records]}); use a new output_dir for a new run')
    return records


class MapFreeze:
    def __init__(self, *, map_frame, map_session_id, map_revision, pose_max_age_ns, output_dir, wall_clock_ns,
                 git_sha, support_start=None):
        """``support_start``: (TB-5 robot id, its measured start [x, y, yaw] in TB-6's start pose), or None when
        not measured (nothing is cleared then)."""
        if type(pose_max_age_ns) is not int or pose_max_age_ns <= 0 or type(map_revision) is not int or map_revision < 0:
            raise FreezeError('positive pose_max_age_ns and map_revision >= 0 required')
        self.map_frame, self.map_session_id, self.map_revision = map_frame, map_session_id, map_revision
        self.pose_max_age_ns, self.output_dir = pose_max_age_ns, Path(output_dir)
        self.wall_clock_ns, self.git_sha = wall_clock_ns, git_sha
        self.latest_map = self.latest_pose = None
        self.latest_covariance_fresh = False   # the latest pose's covariance is recent enough to count
        self.records = []   # this run's freezes, R0 + 1 first
        self.support_start = support_start
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
        """A SLAM pose with its own covariance; see on_tf_pose."""
        return self.on_tf_pose(p, covariance_fresh=True)

    def on_tf_pose(self, p, *, covariance_fresh):
        """Latest TB-6 pose; the first valid one of the run becomes the start pose (returned once, saved).

        ``covariance_fresh`` False: its covariance (the last SLAM pose's) is too old; it counts for nothing.
        """
        self.latest_pose, self.latest_covariance_fresh = _pose_fields(p), covariance_fresh
        if self.start_pose is not None or not covariance_fresh:
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

    def restore(self, records):
        """Freezes this run already made (load_records), R0 + 1 first."""
        self.records = list(records)

    def freeze(self):
        if len(self.records) >= FREEZES:
            raise FreezeError(f'already frozen two times ({self.records[-1]["occupancy_sha256"]}); '
                              'two freezes per run')
        if self.latest_map is None:
            raise FreezeError('no SLAM map received')
        if self.start_pose is None:
            raise FreezeError('no TB-6 start pose recorded: coordinates measured from the start cannot be placed')
        g, now = self.latest_map, self.wall_clock_ns()
        cleared = None
        if self.support_start is not None:   # TB-5's body off its start spot (C25)
            robot_id, relative = self.support_start
            start = planar(_ns_pose(self.start_pose['position'], self.start_pose['orientation']))
            x, y, _ = compose_planar(start, relative)
            g, cells = clear_disc(g, (x, y), SUPPORT_BODY_CLEAR_M)
            cleared = dict(robot_id=robot_id, xy=[x, y], radius_m=SUPPORT_BODY_CLEAR_M, cells=cells)
        revision = self.map_revision + len(self.records) + 1
        final = revision == self.map_revision + FREEZES
        grid = grid_hash(g, self.map_frame)
        p = None
        if final:   # TB-6 changes to AMCL from this pose: fresh, on the map
            p = self.latest_pose
            _check_pose(p, map_frame=self.map_frame, now_ns=now, max_age_ns=self.pose_max_age_ns)
            if not self.latest_covariance_fresh:
                raise FreezeError('SLAM pose covariance is too old (no recent /pose from slam_toolbox)')
        record = dict(format=FORMAT, map_frame=self.map_frame, map_session_id=self.map_session_id,
                      map_revision=revision, occupancy_sha256=grid.sha256, frozen_at_ns=now,
                      git_sha=self.git_sha, grid=g, pose=p, start_pose=self.start_pose, support_body_cleared=cleared)
        base = self.output_dir / f'frozen-r{revision}-{grid.sha256[:12]}-{now}'
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
        record['final'], record['path'] = final, str(base.with_suffix('.json'))
        self.records.append(record)
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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from rescue_contracts import ros_time_to_ns
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_map_freeze')
    log = node.get_logger()
    S, I = T.PARAMETER_STRING, T.PARAMETER_INTEGER
    values, missing = declare_required(node, {'map_topic': S, 'pose_topic': S, 'map_frame': S, 'map_session_id': S,
                                              'map_revision': I, 'pose_max_age_ns': I, 'output_dir': S,
                                              'tf_topic': S, 'odom_frame': S, 'base_frame': S,
                                              'covariance_max_age_ns': I, 'support_robot_id': S,
                                              'support_start_xy_yaw': T.PARAMETER_DOUBLE_ARRAY},
                                      optional=('support_robot_id', 'support_start_xy_yaw'))
    missing = [name for name in missing if name != 'support_start_xy_yaw']   # left out: TB-5 start not measured
    if missing:
        log.error('map_freeze missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    sha = git_sha()
    said = {'refused': False, 'tilted': False}
    try:
        core = MapFreeze(map_frame=values['map_frame'], map_session_id=values['map_session_id'],
                         map_revision=values['map_revision'], pose_max_age_ns=values['pose_max_age_ns'],
                         output_dir=values['output_dir'], wall_clock_ns=time.time_ns, git_sha=sha,
                         support_start=((values['support_robot_id'], list(values['support_start_xy_yaw']))
                                        if values['support_start_xy_yaw'] and values['support_robot_id'] else None))
        slam = SlamTfPose(map_frame=values['map_frame'], odom_frame=values['odom_frame'],
                          base_frame=values['base_frame'], covariance_max_age_ns=values['covariance_max_age_ns'])
    except ContractError as exc:
        log.error(f'map_freeze TF pose settings: {exc}')
        raise SystemExit(2)
    except FreezeError as exc:
        log.error(f'START_POSE_RESTORE_FAILED {exc}')
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    frozen_pub = node.create_publisher(OccupancyGrid, FROZEN_TOPIC, latched)
    revision_pub = node.create_publisher(String, FROZEN_REVISION_TOPIC, latched)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, FROZEN_POSE_TOPIC, latched)
    start_pub = node.create_publisher(PoseWithCovarianceStamped, START_POSE_TOPIC, latched)

    def publish_start(start, how):
        start_pub.publish(_pose_message({'pose': start}))
        x, y, yaw = planar(_ns_pose(start['position'], start['orientation']))
        log.info(f'START_POSE {how} robot_a map=({x:.3f}, {y:.3f}, {yaw:.3f}) session={values["map_session_id"]} '
                 f'rev={values["map_revision"]} -> {START_POSE_TOPIC} (origin of coordinates measured in advance)')

    def publish(record):
        frozen_pub.publish(_grid_message(record, node.get_clock().now().to_msg()))
        revision_pub.publish(String(data=revision_text(record)))
        tb6 = 'tb6 keeps SLAM'
        if record['final']:
            pose_pub.publish(_pose_message(record))
            x, y, yaw = planar(_ns_pose(record['pose']['position'], record['pose']['orientation']))
            tb6 = f'tb6 amcl start=({x:.3f}, {y:.3f}, {yaw:.3f})'
        body = record.get('support_body_cleared')
        if body is not None:
            log.info(f'FREEZE_CLEARED_SUPPORT_BODY {body["robot_id"]} xy=({body["xy"][0]:.3f}, {body["xy"][1]:.3f}) '
                     f'radius={body["radius_m"]} m cells={body["cells"]} rev={record["map_revision"]}')
        else:
            log.warning(f'FREEZE_SUPPORT_BODY_NOT_CLEARED rev={record["map_revision"]}: no measured TB-5 start '
                        '(robots.<TB-5>.initial_pose_xy_yaw); its body may be on the frozen map')
        log.info(f'MAP_FROZEN sha256={record["occupancy_sha256"]} session={record["map_session_id"]} '
                 f'rev={record["map_revision"]} final={record["final"]} {tb6} record={record["path"]}')

    def on_map(msg):
        o = msg.info.origin
        core.on_map(dict(frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                         resolution=msg.info.resolution, position=(o.position.x, o.position.y, o.position.z),
                         orientation=(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                         data=list(msg.data)))

    def on_pose(msg):   # slam_toolbox pose: only its covariance (TF gives the pose, OPS-179)
        if msg.header.frame_id == values['map_frame'] and len(msg.pose.covariance) == 36:
            slam.on_covariance(ros_time_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec),
                               [float(v) for v in msg.pose.covariance])

    def on_tf(msg):
        if slam.on_tf(msg) and not said['refused']:
            said['refused'] = True
            log.warning('SLAM_TF_POSE a map/odom/base edge that is not finite or not a unit rotation was not used')

    def tf_tick():
        pose = slam.pose(now_ns=time.time_ns())
        if pose is None:
            if slam.tilt_rad is not None and slam.tilt_rad > MAX_TILT_RAD and not said['tilted']:
                said['tilted'] = True
                log.warning(f'SLAM_TF_POSE map -> base tilted {math.degrees(slam.tilt_rad):.1f} deg '
                            f'> {math.degrees(MAX_TILT_RAD):.0f} deg: no TB-6 pose')
            return   # no TF edge, no SLAM pose yet or tilted: no TB-6 pose
        start = core.on_tf_pose(tf_pose_fields(pose, values['map_frame']), covariance_fresh=pose['covariance_fresh'])
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

    log.info(f'git_sha={sha} parameters={values}')
    if core.start_pose is not None:   # same run restarted: the saved start, never a new one
        publish_start(core.start_pose, 'restored')
    try:   # same run restarted: every freeze it made, in order (the last stays latched)
        records = load_records(values['output_dir'], map_frame=values['map_frame'],
                               map_session_id=values['map_session_id'], map_revision=values['map_revision'])
    except FreezeError as exc:
        log.error(f'MAP_FREEZE_RESTORE_FAILED {exc}')
        raise SystemExit(2)
    core.restore(records)
    if core.start_pose is None and records:
        core.start_pose = records[0]['start_pose']
        publish_start(core.start_pose, 'restored from the freeze record')
    for record in records:
        publish(record)
    if len(records) < FREEZES:
        node.create_subscription(OccupancyGrid, values['map_topic'], on_map, latched)
        node.create_subscription(PoseWithCovarianceStamped, values['pose_topic'], on_pose, qos_profile_sensor_data)
        node.create_subscription(TFMessage, values['tf_topic'], on_tf, 100)
        node.create_timer(TF_POSE_PERIOD_S, tf_tick)
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
