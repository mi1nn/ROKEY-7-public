#!/usr/bin/env python3
"""2D stand-in for the two TurtleBot4s, their Pis and the doll detector (one-PC full-scenario rehearsal only).

Not field data, not a physics simulation. Replaces exactly what the robots and the OAK-D detector would publish, on
the field names of the profile, so the real SLAM, AMCL, start_align, Nav2, executors, watchdogs, coordinator,
operations nodes, dock node and alert run unchanged (scripts/field/full-scenario-rehearsal/run.sh):

- per robot: ``<ns>/odom`` (perfect odometry from the robot's own start), ``<ns>/tf`` odom -> base_link,
  ``<ns>/tf_static`` base_link -> rplidar_link / oakd_rgb_camera_optical_frame, ``<ns>/scan`` (360 rays cast in the
  world grid, the other robot included as a 0.17 m disc), ``<ns>/dock_status``, the readiness camera topics and the
  passage depth topic (empty CompressedImage), ``<ns>/passage_points`` (empty cloud: no overhead structure), and the
  robot moves by integrating ``<ns>/cmd_vel`` (TwistStamped from the command watchdog; 0.5 s without one = stop;
  a pose that would hit a wall is not taken);
- ``<ns>/dock`` (Create3 Dock action): docks only within ``dock_radius_m`` of the world's dock pose (then it moves the
  robot onto it and reports is_docked), else answers is_docked false; ``<ns>/undock`` likewise;
- the doll detector: once TB-6's camera (bearing within ``fov_rad``, closer than ``see_m``, line of sight in the
  grid) sees the doll, ONE TargetObservation on /rescue/observations (one-shot, like amr_target) with the doll in
  the camera optical frame at the capture time of a published odom TF; the real target_locator projects it with the
  SLAM TF. The exit QR likewise on the exit observations topic (track_id = the world's exit id).

    python3 sim_world.py --profile <profile.json> --field <field.json> --world <world.json>
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

import numpy as np

RADIUS = 0.17            # TurtleBot4 body radius used for walls and the other robot in the scan [estimate]
LASER = (-0.04, 0.0, 0.19)   # base_link -> rplidar_link [estimate, rehearsal only]
CAMERA = (0.06, 0.0, 0.24)   # base_link -> oakd_rgb_camera_optical_frame (RPY -90, 0, -90) [estimate]
RAYS, RANGE_MIN, RANGE_MAX = 1440, 0.15, 12.0   # about an RPLIDAR A1 revolution (0.25 deg) [estimate]


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class World:
    def __init__(self, spec):
        self.res = float(spec['resolution'])
        self.w, self.h = int(round(spec['width_m'] / self.res)), int(round(spec['height_m'] / self.res))
        self.occ = np.zeros((self.h, self.w), dtype=bool)
        for x0, y0, x1, y1 in spec['walls']:
            i0, i1 = sorted((int(math.floor(x0 / self.res)), int(math.ceil(x1 / self.res))))
            j0, j1 = sorted((int(math.floor(y0 / self.res)), int(math.ceil(y1 / self.res))))
            self.occ[max(j0, 0):min(j1, self.h), max(i0, 0):min(i1, self.w)] = True

    def blocked(self, x, y, radius=RADIUS):
        r = int(math.ceil(radius / self.res))
        i, j = int(x / self.res), int(y / self.res)
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if (di * di + dj * dj) * self.res ** 2 > radius ** 2:
                    continue
                ii, jj = i + di, j + dj
                if not (0 <= ii < self.w and 0 <= jj < self.h) or self.occ[jj, ii]:
                    return True
        return False

    def cast(self, x, y, angles, others=()):
        steps = np.arange(RANGE_MIN, RANGE_MAX, self.res / 2)
        px = x + np.outer(np.cos(angles), steps)
        py = y + np.outer(np.sin(angles), steps)
        ii, jj = (px / self.res).astype(int), (py / self.res).astype(int)
        inside = (ii >= 0) & (ii < self.w) & (jj >= 0) & (jj < self.h)
        hit = ~inside
        hit[inside] = self.occ[jj[inside], ii[inside]]
        for ox, oy in others:
            hit |= (px - ox) ** 2 + (py - oy) ** 2 < RADIUS ** 2
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        out = np.where(first >= 0, steps[np.clip(first, 0, None)], np.inf)
        return out

    def clear_line(self, a, b):
        n = int(math.dist(a, b) / (self.res / 2)) + 1
        for k in range(1, n):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            i, j = int(x / self.res), int(y / self.res)
            if not (0 <= i < self.w and 0 <= j < self.h) or self.occ[j, i]:
                return False
        return True


class Robot:
    def __init__(self, robot_id, ns, start):
        self.id, self.ns = robot_id, ns
        self.start = tuple(float(v) for v in start)
        self.pose = list(self.start)            # world x, y, yaw
        self.cmd, self.cmd_at = (0.0, 0.0), 0.0
        self.vel = (0.0, 0.0)
        self.docked = False
        self.lock = threading.Lock()

    def odom_pose(self):
        sx, sy, syaw = self.start
        dx, dy = self.pose[0] - sx, self.pose[1] - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(self.pose[2] - syaw), math.cos(self.pose[2] - syaw))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', required=True)
    parser.add_argument('--field', required=True)
    parser.add_argument('--world', required=True)
    args = parser.parse_args()
    spec = json.loads(Path(args.world).read_text())

    import rclpy
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    from rescue_bringup.field_launch import load_field
    from rescue_bringup.profile import load_deployment
    from rescue_interfaces.msg import TargetObservation
    from rescue_perception.exit_qr import EXIT_OBSERVATIONS_TOPIC

    deployment, field = load_deployment(args.profile), load_field(args.field)
    roles = {role: robot for robot, role in deployment.mission.roles}
    world = World(spec)
    robots = {}
    for r in deployment.robots:
        robots[r.id] = Robot(r.id, r.parameters.namespace, spec['robots'][r.id]['start'])
    scout = robots[roles['SCOUT']]

    rclpy.init()
    node = rclpy.create_node('rehearsal_sim_world')
    log = node.get_logger()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {}
    for r in deployment.robots:
        rb, ns, params = robots[r.id], r.parameters.namespace, r.parameters
        cams = list(params.readiness.camera_topics)
        depth = field.robots[r.id].get('passage.depth_topic')
        if depth:
            cams.append(depth + ('/compressedDepth' if field.robots[r.id].get('passage.depth_transport')
                                 == 'compressedDepth' else ''))
        pubs[r.id] = dict(
            odom=node.create_publisher(Odometry, params.odom_topic, qos_profile_sensor_data),
            tf=node.create_publisher(TFMessage, params.readiness.tf_topic, 100),
            tf_static=node.create_publisher(TFMessage, ns + '/tf_static', latched),
            scan=node.create_publisher(LaserScan, params.readiness.scan_topic, qos_profile_sensor_data),
            dock=node.create_publisher(DockStatus, params.dock_status_topic, qos_profile_sensor_data),
            points=node.create_publisher(PointCloud2, ns + '/passage_points', qos_profile_sensor_data),
            cams=[node.create_publisher(CompressedImage, t, qos_profile_sensor_data) for t in dict.fromkeys(cams)])

        def on_cmd(msg, rb=rb):
            with rb.lock:
                rb.cmd, rb.cmd_at = (msg.twist.linear.x, msg.twist.angular.z), time.monotonic()
        node.create_subscription(TwistStamped, ns + '/cmd_vel', on_cmd, 10)

    def stamp(ns_time):
        from builtin_interfaces.msg import Time
        return Time(sec=ns_time // 1_000_000_000, nanosec=ns_time % 1_000_000_000)

    def edge(parent, child, xyz, q, t_ns):
        e = TransformStamped()
        e.header.frame_id, e.child_frame_id, e.header.stamp = parent, child, stamp(t_ns)
        e.transform.translation.x, e.transform.translation.y, e.transform.translation.z = xyz
        e.transform.rotation.x, e.transform.rotation.y, e.transform.rotation.z, e.transform.rotation.w = q
        return e

    now_ns = time.time_ns()
    cam_q = (-0.5, 0.5, -0.5, 0.5)   # RPY (-90, 0, -90): optical z forward, x right, y down
    for rid, rb in robots.items():
        pubs[rid]['tf_static'].publish(TFMessage(transforms=[
            edge('base_link', 'rplidar_link', LASER, (0., 0., 0., 1.), now_ns),
            edge('base_link', 'oakd_rgb_camera_optical_frame', CAMERA, cam_q, now_ns)]))
    last_tf = {}   # robot -> (t_ns, odom pose) of the latest published odom -> base_link

    def step():
        dt = 0.02
        t_ns = time.time_ns()
        for rid, rb in robots.items():
            with rb.lock:
                v, w = rb.cmd if time.monotonic() - rb.cmd_at < 0.5 and not rb.docked else (0.0, 0.0)
                x, y, yaw = rb.pose
                nx, ny = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
                others = [o.pose[:2] for o in robots.values() if o is not rb]
                if world.blocked(nx, ny) or any(math.dist((nx, ny), o) < 2 * RADIUS for o in others):
                    nx, ny, v = x, y, 0.0
                rb.pose = [nx, ny, math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))]
                rb.vel = (v, w)
                ox, oy, oyaw = rb.odom_pose()
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(t_ns), 'odom', 'base_link'
            p = msg.pose.pose
            p.position.x, p.position.y = ox, oy
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat(oyaw)
            msg.twist.twist.linear.x, msg.twist.twist.angular.z = v, w
            pubs[rid]['odom'].publish(msg)
            pubs[rid]['tf'].publish(TFMessage(transforms=[edge('odom', 'base_link', (ox, oy, 0.0), quat(oyaw), t_ns)]))
            last_tf[rid] = (t_ns, (ox, oy, oyaw), list(rb.pose))

    def scans():
        angles = np.linspace(-math.pi, math.pi, RAYS, endpoint=False)
        t_ns = time.time_ns()
        for rid, rb in robots.items():
            with rb.lock:
                x, y, yaw = rb.pose
            lx, ly = x + LASER[0] * math.cos(yaw), y + LASER[0] * math.sin(yaw)
            others = [o.pose[:2] for o in robots.values() if o is not rb]
            ranges = world.cast(lx, ly, angles + yaw, others)
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp(t_ns), 'rplidar_link'
            msg.angle_min, msg.angle_max = float(angles[0]), float(angles[-1])
            msg.angle_increment = float(angles[1] - angles[0])
            msg.scan_time, msg.time_increment = 0.1, 0.0
            msg.range_min, msg.range_max = RANGE_MIN, RANGE_MAX
            msg.ranges = [float(r) for r in ranges]
            pubs[rid]['scan'].publish(msg)

    def slow():
        t_ns = time.time_ns()
        for rid, rb in robots.items():
            d = DockStatus()
            d.header.stamp, d.is_docked, d.dock_visible = stamp(t_ns), rb.docked, False
            pubs[rid]['dock'].publish(d)
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp(t_ns), 'base_link'
            cloud.height, cloud.width, cloud.is_dense = 1, 0, True
            # x, y, z declared like depth_image_proc's cloud: collision_monitor aborts on a cloud without them
            # ('Field x does not exist', rehearsal 2026-10-15)
            cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate('xyz')]
            cloud.point_step, cloud.row_step = 12, 0
            pubs[rid]['points'].publish(cloud)
            for pub in pubs[rid]['cams']:
                img = CompressedImage()
                img.header.stamp, img.header.frame_id, img.format = stamp(t_ns), 'oakd_rgb_camera_optical_frame', 'jpeg'
                pub.publish(img)

    # doll detector and exit QR: one observation each, when TB-6's camera sees them
    # the QoS amr_target publishes with (target_locator asks TRANSIENT_LOCAL: a volatile writer is never matched)
    obs_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    obs_pub = node.create_publisher(TargetObservation, '/rescue/observations', obs_qos)
    exit_pub = node.create_publisher(TargetObservation, EXIT_OBSERVATIONS_TOPIC, obs_qos)
    seen = set()
    see = spec['detector']
    clock_domain = field.coordinator['target_clock_domain_id']
    boot = str(uuid4())

    def observe(kind, xy, publisher, label, track_id=''):
        if kind in seen or scout.id not in last_tf:
            return
        t_ns, _, (x, y, yaw) = last_tf[scout.id]
        cx, cy = x + CAMERA[0] * math.cos(yaw), y + CAMERA[0] * math.sin(yaw)
        dist, bearing = math.dist((cx, cy), xy), math.atan2(xy[1] - cy, xy[0] - cx) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if dist > see['see_m'] or abs(bearing) > see['fov_rad'] or not world.clear_line((cx, cy), xy):
            return
        seen.add(kind)
        fwd, left = dist * math.cos(bearing), dist * math.sin(bearing)
        o = TargetObservation()
        o.observation_id, o.source_robot, o.source_camera, o.source_boot_id = str(uuid4()), scout.id, 'oakd', boot
        t = o.time
        t.observed_at_ns, t.valid_until_ns = t_ns, t_ns + 3_000_000_000
        t.stamp_semantics, t.clock_domain_id, t.clock_state = 'CAPTURE', clock_domain, 'SYNCED'
        t.has_clock_error_bound, t.clock_error_bound_ns = True, 100_000_000
        o.class_label, o.confidence = label, 0.9
        o.model_id, o.model_sha256 = 'rehearsal-sim', hashlib.sha256(b'rehearsal-sim').hexdigest()
        o.tracking_mode = 'NONE'
        o.has_track_id, o.track_id = bool(track_id), track_id
        o.bbox_xyxy_pixels = [100.0, 100.0, 200.0, 300.0]
        o.sensor_frame, o.localization_status, o.depth_status = 'oakd_rgb_camera_optical_frame', 'VALID', 'VALID'
        o.sensor_position.x, o.sensor_position.y, o.sensor_position.z = -left, CAMERA[2] - 0.1, fwd
        o.position_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
        payload = json.dumps(dict(synthetic=True, kind=kind, world_xy=list(xy))).encode()
        e = o.evidence
        e.evidence_id, e.sha256, e.mime_type = str(uuid4()), hashlib.sha256(payload).hexdigest(), 'application/json'
        e.source_id, e.source_boot_id, e.observed_at_ns = 'oakd', boot, t_ns
        env = o.envelope
        env.schema_version, env.publisher_id, env.publisher_boot_id = 'rescue-envelope/1', 'rehearsal.sim_world', boot
        env.sequence, env.published_at_ns = len(seen), time.time_ns()
        publisher.publish(o)
        log.warning(f'SIM_DETECTOR {kind} seen at world {xy} from TB-6 world ({x:.2f}, {y:.2f}, {yaw:.2f}) '
                    f'dist={dist:.2f} bearing={bearing:.2f} -> {o.observation_id}')

    def detect():
        observe('doll', tuple(spec['doll']), obs_pub, 'survivor')
        if 'exit' in spec:
            observe('exit', tuple(spec['exit']['xy']), exit_pub, 'exit_qr', spec['exit']['id'])

    # Create3 dock / undock
    def make_dock(rb):
        def execute(goal):
            target = spec.get('docks', {}).get(rb.id)
            with rb.lock:
                x, y, yaw = rb.pose
            ok = target is not None and math.dist((x, y), target[:2]) <= spec['dock_radius_m']
            log.info(f'SIM_DOCK {rb.id} at ({x:.2f}, {y:.2f}) dock={target} within={ok}')
            if ok:
                for k in range(1, 21):   # 2 s onto the dock
                    with rb.lock:
                        rb.pose = [x + (target[0] - x) * k / 20, y + (target[1] - y) * k / 20, target[2]]
                    time.sleep(0.1)
                with rb.lock:
                    rb.docked = True
            goal.succeed()
            result = Dock.Result()
            result.is_docked = ok
            return result
        return execute

    def make_undock(rb):
        def execute(goal):
            with rb.lock:
                rb.docked = False
            goal.succeed()
            return Undock.Result(is_docked=False)
        return execute

    for rb in robots.values():
        ActionServer(node, Dock, rb.ns + '/dock', make_dock(rb))
        ActionServer(node, Undock, rb.ns + '/undock', make_undock(rb))
    node.create_timer(0.02, step)
    node.create_timer(0.1, scans)
    node.create_timer(0.1, slow)
    node.create_timer(0.2, detect)
    log.info(f'sim world {world.w}x{world.h} cells, robots {[(r.id, r.ns, r.start) for r in robots.values()]}, '
             f'doll {spec["doll"]}, exit {spec.get("exit")}; rehearsal only')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
