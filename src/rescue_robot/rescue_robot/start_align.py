"""TB-5 start alignment on the frozen TB-6 map (재범 decision 2026-10-15, method B).

frozen_localization.launch.py starts AMCL at the measured start (set_initial_pose parameter). Nav2
Jazzy builds that pose with an all-zero covariance [Nav2 jazzy amcl_node.cpp:283-295, 621-631]:
every particle sits on the measured pose, so AMCL cannot correct a tape-measure error while TB-5
stands still, and its small covariance passes the pose validity check at once.

This node, once TB-5 is on the frozen map, undocked, stopped and scanning, and AMCL's services are
there:
1. re-spreads the particles around AMCL's current pose (``<ns>/set_initial_pose`` with
   ``initial_sigma_*``; the measured start is only this first guess);
2. asks a no-motion update (``<ns>/request_nomotion_update``) every ``update_period_s``, at most
   ``max_updates`` times;
3. accepts AMCL's pose only when its spread is under ``accept_sigma_*``, the last ``settle_count``
   poses agree within ``settle_*``, it moved at most ``max_jump_*`` from the first guess, and at
   least ``min_match_ratio`` of the scan points (``min_match_points`` or more) lie within
   ``match_hit_m`` of an occupied cell of the frozen map.
The result is latched on ``<ns>/rescue_start_align`` (JSON) with a reason code; a failure is never
replaced by another method. ``~/restart`` starts again (TB-5 put back on its spot first).
Every value is [제안] until measured in a supervised run.
"""
import json
import math

CODES = ('START_ALIGN_WAIT_MAP', 'START_ALIGN_DOCKED', 'START_ALIGN_NOT_STOPPED', 'START_ALIGN_NO_SCAN',
         'START_ALIGN_WAIT_AMCL', 'START_ALIGN_NO_SERVICE', 'START_ALIGN_NOT_CONVERGED',
         'START_ALIGN_JUMP_TOO_LARGE', 'START_ALIGN_LOW_MATCH')
OCCUPIED = 65   # occupancy value counted as a wall (Nav2 map_server occupied_thresh 0.65; slam_toolbox writes 100)
ROUNDING = 1e-9   # m^2 / rad^2: below this a variance is rounding, not a value
COUNTS = ('max_updates', 'settle_count', 'min_match_points')
FIELDS = ('initial_sigma_xy_m', 'initial_sigma_yaw_rad', 'update_period_s', 'max_updates', 'accept_sigma_xy_m',
          'accept_sigma_yaw_rad', 'settle_count', 'settle_xy_m', 'settle_yaw_rad', 'max_jump_xy_m',
          'max_jump_yaw_rad', 'match_hit_m', 'min_match_ratio', 'min_match_points', 'scan_max_age_s',
          'stopped_linear_mps', 'stopped_angular_rps', 'service_wait_s')


class AlignConfig:
    def __init__(self, *, initial_sigma_xy_m, initial_sigma_yaw_rad, update_period_s, max_updates,
                 accept_sigma_xy_m, accept_sigma_yaw_rad, settle_count, settle_xy_m, settle_yaw_rad, max_jump_xy_m,
                 max_jump_yaw_rad, match_hit_m, min_match_ratio, min_match_points, scan_max_age_s,
                 stopped_linear_mps, stopped_angular_rps, service_wait_s):
        values = dict(locals())
        for name in FIELDS:
            value = values[name]
            if name in COUNTS:
                if type(value) is not int or value < (2 if name == 'settle_count' else 1):
                    raise ValueError(f'start_align {name} must be an integer'
                                     f' >= {2 if name == "settle_count" else 1}')
            elif (not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value)
                  or value <= 0):
                raise ValueError(f'start_align {name} must be a positive number')
            setattr(self, name, value)
        if min_match_ratio > 1:
            raise ValueError('start_align min_match_ratio must be at most 1')


def sigmas(covariance):
    """(largest x-y standard deviation, yaw standard deviation) of a 6x6 pose covariance; else ValueError."""
    c = list(covariance)
    if len(c) != 36 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in c):
        raise ValueError('6x6 finite covariance required')
    if min(c[0], c[7], c[35]) < -ROUNDING:
        raise ValueError('covariance is not a covariance')
    # AMCL with zero spread publishes rounding just below zero (yaw -5.7e-14, E3 2026-10-15)
    vx, vy, cxy, vyaw = max(c[0], 0.), max(c[7], 0.), c[1], max(c[35], 0.)
    if not math.isclose(cxy, c[6], abs_tol=ROUNDING) or cxy * cxy > vx * vy + ROUNDING:
        raise ValueError('covariance is not a covariance')
    return math.sqrt((vx + vy + math.hypot(vx - vy, 2 * cxy)) / 2), math.sqrt(vyaw)


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def _angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def match_ratio(grid, scan, *, pose, laser_in_base, hit_m):
    """(scan points within ``hit_m`` of an occupied cell, valid scan points) with the robot at ``pose``.

    ``grid``: width, height, resolution, origin (x, y, yaw), data; ``scan``: LaserScan fields;
    ``laser_in_base``: the laser frame in the robot frame (x, y, yaw), from TF.
    """
    width, height, res = grid['width'], grid['height'], float(grid['resolution'])
    ox, oy, oyaw = grid['origin']
    oc, osn = math.cos(oyaw), math.sin(oyaw)
    data = grid['data']
    lx, ly, lyaw = _compose(pose, laser_in_base)
    reach = math.ceil(hit_m / res)
    hits = points = 0
    for i, r in enumerate(scan['ranges']):
        if not (isinstance(r, (int, float)) and math.isfinite(r) and scan['range_min'] <= r <= scan['range_max']):
            continue
        points += 1
        a = lyaw + scan['angle_min'] + i * scan['angle_increment']
        px, py = lx + r * math.cos(a), ly + r * math.sin(a)
        gx, gy = oc * (px - ox) + osn * (py - oy), -osn * (px - ox) + oc * (py - oy)   # map -> grid frame
        cx, cy = math.floor(gx / res), math.floor(gy / res)
        hit = False
        for y in range(max(cy - reach, 0), min(cy + reach, height - 1) + 1):
            for x in range(max(cx - reach, 0), min(cx + reach, width - 1) + 1):
                if (data[y * width + x] >= OCCUPIED
                        and math.hypot((x + 0.5) * res - gx, (y + 0.5) * res - gy) <= hit_m):
                    hit = True
                    break
            if hit:
                break
        hits += hit
    return hits, points


class AlignCore:
    """The alignment steps and their verdict; no ROS. ``tick`` returns the service calls to make."""

    def __init__(self, config):
        self.config = config
        self.map = self.map_sha256 = self.docked = self.odom = self.scan = self.laser_in_base = None
        self.scan_received_ns = None
        self.amcl = None                   # latest (pose, covariance)
        self.services_ready = None
        self.now_ns = None
        self.restart()

    def restart(self):
        self.status, self.reason = 'WAITING', ''
        self.service_wait_from = None
        self.center = None                 # first guess: AMCL's pose when the alignment starts
        self.acked = False
        self.poses = []                    # ((x, y, yaw), (sigma_xy, sigma_yaw)) after the re-spread
        self.updates, self.next_update_ns = 0, None
        self.verdict = {}

    # inputs
    def on_map(self, grid, *, sha256):
        self.map, self.map_sha256 = grid, sha256

    def on_dock(self, is_docked):
        self.docked = bool(is_docked)

    def on_odom(self, linear_mps, angular_rps):
        self.odom = (abs(linear_mps), abs(angular_rps))

    def on_scan(self, scan, *, laser_in_base, received_ns):
        self.scan, self.laser_in_base, self.scan_received_ns = scan, tuple(laser_in_base), received_ns

    def on_amcl(self, pose, covariance):
        spread = sigmas(covariance)        # ValueError: not used
        self.amcl = (tuple(float(v) for v in pose), spread)
        if self.status == 'ALIGNING' and self.acked:
            self.poses.append(self.amcl)

    def on_services(self, ready, now_ns):
        self.services_ready = bool(ready)

    def on_initial_set(self):
        if self.status == 'ALIGNING':
            self.acked = True

    def fail(self, reason):
        if self.status in ('WAITING', 'ALIGNING'):
            self._final('FAILED', reason)

    # verdict
    def state(self):
        if self.status == 'WAITING':
            return 'WAITING', self._waiting()
        return self.status, self.reason

    def _stopped(self):
        return (self.odom is not None and self.odom[0] <= self.config.stopped_linear_mps
                and self.odom[1] <= self.config.stopped_angular_rps)

    def _scan_fresh(self):
        return self.scan is not None and (self.now_ns is None or self.now_ns - self.scan_received_ns
                                          <= self.config.scan_max_age_s * 1e9)

    def _waiting(self):
        if self.map is None:
            return 'START_ALIGN_WAIT_MAP'
        if self.docked is not False:       # unknown counts as docked
            return 'START_ALIGN_DOCKED'
        if not self._stopped():
            return 'START_ALIGN_NOT_STOPPED'
        if not self._scan_fresh():
            return 'START_ALIGN_NO_SCAN'
        if self.amcl is None:
            return 'START_ALIGN_WAIT_AMCL'
        if not self.services_ready:
            return 'START_ALIGN_NO_SERVICE'
        return ''

    def _final(self, status, reason):
        self.status, self.reason = status, reason

    def tick(self, now_ns):
        self.now_ns = now_ns
        cfg = self.config
        if self.status == 'WAITING':
            reason = self._waiting()
            if reason == 'START_ALIGN_NO_SERVICE':
                self.service_wait_from = self.service_wait_from or now_ns
                if now_ns - self.service_wait_from >= cfg.service_wait_s * 1e9:
                    self._final('FAILED', reason)
                return []
            self.service_wait_from = None
            if reason:
                return []
            self.center = self.amcl[0]
            self.status = 'ALIGNING'
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = cfg.initial_sigma_xy_m ** 2
            covariance[35] = cfg.initial_sigma_yaw_rad ** 2
            return [('set_initial_pose', self.center, covariance)]
        if self.status != 'ALIGNING':
            return []
        if self.docked is not False:
            self._final('FAILED', 'START_ALIGN_DOCKED')
            return []
        if not self._stopped():
            self._final('FAILED', 'START_ALIGN_NOT_STOPPED')
            return []
        if not self._scan_fresh():
            self._final('FAILED', 'START_ALIGN_NO_SCAN')
            return []
        if not self.acked:
            return []
        if self._judge():
            return []
        if self.updates >= cfg.max_updates:
            self._final('FAILED', 'START_ALIGN_NOT_CONVERGED')
            return []
        if self.next_update_ns is None or now_ns >= self.next_update_ns:
            self.updates += 1
            self.next_update_ns = now_ns + int(cfg.update_period_s * 1e9)
            return [('nomotion_update',)]
        return []

    def _judge(self):
        """True when a verdict was reached on the poses received so far."""
        cfg = self.config
        if len(self.poses) < cfg.settle_count:
            return False
        last = self.poses[-cfg.settle_count:]
        pose, (sxy, syaw) = last[-1]
        if sxy > cfg.accept_sigma_xy_m or syaw > cfg.accept_sigma_yaw_rad:
            return False
        if any(math.hypot(p[0] - pose[0], p[1] - pose[1]) > cfg.settle_xy_m
               or abs(_angle(p[2] - pose[2])) > cfg.settle_yaw_rad for p, _ in last):
            return False
        jump_xy = math.hypot(pose[0] - self.center[0], pose[1] - self.center[1])
        jump_yaw = abs(_angle(pose[2] - self.center[2]))
        hits, points = match_ratio(self.map, self.scan, pose=pose, laser_in_base=self.laser_in_base,
                                   hit_m=cfg.match_hit_m)
        ratio = hits / points if points else 0.0
        self.verdict = dict(pose=list(pose), sigma_xy_m=sxy, sigma_yaw_rad=syaw, jump_xy_m=jump_xy,
                            jump_yaw_rad=jump_yaw, match_ratio=ratio, match_points=points)
        if jump_xy > cfg.max_jump_xy_m or jump_yaw > cfg.max_jump_yaw_rad:
            self._final('FAILED', 'START_ALIGN_JUMP_TOO_LARGE')
        elif points < cfg.min_match_points or ratio < cfg.min_match_ratio:
            self._final('FAILED', 'START_ALIGN_LOW_MATCH')
        else:
            self._final('ALIGNED', '')
        return True

    def result(self):
        status, reason = self.state()
        return dict(status=status, reason=reason, updates=self.updates, map_sha256=self.map_sha256,
                    first_guess=list(self.center) if self.center else None, **self.verdict)


def main(args=None):
    import time

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from irobot_create_msgs.msg import DockStatus
    from nav2_msgs.srv import SetInitialPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Empty, Trigger
    import tf2_ros

    from rescue_robot.live_sources import Grid, planar
    from rescue_robot.ros_node import declare_required, git_sha
    from rescue_robot.slam_tf_pose import MAX_TILT_RAD, compose_to_plane

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_start_align')
    log = node.get_logger()
    S, D, I = T.PARAMETER_STRING, T.PARAMETER_DOUBLE, T.PARAMETER_INTEGER
    names = {key: S for key in ('map_topic', 'scan_topic', 'odom_topic', 'dock_topic', 'amcl_pose_topic',
                                'set_initial_pose_service', 'nomotion_update_service', 'status_topic', 'base_frame',
                                'map_frame')}
    names.update({key: I if key in COUNTS else D for key in FIELDS})
    values, missing = declare_required(node, names)
    if missing:
        log.error('start_align missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    try:
        core = AlignCore(AlignConfig(**{key: values[key] for key in FIELDS}))
    except ValueError as exc:
        log.error(f'start_align {exc}')
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = node.create_publisher(String, values['status_topic'], latched)
    set_pose = node.create_client(SetInitialPose, values['set_initial_pose_service'])
    nomotion = node.create_client(Empty, values['nomotion_update_service'])
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    said = {'state': None, 'laser': None}

    def publish():
        result = core.result()
        key = (result['status'], result['reason'])
        if said['state'] == key:   # each state once (latched)
            return
        said['state'] = key
        status.publish(String(data=json.dumps(result, sort_keys=True)))
        line = f'START_ALIGN_{result["status"]} {result["reason"]} ' + json.dumps(result, sort_keys=True)
        if result['status'] == 'FAILED':   # one severity per call site (rclpy raises otherwise)
            log.error(line)
        else:
            log.info(line)

    def on_map(message):
        try:
            grid = Grid(message)
        except Exception as exc:   # an invalid grid is never used
            log.error(f'start_align map refused: {exc}')
            return
        core.on_map(dict(width=grid.width, height=grid.height, resolution=grid.resolution, origin=grid.origin,
                         data=grid.data), sha256=grid.sha256)

    def on_scan(message):
        try:
            t = tf_buffer.lookup_transform(values['base_frame'], message.header.frame_id, Time()).transform
            x, y, yaw, tilt = compose_to_plane(((t.translation.x, t.translation.y, t.translation.z),
                                                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)),
                                               ((0., 0., 0.), (0., 0., 0., 1.)))
            if tilt > MAX_TILT_RAD:
                raise ValueError(f'laser tilted {math.degrees(tilt):.1f} deg')
        except Exception as exc:   # no laser mount: no scan for the match, START_ALIGN_NO_SCAN
            if said['laser'] != str(exc):
                said['laser'] = str(exc)
                log.warning(f'start_align laser {message.header.frame_id} in {values["base_frame"]}: {exc}')
            return
        core.on_scan(dict(angle_min=message.angle_min, angle_increment=message.angle_increment,
                          range_min=message.range_min, range_max=message.range_max, ranges=list(message.ranges)),
                     laser_in_base=(x, y, yaw), received_ns=time.time_ns())

    def on_odom(message):
        v = message.twist.twist
        core.on_odom(math.hypot(v.linear.x, v.linear.y), v.angular.z)

    def on_amcl(message):
        if message.header.frame_id != values['map_frame']:
            log.warning(f'start_align AMCL pose in {message.header.frame_id!r}, not {values["map_frame"]!r}: not used')
            return
        try:
            core.on_amcl(planar(message.pose.pose), list(message.pose.covariance))
        except Exception as exc:   # a broken covariance or pose is not used
            log.warning(f'start_align AMCL pose not used: {exc}')

    def on_initial_set(future):
        if future.exception() is not None:
            core.fail('START_ALIGN_NO_SERVICE')
            log.error(f'start_align set_initial_pose failed: {future.exception()}')
        else:
            core.on_initial_set()

    def tick():
        now = time.time_ns()
        core.on_services(set_pose.service_is_ready() and nomotion.service_is_ready(), now)
        for action in core.tick(now):
            if action[0] == 'set_initial_pose':
                (x, y, yaw), covariance = action[1], action[2]
                request = SetInitialPose.Request()
                p = request.pose
                p.header.frame_id = values['map_frame']   # stamp 0: AMCL uses the latest odom
                p.pose.pose.position.x, p.pose.pose.position.y = float(x), float(y)
                p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.pose.covariance = [float(v) for v in covariance]
                set_pose.call_async(request).add_done_callback(on_initial_set)
            else:
                nomotion.call_async(Empty.Request())
        publish()

    def restart(_request, response):
        core.restart()
        said['state'] = None
        response.success, response.message = True, 'start alignment restarted'
        log.info('START_ALIGN restart requested')
        return response

    log.info(f'git_sha={git_sha()} parameters={values}')
    node.create_subscription(OccupancyGrid, values['map_topic'], on_map, latched)
    node.create_subscription(LaserScan, values['scan_topic'], on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, values['odom_topic'], on_odom, qos_profile_sensor_data)
    node.create_subscription(DockStatus, values['dock_topic'], lambda m: core.on_dock(m.is_docked),
                             qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, values['amcl_pose_topic'], on_amcl, latched)
    node.create_service(Trigger, '~/restart', restart)
    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
