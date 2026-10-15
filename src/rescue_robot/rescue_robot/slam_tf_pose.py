"""TB-6 pose while searching = TF map -> base_link (map lifecycle, OPS-177); no ROS.

slam_toolbox publishes <ns>/pose only for the scans it processes, and it processes a scan only
after the robot moved far enough, so a stopped TB-6 gets no new SLAM pose [estimate, slam_toolbox
defaults]. In the SLAM phase the executor therefore reads TF on its poll period: map -> odom
(slam_toolbox, republished periodically) composed with odom -> base_link (Create3). This is the
pose source of the phase, not a fallback. Its time is the older of the two edges (a stopped
edge makes the pose old, never fresh). The covariance is the last <ns>/pose's (TF has none):
older than ``covariance_max_age_ns`` it is still shown but the pose is not valid. Without both
edges or before any SLAM pose there is no pose.

The edges are 3D (2026-10-14 field: TB-6 map -> odom and odom -> base_link carry a few mm of height and
about 0.5 deg of roll/pitch, and the old per-edge planar check refused both, so the map freeze had no
start pose). They are composed in 3D first, then map -> base_link is projected to the map plane: x, y
and the heading of base_link's x axis. A non-finite edge or a non-unit rotation is refused; a composed
tilt above ``MAX_TILT_RAD`` gives no pose (the plane heading would mean nothing).
"""
import math
from types import SimpleNamespace as NS

from rescue_contracts import ros_time_to_ns
from rescue_robot.executor import ContractError

MAX_TILT_RAD = math.radians(10)   # [제안] composed map -> base_link tilt; measured 2026-10-14 about 0.5 deg


def _edge(p, q):
    """((x, y, z), unit (x, y, z, w)) of one TF edge; else ValueError."""
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
            or not math.isclose(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1., abs_tol=1e-6)):
        raise ValueError('finite edge with a unit rotation required')
    return tuple(float(v) for v in values[:3]), tuple(float(v) for v in values[3:])


def _rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def compose_to_plane(first, second):
    """``first`` then ``second`` (each (xyz, xyzw)) composed in 3D -> (x, y, yaw, tilt_rad) on the map plane."""
    (p1, q1), (p2, q2) = first, second
    r = _rotate(q1, p2)
    q = _multiply(q1, q2)
    heading = _rotate(q, (1., 0., 0.))
    up = _rotate(q, (0., 0., 1.))
    tilt = math.acos(max(-1., min(1., up[2])))
    return p1[0] + r[0], p1[1] + r[1], math.atan2(heading[1], heading[0]), tilt


class SlamTfPose:
    def __init__(self, *, map_frame, odom_frame, base_frame, covariance_max_age_ns):
        for name, value in (('map_frame', map_frame), ('odom_frame', odom_frame), ('base_frame', base_frame)):
            if not isinstance(value, str) or not value:
                raise ContractError(f'SLAM TF pose needs {name}')
        if type(covariance_max_age_ns) is not int or covariance_max_age_ns <= 0:
            raise ContractError('SLAM TF pose needs a positive integer covariance_max_age_ns')
        self.edges = {(map_frame, odom_frame): None, (odom_frame, base_frame): None}   # -> (stamp_ns, xyz, xyzw)
        self.covariance = None   # (stamp_ns, list) of the last SLAM pose
        self.covariance_max_age_ns = covariance_max_age_ns
        self.tilt_rad = None   # of the last composed map -> base_link, for the log

    def on_tf(self, message):
        """Keep the newest of each edge; returns how many edges were refused (not finite, not a unit rotation)."""
        refused = 0
        for t in message.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in self.edges:
                continue
            p, q = t.transform.translation, t.transform.rotation
            try:
                edge = _edge(p, q)
            except ValueError:
                refused += 1
                continue
            stamp = ros_time_to_ns(t.header.stamp.sec, t.header.stamp.nanosec)
            if self.edges[key] is None or stamp > self.edges[key][0]:
                self.edges[key] = (stamp, *edge)
        return refused

    def on_covariance(self, stamp_ns, covariance):
        if len(covariance) != 36:
            raise ContractError('6x6 covariance required')
        if self.covariance is None or stamp_ns > self.covariance[0]:
            self.covariance = (stamp_ns, [float(v) for v in covariance])

    def pose(self, *, now_ns):
        (to_odom, to_base) = self.edges.values()
        if to_odom is None or to_base is None or self.covariance is None:
            return None
        x, y, yaw, self.tilt_rad = compose_to_plane(to_odom[1:], to_base[1:])
        if self.tilt_rad > MAX_TILT_RAD:
            return None
        stamp, covariance = self.covariance
        return dict(stamp_ns=min(to_odom[0], to_base[0]), x=x, y=y, yaw=yaw, covariance=list(covariance),
                    covariance_fresh=now_ns - stamp <= self.covariance_max_age_ns)


class AmclTfPose(SlamTfPose):
    """Frozen-map phase pose = TF map -> base_link: map -> odom from AMCL (sent again with every scan, tf_broadcast)
    and odom -> base_link from Create3 (simulation 2026-10-16 run 3: a standing robot had no amcl_pose, so no pose).

    The covariance is the last amcl_pose's taken after ``after_ns`` (the map switch). It is AMCL's current state
    while the robot has moved less than AMCL's own update thresholds since that amcl_pose (``update_min_d`` /
    ``update_min_a``, localization.yaml): AMCL has not updated, so nothing newer exists. Past them without a new
    amcl_pose the pose is not valid. No amcl_pose after the switch, or no edge: no pose.
    """

    def __init__(self, *, map_frame, odom_frame, base_frame, update_min_d, update_min_a):
        # covariance_max_age_ns is not used here: pose() replaces the age rule with AMCL's motion rule
        super().__init__(map_frame=map_frame, odom_frame=odom_frame, base_frame=base_frame, covariance_max_age_ns=1)
        for name, value in (('update_min_d', update_min_d), ('update_min_a', update_min_a)):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ContractError(f'AMCL TF pose needs a positive {name} (AMCL localization parameters)')
        self.update_min_d, self.update_min_a = float(update_min_d), float(update_min_a)
        self.odom_key = (odom_frame, base_frame)
        self.map_key = (map_frame, odom_frame)
        self.anchor = None   # odom -> base_link (x, y, yaw) when the last amcl_pose came
        self.first_map_odom = None   # (after_ns, stamp): the first map -> odom stamped after a map switch

    def note_map_odom(self, after_ns):
        """Remember the first map -> odom stamped after ``after_ns`` (the map switch): AMCL's, stamped ahead by its
        transform_tolerance. Called after on_tf with the switch time."""
        edge = self.edges[self.map_key]
        if edge is None or edge[0] <= after_ns:
            return
        if self.first_map_odom is None or self.first_map_odom[0] != after_ns:
            self.first_map_odom = (after_ns, edge[0])

    def map_odom_covers(self, *, now_ns, after_ns):
        """Nav2's TF buffer has a map -> odom for ``now_ns`` from after the switch: now has passed the first one."""
        first = self.first_map_odom
        return first is not None and first[0] == after_ns and now_ns >= first[1]

    def _odom_plane(self):
        edge = self.edges[self.odom_key]
        if edge is None:
            return None
        x, y, yaw, _ = compose_to_plane(((0., 0., 0.), (0., 0., 0., 1.)), edge[1:])
        return x, y, yaw

    def on_covariance(self, stamp_ns, covariance):
        super().on_covariance(stamp_ns, covariance)
        if self.covariance[0] == stamp_ns:
            self.anchor = self._odom_plane()

    def needs_covariance(self, *, after_ns):
        """No amcl_pose since ``after_ns`` (the map switch): the TF pose cannot be made yet."""
        return self.covariance is None or self.covariance[0] <= after_ns

    def pose(self, *, now_ns, after_ns=0):
        if self.covariance is not None and self.covariance[0] <= after_ns:
            return None   # an amcl_pose from the previous map
        result = super().pose(now_ns=now_ns)
        if result is None:
            return None
        here = self._odom_plane()
        moved = turned = math.inf
        if self.anchor is not None:
            moved = math.hypot(here[0] - self.anchor[0], here[1] - self.anchor[1])
            turned = abs(math.atan2(math.sin(here[2] - self.anchor[2]), math.cos(here[2] - self.anchor[2])))
        result['covariance_fresh'] = moved < self.update_min_d and turned < self.update_min_a
        return result


def pose_message(pose, frame, factory=None):
    """A PoseWithCovarianceStamped (``factory``) or a look-alike for SavedMapSources.receive_pose."""
    sec, nanosec = divmod(pose['stamp_ns'], 1_000_000_000)
    qz, qw = math.sin(pose['yaw'] / 2), math.cos(pose['yaw'] / 2)
    if factory is None:
        return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
                  pose=NS(pose=NS(position=NS(x=pose['x'], y=pose['y'], z=0.0), orientation=NS(x=0.0, y=0.0, z=qz, w=qw)),
                          covariance=list(pose['covariance'])))
    message = factory()
    message.header.frame_id = frame
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    position, orientation = message.pose.pose.position, message.pose.pose.orientation
    position.x, position.y, position.z = float(pose['x']), float(pose['y']), 0.0
    orientation.x, orientation.y, orientation.z, orientation.w = 0.0, 0.0, qz, qw
    message.pose.covariance = [float(v) for v in pose['covariance']]
    return message
