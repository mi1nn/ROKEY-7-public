"""RobotPose and MapState from this robot's measured inputs (IF-009/010 shape, SDD v0.1).

The coordinator places the SUPPORT robot from /rescue/robots/<id>/pose and compares homes with
/rescue/robots/<id>/map_state; db_writer shows the pose on the console map. task_input already
checks the AMCL pose (frame, covariance) and the hashed saved map (live_sources); this module
turns them into the two messages. The mission pose is VALID only with a valid AMCL pose, the
loaded map, a current alignment and a SYNCED clock; otherwise UNKNOWN and no mission numbers.
The map basis is the declared one (saved_maps); MapState never advances a revision by itself.
Pure builders here; ``attach`` binds them to ROS.
"""
import math
import uuid

POSE_QOS = ('RELIABLE', 'VOLATILE', 1)            # IF-010
MAP_STATE_QOS = ('RELIABLE', 'TRANSIENT_LOCAL', 1)  # IF-009 (latched latest snapshot)
SNAPSHOT_NAMESPACE = uuid.uuid5(uuid.NAMESPACE_URL, 'rescue_robot/map_state/snapshot')  # same map -> same id


def _finite(*values):
    # rclpy gives covariance as a numpy array (numpy.float64, a float subclass); bool is not a number here.
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise ValueError('finite values required')


def mission_from_native(transform, x, y, yaw, covariance):
    """Native map pose -> mission pose under a planar rigid alignment; xy covariance rotated."""
    _finite(x, y, yaw, *covariance)
    if len(covariance) != 36:
        raise ValueError('6x6 covariance required')
    mx, my, _ = transform.apply((x, y, 0.))
    qx, qy, qz, qw = transform.rotation_xyzw
    turn = 2 * math.atan2(qz, qw)
    c, s = math.cos(turn), math.sin(turn)
    cov = list(covariance)
    vx, vy, cxy = cov[0], cov[7], cov[1]
    cov[0] = c * c * vx - 2 * c * s * cxy + s * s * vy
    cov[7] = s * s * vx + 2 * c * s * cxy + c * c * vy
    cov[1] = cov[6] = c * s * (vx - vy) + (c * c - s * s) * cxy
    if turn == 0:  # keep the measured numbers exactly on the identity alignment
        cov = list(covariance)
    return mx, my, math.atan2(math.sin(yaw + turn), math.cos(yaw + turn)), cov


def pose_record(*, robot_id, boot_id, observed_at_ns, pose_valid_ns, native, amcl_valid, map_loaded,
                transform, basis, clock, clock_domain_id):
    """Fields of one RobotPose. ``transform`` None = no current alignment; ``clock`` from the live clock."""
    if type(observed_at_ns) is not int or observed_at_ns <= 0 or type(pose_valid_ns) is not int or pose_valid_ns <= 0:
        raise ValueError('positive observation time and pose validity required')
    x, y, yaw, covariance = native
    _finite(x, y, yaw, *covariance)
    if len(covariance) != 36:
        raise ValueError('6x6 covariance required')
    synced = clock['clock_state'] == 'SYNCED' and clock.get('error_bound_ns') is not None
    valid = bool(amcl_valid and map_loaded and transform is not None and synced)
    return dict(
        robot_id=robot_id, source_boot_id=boot_id,
        time=dict(observed_at_ns=observed_at_ns, valid_until_ns=observed_at_ns + pose_valid_ns,
                  stamp_semantics='CAPTURE',  # AMCL stamps its pose with the laser scan it used
                  clock_domain_id=clock_domain_id, clock_state='SYNCED' if synced else 'UNKNOWN',
                  has_clock_error_bound=synced, clock_error_bound_ns=clock['error_bound_ns'] if synced else 0),
        native_frame=basis['native_frame'], native_pose=(x, y, yaw, list(covariance)),
        mission_frame=basis['mission_frame'],
        mission_pose=mission_from_native(transform, x, y, yaw, covariance) if valid else None,
        frame_ref=dict(robot_id=robot_id, native_frame=basis['native_frame'], mission_frame=basis['mission_frame'],
                       map_session_id=basis['map_session_id'], map_revision=basis['map_revision'],
                       transform_revision=basis['transform_revision'],
                       alignment_status='VALID' if transform is not None else 'INVALID'),
        mission_pose_status='VALID' if valid else 'UNKNOWN')


def map_state_record(*, robot_id, occupancy_sha256, basis, map_loaded, localization_valid, generated_at_ns):
    """Fields of one MapState: the declared basis, the measured localization quality."""
    key = f'{occupancy_sha256}:{basis["map_session_id"]}:{basis["map_revision"]}:{basis["transform_revision"]}'
    return dict(robot_id=robot_id, snapshot_id=str(uuid.uuid5(SNAPSHOT_NAMESPACE, key)),
                map_session_id=basis['map_session_id'], map_revision=basis['map_revision'], map_update_sequence=0,
                generated_at_ns=generated_at_ns,
                localization_quality='VALID' if (map_loaded and localization_valid) else 'UNKNOWN',
                correction_detection_status='UNKNOWN', current_native_map_frame=basis['native_frame'])


# ---- ROS boundary ---------------------------------------------------------------------------

def _quaternion(yaw):
    return 0., 0., math.sin(yaw / 2), math.cos(yaw / 2)


def _fill_pose(stamped, frame, stamp_ns, pose):
    stamped.header.frame_id = frame
    stamped.header.stamp.sec, stamped.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    x, y, yaw, covariance = pose
    p, q = stamped.pose.pose.position, stamped.pose.pose.orientation
    p.x, p.y, p.z = float(x), float(y), 0.
    q.x, q.y, q.z, q.w = _quaternion(yaw)
    stamped.pose.covariance = [float(v) for v in covariance]


class LivePosePublisher:
    """Publishes this robot's RobotPose on each AMCL update and MapState when it changes."""

    def __init__(self, runtime, *, pose_valid_ns):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rescue_interfaces.msg import MapState, RobotPose
        if type(pose_valid_ns) is not int or pose_valid_ns <= 0:
            raise ValueError('task.live.pose_valid_ns must be a positive integer')
        self.runtime, self.pose_valid_ns = runtime, pose_valid_ns
        self.robot = runtime.adapter.robot_id
        self.RobotPose, self.MapState = RobotPose, MapState

        def qos(spec):
            reliability, durability, depth = spec
            return QoSProfile(depth=depth, reliability=getattr(ReliabilityPolicy, reliability),
                              durability=getattr(DurabilityPolicy, durability))
        base = f'/rescue/robots/{self.robot}/'
        self.pose_pub = runtime.node.create_publisher(RobotPose, base + 'pose', qos(POSE_QOS))
        self.map_pub = runtime.node.create_publisher(MapState, base + 'map_state', qos(MAP_STATE_QOS))
        self.last_map_state = None

    def _basis(self):
        c = self.runtime.saved_maps.settings[self.robot]
        return dict(mission_frame=self.runtime.inputs.mission_frame, native_frame=c['map_frame'],
                    map_session_id=c['map_session_id'], map_revision=c['map_revision'],
                    transform_revision=c['transform_revision'])

    def _clock(self):
        from rescue_robot.task_policy import validate_clock_basis
        r = self.runtime
        try:
            basis = r.inputs.clock_basis()
            error = validate_clock_basis(basis, policy=r.inputs.lease_policy, now_ns=r.adapter.wall_clock_ns())
            return dict(clock_state='SYNCED', error_bound_ns=int(error))
        except Exception:  # an unmeasured clock is UNKNOWN, never SYNCED
            return dict(clock_state='UNKNOWN', error_bound_ns=None)

    def _transform(self):
        r = self.runtime
        try:
            return r.inputs.current_alignment(self.robot, now_ns=r.adapter.wall_clock_ns()).transform
        except Exception:
            return None

    def on_pose(self, message, observed_at_ns, valid):
        """Called by SavedMapSources.receive_pose (adapter lock held) after its checks."""
        from rescue_robot.live_sources import planar
        r = self.runtime
        x, y, yaw = planar(message.pose.pose)
        record = pose_record(robot_id=self.robot, boot_id=r.adapter.executor.robot_boot_id,
            observed_at_ns=observed_at_ns, pose_valid_ns=self.pose_valid_ns,
            native=(x, y, yaw, [float(v) for v in message.pose.covariance]), amcl_valid=valid,
            map_loaded=self.robot in r.saved_maps.maps, transform=self._transform(), basis=self._basis(),
            clock=self._clock(), clock_domain_id=r.inputs.lease_policy.clock_domain_id)
        msg = self.RobotPose()
        r.server.stamp_envelope(msg.envelope)
        msg.robot_id, msg.source_boot_id = record['robot_id'], record['source_boot_id']
        for name, value in record['time'].items():
            setattr(msg.time, name, value)
        _fill_pose(msg.native_pose, record['native_frame'], observed_at_ns, record['native_pose'])
        if record['mission_pose'] is not None:
            _fill_pose(msg.mission_pose, record['mission_frame'], observed_at_ns, record['mission_pose'])
        for name, value in record['frame_ref'].items():
            setattr(msg.frame_ref, name, value)
        msg.mission_pose_status = record['mission_pose_status']
        self.pose_pub.publish(msg)
        self.publish_map_state(localization_valid=valid)

    def publish_map_state(self, *, localization_valid):
        r = self.runtime
        record = map_state_record(robot_id=self.robot,
            occupancy_sha256=r.saved_maps.settings[self.robot]['occupancy_sha256'], basis=self._basis(),
            map_loaded=self.robot in r.saved_maps.maps, localization_valid=localization_valid,
            generated_at_ns=r.adapter.wall_clock_ns())
        meaning = {k: v for k, v in record.items() if k != 'generated_at_ns'}
        if meaning == self.last_map_state:
            return
        msg = self.MapState()
        r.server.stamp_envelope(msg.envelope)
        for name, value in record.items():
            setattr(msg, name, value)
        self.map_pub.publish(msg)
        self.last_map_state = meaning
        r.node.get_logger().info(f'MAP_STATE {self.robot} snapshot={record["snapshot_id"]} '
                                 f'session={record["map_session_id"]} rev={record["map_revision"]} '
                                 f'quality={record["localization_quality"]}')
