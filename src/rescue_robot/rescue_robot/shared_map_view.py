"""Draw both robots in one RViz map using their VALID mission poses, without merging native TF.

Presentation choices (arrow size/color, 5 Hz drawing) belong to this viewer,
2026-10-15. They do not change localization, navigation or robot settings.
"""
import math

DRAW_PERIOD_S = 0.2
ARROW_SCALE = (0.35, 0.08, 0.08)
COLORS = ((0.15, 0.6, 1.0), (1.0, 0.5, 0.1))


def pose_for_view(message, *, robot_id, frame, map_session_id, map_revision, transform_revision, now_ns):
    """Return an already transformed mission pose, or None for missing/invalid/expired/wrong-basis data."""
    try:
        f, t = message.frame_ref, message.time
        if (message.robot_id != robot_id or message.mission_pose_status != 'VALID' or
                t.clock_state != 'SYNCED' or not 0 < t.observed_at_ns <= now_ns < t.valid_until_ns or
                f.alignment_status != 'VALID' or (f.mission_frame, f.map_session_id, f.map_revision,
                f.transform_revision) != (frame, map_session_id, map_revision, transform_revision) or
                message.mission_pose.header.frame_id != frame):
            return None
        p = message.mission_pose.pose.pose
        xyzq = (p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if not all(math.isfinite(v) for v in xyzq) or not math.isclose(sum(v*v for v in xyzq[3:]), 1., abs_tol=1e-6):
            return None
        return xyzq
    except (AttributeError, TypeError, ValueError):
        return None


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterType as T
    from rescue_interfaces.msg import RobotPose
    from visualization_msgs.msg import Marker, MarkerArray
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_shared_map_view')
    fields = dict(robot_ids=T.PARAMETER_STRING_ARRAY, frame=T.PARAMETER_STRING,
                  map_session_id=T.PARAMETER_STRING, map_revision=T.PARAMETER_INTEGER,
                  transform_revision=T.PARAMETER_INTEGER, marker_topic=T.PARAMETER_STRING)
    values, missing = declare_required(node, fields)
    ids = values.get('robot_ids', [])
    if missing or not 1 <= len(ids) <= len(COLORS) or len(set(ids)) != len(ids):
        node.get_logger().error(f'shared map viewer needs a basis and one or two unique robot IDs; missing={missing}')
        node.destroy_node(); rclpy.try_shutdown()
        raise SystemExit(2)
    latest = {}
    publisher = node.create_publisher(MarkerArray, values['marker_topic'], 1)
    subscriptions = []
    for robot in values['robot_ids']:
        subscriptions.append(node.create_subscription(RobotPose, f'/rescue/robots/{robot}/pose',
            lambda m, robot=robot: latest.__setitem__(robot, m), 1))

    def draw():
        now = node.get_clock().now().nanoseconds
        out = MarkerArray()
        for i, robot in enumerate(values['robot_ids']):
            m = latest.get(robot)
            pose = pose_for_view(m, robot_id=robot, frame=values['frame'], map_session_id=values['map_session_id'],
                map_revision=values['map_revision'], transform_revision=values['transform_revision'], now_ns=now)
            for marker_id in (0, 1):
                marker = Marker()
                marker.header.frame_id = values['frame']
                marker.header.stamp = node.get_clock().now().to_msg()
                marker.ns, marker.id = robot, marker_id
                marker.action = Marker.DELETE if pose is None else Marker.ADD
                if pose is not None:
                    x, y, z, qx, qy, qz, qw = pose
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
                    q = marker.pose.orientation
                    q.x, q.y, q.z, q.w = qx, qy, qz, qw
                    marker.color.r, marker.color.g, marker.color.b = COLORS[i]
                    marker.color.a = 1.
                    marker.lifetime.sec, marker.lifetime.nanosec = divmod(m.time.valid_until_ns - now, 1_000_000_000)
                    if marker_id == 0:
                        marker.type = Marker.ARROW
                        marker.scale.x, marker.scale.y, marker.scale.z = ARROW_SCALE
                    else:
                        marker.type, marker.text = Marker.TEXT_VIEW_FACING, robot
                        marker.pose.position.z += ARROW_SCALE[0]
                        marker.scale.z = ARROW_SCALE[1]
                out.markers.append(marker)
        publisher.publish(out)

    node.get_logger().info(f'git_sha={git_sha()} shared map view parameters={values}')
    node.create_timer(DRAW_PERIOD_S, draw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
