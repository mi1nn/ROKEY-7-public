"""TB-5's copy of TB-6's live map without TB-5's own body (방식 B; rehearsal finding 2026-10-15).

TB-6's SLAM sees TB-5 standing at its start and keeps it in /robot6/map as an obstacle. Fed to TB-5 as is, TB-5's start
cell is blocked: no grid path for the coordinator, Nav2's planner starts inside lethal cost, and TB-5's way back to its
start spot (its exit when there is no QR exit) ends in it. This relay republishes every TB-6 grid for TB-5 with a disc
of rescue_contracts.occupancy.SELF_CLEAR_RADIUS_M around TB-5's measured start (the seed placed on the map) and around
its latest AMCL pose (at most ``pose_max_age_s`` old) set free. Frame, origin, size and header stamp are unchanged, so
the coordinates are TB-6's map exactly; nothing else is edited. TB-5's AMCL, start_align, executor and Nav2 read it.
"""
import math


def discs(start_xy, pose, now_ns, pose_max_age_ns):
    """Disc centres: the start, plus the pose (x, y, stamp_ns) when fresh."""
    out = [tuple(start_xy)]
    if pose is not None and 0 <= now_ns - pose[2] <= pose_max_age_ns:
        out.append((pose[0], pose[1]))
    return out


def main(args=None):
    import time

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rescue_contracts.occupancy import SELF_CLEAR_RADIUS_M, clear_discs, planar_origin
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_self_clear_map')
    log = node.get_logger()
    values, missing = declare_required(node, {'input_topic': T.PARAMETER_STRING, 'output_topic': T.PARAMETER_STRING,
                                              'pose_topic': T.PARAMETER_STRING, 'start_xy': T.PARAMETER_DOUBLE_ARRAY,
                                              'pose_max_age_s': T.PARAMETER_DOUBLE})
    start = list(values['start_xy'] or [])
    if missing or len(start) != 2 or not all(math.isfinite(v) for v in start) or not values['pose_max_age_s'] > 0:
        log.error(f'self_clear_map parameters missing or invalid: {missing or values}')
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    out = node.create_publisher(OccupancyGrid, values['output_topic'], latched)
    state = {'pose': None, 'count': 0}

    def on_pose(msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        state['pose'] = (msg.pose.pose.position.x, msg.pose.pose.position.y, stamp)

    def on_map(msg):
        info = msg.info
        p, q = info.origin.position, info.origin.orientation
        try:
            data = clear_discs(frame=msg.header.frame_id, width=int(info.width), height=int(info.height),
                               resolution=float(info.resolution),
                               origin=planar_origin(p.x, p.y, p.z, q.x, q.y, q.z, q.w), data=list(msg.data),
                               discs=discs(start, state['pose'], time.time_ns(),
                                           round(values['pose_max_age_s'] * 1e9)))
        except ValueError as exc:   # an invalid grid is not passed on
            log.error(f'SELF_CLEAR_MAP grid refused: {exc}')
            return
        msg.data = data
        out.publish(msg)
        state['count'] += 1
        if state['count'] == 1:
            log.info(f'SELF_CLEAR_MAP first grid {info.width}x{info.height} -> {values["output_topic"]}')

    node.create_subscription(OccupancyGrid, values['input_topic'], on_map, latched)
    node.create_subscription(PoseWithCovarianceStamped, values['pose_topic'], on_pose, qos_profile_sensor_data)
    log.info(f'git_sha={git_sha()} parameters={values} radius={SELF_CLEAR_RADIUS_M}; clears only TB-5 itself')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
