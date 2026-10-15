"""The last slam_toolbox pose, repeated unchanged (방식 B live shared map; rehearsal finding 2026-10-15).

slam_toolbox publishes ``<ns>/pose`` (volatile) only for the scans it processes; a TB-6 standing still gets one at the
first scan and no other (minimum_travel_distance). A node started after that first scan (the executor, which takes
TB-6's pose covariance from this topic, OPS-177/179) never gets a covariance, so TB-6's MapState stays UNKNOWN and the
coordinator never sends its first frontier goal. This node, started before SLAM (live_map.launch.py), keeps the last
pose and republishes the SAME message (its original header stamp) every ``period_s`` on ``output_topic``: the
executor still judges the covariance by that stamp (task.saved_map.slam_covariance_max_age_ns), so an old pose stays
old; nothing is re-stamped or made up.
"""


class Latest:
    """The last pose message, if any; no ROS."""

    def __init__(self):
        self.message = None

    def on_pose(self, message):
        old = self.message
        if old is None or (message.header.stamp.sec, message.header.stamp.nanosec) >= (
                old.header.stamp.sec, old.header.stamp.nanosec):
            self.message = message
            return True
        return False


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_slam_pose_latest')
    log = node.get_logger()
    values, missing = declare_required(node, {'input_topic': T.PARAMETER_STRING, 'output_topic': T.PARAMETER_STRING,
                                              'period_s': T.PARAMETER_DOUBLE})
    if missing or not values['period_s'] > 0:
        log.error(f'slam_pose_latest parameters missing or invalid: {missing or values}')
        raise SystemExit(2)
    core = Latest()
    out = node.create_publisher(PoseWithCovarianceStamped, values['output_topic'], 10)

    def on_pose(message):
        if core.on_pose(message):
            out.publish(message)

    def repeat():
        if core.message is not None:
            out.publish(core.message)

    node.create_subscription(PoseWithCovarianceStamped, values['input_topic'], on_pose, qos_profile_sensor_data)
    node.create_timer(values['period_s'], repeat)
    log.info(f'git_sha={git_sha()} parameters={values}; repeats the last SLAM pose with its own stamp')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
