"""Wait for the TB-6 frozen pose at launch time (frozen_localization.launch.py, robot_a only).

map_freeze publishes /rescue/map/frozen_pose/<robot> (PoseWithCovarianceStamped,
TRANSIENT_LOCAL) when it freezes the TB-6 map: TB-6 starts AMCL there. A private rclpy
context reads one message; no frozen pose within the wait = the launch fails (no AMCL start
is made up).
"""


def wait_frozen_pose(topic, *, timeout_s, map_frame):
    """(x, y, yaw) of the frozen pose on ``map_frame``; RuntimeError if none arrives in time."""
    import time

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_robot.executor import ContractError
    from rescue_robot.live_sources import planar

    context = Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('frozen_pose_wait', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        got = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PoseWithCovarianceStamped, topic, got.append, qos)
        deadline = time.monotonic() + timeout_s
        while not got and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
        if not got:
            raise RuntimeError(f'no frozen pose on {topic} within {timeout_s} s: freeze the map first '
                               '(/rescue/map/freeze), no AMCL start is made up')
        if got[0].header.frame_id != map_frame:
            raise RuntimeError(f'frozen pose frame {got[0].header.frame_id!r} is not {map_frame!r}')
        try:
            return planar(got[0].pose.pose)
        except ContractError as exc:
            raise RuntimeError(f'frozen pose is not planar: {exc}') from None
    finally:
        rclpy.try_shutdown(context=context)
