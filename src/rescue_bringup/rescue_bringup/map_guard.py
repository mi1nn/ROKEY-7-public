"""Refuse to start SLAM when <ns>/map already has a publisher (OPS-184).

Field report 2026-10-13: after docking and a restart the SLAM map was the old one. An earlier
slam_toolbox (or any other map writer) still running would keep publishing its map and the new
run would silently continue it [estimate]. tb6_slam.launch.py and mapping_navigation.launch.py
look first and refuse, naming the publisher; stop it (scripts/field/stop_robot.sh lists PIDs).
"""


class MapGuardError(RuntimeError):
    pass


def refuse_existing_map_publishers(topic, infos):
    """``infos``: rclpy TopicEndpointInfo of ``topic``'s publishers (node_namespace, node_name)."""
    names = sorted({(i.node_namespace.rstrip('/') + '/' + i.node_name) for i in infos})
    if names:
        raise MapGuardError(f'{topic} already has a publisher: {", ".join(names)} (an earlier SLAM still running?). '
                            'Stop it first (scripts/field/stop_robot.sh <ns> lists PIDs); a new run must not '
                            'continue an old map.')


def map_publishers(topic, *, wait_s):
    """Publishers of ``topic`` seen within ``wait_s`` (a short-lived node subscribes so discovery matches)."""
    import time

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('rescue_map_guard', context=context)
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, topic, lambda _m: None, qos)
        deadline = time.monotonic() + wait_s
        infos = []
        # wait for a publisher, then (still within wait_s) for its node name to be discovered too
        while time.monotonic() < deadline and (not infos or any(i.node_name.startswith('_NODE_') for i in infos)):
            time.sleep(0.1)
            infos = node.get_publishers_info_by_topic(topic)
        return infos
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def check_no_map_publisher(topic, *, wait_s):
    refuse_existing_map_publishers(topic, map_publishers(topic, wait_s=wait_s))
