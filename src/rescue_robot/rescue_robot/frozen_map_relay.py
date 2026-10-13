"""Republish the frozen TB-6 map on this robot's map topic (map lifecycle, OPS-113/124).

AMCL and the Nav2 static layer read <ns>/map; task_input loads the map only when <ns>/map is
exactly the frozen grid (live_sources). This node copies /rescue/map/frozen to <ns>/map
unchanged (RELIABLE, TRANSIENT_LOCAL). It refuses to republish while another node publishes
<ns>/map (TB-6 SLAM still running): two writers would make AMCL and task_input disagree.
Then it logs MAP_RELAY_BLOCKED and waits; stop SLAM and the next frozen map is copied.
"""


def main(args=None):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_robot.live_sources import Grid
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_frozen_map_relay')
    log = node.get_logger()
    values, missing = declare_required(node, {'frozen_topic': T.PARAMETER_STRING, 'map_topic': T.PARAMETER_STRING})
    if missing:
        log.error('frozen_map_relay missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, values['map_topic'], latched)
    state = {'pending': None, 'sent': None}

    def others():
        return node.count_publishers(values['map_topic']) - 1   # minus this relay

    def forward():
        message = state['pending']
        if message is None or message is state['sent']:
            return
        if others() > 0:
            log.error(f'MAP_RELAY_BLOCKED another node publishes {values["map_topic"]} (SLAM still running?); '
                      'stop it, the frozen map is copied then')
            return
        publisher.publish(message)
        state['sent'] = message
        log.info(f'MAP_RELAYED {values["frozen_topic"]} -> {values["map_topic"]} sha256={Grid(message).sha256}')

    def receive(message):
        state['pending'] = message
        forward()

    log.info(f'git_sha={git_sha()} parameters={values}')
    node.create_subscription(OccupancyGrid, values['frozen_topic'], receive, latched)
    node.create_timer(1.0, forward)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
