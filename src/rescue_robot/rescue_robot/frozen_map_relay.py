"""Republish this robot's frozen TB-6 map on its map topic (map lifecycle, OPS-113/124/154).

AMCL and the Nav2 static layer read <ns>/map; task_input loads the map only when <ns>/map is
exactly its frozen grid (live_sources). The map is frozen twice per run (R0 + 1, R0 + 2); a
frozen grid has a revision only with its pair on /rescue/map/frozen_revision (same hash). This
node keeps every frozen grid of the run with its revision and copies to <ns>/map, unchanged
(RELIABLE, TRANSIENT_LOCAL), the one whose revision this robot's executor publishes in its
MapState (/rescue/robots/<id>/map_state). So TB-6 (R0 while searching) gets nothing until its
executor takes R0 + 2, and TB-5 stays on R0 + 1 until its executor takes R0 + 2 (stopped, no
task). It refuses to republish while another node publishes <ns>/map (TB-6 SLAM still
running): two writers would make AMCL and task_input disagree. Then it logs MAP_RELAY_BLOCKED
and waits. Another grid for a revision it already has blocks copying (new run required).
"""
from rescue_contracts.occupancy import parse_frozen_revision


class RelayCore:
    """Which frozen grid this robot's map topic should carry; no ROS."""

    def __init__(self):
        self.grids = {}        # sha256 -> message
        self.revisions = {}    # revision -> sha256 (paired or not yet)
        self.revision = None   # this robot's MapState revision
        self.blocked = ''

    def on_grid(self, sha256, message):
        self.grids[sha256] = message

    def on_revision(self, text):
        """ValueError for a pair that is not rescue-map-frozen/1."""
        info = parse_frozen_revision(text)
        old = self.revisions.get(info['map_revision'])
        if old is not None and old != info['occupancy_sha256']:
            self.blocked = f'frozen map R{info["map_revision"]} changed; new run required'
        self.revisions.setdefault(info['map_revision'], info['occupancy_sha256'])

    def on_map_state(self, revision):
        self.revision = revision

    def wanted(self):
        if self.blocked or self.revision not in self.revisions:
            return None
        return self.grids.get(self.revisions[self.revision])


def main(args=None):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rcl_interfaces.msg import ParameterType as T
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import MapState
    from std_msgs.msg import String
    from rescue_robot.live_sources import Grid
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_frozen_map_relay')
    log = node.get_logger()
    S = T.PARAMETER_STRING
    values, missing = declare_required(node, {'frozen_topic': S, 'frozen_revision_topic': S, 'map_state_topic': S,
                                              'map_topic': S})
    if missing:
        log.error('frozen_map_relay missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, values['map_topic'], latched)
    core, state = RelayCore(), {'sent': None, 'said': None}

    def others():
        return node.count_publishers(values['map_topic']) - 1   # minus this relay

    def say(level, text):
        if state['said'] != text:   # each wait/block reason once
            state['said'] = text
            if level == 'error':   # one severity per call site (rclpy raises otherwise)
                log.error(text)
            else:
                log.warning(text)

    def forward():
        message = core.wanted()
        if message is None:
            if core.blocked:
                say('error', f'MAP_RELAY_BLOCKED {core.blocked}')
            elif core.revision in core.revisions:
                say('warning', f'MAP_RELAY_WAITING R{core.revision} grid not received')
            return
        if message is state['sent']:
            return
        if others() > 0:
            say('error', f'MAP_RELAY_BLOCKED another node publishes {values["map_topic"]} (SLAM still running?); '
                         'stop it, the frozen map is copied then')
            return
        publisher.publish(message)
        state['sent'] = message
        log.info(f'MAP_RELAYED {values["frozen_topic"]} R{core.revision} -> {values["map_topic"]} '
                 f'sha256={Grid(message).sha256}')

    def receive_grid(message):
        try:
            core.on_grid(Grid(message).sha256, message)
        except Exception as exc:   # an invalid grid is never copied
            log.error(f'frozen map refused: {exc}')
            return
        forward()

    def receive_revision(message):
        try:
            core.on_revision(message.data)
        except ValueError as exc:
            log.error(f'frozen revision refused: {exc}')
            return
        forward()

    def receive_map_state(message):
        core.on_map_state(message.map_revision)
        forward()

    log.info(f'git_sha={git_sha()} parameters={values}')
    node.create_subscription(OccupancyGrid, values['frozen_topic'], receive_grid, latched)
    node.create_subscription(String, values['frozen_revision_topic'], receive_revision, latched)
    node.create_subscription(MapState, values['map_state_topic'], receive_map_state, latched)
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
