"""TB-5 passage stop from Nav2 collision_monitor state (OPS-115/126/128; scope Q-25 46 cm).

collision_monitor publishes <ns>/collision_monitor_state (nav2_msgs/CollisionMonitorState) on
each change of its action. The passage polygon (field run file task.live.passage.polygon_name)
stops TB-5 when depth points are in the front height band up to 0.46 m. This module only
reads that state; it never sets RobotStatus.stop_reason (the coordinator ends the lease on a
stop_reason) and never moves or stops the robot itself.
"""
from rescue_robot.executor import ContractError

STOP = 1   # nav2_msgs/CollisionMonitorState.STOP (attach maps the message constant onto this)


def passage_settings(value):
    if not isinstance(value, dict) or set(value) != {'polygon_name', 'result_window_ns'}:
        raise ContractError('passage settings need polygon_name and result_window_ns')
    name, window = value['polygon_name'], value['result_window_ns']
    if not isinstance(name, str) or not name or type(window) is not int or window <= 0:
        raise ContractError('passage polygon_name non-empty and result_window_ns positive integer required')
    return dict(value)


class PassageMonitor:
    def __init__(self, settings):
        self.polygon, self.window_ns = settings['polygon_name'], settings['result_window_ns']
        self.holding = False
        self.released_at_ns = None   # when the last passage stop lifted

    def on_state(self, action_type, polygon_name, observed_ns):
        holding = action_type == STOP and polygon_name == self.polygon
        if self.holding and not holding:
            self.released_at_ns = observed_ns
        self.holding = holding

    def blocked(self):
        return self.holding

    def blocked_within(self, *, now_ns):
        """Held now, or released no longer than the result window ago."""
        return self.holding or (self.released_at_ns is not None and now_ns - self.released_at_ns <= self.window_ns)


def attach(runtime, topic):
    """Subscribe <ns>/collision_monitor_state (arrival time = observation time; the message has no stamp)."""
    from nav2_msgs.msg import CollisionMonitorState
    r = runtime

    def receive(message):
        with r.adapter.lock:
            before = r.passage.blocked()
            # the message's own STOP constant, not a copied number
            action = STOP if message.action_type == CollisionMonitorState.STOP else -1
            r.passage.on_state(action, message.polygon_name, r.adapter.wall_clock_ns())
            if r.passage.blocked() != before:
                r.node.get_logger().warning(('PASSAGE_BLOCKED' if r.passage.blocked() else 'PASSAGE_RELEASED')
                                            + f' polygon={message.polygon_name!r} action={message.action_type}')
    handle = r.node.create_subscription(CollisionMonitorState, topic, receive, 10)
    r.input_handles.append(('destroy_subscription', handle))
