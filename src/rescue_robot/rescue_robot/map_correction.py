"""SLAM correction watch at TB-6 (방식 B live shared map, 2026-10-15): map -> odom jumps, logged and published.

slam_toolbox moves ``map -> odom`` a little with every scan it matches, and suddenly when a loop closure re-optimizes
the pose graph. A jump moves every map position near TB-6 at once: a doll position or a TB-5 goal computed before
it can be off by that much (the doll detector sends each doll once, user policy 2026-10-12, so nothing re-observes
it by itself). This node reads TB-6's TF (``map -> odom`` and ``odom -> base_link`` on ``<ns>/tf``), measures how far
TB-6's own map position moved between two ``map -> odom`` samples (``shift_m``, odometry held fixed) and the yaw
change, and publishes every jump of at least ``correction_m`` or ``correction_rad`` on ``/rescue/map/correction``
(std_msgs/String JSON). The coordinator logs it with the targets observed before it; nothing is corrected or made
up. The thresholds are the field run file's ``coordinator.map_correction_*`` ([제안], not measured).
"""
import json
import math


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _apply(t, p):
    c, s = math.cos(t[2]), math.sin(t[2])
    return t[0] + c * p[0] - s * p[1], t[1] + s * p[0] + c * p[1]


def _angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class CorrectionWatch:
    """Planar map -> odom samples -> jump events; no ROS."""

    def __init__(self, *, correction_m, correction_rad):
        for name, value in (('correction_m', correction_m), ('correction_rad', correction_rad)):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'map correction {name} must be a positive number')
        self.correction_m, self.correction_rad = float(correction_m), float(correction_rad)
        self.map_odom = None   # (x, y, yaw)
        self.odom_base = None  # (x, y, yaw)
        self.count = 0

    def on_odom_base(self, xy_yaw):
        self.odom_base = tuple(float(v) for v in xy_yaw)

    def on_map_odom(self, xy_yaw, observed_at_ns):
        """A new map -> odom sample: an event dict when it jumped past a threshold, else None."""
        new = tuple(float(v) for v in xy_yaw)
        if not all(math.isfinite(v) for v in new):
            raise ValueError('map -> odom not finite')
        old, self.map_odom = self.map_odom, new
        if old is None:
            return None
        p = self.odom_base[:2] if self.odom_base is not None else (0.0, 0.0)
        before, after = _apply(old, p), _apply(new, p)
        shift, turn = math.dist(before, after), _angle(new[2] - old[2])
        if shift < self.correction_m and abs(turn) < self.correction_rad:
            return None
        self.count += 1
        return dict(observed_at_ns=int(observed_at_ns), shift_m=shift, yaw_rad=turn, count=self.count,
                    before=list(before), after=list(after))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import ParameterType as T
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rescue_robot.ros_node import declare_required, git_sha

    rclpy.init(args=args)
    node = rclpy.create_node('rescue_map_correction')
    log = node.get_logger()
    S, D = T.PARAMETER_STRING, T.PARAMETER_DOUBLE
    values, missing = declare_required(node, {'robot_id': S, 'tf_topic': S, 'map_frame': S, 'odom_frame': S,
                                              'base_frame': S, 'output_topic': S, 'correction_m': D,
                                              'correction_rad': D})
    if missing:
        log.error('map_correction missing parameters: ' + ', '.join(missing))
        raise SystemExit(2)
    try:
        core = CorrectionWatch(correction_m=values['correction_m'], correction_rad=values['correction_rad'])
    except ValueError as exc:
        log.error(str(exc))
        raise SystemExit(2)
    out = node.create_publisher(String, values['output_topic'], 10)

    def planar(t):
        r = t.rotation
        return t.translation.x, t.translation.y, _yaw((r.x, r.y, r.z, r.w))

    def on_tf(message):
        for edge in message.transforms:
            parent, child = edge.header.frame_id.lstrip('/'), edge.child_frame_id.lstrip('/')
            if (parent, child) == (values['odom_frame'], values['base_frame']):
                core.on_odom_base(planar(edge.transform))
            elif (parent, child) == (values['map_frame'], values['odom_frame']):
                stamp = edge.header.stamp.sec * 1_000_000_000 + edge.header.stamp.nanosec
                try:
                    event = core.on_map_odom(planar(edge.transform), stamp)
                except ValueError as exc:
                    log.warning(f'MAP_CORRECTION sample refused: {exc}')
                    continue
                if event is not None:
                    event.update(robot_id=values['robot_id'], map_frame=values['map_frame'])
                    out.publish(String(data=json.dumps(event, sort_keys=True)))
                    log.warning(f'MAP_CORRECTION {json.dumps(event, sort_keys=True)}')

    node.create_subscription(TFMessage, values['tf_topic'], on_tf, 100)
    log.info(f'git_sha={git_sha()} parameters={values}; publishes map -> odom jumps only')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
