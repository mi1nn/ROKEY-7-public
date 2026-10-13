"""ROS 2 boundary for CommandWatchdog: one robot's last velocity hop.

Launch remaps the mover (Nav2's final velocity output) to `cmd_vel_in_topic`;
only this node publishes `cmd_vel_out_topic` (the robot's cmd_vel). It listens to
the command adapter's executor heartbeat and publishes its own heartbeat, which
the adapter turns into the `watchdog` readiness flag. `~/reset` (std_srvs/Trigger)
is the only way out of STOP_LATCHED. ROS packages are imported inside functions.
"""
import sys
import time

from rescue_contracts import RescueError
from rescue_robot.ros_node import declare_required, git_sha
from rescue_robot.watchdog import CommandWatchdog, WatchdogPolicy

POLICY_FIELDS = ('heartbeat_max_age_s', 'command_max_age_s', 'max_linear_mps', 'max_angular_rps')


def bind_watchdog(node, dog, *, cmd_vel_in_topic, cmd_vel_out_topic, stamped, executor_heartbeat_topic,
                  watchdog_heartbeat_topic, step_period, clock=time.monotonic):
    """Subscriptions, publishers, timer and reset service for one CommandWatchdog."""
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger

    log = node.get_logger()
    kind = TwistStamped if stamped else Twist
    out = node.create_publisher(kind, cmd_vel_out_topic, 10)
    alive = node.create_publisher(Header, watchdog_heartbeat_topic, 10)
    seen = [len(dog.events)]

    def report():
        for state, reason, at in dog.events[seen[0]:]:
            line = f'watchdog {state} reason={reason or "-"} t={at:.3f}'
            if state == 'STOP_LATCHED':   # rclpy: one severity per call site
                log.warn(line)
            else:
                log.info(line)
        seen[0] = len(dog.events)

    def publish(decision, frame_id=''):
        msg = kind()
        twist = msg.twist if stamped else msg
        if decision[0] == 'FORWARD':
            twist.linear.x, twist.linear.y, twist.linear.z = decision[1]
            twist.angular.x, twist.angular.y, twist.angular.z = decision[2]
        if stamped:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
        out.publish(msg)

    def on_command(msg):
        twist = msg.twist if stamped else msg
        decision = dog.command((twist.linear.x, twist.linear.y, twist.linear.z),
                               (twist.angular.x, twist.angular.y, twist.angular.z), clock())
        publish(decision, msg.header.frame_id if stamped else '')
        report()

    def on_heartbeat(_msg):
        dog.heartbeat(clock())
        report()

    def step():
        for decision in dog.step(clock()):
            publish(decision)
        if dog.alive(clock()):
            alive.publish(Header(stamp=node.get_clock().now().to_msg()))
        report()

    def on_reset(_request, response):
        response.success = dog.reset(clock())
        response.message = f'{dog.state} {dog.reason}'.strip()
        log.warn(f'reset requested: success={response.success} state={response.message}')
        report()
        return response

    handles = (node.create_subscription(kind, cmd_vel_in_topic, on_command, 10),
               node.create_subscription(Header, executor_heartbeat_topic, on_heartbeat, 10),
               node.create_timer(step_period, step),
               node.create_service(Trigger, '~/reset', on_reset))
    return handles


def zero_commands(kind, *, stamped, stamp, count):
    """``count`` zero velocity commands (sent when the watchdog is stopped, OPS-183)."""
    out = []
    for _ in range(count):
        msg = kind()
        twist = msg.twist if stamped else msg
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        if stamped:
            msg.header.stamp = stamp
        out.append(msg)
    return out


EXIT_ZERO_COUNT = 3   # zero commands on Ctrl+C / launch shutdown, before the watchdog goes away


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.msg import ParameterType

    # Ctrl+C reaches us as KeyboardInterrupt with the context still up, so zero can still be sent
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('rescue_command_watchdog')
    log = node.get_logger()
    string, double = ParameterType.PARAMETER_STRING, ParameterType.PARAMETER_DOUBLE
    names = {'namespace': string, 'cmd_vel_in_topic': string, 'cmd_vel_out_topic': string,
             'cmd_vel_stamped': ParameterType.PARAMETER_BOOL, 'executor_heartbeat_topic': string,
             'watchdog_heartbeat_topic': string, 'step_period': double, 'watchdog.version': string}
    names.update({'watchdog.' + field: double for field in POLICY_FIELDS})
    value, missing = declare_required(node, names)  # no defaults: launch/profile gives all
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    if node.get_namespace() != value['namespace']:
        log.fatal(f'namespace mismatch: node {node.get_namespace()} != profile {value["namespace"]}')
        raise SystemExit(2)
    if value['cmd_vel_in_topic'] == value['cmd_vel_out_topic']:
        log.fatal('cmd_vel_in_topic and cmd_vel_out_topic must differ (the watchdog is the only writer)')
        raise SystemExit(2)
    try:
        policy = WatchdogPolicy(value['watchdog.version'], *(value['watchdog.' + field] for field in POLICY_FIELDS))
    except RescueError as exc:
        log.fatal(f'watchdog policy rejected: {exc}')
        raise SystemExit(2)
    bind_watchdog(node, CommandWatchdog(policy), cmd_vel_in_topic=value['cmd_vel_in_topic'],
                  cmd_vel_out_topic=value['cmd_vel_out_topic'], stamped=value['cmd_vel_stamped'],
                  executor_heartbeat_topic=value['executor_heartbeat_topic'],
                  watchdog_heartbeat_topic=value['watchdog_heartbeat_topic'], step_period=value['step_period'])
    log.info(f'git_sha={git_sha()} parameters={value}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # launch stops nodes with SIGINT
        pass
    finally:
        try:   # stopped (Ctrl+C or launch shutdown): leave the robot with zero, not with the last command
            from geometry_msgs.msg import Twist, TwistStamped
            kind = TwistStamped if value['cmd_vel_stamped'] else Twist
            zero = node.create_publisher(kind, value['cmd_vel_out_topic'], 10)
            for msg in zero_commands(kind, stamped=value['cmd_vel_stamped'], stamp=node.get_clock().now().to_msg(),
                                     count=EXIT_ZERO_COUNT):
                zero.publish(msg)
            log.info(f'watchdog stopped: {EXIT_ZERO_COUNT} zero commands on {value["cmd_vel_out_topic"]}')
        except Exception as exc:   # the context may already be gone (SIGTERM): say so, do not hide it
            print(f'watchdog stopped: zero commands not sent: {exc}', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
