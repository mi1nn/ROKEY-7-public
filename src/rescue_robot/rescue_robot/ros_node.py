"""ROS 2 boundary for CommandAdapter: NavigateToPose client, odometry, dock status.

ROS packages are imported inside functions so `rescue_robot` stays importable
and testable without ROS. This node has no task ingress yet (ExecuteTask lives
in the unimplemented rescue_interfaces) and its guard rejects every dispatch
until the mapping revision guard is connected, so it cannot start a motion on
its own. It observes odometry, dock status and Nav2 availability.
"""
import math
import subprocess
from pathlib import Path
from uuid import uuid4

from rescue_contracts import RescueError
from rescue_robot.executor import Executor, Policy
from rescue_robot.nav2_adapter import CommandAdapter
from rescue_robot.readiness import ReadinessMonitor, ReadinessPolicy

POLICY_FIELDS = ('acceptance_timeout', 'stop_timeout', 'stop_window', 'max_sample_gap',
                 'evidence_max_age', 'linear_stop_max', 'angular_stop_max')


def yaw_to_quaternion(yaw):
    """Planar yaw -> (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def git_sha():
    """Source checkout SHA, or 'UNKNOWN' (installed copies have no .git)."""
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


class Nav2Port:
    """NavigationPort over one robot's NavigateToPose action (namespace-relative name)."""

    def __init__(self, node, action_name):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        self.node = node
        self.goal_type = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.adapter = None
        self.handles = {}
        self.pending_cancel = set()

    def bind(self, adapter):
        self.adapter = adapter

    def server_ready(self):
        return self.client.server_is_ready()

    def send_goal(self, command_id, pose):
        goal = self.goal_type.Goal()
        goal.pose.header.frame_id = pose.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose.x)
        goal.pose.pose.position.y = float(pose.y)
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(float(pose.yaw))
        self.node.get_logger().info(f'SEND {command_id} {pose}')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_response(command_id, done))

    def cancel_goal(self, command_id):
        handle = self.handles.get(command_id)
        self.node.get_logger().info(f'CANCEL {command_id} handle={handle is not None}')
        if handle is None:
            self.pending_cancel.add(command_id)  # issued once the goal handle arrives
            return
        handle.cancel_goal_async().add_done_callback(
            lambda done: self._guard(self.adapter.on_cancel_response, command_id,
                                     bool(done.result().goals_canceling)))

    def _on_response(self, command_id, future):
        handle = future.result()
        if handle.accepted:
            self.handles[command_id] = handle
            handle.get_result_async().add_done_callback(
                lambda done: self._guard(self.adapter.on_terminal, command_id, done.result().status))
        self._guard(self.adapter.on_goal_response, command_id, bool(handle.accepted))
        if handle.accepted and command_id in self.pending_cancel:
            self.pending_cancel.discard(command_id)
            self.cancel_goal(command_id)

    def _guard(self, callback, *args):
        try:
            callback(*args)
        except RescueError as exc:
            # The executor already latched/cancelled where needed; make it visible.
            self.node.get_logger().error(f'{callback.__name__}{args}: {exc}')


def bind_odometry(node, adapter, topic, monitor=None):
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    def on_odom(msg):
        twist = msg.twist.twist
        if monitor is not None:
            monitor.arrived('odom', adapter.clock())
        try:
            adapter.on_odom(stamp_sec=msg.header.stamp.sec, stamp_nanosec=msg.header.stamp.nanosec,
                            linear=(twist.linear.x, twist.linear.y, twist.linear.z),
                            angular=(twist.angular.x, twist.angular.y, twist.angular.z))
        except RescueError as exc:
            node.get_logger().error(f'odom: {exc}')
    return node.create_subscription(Odometry, topic, on_odom, qos_profile_sensor_data)


def bind_dock_status(node, adapter, topic):
    from irobot_create_msgs.msg import DockStatus
    from rclpy.qos import qos_profile_sensor_data
    return node.create_subscription(DockStatus, topic, lambda msg: adapter.on_dock_status(bool(msg.is_docked)),
                                    qos_profile_sensor_data)


def camera_message_type(topic):
    """sensor_msgs type name for a camera topic: image_transport's .../compressed is
    CompressedImage (TB4 OAK-D default, P4 DEC-004), anything else Image."""
    return 'CompressedImage' if topic.rstrip('/').endswith('/compressed') else 'Image'


def bind_readiness(node, monitor, adapter, *, scan_topic, camera_topics, wheel_status_topic,
                   tf_topic, map_frame, odom_frame):
    """Arrival-only subscriptions feeding ReadinessMonitor (no message content judged,
    except that only the map->odom edge counts for slam freshness)."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, LaserScan
    from tf2_msgs.msg import TFMessage

    def arrival(source):
        return lambda _msg: monitor.arrived(source, adapter.clock())

    subs = [node.create_subscription(LaserScan, scan_topic, arrival('scan'), qos_profile_sensor_data)]
    for topic in camera_topics:
        kind = CompressedImage if camera_message_type(topic) == 'CompressedImage' else Image
        subs.append(node.create_subscription(kind, topic, arrival('camera:' + topic), qos_profile_sensor_data))
    if wheel_status_topic:
        from irobot_create_msgs.msg import WheelStatus
        subs.append(node.create_subscription(WheelStatus, wheel_status_topic, arrival('wheel_status'),
                                             qos_profile_sensor_data))

    def on_tf(msg):
        if any(t.header.frame_id == map_frame and t.child_frame_id == odom_frame for t in msg.transforms):
            monitor.arrived('map_to_odom_tf', adapter.clock())
    subs.append(node.create_subscription(TFMessage, tf_topic, on_tf, 100))
    return subs


def readiness_policy(version, ages, camera_topics, wheel_status_topic):
    """driver=odom(+wheel_status), sensors=scan(+cameras), slam=map->odom TF; no watchdog source."""
    max_age = {'odom': ages['odom'], 'scan': ages['scan'], 'map_to_odom_tf': ages['map_to_odom_tf']}
    driver, sensors = ('odom',), ('scan',)
    if wheel_status_topic:
        max_age['wheel_status'] = ages['wheel_status']
        driver += ('wheel_status',)
    for topic in camera_topics:
        max_age['camera:' + topic] = ages['camera']
        sensors += ('camera:' + topic,)
    return ReadinessPolicy(version, max_age, {'driver': driver, 'sensors': sensors,
                                              'slam': ('map_to_odom_tf',)})


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType

    rclpy.init(args=args)
    node = Node('rescue_command_adapter')
    log = node.get_logger()
    string, double = ParameterType.PARAMETER_STRING, ParameterType.PARAMETER_DOUBLE
    names = {'allowed_robot_ids': ParameterType.PARAMETER_STRING_ARRAY,
             'robot_id': string, 'namespace': string, 'authority_epoch': string,
             'nav_action': string, 'odom_topic': string, 'dock_status_topic': string,
             'policy_version': string, 'step_period': double, 'clock_error_bound_s': double,
             'scan_topic': string, 'tf_topic': string, 'map_frame': string, 'odom_frame': string,
             'readiness.version': string}
    names.update({'policy.' + field: double for field in POLICY_FIELDS})
    names.update({'readiness.max_age_s.' + source: double
                  for source in ('odom', 'scan', 'camera', 'wheel_status', 'map_to_odom_tf')})
    # May be empty on purpose: no camera / no wheel_status / no watchdog source yet.
    optional = {'camera_topics': ParameterType.PARAMETER_STRING_ARRAY,
                'wheel_status_topic': string, 'watchdog_heartbeat_topic': string}
    names.update(optional)
    for name, kind in names.items():  # no defaults: every value comes from launch/profile
        node.declare_parameter(name, descriptor=ParameterDescriptor(type=kind, dynamic_typing=False))
    value = {name: node.get_parameter(name).value for name in names}
    missing = sorted(name for name, item in value.items()
                     if item is None or (name not in optional and item in ('', [])))
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    if value['watchdog_heartbeat_topic']:
        # Robot-side command watchdog + heartbeat does not exist yet (C-01, TR-11).
        log.fatal('watchdog_heartbeat_topic must be empty until the robot-side watchdog exists')
        raise SystemExit(2)
    # Profile values (config/deployment.*.json) arrive as parameters from bringup;
    # this package does not import rescue_bringup (dependency points the other way).
    if node.get_namespace() != value['namespace']:
        log.fatal(f'namespace mismatch: node {node.get_namespace()} != profile {value["namespace"]}')
        raise SystemExit(2)
    policy = Policy(value['policy_version'], *(value['policy.' + field] for field in POLICY_FIELDS))
    executor = Executor(value['robot_id'], str(uuid4()), value['authority_epoch'], policy,
                        allowed_robot_ids=tuple(value['allowed_robot_ids']))
    port = Nav2Port(node, value['nav_action'])

    def guards(task):
        # MissionGoalBinder needs an alignment source in this process (mapping ROS
        # adapter, not built yet); until then every dispatch is refused.
        log.warn(f'guard rejects {task.command_id}: mapping alignment input not connected')
        return False
    adapter = CommandAdapter(executor, port, guards=guards, clock_error_bound_s=value['clock_error_bound_s'])
    port.bind(adapter)
    cameras = tuple(value['camera_topics'])
    ages = {source: value['readiness.max_age_s.' + source]
            for source in ('odom', 'scan', 'camera', 'wheel_status', 'map_to_odom_tf')}
    monitor = ReadinessMonitor(readiness_policy(value['readiness.version'], ages, cameras,
                                                value['wheel_status_topic']))
    bind_odometry(node, adapter, value['odom_topic'], monitor)
    bind_dock_status(node, adapter, value['dock_status_topic'])
    bind_readiness(node, monitor, adapter, scan_topic=value['scan_topic'], camera_topics=cameras,
                   wheel_status_topic=value['wheel_status_topic'], tf_topic=value['tf_topic'],
                   map_frame=value['map_frame'], odom_frame=value['odom_frame'])

    def step():
        adapter.observe_status(nav2=port.server_ready())
        monitor.publish(adapter)
        adapter.step()
    node.create_timer(value['step_period'], step)
    log.info(f'git_sha={git_sha()} boot={executor.robot_boot_id} parameters={value}')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
