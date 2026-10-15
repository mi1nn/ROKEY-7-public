"""ROS 2 boundary for CommandAdapter: NavigateToPose client, odometry, dock status.

ROS packages are imported inside functions so `rescue_robot` stays importable
and testable without ROS. ExecuteTask IDL exists in rescue_interfaces, but this
default CLI has no task runtime attached, and its guard rejects every dispatch
until the mapping revision guard is connected, so it cannot start a motion on
its own. It observes odometry, dock status and Nav2 availability.
"""
import math
import subprocess
import time
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
    """NavigationPort over one robot's NavigateToPose action (namespace-relative name).

    Ready = the action server is on the graph AND the Nav2 lifecycle manager answered is_active success within
    max_age_s. bt_navigator creates its action server while configuring, so the graph alone says nothing about an
    aborted bringup (field run1015b, 2026-10-15: inactive bt_navigator, nav2 READY, first goal rejected).
    No answer, an old answer, a failed call or no service is not ready.
    """

    def __init__(self, node, action_name, lifecycle_service, *, clock, max_age_s):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from std_srvs.srv import Trigger
        self.node = node
        self.goal_type = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.trigger = Trigger
        self.lifecycle = node.create_client(Trigger, lifecycle_service)
        self.lifecycle_service = lifecycle_service
        self.clock, self.max_age_s = clock, max_age_s
        self.active = None          # (is_active success, clock() when answered)
        self.asking = None          # (future, clock() when asked)
        self.adapter = None
        self.handles = {}
        self.pending_cancel = set()

    def bind(self, adapter):
        self.adapter = adapter

    def server_ready(self):
        now = self.clock()
        self._ask(now)
        fresh = self.active is not None and 0 <= now - self.active[1] <= self.max_age_s
        return bool(self.client.server_is_ready() and fresh and self.active[0])

    def _ask(self, now):
        # three questions per max_age_s; an unanswered one is dropped and asked again
        if self.asking is not None and now - self.asking[1] < self.max_age_s / 3:
            return
        if self.asking is not None:
            if self.asking[0] is not None:
                self.lifecycle.remove_pending_request(self.asking[0])
            self.asking = None
        if not self.lifecycle.service_is_ready():
            return
        future = self.lifecycle.call_async(self.trigger.Request())
        self.asking = (future, now)
        future.add_done_callback(self._on_active)

    def _on_active(self, future):
        if self.asking is not None and self.asking[0] is future:
            self.asking = (None, self.asking[1])     # answered: ask again on schedule, nothing to drop
        try:
            success = bool(future.result().success)
        except Exception as exc:  # a failed call is no answer
            self.node.get_logger().warn(f'NAV2_LIFECYCLE {self.lifecycle_service} no answer: {exc}')
            return
        if self.active is None or self.active[0] != success:
            self.node.get_logger().info(f'NAV2_LIFECYCLE {self.lifecycle_service} '
                                        f'{"active" if success else "inactive: nav2 NOT_READY"}')
        self.active = (success, self.clock())

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
    CompressedImage (TB4 OAK-D default, P4 DEC-004), so is .../compressedDepth (passage depth); .../camera_info is
    CameraInfo (the camera runs, without pulling its images over Wi-Fi again, 실습 기록 #339 B7); anything else Image."""
    name = topic.rstrip('/')
    if name.endswith('/camera_info'):
        return 'CameraInfo'
    return 'CompressedImage' if name.endswith(('/compressed', '/compressedDepth')) else 'Image'


def bind_readiness(node, monitor, adapter, *, scan_topic, camera_topics, wheel_status_topic,
                   tf_topic, map_frame, odom_frame, watchdog_heartbeat_topic):
    """Arrival-only subscriptions feeding ReadinessMonitor (no message content judged,
    except that only the map->odom edge counts for slam freshness)."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
    from tf2_msgs.msg import TFMessage

    def arrival(source):
        return lambda _msg: monitor.arrived(source, adapter.clock())

    subs = [node.create_subscription(LaserScan, scan_topic, arrival('scan'), qos_profile_sensor_data)]
    for topic in camera_topics:
        kind = {'CameraInfo': CameraInfo, 'CompressedImage': CompressedImage}.get(camera_message_type(topic), Image)
        subs.append(node.create_subscription(kind, topic, arrival('camera:' + topic), qos_profile_sensor_data))
    if wheel_status_topic:
        from irobot_create_msgs.msg import WheelStatus
        subs.append(node.create_subscription(WheelStatus, wheel_status_topic, arrival('wheel_status'),
                                             qos_profile_sensor_data))

    if watchdog_heartbeat_topic:
        from std_msgs.msg import Header
        subs.append(node.create_subscription(Header, watchdog_heartbeat_topic,
                                             arrival('watchdog_heartbeat'), 10))

    def on_tf(msg):
        if any(t.header.frame_id == map_frame and t.child_frame_id == odom_frame for t in msg.transforms):
            monitor.arrived('map_to_odom_tf', adapter.clock())
    subs.append(node.create_subscription(TFMessage, tf_topic, on_tf, 100))
    return subs


def readiness_policy(version, ages, camera_topics, wheel_status_topic, watchdog_heartbeat_topic):
    """driver=odom(+wheel_status), sensors=scan(+cameras), slam=map->odom TF,
    watchdog=command watchdog heartbeat (only when its topic is configured)."""
    max_age = {'odom': ages['odom'], 'scan': ages['scan'], 'map_to_odom_tf': ages['map_to_odom_tf']}
    driver, sensors = ('odom',), ('scan',)
    if wheel_status_topic:
        max_age['wheel_status'] = ages['wheel_status']
        driver += ('wheel_status',)
    for topic in camera_topics:
        max_age['camera:' + topic] = ages['camera']
        sensors += ('camera:' + topic,)
    flags = {'driver': driver, 'sensors': sensors, 'slam': ('map_to_odom_tf',)}
    if watchdog_heartbeat_topic:
        max_age['watchdog_heartbeat'] = ages['watchdog_heartbeat']
        flags['watchdog'] = ('watchdog_heartbeat',)
    return ReadinessPolicy(version, max_age, flags)


def declare_required(node, names, optional=()):
    """Declare statically typed parameters without defaults -> (values, missing names).

    rclpy (jazzy) refuses `declare_parameter(name, descriptor=...)` with no value,
    so the type goes in as `Parameter.Type`; an unset one stays uninitialized and
    reads as None. Empty strings/arrays are missing unless named in `optional`.
    """
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterDescriptor
    values = {}
    for name, kind in names.items():  # kind: rcl_interfaces ParameterType value
        node.declare_parameter(name, Parameter.Type(kind), descriptor=ParameterDescriptor(
            type=kind,read_only=True,description='Explicit startup setting; no deployment default'))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, item in values.items()
                     if item is None or (name not in optional and item in ('', [])))
    return values, missing


READINESS_SOURCES = ('odom', 'scan', 'camera', 'wheel_status', 'map_to_odom_tf', 'watchdog_heartbeat')


def main(args=None, *, task_runtime_factory=None):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterType

    rclpy.init(args=args)
    node = None
    task_runtime = None
    try:
        node = Node('rescue_command_adapter')
        log = node.get_logger()
        string, double = ParameterType.PARAMETER_STRING, ParameterType.PARAMETER_DOUBLE
        names = {'allowed_robot_ids': ParameterType.PARAMETER_STRING_ARRAY,
                 'robot_id': string, 'namespace': string, 'authority_epoch': string,
                 'nav_action': string, 'odom_topic': string, 'dock_status_topic': string,
                 'policy_version': string, 'step_period': double, 'clock_error_bound_s': double,
                 'scan_topic': string, 'tf_topic': string, 'map_frame': string, 'odom_frame': string,
                 'readiness.version': string, 'executor_heartbeat_topic': string,
                 'nav2_lifecycle_service': string}
        names.update({'policy.' + field: double for field in POLICY_FIELDS})
        names.update({'readiness.max_age_s.' + source: double
                      for source in READINESS_SOURCES})
        # May be empty on purpose: no camera / no wheel_status / no command watchdog
        # (empty watchdog_heartbeat_topic = no watchdog flag = never ready to move).
        optional = {'camera_topics': ParameterType.PARAMETER_STRING_ARRAY,
                    'wheel_status_topic': string, 'watchdog_heartbeat_topic': string}
        names.update(optional)
        # No defaults: every value comes from launch/profile.
        value, missing = declare_required(node, names, optional)
        if missing:
            log.fatal(f'missing parameters: {missing}')
            raise SystemExit(2)
        # Profile values (config/deployment.*.json) arrive as parameters from bringup;
        # this package does not import rescue_bringup (dependency points the other way).
        if node.get_namespace() != value['namespace']:
            log.fatal(f'namespace mismatch: node {node.get_namespace()} != profile {value["namespace"]}')
            raise SystemExit(2)
        policy = Policy(value['policy_version'], *(value['policy.' + field] for field in POLICY_FIELDS))
        executor = Executor(value['robot_id'], str(uuid4()), value['authority_epoch'], policy,
                            allowed_robot_ids=tuple(value['allowed_robot_ids']))
        # Nav2 READY needs its lifecycle manager active too; an answer counts as long as other evidence does
        port = Nav2Port(node, value['nav_action'], value['nav2_lifecycle_service'], clock=time.monotonic,
                        max_age_s=policy.evidence_max_age)

        def guards(task):
            # MissionGoalBinder needs an alignment source in this process (mapping ROS
            # adapter, not built yet); until then every dispatch is refused.
            log.warn(f'guard rejects {task.command_id}: mapping alignment input not connected')
            return False
        adapter = CommandAdapter(executor, port, guards=guards, clock_error_bound_s=value['clock_error_bound_s'])
        port.bind(adapter)
        cameras = tuple(value['camera_topics'])
        ages = {source: value['readiness.max_age_s.' + source]
                for source in READINESS_SOURCES}
        monitor = ReadinessMonitor(readiness_policy(value['readiness.version'], ages, cameras,
                                                    value['wheel_status_topic'],
                                                    value['watchdog_heartbeat_topic']))
        bind_odometry(node, adapter, value['odom_topic'], monitor)
        bind_dock_status(node, adapter, value['dock_status_topic'])
        bind_readiness(node, monitor, adapter, scan_topic=value['scan_topic'], camera_topics=cameras,
                       wheel_status_topic=value['wheel_status_topic'], tf_topic=value['tf_topic'],
                       map_frame=value['map_frame'], odom_frame=value['odom_frame'],
                       watchdog_heartbeat_topic=value['watchdog_heartbeat_topic'])
        # A site integration supplies authenticated lease/goal records, mapping
        # authority and a result encoder. The ordinary CLI stays fail-closed.
        if task_runtime_factory is not None:
            if not callable(task_runtime_factory):
                raise ValueError('task_runtime_factory must be callable')
            task_runtime = task_runtime_factory(node, adapter)
        from std_msgs.msg import Header
        # Executor liveness for the command watchdog (C-01): one Header per step.
        heartbeat = node.create_publisher(Header, value['executor_heartbeat_topic'], 10)

        def step():
            adapter.observe_status(nav2=port.server_ready())
            monitor.publish(adapter)
            adapter.step()
            heartbeat.publish(Header(stamp=node.get_clock().now().to_msg()))
        node.create_timer(value['step_period'], step)
        log.info(f'git_sha={git_sha()} boot={executor.robot_boot_id} parameters={value}')
        rclpy.spin(node)
    except (RescueError, ValueError, OSError) as exc:
        if node is not None:
            node.get_logger().fatal('command adapter failed: '+str(exc))
        raise SystemExit(2) from exc
    finally:
        try:
            if task_runtime is not None:
                task_runtime.destroy()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown()
