"""ROS 2 boundary of the DB writer: subscribe, convert with ros_ingest, write site_state.

The only process that writes the site-state DB (db-schema: one writer per file). It
subscribes to RobotStatus and RobotPose per robot and to the fixed camera's SiteEvent
(1차 IDL), and never publishes or commands anything. ROS packages are imported inside
main() so this module imports without ROS. Topic names, roles and the DB path come
from parameters (launch/profile); nothing here has a default (AGENTS 8절).

Not yet: the mission state (SystemState is not in the 1차 IDL on main; #88), target
states, operator commands and map snapshots.
"""
import subprocess
import time
from pathlib import Path

from rescue_contracts import ROBOT_ROLES

from .schema import OperationsError, allowed_robots, require, text
from .site_state import SCHEMA_VERSION

# SDD v0.1 IF-010/011/042/043/076 [제안]: reliability / durability / depth per topic.
QOS = {'status': ('RELIABLE', 'TRANSIENT_LOCAL', 1), 'pose': ('RELIABLE', 'VOLATILE', 1),
       'site_events': ('RELIABLE', 'VOLATILE', 10)}
ROBOT_LISTS = ('robot_roles', 'status_topics', 'pose_topics')


def git_sha():
    """Source checkout SHA, or 'UNKNOWN' (installed copies have no .git)."""
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def node_config(values):
    """Parameter values -> checked config. Raises OperationsError(INVALID_INPUT) before anything starts."""
    ids = list(values['allowed_robot_ids'])
    allowed_robots(ids)  # exactly two distinct profile robot IDs
    for name in ROBOT_LISTS:
        require(len(values[name]) == len(ids), f'{name} must list one entry per allowed robot, in the same order')
    roles = list(values['robot_roles'])
    require(all(role in ROBOT_ROLES for role in roles) and len(set(roles)) == len(roles),
            'robot_roles must be SCOUT and SUPPORT, one each')
    topics = [text(topic, name) for name in ('status_topics', 'pose_topics') for topic in values[name]]
    topics.append(text(values['site_events_topic'], 'site_events_topic'))
    require(len(set(topics)) == len(topics), 'every subscribed topic must be different')
    # The path itself (parent exists, not a directory) is checked when the ledger opens it.
    return {'db_path': text(values['db_path'], 'db_path', 4096), 'site_events_topic': values['site_events_topic'],
            'roles': dict(zip(ids, roles)),
            'robots': list(zip(ids, values['status_topics'], values['pose_topics']))}


def main(args=None):
    import rclpy
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import RobotPose, RobotStatus, SiteEvent

    from .ros_ingest import SiteStateIngest
    from .site_state import SiteStateLedger

    rclpy.init(args=args)
    node = Node('rescue_db_writer')
    log = node.get_logger()
    names = {'db_path': Parameter.Type.STRING, 'allowed_robot_ids': Parameter.Type.STRING_ARRAY,
             'robot_roles': Parameter.Type.STRING_ARRAY, 'status_topics': Parameter.Type.STRING_ARRAY,
             'pose_topics': Parameter.Type.STRING_ARRAY, 'site_events_topic': Parameter.Type.STRING}
    values = {}
    for name, kind in names.items():  # no defaults: every value comes from launch/profile
        node.declare_parameter(name, kind)
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, value in values.items() if value is None)
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    try:
        config = node_config(values)
        ledger = SiteStateLedger(config['db_path'], allowed_robot_ids=tuple(config['roles']))
    except OperationsError as exc:
        log.fatal(f'cannot start DB writer: {exc.detail_code}: {exc.detail}')
        raise SystemExit(2) from exc
    ingest = SiteStateIngest(ledger, roles=config['roles'])
    log.info(f'rescue_db_writer git={git_sha()} db={config["db_path"]} schema={SCHEMA_VERSION} '
             f'robots={config["robots"]} roles={config["roles"]} site_events={config["site_events_topic"]}')

    def qos(kind):
        reliability, durability, depth = QOS[kind]
        return QoSProfile(depth=depth, reliability=getattr(ReliabilityPolicy, reliability),
                          durability=getattr(DurabilityPolicy, durability))

    def handler(method, label):
        def on_message(msg):
            try:
                result = method(msg, received_at_ns=time.time_ns())
            except OperationsError as exc:  # the DB itself failed; it cannot hold this error
                log.error(f'{label}: DB write failed: {exc.detail_code}: {exc.detail}')
                return
            if result is None:
                log.warning(f'{label}: refused, see error_log')
        return on_message

    subscriptions = [node.create_subscription(SiteEvent, config['site_events_topic'],
                                              handler(ingest.on_site_event, 'site_event'), qos('site_events'))]
    for robot, status_topic, pose_topic in config['robots']:
        subscriptions.append(node.create_subscription(
            RobotStatus, status_topic, handler(ingest.on_robot_status, robot + ' status'), qos('status')))
        subscriptions.append(node.create_subscription(
            RobotPose, pose_topic, handler(ingest.on_robot_pose, robot + ' pose'), qos('pose')))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ledger.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
