"""ROS 2 boundary of the DB writer: subscribe, convert with ros_ingest, write site_state.

The only process that writes the site-state DB (db-schema: one writer per file). It
subscribes to RobotStatus and RobotPose per robot, the fixed camera's SiteEvent and the
coordinator's SystemState and TargetSnapshot (1차 IDL). It also serves the operator command API
(command_api.py, #115): the console posts a command, this calls the coordinator's
SubmitOperatorCommand service and records the answer. It publishes nothing. ROS packages are imported inside
main() so this module imports without ROS. Topic names, roles and the DB path come
from parameters (launch/profile); nothing here has a default (AGENTS 8절).

Not yet: map snapshots, RequestStop and the other HTTP routes of console-and-db-plan 4절.
"""
import signal
import subprocess
import threading
import time
from pathlib import Path
from uuid import uuid4

from rescue_contracts import ROBOT_ROLES

from .schema import OperationsError, allowed_robots, require, text
from .shutdown import spin_until_shutdown
from .site_state import SCHEMA_VERSION

# Subscriber QoS (reliability, durability, depth). Reliability/depth follow SDD v0.1 IF-010/011/071/076 [제안].
# Durability is VOLATILE even where the SDD publisher is TRANSIENT_LOCAL (IF-011): a VOLATILE
# subscription matches both kinds of publisher, a TRANSIENT_LOCAL one refuses a VOLATILE publisher
# (local ROS run with the #90 probe, 2026-10-11). Status is periodic, so no latched copy is needed.
# SystemState keeps TRANSIENT_LOCAL: its only publisher (#90) latches it, and a late start must still
# get the coordinator's current mission state. TargetSnapshot likewise (#96 latches it, IF-070).
QOS = {'status': ('RELIABLE', 'VOLATILE', 1), 'pose': ('RELIABLE', 'VOLATILE', 1),
       'site_events': ('RELIABLE', 'VOLATILE', 10), 'system_state': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
       'targets': ('RELIABLE', 'TRANSIENT_LOCAL', 1)}
ROBOT_LISTS = ('robot_roles', 'status_topics', 'pose_topics')
# The console gives up after 5 s (#115 COMMAND_TIMEOUT_S); this must answer before that.
CONSOLE_TIMEOUT_S = 5.0


class LockedLedger:
    """The one SiteStateLedger behind a lock: every method call runs alone."""

    def __init__(self, ledger):
        self._ledger = ledger
        self._lock = threading.Lock()

    def __getattr__(self, name):
        value = getattr(self._ledger, name)
        if not callable(value):
            return value

        def locked(*args, **kwargs):
            with self._lock:
                return value(*args, **kwargs)
        return locked


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
    topics.append(text(values['system_state_topic'], 'system_state_topic'))
    topics.append(text(values['targets_topic'], 'targets_topic'))
    require(len(set(topics)) == len(topics), 'every subscribed topic must be different')
    port, timeout_s = values['api_port'], values['command_timeout_s']
    require(type(port) is int and 1 <= port <= 65535, 'api_port must be 1..65535')
    require(isinstance(timeout_s, float) and 0.0 < timeout_s < CONSOLE_TIMEOUT_S,
            f'command_timeout_s must be over 0 and under the console {CONSOLE_TIMEOUT_S:g} s')
    # The path itself (parent exists, not a directory) is checked when the ledger opens it.
    return {'db_path': text(values['db_path'], 'db_path', 4096), 'site_events_topic': values['site_events_topic'],
            'system_state_topic': values['system_state_topic'], 'targets_topic': values['targets_topic'],
            'api_host': text(values['api_host'], 'api_host'), 'api_port': port,
            'operator_command_service': text(values['operator_command_service'], 'operator_command_service'),
            'command_timeout_ns': round(timeout_s * 1e9),
            'roles': dict(zip(ids, roles)),
            'robots': list(zip(ids, values['status_topics'], values['pose_topics']))}


def teardown(*steps):
    """Ignore SIGINT for the rest of the process, then run every cleanup step; the first error is raised
    after all of them ran.

    ``ros2 launch`` and the terminal both send SIGINT on Ctrl-C. The first one stops ``rclpy.spin``;
    the second used to land in ``ThreadingHTTPServer.shutdown()`` as a KeyboardInterrupt, so the
    ledger and node were never closed and the process ended with a traceback (exit -2, measured
    2026-10-13). The handler is not put back: the process is ending, and a SIGINT after the last
    step would still end it with -2. A hung step is ended by launch's SIGTERM/SIGKILL.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    error = None
    for step in steps:
        try:
            step()
        except Exception as exc:  # keep closing; report the first failure afterwards
            error = error or exc
    if error is not None:
        raise error


def main(args=None):
    import rclpy
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import RobotPose, RobotStatus, SiteEvent, SystemState, TargetSnapshot
    from rescue_interfaces.srv import SubmitOperatorCommand

    from .command_api import CommandGate, CommandTimeout, CommandUnavailable, make_server

    from .ros_ingest import SiteStateIngest
    from .site_state import SiteStateLedger

    rclpy.init(args=args)
    node = Node('rescue_db_writer')
    log = node.get_logger()
    names = {'db_path': Parameter.Type.STRING, 'allowed_robot_ids': Parameter.Type.STRING_ARRAY,
             'robot_roles': Parameter.Type.STRING_ARRAY, 'status_topics': Parameter.Type.STRING_ARRAY,
             'pose_topics': Parameter.Type.STRING_ARRAY, 'site_events_topic': Parameter.Type.STRING,
             'system_state_topic': Parameter.Type.STRING, 'targets_topic': Parameter.Type.STRING,
             'api_host': Parameter.Type.STRING, 'api_port': Parameter.Type.INTEGER,
             'operator_command_service': Parameter.Type.STRING, 'command_timeout_s': Parameter.Type.DOUBLE}
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
    ledger = LockedLedger(ledger)  # subscriptions (executor thread) and HTTP requests share one writer
    ingest = SiteStateIngest(ledger, roles=config['roles'])
    log.info(f'rescue_db_writer git={git_sha()} db={config["db_path"]} schema={SCHEMA_VERSION} '
             f'robots={config["robots"]} roles={config["roles"]} site_events={config["site_events_topic"]} '
             f'system_state={config["system_state_topic"]} targets={config["targets_topic"]} '
             f'api=http://{config["api_host"]}:{config["api_port"]} service={config["operator_command_service"]}')

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
                                              handler(ingest.on_site_event, 'site_event'), qos('site_events')),
                     node.create_subscription(SystemState, config['system_state_topic'],
                                              handler(ingest.on_system_state, 'system_state'), qos('system_state')),
                     node.create_subscription(TargetSnapshot, config['targets_topic'],
                                              handler(ingest.on_target_snapshot, 'targets'), qos('targets'))]
    for robot, status_topic, pose_topic in config['robots']:
        subscriptions.append(node.create_subscription(
            RobotStatus, status_topic, handler(ingest.on_robot_status, robot + ' status'), qos('status')))
        subscriptions.append(node.create_subscription(
            RobotPose, pose_topic, handler(ingest.on_robot_pose, robot + ' pose'), qos('pose')))
    client = node.create_client(SubmitOperatorCommand, config['operator_command_service'])
    boot_id, sequence = str(uuid4()), [0]

    def send(fields):
        """One SubmitOperatorCommand call from an HTTP thread; the executor delivers the answer."""
        if not client.service_is_ready():
            raise CommandUnavailable(f'{config["operator_command_service"]} not available')
        req = SubmitOperatorCommand.Request(**fields)
        sequence[0] += 1
        env = req.envelope
        env.schema_version, env.publisher_id, env.publisher_boot_id = '1.0', 'rescue_operations', boot_id
        env.sequence, env.published_at_ns = sequence[0], time.time_ns()
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(config['command_timeout_ns'] / 1e9):
            client.remove_pending_request(future)
            raise CommandTimeout(f'no answer within {config["command_timeout_ns"] / 1e9:g} s')
        res = future.result()
        return dict(accepted=res.accepted, request_state=res.request_state, reason_code=res.reason_code,
                    created_task_id=res.created_task_id)

    gate = CommandGate(ledger, send, timeout_ns=config['command_timeout_ns'])
    try:
        server = make_server(config['api_host'], config['api_port'], gate, log=log.debug)
    except OSError as exc:
        log.fatal(f'cannot serve the command API on {config["api_host"]}:{config["api_port"]}: {exc}')
        ledger.close()
        raise SystemExit(2) from exc
    threading.Thread(target=server.serve_forever, name='command-api', daemon=True).start()
    try:
        spin_until_shutdown(lambda: rclpy.spin(node), node.context.ok, RCLError)
    except (KeyboardInterrupt, ExternalShutdownException):  # jazzy: SIGINT shuts the context down
        pass
    finally:
        teardown(lambda: log.info('rescue_db_writer stopping'), server.shutdown, server.server_close, ledger.close,
                 node.destroy_node, rclpy.try_shutdown)
