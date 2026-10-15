"""ROS 2 boundary of the DB writer: subscribe, convert with ros_ingest, write site_state.

The only process that writes the site-state DB (db-schema: one writer per file). It
subscribes to RobotStatus and RobotPose per robot, the fixed camera's SiteEvent and the
coordinator's SystemState and TargetSnapshot (1차 IDL), each robot's overhead passage list (C-04b) and the SLAM
robot's MapState (the current map the web compares passages to, OPS-194). It also serves the operator command API
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
       'targets': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
       # QR exit observations: VOLATILE matches a latched or a plain publisher, every one is kept
       'exit_location': ('RELIABLE', 'VOLATILE', 10),
       # C-04b and IF-009 publishers latch the latest; a late start still gets each robot's list and the map state
       'overhead_passages': ('RELIABLE', 'TRANSIENT_LOCAL', 1), 'map_state': ('RELIABLE', 'TRANSIENT_LOCAL', 1)}
ROBOT_LISTS = ('robot_roles', 'status_topics', 'pose_topics', 'overhead_passages_topics')
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


CAMERA_BY_ID_DIR = '/dev/v4l/by-id'   # same naming as the launch webcam_device (#204)


def plugged_cameras(by_id_dir):
    """Plugged camera names: the capture node of each (``*-video-index0``), as #204 lists them."""
    return sorted(p.name for p in Path(by_id_dir).glob('*-video-index0'))


def node_config(values):
    """Parameter values -> checked config. Raises OperationsError(INVALID_INPUT) before anything starts."""
    ids = list(values['allowed_robot_ids'])
    allowed_robots(ids)  # exactly two distinct profile robot IDs
    for name in ROBOT_LISTS:
        require(len(values[name]) == len(ids), f'{name} must list one entry per allowed robot, in the same order')
    roles = list(values['robot_roles'])
    require(all(role in ROBOT_ROLES for role in roles) and len(set(roles)) == len(roles),
            'robot_roles must be SCOUT and SUPPORT, one each')
    topics = [text(topic, name) for name in ('status_topics', 'pose_topics', 'overhead_passages_topics')
              for topic in values[name]]
    topics.append(text(values['map_state_topic'], 'map_state_topic'))  # the SLAM robot's (OPS-194)
    topics.append(text(values['site_events_topic'], 'site_events_topic'))
    topics.append(text(values['system_state_topic'], 'system_state_topic'))
    topics.append(text(values['targets_topic'], 'targets_topic'))
    topics.append(text(values['exit_location_topic'], 'exit_location_topic'))  # QR exit (OPS-144)
    require(len(set(topics)) == len(topics), 'every subscribed topic must be different')
    port, timeout_s = values['api_port'], values['command_timeout_s']
    require(type(port) is int and 1 <= port <= 65535, 'api_port must be 1..65535')
    require(isinstance(timeout_s, float) and 0.0 < timeout_s < CONSOLE_TIMEOUT_S,
            f'command_timeout_s must be over 0 and under the console {CONSOLE_TIMEOUT_S:g} s')
    webcam_node = values['webcam_node']
    require(isinstance(webcam_node, str) and (webcam_node == '' or webcam_node.startswith('/')),
            "webcam_node must be the webcam_node's absolute name, or '' for no console camera pick")
    # The path itself (parent exists, not a directory) is checked when the ledger opens it.
    return {'webcam_node': webcam_node, 'db_path': text(values['db_path'], 'db_path', 4096), 'site_events_topic': values['site_events_topic'],
            'system_state_topic': values['system_state_topic'], 'targets_topic': values['targets_topic'],
            'exit_location_topic': values['exit_location_topic'],
            'overhead_passages_topics': dict(zip(ids, values['overhead_passages_topics'])),
            'map_state_topic': values['map_state_topic'],
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
    from rescue_interfaces.msg import (MapState, RobotPose, RobotStatus, SiteEvent, SystemState, TargetLocation,
                                       TargetSnapshot)
    from std_msgs.msg import String
    from rescue_interfaces.srv import SubmitOperatorCommand

    from .command_api import CommandGate, CommandTimeout, CommandUnavailable, RunReset, WebcamSwitch, make_server

    from .ros_ingest import SiteStateIngest
    from .site_state import SiteStateLedger

    rclpy.init(args=args)
    node = Node('rescue_db_writer')
    log = node.get_logger()
    names = {'db_path': Parameter.Type.STRING, 'allowed_robot_ids': Parameter.Type.STRING_ARRAY,
             'robot_roles': Parameter.Type.STRING_ARRAY, 'status_topics': Parameter.Type.STRING_ARRAY,
             'pose_topics': Parameter.Type.STRING_ARRAY, 'site_events_topic': Parameter.Type.STRING,
             'system_state_topic': Parameter.Type.STRING, 'targets_topic': Parameter.Type.STRING,
             'exit_location_topic': Parameter.Type.STRING,
             'overhead_passages_topics': Parameter.Type.STRING_ARRAY, 'map_state_topic': Parameter.Type.STRING,
             'api_host': Parameter.Type.STRING, 'api_port': Parameter.Type.INTEGER,
             'operator_command_service': Parameter.Type.STRING, 'command_timeout_s': Parameter.Type.DOUBLE,
             'webcam_node': Parameter.Type.STRING}
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
             f'exit_location={config["exit_location_topic"]} '
             f'overhead_passages={config["overhead_passages_topics"]} map_state={config["map_state_topic"]} '
             f'api=http://{config["api_host"]}:{config["api_port"]} service={config["operator_command_service"]}')

    def qos(kind):
        reliability, durability, depth = QOS[kind]
        return QoSProfile(depth=depth, reliability=getattr(ReliabilityPolicy, reliability),
                          durability=getattr(DurabilityPolicy, durability))

    # 구독 콜백 공통 틀: 받은 메시지에 이 PC 의 수신 시각을 붙여 ingest 의 on_* 로 넘긴다.
    # on_* 는 기록 결과(True/False)를 돌려주고, 형식이 틀려 거절했으면 None 을 돌려준다.
    # 거절 사유는 on_* 가 error_log 표에 남기고(관제 웹의 명령·오류 기록에 보임), 여기서는 경고 로그만 쓴다.
    # DB 자체가 실패하면 error_log 에도 쓸 수 없으므로 오류 로그만 남긴다.
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

    # 구독 목록: 받는 토픽(메시지 타입, 보내는 노드) → ingest 함수 → 기록하는 현장 상태 DB 표.
    # 관제 웹은 이 표들을 읽기 전용으로만 읽는다. 토픽 이름은 모두 launch 파라미터에서 온다.
    #   site_events_topic   (SiteEvent, 고정 웹캠 webcam_node)          → on_site_event      → site_events
    #   system_state_topic  (SystemState, mission_coordinator)          → on_system_state    → mission_state(1행)
    #   targets_topic       (TargetSnapshot, mission_coordinator)       → on_target_snapshot → target_states
    #   exit_location_topic (TargetLocation, QR 출구 exit_locator)      → on_exit_location   → exit_locations
    #   map_state_topic     (MapState, SLAM 로봇의 task_input)          → on_map_state       → current_map(1행)
    # 아래 두 반복문은 로봇마다 구독한다(터널 판정 목록, 로봇 상태·위치).
    subscriptions = [node.create_subscription(SiteEvent, config['site_events_topic'],
                                              handler(ingest.on_site_event, 'site_event'), qos('site_events')),
                     node.create_subscription(SystemState, config['system_state_topic'],
                                              handler(ingest.on_system_state, 'system_state'), qos('system_state')),
                     node.create_subscription(TargetSnapshot, config['targets_topic'],
                                              handler(ingest.on_target_snapshot, 'targets'), qos('targets')),
                     node.create_subscription(TargetLocation, config['exit_location_topic'],
                                              handler(ingest.on_exit_location, 'exit_location'),
                                              qos('exit_location')),
                     node.create_subscription(MapState, config['map_state_topic'],
                                              handler(ingest.on_map_state, 'map_state'), qos('map_state'))]
    # 로봇마다 터널 판정 목록(String JSON, overhead_passage 노드) → overhead_passage_lists·overhead_passages.
    # 메시지가 어느 로봇의 목록인지 콜백이 알아야 해서 robot 을 람다 기본 인자로 묶는다(반복문 변수 늦은 바인딩 방지).
    for robot, topic in config['overhead_passages_topics'].items():
        on_list = handler(lambda msg, received_at_ns, robot=robot: ingest.on_overhead_passages(
            msg, robot_id=robot, received_at_ns=received_at_ns), robot + ' overhead_passages')
        subscriptions.append(node.create_subscription(String, topic, on_list, qos('overhead_passages')))
    # 로봇마다 상태(RobotStatus)와 위치(RobotPose), 둘 다 로봇 PC 의 task_input(rescue_command_adapter)이 보낸다.
    # 상태는 robot_status 표의 그 로봇 행을 통째로 바꾸고, 위치는 같은 행의 위치만 고친다(VALID 위치만 보이게).
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
    webcam = None
    if config['webcam_node']:   # the console's camera pick (OPS-183): webcam_node's own parameter services
        from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        get_client = node.create_client(GetParameters, config['webcam_node'] + '/get_parameters')
        set_client = node.create_client(SetParameters, config['webcam_node'] + '/set_parameters')

        def call(client, req):
            if not client.service_is_ready():
                raise CommandUnavailable(f'{client.srv_name} not available')
            done = threading.Event()
            future = client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            if not done.wait(config['command_timeout_ns'] / 1e9):
                client.remove_pending_request(future)
                raise CommandTimeout(f'no answer within {config["command_timeout_ns"] / 1e9:g} s')
            return future.result()

        def get_device():
            return call(get_client, GetParameters.Request(names=['device_path'])).values[0].string_value

        def set_device(path):
            value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=path)
            result = call(set_client, SetParameters.Request(
                parameters=[ParameterMsg(name='device_path', value=value)])).results[0]
            return result.successful, result.reason

        webcam = WebcamSwitch(ledger, list_cameras=lambda: plugged_cameras(CAMERA_BY_ID_DIR), get_device=get_device,
                              set_device=set_device, by_id_dir=CAMERA_BY_ID_DIR + '/')
    try:
        # The reset runs on this process's one ledger (the file's only writer) and reads the coordinator boot
        # the ingest heard last, so the new run stops recording that coordinator (초기화, 1.5).
        reset = RunReset(ledger, coordinator_boot_id=lambda: ingest.coordinator_boot_id)
        server = make_server(config['api_host'], config['api_port'], gate, log=log.debug, webcam=webcam, reset=reset)
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
