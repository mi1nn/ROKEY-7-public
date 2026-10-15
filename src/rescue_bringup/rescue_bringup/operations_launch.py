"""operations PC launch values: profile v2.1 -> db_writer, report_node, mjpeg_relay parameters (no ROS import).

``launch/operations.launch.py`` loads the profile, parses its text launch arguments with
``parse_launch_arguments`` and hands ``db_writer_parameters`` to the node. Robot IDs and
roles come from the profile (mission roles), so a field run with other robots changes the
profile file only. Topic and service names are the coordinator's (rescue_coordination
ros_node constants); a test keeps them equal without importing it here.
"""
import math
import re
import os

from rescue_contracts.overhead import overhead_passages_topic

from .profile import Deployment

SITE_EVENTS_TOPIC = '/rescue/site_events'
SYSTEM_STATE_TOPIC = '/rescue/system_state'
TARGETS_TOPIC = '/rescue/targets'
EXIT_LOCATION_TOPIC = '/rescue/exit_location'  # QR exit (통합-인식 안 「나」, OPS-144)
OPERATOR_COMMAND_SERVICE = '/rescue/operator_command'
PASSAGE_APPROVE_SERVICE = '/rescue/passage_keepout/approve'  # rescue_robot passage_keepout.APPROVE_SERVICE (#406)
FROZEN_MAP_TOPIC = '/rescue/map/frozen'               # map_freeze (rescue_robot), as the coordinator subscribes
FROZEN_REVISION_TOPIC = '/rescue/map/frozen_revision'
REPORTS_TOPIC = '/rescue/reports'  # IF-072, coordinator -> report_node
REPORT_RECEIPTS_TOPIC = '/rescue/report_receipts'  # IF-073, report_node -> coordinator
ROBOT_PREFIX = '/rescue/robots/'
# The command API has no login of its own; the console on the same PC checks ADMIN (#125).
API_HOST = '127.0.0.1'
CONSOLE_TIMEOUT_S = 5.0  # the console gives up after this (#115); the API must answer first
# Stable camera names (the /dev/videoN number follows plug order). Only *-video-index0 gives frames.
CAMERA_BY_ID_DIR = '/dev/v4l/by-id'


class LaunchSettingError(ValueError):
    """A launch argument or profile that cannot start the operations PC."""


def parse_launch_arguments(*, db_path, api_port, command_timeout_s):
    """Launch arguments arrive as text; parse them strictly (no silent defaults)."""
    try:
        port = int(api_port)
        timeout = float(command_timeout_s)
    except ValueError as exc:
        raise LaunchSettingError(f'api_port must be an integer and command_timeout_s a number: {exc}') from exc
    if not math.isfinite(timeout):
        raise LaunchSettingError('command_timeout_s must be finite')
    return dict(db_path=db_path, api_port=port, command_timeout_s=timeout)


def db_writer_parameters(deployment, *, db_path, api_port, command_timeout_s, webcam_node, map_topic, map_dir):
    """Profile + launch values -> every parameter rescue_operations db_writer requires.

    map_topic/map_dir are the launch's SLAM map arguments (map_snapshot_parameters): with them db_writer also
    keeps the run's maps for replay under map_dir/<run_id>/ (초기화·리플레이 3단계); '' for both is off."""
    if not isinstance(deployment, Deployment) or deployment.mission is None:
        raise LaunchSettingError('profile v2 with mission roles is required (config/deployment.v2.example.json)')
    if not isinstance(db_path, str) or not db_path:
        raise LaunchSettingError('db_path is required')
    if type(api_port) is not int or not 1 <= api_port <= 65535:
        raise LaunchSettingError('api_port must be 1..65535')
    if not isinstance(command_timeout_s, float) or not 0.0 < command_timeout_s < CONSOLE_TIMEOUT_S:
        raise LaunchSettingError(f'command_timeout_s must be over 0 and under the console {CONSOLE_TIMEOUT_S:g} s')
    map_snapshot_parameters(map_topic=map_topic, map_dir=map_dir)  # both or neither, absolute topic
    robots = [robot.id for robot in deployment.robots]
    (slam_robot,) = [robot for robot in robots if deployment.mission.role_of(robot) == 'SCOUT']
    return {
        'db_path': db_path,
        'allowed_robot_ids': robots,
        'robot_roles': [deployment.mission.role_of(robot) for robot in robots],
        'status_topics': [ROBOT_PREFIX + robot + '/status' for robot in robots],
        'pose_topics': [ROBOT_PREFIX + robot + '/pose' for robot in robots],
        'overhead_passages_topics': [overhead_passages_topic(robot) for robot in robots],  # C-04b (OPS-194)
        # passages of both robots are measured on the SLAM robot's (SCOUT) map: its MapState is the current map
        'map_state_topic': ROBOT_PREFIX + slam_robot + '/map_state',
        'site_events_topic': SITE_EVENTS_TOPIC,
        'system_state_topic': SYSTEM_STATE_TOPIC,
        'targets_topic': TARGETS_TOPIC,
        'exit_location_topic': EXIT_LOCATION_TOPIC,
        'api_host': API_HOST,
        'api_port': api_port,
        'operator_command_service': OPERATOR_COMMAND_SERVICE,
        'passage_approve_service': PASSAGE_APPROVE_SERVICE,  # the console's tunnel pass approval (I-8)
        'command_timeout_s': command_timeout_s,
        # the console's camera pick goes to this webcam_node ('' = no webcam in this launch, OPS-183)
        'webcam_node': webcam_node,
        'map_archive_topic': map_topic,
        'map_archive_dir': map_dir,
        'frozen_map_topic': FROZEN_MAP_TOPIC,
        'frozen_revision_topic': FROZEN_REVISION_TOPIC,
    }


def report_node_parameters(*, db_path, report_db_path, envelope_schema_version, publisher_id):
    """Launch values -> every parameter rescue_operations report_node (#137) requires.

    The report store is its own SQLite file: one writing process per file, so it must not
    be the db_writer site-state file. Every value is text (STRING parameters).
    """
    for name, value in (('report_db_path', report_db_path), ('envelope_schema_version', envelope_schema_version),
                        ('publisher_id', publisher_id)):
        if not isinstance(value, str) or not value:
            raise LaunchSettingError(f'{name} is required')
    if os.path.realpath(report_db_path) == os.path.realpath(db_path):
        raise LaunchSettingError('report_db_path must be another file than the site-state db_path')
    return {
        'db_path': report_db_path,
        'reports_topic': REPORTS_TOPIC,
        'receipts_topic': REPORT_RECEIPTS_TOPIC,
        'envelope_schema_version': envelope_schema_version,
        'publisher_id': publisher_id,
    }


def read_webcam_parameters(path):
    """The webcam_node parameters YAML (the file webcam_node itself reads) -> its ros__parameters."""
    try:
        import yaml
        with open(path, encoding='utf-8') as handle:
            values = yaml.safe_load(handle)['webcam_node']['ros__parameters']
    except (ImportError, OSError, KeyError, TypeError, ValueError) as exc:
        raise LaunchSettingError(f'cannot read webcam parameters {path}: {type(exc).__name__}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise LaunchSettingError(f'cannot read webcam parameters {path}: invalid YAML: {exc}') from exc
    if not isinstance(values, dict):
        raise LaunchSettingError(f'{path}: webcam_node.ros__parameters must be a mapping')
    return values


def available_cameras(by_id_dir=CAMERA_BY_ID_DIR):
    """Plugged cameras that can be opened for frames (index1 is the same camera's metadata node)."""
    try:
        return sorted(name for name in os.listdir(by_id_dir) if name.endswith('-video-index0'))
    except OSError:
        return []


def webcam_device(device, *, by_id_dir=CAMERA_BY_ID_DIR):
    """webcam_device:= -> the device_path that overrides the webcam YAML; '' keeps the YAML's camera.

    A bare name is looked up under ``by_id_dir`` (what `ls /dev/v4l/by-id/` prints). A camera that
    is not plugged in is refused here with the plugged ones listed, instead of webcam_node retrying
    a missing device forever.
    """
    if device == '':
        return ''
    path = device if os.path.isabs(device) else os.path.join(by_id_dir, device)
    if not os.path.exists(path):
        raise LaunchSettingError(f'webcam_device {device} is not plugged in; plugged: '
                                 + (', '.join(available_cameras(by_id_dir)) or 'none'))
    return path


def relay_requested(*, video_port, webcam_params_file):
    """The video relay is on only with a video_port, and only with the webcam on this PC."""
    if video_port == '':
        return False
    if webcam_params_file == '':
        raise LaunchSettingError('video_port needs webcam_params_file: the relay forwards this PC\'s webcam video')
    return True


def mjpeg_relay_parameters(*, webcam, bind_host, port, stale_s):
    """Webcam settings + text launch arguments -> every parameter rescue_perception mjpeg_relay (#153) requires.

    The input is the webcam judge video (``debug_image_topic``); the URL path is
    ``/<camera_id>.mjpg`` so the console ``--camera <camera_id>=<url>`` names the same camera.
    """
    topic, camera_id = webcam.get('debug_image_topic'), webcam.get('camera_id')
    if not isinstance(topic, str) or not topic:
        raise LaunchSettingError('webcam debug_image_topic is off or missing: no judge video to relay')
    if not isinstance(camera_id, str) or not camera_id:
        raise LaunchSettingError('webcam camera_id is required')
    if not isinstance(bind_host, str) or not bind_host:
        raise LaunchSettingError('video_bind_host is required with video_port')
    try:
        port, stale = int(port), float(stale_s)
    except ValueError as exc:
        raise LaunchSettingError(f'video_port must be an integer and video_stale_s a number: {exc}') from exc
    if not 1 <= port <= 65535:
        raise LaunchSettingError('video_port must be 1..65535')
    if not math.isfinite(stale) or stale <= 0:
        raise LaunchSettingError('video_stale_s must be finite and positive')
    return {'image_topic': topic, 'bind_host': bind_host, 'port': port, 'path': f'/{camera_id}.mjpg',
            'stale_s': stale}


# absolute ROS namespace: /name(/name)*, each name a letter then letters, digits or _ (no trailing /)
ROS_NAMESPACE = re.compile(r'(/[A-Za-z][A-Za-z0-9_]*)+')


def robot_watch_parameters(*, robot_namespaces, robot_watch_path, robot_ids):
    """robot_namespaces('robot_a=/robot6,robot_b=/robot5') + robot_watch_path -> rescue_operations robot_watch
    parameters, or None when both are empty. 구독만 하는 직접 관측(재범 허용 2026-10-13); 관제 웹 --robot-watch."""
    if robot_namespaces == '' and robot_watch_path == '':
        return None
    if robot_namespaces == '' or robot_watch_path == '':
        raise LaunchSettingError('robot_namespaces and robot_watch_path go together (console --robot-watch)')
    ids, namespaces = [], []
    for pair in robot_namespaces.split(','):
        robot_id, sep, ns = pair.strip().partition('=')
        if (not sep or robot_id not in robot_ids or robot_id in ids or ns in namespaces
                or not ROS_NAMESPACE.fullmatch(ns)):
            raise LaunchSettingError(f'robot_namespaces item {pair!r}: want <profile robot id>=/<namespace>, '
                                     f'once per robot (profile robots {list(robot_ids)})')
        ids.append(robot_id)
        namespaces.append(ns)
    return {'robot_ids': ids, 'namespaces': namespaces, 'out_path': robot_watch_path, 'map_frame': 'map',
            'base_frame': 'base_link'}


ROBOT_VIEW_TOPIC = '/rescue/passage_view/compressed'   # 통합-인식 판정 그린 영상, <ns> 아래(OPS-189)


def robot_view_default(deployment: Deployment):
    """로봇 카메라 칸의 기본 로봇 = SUPPORT(TB-5). 웹에는 TB-5 터널 판정 영상을 띄운다(재범 결정 2026-10-14)."""
    return next(robot.id for robot in deployment.robots if deployment.mission.role_of(robot.id) == 'SUPPORT')


def robot_view_relay_parameters(*, robot_views, bind_host, port, stale_s, robot_ids, default_robot):
    """robot_views('robot_b=/robot5' 또는 'robot_a=/robot6,robot_b=/robot5') -> rescue_perception robot_view_relay
    parameters, or None when empty. 고른 로봇 하나만 중계(OPS-191); 포트는 video_port + 1, 기본은
    ``default_robot``(``robot_view_default``, SUPPORT = TB-5)."""
    if robot_views == '':
        return None
    sources, ids = [], []
    for pair in robot_views.split(','):
        robot_id, sep, ns = pair.strip().partition('=')
        if not sep or robot_id not in robot_ids or robot_id in ids or not ROS_NAMESPACE.fullmatch(ns):
            raise LaunchSettingError(f'robot_views item {pair!r}: want <profile robot id>=/<namespace>, once per robot')
        ids.append(robot_id)
        sources.append(f'{robot_id}={ns}{ROBOT_VIEW_TOPIC}')
    if default_robot not in ids:
        raise LaunchSettingError(f'robot_views must include the default robot {default_robot!r} (SUPPORT, TB-5)')
    try:
        view_port, stale = int(port) + 1, float(stale_s)
    except ValueError as exc:
        raise LaunchSettingError(f'robot_views needs video_port and video_stale_s: {exc}') from exc
    if not 1 <= view_port <= 65535 or not math.isfinite(stale) or stale <= 0 or not bind_host:
        raise LaunchSettingError('robot_views needs video_bind_host, video_port below 65535 and a positive video_stale_s')
    return {'sources': sources, 'default_source': default_robot, 'bind_host': bind_host, 'port': view_port,
            'path': '/robot_view.mjpg', 'stale_s': stale}


def map_snapshot_parameters(*, map_topic, map_dir):
    """map_topic + map_dir -> rescue_operations map_snapshot parameters, or None when both are empty.

    The SLAM map (owner 2026-10-13: TB-6) is written under map_dir for the console --live-map.
    The topic is an argument, not fixed: after the map is frozen it may come from another topic.
    """
    if map_topic == '' and map_dir == '':
        return None
    if map_topic == '' or map_dir == '':
        raise LaunchSettingError('map_topic and map_dir go together (the SLAM map for the console --live-map)')
    if not map_topic.startswith('/'):
        raise LaunchSettingError(f'map_topic must be an absolute topic name, got {map_topic!r}')
    return {'map_topic': map_topic, 'out_dir': map_dir}


def console_camera_argument(relay, *, camera_id):
    """The console --camera value for this relay."""
    return f'{camera_id}=http://{relay["bind_host"]}:{relay["port"]}{relay["path"]}'
