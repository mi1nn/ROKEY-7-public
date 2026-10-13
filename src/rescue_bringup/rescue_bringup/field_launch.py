"""Coordinator and robot launch values: profile + field run file + explicit arguments (no ROS import).

``launch/coordinator.launch.py`` and ``launch/robot.launch.py`` only wrap this module.

- The deployment profile (v2.1) gives robot IDs, namespaces, ROS names, readiness, search areas
  and the map basis (``map_basis``: mission frame, map session, map revision, transform
  revision), which the coordinator goals and the robot's saved-map alignment both use.
- The field run file (``config/field/*.field.json``, format ``rescue-field-run/1``) gives the run
  policy: the #132 field values, written once. Values the coordinator and the robot must share
  (lease, authority epoch, coordinator ID, envelope, clock) are written on both sides
  and checked equal here, so a field edit on one side fails at launch instead of at admission.
- Explicit launch arguments give what is not settled or is measured at the site: the Phase 1
  route/observe goals (#132 placeholders) and the measured occupancy hash. None has a default.
"""
import json
import math
import re
from pathlib import Path

from .profile import Deployment, ProfileError, require_map_basis

FORMAT = 'rescue-field-run/1'
MAX_FIELD_BYTES = 65_536
COORDINATOR_NODE = 'mission_coordinator'  # rescue_coordination ros_node Node name; live inputs at ~/task_inputs
TASK_INPUTS_SERVICE = '/' + COORDINATOR_NODE + '/task_inputs'
SYSTEM_STATE_TOPIC = '/rescue/system_state'
SHA256 = re.compile(r'[0-9a-f]{64}\Z')

S, F, I, B, SL, FL = 'str', 'float', 'int', 'bool', 'str_list', 'float_list'
COORDINATOR_KEYS = {
    'policy_version': S, 'executable_bases': SL, 'task_duration_s': F, 'lease_duration_s': F,
    'executor_policy_version': S, 'authority_epoch': S, 'coordinator_id': S,
    'envelope_schema_version': S, 'status_max_age_s': F, 'tick_period_s': F, 'route_revision': I,
    'merge_distance_m': F, 'report_retry_s': F, 'lease_renew_period_s': F, 'admission_timeout_s': F,
    'lease_message_ttl_s': F, 'live_inputs.enabled': B, 'live_inputs.clock_backend': S,
    'live_inputs.clock_domain_id': S, 'live_inputs.clock_max_age_ns': I, 'live_inputs.clock_drift_ppm': F,
    'live_inputs.poll_period_s': F, 'live_inputs.search_goal_topic': S,
    'camera_watch_timeout_s': F,  # SetCameraWatch answer wait (coordination C2)
    # TB-5 automatic dispatch (user decision 1B, 2026-10-13; coordination)
    'support_dispatch': S, 'approach_standoff_m': F, 'support_pose_max_age_s': F,
    # TargetLocation time checks (coordination OPS-103, 2026-10-13)
    'target_clock_domain_id': S, 'target_max_age_s': F, 'target_max_future_s': F,
}
SUPPORT_DISPATCH_MODES = ('AUTO', 'OPERATOR')
ROBOT_KEYS = {
    'authority_epoch': S, 'policy_version': S, 'step_period': F, 'clock_error_bound_s': F,
    **{'policy.' + name: F for name in ('acceptance_timeout', 'stop_timeout', 'stop_window', 'max_sample_gap',
                                        'evidence_max_age', 'linear_stop_max', 'angular_stop_max')},
    'readiness.max_age_s.watchdog_heartbeat': F,
    'task.runtime_factory': S, 'task.input_mode': S, 'task.schema_version': S,
    'task.coordinator_id': S, 'task.coordinator_state_topic': S, 'task.clock_error_bound_ns': I,
    'task.lease.duration_s': F, 'task.lease.max_age_ns': I, 'task.lease.max_error_ns': I,
    'task.lease.clock_domain_id': S, 'task.max_position_error_m': F, 'task.max_rotation_error_rad': F,
    'task.input_max_age_ns': I, 'task.poll_period_s': F, 'task.live.coordinator_service': S,
    'task.live.clock_backend': S, 'task.live.clock_max_age_ns': I, 'task.live.clock_drift_ppm': F,
    'task.saved_map.transform_xy_yaw': FL,
    'task.saved_map.position_error_m': F, 'task.saved_map.rotation_error_rad': F,
    'task.saved_map.max_position_variance_m2': F, 'task.saved_map.max_yaw_variance_rad2': F,
}
ROBOT_IDENTITY_KEYS = {'task.publisher_id': S, 'task.mapping_authority': S}
ALIGNMENT_KEYS = {'backdate_s': F, 'window_s': F}
WATCHDOG_KEYS = {'watchdog.version': S, 'step_period': F, 'watchdog.heartbeat_max_age_s': F,
                 'watchdog.command_max_age_s': F, 'watchdog.max_linear_mps': F, 'watchdog.max_angular_rps': F}
# (coordinator key, robot key): one value seen from both ends.
SHARED = (
    ('lease_duration_s', 'task.lease.duration_s'), ('authority_epoch', 'authority_epoch'),
    ('coordinator_id', 'task.coordinator_id'), ('envelope_schema_version', 'task.schema_version'),
    ('executor_policy_version', 'policy_version'),
    ('live_inputs.clock_domain_id', 'task.lease.clock_domain_id'),
    ('live_inputs.clock_backend', 'task.live.clock_backend'),
)


class FieldLaunchError(ValueError):
    """A field run file, profile or launch argument that cannot start the coordinator or a robot."""


class FieldRun:
    """A checked field run file; ``raw`` is the parsed JSON."""

    def __init__(self, raw):
        self.raw = raw
        self.coordinator = raw['coordinator']
        self.robot = raw['robot']
        self.robots = raw['robots']
        self.alignment = raw['alignment']
        self.watchdog = raw['watchdog']


def _typed(value, kind):
    if kind == S:
        return isinstance(value, str) and value != ''
    if kind == F:
        return type(value) is float and math.isfinite(value)
    if kind == I:
        return type(value) is int
    if kind == B:
        return type(value) is bool
    if kind == SL:
        return isinstance(value, list) and all(isinstance(v, str) and v for v in value)
    return isinstance(value, list) and all(type(v) is float and math.isfinite(v) for v in value)


def _section(raw, name, keys):
    section = raw.get(name)
    if not isinstance(section, dict) or set(section) != set(keys):
        given = set(section) if isinstance(section, dict) else set()
        raise FieldLaunchError(f'{name}: keys must be exactly the expected ones; '
                               f'missing {sorted(set(keys) - given)}, unknown {sorted(given - set(keys))}')
    for key, kind in keys.items():
        if not _typed(section[key], kind):
            raise FieldLaunchError(f'{name}.{key} must be {kind} (finite, non-empty)')
    return section


def load_field(path):
    """Read and check a field run file; every key is required and typed (no defaults)."""
    path = Path(path)
    try:
        text = path.read_bytes()
    except OSError as exc:
        raise FieldLaunchError(f'cannot read field run file {path}: {exc}') from exc
    if len(text) > MAX_FIELD_BYTES:
        raise FieldLaunchError('field run file too large')
    try:
        raw = json.loads(text, parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    except ValueError as exc:
        raise FieldLaunchError(f'field run file is not strict JSON: {exc}') from exc
    if not isinstance(raw, dict) or set(raw) != {'format', 'source', 'coordinator', 'robot', 'robots',
                                                 'alignment', 'watchdog'}:
        raise FieldLaunchError('field run file needs exactly format, source, coordinator, robot, robots, '
                               'alignment, watchdog')
    if raw['format'] != FORMAT:
        raise FieldLaunchError(f'format must be {FORMAT}')
    if not isinstance(raw['source'], str) or not raw['source']:
        raise FieldLaunchError('source must say where the values come from')
    coordinator = _section(raw, 'coordinator', COORDINATOR_KEYS)
    robot = _section(raw, 'robot', ROBOT_KEYS)
    _section(raw, 'alignment', ALIGNMENT_KEYS)
    _section(raw, 'watchdog', WATCHDOG_KEYS)
    if not isinstance(raw['robots'], dict) or not raw['robots']:
        raise FieldLaunchError('robots must name at least one robot')
    for robot_id in raw['robots']:
        _section(raw['robots'], robot_id, ROBOT_IDENTITY_KEYS)
    for left, right in SHARED:
        if coordinator[left] != robot[right]:
            raise FieldLaunchError(f'coordinator.{left} ({coordinator[left]!r}) and robot.{right} '
                                   f'({robot[right]!r}) must be equal')
    if not coordinator['live_inputs.enabled'] or robot['task.input_mode'] != 'LIVE_ROS':
        raise FieldLaunchError('this launch runs live inputs: live_inputs.enabled true and task.input_mode LIVE_ROS')
    if robot['task.live.coordinator_service'] != TASK_INPUTS_SERVICE:
        raise FieldLaunchError(f'robot.task.live.coordinator_service must be {TASK_INPUTS_SERVICE} '
                               f'(the coordinator node this launch starts)')
    if coordinator['support_dispatch'] not in SUPPORT_DISPATCH_MODES:
        raise FieldLaunchError(f'coordinator.support_dispatch must be one of {list(SUPPORT_DISPATCH_MODES)}')
    for key in ('approach_standoff_m', 'support_pose_max_age_s', 'target_max_age_s'):
        if coordinator[key] <= 0:
            raise FieldLaunchError(f'coordinator.{key} must be positive')
    if coordinator['target_max_future_s'] < 0:
        raise FieldLaunchError('coordinator.target_max_future_s must be zero or positive')
    if coordinator['target_clock_domain_id'] != coordinator['live_inputs.clock_domain_id']:
        raise FieldLaunchError('coordinator.target_clock_domain_id must be the coordinator clock domain '
                               '(live_inputs.clock_domain_id)')
    if robot['task.coordinator_state_topic'] != SYSTEM_STATE_TOPIC:
        raise FieldLaunchError(f'robot.task.coordinator_state_topic must be {SYSTEM_STATE_TOPIC}')
    if len(robot['task.saved_map.transform_xy_yaw']) != 3:
        raise FieldLaunchError('robot.task.saved_map.transform_xy_yaw must be [x, y, yaw]')
    return FieldRun(raw)


def _list_argument(text, name, kind):
    """A launch argument such as '[0.0, 0.0, 0.0]' or '[false]' -> list; strict JSON, no NaN."""
    try:
        value = json.loads(text, parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    except (TypeError, ValueError) as exc:
        raise FieldLaunchError(f'{name} must be a JSON list, e.g. [0.0, 0.0, 0.0]: {text!r}') from exc
    if not isinstance(value, list):
        raise FieldLaunchError(f'{name} must be a JSON list: {text!r}')
    if kind == B:
        if not all(type(v) is bool for v in value):
            raise FieldLaunchError(f'{name} must hold true/false only')
        return value
    if not all(type(v) in (int, float) and math.isfinite(v) for v in value):
        raise FieldLaunchError(f'{name} must hold finite numbers only')
    return [float(v) for v in value]


def _deployment(deployment):
    if not isinstance(deployment, Deployment) or deployment.mission is None:
        raise FieldLaunchError('profile v2 with mission roles is required (config/deployment.v2.example.json)')
    try:
        return deployment, require_map_basis(deployment)
    except ProfileError as exc:
        raise FieldLaunchError(str(exc)) from exc


def coordinator_parameters(deployment, *, profile_path, field, route_segment_goals, observe_goal,
                           route_segment_passage_free, camera_watch_roi=''):
    """Profile + field file + placeholder goals (text launch arguments) -> mission_coordinator parameters.

    ``camera_watch_roi`` ('[x0, y0, x1, y1]', set at the site) is optional: empty leaves the
    parameter unset and the coordinator refuses camera watch requests (no whole-frame default).
    """
    deployment, basis = _deployment(deployment)
    yaws = [area.entry_yaw for area in deployment.mission.search_areas]
    if any(yaw is None for yaw in yaws):
        raise FieldLaunchError('every profile search_areas[].entry_yaw is required (profile v2.1)')
    goals = _list_argument(route_segment_goals, 'route_segment_goals', F)
    if not goals or len(goals) % 3:
        raise FieldLaunchError('route_segment_goals must be [x, y, yaw] * N, N >= 1')
    observe = _list_argument(observe_goal, 'observe_goal', F)
    if len(observe) != 3:
        raise FieldLaunchError('observe_goal must be [x, y, yaw]')
    passage = _list_argument(route_segment_passage_free, 'route_segment_passage_free', B)
    if not passage:
        # The coordinator accepts an empty list, but ROS cannot type an empty array parameter:
        # launch_ros refuses it and --params-file / -p leave it unset (measured 2026-10-13).
        raise FieldLaunchError('route_segment_passage_free cannot be [] through ROS parameters; '
                               'give one true/false per segment end')
    if not profile_path:
        raise FieldLaunchError('profile_path is required')
    roi = {}
    if camera_watch_roi != '':
        roi['camera_watch_roi'] = _list_argument(camera_watch_roi, 'camera_watch_roi', F)
        if len(roi['camera_watch_roi']) != 4:
            raise FieldLaunchError('camera_watch_roi must be 4 numbers')
    return {**roi, 'profile_path': str(profile_path), 'entry_yaws': [float(yaw) for yaw in yaws], **field.coordinator,
            'mission_frame_id': basis.frame_id, 'goal_map_session_id': basis.map_session_id,
            'goal_map_revision': basis.map_revision, 'goal_transform_revision': basis.transform_revision,
            'route_segment_goals': goals, 'observe_goal': observe, 'route_segment_passage_free': passage}


def robot_launch_values(deployment, *, field, robot_id, occupancy_sha256, now_ns):
    """Profile + field file + measured map hash -> task_input and command_watchdog parameters.

    ``now_ns`` is the launch wall time: the declared alignment window (#132 start_robot.sh)
    runs from ``now - backdate_s`` to ``now + window_s``. It is a declared session window,
    not an observation.
    """
    deployment, basis = _deployment(deployment)
    robots = {robot.id: robot for robot in deployment.robots}
    if robot_id not in robots:
        raise FieldLaunchError(f'robot_id {robot_id!r} is not in the profile ({sorted(robots)})')
    if robot_id not in field.robots:
        raise FieldLaunchError(f'field run file has no robots.{robot_id} (task.publisher_id, task.mapping_authority)')
    if not isinstance(occupancy_sha256, str) or not SHA256.match(occupancy_sha256):
        raise FieldLaunchError('occupancy_sha256 must be the measured 64 lowercase hex map hash')
    if type(now_ns) is not int or now_ns <= 0:
        raise FieldLaunchError('now_ns must be a positive wall-clock time')
    robot = robots[robot_id].parameters
    readiness = robot.readiness
    if readiness is None or robot.nav_action is None:
        raise FieldLaunchError('profile v2 robot parameters with readiness are required')
    if readiness.map_frame != basis.frame_id:
        raise FieldLaunchError(f'robot {robot_id} readiness.map_frame {readiness.map_frame!r} must be the '
                               f'map_basis frame {basis.frame_id!r} (identity alignment is declared)')
    if not readiness.camera_topics:
        raise FieldLaunchError(f'robot {robot_id} readiness.camera_topics is empty: ROS cannot pass an empty '
                               f'array parameter, so task_input would see it as missing')
    ns = robot.namespace
    executor_heartbeat, watchdog_heartbeat = ns + '/rescue/executor_heartbeat', ns + '/rescue/watchdog_heartbeat'
    backdate_ns = round(field.alignment['backdate_s'] * 1e9)
    window_ns = round(field.alignment['window_s'] * 1e9)
    if backdate_ns < 0 or window_ns <= 0:
        raise FieldLaunchError('alignment.backdate_s must be >= 0 and alignment.window_s > 0')
    task_input = {
        'allowed_robot_ids': [r.id for r in deployment.robots], 'robot_id': robot_id, 'namespace': ns,
        'nav_action': robot.nav_action, 'odom_topic': robot.odom_topic, 'dock_status_topic': robot.dock_status_topic,
        'scan_topic': readiness.scan_topic, 'tf_topic': readiness.tf_topic, 'map_frame': readiness.map_frame,
        'odom_frame': readiness.odom_frame, 'readiness.version': readiness.version,
        **{'readiness.max_age_s.' + source: seconds for source, seconds in readiness.max_age_s},
        'camera_topics': list(readiness.camera_topics), 'wheel_status_topic': readiness.wheel_status_topic or '',
        'executor_heartbeat_topic': executor_heartbeat, 'watchdog_heartbeat_topic': watchdog_heartbeat,
        **field.robot, **field.robots[robot_id], 'task.mission_frame': basis.frame_id,
        'task.saved_map.map_session_id': basis.map_session_id, 'task.saved_map.map_revision': basis.map_revision,
        'task.saved_map.transform_revision': basis.transform_revision,
        'task.saved_map.map_topic': ns + '/map', 'task.saved_map.localization_topic': ns + '/amcl_pose',
        'task.saved_map.map_frame': readiness.map_frame, 'task.saved_map.occupancy_sha256': occupancy_sha256,
        'task.saved_map.valid_from_ns': now_ns - backdate_ns,
        'task.saved_map.transform_valid_until_ns': now_ns + window_ns,
    }
    upstream = ns + '/cmd_vel_watchdog_in'
    watchdog = {'namespace': ns, 'cmd_vel_in_topic': upstream, 'cmd_vel_out_topic': ns + '/cmd_vel',
                'cmd_vel_stamped': True, 'executor_heartbeat_topic': executor_heartbeat,
                'watchdog_heartbeat_topic': watchdog_heartbeat, **field.watchdog}
    # Nav2's final velocity writers go to the watchdog, which alone writes <ns>/cmd_vel (#132).
    remaps = [('collision_monitor:' + ns + '/cmd_vel', upstream), ('docking_server:' + ns + '/cmd_vel', upstream)]
    return {'namespace': ns, 'use_sim_time': robot.use_sim_time, 'task_input': task_input, 'watchdog': watchdog,
            'nav2_remaps': remaps}
