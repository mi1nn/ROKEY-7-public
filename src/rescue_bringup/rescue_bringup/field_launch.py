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
    # TB-5 waits at the target (APPROACH_TARGET completed, stop confirmed) before leaving for the exit
    'support_exit_wait_s': F,
    # Recheck -> rescue verdict: largest clock error of a recheck observation the coordinator accepts
    'recheck_max_clock_error_s': F,
    # SLAM map frozen before TB-5 goes (owner decision 2026-10-13, Q-29 ⑦; coordination OPS-119)
    'map_mode': S, 'freeze_clearance_m': F, 'map_freeze_timeout_s': F,
    # TB-6 search starts on the disaster sign (owner decision 2026-10-13 evening, OPS-136)
    'search_start': S,
    # TB-6 search (OPS-136·139): PATROL (points in TB-6's start pose) | AREAS; TB-5 exit approach standoff
    'search_mode': S, 'patrol_points': FL, 'patrol_rounds': I, 'exit_standoff_m': F,
    # FRONTIER exploration (owner decision 2026-10-13 night, OPS-154·156)
    'frontier_min_cells': I, 'frontier_reach_m': F, 'frontier_goal_max_m': F, 'frontier_blacklist_m': F,
    'frontier_issue_delay_s': F, 'exploration_empty_checks': I, 'exploration_timeout_s': F,
}
SUPPORT_DISPATCH_MODES = ('AUTO', 'OPERATOR')
SEARCH_START_MODES = ('AUTO', 'OPERATOR')  # rescue_coordination.ros_node.SEARCH_START_MODES
SEARCH_MODES = ('PATROL', 'AREAS', 'FRONTIER')  # rescue_coordination.ros_node.SEARCH_MODES
MAP_MODES = ('SLAM_FREEZE', 'SAVED_MAP')  # rescue_coordination.ros_node.MAP_MODES
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
    # ObserveTarget after a REOBSERVE_TARGET stop (C1-b, 2026-10-13)
    'task.live.recheck.window_ns': I, 'task.live.recheck.deadline_ns': I,
    'task.live.pose_valid_ns': I,  # RobotPose validity after the AMCL stamp (live_pose, 2026-10-13)
    # Overhead passage stop, the same for every robot (owner decisions 2026-10-13: 46 cm, scope Q-25;
    # both robots, OPS-135). robot.launch.py adds it to the Nav2 collision_monitor (nav2_with_passage).
    'passage.min_height_m': F, 'passage.max_height_m': F, 'passage.front_m': FL, 'passage.half_width_m': F,
    'passage.min_points': I,
    'task.live.passage.polygon_name': S, 'task.live.passage.result_window_ns': I,
}
# Per robot: identity, automatic target policy, and the Pi clock behind its AMCL scan stamps.
# Pi bound -1 is not measured and keeps RobotPose UNKNOWN (decision 2026-10-13).
ROBOT_IDENTITY_KEYS = {'task.publisher_id': S, 'task.mapping_authority': S,
                       'task.live.auto_target.enabled': B, 'task.live.auto_target.kinds': SL,
                       'task.live.auto_target.goal_clear_radius_m': F, 'task.live.auto_target.policy_version': S,
                       'task.live.sensor_host_clock_domain_id': S, 'task.live.sensor_host_clock_error_bound_ns': I}
# Map lifecycle (scope Q-29 ⑦, OPS-113/124/126): every robot names its map mode; the SLAM robot
# also the freeze pose age, a FROZEN_MAP_AMCL robot its measured AMCL start (once measured).
ROBOT_IDENTITY_KEYS['task.saved_map.mode'] = S
ROBOT_MAP_MODES = ('SAVED_MAP_AMCL', 'SLAM_THEN_FROZEN', 'FROZEN_MAP_AMCL')
SLAM_ROBOT_KEYS = {'map_freeze.pose_max_age_ns': I}     # TB-6 SLAM pose freshness at the freeze
FROZEN_ROBOT_OPTIONAL_KEYS = {'initial_pose_xy_yaw': FL}  # scripts/field/capture_initial_pose.py
# This robot's depth image for the passage stop (all three, or none until measured: then it does not start)
PASSAGE_DEPTH_KEYS = {'passage.depth_topic': S, 'passage.camera_info_topic': S, 'passage.depth_transport': S}
DEPTH_TRANSPORTS = ('raw', 'compressedDepth')
PASSAGE_POINTS = '/passage_points'   # <ns> + this: depth_image_proc output, collision_monitor input
FROZEN_TOPIC = '/rescue/map/frozen'                     # rescue_robot map_freeze / live_parameters
FROZEN_POSE_TOPIC = '/rescue/map/frozen_pose/'          # + SLAM robot id
START_POSE_TOPIC = '/rescue/map/start_pose/'            # + SLAM robot id: origin of coordinates measured in advance
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
    robot_modes = _robot_map_modes(raw['robots'])
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
    if coordinator['search_mode'] not in SEARCH_MODES:
        raise FieldLaunchError(f'coordinator.search_mode must be one of {list(SEARCH_MODES)}')
    if len(coordinator['patrol_points']) % 3:
        raise FieldLaunchError('coordinator.patrol_points must be [x, y, yaw] * N')
    if coordinator['search_mode'] == 'PATROL':
        if not coordinator['patrol_points']:
            raise FieldLaunchError('coordinator.search_mode PATROL needs patrol_points')
        if coordinator['live_inputs.search_goal_topic']:
            raise FieldLaunchError('coordinator.search_mode PATROL replaces the RViz search goal pick: '
                                   'live_inputs.search_goal_topic must be empty')
        if 'PATROL_POINT' not in coordinator['executable_bases']:
            raise FieldLaunchError('coordinator.search_mode PATROL needs PATROL_POINT in executable_bases')
    if coordinator['search_mode'] == 'FRONTIER':
        if coordinator['live_inputs.search_goal_topic']:
            raise FieldLaunchError('coordinator.search_mode FRONTIER replaces the RViz search goal pick: '
                                   'live_inputs.search_goal_topic must be empty')
        if 'FRONTIER' not in coordinator['executable_bases']:
            raise FieldLaunchError('coordinator.search_mode FRONTIER needs FRONTIER in executable_bases')
    for key in ('frontier_min_cells', 'exploration_empty_checks'):
        if coordinator[key] < 1:
            raise FieldLaunchError(f'coordinator.{key} must be 1 or more')
    if coordinator['frontier_issue_delay_s'] < coordinator['live_inputs.poll_period_s']:
        # the robot reads ~/task_inputs every poll period; a frontier task before that is refused (OPS-154 ①)
        raise FieldLaunchError('coordinator.frontier_issue_delay_s must be at least live_inputs.poll_period_s')
    if coordinator['patrol_rounds'] < 1:
        raise FieldLaunchError('coordinator.patrol_rounds must be 1 or more')
    if coordinator['search_start'] not in SEARCH_START_MODES:
        raise FieldLaunchError(f'coordinator.search_start must be one of {list(SEARCH_START_MODES)}')
    if coordinator['map_mode'] not in MAP_MODES:
        raise FieldLaunchError(f'coordinator.map_mode must be one of {list(MAP_MODES)}')
    _pair_map_modes(coordinator['map_mode'], robot_modes)
    for robot_id, identity in raw['robots'].items():
        # The coordinator checks the approach point and homes on the frozen map with the radius each robot checks
        # its goals with; a different one passes points the robot then refuses (작전 OPS-116).
        if identity['task.live.auto_target.goal_clear_radius_m'] != coordinator['freeze_clearance_m']:
            raise FieldLaunchError(f'coordinator.freeze_clearance_m must equal robots.{robot_id}.'
                                   'task.live.auto_target.goal_clear_radius_m')
    for key in ('approach_standoff_m', 'support_pose_max_age_s', 'target_max_age_s', 'recheck_max_clock_error_s',
                'freeze_clearance_m', 'map_freeze_timeout_s', 'exit_standoff_m', 'frontier_reach_m',
                'frontier_goal_max_m', 'frontier_blacklist_m', 'exploration_timeout_s'):
        if coordinator[key] <= 0:
            raise FieldLaunchError(f'coordinator.{key} must be positive')
    if coordinator['support_exit_wait_s'] < 0:
        raise FieldLaunchError('coordinator.support_exit_wait_s must be zero or positive')
    if coordinator['target_max_future_s'] < 0:
        raise FieldLaunchError('coordinator.target_max_future_s must be zero or positive')
    if coordinator['target_clock_domain_id'] != coordinator['live_inputs.clock_domain_id']:
        raise FieldLaunchError('coordinator.target_clock_domain_id must be the coordinator clock domain '
                               '(live_inputs.clock_domain_id)')
    if robot['task.coordinator_state_topic'] != SYSTEM_STATE_TOPIC:
        raise FieldLaunchError(f'robot.task.coordinator_state_topic must be {SYSTEM_STATE_TOPIC}')
    if len(robot['task.saved_map.transform_xy_yaw']) != 3:
        raise FieldLaunchError('robot.task.saved_map.transform_xy_yaw must be [x, y, yaw]')
    front = robot['passage.front_m']
    if not (0 <= robot['passage.min_height_m'] < robot['passage.max_height_m']):
        raise FieldLaunchError('robot.passage heights must be 0 <= min_height_m < max_height_m')
    if len(front) != 2 or not 0 <= front[0] < front[1]:
        raise FieldLaunchError('robot.passage.front_m must be [near, far] with 0 <= near < far')
    if robot['passage.half_width_m'] <= 0 or robot['passage.min_points'] < 1:
        raise FieldLaunchError('robot.passage.half_width_m must be positive and passage.min_points at least 1')
    if robot['task.live.passage.result_window_ns'] <= 0:
        raise FieldLaunchError('robot.task.live.passage.result_window_ns must be positive')
    return FieldRun(raw)


def _robot_map_modes(robots):
    """robots.<id>: identity keys + map mode; one SLAM robot feeds the frozen-map robots; no mixing with saved maps."""
    modes = {}
    for robot_id, section in robots.items():
        mode = section.get('task.saved_map.mode') if isinstance(section, dict) else None
        if mode not in ROBOT_MAP_MODES:
            raise FieldLaunchError(f'robots.{robot_id}.task.saved_map.mode must be one of {list(ROBOT_MAP_MODES)}')
        keys = dict(ROBOT_IDENTITY_KEYS)
        if mode == 'SLAM_THEN_FROZEN':
            keys.update(SLAM_ROBOT_KEYS)
        if mode == 'FROZEN_MAP_AMCL' and 'initial_pose_xy_yaw' in section:
            keys.update(FROZEN_ROBOT_OPTIONAL_KEYS)
        if any(key in section for key in PASSAGE_DEPTH_KEYS):
            keys.update(PASSAGE_DEPTH_KEYS)
        _section(robots, robot_id, keys)
        if mode == 'SLAM_THEN_FROZEN' and section['map_freeze.pose_max_age_ns'] <= 0:
            raise FieldLaunchError(f'robots.{robot_id}.map_freeze.pose_max_age_ns must be positive')
        if 'initial_pose_xy_yaw' in section and len(section['initial_pose_xy_yaw']) != 3:
            raise FieldLaunchError(f'robots.{robot_id}.initial_pose_xy_yaw must be [x, y, yaw]')
        if 'passage.depth_transport' in section and section['passage.depth_transport'] not in DEPTH_TRANSPORTS:
            raise FieldLaunchError(f'robots.{robot_id}.passage.depth_transport must be one of {list(DEPTH_TRANSPORTS)}')
        modes[robot_id] = mode
    slam = [robot_id for robot_id, mode in modes.items() if mode == 'SLAM_THEN_FROZEN']
    frozen = [robot_id for robot_id, mode in modes.items() if mode == 'FROZEN_MAP_AMCL']
    if len(slam) > 1:
        raise FieldLaunchError(f'only one robot makes the map (SLAM_THEN_FROZEN): {slam}')
    if frozen and not slam:
        raise FieldLaunchError('FROZEN_MAP_AMCL needs the SLAM_THEN_FROZEN robot that makes the frozen map')
    if (slam or frozen) and 'SAVED_MAP_AMCL' in modes.values():
        raise FieldLaunchError('saved maps and the frozen SLAM map cannot be mixed in one run')
    return modes


def _pair_map_modes(map_mode, robot_modes):
    """One map decision (OPS-126): SLAM_FREEZE = one SLAM_THEN_FROZEN robot and the rest FROZEN_MAP_AMCL (at least
    one); SAVED_MAP = every robot SAVED_MAP_AMCL. Anything else starts nothing."""
    modes = sorted(robot_modes.values())
    if map_mode == 'SAVED_MAP':
        ok = set(modes) == {'SAVED_MAP_AMCL'}
    else:
        ok = modes.count('SLAM_THEN_FROZEN') == 1 and modes.count('FROZEN_MAP_AMCL') == len(modes) - 1 >= 1
    if not ok:
        raise FieldLaunchError(f'coordinator.map_mode {map_mode} does not match the robots\' task.saved_map.mode '
                               f'{robot_modes}: SLAM_FREEZE needs one SLAM_THEN_FROZEN robot and FROZEN_MAP_AMCL for '
                               'the others, SAVED_MAP needs SAVED_MAP_AMCL for every robot')


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
    if len(passage) != len(goals) // 3:
        # The coordinator holds every segment on a length mismatch (PASSAGE_ASSESSMENT_REQUIRED);
        # a mismatch is never meant, so refuse it here instead of at run time.
        raise FieldLaunchError(f'route_segment_passage_free needs one true/false per segment end: '
                               f'{len(goals) // 3} segments, {len(passage)} flags')
    if not profile_path:
        raise FieldLaunchError('profile_path is required')
    roi = {}
    if camera_watch_roi != '':
        roi['camera_watch_roi'] = _list_argument(camera_watch_roi, 'camera_watch_roi', F)
        if len(roi['camera_watch_roi']) != 4:
            raise FieldLaunchError('camera_watch_roi must be 4 numbers')
    homes = {}
    if deployment.home_positions:
        scout, support = (dict((role, robot) for robot, role in deployment.mission.roles)[r] for r in ('SCOUT', 'SUPPORT'))
        if deployment.home_of(scout) is None or deployment.home_of(support) is None:
            raise FieldLaunchError('profile home_positions needs both the SCOUT and the SUPPORT robot '
                                   '(home_poses = SCOUT [x, y, yaw] then SUPPORT); or leave the block out')
        homes['home_poses'] = [*deployment.home_of(scout), *deployment.home_of(support)]
    # An empty array cannot go through ROS parameters (measured 2026-10-13): no patrol points = left unset.
    coordinator = {k: v for k, v in field.coordinator.items() if not (k == 'patrol_points' and not v)}
    return {**roi, **homes, 'profile_path': str(profile_path), 'entry_yaws': [float(yaw) for yaw in yaws], **coordinator,
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
    mode = field.robots[robot_id]['task.saved_map.mode']
    if mode == 'SAVED_MAP_AMCL':
        if not isinstance(occupancy_sha256, str) or not SHA256.match(occupancy_sha256):
            raise FieldLaunchError('occupancy_sha256 must be the measured 64 lowercase hex map hash')
    elif occupancy_sha256 != '':
        raise FieldLaunchError(f'{robot_id} is {mode}: the map freeze decides the map hash; '
                               'give no occupancy_sha256')
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
    passage = _passage(field, robot_id, ns)
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
        # the passage depth image is a readiness camera too: lost depth = NOT_READY (OPS-115). Watched over the
        # transport the points use, so readiness does not pull the raw depth over Wi-Fi as well (REV-226-01)
        'camera_topics': list(dict.fromkeys([*readiness.camera_topics, passage['readiness_topic']])),
        'wheel_status_topic': readiness.wheel_status_topic or '',
        'executor_heartbeat_topic': executor_heartbeat, 'watchdog_heartbeat_topic': watchdog_heartbeat,
        **{k: v for k, v in field.robot.items() if not k.startswith('passage.')},
        **_task_keys(field.robots[robot_id]), 'task.mission_frame': basis.frame_id,
        'task.saved_map.map_session_id': basis.map_session_id, 'task.saved_map.map_revision': basis.map_revision,
        'task.saved_map.transform_revision': basis.transform_revision,
        'task.saved_map.map_topic': ns + '/map', 'task.saved_map.localization_topic': ns + '/amcl_pose',
        'task.saved_map.map_frame': readiness.map_frame, 'task.saved_map.occupancy_sha256': occupancy_sha256,
        'task.saved_map.valid_from_ns': now_ns - backdate_ns,
        'task.saved_map.transform_valid_until_ns': now_ns + window_ns,
    }
    if mode == 'SLAM_THEN_FROZEN':
        task_input['task.saved_map.slam_localization_topic'] = ns + '/pose'   # slam_toolbox pose while searching
    home = deployment.home_of(robot_id)  # RETURN_HOME destination; absent = this robot has no home
    # SLAM_FREEZE: the TB-5 exit is the QR exit TB-6 finds, sent by the coordinator as 'home/robot_b'
    # (OPS-139); a profile home here would be used until the first coordinator record.
    if home is not None and mode != 'FROZEN_MAP_AMCL':
        task_input['task.live.home_xy_yaw'] = list(home)
    upstream = ns + '/cmd_vel_watchdog_in'
    watchdog = {'namespace': ns, 'cmd_vel_in_topic': upstream, 'cmd_vel_out_topic': ns + '/cmd_vel',
                'cmd_vel_stamped': True, 'executor_heartbeat_topic': executor_heartbeat,
                'watchdog_heartbeat_topic': watchdog_heartbeat, **field.watchdog}
    # Nav2's final velocity writers go to the watchdog, which alone writes <ns>/cmd_vel (#132).
    remaps = [('collision_monitor:' + ns + '/cmd_vel', upstream), ('docking_server:' + ns + '/cmd_vel', upstream)]
    return {'namespace': ns, 'use_sim_time': robot.use_sim_time, 'task_input': task_input, 'watchdog': watchdog,
            'passage': passage,
            'nav2_remaps': remaps}


def _task_keys(section):
    """The task_input parameters of a robots.<id> section (map lifecycle keys stay in the launch)."""
    return {key: value for key, value in section.items() if key.startswith('task.')}


def _robot(deployment, field, robot_id):
    deployment, basis = _deployment(deployment)
    robots = {robot.id: robot for robot in deployment.robots}
    if robot_id not in robots:
        raise FieldLaunchError(f'robot_id {robot_id!r} is not in the profile ({sorted(robots)})')
    if robot_id not in field.robots:
        raise FieldLaunchError(f'field run file has no robots.{robot_id}')
    return basis, robots[robot_id].parameters, field.robots[robot_id]


def slam_launch_values(deployment, *, field, robot_id):
    """TB-6 SLAM while searching (turtlebot4_navigation slam.launch.py): only the SLAM_THEN_FROZEN robot."""
    _, robot, section = _robot(deployment, field, robot_id)
    if section['task.saved_map.mode'] != 'SLAM_THEN_FROZEN':
        raise FieldLaunchError(f'{robot_id} is {section["task.saved_map.mode"]}, not the SLAM robot')
    return {'namespace': robot.namespace, 'use_sim_time': robot.use_sim_time}


def frozen_localization_values(deployment, *, field, robot_id):
    """AMCL on the frozen map: TB-6 starts at its frozen pose, a FROZEN_MAP_AMCL robot at its measured start."""
    _, robot, section = _robot(deployment, field, robot_id)
    mode, ns = section['task.saved_map.mode'], robot.namespace
    if mode == 'SAVED_MAP_AMCL':
        raise FieldLaunchError(f'{robot_id} uses a saved map (SAVED_MAP_AMCL); no frozen-map localization')
    start = None
    if mode == 'FROZEN_MAP_AMCL':
        if 'initial_pose_xy_yaw' not in section:
            raise FieldLaunchError(f'robots.{robot_id}.initial_pose_xy_yaw is not measured yet: click the start on '
                                   f'the TB-6 map in a trial run, then python3 scripts/field/capture_initial_pose.py '
                                   f'{ns}/amcl_pose map and add its line (relative to the TB-6 start pose) to the '
                                   f'field run file')
        start = list(section['initial_pose_xy_yaw'])
    return {'namespace': ns, 'use_sim_time': robot.use_sim_time, 'map_topic': ns + '/map',
            'frozen_topic': FROZEN_TOPIC, 'initial_pose_xy_yaw': start,
            'initial_pose_topic': FROZEN_POSE_TOPIC + robot_id if mode == 'SLAM_THEN_FROZEN' else None,
            # initial_pose_xy_yaw is relative to the TB-6 start pose (OPS-141); the launch places it on the map
            'start_pose_topic': START_POSE_TOPIC + _slam_robot(field) if mode == 'FROZEN_MAP_AMCL' else None}


def _slam_robot(field):
    slam = [robot_id for robot_id, section in field.robots.items()
            if section['task.saved_map.mode'] == 'SLAM_THEN_FROZEN']
    if len(slam) != 1:
        raise FieldLaunchError('one SLAM_THEN_FROZEN robot is needed for the start pose')
    return slam[0]


def amcl_parameters(localization_yaml, *, initial_pose_xy_yaw, use_sim_time):
    """turtlebot4_navigation localization.yaml amcl values + the start pose (nav2 amcl set_initial_pose)."""
    try:
        params = dict(localization_yaml['amcl']['ros__parameters'])
    except (KeyError, TypeError):
        raise ValueError('localization parameters have no amcl.ros__parameters') from None
    if (len(initial_pose_xy_yaw) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                                 and math.isfinite(v) for v in initial_pose_xy_yaw)):
        raise ValueError('AMCL start must be three finite numbers [x, y, yaw]')
    x, y, yaw = (float(v) for v in initial_pose_xy_yaw)
    params.update(use_sim_time=use_sim_time, set_initial_pose=True, initial_pose={'x': x, 'y': y, 'z': 0.0, 'yaw': yaw})
    return params


def map_freeze_parameters(deployment, *, field, output_dir, record_path):
    """map_freeze (compute_a): the SLAM robot's map and SLAM pose on the profile map basis (revision R0)."""
    deployment, basis = _deployment(deployment)
    slam = [robot_id for robot_id, section in field.robots.items()
            if section['task.saved_map.mode'] == 'SLAM_THEN_FROZEN']
    if not slam:
        raise FieldLaunchError('no SLAM_THEN_FROZEN robot in the field run file; nothing to freeze')
    _, robot, section = _robot(deployment, field, slam[0])
    for name, value in (('output_dir', output_dir), ('record_path', record_path)):
        if not isinstance(value, str) or (value and not value.startswith('/')) or (name == 'output_dir' and not value):
            raise FieldLaunchError(f'{name} must be an absolute path' + ('' if name == 'output_dir' else ' or empty'))
    params = {'map_topic': robot.namespace + '/map', 'pose_topic': robot.namespace + '/pose',
              'map_frame': basis.frame_id, 'map_session_id': basis.map_session_id,
              'map_revision': basis.map_revision, 'pose_max_age_ns': section['map_freeze.pose_max_age_ns'],
              'output_dir': output_dir}
    if record_path:
        params['record_path'] = record_path   # restart: republish the saved freeze
    return params


def _passage(field, robot_id, ns):
    """The shared passage stop values + this robot's depth image; no depth image = no start (OPS-137 A)."""
    section = field.robots[robot_id]
    if 'passage.depth_topic' not in section:
        raise FieldLaunchError(f'robots.{robot_id} has no passage depth image (passage.depth_topic, '
                               'passage.camera_info_topic, passage.depth_transport): without the overhead passage '
                               'check this robot does not start (OPS-137)')
    for key in ('passage.depth_topic', 'passage.camera_info_topic'):
        if not section[key].startswith(ns + '/'):
            raise FieldLaunchError(f'robots.{robot_id}.{key} must be a topic of this robot ({ns}/...)')
    r = field.robot
    return {'min_height_m': r['passage.min_height_m'], 'max_height_m': r['passage.max_height_m'],
            'front_m': list(r['passage.front_m']), 'half_width_m': r['passage.half_width_m'],
            'min_points': r['passage.min_points'], 'polygon_name': r['task.live.passage.polygon_name'],
            'depth_topic': section['passage.depth_topic'], 'camera_info_topic': section['passage.camera_info_topic'],
            'depth_transport': section['passage.depth_transport'], 'points_topic': ns + PASSAGE_POINTS,
            'readiness_topic': section['passage.depth_topic'] + (
                '/compressedDepth' if section['passage.depth_transport'] == 'compressedDepth' else '')}


def nav2_with_passage(params, passage):
    """Nav2 parameters + the passage stop polygon and depth source in collision_monitor (dicts, no YAML here).

    Existing polygons keep the sources they read before (sources_names); only the new polygon reads
    the depth points <ns>/passage_points in the height band. Anything unexpected is refused.
    """
    try:
        cm = params['collision_monitor']['ros__parameters']
        polygons, sources = list(cm['polygons']), list(cm['observation_sources'])
        cm['state_topic']
    except (KeyError, TypeError):
        raise ValueError('Nav2 parameters need collision_monitor polygons, observation_sources and state_topic '
                         '(the executor reads collision_monitor_state)') from None
    name, source = passage['polygon_name'], 'passage_depth'
    if name in polygons or name in cm or source in sources or source in cm:
        raise ValueError(f'Nav2 parameters already define {name} or {source}')
    for polygon in polygons:
        cm[polygon] = dict(cm[polygon], sources_names=list(sources))
    near, far = passage['front_m']
    half = passage['half_width_m']
    # Jazzy polygon points are one string: ROS parameters cannot hold nested arrays (like footprint in nav2.yaml)
    points = json.dumps([[far, half], [far, -half], [near, -half], [near, half]])
    cm[name] = {'type': 'polygon', 'action_type': 'stop', 'points': points,
                'min_points': passage['min_points'], 'visualize': True, 'polygon_pub_topic': 'passage_stop_polygon',
                'enabled': True, 'sources_names': [source]}
    cm[source] = {'type': 'pointcloud', 'topic': PASSAGE_POINTS.lstrip('/'), 'min_height': passage['min_height_m'],
                  'max_height': passage['max_height_m'], 'enabled': True}
    cm['polygons'], cm['observation_sources'] = polygons + [name], sources + [source]
    return params


def passage_points_launch_values(deployment, *, field, robot_id):
    """depth_image_proc for this robot's passage depth source (passage_points.launch.py)."""
    _, robot, _ = _robot(deployment, field, robot_id)
    p = _passage(field, robot_id, robot.namespace)
    return {'namespace': robot.namespace, 'image_rect': p['depth_topic'], 'camera_info': p['camera_info_topic'],
            'points': p['points_topic'], 'depth_transport': p['depth_transport']}
