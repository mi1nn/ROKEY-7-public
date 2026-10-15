"""Build the LIVE_ROS runtime from typed ROS parameters; no file paths.

These are deployment settings, not observed maps/poses/clock records. No
production threshold, calibration or identity is inferred from a goal.
"""
import math
import re

from rescue_robot.executor import ContractError
from rescue_robot.live_sources import FREEZES

SHA256 = re.compile(r'[0-9a-f]{64}')   # as rescue_bringup field_launch (robot_launch_values)

FROZEN_MAP_TOPIC = '/rescue/map/frozen'   # the latest frozen TB-6 map (map_freeze on compute_a, OPS-113)
FROZEN_REVISION_TOPIC = '/rescue/map/frozen_revision'   # its revision (rescue_contracts.occupancy, OPS-159)


def frozen_revision(saved_map):
    """The revision a launch home is on: the declared one for a saved map, R0 + 2 for TB-6.

    The map is frozen twice (OPS-154); TB-6 uses its home (docking approach) only after the last
    freeze, R0 + 2. A FROZEN_MAP_AMCL robot moves from R0 + 1 to R0 + 2, so a fixed launch home
    cannot be right on both: its home (TB-5 exit) comes from the coordinator (home/<id>, OPS-139).
    """
    mode = saved_map['mode']
    if mode == 'FROZEN_MAP_AMCL':
        raise ContractError('a FROZEN_MAP_AMCL robot has no launch home; the coordinator sends home/<robot_id>')
    return saved_map['map_revision'] + (0 if mode == 'SAVED_MAP_AMCL' else FREEZES)


def home_destinations(robot_id, xy_yaw, *, frame_id, map_session_id, map_revision, transform_revision):
    """This robot's RETURN_HOME destination as a configured destination (home_ref 'home/<robot_id>')."""
    if (not isinstance(xy_yaw, (list, tuple)) or len(xy_yaw) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in xy_yaw)):
        raise ContractError('home [x, y, yaw] must be three finite numbers')
    return {'home/' + robot_id: dict(frame_id=frame_id, robot_id=robot_id, map_session_id=map_session_id,
                                      map_revision=map_revision, transform_revision=transform_revision,
                                      xy_yaw=[float(v) for v in xy_yaw])}


def passage_from_parameters(polygon_name, result_window_ns):
    """TB-5 passage reporting (live_passage): both values or neither; half a setting is refused."""
    if polygon_name is None and result_window_ns is None:
        return None
    if polygon_name is None or result_window_ns is None:
        raise ContractError('task.live.passage.polygon_name and task.live.passage.result_window_ns go together')
    return dict(polygon_name=polygon_name, result_window_ns=result_window_ns)


def read_config(node, adapter):
    from rcl_interfaces.msg import ParameterType as T
    from rescue_robot.ros_node import declare_required
    S, D, I = T.PARAMETER_STRING, T.PARAMETER_DOUBLE, T.PARAMETER_INTEGER
    fields = {
        'task.mission_frame': S, 'task.schema_version': S, 'task.publisher_id': S,
        'task.coordinator_id': S, 'task.coordinator_state_topic': S,
        'task.clock_error_bound_ns': I, 'task.lease.duration_s': D,
        'task.lease.max_age_ns': I, 'task.lease.max_error_ns': I, 'task.lease.clock_domain_id': S,
        'task.mapping_authority': S, 'task.max_position_error_m': D, 'task.max_rotation_error_rad': D,
        'task.input_max_age_ns': I, 'task.poll_period_s': D,
        'task.live.coordinator_service': S, 'task.live.clock_backend': S,
        'task.live.clock_max_age_ns': I, 'task.live.clock_drift_ppm': D,
        # SAVED_MAP_AMCL | SLAM_THEN_FROZEN (TB-6) | FROZEN_MAP_AMCL (TB-5), map lifecycle (OPS-113); the
        # two frozen modes take occupancy_sha256 '' (the freeze decides the hash)
        'task.saved_map.mode': S,
        'task.saved_map.map_topic': S, 'task.saved_map.localization_topic': S,
        'task.saved_map.map_frame': S, 'task.saved_map.occupancy_sha256': S,
        'task.saved_map.map_session_id': S, 'task.saved_map.map_revision': I,
        'task.saved_map.transform_revision': I,
        'task.saved_map.transform_xy_yaw': T.PARAMETER_DOUBLE_ARRAY,
        'task.saved_map.valid_from_ns': I, 'task.saved_map.transform_valid_until_ns': I,
        'task.saved_map.position_error_m': D, 'task.saved_map.rotation_error_rad': D,
        'task.saved_map.max_position_variance_m2': D, 'task.saved_map.max_yaw_variance_rad2': D,
        # Automatic target moves without an operator approval (C1, 2026-10-13); enabled=false rejects them.
        'task.live.auto_target.enabled': T.PARAMETER_BOOL, 'task.live.auto_target.kinds': T.PARAMETER_STRING_ARRAY,
        'task.live.auto_target.goal_clear_radius_m': D, 'task.live.auto_target.policy_version': S,
        # Arm again after a COMPLETED task with a confirmed stop (OPS-231, owner 2026-10-14); false = task_arm each time
        'task.live.auto_rearm': T.PARAMETER_BOOL,
        # ObserveTarget after a REOBSERVE_TARGET stop (C1-b): observation window and whole-call deadline
        'task.live.recheck.window_ns': I, 'task.live.recheck.deadline_ns': I,
        # RobotPose time.valid_until_ns = AMCL stamp + this (live_pose; coordinator applies its own age limit)
        'task.live.pose_valid_ns': I,
        # The robot Pi clock that stamps the scans (AMCL pose time): domain and bound measured on the Pi, -1 = not
        'task.live.sensor_host_clock_domain_id': S, 'task.live.sensor_host_clock_error_bound_ns': I}
    # occupancy_sha256 must be given, but '' is the value of the two frozen modes (checked per mode below)
    values, missing = declare_required(node, fields, optional=('task.saved_map.occupancy_sha256',))
    if missing:
        raise ContractError('LIVE_ROS missing deployment parameters: ' + ', '.join(missing))
    def get(name):
        return values['task.' + name]
    service = get('live.coordinator_service')
    if not service.startswith('/') or service.endswith('/'):
        raise ContractError('explicit absolute coordinator input service required')
    c = dict(format='rescue-robot-runtime/1', input_mode='LIVE_ROS', robot_id=adapter.robot_id,
        mission_frame=get('mission_frame'), schema_version=get('schema_version'), publisher_id=get('publisher_id'),
        clock_error_bound_ns=get('clock_error_bound_ns'),
        lease={key:get('lease.'+key) for key in ('duration_s','max_age_ns','max_error_ns','clock_domain_id')},
        mapping_authority=get('mapping_authority'), max_position_error_m=get('max_position_error_m'),
        max_rotation_error_rad=get('max_rotation_error_rad'), input_max_age_ns=get('input_max_age_ns'),
        poll_period_s=get('poll_period_s'), coordinator_state_topic=get('coordinator_state_topic'),
        coordinator_authority=dict(coordinator_id=get('coordinator_id'),authority_epoch=adapter.executor.authority_epoch),
        configured_destinations={}, live_clock=dict(backend=get('live.clock_backend'),
        max_age_ns=get('live.clock_max_age_ns'),drift_ppm=get('live.clock_drift_ppm'),
        coordinator_service=service,poll_period_s=get('poll_period_s')),
        endpoints=dict(action_name=f'/rescue/robots/{adapter.robot_id}/execute_task',
            lease_topic=f'/rescue/robots/{adapter.robot_id}/command_lease',
            admission_topic=f'/rescue/robots/{adapter.robot_id}/task_admission',
            status_topic=f'/rescue/robots/{adapter.robot_id}/status'))
    m = {key:get('saved_map.'+key) for key in ('map_topic','localization_topic','map_frame','occupancy_sha256',
        'map_session_id','map_revision','transform_revision','transform_xy_yaw','valid_from_ns',
        'transform_valid_until_ns','position_error_m','rotation_error_rad',
        'max_position_variance_m2','max_yaw_variance_rad2')}
    if len(m['transform_xy_yaw']) != 3:
        raise ContractError('explicit measured [x,y,yaw] transform required')
    m.update(mode=get('saved_map.mode'),authority_id=c['mapping_authority'],clock_domain_id=c['lease']['clock_domain_id'])
    if m['mode'] == 'SAVED_MAP_AMCL':
        if not SHA256.fullmatch(m['occupancy_sha256']):
            raise ContractError('SAVED_MAP_AMCL needs the measured task.saved_map.occupancy_sha256 (64 lowercase hex)')
    elif m['occupancy_sha256'] != '':
        raise ContractError(f'{m["mode"]}: the freeze decides the map hash; task.saved_map.occupancy_sha256 '
                            'must be empty')
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    if m['mode'] != 'SAVED_MAP_AMCL':
        m['frozen_topic'], m['frozen_revision_topic'] = FROZEN_MAP_TOPIC, FROZEN_REVISION_TOPIC
    # TB-5 AMCL initial pose topic: re-aligning on a newer frozen map starts from its own last pose
    node.declare_parameter('task.saved_map.initial_pose_topic', Parameter.Type.STRING)
    try:
        m['initial_pose_topic'] = node.get_parameter('task.saved_map.initial_pose_topic').value
    except ParameterUninitializedException:
        if m['mode'] == 'FROZEN_MAP_AMCL':
            raise ContractError('FROZEN_MAP_AMCL needs task.saved_map.initial_pose_topic') from None
    # TB-5 start alignment verdict (rescue_robot.start_align, #317): VALID only after ALIGNED
    node.declare_parameter('task.saved_map.start_align_topic', Parameter.Type.STRING)
    try:
        m['start_align_topic'] = node.get_parameter('task.saved_map.start_align_topic').value
    except ParameterUninitializedException:
        if m['mode'] == 'FROZEN_MAP_AMCL':
            raise ContractError('FROZEN_MAP_AMCL needs task.saved_map.start_align_topic') from None
    # TB-6 SLAM pose while searching (slam_toolbox), only in SLAM_THEN_FROZEN
    node.declare_parameter('task.saved_map.slam_localization_topic', Parameter.Type.STRING)
    try:
        m['slam_localization_topic'] = node.get_parameter('task.saved_map.slam_localization_topic').value
    except ParameterUninitializedException:
        if m['mode'] == 'SLAM_THEN_FROZEN':
            raise ContractError('SLAM_THEN_FROZEN needs task.saved_map.slam_localization_topic') from None
    # TB-6 pose while searching = TF map -> base_link, covariance from the SLAM pose (OPS-177)
    for name, kind in (('slam_tf_topic', Parameter.Type.STRING), ('slam_odom_frame', Parameter.Type.STRING),
                       ('slam_base_frame', Parameter.Type.STRING), ('slam_covariance_max_age_ns', Parameter.Type.INTEGER)):
        node.declare_parameter('task.saved_map.' + name, kind)
        try:
            m[name] = node.get_parameter('task.saved_map.' + name).value
        except ParameterUninitializedException:
            if m['mode'] == 'SLAM_THEN_FROZEN':
                raise ContractError('SLAM_THEN_FROZEN needs task.saved_map.' + name) from None
    c['saved_maps'] = {adapter.robot_id:m}
    c['auto_target'] = {key:get('live.auto_target.'+key) for key in
                        ('enabled','kinds','goal_clear_radius_m','policy_version')}
    c['auto_rearm'] = get('live.auto_rearm')
    c['recheck'] = {key:get('live.recheck.'+key) for key in ('window_ns','deadline_ns')}
    # The coordinator's TargetSnapshot: target revision and position for REOBSERVE (IF-070)
    c['coordinator_targets_topic'] = '/rescue/targets'
    c['pose_valid_ns'] = get('live.pose_valid_ns')
    c['sensor_host_clock'] = dict(domain_id=get('live.sensor_host_clock_domain_id'),
                                  error_bound_ns=get('live.sensor_host_clock_error_bound_ns'))
    # Optional: this robot's home (profile home_positions). Not given = no home; RETURN_HOME is refused.
    node.declare_parameter('task.live.home_xy_yaw', Parameter.Type.DOUBLE_ARRAY)
    try:
        home = node.get_parameter('task.live.home_xy_yaw').value
    except ParameterUninitializedException:
        home = None
    if home is not None:
        c['configured_destinations'] = home_destinations(adapter.robot_id, list(home), frame_id=c['mission_frame'],
            map_session_id=m['map_session_id'], map_revision=frozen_revision(m),
            transform_revision=m['transform_revision'])
    # Optional (TB-5): the collision_monitor polygon that stops it before a low passage (OPS-128)
    passage = []
    for name, kind in (('polygon_name', Parameter.Type.STRING), ('result_window_ns', Parameter.Type.INTEGER)):
        node.declare_parameter('task.live.passage.' + name, kind)
        try:
            passage.append(node.get_parameter('task.live.passage.' + name).value)
        except ParameterUninitializedException:
            passage.append(None)
    passage = passage_from_parameters(*passage)
    if passage is not None:
        c['passage'] = passage
    if not 0 < c['poll_period_s'] * 1e9 < c['live_clock']['max_age_ns']:
        raise ContractError('clock polling period must be positive and shorter than measurement age budget')
    if c['live_clock']['backend'] not in ('chrony','timesyncd') or c['live_clock']['drift_ppm'] <= 0:
        raise ContractError('explicit clock backend and positive drift budget required')
    return c
