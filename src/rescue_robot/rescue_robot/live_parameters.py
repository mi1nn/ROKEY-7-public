"""Build the LIVE_ROS runtime from typed ROS parameters; no file paths.

These are deployment settings, not observed maps/poses/clock records. No
production threshold, calibration or identity is inferred from a goal.
"""
import math

from rescue_robot.executor import ContractError


def home_destinations(robot_id, xy_yaw, *, frame_id, map_session_id, map_revision, transform_revision):
    """This robot's RETURN_HOME destination as a configured destination (home_ref 'home/<robot_id>')."""
    if (not isinstance(xy_yaw, (list, tuple)) or len(xy_yaw) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in xy_yaw)):
        raise ContractError('home [x, y, yaw] must be three finite numbers')
    return {'home/' + robot_id: dict(frame_id=frame_id, robot_id=robot_id, map_session_id=map_session_id,
                                      map_revision=map_revision, transform_revision=transform_revision,
                                      xy_yaw=[float(v) for v in xy_yaw])}


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
        # ObserveTarget after a REOBSERVE_TARGET stop (C1-b): observation window and whole-call deadline
        'task.live.recheck.window_ns': I, 'task.live.recheck.deadline_ns': I}
    values, missing = declare_required(node, fields)
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
    m.update(mode='SAVED_MAP_AMCL',authority_id=c['mapping_authority'],clock_domain_id=c['lease']['clock_domain_id'])
    c['saved_maps'] = {adapter.robot_id:m}
    c['auto_target'] = {key:get('live.auto_target.'+key) for key in
                        ('enabled','kinds','goal_clear_radius_m','policy_version')}
    c['recheck'] = {key:get('live.recheck.'+key) for key in ('window_ns','deadline_ns')}
    # The coordinator's TargetSnapshot: target revision and position for REOBSERVE (IF-070)
    c['coordinator_targets_topic'] = '/rescue/targets'
    # Optional: this robot's home (profile home_positions). Not given = no home; RETURN_HOME is refused.
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    node.declare_parameter('task.live.home_xy_yaw', Parameter.Type.DOUBLE_ARRAY)
    try:
        home = node.get_parameter('task.live.home_xy_yaw').value
    except ParameterUninitializedException:
        home = None
    if home is not None:
        c['configured_destinations'] = home_destinations(adapter.robot_id, list(home), frame_id=c['mission_frame'],
            map_session_id=m['map_session_id'], map_revision=m['map_revision'],
            transform_revision=m['transform_revision'])
    if not 0 < c['poll_period_s'] * 1e9 < c['live_clock']['max_age_ns']:
        raise ContractError('clock polling period must be positive and shorter than measurement age budget')
    if c['live_clock']['backend'] not in ('chrony','timesyncd') or c['live_clock']['drift_ppm'] <= 0:
        raise ContractError('explicit clock backend and positive drift budget required')
    return c
