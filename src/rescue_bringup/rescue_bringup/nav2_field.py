"""The Nav2 parameters our launches run with, on both robots; no ROS.

TurtleBot4's nav2.yaml is read as is and changed here only:
- no backup recovery (nav2_recovery, OPS-207);
- controller_server.FollowPath = Rotation Shim + Regulated Pure Pursuit instead of DWB (재범 request,
  2026-10-14 evening). Sent to a goal behind the back wall of a U-shaped dead end, TB-6 turned its head
  left and right inside the U and got stuck although the global path went out through the entrance. DWB
  scores GoalDist and GoalAlign (24 each) toward the goal against PathAlign and PathDist (32) along the
  path [추정]; RPP follows the path itself, the shim turns in place first when the path starts behind.
Anything unexpected is refused, never patched around. The values are [제안] until a supervised
before/after run at the same spot (NOT_RUN).
"""
import copy

from rescue_bringup.nav2_recovery import nav2_without_backup

DWB = 'dwb_core::DWBLocalPlanner'
SHIM = 'nav2_rotation_shim_controller::RotationShimController'
RPP = 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
# [제안] Nav2 Jazzy parameter names (Rotation Shim / Regulated Pure Pursuit configuration pages); the DWB block
# it replaces: max_vel_x 0.26, max_vel_theta 1.0, acc_lim_theta 3.2, sim_time 1.7, xy_goal_tolerance 0.25.
FOLLOW_PATH = {
    'plugin': SHIM,
    'primary_controller': RPP,
    # shim: turn in place while the path heading is more than 45 deg off (Nav2 example value)
    'angular_dist_threshold': 0.785,
    'forward_sampling_distance': 0.5,
    'rotate_to_heading_angular_vel': 1.0,      # = DWB max_vel_theta
    'max_angular_accel': 3.2,                  # = DWB acc_lim_theta
    'simulate_ahead_time': 1.0,                # collision check of the in-place turn
    # at the goal the shim turns in place to the goal heading. With it off (2026-10-14, wrong comment: "the goal
    # checker's 0.25 rad tolerance stays the judge") nothing turned the robot, since RPP's use_rotate_to_heading must be
    # off with the shim: run 10 (2026-10-16) TB-6 expired 0.035 m from a goal. 재범 decision 2026-10-16 05:0x 「켜기」
    'rotate_to_goal_heading': True,
    # RPP
    # = DWB max_vel_x and the velocity smoother cap 0.26 (TurtleBot4 nav2.yaml); 0.2 before. 재범 2026-10-16 05:4x
    # 「TB 속도가 실제에 비해 너무 느리다. 실제의 100%로 맞춰라.」 (grade 4). The watchdog limit is 0.3
    'desired_linear_vel': 0.26,
    'lookahead_dist': 0.4,
    'min_lookahead_dist': 0.3,
    'max_lookahead_dist': 0.6,
    'lookahead_time': 1.5,
    'use_velocity_scaled_lookahead_dist': False,
    'transform_tolerance': 0.1,
    'min_approach_linear_velocity': 0.05,
    'approach_velocity_scaling_dist': 0.6,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.9,
    'regulated_linear_scaling_min_speed': 0.1,
    'use_rotate_to_heading': False,            # must be off with the shim (Nav2 docs)
    'allow_reversing': False,                  # no reversing (OPS-207)
    'max_robot_pose_search_dist': 10.0,
}


class Nav2FieldError(ValueError):
    pass


def nav2_with_rotation_shim(params):
    """A copy of the Nav2 parameters whose FollowPath is Rotation Shim + RPP (the DWB block replaced whole)."""
    params = copy.deepcopy(params)
    try:
        controller = params['controller_server']['ros__parameters']
        plugins = list(controller['controller_plugins'])
        follow = controller['FollowPath']
    except (KeyError, TypeError):
        raise Nav2FieldError('Nav2 parameters need controller_server controller_plugins and FollowPath') from None
    if 'FollowPath' not in plugins:
        raise Nav2FieldError(f'controller_server controller_plugins {plugins} has no FollowPath')
    if not isinstance(follow, dict) or follow.get('plugin') != DWB:
        raise Nav2FieldError(f'FollowPath is {follow.get("plugin") if isinstance(follow, dict) else follow!r}, '
                             f'expected the TurtleBot4 {DWB} to replace')
    controller['FollowPath'] = dict(FOLLOW_PATH)
    return params


def field_nav2(params):
    """TurtleBot4 Nav2 parameters (dicts) -> the ones our launches run: no backup, Rotation Shim + RPP."""
    return nav2_with_rotation_shim(nav2_without_backup(params))


KEEPOUT_INFO_TOPIC = '/rescue/passage_keepout/filter_info'   # rescue_robot.passage_keepout (both robots read it)
KEEPOUT_FILTER = 'tunnel_keepout'


def nav2_with_tunnel_keepout(params):
    """A copy with a Nav2 KeepoutFilter on the global and local costmaps reading the tunnel keepout mask
    (재범 decision 2026-10-16, option A: tunnels the vision judgement found BLOCKED are kept out of paths).
    robot.launch.py only: the manual launches have no overhead_passage and no mask."""
    params = copy.deepcopy(params)
    for costmap in ('global_costmap', 'local_costmap'):
        try:
            p = params[costmap][costmap]['ros__parameters']
        except (KeyError, TypeError):
            raise Nav2FieldError(f'Nav2 parameters need {costmap}.{costmap}.ros__parameters') from None
        filters = list(p.get('filters', []))
        if KEEPOUT_FILTER in filters or KEEPOUT_FILTER in p:
            raise Nav2FieldError(f'{costmap} already has {KEEPOUT_FILTER}')
        p['filters'] = filters + [KEEPOUT_FILTER]
        p[KEEPOUT_FILTER] = {'plugin': 'nav2_costmap_2d::KeepoutFilter', 'enabled': True,
                             'filter_info_topic': KEEPOUT_INFO_TOPIC}
    return params


DEPTH_LOW_TOPIC = 'rescue/low_obstacle_points'   # rescue_perception overhead_passage, <ns>/... (relative)
DEPTH_LOW_LAYER = 'depth_low_layer'


def _scan_source(costmap):
    """The lidar source of a costmap's obstacle or voxel layer: its ranges are the depth layer's."""
    for name in costmap.get('plugins', []):
        layer = costmap.get(name, {})
        if 'scan' in str(layer.get('observation_sources', '')).split():
            return layer['scan']
    raise Nav2FieldError('costmap has no layer with a scan source to take the ranges from')


def nav2_with_depth_low_layer(params, passage):
    """A copy with a depth-only ObstacleLayer before the inflation layer of the global and local costmaps
    (재범 decision 2026-10-16, option D): objects under the lidar plane (dock, doll) are kept out of paths.
    Marking: heights min_height_m .. overhead_lidar_plane_m above the floor seen; clearing: raytrace to the floor
    points. The scan is not a source, so it cannot clear it. Ranges are the scan source's; observation_persistence
    and expected_update_rate are Nav2's defaults (0.0): without a cloud the layer keeps its marks.
    robot.launch.py only (overhead_passage publishes the cloud)."""
    params = copy.deepcopy(params)
    low, top = passage['min_height_m'], passage['overhead_lidar_plane_m']
    for costmap in ('global_costmap', 'local_costmap'):
        try:
            p = params[costmap][costmap]['ros__parameters']
            plugins = list(p['plugins'])
        except (KeyError, TypeError):
            raise Nav2FieldError(f'Nav2 parameters need {costmap}.{costmap}.ros__parameters.plugins') from None
        if DEPTH_LOW_LAYER in plugins or DEPTH_LOW_LAYER in p:
            raise Nav2FieldError(f'{costmap} already has {DEPTH_LOW_LAYER}')
        if not plugins or plugins[-1] != 'inflation_layer':
            raise Nav2FieldError(f'{costmap} plugins {plugins} do not end with inflation_layer')
        scan = _scan_source(p)
        ranges = {k: scan.get(k, default) for k, default in (('obstacle_max_range', 2.5), ('obstacle_min_range', 0.0),
                                                             ('raytrace_max_range', 3.0), ('raytrace_min_range', 0.0))}
        common = dict(topic=DEPTH_LOW_TOPIC, data_type='PointCloud2', max_obstacle_height=top,
                      observation_persistence=0.0, expected_update_rate=0.0, **ranges)
        p[DEPTH_LOW_LAYER] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True, 'observation_sources': 'depth_low depth_floor',
            'depth_low': dict(common, marking=True, clearing=False, min_obstacle_height=low),
            'depth_floor': dict(common, marking=False, clearing=True, min_obstacle_height=-low)}
        p['plugins'] = plugins[:-1] + [DEPTH_LOW_LAYER, plugins[-1]]
    return params


def write_field_nav2(base_path, *, prefix):
    """Read the Nav2 yaml at ``base_path``, apply field_nav2, write a temporary yaml; returns its path."""
    import tempfile
    from pathlib import Path

    import yaml

    params = field_nav2(yaml.safe_load(Path(base_path).read_text()))
    with tempfile.NamedTemporaryFile('w', prefix=prefix, suffix='.yaml', delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name
