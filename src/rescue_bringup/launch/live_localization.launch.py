"""TB-5 AMCL on TB-6's live SLAM map (방식 B): measured start seed + start_align gate; no map_server, no freeze.

    ros2 launch rescue_bringup live_localization.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/live_map.overlay.json robot_id:=robot_b \
        start_pose_wait_s:=10.0 [tb5_start_xy_yaw:='[x, y, yaw]']

- AMCL (turtlebot4_navigation localization.yaml values) reads every full grid of TB-6's live map as relayed by
  self_clear_map on <ns>/rescue/shared_map: TB-6's grid with TB-5's own body cleared around its start and its latest
  pose, coordinates unchanged (first_map_only false, tf_broadcast true: AMCL alone publishes TB-5's map -> odom).
- Its start = the measured TB-5 start [x m, y m, yaw rad] relative to the TB-6 start pose (OPS-141), placed on the
  map with /rescue/map/start_pose/robot_a (live_map.launch.py; waits start_pose_wait_s, then the launch fails).
  The value is tb5_start_xy_yaw when given, else the field run file robots.<id>.initial_pose_xy_yaw; neither =
  refused (never made up).
- start_align re-spreads the particles, asks no-motion updates on the live map and judges convergence; only while it
  is ALIGNED are AMCL poses relayed to <ns>/localized_pose, the executor's localization input (task_input
  task.saved_map.localization_topic). Its status is latched on <ns>/rescue_start_align.
Every node is in the robot namespace. No arm, task or movement command is sent.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import yaml

from rescue_contracts.occupancy import compose_planar

from rescue_bringup.field_launch import _list_argument, amcl_parameters, live_localization_values, load_field
from rescue_bringup.frozen_pose import wait_frozen_pose
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    deployment = load_deployment(arg('profile_path'))
    override = arg('tb5_start_xy_yaw')
    values = live_localization_values(deployment, field=load_field(arg('field_path')), robot_id=arg('robot_id'),
                                      start_xy_yaw=_list_argument(override, 'tb5_start_xy_yaw', 'float')
                                      if override else None)
    ns, use_sim_time = values['namespace'], values['use_sim_time']
    tb6_start = wait_frozen_pose(values['start_pose_topic'], timeout_s=float(arg('start_pose_wait_s')),
                                 map_frame=values['map_frame'])
    start = list(compose_planar(tb6_start, values['initial_pose_xy_yaw']))
    params_file = arg('localization_params_file') or str(
        Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'localization.yaml')
    amcl = amcl_parameters(yaml.safe_load(Path(params_file).read_text()), initial_pose_xy_yaw=start,
                           use_sim_time=use_sim_time, map_topic=values['map_topic'], live=True)
    amcl.update(global_frame_id=values['map_frame'], odom_frame_id=values['odom_frame'],
                base_frame_id=values['base_frame'], scan_topic=values['scan_topic'])
    return [
        LogInfo(msg=f'LIVE_MAP_AMCL {arg("robot_id")} ns={ns} map={values["map_topic"]} amcl_start='
                    f'{[round(v, 3) for v in start]} from {values["start_source"]} {values["initial_pose_xy_yaw"]} '
                    f'relative to the TB-6 start {[round(v, 3) for v in tb6_start]} ({values["start_pose_topic"]}) '
                    f'+ start_align gate -> {values["start_align"]["output_pose_topic"]} params={params_file}'),
        GroupAction([
            SetRemap('/tf', values['tf_topic']), SetRemap('/tf_static', values['tf_static_topic']),
            Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=ns, output='screen',
                 parameters=[typed(amcl)]),
            Node(package='rescue_robot', executable='self_clear_map', name='rescue_self_clear_map', namespace=ns,
                 output='screen', parameters=[typed(values['self_clear'] | {'start_xy': [float(v) for v in start[:2]]})]),
            Node(package='rescue_robot', executable='start_align', name='rescue_start_align', namespace=ns,
                 output='screen', parameters=[typed(values['start_align'])]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['amcl']}]),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='SLAM_LIVE field run file or overlay '
                                                        '(config/field/live_map.overlay.json)'),
        DeclareLaunchArgument('robot_id', description='The LIVE_MAP_AMCL robot (TB-5, robot_b)'),
        DeclareLaunchArgument('start_pose_wait_s',
                              description='How long to wait (s) for /rescue/map/start_pose/<TB-6 id>'),
        DeclareLaunchArgument('tb5_start_xy_yaw', default_value='',
                              description="Measured TB-5 start '[x, y, yaw]' (m, m, rad) relative to the TB-6 start "
                                          'pose; empty = the field run file robots.<id>.initial_pose_xy_yaw'),
        DeclareLaunchArgument('localization_params_file', default_value='',
                              description='AMCL parameters; empty = turtlebot4_navigation config/localization.yaml'),
        OpaqueFunction(function=_nodes),
    ])
