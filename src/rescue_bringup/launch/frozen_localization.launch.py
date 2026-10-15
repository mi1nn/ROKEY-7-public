"""AMCL on the frozen TB-6 map (map lifecycle, OPS-113/124): one robot, after the freeze.

    ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_b frozen_pose_wait_s:=10.0

- frozen_map_relay copies to <ns>/map the frozen grid of the revision this robot's executor is on
  (its MapState; pair /rescue/map/frozen_revision, OPS-154), only while no other node publishes
  <ns>/map: stop TB-6 SLAM first. TB-6 gets the second (last) freeze; TB-5 the first, then the
  second once its executor takes it (stopped, no task).
- AMCL (nav2_amcl, turtlebot4_navigation localization.yaml values, map from <ns>/map, no
  map_server) starts at: robot_a = the frozen pose /rescue/map/frozen_pose/robot_a (this launch
  waits up to frozen_pose_wait_s for it, then fails); a FROZEN_MAP_AMCL robot = its measured
  robots.<id>.initial_pose_xy_yaw, relative to the TB-6 start pose, placed on the map with
  /rescue/map/start_pose/robot_a (waits frozen_pose_wait_s; missing value or start = the launch refuses).
- start_align (FROZEN_MAP_AMCL robot only, rescue_robot.start_align): AMCL starts there with zero spread
  [Nav2 jazzy amcl_node.cpp:283-295], so once the robot is undocked and stopped this node re-spreads the
  particles, asks no-motion updates and latches ALIGNED or a reason code on <ns>/rescue_start_align.
Every node is in the robot namespace, so both robots' launches can run on one PC.
No arm, task or movement command is sent.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import yaml

from rescue_contracts.occupancy import compose_planar

from rescue_bringup.field_launch import amcl_parameters, frozen_localization_values, load_field
from rescue_bringup.frozen_pose import wait_frozen_pose
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    deployment = load_deployment(arg('profile_path'))
    values = frozen_localization_values(deployment, field=load_field(arg('field_path')), robot_id=arg('robot_id'))
    ns, use_sim_time = values['namespace'], values['use_sim_time']
    if values['initial_pose_topic'] is not None:
        start = list(wait_frozen_pose(values['initial_pose_topic'], timeout_s=float(arg('frozen_pose_wait_s')),
                                      map_frame=deployment.map_basis.frame_id))
        source = values['initial_pose_topic']
    else:
        # field value relative to the TB-6 start pose (OPS-141) -> map, with this run's start pose
        tb6_start = wait_frozen_pose(values['start_pose_topic'], timeout_s=float(arg('frozen_pose_wait_s')),
                                     map_frame=deployment.map_basis.frame_id)
        start = list(compose_planar(tb6_start, values['initial_pose_xy_yaw']))
        source = (f'field run file robots.{arg("robot_id")} {values["initial_pose_xy_yaw"]} relative to the TB-6 '
                  f'start {[round(v, 3) for v in tb6_start]} ({values["start_pose_topic"]})')
    params_file = arg('localization_params_file') or str(
        Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'localization.yaml')
    amcl = amcl_parameters(yaml.safe_load(Path(params_file).read_text()), initial_pose_xy_yaw=start,
                           use_sim_time=use_sim_time)
    return [
        LogInfo(msg=f'frozen localization {arg("robot_id")} ns={ns} map={values["map_topic"]} <- '
                    f'{values["frozen_topic"]} amcl_start={start} from {source} params={params_file}'
                    + (' + start_align' if values['start_align'] is not None else '')),
        Node(package='rescue_robot', executable='frozen_map_relay', name='rescue_frozen_map_relay', namespace=ns,
             output='screen', parameters=[typed({key: values[key] for key in (
                 'frozen_topic', 'frozen_revision_topic', 'map_state_topic', 'map_topic')})]),
        GroupAction([
            SetRemap('/tf', ns + '/tf'), SetRemap('/tf_static', ns + '/tf_static'),
            Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=ns, output='screen',
                 parameters=[typed(amcl)]),
            *([Node(package='rescue_robot', executable='start_align', name='rescue_start_align', namespace=ns,
                    output='screen', parameters=[typed(values['start_align'])])]
              if values['start_align'] is not None else []),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['amcl']}]),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='robot_a (TB-6, after SLAM is stopped) or robot_b (TB-5)'),
        DeclareLaunchArgument('frozen_pose_wait_s',
                              description='How long to wait (s) for /rescue/map/frozen_pose/robot_a (robot_a) or '
                                          '/rescue/map/start_pose/robot_a (robot_b)'),
        DeclareLaunchArgument('localization_params_file', default_value='',
                              description='AMCL parameters; empty = turtlebot4_navigation config/localization.yaml'),
        OpaqueFunction(function=_nodes),
    ])
