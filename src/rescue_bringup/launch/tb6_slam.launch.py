"""TB-6 SLAM while searching (map lifecycle, OPS-113): slam_toolbox only, no Nav2 (robot.launch.py has it).

    ros2 launch rescue_bringup tb6_slam.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a

Only the SLAM_THEN_FROZEN robot of the field run file. Wraps turtlebot4_navigation
slam.launch.py: map on <ns>/map, pose on <ns>/pose (task_input searching and map_freeze read
them). Stop it (Ctrl-C) after the freeze, before frozen_localization.launch.py robot_id:=robot_a.
Refuses to start while <ns>/map already has a publisher (an earlier SLAM: its old map would be
continued), naming it (OPS-184).

Parameters: TurtleBot4 slam.yaml (or slam_params_file) with only minimum_travel_distance / heading 0.0, so a
standing TB-6 gives <ns>/pose (rescue_bringup.slam_field, 실습 기록 #339 B3). slam_active_check logs
SLAM_ACTIVE or SLAM_NOT_ACTIVE slam_active_wait_s after the start and sends activate once to an inactive
slam_toolbox when slam_activate_once is true (B4).
"""
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import load_field, slam_launch_values
from rescue_bringup.map_guard import check_no_map_publisher
from rescue_bringup.profile import load_deployment
from rescue_bringup.slam_field import STANDING, field_slam


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = slam_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                robot_id=arg('robot_id'))
    # an earlier SLAM still publishing <ns>/map would be continued silently: refuse and name it (OPS-184)
    check_no_map_publisher(values['namespace'] + '/map', wait_s=float(arg('map_publisher_wait_s')))
    share = Path(get_package_share_directory('turtlebot4_navigation'))
    base = arg('slam_params_file') or str(share / 'config' / 'slam.yaml')
    with tempfile.NamedTemporaryFile('w', prefix=f'slam_{arg("robot_id")}_', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(field_slam(yaml.safe_load(Path(base).read_text())), out, sort_keys=False)
    params = out.name
    wait_s, activate_once = float(arg('slam_active_wait_s')), arg('slam_activate_once') == 'true'
    if wait_s <= 0 or arg('slam_activate_once') not in ('true', 'false'):
        raise ValueError('slam_active_wait_s must be > 0 and slam_activate_once true or false')
    return [
        LogInfo(msg=f'TB-6 SLAM {arg("robot_id")} ns={values["namespace"]} params={base} + {STANDING} -> {params}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'slam.launch.py')),
            launch_arguments={'namespace': values['namespace'], 'use_sim_time': str(values['use_sim_time']).lower(),
                              'params': params}.items()),
        Node(package='rescue_bringup', executable='slam_active_check', name='tb6_slam_active_check',
             output='screen', parameters=[{'node': values['namespace'] + '/slam_toolbox', 'wait_s': wait_s,
                                           'activate_once': activate_once}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_publisher_wait_s', default_value='3.0',
                              description='How long to look for an earlier <ns>/map publisher before starting'),
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='The SLAM_THEN_FROZEN robot (robot_a, TB-6)'),
        DeclareLaunchArgument('slam_params_file', default_value='',
                              description='slam_toolbox parameters; empty = turtlebot4_navigation config/slam.yaml '
                                          '(either way minimum_travel_distance / heading become 0.0)'),
        DeclareLaunchArgument('slam_active_wait_s', default_value='15.0',
                              description='[제안] seconds after the start to check slam_toolbox is active'),
        DeclareLaunchArgument('slam_activate_once', default_value='true', choices=['true', 'false'],
                              description='send activate once (logged) to a slam_toolbox still inactive then'),
        OpaqueFunction(function=_nodes),
    ])
