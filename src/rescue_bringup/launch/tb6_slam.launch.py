"""TB-6 SLAM while searching (map lifecycle, OPS-113): slam_toolbox only, no Nav2 (robot.launch.py has it).

    ros2 launch rescue_bringup tb6_slam.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a

Only the SLAM_THEN_FROZEN robot of the field run file. Wraps turtlebot4_navigation
slam.launch.py: map on <ns>/map, pose on <ns>/pose (task_input searching and map_freeze read
them). Stop it (Ctrl-C) after the freeze, before frozen_localization.launch.py robot_id:=robot_a.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rescue_bringup.field_launch import load_field, slam_launch_values
from rescue_bringup.profile import load_deployment


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = slam_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                robot_id=arg('robot_id'))
    share = Path(get_package_share_directory('turtlebot4_navigation'))
    params = arg('slam_params_file') or str(share / 'config' / 'slam.yaml')
    return [
        LogInfo(msg=f'TB-6 SLAM {arg("robot_id")} ns={values["namespace"]} params={params}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'slam.launch.py')),
            launch_arguments={'namespace': values['namespace'], 'use_sim_time': str(values['use_sim_time']).lower(),
                              'params': params}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='The SLAM_THEN_FROZEN robot (robot_a, TB-6)'),
        DeclareLaunchArgument('slam_params_file', default_value='',
                              description='slam_toolbox parameters; empty = turtlebot4_navigation config/slam.yaml'),
        OpaqueFunction(function=_nodes),
    ])
