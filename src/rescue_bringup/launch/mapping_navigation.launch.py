"""Start standalone TurtleBot4 SLAM, Nav2 and optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation = FindPackageShare('turtlebot4_navigation')
    shared_arguments = {
        'namespace': LaunchConfiguration('namespace'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Namespace of the physical robot'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([navigation, 'config', 'slam.yaml']),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([navigation, 'config', 'nav2.yaml']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([navigation, 'launch', 'slam.launch.py'])
            ),
            launch_arguments={
                **shared_arguments, 'params': LaunchConfiguration('slam_params_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([navigation, 'launch', 'nav2.launch.py'])
            ),
            launch_arguments={
                **shared_arguments, 'params_file': LaunchConfiguration('nav2_params_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'), 'launch', 'view_navigation.launch.py',
            ])),
            launch_arguments=shared_arguments.items(),
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
