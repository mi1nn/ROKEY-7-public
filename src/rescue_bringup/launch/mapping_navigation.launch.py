"""Start standalone TurtleBot4 SLAM, Nav2 and optional RViz.

Refuses to start while <namespace>/map already has a publisher (an earlier SLAM: its old map
would be continued), naming it (OPS-184). No overhead passage stop here (robot.launch.py only).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from rescue_bringup.map_guard import check_no_map_publisher


def _no_old_map(context):
    namespace = '/' + LaunchConfiguration('namespace').perform(context).strip('/')
    check_no_map_publisher(namespace + '/map', wait_s=float(LaunchConfiguration('map_publisher_wait_s').perform(context)))
    return []


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
        DeclareLaunchArgument('map_publisher_wait_s', default_value='3.0',
                              description='How long to look for an earlier <namespace>/map publisher before starting'),
        OpaqueFunction(function=_no_old_map),
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
