"""Start standalone TurtleBot4 saved-map localization, Nav2 and optional RViz.

Nav2 runs without the backup recovery (OPS-207, rescue_bringup.nav2_recovery).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from rescue_bringup.nav2_field import write_field_nav2


def _nav2(context, shared_arguments):
    """TurtleBot4 Nav2 with nav2_params_file minus the backup recovery (OPS-207)."""
    base = LaunchConfiguration('nav2_params_file').perform(context)
    params = write_field_nav2(base, prefix='nav2_field_')
    return [
        LogInfo(msg=f'nav2_params={base} + no BackUp recovery + FollowPath Rotation Shim/RPP -> {params}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('turtlebot4_navigation'), 'launch', 'nav2.launch.py'])
            ),
            launch_arguments={**shared_arguments, 'params_file': params}.items(),
        ),
    ]


def generate_launch_description():
    navigation = FindPackageShare('turtlebot4_navigation')
    shared_arguments = {
        'namespace': LaunchConfiguration('namespace'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Namespace of the physical robot'),
        DeclareLaunchArgument('map', description='Absolute path to the saved occupancy-map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=PathJoinSubstitution([navigation, 'config', 'localization.yaml']),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([navigation, 'config', 'nav2.yaml']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([navigation, 'launch', 'localization.launch.py'])
            ),
            launch_arguments={
                **shared_arguments,
                'map': LaunchConfiguration('map'),
                'params': LaunchConfiguration('localization_params_file'),
            }.items(),
        ),
        OpaqueFunction(function=_nav2, args=[shared_arguments]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'), 'launch', 'view_navigation.launch.py',
            ])),
            launch_arguments=shared_arguments.items(),
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
