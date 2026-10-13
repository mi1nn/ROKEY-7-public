"""Map freeze node on compute_a (map lifecycle, OPS-113): /rescue/map/freeze -> /rescue/map/frozen.

    ros2 launch rescue_bringup map_freeze.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json output_dir:=<absolute dir> [record_path:=<saved .json>]

Reads the SLAM_THEN_FROZEN robot's <ns>/map and <ns>/pose on the profile map_basis (revision
R0). record_path (absolute) restarts from a saved freeze and republishes the same grid.
The coordinator asks for the freeze; this launch sends no request itself.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import load_field, map_freeze_parameters
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params = map_freeze_parameters(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                   output_dir=arg('output_dir'), record_path=arg('record_path'))
    return [Node(package='rescue_robot', executable='map_freeze', name='rescue_map_freeze', output='screen',
                 parameters=[typed(params)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('output_dir', description='Absolute directory for the frozen map record'),
        DeclareLaunchArgument('record_path', default_value='',
                              description='Absolute saved freeze .json to republish after a restart; empty = new freeze'),
        OpaqueFunction(function=_nodes),
    ])
