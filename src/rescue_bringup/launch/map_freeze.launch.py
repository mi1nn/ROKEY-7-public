"""Map freeze node on compute_a (map lifecycle, OPS-113): /rescue/map/freeze -> /rescue/map/frozen.

    ros2 launch rescue_bringup map_freeze.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json output_dir:=<absolute dir>

Reads the SLAM_THEN_FROZEN robot's <ns>/map and <ns>/pose on the profile map_basis (revision
R0). Two freezes per run (R0 + 1, R0 + 2). Restarted with the same output_dir, it republishes
the freezes (and the TB-6 start pose) it already made; a new run needs a new output_dir.
The coordinator asks for the freeze; this launch sends no request itself.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import FieldLaunchError, load_field, map_freeze_parameters
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    field = load_field(arg('field_path'))
    if field.coordinator['map_mode'] != 'SLAM_FREEZE':
        raise FieldLaunchError('map_freeze.launch.py requires legacy SLAM_FREEZE; use live_map.launch.py')
    params = map_freeze_parameters(load_deployment(arg('profile_path')), field=field,
                                   output_dir=arg('output_dir'))
    return [Node(package='rescue_robot', executable='map_freeze', name='rescue_map_freeze', output='screen',
                 parameters=[typed(params)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('output_dir', description='Absolute directory of this run (start pose, freezes)'),
        OpaqueFunction(function=_nodes),
    ])
