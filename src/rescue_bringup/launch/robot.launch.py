"""Robot side: TurtleBot4 Nav2 + command_watchdog + task_input (SEARCH_AREA, live inputs).

    ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a \
        occupancy_sha256:=$(python3 scripts/field/map_occupancy_sha256.py /robot6/map map)

robot_b is the same command with robot_id:=robot_b once the field run file has robots.robot_b.
Localization (turtlebot4_navigation localization.launch.py with the prior map) runs before
this, once, and the map hash is measured from its map topic. Nav2's final velocity writers
(collision_monitor, docking_server) are remapped to <ns>/cmd_vel_watchdog_in; only the
watchdog writes <ns>/cmd_vel. Remove any other final cmd_vel writer (e.g. joystick) first.
No arm, task or movement command is sent by this launch.
"""
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from rescue_bringup.field_launch import load_field, robot_launch_values
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = robot_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                 robot_id=arg('robot_id'), occupancy_sha256=arg('occupancy_sha256'),
                                 now_ns=time.time_ns())
    ns, task = values['namespace'], values['task_input']
    nav2_params = arg('nav2_params_file') or str(
        Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'nav2.yaml')
    nav2 = GroupAction([
        *(SetRemap(src=src, dst=dst) for src, dst in values['nav2_remaps']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('turtlebot4_navigation')) / 'launch' / 'nav2.launch.py')),
            launch_arguments={'namespace': ns, 'use_sim_time': str(values['use_sim_time']).lower(),
                              'params_file': nav2_params}.items()),
    ])
    return [
        LogInfo(msg=f'robot {arg("robot_id")} ns={ns} nav2_params={nav2_params} '
                    f'map_sha256={task["task.saved_map.occupancy_sha256"]} declared alignment window '
                    f'{task["task.saved_map.valid_from_ns"]}..{task["task.saved_map.transform_valid_until_ns"]} ns'),
        nav2,
        Node(package='rescue_robot', executable='command_watchdog', name='rescue_command_watchdog', namespace=ns,
             output='screen', parameters=[typed(values['watchdog'])]),
        Node(package='rescue_robot', executable='task_input', name='rescue_command_adapter', namespace=ns,
             output='screen', parameters=[typed(task)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (field values, outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='Logical robot ID in the profile (robot_a, robot_b)'),
        DeclareLaunchArgument('occupancy_sha256', description='Measured hash of the map topic (map_occupancy_sha256.py)'),
        DeclareLaunchArgument('nav2_params_file', default_value='',
                              description='Nav2 parameters; empty = turtlebot4_navigation config/nav2.yaml (as #132)'),
        OpaqueFunction(function=_nodes),
    ])
