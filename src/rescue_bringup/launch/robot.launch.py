"""Robot side: TurtleBot4 Nav2 + command_watchdog + task_input (SEARCH_AREA, live inputs).

    ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a

The map mode is the field run file's robots.<id>.task.saved_map.mode (map lifecycle, OPS-126):
SLAM_THEN_FROZEN (TB-6: tb6_slam.launch.py while searching, then frozen_localization.launch.py)
or FROZEN_MAP_AMCL (TB-5: frozen_localization.launch.py) take no occupancy_sha256, the freeze
decides the hash. SAVED_MAP_AMCL (tests without robots, #132) needs the measured hash:
occupancy_sha256:=$(python3 scripts/field/map_occupancy_sha256.py /robot6/map map) after
turtlebot4_navigation localization.launch.py with the saved map. Nav2's final velocity writers
(collision_monitor, docking_server) are remapped to <ns>/cmd_vel_watchdog_in; only the
watchdog writes <ns>/cmd_vel. Remove any other final cmd_vel writer (e.g. joystick) first.
The overhead passage stop (field run file robot.passage.*, OPS-135) is added to the Nav2
collision_monitor parameters at launch; run passage_points.launch.py for the depth points, or
collision_monitor stops the robot (no depth = no movement). A robot without robots.<id>.passage
depth values does not start. No arm, task or movement command is sent by this launch.
"""
from pathlib import Path
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

import yaml

from rescue_bringup.field_launch import load_field, nav2_with_passage, robot_launch_values
from rescue_bringup.nav2_field import field_nav2
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = robot_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                 robot_id=arg('robot_id'), occupancy_sha256=arg('occupancy_sha256'),
                                 now_ns=time.time_ns())
    ns, task = values['namespace'], values['task_input']
    base_params = arg('nav2_params_file') or str(
        Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'nav2.yaml')
    # The passage stop (field run file robot.passage.*, the same for both robots) is added to collision_monitor
    # here, so no yaml carries its own copy of 0.46 m (OPS-135).
    passage = values['passage']
    # no backing up in recovery (OPS-207) and FollowPath Rotation Shim + RPP instead of DWB (2026-10-14), both robots
    params = nav2_with_passage(field_nav2(yaml.safe_load(Path(base_params).read_text())), passage)
    with tempfile.NamedTemporaryFile('w', prefix=f'nav2_{arg("robot_id")}_', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)
    nav2_params = out.name
    nav2 = GroupAction([
        *(SetRemap(src=src, dst=dst) for src, dst in values['nav2_remaps']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('turtlebot4_navigation')) / 'launch' / 'nav2.launch.py')),
            launch_arguments={'namespace': ns, 'use_sim_time': str(values['use_sim_time']).lower(),
                              'params_file': nav2_params}.items()),
    ])
    return [
        LogInfo(msg=f'robot {arg("robot_id")} ns={ns} nav2_params={base_params} + passage stop '
                    f'{passage["min_height_m"]}..{passage["max_height_m"]} m front {passage["front_m"]} '
                    f'+ passage slow x{passage["slow_ratio"]} {passage["overhead_lidar_plane_m"]}..'
                    f'{passage["overhead_top_m"]} m front {passage["slow_front_m"]} '
                    f'depth {passage["depth_topic"]} ({passage["depth_transport"]}) + no BackUp recovery + FollowPath Rotation Shim/RPP -> {nav2_params} '
                    f'map_mode={task["task.saved_map.mode"]} '
                    f'map_sha256={task["task.saved_map.occupancy_sha256"] or "(decided by the freeze)"} '
                    f'declared alignment window '
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
        DeclareLaunchArgument('occupancy_sha256', default_value='',
                              description='SAVED_MAP_AMCL only: measured hash of the map topic (map_occupancy_sha256.py); '
                                          'empty for the SLAM -> frozen map modes'),
        DeclareLaunchArgument('nav2_params_file', default_value='',
                              description='Nav2 parameters; empty = turtlebot4_navigation config/nav2.yaml (as #132)'),
        OpaqueFunction(function=_nodes),
    ])
