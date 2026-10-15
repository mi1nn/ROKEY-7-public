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
There is no depth stop (재범 decision 2026-10-16, option A): the passage slowdown (field run file
robot.passage.*) is added to the Nav2 collision_monitor, and both costmaps read the tunnel keepout mask
(KeepoutFilter) that rescue_robot passage_keepout paints from the vision judgement (overhead_passage
BLOCKED). The robot_a launch also starts passage_keepout, one node for both robots. Run
passage_points.launch.py for the depth points. A robot without robots.<id>.passage depth values does not
start. A FROZEN_MAP_AMCL robot (TB-5) has no map frame before the first freeze,
so its Nav2 starts with autostart false and nav2_start_after_map sends STARTUP once map -> base_link
is on its TF (started at once, the global costmap gives up after 60 s and Nav2 stays inactive, #375).
No arm, task or movement command is sent by this launch.
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

from rescue_bringup.field_launch import (amcl_update_thresholds, load_field, nav2_with_passage,
                                        passage_keepout_launch_values, robot_launch_values)
from rescue_bringup.nav2_field import (KEEPOUT_INFO_TOPIC, field_nav2, nav2_with_depth_low_layer,
                                       nav2_with_tunnel_keepout)
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = robot_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                 robot_id=arg('robot_id'), occupancy_sha256=arg('occupancy_sha256'),
                                 now_ns=time.time_ns())
    ns, task = values['namespace'], values['task_input']
    localization_params = None
    if task['task.saved_map.mode'] != 'SAVED_MAP_AMCL':
        # the frozen-phase TF pose uses AMCL's own update thresholds: the same file frozen_localization.launch.py uses
        localization_params = arg('localization_params_file') or str(
            Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'localization.yaml')
        task = {**task, **amcl_update_thresholds(yaml.safe_load(Path(localization_params).read_text()))}
    base_params = arg('nav2_params_file') or str(
        Path(get_package_share_directory('turtlebot4_navigation')) / 'config' / 'nav2.yaml')
    # The passage slowdown (field run file robot.passage.*, the same for both robots) is added to collision_monitor
    # here, so no yaml carries its own copy (OPS-135). No depth stop: tunnels are kept out by the vision judgement.
    passage = values['passage']
    # no backing up in recovery (OPS-207) and FollowPath Rotation Shim + RPP instead of DWB (2026-10-14), both robots
    # tunnels kept out by the vision judgement (A) and low objects under the lidar plane by depth (D), 2026-10-16
    params = nav2_with_depth_low_layer(nav2_with_tunnel_keepout(
        nav2_with_passage(field_nav2(yaml.safe_load(Path(base_params).read_text())), passage)), passage)
    keepout = []
    if arg('robot_id') == 'robot_a':   # one mask for both robots; TB-6's launch always runs
        k = passage_keepout_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')))
        keepout = [Node(package='rescue_robot', executable='passage_keepout', name='rescue_passage_keepout',
                        output='screen', parameters=[typed(k)])]
    with tempfile.NamedTemporaryFile('w', prefix=f'nav2_{arg("robot_id")}_', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)
    nav2_params = out.name
    nav2 = GroupAction([
        *(SetRemap(src=src, dst=dst) for src, dst in values['nav2_remaps']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('turtlebot4_navigation')) / 'launch' / 'nav2.launch.py')),
            # autostart reaches navigation_launch.py through the launch context (nav2.launch.py does not declare it)
            launch_arguments={'namespace': ns, 'use_sim_time': str(values['use_sim_time']).lower(),
                              'params_file': nav2_params,
                              'autostart': str(values['nav2_autostart']).lower()}.items()),
    ])
    start = values['nav2_start']
    nav2_start = [] if start is None else [
        LogInfo(msg=f'nav2 autostart false: {start["manage_service"]} STARTUP once {start["map_frame"]} -> '
                    f'{start["base_frame"]} is on {start["tf_topic"]} (first frozen map localized)'),
        Node(package='rescue_bringup', executable='nav2_start_after_map', name='rescue_nav2_start', namespace=ns,
             output='screen', remappings=[('/tf', start['tf_topic']), ('/tf_static', start['tf_static_topic'])],
             parameters=[typed({key: start[key] for key in (
                 'manage_service', 'map_frame', 'base_frame', 'poll_period_s', 'answer_wait_s')})])]
    return [
        LogInfo(msg=f'robot {arg("robot_id")} ns={ns} nav2_params={base_params} + no depth stop '
                    f'+ tunnel keepout {KEEPOUT_INFO_TOPIC}' + (' (passage_keepout here)' if keepout else '') + ' '
                    f'+ depth_low_layer {passage["min_height_m"]}..{passage["overhead_lidar_plane_m"]} m above the floor '
                    f'+ passage slow x{passage["slow_ratio"]} {passage["overhead_lidar_plane_m"]}..'
                    f'{passage["overhead_top_m"]} m front {passage["slow_front_m"]} '
                    f'depth {passage["depth_topic"]} ({passage["depth_transport"]}) + no BackUp recovery + FollowPath Rotation Shim/RPP -> {nav2_params} '
                    f'map_mode={task["task.saved_map.mode"]} '
                    + (f'frozen pose TF {task["task.saved_map.amcl_tf_topic"]} with AMCL covariance while moved < '
                       f'{task["task.saved_map.amcl_update_min_d"]} m / {task["task.saved_map.amcl_update_min_a"]} rad '
                       f'({localization_params}) ' if localization_params else '') +
                    f'map_sha256={task["task.saved_map.occupancy_sha256"] or "(decided by the freeze)"} '
                    f'declared alignment window '
                    f'{task["task.saved_map.valid_from_ns"]}..{task["task.saved_map.transform_valid_until_ns"]} ns'),
        nav2,
        *keepout,
        *nav2_start,
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
        DeclareLaunchArgument('localization_params_file', default_value='',
                              description='AMCL parameters frozen_localization.launch.py uses (give the same file); '
                                          'empty = turtlebot4_navigation config/localization.yaml'),
        OpaqueFunction(function=_nodes),
    ])
