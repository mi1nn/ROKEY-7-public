"""One-PC wiring rehearsal: every node of this repository the field launches start, with the real namespaces.

Run by run.sh (it sets a local Discovery Server first; read it before running this alone).

- Whole launch files of this repository: operations.launch.py (no webcam), coordinator.launch.py,
  map_freeze.launch.py, overhead_passage.launch.py for both robots.
- robot.launch.py and frozen_localization.launch.py also start Nav2 / AMCL (turtlebot4_navigation,
  nav2_amcl), which this rehearsal PC may not have. Their nodes of this repository are started here
  with the same values (robot_launch_values, frozen_localization_values); Nav2 / AMCL / SLAM / the
  depth container / the robot Pi are fake nodes (fake_graph.py).
No arm, task or movement command is sent; there is no robot on this graph.
"""
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import frozen_localization_values, load_field, robot_launch_values
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed

SHIM = Path(__file__).resolve().parent / 'host_shim'
PLACEHOLDERS = dict(route_segment_goals='[0.0, 0.0, 0.0]', observe_goal='[0.0, 0.0, 0.0]',
                    route_segment_passage_free='[false]')


def _include(name, **arguments):
    path = Path(get_package_share_directory('rescue_bringup')) / 'launch' / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    profile, field_path, out = arg('profile_path'), arg('field_path'), arg('out_dir')
    phase = arg('slam_phase')
    deployment, field = load_deployment(profile), load_field(field_path)
    common = dict(profile_path=profile, field_path=field_path)
    actions = [
        _include('operations.launch.py', profile_path=profile, db_path=os.path.join(out, 'site_state.sqlite3'),
                 api_port=arg('api_port'), command_timeout_s='3.0',
                 report_db_path=os.path.join(out, 'reports.sqlite3'), envelope_schema_version='rescue-envelope/1',
                 report_publisher_id='wiring-rehearsal'),
        _include('coordinator.launch.py', **common, **PLACEHOLDERS),
        _include('map_freeze.launch.py', **common, output_dir=os.path.join(out, 'freeze')),
    ]
    for robot in [r.id for r in deployment.robots]:
        actions.append(_include('overhead_passage.launch.py', **common, robot_id=robot))
        values = robot_launch_values(deployment, field=field, robot_id=robot, occupancy_sha256='',
                                     now_ns=time.time_ns())
        ns = values['namespace']
        # robot.launch.py's own nodes, same names and values (its Nav2 include is fake_graph.py here)
        actions += [Node(package='rescue_robot', executable='command_watchdog', name='rescue_command_watchdog',
                         namespace=ns, output='screen', parameters=[typed(values['watchdog'])]),
                    Node(package='rescue_robot', executable='task_input', name='rescue_command_adapter',
                         namespace=ns, output='screen', parameters=[typed(values['task_input'])],
                         # rehearsal only: a host id of its own, so the one-PC clock check sees two hosts
                         additional_env={'REHEARSAL_HOST_ID': 'rehearsal-' + robot,
                                         'PYTHONPATH': os.pathsep.join([str(SHIM), os.environ.get('PYTHONPATH', '')])})]
        mode = field.robots[robot]['task.saved_map.mode']
        if mode == 'FROZEN_MAP_AMCL' or (mode == 'SLAM_THEN_FROZEN' and phase == 'FROZEN'):
            loc = frozen_localization_values(deployment, field=field, robot_id=robot)
            # frozen_localization.launch.py's relay, same name and values (AMCL is fake_graph.py here)
            actions.append(Node(package='rescue_robot', executable='frozen_map_relay', name='rescue_frozen_map_relay',
                                namespace=loc['namespace'], output='screen', parameters=[typed(
                                    {key: loc[key] for key in ('frozen_topic', 'frozen_revision_topic',
                                                               'map_state_topic', 'map_topic')})]))
    return [LogInfo(msg=f'wiring rehearsal: slam_phase={phase} out={out}'), *actions]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path'), DeclareLaunchArgument('field_path'),
        DeclareLaunchArgument('out_dir', description='absolute scratch directory of this rehearsal'),
        DeclareLaunchArgument('api_port', description='operations command API port on 127.0.0.1'),
        DeclareLaunchArgument('slam_phase', description='SEARCHING or FROZEN (TB-6 map)'),
        OpaqueFunction(function=_nodes),
    ])
