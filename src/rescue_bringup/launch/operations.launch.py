"""operations PC: db_writer (site-state DB + operator command API), report_node (RescueReport ->
ReportReceipt) and, optionally, the fixed webcam and its judge video relay (MJPEG for the console).

    ros2 launch rescue_bringup operations.launch.py profile_path:=<profile.json> \
        db_path:=<site_state.sqlite3> api_port:=<port> command_timeout_s:=3.0 \
        report_db_path:=<reports.sqlite3> envelope_schema_version:=<text> report_publisher_id:=<text> \
        [webcam_params_file:=<webcam_node.yaml> [webcam_device:=<name in /dev/v4l/by-id>] \
         [video_port:=<port> video_bind_host:=<address> video_stale_s:=<s>]] \
        [map_topic:=/robot6/map map_dir:=<folder>]

map_topic/map_dir write the SLAM map (owner 2026-10-13: TB-6) for the console --live-map; the
launch logs the console value.

webcam_device picks another plugged camera (e.g. a USB one) over the YAML device_path; the
launch log lists the plugged cameras.

Robot IDs, roles and topics come from the deployment profile (v2.1, mission roles); nothing
here has a default except the optional webcam and video relay. The console is not started here:
it is a web service, run with --site-state <db_path> --operations http://127.0.0.1:<api_port> and,
with the relay, the --camera value this launch logs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rescue_bringup.operations_launch import (available_cameras, console_camera_argument, db_writer_parameters,
                                              map_snapshot_parameters, mjpeg_relay_parameters, parse_launch_arguments,
                                              read_webcam_parameters, relay_requested, report_node_parameters,
                                              webcam_device)
from rescue_bringup.profile import load_deployment


def _text_as_text(parameters):
    """launch_ros reads a plain str as YAML ('0.1' -> DOUBLE); keep text for STRING parameters."""
    return {name: ParameterValue(value, value_type=str) if isinstance(value, str) else value
            for name, value in parameters.items()}


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    deployment = load_deployment(arg('profile_path'))
    values = parse_launch_arguments(db_path=arg('db_path'), api_port=arg('api_port'),
                                    command_timeout_s=arg('command_timeout_s'))
    nodes = [Node(package='rescue_operations', executable='db_writer', name='rescue_db_writer', output='screen',
                  parameters=[db_writer_parameters(deployment, **values)]),
             Node(package='rescue_operations', executable='report_node', name='rescue_report_node', output='screen',
                  parameters=[_text_as_text(report_node_parameters(
                      db_path=values['db_path'], report_db_path=arg('report_db_path'),
                      envelope_schema_version=arg('envelope_schema_version'),
                      publisher_id=arg('report_publisher_id')))])]
    webcam = arg('webcam_params_file')
    relay = relay_requested(video_port=arg('video_port'), webcam_params_file=webcam)
    if webcam:
        # A later parameter dict overrides the YAML's device_path.
        device = webcam_device(arg('webcam_device'))
        nodes += [LogInfo(msg='plugged cameras: ' + (', '.join(available_cameras()) or 'none')),
                  Node(package='rescue_perception', executable='webcam_node', name='webcam_node', output='screen',
                       parameters=[webcam] + ([{'device_path': ParameterValue(device, value_type=str)}]
                                              if device else []))]
    if relay:
        settings = read_webcam_parameters(webcam)
        params = mjpeg_relay_parameters(webcam=settings, bind_host=arg('video_bind_host'), port=arg('video_port'),
                                        stale_s=arg('video_stale_s'))
        nodes += [Node(package='rescue_perception', executable='mjpeg_relay', name='mjpeg_relay', output='screen',
                       parameters=[_text_as_text(params)]),
                  LogInfo(msg='console --camera ' + console_camera_argument(params, camera_id=settings['camera_id']))]
    snapshot = map_snapshot_parameters(map_topic=arg('map_topic'), map_dir=arg('map_dir'))
    if snapshot:
        nodes += [Node(package='rescue_operations', executable='map_snapshot', name='rescue_map_snapshot',
                       output='screen', parameters=[_text_as_text(snapshot)]),
                  LogInfo(msg=f'console --live-map {snapshot["out_dir"]}/live_map.yaml --live-map-stale-s <s>')]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (field values, outside the repo)'),
        DeclareLaunchArgument('db_path', description='Site-state SQLite file; a new file per robot set'),
        DeclareLaunchArgument('api_port', description='Operator command API port on 127.0.0.1'),
        DeclareLaunchArgument('command_timeout_s', description='Coordinator answer wait, under the console 5 s'),
        DeclareLaunchArgument('report_db_path', description='Rescue report store SQLite file; not the db_path file'),
        DeclareLaunchArgument('envelope_schema_version', description='ReportReceipt envelope schema_version text'),
        DeclareLaunchArgument('report_publisher_id', description='ReportReceipt envelope publisher_id'),
        DeclareLaunchArgument('webcam_params_file', default_value='',
                              description='webcam_node parameters YAML; empty = no webcam on this PC'),
        DeclareLaunchArgument('webcam_device', default_value='',
                              description='Camera to open instead of the YAML device_path: a name from '
                                          '`ls /dev/v4l/by-id/` (*-video-index0) or a path; empty = the YAML one'),
        DeclareLaunchArgument('video_port', default_value='',
                              description='Judge video MJPEG relay port; empty = no relay (needs the webcam)'),
        DeclareLaunchArgument('video_bind_host', default_value='',
                              description='Relay listen address, required with video_port (the console browser '
                                          'must reach it)'),
        DeclareLaunchArgument('video_stale_s', default_value='',
                              description='Relay: no new frame for this long closes the stream; required with video_port'),
        DeclareLaunchArgument('map_topic', default_value='',
                              description='SLAM map topic (OccupancyGrid) for the console, e.g. /robot6/map; '
                                          'empty = no map relay (needs map_dir)'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description='Folder the map relay writes live_map.yaml into (console --live-map)'),
        OpaqueFunction(function=_nodes),
    ])
