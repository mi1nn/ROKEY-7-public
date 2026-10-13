"""operations PC: db_writer (site-state DB + operator command API) and, optionally, the fixed webcam.

    ros2 launch rescue_bringup operations.launch.py profile_path:=<profile.json> \
        db_path:=<site_state.sqlite3> api_port:=<port> command_timeout_s:=3.0 \
        [webcam_params_file:=<webcam_node.yaml>]

Robot IDs, roles and topics come from the deployment profile (v2.1, mission roles); nothing
here has a default except the optional webcam. The console is not started here: it is a
web service, run with --site-state <db_path> --operations http://127.0.0.1:<api_port>.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.operations_launch import db_writer_parameters, parse_launch_arguments
from rescue_bringup.profile import load_deployment


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    deployment = load_deployment(arg('profile_path'))
    values = parse_launch_arguments(db_path=arg('db_path'), api_port=arg('api_port'),
                                    command_timeout_s=arg('command_timeout_s'))
    nodes = [Node(package='rescue_operations', executable='db_writer', name='rescue_db_writer', output='screen',
                  parameters=[db_writer_parameters(deployment, **values)])]
    webcam = arg('webcam_params_file')
    if webcam:
        nodes.append(Node(package='rescue_perception', executable='webcam_node', name='webcam_node',
                          output='screen', parameters=[webcam]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (field values, outside the repo)'),
        DeclareLaunchArgument('db_path', description='Site-state SQLite file; a new file per robot set'),
        DeclareLaunchArgument('api_port', description='Operator command API port on 127.0.0.1'),
        DeclareLaunchArgument('command_timeout_s', description='Coordinator answer wait, under the console 5 s'),
        DeclareLaunchArgument('webcam_params_file', default_value='',
                              description='webcam_node parameters YAML; empty = no webcam on this PC'),
        OpaqueFunction(function=_nodes),
    ])
