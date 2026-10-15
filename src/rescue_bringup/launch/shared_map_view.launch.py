"""Read-only RViz: shared SLAM map, valid robot poses and the selected native model.

The selected robot defaults to SUPPORT (AMR2). Its own TF topics are remapped
into RViz; other robots are drawn from their already validated mission poses.
No static map->odom transform or robot-state publisher is invented here.
"""
import json
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import load_field
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed
from rescue_bringup.rviz_view import MARKER_TOPIC, rviz_config, view_values


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    values = view_values(load_deployment(arg('profile_path')), load_field(arg('field_path')),
                         robot_id=arg('robot_id'), show_all=arg('show_all') == 'true',
                         marker_topic=arg('marker_topic'), description_topic=arg('description_topic'))
    actions = [
        LogInfo(msg=f'shared view frame={values["frame"]} map={values["map_topic"]} '
                    f'model={values["description_topic"]} TF={values["tf_topic"]}; '
                    'model TF may be provisional; valid pose arrows require converged localization'),
        Node(package='rescue_robot', executable='shared_map_view', name='rescue_shared_map_view',
             output='screen', parameters=[typed(values['markers'] | {'use_sim_time': values['use_sim_time']})]),
    ]
    if arg('rviz') == 'true':
        with tempfile.NamedTemporaryFile(mode='w', prefix='rescue_shared_map_', suffix='.rviz', delete=False) as out:
            json.dump(rviz_config(values), out, indent=2)
        path = Path(out.name)

        def cleanup(_context):
            path.unlink(missing_ok=True)
            return []

        actions += [
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            Node(package='rviz2', executable='rviz2', name='rescue_map_rviz', namespace=values['namespace'],
                 output='screen', arguments=['-d', str(path)],
                 parameters=[{'use_sim_time': values['use_sim_time']}],
                 remappings=[('/tf', values['tf_topic']), ('/tf_static', values['tf_static_topic'])]),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON'),
        DeclareLaunchArgument('field_path', description='SLAM_LIVE field file'),
        DeclareLaunchArgument('robot_id', default_value='', description='Model and TF source; empty = SUPPORT'),
        DeclareLaunchArgument('show_all', default_value='true', choices=['true', 'false'],
                              description='Draw both valid mission poses, or only the selected robot'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Start RViz; false publishes pose markers only'),
        DeclareLaunchArgument('marker_topic', default_value=MARKER_TOPIC),
        DeclareLaunchArgument('description_topic', default_value='',
                              description='Empty = selected namespace/robot_description; must already be published'),
        OpaqueFunction(function=_nodes),
    ])
