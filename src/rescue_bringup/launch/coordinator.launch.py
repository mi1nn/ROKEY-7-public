"""Coordination PC: mission_coordinator with live task inputs (SEARCH_AREA over Nav2).

    ros2 launch rescue_bringup coordinator.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json \
        route_segment_goals:='[x, y, yaw, ...]' observe_goal:='[x, y, yaw]' \
        route_segment_passage_free:='[false, ...]' [camera_watch_roi:='[x0, y0, x1, y1]']

Robot IDs, roles and search areas (with entry_yaw) come from the profile, the run policy
from the field run file, and the Phase 1 route/observe goals from explicit arguments: they
are not settled yet (#132 ran with [0, 0, 0] placeholders), so nothing here defaults them.
No arm, mission or movement command is sent by this launch.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import COORDINATOR_NODE, coordinator_parameters, load_field
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    profile_path = arg('profile_path')
    params = coordinator_parameters(
        load_deployment(profile_path), profile_path=profile_path, field=load_field(arg('field_path')),
        route_segment_goals=arg('route_segment_goals'), observe_goal=arg('observe_goal'),
        route_segment_passage_free=arg('route_segment_passage_free'), camera_watch_roi=arg('camera_watch_roi'))
    return [Node(package='rescue_coordination', executable='mission_coordinator', name=COORDINATOR_NODE,
                 output='screen', parameters=[typed(params)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (field values, outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('route_segment_goals', description="Segment ends '[x, y, yaw, ...]' in the mission frame"),
        DeclareLaunchArgument('observe_goal', description="Observation point '[x, y, yaw]' in the mission frame"),
        DeclareLaunchArgument('route_segment_passage_free',
                              description="One field-checked bool per segment end, e.g. '[false]'"),
        DeclareLaunchArgument('camera_watch_roi', default_value='',
                              description="Webcam watch ROI '[x0, y0, x1, y1]' set at the site; empty = unset, "
                                          'the coordinator refuses camera watch requests'),
        OpaqueFunction(function=_nodes),
    ])
