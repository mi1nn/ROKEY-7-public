"""TB-6 start pose and SLAM correction watch for the live shared map (방식 B); SLAM keeps its own latest map.

    ros2 launch rescue_bringup live_map.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/live_map.overlay.json output_dir:=<new absolute run folder>

Use with tb6_slam.launch.py in SLAM_LIVE mode, started before TB-6 moves. No frozen map, no map_server,
no PGM transfer and no SLAM->AMCL switch. output_dir holds only start_pose.json (rescue_slam_start_pose:
/rescue/map/start_pose/<TB-6 id>, the origin of TB-5's start seed and TB-6's return goal). rescue_map_correction
publishes map -> odom jumps at TB-6 on /rescue/map/correction (logged by the coordinator). rescue_slam_pose_latest
repeats SLAM's last <ns>/pose with its own stamp on <ns>/rescue/slam_pose_latest, the executor's covariance input
(a standing TB-6 gets one SLAM pose only).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rescue_bringup.field_launch import (FieldLaunchError, load_field, map_correction_values, map_freeze_parameters,
                                         slam_pose_latest_values)
from rescue_bringup.profile import load_deployment
from rescue_bringup.ros_parameters import typed


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    field = load_field(arg('field_path'))
    if field.coordinator['map_mode'] != 'SLAM_LIVE':
        raise FieldLaunchError('live_map.launch.py requires SLAM_LIVE')
    deployment = load_deployment(arg('profile_path'))
    params = map_freeze_parameters(deployment, field=field, output_dir=arg('output_dir'))
    return [Node(package='rescue_robot', executable='slam_start_pose', output='screen', parameters=[typed(params)]),
            Node(package='rescue_robot', executable='map_correction', output='screen',
                 parameters=[typed(map_correction_values(deployment, field=field))]),
            Node(package='rescue_robot', executable='slam_pose_latest', output='screen',
                 parameters=[typed(slam_pose_latest_values(deployment, field=field))])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile JSON'),
        DeclareLaunchArgument('field_path', description='SLAM_LIVE field file'),
        DeclareLaunchArgument('output_dir', description='New run directory for the measured scout start pose'),
        OpaqueFunction(function=_nodes),
    ])
