"""Depth image -> <ns>/passage_points for the Nav2 collision_monitor passage stop (OPS-128/135).

    ros2 launch rescue_bringup passage_points.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a

The depth and camera_info topics and the transport come from the field run file
(robots.<id>.passage.*). Runs depth_image_proc PointCloudXyzNode on that robot's PC (install
depth_image_proc there only with the owner's approval). Enabling OAK-D depth is a robot
setting (grade 4) and is not done here. No arm, task or movement command is sent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from rescue_bringup.field_launch import load_field, passage_points_launch_values
from rescue_bringup.profile import load_deployment


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    v = passage_points_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                     robot_id=arg('robot_id'))
    return [
        LogInfo(msg=f'passage points {v["image_rect"]} ({v["depth_transport"]}) + {v["camera_info"]} -> {v["points"]}'),
        ComposableNodeContainer(
            name='passage_points_container', namespace=v['namespace'], package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzNode', name='passage_points',
                namespace=v['namespace'],
                # image_transport of the depth subscription [추정: depth_image_proc parameter name, not run here]
                parameters=[{'depth_image_transport': v['depth_transport']}],
                remappings=[('image_rect', v['image_rect']), ('camera_info', v['camera_info']),
                            ('points', v['points'])])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='robot_a (TB-6) or robot_b (TB-5)'),
        OpaqueFunction(function=_nodes),
    ])
