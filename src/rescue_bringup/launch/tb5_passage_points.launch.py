"""TB-5 depth image -> <ns>/passage_points for the collision_monitor passage stop (OPS-128).

    ros2 launch rescue_bringup tb5_passage_points.launch.py profile_path:=<profile.json> robot_id:=robot_b \
        depth_topic:=<TB-5 depth image topic> camera_info_topic:=<its camera_info>

Runs depth_image_proc PointCloudXyzNode on the TB-5 PC (needs depth_image_proc installed there;
do not install it without the owner's approval). Topic names have no default: confirm them on
the TB-5 PC (ros2 topic list --no-daemon | grep oakd). Enabling OAK-D depth is a robot setting
(grade 4) and is not done here. No arm, task or movement command is sent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from rescue_bringup.passage_launch import passage_points_values
from rescue_bringup.profile import load_deployment


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robots = {robot.id: robot for robot in load_deployment(arg('profile_path')).robots}
    if arg('robot_id') not in robots:
        raise ValueError(f'robot_id {arg("robot_id")!r} is not in the profile ({sorted(robots)})')
    v = passage_points_values(namespace=robots[arg('robot_id')].namespace, depth_topic=arg('depth_topic'),
                              camera_info_topic=arg('camera_info_topic'))
    return [
        LogInfo(msg=f'passage points {v["image_rect"]} + {v["camera_info"]} -> {v["points"]}'),
        ComposableNodeContainer(
            name='passage_points_container', namespace=v['namespace'], package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzNode', name='passage_points',
                namespace=v['namespace'],
                remappings=[('image_rect', v['image_rect']), ('camera_info', v['camera_info']),
                            ('points', v['points'])])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('robot_id', description='robot_b (TB-5)'),
        DeclareLaunchArgument('depth_topic', description='TB-5 raw depth image topic (confirmed on the TB-5 PC)'),
        DeclareLaunchArgument('camera_info_topic', description='camera_info of that depth image'),
        OpaqueFunction(function=_nodes),
    ])
