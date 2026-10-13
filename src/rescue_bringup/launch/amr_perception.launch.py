"""AMR1 perception PC: amr_target (OAK-D RGB-D -> YOLO -> TargetObservation on /rescue/observations).

    ros2 launch rescue_bringup amr_perception.launch.py \
        image_topic:=<RGB topic> depth_topic:=<depth topic> camera_info_topic:=<CameraInfo topic> \
        rgb_optical_frame:=<RGB frame_id> depth_optical_frame:=<depth frame_id> \
        tf_topic:=<robot /tf> tf_static_topic:=<robot /tf_static>

Every argument is a field value read from the robot (`ros2 topic list`, the message header
frame_id); none has a default. The robot's TF is namespaced, so /tf and /tf_static are
remapped to the given topics for the capture-time camera -> base lookup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FIELD_ARGUMENTS = {
    'image_topic': 'RGB image topic (compressed), e.g. as listed on the robot',
    'depth_topic': 'Depth topic (compressedDepth)',
    'camera_info_topic': 'RGB CameraInfo topic',
    'rgb_optical_frame': 'frame_id in the RGB message header',
    'depth_optical_frame': 'frame_id in the depth message header',
    'tf_topic': 'The robot TF topic (namespaced)',
    'tf_static_topic': 'The robot static TF topic (namespaced)',
}


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    parameters = {name: arg(name) for name in ('image_topic', 'depth_topic', 'camera_info_topic',
                                               'rgb_optical_frame', 'depth_optical_frame')}
    return [Node(package='rescue_perception', executable='amr_target', name='amr_target', output='screen',
                 parameters=[parameters],
                 remappings=[('/tf', arg('tf_topic')), ('/tf_static', arg('tf_static_topic'))])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, description=text)
                              for name, text in FIELD_ARGUMENTS.items()] + [OpaqueFunction(function=_nodes)])
