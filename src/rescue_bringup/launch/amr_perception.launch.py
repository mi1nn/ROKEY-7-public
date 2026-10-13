"""AMR1 perception PC: amr_target (OAK-D RGB-D -> YOLO -> TargetObservation on /rescue/observations).

    ros2 launch rescue_bringup amr_perception.launch.py \
        image_topic:=<RGB topic> depth_topic:=<depth topic> camera_info_topic:=<CameraInfo topic> \
        rgb_optical_frame:=<RGB frame_id> depth_optical_frame:=<depth frame_id> \
        rgb_stamp_semantics:=<CAPTURE|DRIVER_RECEIVE|UNKNOWN> depth_stamp_semantics:=<...> \
        rgb_depth_alignment_verified:=<true|false> rgb_rectified:=<true|false> \
        calibration_id:=<id> alignment_evidence_id:=<id> \
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
    # Localization evidence: without known stamps and a verified, rectified RGB-Depth alignment no
    # detection gets a position (core.locate_detection). Values come from the camera configuration.
    'rgb_stamp_semantics': 'RGB header.stamp meaning: CAPTURE, DRIVER_RECEIVE or UNKNOWN',
    'depth_stamp_semantics': 'Depth header.stamp meaning: CAPTURE, DRIVER_RECEIVE or UNKNOWN',
    'rgb_depth_alignment_verified': 'true only when Depth is registered to the RGB pixel grid (checked)',
    'rgb_rectified': 'true only when the RGB image is rectified (CameraInfo D zero or a rect topic)',
    'calibration_id': 'ID of the camera calibration the alignment evidence refers to',
    'alignment_evidence_id': 'ID of the record that shows the RGB-Depth alignment',
}
FLAGS = ('rgb_depth_alignment_verified', 'rgb_rectified')


def _flag(name, text):
    if text not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false, got {text!r}')
    return text == 'true'


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    parameters = {name: arg(name) for name in ('image_topic', 'depth_topic', 'camera_info_topic',
                                               'rgb_optical_frame', 'depth_optical_frame',
                                               'rgb_stamp_semantics', 'depth_stamp_semantics',
                                               'calibration_id', 'alignment_evidence_id')}
    parameters.update({name: _flag(name, arg(name)) for name in FLAGS})
    return [Node(package='rescue_perception', executable='amr_target', name='amr_target', output='screen',
                 parameters=[parameters],
                 remappings=[('/tf', arg('tf_topic')), ('/tf_static', arg('tf_static_topic'))])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, description=text)
                              for name, text in FIELD_ARGUMENTS.items()] + [OpaqueFunction(function=_nodes)])
