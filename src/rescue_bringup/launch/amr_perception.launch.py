"""TB-6 perception PC: amr_target (OAK-D RGB-D -> YOLO -> TargetObservation on /rescue/observations)
and target_locator (capture-time TF -> TargetLocation on /rescue/target_locations).

    ros2 launch rescue_bringup amr_perception.launch.py \
        image_topic:=<RGB topic> depth_topic:=<depth topic> camera_info_topic:=<CameraInfo topic> \
        rgb_optical_frame:=<RGB frame_id> depth_optical_frame:=<depth frame_id> \
        rgb_stamp_semantics:=<CAPTURE|DRIVER_RECEIVE|UNKNOWN> depth_stamp_semantics:=<...> \
        rgb_depth_alignment_verified:=<true|false> rgb_rectified:=<true|false> \
        calibration_id:=<id> alignment_evidence_id:=<id> \
        sync_queue_size:=<RGB frames kept for a late Depth> sync_slop_ns:=<ns> camera_info_max_age_ns:=<ns> \
        tf_topic:=<robot /tf> tf_static_topic:=<robot /tf_static> \
        clock_domain_id:=<domain> clock_backend:=<timesyncd|chrony|NONE> clock_max_age_ns:=<ns> \
        clock_drift_ppm:=<ppm> clock_poll_period_s:=<s> \
        sensor_host_clock_error_bound_ns:=<ns measured on the camera driver host> sensor_host_clock_domain_id:=<domain> \
        profile_path:=<profile.json> tf_wait_s:=<seconds> \
        alignment_authority_id:=<who declared the map alignment> alignment_valid_from_ns:=<ns> alignment_valid_until_ns:=<ns> \
        recheck_action_name:=<ObserveTarget action> recheck_map_frame:=<map frame> recheck_scan_topic:=<robot scan> \
        recheck_*:=<recheck window policy, see rescue_perception.recheck_feed.RECHECK_PARAMS>

Every argument is a field value read from the robot (`ros2 topic list`, the message header
frame_id); none has a default. The robot's TF is namespaced, so /tf and /tf_static are
remapped to the given topics for the capture-time TF lookups. target_locator takes the map
basis and the SCOUT robot from the deployment profile's map_basis block, the same block the
coordinator reads; the launch fails without it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.profile import load_deployment, require_map_basis
from rescue_perception.target_locator import profile_parameters
from rescue_perception.recheck_feed import RECHECK_PARAMS

FIELD_ARGUMENTS = {
    'image_topic': 'RGB image topic, e.g. as listed on the robot',
    'image_type': 'RGB message type: raw (sensor_msgs/Image) or compressed (CompressedImage)',
    'depth_transport': 'Depth message: raw (16UC1/32FC1 Image) or compressedDepth (CompressedImage)',
    'depth_topic': 'Depth topic (registered to RGB; its transport is depth_transport)',
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
    # Pairing: Depth may arrive well after RGB (TB-6 measured about 0.78 s, RGB 29.7 Hz, 2026-10-13).
    'sync_queue_size': 'RGB/Depth messages kept for pairing; must cover the Depth delay at the RGB rate',
    'sync_slop_ns': 'largest RGB-Depth stamp difference paired (and the localization skew limit), ns',
    'camera_info_max_age_ns': 'largest CameraInfo-RGB stamp difference when the pair is processed, ns',
    # Observation clock measured on this host, like the webcam (OPS-103 decision 1).
    'clock_domain_id': 'Measured clock domain written into the observations (field value)',
    'clock_backend': 'timesyncd, chrony or NONE (never measured: every observation UNKNOWN)',
    'clock_max_age_ns': 'NTP reply older than this -> UNKNOWN',
    'clock_drift_ppm': 'clock drift bound added after the measurement',
    'clock_poll_period_s': 'clock measurement read period',
    'sensor_host_clock_error_bound_ns': 'sync error bound of the host whose clock stamps the images (the camera '
                                        'driver host, TB-6 Pi), measured there; 0 only when the driver runs here',
    'sensor_host_clock_domain_id': 'clock domain of that host; the same value only when it is synced to the '
                                   'same reference',
    'profile_path': 'Deployment profile v2 JSON with the map_basis block (shared with the coordinator)',
    'tf_wait_s': 'How long target_locator waits for the TF at a capture time',
    # OPS-103 decision 2 (C-04): the alignment authority (profile map_basis + the robot's saved-map identity)
    # and the window it holds for; target_locator publishes nothing VALID outside it.
    'alignment_authority_id': 'ID of the declared map -> mission alignment (field record)',
    'alignment_valid_from_ns': 'start of the declared alignment window, ns (this run)',
    'alignment_valid_until_ns': 'end of the declared alignment window, ns',
}
CLOCK_TYPES = {'clock_domain_id': str, 'clock_backend': str, 'clock_max_age_ns': int, 'clock_drift_ppm': float,
               'clock_poll_period_s': float, 'sensor_host_clock_error_bound_ns': int,
               'sensor_host_clock_domain_id': str}
INTEGERS = ('sync_queue_size', 'sync_slop_ns', 'camera_info_max_age_ns')
FLAGS = ('rgb_depth_alignment_verified', 'rgb_rectified')


def _integer(name, text):
    if not text.isdigit():
        raise ValueError(f'{name} must be a non-negative integer, got {text!r}')
    return int(text)


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
                                               'calibration_id', 'alignment_evidence_id',
                                               'image_type', 'depth_transport')}
    parameters.update({name: _flag(name, arg(name)) for name in FLAGS})
    parameters.update({name: _integer(name, arg(name)) for name in INTEGERS})
    for name, kind in RECHECK_PARAMS.items():   # TB-6 recheck window (ObserveTarget server in amr_target)
        text = arg(name)
        try:
            parameters[name] = {'STRING': str, 'INTEGER': int, 'DOUBLE': float}[kind](text)
        except ValueError as exc:
            raise ValueError(f'{name} must be {kind}, got {text!r}') from exc
    for name, kind in CLOCK_TYPES.items():
        try:
            parameters[name] = kind(arg(name))
        except ValueError as exc:
            raise ValueError(f'{name} must be {kind.__name__}, got {arg(name)!r}') from exc
    deployment = load_deployment(arg('profile_path'))
    locator = profile_parameters(deployment, require_map_basis(deployment), tf_wait_s=float(arg('tf_wait_s')),
                                 clock_domain_id=arg('clock_domain_id'),   # the same measured domain as amr_target
                                 alignment_authority_id=arg('alignment_authority_id'),
                                 alignment_valid_from_ns=_integer('alignment_valid_from_ns',
                                                                  arg('alignment_valid_from_ns')),
                                 alignment_valid_until_ns=_integer('alignment_valid_until_ns',
                                                                   arg('alignment_valid_until_ns')))
    tf = [('/tf', arg('tf_topic')), ('/tf_static', arg('tf_static_topic'))]
    return [Node(package='rescue_perception', executable='amr_target', name='amr_target', output='screen',
                 parameters=[parameters], remappings=tf),
            Node(package='rescue_perception', executable='target_locator', name='target_locator', output='screen',
                 parameters=[locator], remappings=tf)]


def generate_launch_description():
    recheck = [DeclareLaunchArgument(name, description=f'TB-6 recheck: {kind} (recheck_feed.RECHECK_PARAMS)')
               for name, kind in RECHECK_PARAMS.items()]
    return LaunchDescription([DeclareLaunchArgument(name, description=text)
                              for name, text in FIELD_ARGUMENTS.items()] + recheck + [OpaqueFunction(function=_nodes)])
