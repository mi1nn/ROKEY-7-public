"""TB-5 depth points for the passage stop polygon (tb5_passage_points.launch.py, no ROS import).

depth_image_proc PointCloudXyz turns the TB-5 OAK-D depth image into <ns>/passage_points,
which collision_monitor's passage_depth source reads (config/robots/tb5_nav2.yaml). The depth
and camera_info topic names are launch arguments without defaults: they are confirmed on the
TB-5 PC first (OPS-129).
"""


def passage_points_values(*, namespace, depth_topic, camera_info_topic):
    if not isinstance(namespace, str) or not namespace.startswith('/') or namespace.endswith('/'):
        raise ValueError('namespace must be absolute, e.g. /robot5')
    for name, topic in (('depth_topic', depth_topic), ('camera_info_topic', camera_info_topic)):
        if not isinstance(topic, str) or not topic.startswith(namespace + '/'):
            raise ValueError(f'{name} must be an absolute topic of this robot ({namespace}/...): {topic!r}')
    if depth_topic.endswith('/compressed'):
        raise ValueError('depth_topic must be a raw depth image (sensor_msgs/Image), not a compressed stream')
    return {'namespace': namespace, 'image_rect': depth_topic, 'camera_info': camera_info_topic,
            'points': namespace + '/passage_points'}
