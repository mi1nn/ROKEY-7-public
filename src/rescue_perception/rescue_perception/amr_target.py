"""AMR RGB-D observation node: detection, depth correction and TargetObservation.

Map projection is separate code for later connection to existing rescue_mapping.
Camera-to-base TF produces auxiliary relative points; no map/odom lookup
or TargetLocation publisher is connected.
"""
import json
import os
import sys
import time
from uuid import uuid4

import cv2
import rclpy
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from rescue_interfaces.msg import TargetObservation
from ultralytics import YOLO
from rclpy.executors import ExternalShutdownException

from .camera_utils import (PreviewProcessor, field_values, git_sha, read_field_parameters,
    validate_labels, verify_model)
from .target_utils import (FIELD_PARAMS, SETTINGS, target_config, PreparedDetector, OneShotTarget, fill_envelope,
    fill_observation, fill_base_point, inference_from_result, decode_depth_png)
from .adapter import stamp_ns
from .core import ModelManifest
from .person_pipeline import PersonPipeline


def create_node(*, model_factory=None, transform_lookup=None):
    """Create the RGB-D observation node; mapping integration is a later step."""

    node = Node('amr_target')
    try:
        v = field_values(SETTINGS, read_field_parameters(node, FIELD_PARAMS), FIELD_PARAMS)
        boot_id = str(uuid4())
        binding, alignment, depth_policy, expiry, confirmation, covariance = target_config(v, boot_id)
        topic_keys = ('image_topic', 'depth_topic', 'camera_info_topic', 'annotated_topic',
                      'observations_topic', 'base_point_topic')
        topics = {key: node.resolve_topic_name(v[key]) for key in topic_keys}
        if len(set(topics.values())) != len(topics):
            raise ValueError('remapped input/output topics must differ')
        path, model_hash = verify_model(v['model_path'], v['model_sha256'])
        if model_factory is None:
            model_factory = YOLO
        model = model_factory(path)
        validate_labels(model.names, v['model_label'])
        bridge = CvBridge()
        preview = PreviewProcessor(model, bridge, v)
        manifest = ModelManifest(v['model_id'], model_hash, (v['model_label'],))
        detector = PreparedDetector()
        pipeline = PersonPipeline(binding=binding, alignment=alignment, rectified=v['rgb_rectified'],
            depth_policy=depth_policy, observation_policy=expiry, confirmation_policy=confirmation, detector=detector)
        if v['show']:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise ValueError('show=true requires a desktop display')
            cv2.namedWindow('AMR YOLO target', cv2.WINDOW_NORMAL)
        sensor_qos = QoSProfile(depth=v['sync_queue_size'], reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        observation_pub = node.create_publisher(TargetObservation, topics['observations_topic'],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        base_point_pub = node.create_publisher(PointStamped, topics['base_point_topic'],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if transform_lookup is None:
            buffer = Buffer(node=node)
            node.relative_tf_listener = TransformListener(buffer, node, spin_thread=False)
            transform_lookup = lambda target, source, stamp: buffer.lookup_transform(
                target, source, Time(nanoseconds=stamp))
        image_pub = node.create_publisher(Image, topics['annotated_topic'], 1)
        state = {'info': None, 'last_success': time.monotonic(),
                 'warning': 0., 'frames': 0, 'observation_seq': 0}

        sender = OneShotTarget()

        def envelope(message, key, now):
            state[key] += 1
            fill_envelope(message, version=v['envelope_version'], publisher_id=v['publisher_id'],
                          boot_id=boot_id, sequence=state[key], now_ns=now)

        node.create_subscription(CameraInfo, topics['camera_info_topic'],
                                 lambda message: state.update(info=message), sensor_qos)


        def publish_target(o):
            observation = fill_observation(TargetObservation(), o, covariance)
            now = node.get_clock().now().nanoseconds
            envelope(observation.envelope, 'observation_seq', now)
            observation_pub.publish(observation)
            try:
                base_point = fill_base_point(PointStamped(), o, transform_lookup,
                                             base_frame=v['base_frame'])
                base_point_pub.publish(base_point)
            except Exception as exc:
                node.get_logger().warning(
                    f'base point unavailable; camera observation {o.observation_id} retained: '
                    f'{type(exc).__name__}: {exc}')

        def on_pair(rgb_message, depth_message):
            try:
                info = state['info']
                stamp = stamp_ns(rgb_message.header, 'rgb')
                if info is None or abs(stamp - stamp_ns(info.header, 'camera_info')) > v['camera_info_max_age_ns']:
                    raise ValueError('fresh matching CameraInfo required')
                if rgb_message.header.frame_id != v['rgb_optical_frame'] or depth_message.header.frame_id != v['depth_optical_frame']:
                    raise ValueError('RGB/Depth optical frames differ from configured alignment evidence')
                frame, result = preview.infer(rgb_message)
                rgb = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                rgb.header = rgb_message.header
                depth = (decode_depth_png(depth_message, byte_order=v['depth_byte_order'], bridge=bridge)
                         if v['depth_transport'] == 'compressedDepth' else depth_message)
                detector.prepare(stamp, inference_from_result(result, manifest))
                findings = pipeline.process(rgb, depth, info)
                if findings.status != 'PROCESSED':
                    node.get_logger().warning(f'RGB-D frame skipped: {findings.status}')
                    return
                canvas = result.plot(labels=True, conf=True)
                annotated = bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                annotated.header = rgb.header
                image_pub.publish(annotated)
                if v['show']:
                    cv2.imshow('AMR YOLO target', canvas)
                    cv2.waitKey(1)
                for candidate in findings.candidates:
                    if sender.state == 'SENT':
                        break
                    if sender.send(candidate, publish_target):
                        node.get_logger().info(
                            f'target state=SENT observation_id={sender.observation_id} '
                            f'undistortion={candidate.observation.localization.undistortion_id}; '
                            'coordinate publication disabled for this run')
                        break
                    node.get_logger().warning(f'target not sent: {sender.reason(candidate)}')
                state['frames'] += 1
                state['last_success'] = time.monotonic()
                if state['frames'] % v['log_every'] == 0:
                    node.get_logger().info(f'frames={state["frames"]} observations={state["observation_seq"]} '
                                          f'target_state={sender.state} last_stamp={stamp}')
            except Exception as exc:
                node.get_logger().error(f'AMR target frame failed (no fabricated coordinates): {type(exc).__name__}: {exc}')

        rgb_sub = message_filters.Subscriber(node, Image if v['image_type'] == 'raw' else CompressedImage,
                                             topics['image_topic'], qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(node, Image if v['depth_transport'] == 'raw' else CompressedImage,
                                               topics['depth_topic'], qos_profile=sensor_qos)
        synchronizer = (message_filters.TimeSynchronizer([rgb_sub, depth_sub], v['sync_queue_size'])
            if v['sync_slop_ns'] == 0 else message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], v['sync_queue_size'], v['sync_slop_ns'] / 1e9))
        synchronizer.registerCallback(on_pair)
        node.target_synchronizer = synchronizer
        def warn_stalled():
            now = time.monotonic()
            if now - state['last_success'] >= v['no_frame_warn_s'] and now - state['warning'] >= v['no_frame_warn_s']:
                state['warning'] = now
                node.get_logger().warning('No successful synchronized RGB-D frame; check both topics, QoS, CameraInfo and frame errors')
        node.create_timer(v['no_frame_warn_s'], warn_stalled)
        node.get_logger().info(f'amr_target git={git_sha()} boot={boot_id} model_sha256={model_hash} '
            f'parameters={json.dumps(v, sort_keys=True)} depth_scale={depth_policy.scale} '
            f'depth_offset_m={depth_policy.offset_m} calibration_id={binding.calibration_id}')
        return node
    except Exception:
        node.destroy_node()
        raise


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = create_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f'AMR target startup/runtime failed: {exc}', file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
