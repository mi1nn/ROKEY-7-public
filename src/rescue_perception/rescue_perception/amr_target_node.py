"""AMR RGB-D detection node, following existing ROS boundary patterns.

Compressed RGB + aligned depth + CameraInfo -> PersonPipeline -> TargetObservation.
Optional local projection adapter composes mapping-owned metadata/TF and emits
TargetLocation. It does not invent sessions, revisions, alignments or clock facts.
This composition is a field-test bridge; deployment can move projection to the
mapping owner's node without changing message contracts. No robot commands.
"""
import json
import os
import sys
import time
from dataclasses import replace
from pathlib import Path
from uuid import uuid4

from .amr_camera_support import PARAMS as PREVIEW_PARAMS, PreviewProcessor, git_sha, validate_labels, verify_model
from .amr_target_support import (MapStateTracker, PreparedDetector, ProjectionMetadata, check_projection_inputs, covariance_matrix,
    fill_envelope, fill_location, fill_observation, inference_from_result, stamped_transform, load_depth_calibration, decode_depth_png)
from .adapter import ObservationPolicy, SourceBinding, stamp_ns
from .core import AlignmentEvidence, DepthPolicy, ModelManifest, TimePolicy
from .person_pipeline import ConfirmationPolicy, PersonPipeline

PARAMS = dict(PREVIEW_PARAMS, **{
    'physical_robot_id': 'STRING', 'depth_calibration_path': 'STRING',
    'depth_topic': 'STRING', 'depth_transport': 'STRING', 'depth_byte_order': 'STRING',
    'camera_info_topic': 'STRING', 'observations_topic': 'STRING', 'locations_topic': 'STRING',
    'map_state_topic': 'STRING', 'robot_id': 'STRING', 'camera_id': 'STRING', 'model_id': 'STRING',
    'rgb_optical_frame': 'STRING', 'depth_optical_frame': 'STRING', 'calibration_id': 'STRING',
    'rgb_stamp_semantics': 'STRING', 'depth_stamp_semantics': 'STRING', 'clock_domain_id': 'STRING',
    'clock_state': 'STRING', 'alignment_evidence_id': 'STRING', 'projection_metadata_path': 'STRING',
    'publisher_id': 'STRING', 'envelope_version': 'STRING', 'alignment_authority': 'STRING',
    'rgb_rectified': 'BOOL', 'rgb_depth_alignment_verified': 'BOOL', 'locations_enabled': 'BOOL',
    'clock_error_bound_ns': 'INTEGER', 'receiver_clock_error_ns': 'INTEGER', 'sync_queue_size': 'INTEGER',
    'sync_slop_ns': 'INTEGER', 'camera_info_max_age_ns': 'INTEGER', 'observation_validity_ns': 'INTEGER',
    'max_map_age_ns': 'INTEGER', 'max_age_ns': 'INTEGER', 'max_future_ns': 'INTEGER',
    'max_clock_error_ns': 'INTEGER', 'stamp_delay_bound_ns': 'INTEGER', 'min_valid_pixels': 'INTEGER',
    'confirm_confidence_min': 'DOUBLE', 'min_depth_m': 'DOUBLE', 'max_depth_m': 'DOUBLE',
    'min_valid_fraction': 'DOUBLE', 'max_depth_spread_m': 'DOUBLE',
    'max_alignment_position_error_m': 'DOUBLE', 'max_alignment_rotation_error_rad': 'DOUBLE',
    'sensor_variance_xyz': 'DOUBLE_ARRAY',
})


def target_config(v, boot_id):
    from .amr_camera_support import validate_config
    validate_config(v)
    for key, kind in PARAMS.items():
        expected = {'STRING': str, 'INTEGER': int, 'DOUBLE': float, 'BOOL': bool, 'DOUBLE_ARRAY': (list, tuple)}[kind]
        if key not in v or not isinstance(v[key], expected) or (kind == 'INTEGER' and type(v[key]) is not int):
            raise ValueError(f'missing/invalid parameter: {key} ({kind})')
        if kind == 'STRING' and not v[key].strip():
            raise ValueError(f'empty parameter: {key}')
    if v['depth_transport'] not in ('raw', 'compressedDepth') or v['depth_byte_order'] not in ('little', 'big'):
        raise ValueError('depth_transport must be raw/compressedDepth; specify publisher byte order')
    for key in ('sync_queue_size', 'camera_info_max_age_ns', 'observation_validity_ns', 'max_map_age_ns'):
        if v[key] <= 0:
            raise ValueError(f'{key} must be positive')
    topics = [v[key] for key in ('image_topic', 'depth_topic', 'camera_info_topic', 'annotated_topic',
                                 'observations_topic', 'locations_topic', 'map_state_topic')]
    if len(topics) != len(set(topics)):
        raise ValueError('all input/output topics must differ')
    bound = v['clock_error_bound_ns']
    if bound < -1:
        raise ValueError('clock_error_bound_ns must be >=0 or -1 (UNKNOWN)')
    binding = SourceBinding(v['robot_id'], v['camera_id'], boot_id, v['clock_domain_id'],
        v['rgb_stamp_semantics'], v['depth_stamp_semantics'], v['calibration_id'], v['clock_state'],
        bound if bound >= 0 else None)
    alignment = AlignmentEvidence(v['alignment_evidence_id'], v['calibration_id'],
        v['rgb_optical_frame'], v['depth_optical_frame'], v['rgb_depth_alignment_verified'])
    depth = DepthPolicy(v['sync_slop_ns'], v['min_depth_m'], v['max_depth_m'],
                        v['min_valid_pixels'], v['min_valid_fraction'], v['max_depth_spread_m'])
    if v['depth_calibration_path'] != 'UNCONFIGURED':
        calibration = load_depth_calibration(v['depth_calibration_path'], physical_robot_id=v['physical_robot_id'])
        depth = replace(depth, scale=calibration.scale, offset_m=calibration.offset_m)
        calibration_id = f'{binding.calibration_id}|depth:{calibration.calibration_id}:{calibration.sha256}'
        binding = replace(binding, calibration_id=calibration_id)
        alignment = replace(alignment, calibration_id=calibration_id)
    expiry = ObservationPolicy(v['observation_validity_ns'])
    confirmation = ConfirmationPolicy((v['model_label'],), v['confirm_confidence_min'])
    if v['confirm_confidence_min'] < v['model_conf']:
        raise ValueError('confirm_confidence_min must not be below model_conf')
    freshness = TimePolicy(v['max_age_ns'], v['max_future_ns'], v['max_clock_error_ns'], v['stamp_delay_bound_ns'])
    if v['envelope_version'] != '0.1':
        raise ValueError('only the current ROS wire envelope version 0.1 is supported')
    covariance = covariance_matrix(v['sensor_variance_xyz'])
    if v['receiver_clock_error_ns'] < 0:
        raise ValueError('receiver_clock_error_ns must be nonnegative')
    return binding, alignment, depth, expiry, confirmation, freshness, covariance


def create_node(*, model_factory=None, transform_lookup=None):
    import cv2
    import message_filters
    from cv_bridge import CvBridge
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from rescue_interfaces.msg import MapState, TargetLocation, TargetObservation

    node = Node('amr_target_node')
    try:
        v = {}
        for key, kind in PARAMS.items():
            node.declare_parameter(key, getattr(Parameter.Type, kind),
                ParameterDescriptor(type=getattr(ParameterType, 'PARAMETER_' + kind), read_only=True))
            try:
                value = node.get_parameter(key).value
                v[key] = list(value) if kind == 'DOUBLE_ARRAY' else value
            except ParameterUninitializedException:
                v[key] = None
        missing = [k for k, value in v.items() if value is None]
        if missing:
            raise ValueError(f'missing required parameters: {missing}')
        boot_id = str(uuid4())
        binding, alignment, depth_policy, expiry, confirmation, freshness, covariance = target_config(v, boot_id)
        topic_keys = ('image_topic', 'depth_topic', 'camera_info_topic', 'annotated_topic',
                      'observations_topic', 'locations_topic', 'map_state_topic')
        topics = {key: node.resolve_topic_name(v[key]) for key in topic_keys}
        if len(set(topics.values())) != len(topics):
            raise ValueError('remapped input/output topics must differ')
        metadata = None
        if v['locations_enabled']:
            metadata = ProjectionMetadata.from_dict(json.loads(Path(v['projection_metadata_path']).read_text()))
            a = metadata.alignment
            if (a.source_robot != binding.source_robot or a.clock_domain_id != binding.clock_domain_id
                    or a.authority_id != v['alignment_authority']):
                raise ValueError('mapping metadata robot/clock domain mismatch')
            if covariance_matrix(metadata.sensor_variance_xyz) != covariance:
                raise ValueError('configured sensor variances differ from mapping covariance evidence')
            import math
            for value, limit in ((a.position_error_m, v['max_alignment_position_error_m']),
                                 (a.rotation_error_rad, v['max_alignment_rotation_error_rad'])):
                if not math.isfinite(limit) or limit < 0 or value > limit:
                    raise ValueError('mapping alignment error exceeds configured limit')
        path, model_hash = verify_model(v['model_path'], v['model_sha256'])
        if model_factory is None:
            from ultralytics import YOLO
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
        if metadata is not None and transform_lookup is None:
            from tf2_ros import Buffer, TransformListener
            buffer = Buffer(node=node)
            listener = TransformListener(buffer, node, spin_thread=False)
            node.target_tf_listener = listener
            transform_lookup = lambda target, source, stamp: buffer.lookup_transform(
                target, source, Time(nanoseconds=stamp))  # never Time() / latest fallback
        sensor_qos = QoSProfile(depth=v['sync_queue_size'], reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        observation_pub = node.create_publisher(TargetObservation, topics['observations_topic'],
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        location_pub = node.create_publisher(TargetLocation, topics['locations_topic'],
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        image_pub = node.create_publisher(Image, topics['annotated_topic'], 1)
        state = {'info': None, 'map': None, 'map_received': 0., 'last_success': time.monotonic(),
                 'warning': 0., 'frames': 0, 'observation_seq': 0, 'location_seq': 0}

        def envelope(message, key, now):
            state[key] += 1
            fill_envelope(message, version=v['envelope_version'], publisher_id=v['publisher_id'],
                          boot_id=boot_id, sequence=state[key], now_ns=now)

        map_tracker = MapStateTracker(binding.source_robot)
        def receive_map(message):
            if message.robot_id != binding.source_robot:
                return
            try:
                state['map'] = map_tracker.update(message)
                state['map_received'] = time.monotonic()
            except Exception as exc:
                node.get_logger().warning(f'MapState discarded: {exc}')

        node.create_subscription(CameraInfo, topics['camera_info_topic'],
                                 lambda message: state.update(info=message), sensor_qos)
        node.create_subscription(MapState, topics['map_state_topic'], receive_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

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
                    o = candidate.observation
                    observation = fill_observation(TargetObservation(), o, covariance)
                    now = node.get_clock().now().nanoseconds
                    envelope(observation.envelope, 'observation_seq', now)
                    observation_pub.publish(observation)
                    if candidate.status != 'CONFIRMED':
                        node.get_logger().warning(f'location blocked: {candidate.reasons}')
                        continue
                    if metadata is None:
                        node.get_logger().warning('location blocked: mapping projection disabled/not configured')
                        continue
                    try:
                        if time.monotonic() - state['map_received'] > v['max_map_age_ns'] / 1e9:
                            raise ValueError('MapState receive timeout')
                        check_projection_inputs(o, metadata, state['map'], now_ns=now,
                            max_map_age_ns=v['max_map_age_ns'], receiver_clock_error_ns=v['receiver_clock_error_ns'],
                            time_policy=freshness)
                        transform = stamped_transform(transform_lookup(metadata.alignment.map_frame, o.frame.frame_id,
                            o.observed_at_ns), source_frame=o.frame.frame_id, target_frame=metadata.alignment.map_frame,
                            observed_at_ns=o.observed_at_ns)
                        # Recheck age/version at emission. A slow inference/lookup cannot extend source lifetime.
                        now = node.get_clock().now().nanoseconds
                        check_projection_inputs(o, metadata, state['map'], now_ns=now,
                            max_map_age_ns=v['max_map_age_ns'], receiver_clock_error_ns=v['receiver_clock_error_ns'],
                            time_policy=freshness)
                        location = fill_location(TargetLocation(), observation, o, metadata, transform, covariance)
                        envelope(location.envelope, 'location_seq', now)
                        location_pub.publish(location)
                        node.get_logger().info(f'TargetLocation id={location.location_id} '
                            f'frame={location.frame_ref.mission_frame} xyz={location.mission_position}')
                    except Exception as exc:
                        node.get_logger().warning(f'location blocked: {type(exc).__name__}: {exc}')
                state['frames'] += 1
                state['last_success'] = time.monotonic()
                if state['frames'] % v['log_every'] == 0:
                    node.get_logger().info(f'frames={state["frames"]} observations={state["observation_seq"]} '
                                          f'locations={state["location_seq"]} last_stamp={stamp}')
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
        node.get_logger().info(f'amr_target_node git={git_sha()} boot={boot_id} model_sha256={model_hash} '
            f'parameters={json.dumps(v, sort_keys=True)} depth_scale={depth_policy.scale} '
            f'depth_offset_m={depth_policy.offset_m} calibration_id={binding.calibration_id} mapping_snapshot={metadata}')
        return node
    except Exception:
        node.destroy_node()
        raise


def main(args=None):
    node = None
    try:
        import cv2
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError as exc:
        print(f'AMR target dependency missing: {exc}', file=sys.stderr)
        return 2
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
