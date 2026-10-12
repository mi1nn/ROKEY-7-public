"""ROS boundary: compressed-first AMR RGB -> local YOLO -> annotated Image.

Following PR #113: explicit typed parameters, local model hash, startup/failure
logs, lazy ROS/model imports, ROS-free helpers. This is a diagnostic preview,
not a PersonPipeline/Observation producer. No robot control or sensor restart.
"""
import json
import os
import sys
import time

from .amr_camera_support import (PARAMS, PreviewProcessor, git_sha, validate_config,
                                 validate_labels, verify_model)


def create_node(*, model_factory=None):
    """Create the ROS node. model_factory is an injection point for synthetic tests."""
    import cv2
    from cv_bridge import CvBridge
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image

    node = Node('amr_camera_node')
    try:
        values = {}
        for key, kind in PARAMS.items():
            node.declare_parameter(key, getattr(Parameter.Type, kind),
                ParameterDescriptor(type=getattr(ParameterType, 'PARAMETER_' + kind), read_only=True,
                                    description='Required AMR YOLO preview startup parameter'))
            try:
                values[key] = node.get_parameter(key).value
            except ParameterUninitializedException:
                values[key] = None
        v = validate_config(values)
        source = node.resolve_topic_name(v['image_topic'])
        output = node.resolve_topic_name(v['annotated_topic'])
        if source == output:
            raise ValueError('input/output topics resolve to the same name (including remaps)')
        model_path, model_hash = verify_model(v['model_path'], v['model_sha256'])
        if v['show']:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise RuntimeError('show=true requires a desktop display; use show=false for headless ROS')
            cv2.namedWindow('AMR YOLO', cv2.WINDOW_NORMAL)
        if model_factory is None:
            from ultralytics import YOLO
            model_factory = YOLO
        model = model_factory(model_path)
        validate_labels(model.names, v['model_label'])
        processor = PreviewProcessor(model, CvBridge(), v)
        # Single-threaded executor + depth=1 bounds queued frames; throughput must be measured.
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        publisher = node.create_publisher(Image, output, output_qos)
        started = time.monotonic()
        state = {'last_input': started, 'last_success': started, 'frames': 0, 'errors': 0,
                 'warning': started}
        node.get_logger().info(
            f'AMR YOLO startup git={git_sha()} model_sha256={model_hash} '
            f'input={source} output={output} config={json.dumps(v, sort_keys=True)}')

        def on_image(message):
            state['last_input'] = time.monotonic()
            try:
                image, canvas, detections = processor.process(message)
                if v['show']:
                    cv2.imshow('AMR YOLO', canvas)
                    cv2.waitKey(1)
                publisher.publish(image)
            except Exception as exc:
                state['errors'] += 1
                node.get_logger().error(f'AMR YOLO frame failed (no output): {type(exc).__name__}: {exc}')
                return
            state['frames'] += 1
            state['last_success'] = time.monotonic()
            if state['frames'] % v['log_every'] == 0:
                elapsed = state['last_success'] - started
                node.get_logger().info(
                    f'frames={state["frames"]} errors={state["errors"]} detections={detections} '
                    f'processed_fps={state["frames"] / elapsed:.2f} '
                    f'frame={message.header.frame_id} '
                    f'stamp={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}')

        def warn_if_stalled():
            now = time.monotonic()
            if now - state['last_success'] >= v['no_frame_warn_s'] and now - state['warning'] >= v['no_frame_warn_s']:
                state['warning'] = now
                node.get_logger().warning(
                    f'No successful output for {now - state["last_success"]:.1f}s; '
                    f'last input {now - state["last_input"]:.1f}s ago; '
                    f'publishers={node.count_publishers(source)}. Check topic type/QoS/discovery and frame errors.')

        node.create_subscription(Image if v['image_type'] == 'raw' else CompressedImage,
                                 source, on_image, input_qos)
        node.create_timer(v['no_frame_warn_s'], warn_if_stalled)
        return node
    except Exception:
        node.destroy_node()
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            cv2.destroyAllWindows()
        raise


def main(args=None):
    try:
        import cv2
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError as exc:
        print(f'AMR YOLO dependency unavailable: {exc}. Source ROS and prepare the preview environment.', file=sys.stderr)
        return 2
    node = None
    try:
        rclpy.init(args=args)
        node = create_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f'AMR YOLO startup/runtime failed: {type(exc).__name__}: {exc}', file=sys.stderr)
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
