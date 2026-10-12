#!/usr/bin/env python3
"""Local ROS transport smoke test, with synthetic pixels and an injected fake model.

Checks raw/compressed decode, annotated transport, source header preservation,
startup hash and a failed inference frame producing no output. NOT a YOLO test.
No robot subscription; LOCALHOST discovery is mandatory.
Run from a checkout with its two Python packages in PYTHONPATH.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-type', choices=('raw', 'compressed'), default='compressed',
                        help='compressed by default; raw is for explicit comparison')
    parser.add_argument('--real-model',action='store_true',help='use the actual manifest-selected best.pt instead of the fake model')
    parser.add_argument('--image',type=Path,help='required local source image in --real-model mode')
    args = parser.parse_args()
    if args.real_model and args.image is None:
        parser.error('--real-model requires --image')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST for this synthetic test')
    # Discovery Server can bypass the intended local-only test graph. Do not use it here.
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        parser.error('unset ROS_DISCOVERY_SERVER for this synthetic test')
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
    from rescue_perception.amr_camera_node import create_node

    class FakeResult:
        boxes = ()
        def __init__(self, frame):
            self.frame = frame
        def plot(self, **kwargs):
            return self.frame.copy()

    class FakeModel:
        names = {0: 'survivor'}
        calls = 0
        def __init__(self, path):
            pass
        def predict(self, frame, **kwargs):
            self.calls += 1
            if self.calls == 1:
                raise RuntimeError('intentional synthetic inference failure')
            return [FakeResult(frame)]

    preview = source = executor = None
    with TemporaryDirectory() as directory:
        weights = Path(directory) / 'synthetic.pt'
        weights.write_bytes(b'fake-model-fixture; no trained weights')
        config = dict(image_topic='/preview_smoke/input', image_type=args.image_type,
                      annotated_topic='/preview_smoke/output', model_path=str(weights),
                      model_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
                      model_label='survivor', model_imgsz=64, model_conf=0.25, device='cpu',
                      show=False, no_frame_warn_s=2.0, log_every=1)
        root=Path(__file__).resolve().parents[1]
        if args.real_model:
            manifest=json.loads((root/'assets/manifest/yolo26n-survivor-v2.json').read_text())['uploaded_model']
            config.update(model_path=str(root/manifest['path']),model_sha256=manifest['sha256'],
                          model_imgsz=manifest['imgsz'],model_label=manifest['class_names'][0])
            config_dir=root/'.artifacts/ultralytics-config'
            config_dir.mkdir(parents=True,exist_ok=True)
            os.environ['YOLO_CONFIG_DIR']=str(config_dir)
        ros_args = ['--ros-args']
        for key, value in config.items():
            ros_args.extend(['-p', f'{key}:={str(value).lower() if type(value) is bool else value}'])
        rclpy.init(args=ros_args)
        try:
            preview = create_node(model_factory=None if args.real_model else FakeModel)
            source = Node('amr_camera_smoke')
            bridge = CvBridge()
            frame = np.zeros((48, 64, 3), dtype=np.uint8)
            frame[:, :, 1] = 127
            if args.real_model:
                import cv2
                frame=cv2.imread(str(args.image),cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('cannot decode the real-model smoke source image')
            publisher = source.create_publisher(
                Image if args.image_type == 'raw' else CompressedImage, config['image_topic'],
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            outputs = []
            expected = {}
            def receive(message):
                stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
                if stamp not in expected or message.header.frame_id != 'synthetic_rgb_optical':
                    raise AssertionError('source header was lost or replaced')
                if not args.real_model and stamp == (1700000000, 1):
                    raise AssertionError('failed inference produced output')
                decoded = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                if decoded.shape != frame.shape or message.encoding != 'bgr8':
                    raise AssertionError('annotated output format mismatch')
                outputs.append(stamp)
            source.create_subscription(Image, config['annotated_topic'], receive, 1)
            executor = SingleThreadedExecutor()
            executor.add_node(preview)
            executor.add_node(source)
            deadline = time.monotonic() + 10.0
            sequence = 0
            next_send = time.monotonic() + 0.5
            while time.monotonic() < deadline and len(outputs) < 3:
                now = time.monotonic()
                if now >= next_send and publisher.get_subscription_count():
                    sequence += 1
                    message = (bridge.cv2_to_imgmsg(frame, encoding='bgr8') if args.image_type == 'raw'
                               else bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg'))
                    message.header.stamp.sec = 1700000000
                    message.header.stamp.nanosec = sequence
                    message.header.frame_id = 'synthetic_rgb_optical'
                    expected[(1700000000, sequence)] = True
                    publisher.publish(message)
                    next_send = now + 0.2
                executor.spin_once(timeout_sec=0.05)
            if len(outputs) < 3:
                raise RuntimeError(f'timeout: received {len(outputs)} annotated frames')
            if args.real_model:
                print(f'PASSED real best.pt ROS {args.image_type}: {len(outputs)} annotated outputs; source headers preserved. '
                      'Input is a local recorded image, not a live camera; camera/GUI/Depth NOT_RUN.')
            else:
                print(f'PASSED synthetic ROS {args.image_type}: {len(outputs)} outputs, source headers preserved; '
                      'intentional failed frame absent. Real YOLO/camera/GUI NOT_RUN.')
        finally:
            if executor is not None:
                executor.shutdown()
            for node in (source, preview):
                if node is not None:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
