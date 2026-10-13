#!/usr/bin/env python3
"""Read-only raw 16UC1 depth viewer with the perception depth calibration applied for display.

Ported from the user's tested standalone viewer on 2026-10-12. The scale and offset
come from the same calibration file the perception core reads
(``rescue_perception/config/robot6_depth.json`` by default, via
``target_utils.load_depth_calibration``) so the click display and the core cannot
disagree. ``--depth-scale``/``--depth-offset-m`` override it for an experiment only.
It does not calibrate or republish the ROS image topic.

ROS 2, cv_bridge, NumPy and OpenCV are imported in ``run`` so the click arithmetic
can be tested without them (tests/test_depth_click_check.py).
"""
import argparse
import hashlib
import math
from pathlib import Path
import statistics
import time

# ================================
# 설정 상수
# ================================
NORMALIZE_DEPTH_RANGE = 3.0            # 시각화 정규화 범위 (m)
WINDOW_NAME = 'Depth Image (Click to get distance)'
MEDIAN_KERNEL_SIZE = 5                 # 클릭 지점 주변 NxN 픽셀 중앙값 사용 (홀수)
DEFAULT_CALIBRATION = 'package:robot6_depth.json'  # perception 코어와 같은 보정 파일
DEFAULT_PHYSICAL_ROBOT_ID = 'robot6'   # 위 보정 파일의 physical_robot_id
STALE_FRAME_SECONDS = 2.0  # Existing viewer receive-silence guard; not capture age.
WAIT_LOG_SECONDS = 10.0
# ================================


def patch_median_mm(depth_mm, x, y, kernel=MEDIAN_KERNEL_SIZE):
    """(median of valid mm, valid count, patch count) in the kernel x kernel patch around (x, y).

    The patch is clipped at the image edge. 0 and non-finite values are not depth.
    """
    half = kernel // 2
    height, width = len(depth_mm), len(depth_mm[0])
    values = [float(depth_mm[row][col])
              for row in range(max(0, y - half), min(height, y + half + 1))
              for col in range(max(0, x - half), min(width, x + half + 1))]
    valid = [value for value in values if math.isfinite(value) and value > 0]
    return (statistics.median(valid) if valid else None), len(valid), len(values)


def corrected_m(raw_m, scale, offset_m):
    return raw_m * scale + offset_m


def positive_finite(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return number


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('must be finite')
    return number


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--depth-topic', required=True)
    parser.add_argument('--camera-info-topic', required=True)
    parser.add_argument('--calibration', default=DEFAULT_CALIBRATION,
                        help='depth calibration JSON: package:<file name> or a path (default: %(default)s)')
    parser.add_argument('--physical-robot-id', default=DEFAULT_PHYSICAL_ROBOT_ID,
                        help='must match physical_robot_id in the calibration file (default: %(default)s)')
    parser.add_argument('--depth-scale', type=positive_finite, default=None,
                        help='experiment only: override the calibration file scale')
    parser.add_argument('--depth-offset-m', type=finite, default=None,
                        help='experiment only: override the calibration file offset (m)')
    return parser.parse_args(argv)


def resolve_calibration(args, load):
    """(scale, offset_m, log label). ``load`` is target_utils.load_depth_calibration."""
    calibration = load(args.calibration, physical_robot_id=args.physical_robot_id)
    label = f'calibration={calibration.calibration_id} file={args.calibration} sha256={calibration.sha256}'
    scale, offset_m = calibration.scale, calibration.offset_m
    if args.depth_scale is not None:
        scale, label = args.depth_scale, f'{label} scale override (file {calibration.scale:.9f})'
    if args.depth_offset_m is not None:
        offset_m, label = args.depth_offset_m, f'{label} offset override (file {calibration.offset_m:.6f})'
    return scale, offset_m, label


def run(args, ros_args):
    import cv2
    from cv_bridge import CvBridge
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rescue_perception.target_utils import load_depth_calibration
    from sensor_msgs.msg import CameraInfo, Image

    scale, offset_m, label = resolve_calibration(args, load_depth_calibration)

    class DepthChecker(Node):
        def __init__(self):
            super().__init__('depth_checker')
            self.bridge = CvBridge()
            self.K = None
            self.should_exit = False
            self.depth_mm = None  # 최신 depth 이미지 저장
            self.depth_colored = None  # 시각화 이미지 저장
            self.last_depth_at = None
            self.wait_log_at = time.monotonic()

            self.subscription = self.create_subscription(
                Image, args.depth_topic, self.depth_callback, qos_profile_sensor_data)
            self.camera_info_subscription = self.create_subscription(
                CameraInfo, args.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)

            # OpenCV 마우스 콜백 설정
            cv2.namedWindow(WINDOW_NAME)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
            source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            self.get_logger().info(
                f"source_sha256={source_sha256}; depth={args.depth_topic}; "
                f"camera_info={args.camera_info_topic}; scale={scale:.9f}; offset_m={offset_m:.6f}; {label}")

        def camera_info_callback(self, msg):
            if self.K is None:
                self.K = np.array(msg.k).reshape(3, 3)
                self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

        def depth_callback(self, msg):
            if self.should_exit:
                return

            if msg.encoding != '16UC1':
                self.get_logger().error(f'Expected 16UC1/mm, received {msg.encoding}', throttle_duration_sec=2.0)
                return
            self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_at = time.monotonic()
            height, width = self.depth_mm.shape

            # 시각화 이미지 생성
            depth_vis = np.nan_to_num(self.depth_mm, nan=0.0)
            depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
            depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
            self.depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            # 중심점 표시
            cx = int(self.K[0, 2]) if self.K is not None else width // 2
            cy = int(self.K[1, 2]) if self.K is not None else height // 2
            cv2.circle(self.depth_colored, (cx, cy), 5, (0, 0, 0), -1)
            cv2.line(self.depth_colored, (0, cy), (width, cy), (0, 0, 0), 1)
            cv2.line(self.depth_colored, (cx, 0), (cx, height), (0, 0, 0), 1)

        def mouse_callback(self, event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and self.depth_mm is not None:
                if time.monotonic() - self.last_depth_at > STALE_FRAME_SECONDS:
                    self.get_logger().warn("Old frame: wait for fresh Depth before measuring")
                    return
                # 클릭 지점 주변 NxN 패치에서 유효값(0, NaN 제외)의 중앙값 사용
                median_mm, valid, total = patch_median_mm(self.depth_mm, x, y)
                if median_mm is None:
                    self.get_logger().warn(f"Clicked at (u={x}, v={y}) → No valid depth")
                    return
                raw_m = median_mm / 1000.0  # mm → m
                self.get_logger().info(
                    f"Clicked at (u={x}, v={y}) → Raw = {raw_m:.3f} m, "
                    f"Corrected = {corrected_m(raw_m, scale, offset_m):.3f} m (valid {valid}/{total} px)")

    rclpy.init(args=ros_args)
    node = DepthChecker()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if node.depth_colored is None:
                display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display, 'Waiting for Depth...', (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if now - node.wait_log_at > WAIT_LOG_SECONDS:
                    node.get_logger().warn('No Depth received: check robot connection and topic publisher')
                    node.wait_log_at = now
            else:
                display = node.depth_colored.copy()
                if now - node.last_depth_at > STALE_FRAME_SECONDS:
                    cv2.putText(display, 'STALE FRAME', (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(WINDOW_NAME, display)
            # Pump window events even when the robot is not sending images.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                node.should_exit = True
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                node.should_exit = True
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


def main():
    import sys
    from rclpy.utilities import remove_ros_args
    run(parse_args(remove_ros_args(sys.argv)[1:]), sys.argv)


if __name__ == '__main__':
    main()
