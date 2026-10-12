#!/usr/bin/env python3
"""Read-only raw 16UC1 depth viewer with optional local display calibration.

Ported from the user's tested standalone viewer on 2026-10-12. The empirical
scale is based on one 0.82 m check; it does not calibrate the ROS image topic.
"""
import argparse
import hashlib
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge

# ================================
# 설정 상수
# ================================
NORMALIZE_DEPTH_RANGE = 3.0            # 시각화 정규화 범위 (m)
WINDOW_NAME = 'Depth Image (Click to get distance)'
MEDIAN_KERNEL_SIZE = 5                 # 클릭 지점 주변 NxN 픽셀 중앙값 사용 (홀수)
# 2026-10-12: actual 0.82 m / mean raw of three user-provided clicks.
DEPTH_SCALE = 0.82 / ((0.854 + 0.853 + 0.857) / 3)
# Display-only estimate for this measurement; --depth-scale 1.0 disables scaling.
DEPTH_OFFSET_M = 0.0                   # 보정 오프셋 (m)
STALE_FRAME_SECONDS = 2.0  # Existing viewer receive-silence guard; not capture age.
WAIT_LOG_SECONDS = 10.0
# ================================

class DepthChecker(Node):
    def __init__(self, args):
        super().__init__('depth_checker')
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.should_exit = False
        self.depth_mm = None  # 최신 depth 이미지 저장
        self.depth_colored = None  # 시각화 이미지 저장
        self.last_depth_at = None
        self.wait_log_at = time.monotonic()

        self.subscription = self.create_subscription(
            Image,
            args.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data)

        # OpenCV 마우스 콜백 설정
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.get_logger().info(
            f"source_sha256={source_sha256}; depth={args.depth_topic}; "
            f"camera_info={args.camera_info_topic}; scale={args.depth_scale:.9f}; "
            f"offset_m={args.depth_offset_m:.6f}")

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
            half = MEDIAN_KERNEL_SIZE // 2
            height, width = self.depth_mm.shape
            patch = self.depth_mm[max(0, y - half):min(height, y + half + 1),
                                  max(0, x - half):min(width, x + half + 1)].astype(np.float32)
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size == 0:
                self.get_logger().warn(f"Clicked at (u={x}, v={y}) → No valid depth")
                return

            raw_m = float(np.median(valid)) / 1000.0  # mm → m
            corrected_m = raw_m * self.args.depth_scale + self.args.depth_offset_m
            self.get_logger().info(
                f"Clicked at (u={x}, v={y}) → Raw = {raw_m:.3f} m, Corrected = {corrected_m:.3f} m "
                f"(valid {valid.size}/{patch.size} px)")

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--depth-topic', required=True)
    parser.add_argument('--camera-info-topic', required=True)
    parser.add_argument('--depth-scale', type=positive_finite, default=DEPTH_SCALE)
    parser.add_argument('--depth-offset-m', type=finite, default=DEPTH_OFFSET_M)
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = DepthChecker(args)

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
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
