"""E3 (partial): the real amr_target on synthetic TB-6 inputs -> TargetObservation and ObserveTarget (2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy, a built rescue_interfaces, OpenCV, cv_bridge).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    source <scratch>/install/setup.bash   # colcon build --packages-select rescue_interfaces
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_mapping:<REPO>/src/rescue_perception:$PYTHONPATH \
      python3 tests/ros/e3_amr_target_probe.py <REPO>

amr_target runs as a child with tests/ros/stub_ultralytics first on its path (a red square is the
only "survivor"; no model). Pass --compressed-depth to also test PNG transport. The probe publishes compressed RGB + raw 16UC1 Depth + CameraInfo with
the same stamp (unrectified, plumb_bob zero distortion -> the box-point undistortion path), static
TF map -> base_link -> camera optical, map -> laser, and LaserScan. All values are synthetic test values. Checks:
1. without recheck values the node refuses to start (exit 2); 2. the doll is sent once as a
TargetObservation with a located camera point and the host's measured clock (SYNCED when timesyncd
answers); 3. ObserveTarget with the doll at the point -> PRESENT, LiDAR BLOCKED, coverage VALID;
4. doll gone -> ABSENT, CLEAR, VALID; 5. a QR with other content -> no exit observation (OPS-136);
6. the exit QR EXIT-1 -> one exit TargetObservation (class exit_qr, track_id EXIT-1, point at the QR
depth), not repeated while it stays in view; 7. a new TB-6 map revision -> one more; 8. the second
listed exit EXIT-2 -> one observation with track_id EXIT-2 (OPS-154/157). Exit code 0 = all held.
"""
import math
import os
import struct
import subprocess
import sys
import time
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_interfaces.action import ObserveTarget
from rescue_interfaces.msg import MapState, TargetObservation
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage

W, H, F = 160, 120, 100.0              # synthetic camera: 160x120, f = 100 px, centre (80, 60)
OPTICAL, LASER = 'e3_camera_optical', 'e3_laser'
DOLL = (70, 50, 90, 70)                # red square (x0, y0, x1, y1) around the image centre
FAR_MM, DOLL_MM = 4000, 2000
ACTION = '/rescue/robots/robot_a/observe_target'
SCALE = 0.9594383775351013             # robot_depth.json, applied by amr_target
POINT = (DOLL_MM / 1000 * SCALE, 0.0, 0.3)   # doll centre in the map: straight ahead of the camera
EXIT_PAYLOADS, QR_MM = ('EXIT-1', 'EXIT-2'), 1500
MAP_STATE = '/rescue/robots/robot_a/map_state'
COMPRESSED_DEPTH = '--compressed-depth' in sys.argv


def qr_image(text):
    """(code image (3 px per module, white quiet zone), placed at its top-left) for the 160x120 frame."""
    code = cv2.resize(cv2.QRCodeEncoder.create().encode(text), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    return code, ((W - code.shape[1]) // 2, (H - code.shape[0]) // 2)


def amr_args(with_recheck=True):
    p = {'image_topic': '/e3robot/rgb/compressed', 'image_type': 'compressed', 'depth_topic': '/e3robot/depth',
         'depth_transport': 'compressedDepth' if COMPRESSED_DEPTH else 'raw', 'base_point_topic': '/e3robot/perception/target_base_point', 'camera_info_topic': '/e3robot/camera_info', 'rgb_optical_frame': OPTICAL,
         'depth_optical_frame': OPTICAL, 'rgb_stamp_semantics': 'CAPTURE', 'depth_stamp_semantics': 'CAPTURE',
         'rgb_depth_alignment_verified': 'true', 'rgb_rectified': 'false', 'calibration_id': 'e3-cal',
         'alignment_evidence_id': 'e3-align', 'sync_queue_size': '10', 'sync_slop_ns': '20000000',
         'physical_robot_id': 'robot5', 'depth_calibration_path': 'package:robot_depth.json',
         'camera_info_max_age_ns': '1500000000', 'clock_domain_id': 'e3-clock', 'clock_backend': 'timesyncd',
         'clock_max_age_ns': '3600000000000', 'clock_drift_ppm': '100.0', 'clock_poll_period_s': '0.5',
         'sensor_host_clock_error_bound_ns': '0', 'sensor_host_clock_domain_id': 'e3-clock'}
    if with_recheck:
        p.update({'recheck_action_name': ACTION, 'recheck_map_frame': 'map', 'recheck_scan_topic': '/e3robot/scan',
                  'recheck_max_window_ns': '10000000000', 'recheck_validity_ns': '5000000000',
                  'recheck_presence_radius_m': '0.5', 'recheck_lidar_radius_m': '0.3', 'recheck_min_frames': '3',
                  'recheck_min_scans': '3', 'recheck_min_lidar_rays': '3', 'recheck_min_lidar_hits': '2',
                  'recheck_image_margin_px': '5', 'recheck_min_depth_m': '0.2', 'recheck_max_depth_m': '5.0',
                  'recheck_occlusion_tolerance_m': '0.3', 'recheck_depth_window_px': '2'})
        p.update({'exit_qr_payloads': list(EXIT_PAYLOADS), 'exit_qr_required_frames': '5',
                  'exit_qr_position_tolerance_m': '0.2', 'exit_qr_min_valid_depth_ratio': '0.5', 'exit_qr_every_n_frames': '2',
                  'exit_map_state_topic': MAP_STATE})
    flags = []
    for name, value in p.items():
        flags += ['-p', f'{name}:={value}']
    return flags + ['-r', '/tf:=/e3robot/tf', '-r', '/tf_static:=/e3robot/tf_static']


def amr_command(repo, extra):
    code = 'from rescue_perception.amr_target import main; raise SystemExit(main())'
    return [sys.executable, '-c', code, '--ros-args'] + extra


def child_env(repo):
    env = dict(os.environ)
    env['PYTHONPATH'] = os.path.join(repo, 'tests', 'ros', 'stub_ultralytics') + os.pathsep + env.get('PYTHONPATH', '')
    return env


def static_tf(parent, child, xyz, q):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t


def stamp_of(node):
    now = node.get_clock().now().nanoseconds
    return now, divmod(now, 10**9)


def main():
    repo = sys.argv[1]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    checks = {}
    refused = subprocess.run(amr_command(repo, amr_args(with_recheck=False)), cwd=repo, env=child_env(repo),
                             capture_output=True, text=True, timeout=60)
    checks['no recheck values -> exit 2'] = refused.returncode == 2 and 'recheck_' in refused.stderr
    child = subprocess.Popen(amr_command(repo, amr_args()), cwd=repo, env=child_env(repo),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('e3_amr_target_probe')
    sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    rgb_pub = node.create_publisher(CompressedImage, '/e3robot/rgb/compressed', sensor)
    depth_pub = node.create_publisher(CompressedImage if COMPRESSED_DEPTH else Image, '/e3robot/depth', sensor)
    info_pub = node.create_publisher(CameraInfo, '/e3robot/camera_info', sensor)
    scan_pub = node.create_publisher(LaserScan, '/e3robot/scan', sensor)
    static_pub = node.create_publisher(TFMessage, '/e3robot/tf_static', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    base_points = []
    node.create_subscription(PointStamped, '/e3robot/perception/target_base_point', base_points.append, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    observations = []
    node.create_subscription(TargetObservation, '/rescue/observations', observations.append, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    exits = []
    node.create_subscription(TargetObservation, '/rescue/exit_observations', exits.append, QoSProfile(
        depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    map_pub = node.create_publisher(MapState, MAP_STATE, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = ActionClient(node, ObserveTarget, ACTION)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    scene = {'doll': True, 'qr': None}

    def publish_frame():
        now, (sec, nsec) = stamp_of(node)
        image = np.full((H, W, 3), 90, np.uint8)
        depth = np.full((H, W), FAR_MM, np.uint16)
        if scene['doll']:
            x0, y0, x1, y1 = DOLL
            image[y0:y1, x0:x1] = (0, 0, 255)
            depth[y0:y1, x0:x1] = DOLL_MM
        if scene['qr']:
            code, (x, y) = qr_image(scene['qr'])
            image[y:y + code.shape[0], x:x + code.shape[1]] = cv2.cvtColor(code, cv2.COLOR_GRAY2BGR)
            depth[y:y + code.shape[0], x:x + code.shape[1]] = QR_MM
        rgb = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes())
        d = (CompressedImage(format='16UC1; compressedDepth png',
             data=struct.pack('<iff', 0, 0.0, 0.0)+cv2.imencode('.png',depth)[1].tobytes())
             if COMPRESSED_DEPTH else
             Image(height=H, width=W, encoding='16UC1', is_bigendian=0, step=W * 2, data=depth.tobytes()))
        info = CameraInfo(width=W, height=H, distortion_model='plumb_bob', d=[0.0] * 5,
                          k=[F, 0.0, W / 2, 0.0, F, H / 2, 0.0, 0.0, 1.0],
                          p=[F, 0.0, W / 2, 0.0, 0.0, F, H / 2, 0.0, 0.0, 0.0, 1.0, 0.0])
        for msg in (rgb, d, info):
            msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec = OPTICAL, sec, nsec
        n = 101
        ranges = [6.0] * n
        if scene['doll']:
            for i in range(n):   # rays toward the doll return at its distance
                if abs(-0.5 + i * 0.01) <= 0.1:
                    ranges[i] = POINT[0]
        scan = LaserScan(angle_min=-0.5, angle_max=0.5, angle_increment=0.01, range_min=0.1, range_max=8.0,
                         ranges=ranges)
        scan.header.frame_id, scan.header.stamp.sec, scan.header.stamp.nanosec = LASER, sec, nsec
        info_pub.publish(info)
        rgb_pub.publish(rgb)
        depth_pub.publish(d)
        scan_pub.publish(scan)

    def pump(seconds):
        end = time.monotonic() + seconds
        next_frame = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_frame:
                publish_frame()
                next_frame = time.monotonic() + 0.1
            executor.spin_once(timeout_sec=0.02)

    def observe(window_s=1.5):
        goal = ObserveTarget.Goal(request_id=str(uuid4()), task_id=str(uuid4()), target_id=str(uuid4()),
                                  robot_id='robot_a', requested_window_ns=int(window_s * 1e9), policy_version='e3',
                                  deadline_ns=node.get_clock().now().nanoseconds + 30_000_000_000)
        goal.watched_native_point.x, goal.watched_native_point.y, goal.watched_native_point.z = POINT
        goal.frame_ref.robot_id, goal.frame_ref.native_frame, goal.frame_ref.mission_frame = 'robot_a', 'map', 'map'
        future = client.send_goal_async(goal)
        while not future.done():
            pump(0.05)
        result = future.result().get_result_async()
        while not result.done():
            pump(0.05)
        return result.result().result

    try:
        end = time.monotonic() + 30
        while not client.server_is_ready() and time.monotonic() < end:
            pump(0.2)
        pump(0.5)
        checks['missing base TF -> no success publication; retry stays enabled'] = not observations and not base_points
        static_pub.publish(TFMessage(transforms=[
            static_tf('map', 'base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            static_tf('base_link', OPTICAL, (0.0, 0.0, 0.3), (-0.5, 0.5, -0.5, 0.5)),   # looking along map +x
            static_tf('map', LASER, (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))]))
        pump(3.0)   # frames with the doll; the first measured clock arrives within the poll period
        o = observations[0] if observations else None
        checks['doll -> one TargetObservation with a located point'] = bool(o) and o.localization_status == 'VALID' \
            and abs(o.sensor_position.z - POINT[0]) < 0.05 and abs(o.sensor_position.x) < 0.05
        bp = base_points[0] if base_points else None
        checks['capture-time base_link XYZ matches camera point transformed by TF'] = bool(bp and o) and \
            bp.header.frame_id == 'base_link' and \
            bp.header.stamp.sec * 10**9 + bp.header.stamp.nanosec == o.time.observed_at_ns and \
            abs(bp.point.x - o.sensor_position.z) < 1e-6 and \
            abs(bp.point.y + o.sensor_position.x) < 1e-6 and \
            abs(bp.point.z - (0.3 - o.sensor_position.y)) < 1e-6
        if bp:
            print(f'  base_link: ({bp.point.x:.3f}, {bp.point.y:.3f}, {bp.point.z:.3f})')
        if o:
            print(f'  observation: point=({o.sensor_position.x:.3f}, {o.sensor_position.y:.3f}, '
                  f'{o.sensor_position.z:.3f}) clock={o.time.clock_state} bound={o.time.clock_error_bound_ns} '
                  f'domain={o.time.clock_domain_id}')
        r = observe()
        p = r.observation
        print(f'  doll at point: {r.outcome} camera={p.camera_state} lidar={p.lidar_state} '
              f'coverage={p.coverage_state} {p.detector_health} clock={p.time.clock_state}')
        checks['doll at point -> PRESENT / BLOCKED / VALID'] = (r.outcome, p.camera_state, p.lidar_state,
                                                                 p.coverage_state) == ('OBSERVED', 'PRESENT', 'BLOCKED', 'VALID')
        scene['doll'] = False
        pump(0.5)
        r = observe()
        p = r.observation
        print(f'  doll gone:     {r.outcome} camera={p.camera_state} lidar={p.lidar_state} '
              f'coverage={p.coverage_state} {p.detector_health} clock={p.time.clock_state}')
        checks['doll gone -> ABSENT / CLEAR / VALID'] = (r.outcome, p.camera_state, p.lidar_state,
                                                         p.coverage_state) == ('OBSERVED', 'ABSENT', 'CLEAR', 'VALID')
        scene['qr'] = 'exit-1'   # not exactly a listed exit
        pump(2.0)
        checks['other QR content -> no exit observation'] = not exits
        scene['qr'] = EXIT_PAYLOADS[0]
        pump(3.0)
        e = exits[0] if exits else None
        if e:
            print(f'  exit: {e.class_label} {e.model_id} point=({e.sensor_position.x:.3f}, {e.sensor_position.y:.3f}, '
                  f'{e.sensor_position.z:.3f}) clock={e.time.clock_state}')
        checks['exit QR -> one exit_qr observation at the QR depth'] = len(exits) == 1 and e.class_label == 'exit_qr' \
            and e.localization_status == 'VALID' and abs(e.sensor_position.z - QR_MM / 1000 * SCALE) < 0.05 \
            and e.confidence == 1.0 and e.model_id.startswith('opencv.QRCodeDetector/') \
            and (e.tracking_mode, e.has_track_id, e.track_id) == ('TRACKING', True, 'EXIT-1')
        map_state = MapState(robot_id='robot_a', map_session_id='e3-session', map_revision=1,
                             localization_quality='VALID', current_native_map_frame='map')
        map_state.generated_at_ns = node.get_clock().now().nanoseconds
        map_pub.publish(map_state)
        pump(3.0)
        checks['new map revision -> one more exit observation'] = len(exits) == 2 and exits[1].track_id == 'EXIT-1'
        scene['qr'] = EXIT_PAYLOADS[1]
        pump(3.0)
        checks['second listed exit -> one observation with its own track_id'] = (
            [x.track_id for x in exits] == ['EXIT-1', 'EXIT-1', 'EXIT-2'])
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        child.send_signal(2)
        try:
            output = child.communicate(timeout=15)[0]
        except subprocess.TimeoutExpired:
            child.kill()
            output = child.communicate()[0]
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    if not all(checks.values()):
        print('--- amr_target output (tail) ---\n' + '\n'.join(output.splitlines()[-30:]))
    return 0 if checks and all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
