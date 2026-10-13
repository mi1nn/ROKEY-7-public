"""E3 (partial): headroom_record on a local ROS graph with synthetic TB-5 depth (OPS-115, 2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy). Run on an isolated graph, never a robot's
domain, with no Discovery Server:

    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_perception:$PYTHONPATH python3 tests/ros/e3_headroom_probe.py

A camera 0.28 m up looking ahead sees a flat surface 0.8 m away (64x48, fx=fy=40). Checks:
1. the whole surface (a plate down to the floor) -> every run STOP; 2. only rows 0.48 m and up
(a crossbar above the 46 cm clearance, nothing below) -> every run PASS, lowest 0.48 m;
3. no depth published -> every run UNKNOWN NO_DEPTH; 4. no TF to base_frame -> every run UNKNOWN
NO_TF. Neither is ever PASS. Exit code 0 = all held.
"""
import json
import os
import subprocess
import sys
import tempfile
import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

PARAMS = dict(depth_topic='/e3/depth', camera_info_topic='/e3/camera_info', base_frame='base_link', samples=3,
              stride=1, max_depth_age_s=0.5, clearance_m=0.46, floor_noise_m=0.05, box_x_min_m=0.3,
              box_x_max_m=1.2, box_half_width_m=0.2, min_points=3)


def record(node, scene):
    """Run headroom_record against ``scene`` ('plate', 'crossbar', None) and return its JSON."""
    path = tempfile.mktemp(suffix='.json')
    args = [a for k, v in {**PARAMS, 'output_path': path}.items() for a in ('-p', f'{k}:={v}')]
    stop = threading.Event()

    def publish():
        info_pub = node.create_publisher(CameraInfo, '/e3/camera_info', 1)
        depth_pub = node.create_publisher(Image, '/e3/depth', 1)
        depth = np.full((48, 64), 0.8, dtype=np.float32)
        if scene == 'crossbar':
            depth[15:, :] = 0.0   # rows below 0.48 m: no return
        while not stop.is_set() and scene is not None:
            stamp = node.get_clock().now().to_msg()
            info = CameraInfo(width=64, height=48, k=[40.0, 0.0, 32.0, 0.0, 40.0, 24.0, 0.0, 0.0, 1.0])
            info.header.stamp, info.header.frame_id = stamp, 'camera_optical'
            image = Image(height=48, width=64, encoding='32FC1', step=64 * 4, data=depth.tobytes())
            image.header.stamp, image.header.frame_id = stamp, 'camera_optical'
            info_pub.publish(info)
            depth_pub.publish(image)
            stop.wait(0.05)
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    subprocess.run([sys.executable, '-m', 'rescue_perception.headroom', '--ros-args', *args], check=True, timeout=60)
    stop.set()
    thread.join()
    with open(path) as handle:
        return json.load(handle)


def main():
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('e3_headroom_feed')
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'camera_optical'
    tf.transform.translation.z = 0.28
    r = tf.transform.rotation
    r.x, r.y, r.z, r.w = -0.5, 0.5, -0.5, 0.5
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(tf)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    checks = {}
    try:
        plate = record(node, 'plate')
        print('  plate', plate['summary'], [r['reason'] for r in plate['runs']])
        checks['plate down to the floor -> every run STOP'] = plate['summary']['counts'] == {'STOP': 3}
        bar = record(node, 'crossbar')
        print('  crossbar', bar['summary'], [r['reason'] for r in bar['runs']])
        checks['crossbar from 0.48 m -> every run PASS, lowest 0.48 m'] = (
            bar['summary']['counts'] == {'PASS': 3} and abs(bar['summary']['lowest_min_m'] - 0.48) < 1e-3)
        checks['record carries git and the clearance source'] = bool(bar['git']) and '46 cm' in bar['clearance_source']
        none = record(node, None)
        checks['no depth -> UNKNOWN NO_DEPTH, never PASS'] = (
            none['summary']['counts'] == {'UNKNOWN': 3} and {r['reason'] for r in none['runs']} == {'NO_DEPTH'})
        tf.header.frame_id = 'other_base'   # TF never reaches base_link: UNKNOWN with the TF reason
        broadcaster.sendTransform(tf)
        PARAMS['base_frame'] = 'missing_base'
        no_tf = record(node, 'plate')
        checks['no TF -> UNKNOWN NO_TF, never PASS'] = (
            no_tf['summary']['counts'] == {'UNKNOWN': 3} and all(r['reason'].startswith('NO_TF') for r in no_tf['runs']))
    finally:
        executor.shutdown()
        spin.join()
        node.destroy_node()
        rclpy.try_shutdown()
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    return 0 if checks and all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
