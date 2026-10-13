"""E3 (partial): the real overhead_passage node on synthetic depth points -> the map list (OPS-185/186).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy, a built rescue_interfaces, NumPy).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    source <scratch>/install/setup.bash   # colcon build --packages-select rescue_interfaces
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_mapping:<REPO>/src/rescue_perception:$PYTHONPATH \
      python3 tests/ros/e3_overhead_probe.py <REPO>

The probe ray-casts an organized PointCloud2 (176x176, optical frame) for a camera 0.244 m above
base_link looking ahead: floor, and either a plate at a height over base x 0.6-1.2 m or a wall
from the floor at x 0.8 m. Static TF map -> base_link (x 2, y 1, yaw 90 deg) -> optical, and a
VALID TB-6 MapState. All values are synthetic test values. Checks:
1. a missing value -> exit 2; 2. an empty list at start (latched); 3. plate 0.47 m -> one PASSABLE
item, lowest 0.47, at the map point under the plate; 4. plate 0.40 m at the same place -> the same
item BLOCKED 0.40; 5. a wall from the floor -> no new item; 6. a new map revision -> a second item
with that revision, the first kept. Exit code 0 = all held.
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_contracts.overhead import parse_overhead_passages
from rescue_interfaces.msg import MapState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SIZE, F, CAM_H = 176, 112.0, 0.244
OPTICAL = 'e3_camera_optical'
SESSION = '22222222-2222-4222-8222-222222222222'
TOPIC = '/rescue/robots/robot_a/overhead_passages'
MAP_STATE = '/rescue/robots/robot_a/map_state'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PARAMS = dict(points_topic='/e3robot/passage_points', base_frame='base_link', map_frame='map', robot_id='robot_a',
              map_robot_id='robot_a', map_state_topic=MAP_STATE, clock_error_bound_ns=0, process_period_s=0.3,
              max_cloud_age_s=1.0, stride=1, clearance_m=0.46, floor_noise_m=0.05, front_near_m=0.4,
              front_far_m=1.0, half_width_m=0.2, lidar_plane_m=0.23, top_m=1.0, min_points=5, floor_min_points=20,
              confirm_frames=3, position_tolerance_m=0.3, merge_radius_m=0.5)


def command(params):
    flags = []
    for name, value in params.items():
        flags += ['-p', f'{name}:={value!r}' if isinstance(value, float) else f'{name}:={value}']
    code = 'from rescue_perception.overhead import main; raise SystemExit(main())'
    return [sys.executable, '-c', code, '--ros-args'] + flags + ['-r', '/tf:=/e3robot/tf',
                                                                 '-r', '/tf_static:=/e3robot/tf_static']


def cloud(scene, stamp_ns):
    """Ray-cast depth points: floor + scene ('plate', height) or ('wall', x)."""
    v, u = np.mgrid[0:SIZE, 0:SIZE].astype(np.float64)
    dx, dy = (u - SIZE / 2) / F, (v - SIZE / 2) / F     # optical ray (dx, dy, 1); base x = depth, z = CAM_H - dy*depth
    depth = np.full((SIZE, SIZE), 4.0)
    down = dy > 0
    depth = np.minimum(depth, np.where(down, CAM_H / np.where(down, dy, 1), np.inf))
    kind, value = scene
    if kind == 'plate':
        up = dy < 0
        t = np.where(up, (value - CAM_H) / np.where(up, -dy, 1), np.inf)
        depth = np.where((t >= 0.6) & (t <= 1.2) & (t < depth), t, depth)
    elif kind == 'wall':
        height = CAM_H - dy * value
        depth = np.where((height >= 0.0) & (height <= 0.9) & (value < depth), value, depth)
    points = np.zeros((SIZE, SIZE, 4), np.float32)
    points[..., 0], points[..., 1], points[..., 2] = dx * depth, dy * depth, depth
    msg = PointCloud2(height=SIZE, width=SIZE, is_bigendian=False, point_step=16, row_step=16 * SIZE, is_dense=True)
    msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                  for n, o in (('x', 0), ('y', 4), ('z', 8))]
    msg.header.frame_id = OPTICAL
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    msg.data = points.tobytes()
    return msg


def static_tf(parent, child, xyz, q):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t


def main():
    repo = sys.argv[1]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    checks = {}
    missing = {k: v for k, v in PARAMS.items() if k != 'lidar_plane_m'}
    refused = subprocess.run(command(missing), cwd=repo, capture_output=True, text=True, timeout=60)
    checks['missing value -> exit 2'] = refused.returncode == 2 and 'lidar_plane_m' in refused.stderr
    rclpy.init()
    node = rclpy.create_node('e3_overhead_probe')
    cloud_pub = node.create_publisher(PointCloud2, '/e3robot/passage_points',
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    static_pub = node.create_publisher(TFMessage, '/e3robot/tf_static', LATCHED)
    map_pub = node.create_publisher(MapState, MAP_STATE, LATCHED)
    lists = []
    node.create_subscription(String, TOPIC, lambda m: lists.append(parse_overhead_passages(m.data)), LATCHED)
    yaw = math.pi / 2
    static_pub.publish(TFMessage(transforms=[
        static_tf('map', 'base_link', (2.0, 1.0, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))),
        static_tf('base_link', OPTICAL, (0.0, 0.0, CAM_H), (-0.5, 0.5, -0.5, 0.5))]))

    def map_state(revision):
        m = MapState(robot_id='robot_a', map_session_id=SESSION, map_revision=revision, localization_quality='VALID',
                     current_native_map_frame='map')
        m.generated_at_ns = node.get_clock().now().nanoseconds
        map_pub.publish(m)

    map_state(1)
    child = subprocess.Popen(command(PARAMS), cwd=repo, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    scene = {'now': ('none', None)}

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            cloud_pub.publish(cloud(scene['now'], node.get_clock().now().nanoseconds))
            rclpy.spin_once(node, timeout_sec=0.1)

    try:
        pump(4.0)
        checks['empty list at start (latched)'] = bool(lists) and lists[-1]['items'] == []
        scene['now'] = ('plate', 0.47)
        pump(4.0)
        items = lists[-1]['items'] if lists else []
        first = items[0] if items else None
        if first:
            print(f'  plate 0.47: {first["state"]} lowest={first["lowest_m"]} map=({first["x"]:.3f}, {first["y"]:.3f}) '
                  f'revision={first["map_revision"]} frames={first["frames"]}')
        # under the plate: base x about 0.6-1.0 (the box ends at 1.0), y 0 -> map (2, 1 + x)
        checks['plate 0.47 -> one PASSABLE item at the map point under it'] = (
            len(items) == 1 and first['state'] == 'PASSABLE' and abs(first['lowest_m'] - 0.47) < 0.01
            and abs(first['x'] - 2.0) < 0.05 and 1.6 < first['y'] < 2.0 and first['map_revision'] == 1)
        scene['now'] = ('plate', 0.40)
        pump(4.0)
        items = lists[-1]['items']
        checks['plate 0.40 at the same place -> the same item BLOCKED'] = (
            len(items) == 1 and items[0]['id'] == first['id'] and items[0]['state'] == 'BLOCKED'
            and abs(items[0]['lowest_m'] - 0.40) < 0.01)
        count = len(lists)
        scene['now'] = ('wall', 0.8)
        pump(3.0)
        checks['wall from the floor -> no new item'] = len(lists) == count and len(lists[-1]['items']) == 1
        map_state(2)
        pump(1.0)
        scene['now'] = ('plate', 0.47)
        pump(4.0)
        items = lists[-1]['items']
        checks['new map revision -> a second item with it, the first kept'] = (
            [(i['map_revision'], i['state']) for i in items] == [(1, 'BLOCKED'), (2, 'PASSABLE')])
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        child.send_signal(2)
        try:
            output = child.communicate(timeout=15)[0]
        except subprocess.TimeoutExpired:
            child.kill()
            output = child.communicate()[0]
    print('  ' + '\n  '.join(line for line in output.splitlines() if 'mean_ms' in line)[-400:])
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    if not all(checks.values()):
        print('--- overhead_passage output (tail) ---\n' + '\n'.join(output.splitlines()[-30:]))
        print(json.dumps(lists[-1] if lists else None, indent=1))
    return 0 if checks and all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
