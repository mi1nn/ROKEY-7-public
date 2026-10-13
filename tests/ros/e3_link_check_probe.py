"""E3: scripts/field/link_check.py against fake publishers on a local ROS graph (2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_bringup:$PYTHONPATH python3 tests/ros/e3_link_check_probe.py <REPO> <SCRATCH_DIR>

This probe publishes robot_a's topics of config/deployment.v2.example.json: executor heartbeat at
20 Hz with one 0.7 s pause, watchdog heartbeat and odom at 20 Hz, TF at 10 Hz, RobotStatus at 5 Hz
(observed_at_ns = send time), cam_fixed CameraStatus at 2 Hz, and no scan. It runs link_check for 4 s
(1 s warm-up). Checks:
1. executor_heartbeat FAILED, one silence over 0.5 s and none over 1.0 s, longest ~0.7 s;
2. watchdog_heartbeat, odom, tf, robot_status, camera_status PASSED with counts near their rates;
3. scan FAILED with nothing received; exit code 2; the JSON file matches the table;
4. stamp delay present for the Header topics and small (same host clock);
5. while it ran, the link_check node published on no topic and offered no service.
Exit code 0 = all held.
"""
import json
import os
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rescue_interfaces.msg import CameraStatus, RobotStatus
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

DURATION, WARMUP = 4.0, 1.0


def main():
    repo, scratch = sys.argv[1], sys.argv[2]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('e3_link_check_fake')
    pub = dict(executor=node.create_publisher(Header, '/robot_a/rescue/executor_heartbeat', 10),
               watchdog=node.create_publisher(Header, '/robot_a/rescue/watchdog_heartbeat', 10),
               odom=node.create_publisher(Odometry, '/robot_a/odom', 10),
               tf=node.create_publisher(TFMessage, '/robot_a/tf', 10),
               status=node.create_publisher(RobotStatus, '/rescue/robots/robot_a/status', 10),
               camera=node.create_publisher(CameraStatus, '/rescue/cameras/cam_fixed/status', 10))
    clock = node.get_clock()
    paused = {'until': 0.0}
    started = time.monotonic()

    def stamped(msg):
        stamp = clock.now().to_msg()
        (msg.header if hasattr(msg, 'header') else msg).stamp = stamp
        return msg

    def fast():
        t = time.monotonic() - started
        if not WARMUP + 1.5 <= t < WARMUP + 2.2:  # one 0.7 s pause inside the window
            pub['executor'].publish(stamped(Header()))
        pub['watchdog'].publish(stamped(Header()))
        pub['odom'].publish(stamped(Odometry()))

    def tf():
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = clock.now().to_msg(), 'map', 'odom'
        pub['tf'].publish(TFMessage(transforms=[t]))

    node.create_timer(0.05, fast)
    node.create_timer(0.1, tf)
    node.create_timer(0.2, lambda: pub['status'].publish(RobotStatus(robot_id='robot_a',
                                                                     observed_at_ns=clock.now().nanoseconds)))
    node.create_timer(0.5, lambda: pub['camera'].publish(CameraStatus(camera_id='cam_fixed')))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()
    spinner = threading.Thread(target=lambda: [executor.spin_once(timeout_sec=.01) for _ in iter(stop.is_set, True)])
    spinner.start()
    out_json = os.path.join(scratch, f'link_check-{os.getpid()}.json')
    tool = subprocess.Popen([sys.executable, os.path.join(repo, 'scripts', 'field', 'link_check.py'),
                             '--profile', os.path.join(repo, 'config', 'deployment.v2.example.json'),
                             '--robot', 'robot_a', '--duration-s', str(DURATION), '--warmup-s', str(WARMUP),
                             '--heartbeat-max-age-s', '0.5', '--json', out_json],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    footprint = {'topics': set(), 'services': set(), 'seen': False}
    time.sleep(WARMUP + 1.0)
    for name in node.get_node_names():
        if name.startswith('link_check_'):
            footprint['seen'] = True
            footprint['topics'] |= {t for t, _ in node.get_publisher_names_and_types_by_node(name, '/')}
            footprint['services'] |= {s for s, _ in node.get_service_names_and_types_by_node(name, '/')}
    text, _ = tool.communicate(timeout=30)
    stop.set()
    spinner.join()
    node.destroy_node()
    rclpy.try_shutdown()
    print(text)
    result = json.load(open(out_json))
    rows = {r['role']: r for r in result['topics']}
    st = {role: r['stats'] for role, r in rows.items()}
    checks = {}
    e = st['executor_heartbeat']
    checks['1 executor heartbeat pause -> FAILED, one >0.5 s, none >1.0 s, max ~0.7 s'] = (
        rows['executor_heartbeat']['status'], e['gaps_over_0_5_s'], e['gaps_over_1_0_s']) == ('FAILED', 1, 0) and (
        0.6 <= e['max_gap_s'] <= 0.85)
    expect = {'watchdog_heartbeat': 80, 'odom': 80, 'tf': 40, 'robot_status': 20, 'camera_status:cam_fixed': 8}
    checks['2 steady topics PASSED with counts near their rates'] = all(
        rows[role]['status'] == 'PASSED' and 0.8 * n <= st[role]['count'] <= 1.2 * n for role, n in expect.items())
    checks['3 no scan -> FAILED count 0; exit 2; JSON exit_code 2'] = (
        rows['scan']['status'], st['scan']['count'], tool.returncode, result['exit_code']) == ('FAILED', 0, 2, 2)
    d = rows['watchdog_heartbeat']['stamp_delay']
    checks['4 stamp delay [참고] present and small on one host'] = bool(d) and d['basis'] == '참고' and (
        -0.05 < d['mean_s'] < 0.1)
    checks['5 link_check published nothing, offered no service'] = footprint['seen'] and not footprint['topics'] and (
        not footprint['services'])
    print(f'footprint={footprint}')
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    print(f'PASS {sum(checks.values())} FAIL {len(checks) - sum(checks.values())}')
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
