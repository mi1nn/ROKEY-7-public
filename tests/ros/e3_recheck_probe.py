"""E3 (partial): ObserveTarget action server -> RecheckObservation on a local ROS graph (2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    source <scratch>/install/setup.bash   # colcon build --packages-select rescue_interfaces
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_mapping:<REPO>/src/rescue_perception:$PYTHONPATH \
      python3 tests/ros/e3_recheck_probe.py

The server runs with synthetic sensor input pushed by a feeder thread (what the camera/LiDAR
node does on the robot) and the host clock measured with timesyncd when available. Checks:
1. rescued spot (in view, no detection, LiDAR clear) -> OBSERVED ABSENT/CLEAR/VALID, window after
the goal; 2. a confirmed detection at the point -> PRESENT, LiDAR BLOCKED; 3. cancel -> UNKNOWN with
every state UNKNOWN; 4. another robot's goal -> FAILED ROBOT_MISMATCH; 5. samples only before the
goal -> nothing counts (UNKNOWN states); 6. two goals sent back to back -> the second is rejected
(one window at a time, #165 review CDX-165-03). Exit code 0 = all held.
"""
import os
import sys
import threading
import time
from uuid import uuid4

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rescue_contracts.clock_measurement import ClockSampler
from rescue_interfaces.action import ObserveTarget
from rescue_perception.recheck import RecheckPolicy
from rescue_perception.recheck_server import RecheckServer

POLICY = RecheckPolicy(presence_radius_m=0.5, lidar_radius_m=0.3, min_frames=3, min_scans=3, min_lidar_rays=3,
                       min_lidar_hits=2, image_margin_px=10, min_depth_m=0.2, max_depth_m=5.0,
                       occlusion_tolerance_m=0.3, depth_window_px=2)
ACTION = '/rescue/robots/robot_a/observe_target'
POINT = (2.0, 1.0, 0.3)


def main():
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('e3_recheck_server')
    sampler = ClockSampler('timesyncd', 3_600_000_000_000, 100.0)
    clock = {'record': sampler.poll()}
    node.create_timer(0.5, lambda: clock.update(record=sampler.poll()))
    server = RecheckServer(node, action_name=ACTION, robot_id='robot_a', map_frame='map', policy=POLICY,
                           max_window_ns=10_000_000_000, validity_ns=5_000_000_000, clock_domain_id='rokey7-utc-field',
                           clock_record=lambda: clock['record'], publisher_id='e3.recheck', envelope_version='0.1',
                           callback_group=ReentrantCallbackGroup())
    client_node = rclpy.create_node('e3_recheck_client')
    client = ActionClient(client_node, ObserveTarget, ACTION)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    scene = {'mode': 'empty', 'feed': True}

    def feed():
        while scene['feed'] is not None:
            if scene['feed']:
                now = server.now_ns()
                present = scene['mode'] == 'present'
                server.add_frame(received_ns=now, coverage='VALID', unconfirmed_at_point=False,
                                 confirmed_points=[(2.1, 1.0, 0.3)] if present else [])
                server.add_scan(received_ns=now, state='BLOCKED' if present else 'CLEAR')
            time.sleep(0.1)
    threading.Thread(target=feed, daemon=True).start()

    def goal(robot='robot_a', window_s=1.0):
        g = ObserveTarget.Goal(request_id=str(uuid4()), task_id=str(uuid4()), target_id=str(uuid4()), robot_id=robot,
                               requested_window_ns=int(window_s * 1e9), policy_version='e3',
                               deadline_ns=server.now_ns() + 30_000_000_000)
        g.watched_native_point.x, g.watched_native_point.y, g.watched_native_point.z = POINT
        f = g.frame_ref
        f.robot_id, f.native_frame, f.mission_frame = robot, 'map', 'map'
        f.map_session_id, f.map_revision, f.transform_revision, f.alignment_status = (
            '22222222-2222-4222-8222-222222222222', 0, 1, 'VALID')
        return g

    def run(g, cancel_after=None):
        sent = server.now_ns()
        handle_future = client.send_goal_async(g)
        while not handle_future.done():
            time.sleep(0.02)
        handle = handle_future.result()
        if cancel_after is not None:
            time.sleep(cancel_after)
            handle.cancel_goal_async()
        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.02)
        return sent, result_future.result().result

    checks = {}
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise SystemExit('action server not found')
        end = time.monotonic() + 8
        while clock['record'].get('state') != 'SYNCED' and time.monotonic() < end:
            time.sleep(0.2)
        clock_ok = clock['record'].get('state') == 'SYNCED'
        sent, r = run(goal())
        o = r.observation
        checks['rescued spot -> OBSERVED ABSENT/CLEAR/VALID'] = (r.outcome, o.camera_state, o.lidar_state,
                                                                  o.coverage_state) == ('OBSERVED', 'ABSENT', 'CLEAR', 'VALID')
        checks['window starts after the goal was sent'] = o.sensor_window_start_ns >= sent and (
            o.sensor_window_end_ns > o.sensor_window_start_ns)
        checks['observation time = measured host clock'] = (o.time.clock_state == 'SYNCED' and o.time.has_clock_error_bound
                                                             ) if clock_ok else o.time.clock_state == 'UNKNOWN'
        print(f'  absent: {o.detector_health} clock={o.time.clock_state} bound={o.time.clock_error_bound_ns}')
        scene['mode'] = 'present'
        _, r = run(goal())
        checks['detection at the point -> PRESENT, LiDAR BLOCKED'] = (r.observation.camera_state,
                                                                      r.observation.lidar_state) == ('PRESENT', 'BLOCKED')
        scene['mode'] = 'empty'
        _, r = run(goal(window_s=5.0), cancel_after=0.5)
        o = r.observation
        checks['cancel -> UNKNOWN, no absence'] = (r.outcome, r.reason_code, o.camera_state, o.lidar_state,
                                                   o.coverage_state) == ('UNKNOWN', 'CANCELED', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN')
        _, r = run(goal(robot='robot_b'))
        checks['other robot -> FAILED ROBOT_MISMATCH'] = (r.outcome, r.reason_code) == ('FAILED', 'ROBOT_MISMATCH')
        first_future = client.send_goal_async(goal(window_s=2.0))
        second_future = client.send_goal_async(goal(window_s=2.0))   # before the first one executes
        while not (first_future.done() and second_future.done()):
            time.sleep(0.02)
        first, second = first_future.result(), second_future.result()
        if first.accepted:
            result = first.get_result_async()
            while not result.done():
                time.sleep(0.02)
        checks['back-to-back goals -> second rejected'] = first.accepted and not second.accepted
        _, r = run(goal())
        checks['slot free again after the first ends'] = r.outcome == 'OBSERVED'
        scene['feed'] = False   # camera and LiDAR stopped: only older samples exist
        _, r = run(goal())
        o = r.observation
        checks['no new samples -> UNKNOWN states'] = (o.camera_state, o.lidar_state, o.coverage_state,
                                                      o.detector_health) == ('UNKNOWN', 'UNKNOWN', 'UNKNOWN',
                                                                             'frames=0 in_view=0 scans=0')
    finally:
        scene['feed'] = None
        sampler.close()
        executor.shutdown()
        rclpy.try_shutdown()
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    return 0 if checks and all(checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
