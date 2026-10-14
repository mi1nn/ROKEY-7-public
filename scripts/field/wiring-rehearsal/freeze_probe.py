#!/usr/bin/env python3
"""One-PC rehearsal, frozen map path: synthetic TB-6 SLAM and TB-5 AMCL data, one freeze, then what reached whom.

Run by run.sh when REHEARSAL_PROBE=freeze, after wiring_check, on the same local Discovery Server graph.
It publishes synthetic SLAM / AMCL / Pi data (feed.py; TB-6 TF measured on 2026-10-14 by default, --tf flat
for an exact plane), calls the map freeze service (the coordinator's own call; no robot command) and reports
each step PASSED / FAILED with what arrived:
1. TB-6 MapState from the executor (quality, revision)        4. /rescue/map/frozen + frozen_revision
2. TB-6 start pose from map_freeze                            5. /robot5/map from the TB-5 relay
3. the freeze answer                                          6. TB-5 MapState after the freeze
--final (slam_phase FROZEN): 7. second freeze, 8. TB-6 relay after SLAM stops, 9-10. both MapStates R2.
The data are synthetic and say nothing about a real map, SLAM or AMCL.
"""
import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rescue_interfaces.msg import MapState
from std_msgs.msg import String

from feed import LATCHED, Feeder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--final', action='store_true', help='slam_phase FROZEN: second freeze, TB-6 leaves SLAM')
    ap.add_argument('--tf', choices=('measured', 'flat'), default='measured', help='TB-6 TF (feed.py)')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('rehearsal_freeze_probe')
    feeder = Feeder(node, tf=args.tf)
    print(f'TB-6 TF: {args.tf}', flush=True)
    seen = {}

    def keep(key):
        return lambda msg: seen.setdefault(key, []).append(msg)
    for key, kind, topic, qos in (('state_a', MapState, '/rescue/robots/robot_a/map_state', LATCHED),
                                  ('state_b', MapState, '/rescue/robots/robot_b/map_state', LATCHED),
                                  ('start', PoseWithCovarianceStamped, '/rescue/map/start_pose/robot_a', LATCHED),
                                  ('frozen', OccupancyGrid, '/rescue/map/frozen', LATCHED),
                                  ('revision', String, '/rescue/map/frozen_revision', LATCHED),
                                  ('map5', OccupancyGrid, '/robot5/map', LATCHED)):
        node.create_subscription(kind, topic, keep(key), qos)
    steps = []

    def spin(seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(node, timeout_sec=0.05)

    def step(name, ok, detail):
        steps.append(dict(step=name, status='PASSED' if ok else 'FAILED', detail=detail))
        print(f'{"PASSED" if ok else "FAILED":8} {name}: {detail}', flush=True)

    def state(msg):
        return dict(revision=msg.map_revision, quality=msg.localization_quality, session=msg.map_session_id,
                    frame=msg.current_native_map_frame) if msg else None
    spin(30, lambda: any(m.localization_quality == 'VALID' for m in seen.get('state_a', [])))
    last = (seen.get('state_a') or [None])[-1]
    step('1 TB-6 MapState VALID', last is not None and last.localization_quality == 'VALID', state(last))
    spin(10, lambda: 'start' in seen)
    step('2 TB-6 start pose', 'start' in seen, 'received' if 'start' in seen else 'none in 10 s')
    # std_srvs/Trigger, as map_freeze serves it (a plain client cannot list services under a Discovery Server)
    from std_srvs.srv import Trigger
    kind, answer = 'std_srvs/srv/Trigger', None
    client = node.create_client(Trigger, '/rescue/map/freeze')
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(Trigger.Request())
        spin(15, future.done)
        answer = future.result() if future.done() else None
    step('3 freeze answer', answer is not None and answer.success,
         f'{kind}: ' + (f'success={answer.success} {answer.message}' if answer else 'no answer'))
    spin(10, lambda: 'frozen' in seen and 'revision' in seen)
    step('4 frozen map + revision', 'frozen' in seen and 'revision' in seen,
         dict(frozen=len(seen.get('frozen', [])), revision=[m.data for m in seen.get('revision', [])][-1:]))
    spin(20, lambda: any(m.info.width == 80 for m in seen.get('map5', [])))
    got = (seen.get('map5') or [None])[-1]
    step('5 TB-5 map from the relay', got is not None and got.info.width == 80,
         dict(width=got.info.width, frame=got.header.frame_id) if got else 'no /robot5/map in 20 s')
    spin(20, lambda: any(m.localization_quality == 'VALID' for m in seen.get('state_b', [])))
    last_b = (seen.get('state_b') or [None])[-1]
    step('6 TB-5 MapState after the freeze', last_b is not None and last_b.localization_quality == 'VALID',
         state(last_b))
    if args.final:   # slam_phase FROZEN: the TB-6 relay is already running, waiting for SLAM to stop
        future = client.call_async(Trigger.Request())
        spin(15, future.done)
        answer = future.result() if future.done() else None
        step('7 second (final) freeze', answer is not None and answer.success,
             f'success={answer.success} {answer.message}' if answer else 'no answer')
        feeder.stop_slam()   # TB-6 stops SLAM and AMCL takes over
        seen.pop('state_a', None)
        node.create_subscription(OccupancyGrid, '/robot6/map', keep('map6_relay'), LATCHED)
        spin(30, lambda: any(m.map_revision == 2 and m.localization_quality == 'VALID'
                             for m in seen.get('state_a', [])))
        got6 = [m for m in seen.get('map6_relay', []) if m.info.width == 80]
        step('8 TB-6 map from its relay after SLAM stops', bool(got6), f'{len(got6)} grids')
        last = (seen.get('state_a') or [None])[-1]
        step('9 TB-6 MapState R2 VALID', last is not None and last.map_revision == 2
             and last.localization_quality == 'VALID', state(last))
        spin(20, lambda: any(m.map_revision == 2 and m.localization_quality == 'VALID'
                             for m in seen.get('state_b', [])))
        last_b = (seen.get('state_b') or [None])[-1]
        step('10 TB-5 MapState R2 VALID', last_b is not None and last_b.map_revision == 2
             and last_b.localization_quality == 'VALID', state(last_b))
    print(json.dumps(steps, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all(s['status'] == 'PASSED' for s in steps) else 2


if __name__ == '__main__':
    sys.exit(main())
