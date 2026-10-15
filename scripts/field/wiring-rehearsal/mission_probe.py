#!/usr/bin/env python3
"""One-PC rehearsal, mission start: DISASTER -> TB-6 FRONTIER goal -> executor -> (fake, refusing) Nav2 -> stop latch.

Run by run.sh when REHEARSAL_PROBE=mission (slam_phase SEARCHING). Synthetic data from feed.py (TB-6 TF flat:
the measured TF stops TB-6 MapState on main, see freeze_probe.py). Steps, each PASSED / FAILED with what arrived:
1. both executors report a fresh status          4. a TB-6 task is admitted (TaskAdmission)
2. refresh_state, task_arm on both (step 3)      5. the fake bt_navigator refuses the NavigateToPose goal, so
3. a DISASTER_SIGN (as the webcam) -> DISASTER      the task ends FAILED and TB-6 stays STOP_LATCHED:
                                                    RobotStatus NOT_READY, stop_reason not REARM_REFUSED_*
                                                    (OPS-231 re-arms only after COMPLETED)
No robot is on this graph; the fake Nav2 never accepts a goal, so nothing would move even if one were.
"""
import json
import sys
import time
from uuid import uuid4

import rclpy
from rescue_interfaces.msg import RobotStatus, SiteEvent, SystemState, TaskAdmission
from std_srvs.srv import Trigger

from feed import LATCHED, Feeder

CAMERA, CLOCK_DOMAIN = 'cam_fixed', 'rokey7-utc-field'   # profile site_cameras; field target_clock_domain_id


def main():
    rclpy.init()
    node = rclpy.create_node('rehearsal_mission_probe')
    Feeder(node, tf='flat')
    seen = {}

    def keep(key):
        return lambda msg: seen.setdefault(key, []).append(msg)
    for robot in ('robot_a', 'robot_b'):
        node.create_subscription(RobotStatus, f'/rescue/robots/{robot}/status', keep('status_' + robot), 10)
        node.create_subscription(TaskAdmission, f'/rescue/robots/{robot}/task_admission', keep('adm_' + robot), 10)
    node.create_subscription(SystemState, '/rescue/system_state', keep('state'), LATCHED)
    site = node.create_publisher(SiteEvent, '/rescue/site_events', 10)
    steps = []

    def spin(seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(node, timeout_sec=0.05)

    def step(name, ok, detail):
        steps.append(dict(step=name, status='PASSED' if ok else 'FAILED', detail=detail))
        print(f'{"PASSED" if ok else "FAILED":8} {name}: {detail}', flush=True)

    def status(robot):
        msgs = seen.get('status_' + robot) or [None]
        m = msgs[-1]
        return None if m is None else dict(readiness=m.readiness, task_state=m.task_state, motion=m.motion_state,
                                           stop_reason=m.stop_reason, active=m.active_task_id)
    spin(20, lambda: all(seen.get('status_' + r) for r in ('robot_a', 'robot_b')))
    step('1 executor status', all(seen.get('status_' + r) for r in ('robot_a', 'robot_b')),
         {r: status(r) for r in ('robot_a', 'robot_b')})
    # field step 3 order (rescue_bringup README): coordinator state again, then arm each executor
    refresh = node.create_client(Trigger, '/mission_coordinator/refresh_state')
    refreshed = 'no service'
    if refresh.wait_for_service(timeout_sec=5.0):
        future = refresh.call_async(Trigger.Request())
        spin(5, future.done)
        refreshed = f'{future.result().success} {future.result().message}' if future.done() else 'no answer'
    spin(2)
    armed = {'refresh_state': refreshed}
    for robot, ns in (('robot_a', '/robot6'), ('robot_b', '/robot5')):
        client = node.create_client(Trigger, f'{ns}/rescue_command_adapter/task_arm')
        answer = None
        if client.wait_for_service(timeout_sec=5.0):
            for _ in range(10):   # readiness may need a moment after the first sensor data
                future = client.call_async(Trigger.Request())
                spin(5, future.done)
                answer = future.result() if future.done() else None
                if answer is not None and answer.success:
                    break
                spin(1)
        armed[robot] = f'{answer.success} {answer.message}' if answer else 'no answer'
    spin(2)
    step('2 task_arm', all(armed[r].startswith('True') for r in ('robot_a', 'robot_b')),
         dict(arm=armed, status={r: status(r) for r in ('robot_a', 'robot_b')}))
    event = SiteEvent(event_id=str(uuid4()), camera_id=CAMERA, kind='DISASTER_SIGN', confidence=0.9,
                      visibility_state='VALID')
    event.time.observed_at_ns = node.get_clock().now().nanoseconds
    event.time.stamp_semantics, event.time.clock_domain_id, event.time.clock_state = 'CAPTURE', CLOCK_DOMAIN, 'SYNCED'
    event.time.has_clock_error_bound, event.time.clock_error_bound_ns = True, 5_000_000
    event.evidence.evidence_id, event.evidence.sha256 = str(uuid4()), 'a' * 64
    site.publish(event)
    spin(10, lambda: any(m.mode == 'DISASTER' for m in seen.get('state', [])))
    last = (seen.get('state') or [None])[-1]
    step('3 DISASTER', last is not None and last.mode == 'DISASTER',
         dict(mode=last.mode, phase=last.phase, reason=last.reason) if last else 'no SystemState')
    spin(30, lambda: any(a.accepted for a in seen.get('adm_robot_a', [])))
    admissions = [(a.accepted, a.reason_code) for a in seen.get('adm_robot_a', [])]
    step('4 TB-6 task admitted', any(ok for ok, _ in admissions), dict(admissions=admissions[-5:],
         state=(dict(mode=last.mode, phase=last.phase, reason=last.reason) if (last := (seen.get('state') or
                [None])[-1]) else None)))
    spin(20, lambda: (status('robot_a') or {}).get('task_state') == 'FINISHED')
    spin(2)
    final = status('robot_a')
    latched = (final is not None and final['task_state'] == 'FINISHED' and final['readiness'] == 'NOT_READY'
               and not final['stop_reason'].startswith('REARM_REFUSED_'))
    step('5 refused goal -> FAILED, still latched (no auto re-arm)', latched, final)
    print(json.dumps(steps, ensure_ascii=False, default=str), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all(s['status'] == 'PASSED' for s in steps) else 2


if __name__ == '__main__':
    sys.exit(main())
