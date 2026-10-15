#!/usr/bin/env python3
"""One-PC rehearsal of the run without the web console (웹 없는 트랙 B-5, 2026-10-15), on the fake robots.

Run by run.sh when REHEARSAL_PROBE=noweb (slam_phase SEARCHING). The operator's two terminal tools of
scripts/field/noweb do what the console buttons and cards do:
1. both executors report a fresh status       5. the fake bt_navigator refuses the goal: FAILED, latched
2. refresh_state, task_arm on both             6. the fake robots dock (dock_status), every status STOPPED
3. operator_command.py DECLARE_DISASTER: 0     7. operator_command.py RESET_RUN: 0
   and a status_watch.py line 「임무 DISASTER」  8. a status_watch.py line 「초기화 확인 …」 after the reset
4. a TB-6 task is admitted                        (the restarts are not done here, so it says 아직)
status_watch.py runs the whole time and writes <out>/status.log. Synthetic data from feed.py (TB-6 TF flat).
No robot is on this graph; the fake Nav2 never accepts a goal, so nothing would move even if one were.

    noweb_probe.py <out dir> <api port>
"""
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rescue_interfaces.msg import RobotStatus, SystemState, TaskAdmission
from std_srvs.srv import Trigger

from feed import LATCHED, Feeder

REPO = Path(__file__).resolve().parents[3]
NOWEB = REPO / 'scripts' / 'field' / 'noweb'
OPERATOR = 'rehearsal-operator'


def main():
    out, port = Path(sys.argv[1]), sys.argv[2]
    db, log = out / 'site_state.sqlite3', out / 'status.log'
    watch = subprocess.Popen([sys.executable, str(NOWEB / 'status_watch.py'), '--db', str(db), '--log', str(log)],
                             stdout=subprocess.DEVNULL, stderr=open(out / 'status_watch.err', 'w'))
    rclpy.init()
    node = rclpy.create_node('rehearsal_noweb_probe')
    feeder = Feeder(node, tf='flat')
    seen = {}

    def keep(key):
        return lambda msg: seen.setdefault(key, []).append(msg)
    for robot in ('robot_a', 'robot_b'):
        node.create_subscription(RobotStatus, f'/rescue/robots/{robot}/status', keep('status_' + robot), 10)
        node.create_subscription(TaskAdmission, f'/rescue/robots/{robot}/task_admission', keep('adm_' + robot), 10)
    node.create_subscription(SystemState, '/rescue/system_state', keep('state'), LATCHED)
    steps = []

    def spin(seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(node, timeout_sec=0.05)

    def step(name, ok, detail):
        steps.append(dict(step=name, status='PASSED' if ok else 'FAILED', detail=detail))
        print(f'{"PASSED" if ok else "FAILED":8} {name}: {detail}', flush=True)

    def status(robot):
        m = (seen.get('status_' + robot) or [None])[-1]
        return None if m is None else dict(readiness=m.readiness, task_state=m.task_state, motion=m.motion_state,
                                           dock=m.dock_state, stop_reason=m.stop_reason, active=m.active_task_id)

    def operator(*args):
        """operator_command.py while this node keeps feeding (the tool waits up to its own timeout)."""
        proc = subprocess.Popen([sys.executable, str(NOWEB / 'operator_command.py'), '--port', port,
                                 '--operator', OPERATOR, *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True)
        spin(20, lambda: proc.poll() is not None)
        text = proc.communicate(timeout=5)[0].strip() if proc.poll() is not None else 'still running'
        return proc.returncode, text

    def log_line(text, seconds):
        def found():
            return log.exists() and any(text in line for line in log.read_text(encoding='utf-8').splitlines())
        spin(seconds, found)
        lines = [line for line in (log.read_text(encoding='utf-8').splitlines() if log.exists() else [])
                 if text in line]
        return lines[-1] if lines else None

    try:
        spin(20, lambda: all(seen.get('status_' + r) for r in ('robot_a', 'robot_b')))
        step('1 executor status', all(seen.get('status_' + r) for r in ('robot_a', 'robot_b')),
             {r: status(r) for r in ('robot_a', 'robot_b')})
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
        step('2 task_arm', all(armed[r].startswith('True') for r in ('robot_a', 'robot_b')), armed)
        code, text = operator('DECLARE_DISASTER')
        line = log_line('임무 DISASTER', 15)
        step('3 operator_command DECLARE_DISASTER -> status line 임무 DISASTER', code == 0 and line is not None,
             dict(exit=code, answer=text, line=line))
        spin(30, lambda: any(a.accepted for a in seen.get('adm_robot_a', [])))
        admissions = [(a.accepted, a.reason_code) for a in seen.get('adm_robot_a', [])]
        step('4 TB-6 task admitted', any(ok for ok, _ in admissions), dict(admissions=admissions[-5:]))
        spin(20, lambda: (status('robot_a') or {}).get('task_state') == 'FINISHED')
        spin(2)
        final = status('robot_a')
        step('5 refused goal -> FAILED, latched', final is not None and final['task_state'] == 'FINISHED'
             and final['readiness'] == 'NOT_READY', final)
        feeder.docked = True
        spin(15, lambda: all((status(r) or {}).get('dock') == 'DOCKED' for r in ('robot_a', 'robot_b')))
        spin(2)
        docked = {r: status(r) for r in ('robot_a', 'robot_b')}
        step('6 fake robots docked and stopped', all(s and s['dock'] == 'DOCKED' and s['motion'] == 'STOPPED'
                                                     and not s['active'] for s in docked.values()), docked)
        code, text = operator('RESET_RUN', '--note', 'one-PC noweb rehearsal')
        step('7 operator_command RESET_RUN', code == 0, dict(exit=code, answer=text))
        line = log_line('초기화 확인', 15)
        step('8 status line 초기화 확인 after the reset', line is not None, line)
    finally:
        print(json.dumps(steps, ensure_ascii=False, default=str), flush=True)
        watch.terminate()
        watch.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if len(steps) == 8 and all(s['status'] == 'PASSED' for s in steps) else 2


if __name__ == '__main__':
    sys.exit(main())
