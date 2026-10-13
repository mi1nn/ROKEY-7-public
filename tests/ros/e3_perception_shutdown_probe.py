"""E3: perception nodes end cleanly on SIGTERM (OPS-129, 2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    source <scratch>/install/setup.bash   # colcon build --packages-select rescue_interfaces
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_mapping:<REPO>/src/rescue_perception:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:<REPO>/tests/ros:$PYTHONPATH \
      python3 tests/ros/e3_perception_shutdown_probe.py [runs]   # not -I: ROS needs PYTHONPATH

target_locator is started ``runs`` times (default 20) with the e3_target_locator_probe parameters
and sent SIGTERM at a varying moment after it started spinning, while MapState arrives at 100 Hz. Rehearsal #220 saw 1 of 3 runs end
with RCLError("the given context is not valid") and exit 1. Check: every run exits 0 with no
traceback. Exit code 0 = all held.
"""
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rescue_interfaces.msg import MapState

from e3_target_locator_probe import LOCATOR_ARGS, locator_command, window_args


def one_run(delay_s):
    process = subprocess.Popen(locator_command(LOCATOR_ARGS + window_args()), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    lines, started = [], time.monotonic()
    while time.monotonic() - started < 20:
        line = process.stdout.readline()
        if not line:
            break
        lines.append(line)
        if 'target_locator git=' in line:   # spinning from here on
            time.sleep(delay_s)
            process.send_signal(signal.SIGTERM)
            break
    output, _ = process.communicate(timeout=20)
    return process.returncode, ''.join(lines) + output


def main():
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    rclpy.init()
    node = rclpy.create_node('e3_shutdown_load')
    publisher = node.create_publisher(MapState, '/rescue/robots/robot_a/map_state', 10)
    loading = threading.Event()

    def load():   # callbacks keep arriving while the signal lands, as in the rehearsal
        while not loading.wait(0.01):
            publisher.publish(MapState(robot_id='robot_a', state='VALID', map_session_id='e3-load', map_revision=1))
    threading.Thread(target=load, daemon=True).start()
    failures = []
    for i in range(runs):
        code, output = one_run(0.2 + 0.037 * i)   # spread across the 0.1 s timer period
        if code != 0 or 'Traceback' in output:
            failures.append((i, code, output.strip().splitlines()[-1:] if output.strip() else []))
    loading.set()
    node.destroy_node()
    rclpy.try_shutdown()
    for failure in failures:
        print('  failed run', failure)
    ok = not failures
    print(('PASS ' if ok else 'FAIL ') + f'SIGTERM -> exit 0, no traceback ({runs - len(failures)}/{runs})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
