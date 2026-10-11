"""E3 (partial): the mission_coordinator node against a fake executor on a local ROS graph.

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:$PYTHONPATH \
      python3 tests/ros/e3_coordinator_probe.py <REPO>   # not -I: ROS needs PYTHONPATH

The node runs as a child process with config/deployment.v2.example.json (robot_a SCOUT,
robot_b SUPPORT). This probe is the fake executor of robot_a: it accepts ExecuteTask,
publishes TaskAdmission and returns COMPLETED with a confirmed stop. Checks:
1. no goal before DISASTER_SIGN; 2. DISASTER_SIGN + READY status -> SEARCH_AREA for the
first area; 3. admission + completed result -> SEARCH_AREA for the second area;
4. REQUEST_RECHECK of an unknown target -> REJECTED/NOT_FOUND, replay -> DUPLICATE;
5. the node refuses to start without parameters (exit 2). Exit code 0 = all held.
"""
import os
import subprocess
import sys
import time
from uuid import uuid4

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rescue_interfaces.action import ExecuteTask
from rescue_interfaces.msg import RobotStatus, SiteEvent, TaskAdmission
from rescue_interfaces.srv import SubmitOperatorCommand

SCOUT = 'robot_a'
AREAS = ('11111111-1111-4111-8111-111111111111', '11111111-1111-4111-8111-111111111112')
NODE_ARGS = ['-p', 'entry_yaws:=[0.0, 1.5]', '-p', 'policy_version:=e3-probe', '-p', 'executable_bases:=[PRIOR_MAP]',
             '-p', 'task_duration_s:=60.0', '-p', 'lease_duration_s:=10.0', '-p', 'executor_policy_version:=e3',
             '-p', 'mission_frame_id:=mission', '-p', 'authority_epoch:=00000000-0000-4000-8000-000000000004',
             '-p', 'coordinator_id:=coord-e3', '-p', 'envelope_schema_version:=e3', '-p', 'status_max_age_s:=5.0',
             '-p', 'tick_period_s:=0.2']


def node_command(repo, extra):
    code = 'from rescue_coordination.ros_node import main; main()'
    return [sys.executable, '-c', code, '--ros-args'] + extra


class FakeExecutor(Node):
    def __init__(self):
        super().__init__('e3_fake_executor')
        self.goals = []
        self.admission = self.create_publisher(TaskAdmission, f'/rescue/robots/{SCOUT}/task_admission', 10)
        self.status = self.create_publisher(RobotStatus, f'/rescue/robots/{SCOUT}/status', 1)
        self.site = self.create_publisher(SiteEvent, '/rescue/site_events', 10)
        self.command = self.create_client(SubmitOperatorCommand, '/rescue/operator_command')
        self.boot = str(uuid4())
        ActionServer(self, ExecuteTask, f'/rescue/robots/{SCOUT}/execute_task', self.execute)

    def execute(self, handle):
        goal = handle.request
        self.goals.append(goal)
        admission = TaskAdmission(command_id=goal.command_id, task_id=goal.task_id, robot_id=goal.robot_id,
                                  robot_boot_id=goal.expected_robot_boot_id, authority_epoch=goal.authority_epoch,
                                  generation=goal.generation, accepted=True, admission_id=str(uuid4()))
        self.admission.publish(admission)
        time.sleep(.5)  # let the admission land before the result
        handle.succeed()
        result = ExecuteTask.Result()
        result.result.task_id, result.result.outcome, result.result.stop_confirmed = goal.task_id, 'COMPLETED', True
        return result

    def publish_status(self):
        self.status.publish(RobotStatus(robot_id=SCOUT, robot_boot_id=self.boot, readiness='READY', lease_valid=True,
                                        observed_at_ns=self.get_clock().now().nanoseconds))

    def publish_disaster(self):
        msg = SiteEvent(event_id=str(uuid4()), camera_id='cam_fixed', kind='DISASTER_SIGN', confidence=.9,
                        visibility_state='VALID')
        msg.time.observed_at_ns = self.get_clock().now().nanoseconds
        msg.time.stamp_semantics, msg.time.clock_domain_id, msg.time.clock_state = 'CAPTURE', 'e3-clock', 'SYNCED'
        msg.time.has_clock_error_bound, msg.time.clock_error_bound_ns = True, 5_000_000
        msg.evidence.evidence_id, msg.evidence.sha256 = str(uuid4()), 'a' * 64
        self.site.publish(msg)


def wait(executor, condition, timeout, tick=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tick:
            tick()
        executor.spin_once(timeout_sec=.1)
        if condition():
            return True
    return False


def call(executor, probe, req):
    future = probe.command.call_async(req)
    wait(executor, future.done, 5)
    return future.result()


def main():
    repo = sys.argv[1]
    for key in ('ROS_DISCOVERY_SERVER',):
        if os.environ.get(key):
            raise SystemExit(f'unset {key} first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    checks = {}
    refused = subprocess.run(node_command(repo, []), capture_output=True, text=True, timeout=30)
    checks['missing parameters -> exit 2'] = refused.returncode == 2 and 'missing parameters' in refused.stderr
    profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
    child = subprocess.Popen(node_command(repo, ['-p', f'profile_path:={profile}'] + NODE_ARGS),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    probe = FakeExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(probe)
    try:
        wait(executor, probe.command.service_is_ready, 20)
        wait(executor, lambda: False, 2, probe.publish_status)
        checks['no goal before DISASTER_SIGN'] = not probe.goals
        probe.publish_disaster()
        got_first = wait(executor, lambda: len(probe.goals) >= 1, 10, probe.publish_status)
        checks['DISASTER_SIGN -> SEARCH_AREA first area'] = got_first and (
            probe.goals[0].kind, probe.goals[0].robot_id, probe.goals[0].search_area_ref) == (
            'SEARCH_AREA', SCOUT, AREAS[0])
        got_second = wait(executor, lambda: len(probe.goals) >= 2, 10, probe.publish_status)
        checks['completed + admission -> second area'] = got_second and probe.goals[1].search_area_ref == AREAS[1]
        wait(executor, lambda: False, 1.5)  # let the second goal finish before teardown
        req = SubmitOperatorCommand.Request(command_id=str(uuid4()), operation='REQUEST_RECHECK', operator_id='e3',
                                            target_id=str(uuid4()), issued_at_ns=probe.get_clock().now().nanoseconds)
        first = call(executor, probe, req)
        replay = call(executor, probe, req)  # same command_id, same content
        checks['unknown recheck -> REJECTED NOT_FOUND'] = (first.accepted, first.request_state, first.reason_code) == (
            False, 'REJECTED', 'NOT_FOUND')
        checks['replay -> DUPLICATE'] = (replay.request_state, replay.accepted) == ('DUPLICATE', False)
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()
        child.terminate()
        output, _ = child.communicate(timeout=10)
    print(output)
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    raise SystemExit(0 if checks and all(checks.values()) else 1)


if __name__ == '__main__':
    main()
