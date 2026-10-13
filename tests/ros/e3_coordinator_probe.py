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
1. no goal before DISASTER_SIGN, nor after it until START_SEARCH (owner decision
2026-10-11, #85); START_SEARCH while the SCOUT is NOT_READY -> REJECTED/NAV_NOT_READY and
no goal (#98); 2. START_SEARCH (new command_id) + READY status -> SEARCH_AREA for the
first area, while the idle robot reports lease_valid=false (#100); 3. admission +
completed result -> SEARCH_AREA for the second area; 4. REQUEST_RECHECK of an unknown
target -> REJECTED/NOT_FOUND, replay -> DUPLICATE; 5. the node refuses to start without
parameters (exit 2); 6. a VALID TargetLocation -> RescueReport, PERSISTED receipt ->
SystemState TARGET_FOUND; 7. DISPATCH_SUPPORT -> one MOVE_SEGMENT to robot_b with route
(UNKNOWN assessment, no planner values) and a ONE_SEGMENT approval; its completed result
-> AWAITING_CONTINUE and no next segment without CONTINUE_SEGMENT; its
route.generated_at_ns is set, not after the goal; 8. CommandLease for the first
SEARCH_AREA: same lease_id and generation as the goal, renewal_sequence strictly rising,
none after its result; 9. a RobotStatus with a stop reason for the running MOVE_SEGMENT ->
no further renewals; 10. a REOBSERVE_TARGET the executor never admits: renewals stop after
admission_timeout_s and, once a STOPPED status arrives, it is issued again as a new task;
11. a TargetSnapshot names the target with its saved report.
Exit code 0 = all held.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_interfaces.msg import (CommandLease, ReportReceipt, RescueReport, RobotStatus, SiteEvent, SystemState,
                                   TargetLocation, TargetSnapshot, TaskAdmission)
from rescue_interfaces.srv import SubmitOperatorCommand

SCOUT, SUPPORT = 'robot_a', 'robot_b'
CLOCK_DOMAIN = 'e3-clock'


def fill_observation_time(t, now_ns):
    """A declared time basis on CLOCK_DOMAIN (a test value, not a measured clock)."""
    t.observed_at_ns, t.valid_until_ns = now_ns, now_ns + 10**10
    t.stamp_semantics, t.clock_domain_id, t.clock_state = 'CAPTURE', CLOCK_DOMAIN, 'SYNCED'
    t.has_clock_error_bound, t.clock_error_bound_ns = True, 5_000_000
MAP = '22222222-2222-4222-8222-222222222222'
AREAS = ('11111111-1111-4111-8111-111111111111', '11111111-1111-4111-8111-111111111112')
NODE_ARGS = ['-p', 'entry_yaws:=[0.0, 1.5]', '-p', 'policy_version:=e3-probe', '-p', 'executable_bases:=[PRIOR_MAP]',
             '-p', 'task_duration_s:=60.0', '-p', 'lease_duration_s:=10.0', '-p', 'executor_policy_version:=e3',
             '-p', 'mission_frame_id:=mission', '-p', 'authority_epoch:=00000000-0000-4000-8000-000000000004',
             '-p', 'coordinator_id:=coord-e3', '-p', 'envelope_schema_version:=e3', '-p', 'status_max_age_s:=5.0',
             '-p', 'tick_period_s:=0.2', '-p', 'route_segment_goals:=[1.0, 0.0, 0.0, 2.0, 0.0, 0.0]',
             '-p', 'route_revision:=1', '-p', 'observe_goal:=[3.0, 1.0, 0.0]',
             '-p', 'goal_map_session_id:=22222222-2222-4222-8222-222222222222', '-p', 'goal_map_revision:=0',
             '-p', 'goal_transform_revision:=1', '-p', 'merge_distance_m:=0.5', '-p', 'report_retry_s:=1.0',
             '-p', 'lease_renew_period_s:=0.2', '-p', 'admission_timeout_s:=1.0', '-p', 'lease_message_ttl_s:=1.0',
             '-p', 'route_segment_passage_free:=[true, true]', '-p', 'camera_watch_timeout_s:=2.0',
             # OPS-103: TargetLocation time gate (the probe's observations use this domain, SYNCED, 5 ms bound).
             '-p', f'target_clock_domain_id:={CLOCK_DOMAIN}', '-p', 'target_max_age_s:=5.0',
             '-p', 'target_max_future_s:=0.05',
             # TB-5 dispatch: OPERATOR here (this probe drives DISPATCH_SUPPORT); the rehearsal overrides it to AUTO.
             '-p', 'support_dispatch:=OPERATOR', '-p', 'approach_standoff_m:=0.8', '-p', 'support_pose_max_age_s:=2.0',
             '-p', 'support_exit_wait_s:=1.0', '-p', 'recheck_max_clock_error_s:=2.0',
             '-p', 'map_mode:=SAVED_MAP', '-p', 'freeze_clearance_m:=0.2', '-p', 'map_freeze_timeout_s:=5.0',
             '-p', 'search_mode:=AREAS', '-p', 'patrol_rounds:=1', '-p', 'exit_standoff_m:=0.5',
             '-p', 'frontier_min_cells:=3', '-p', 'frontier_reach_m:=1.0', '-p', 'frontier_goal_max_m:=1.5',
             '-p', 'frontier_blacklist_m:=0.5', '-p', 'frontier_issue_delay_s:=2.0',
             '-p', 'exploration_empty_checks:=3', '-p', 'exploration_timeout_s:=600.0',
             '-p', 'completion_mode:=RECHECK',  # the rehearsal's C stages run the recheck flow
             # OPERATOR: this probe and the rehearsal press START_SEARCH (OPS-136 AUTO is unit-tested)
             '-p', 'search_start:=OPERATOR']


def node_command(repo, extra):
    code = 'from rescue_coordination.ros_node import main; main()'
    return [sys.executable, '-c', code, '--ros-args'] + extra


class FakeExecutor(Node):
    """Fake executors of both robots plus the operations side (reports, receipts, operator commands)."""

    def __init__(self):
        super().__init__('e3_fake_executor')
        self.goals, self.support_goals, self.reports, self.states, self.leases = [], [], [], [], []
        self.snapshots = []
        self.admission = {r: self.create_publisher(TaskAdmission, f'/rescue/robots/{r}/task_admission', 10)
                          for r in (SCOUT, SUPPORT)}
        self.status = {r: self.create_publisher(RobotStatus, f'/rescue/robots/{r}/status', 1) for r in (SCOUT, SUPPORT)}
        self.site = self.create_publisher(SiteEvent, '/rescue/site_events', 10)
        self.location = self.create_publisher(TargetLocation, '/rescue/target_locations', 10)
        self.receipt = self.create_publisher(ReportReceipt, '/rescue/report_receipts', 10)
        self.command = self.create_client(SubmitOperatorCommand, '/rescue/operator_command')
        self.create_subscription(RescueReport, '/rescue/reports', self.reports.append, 10)
        self.create_subscription(CommandLease, f'/rescue/robots/{SCOUT}/command_lease', self.leases.append, 10)
        self.create_subscription(CommandLease, f'/rescue/robots/{SUPPORT}/command_lease', self.leases.append, 10)
        self.active, self.stop_reason = {SCOUT: '', SUPPORT: ''}, {SCOUT: '', SUPPORT: ''}
        self.readiness = {SCOUT: 'READY', SUPPORT: 'READY'}
        snapshot = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SystemState, '/rescue/system_state', self.states.append, snapshot)
        self.create_subscription(TargetSnapshot, '/rescue/targets', self.snapshots.append, snapshot)
        self.boot = {r: str(uuid4()) for r in (SCOUT, SUPPORT)}
        ActionServer(self, ExecuteTask, f'/rescue/robots/{SCOUT}/execute_task',
                     lambda handle: self.execute(handle, self.goals))
        ActionServer(self, ExecuteTask, f'/rescue/robots/{SUPPORT}/execute_task',
                     lambda handle: self.execute(handle, self.support_goals))

    def execute(self, handle, goals):
        goal = handle.request
        goals.append(goal)
        if goal.kind == 'REOBSERVE_TARGET' and sum(g.kind == 'REOBSERVE_TARGET' for g in goals) == 1:
            handle.abort()  # the first recheck is never admitted (check 10): no TaskAdmission, no outcome
            return ExecuteTask.Result()
        admission = TaskAdmission(command_id=goal.command_id, task_id=goal.task_id, robot_id=goal.robot_id,
                                  robot_boot_id=goal.expected_robot_boot_id, authority_epoch=goal.authority_epoch,
                                  generation=goal.generation, accepted=True, admission_id=str(uuid4()))
        self.admission[goal.robot_id].publish(admission)
        self.active[goal.robot_id] = goal.task_id
        # Let the admission and a few lease renewals land; the SUPPORT segment runs long enough for check 9.
        time.sleep(4. if goal.robot_id == SUPPORT else 1.)
        self.active[goal.robot_id] = ''
        handle.succeed()
        result = ExecuteTask.Result()
        result.result.task_id, result.result.outcome, result.result.stop_confirmed = goal.task_id, 'COMPLETED', True
        return result

    def publish_status(self):
        for robot, pub in self.status.items():
            # Like the executor (#100): a lease belongs to a task, so an idle robot reports lease_valid=false.
            pub.publish(RobotStatus(robot_id=robot, robot_boot_id=self.boot[robot], readiness=self.readiness[robot],
                                    lease_valid=bool(self.active[robot]),
                                    observed_at_ns=self.get_clock().now().nanoseconds,
                                    active_task_id=self.active[robot], stop_reason=self.stop_reason[robot],
                                    motion_state='MOVING' if self.active[robot] else 'STOPPED'))

    def publish_location(self):
        msg = TargetLocation(location_id=str(uuid4()), projection_status='VALID')
        f = msg.frame_ref
        f.robot_id, f.native_frame, f.mission_frame, f.map_session_id = SCOUT, 'map', 'mission', MAP
        f.map_revision, f.transform_revision, f.alignment_status = 0, 1, 'VALID'
        msg.mission_position.x, msg.mission_position.y = 2.5, 0.5
        fill_observation_time(msg.observation.time, self.get_clock().now().nanoseconds)
        self.location.publish(msg)
        return msg.location_id

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
    slow = subprocess.run(node_command(repo, ['-p', f'profile_path:={profile}'] + NODE_ARGS
                                       + ['-p', 'lease_renew_period_s:=6.0']), capture_output=True, text=True, timeout=30)
    checks['renewal period > lease/2 -> exit 2'] = slow.returncode == 2 and 'configuration rejected' in slow.stderr
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
        wait(executor, lambda: False, 2, probe.publish_status)
        checks['DISASTER_SIGN alone -> no goal'] = not probe.goals
        probe.readiness[SCOUT] = 'NOT_READY'
        wait(executor, lambda: False, 1, probe.publish_status)
        early = call(executor, probe, SubmitOperatorCommand.Request(
            command_id=str(uuid4()), operation='START_SEARCH', operator_id='e3',
            issued_at_ns=probe.get_clock().now().nanoseconds))
        wait(executor, lambda: False, 1, probe.publish_status)
        checks['START_SEARCH before ready -> REJECTED NAV_NOT_READY, no goal'] = (
            early.accepted, early.request_state, early.reason_code, len(probe.goals)) == (
            False, 'REJECTED', 'NAV_NOT_READY', 0)
        probe.readiness[SCOUT] = 'READY'
        wait(executor, lambda: False, 1, probe.publish_status)
        start = call(executor, probe, SubmitOperatorCommand.Request(
            command_id=str(uuid4()), operation='START_SEARCH', operator_id='e3',
            issued_at_ns=probe.get_clock().now().nanoseconds))
        got_first = wait(executor, lambda: len(probe.goals) >= 1, 10, probe.publish_status)
        checks['START_SEARCH -> SEARCH_AREA first area'] = got_first and (
            start.accepted, start.created_task_id, probe.goals[0].kind, probe.goals[0].robot_id,
            probe.goals[0].search_area_ref) == (True, probe.goals[0].task_id, 'SEARCH_AREA', SCOUT, AREAS[0])
        got_second = wait(executor, lambda: len(probe.goals) >= 2, 10, probe.publish_status)
        checks['completed + admission -> second area'] = got_second and probe.goals[1].search_area_ref == AREAS[1]
        wait(executor, lambda: False, 1.5)  # let the second goal finish before teardown
        first_goal = probe.goals[0] if probe.goals else None
        mine = [l for l in probe.leases if first_goal and l.lease_id == first_goal.lease_id]
        sequences = [l.renewal_sequence for l in mine]
        checks['CommandLease: same lease/generation, rising sequence'] = bool(first_goal) and len(mine) >= 2 and all(
            l.generation == first_goal.generation and l.robot_boot_id == first_goal.expected_robot_boot_id
            for l in mine) and sequences == sorted(set(sequences))
        count = len(mine)
        wait(executor, lambda: False, 1.0, probe.publish_status)
        checks['CommandLease: none after the task result'] = count == len(
            [l for l in probe.leases if first_goal and l.lease_id == first_goal.lease_id])
        req = SubmitOperatorCommand.Request(command_id=str(uuid4()), operation='REQUEST_RECHECK', operator_id='e3',
                                            target_id=str(uuid4()), issued_at_ns=probe.get_clock().now().nanoseconds)
        first = call(executor, probe, req)
        replay = call(executor, probe, req)  # same command_id, same content
        checks['unknown recheck -> REJECTED NOT_FOUND'] = (first.accepted, first.request_state, first.reason_code) == (
            False, 'REJECTED', 'NOT_FOUND')
        checks['replay -> DUPLICATE'] = (replay.request_state, replay.accepted) == ('DUPLICATE', False)
        location_id = probe.publish_location()
        got_report = wait(executor, lambda: probe.reports, 10, probe.publish_status)
        report = probe.reports[0] if got_report else None
        checks['TargetLocation -> RescueReport'] = bool(report) and list(report.record_ids) == [location_id]
        if report:
            probe.receipt.publish(ReportReceipt(report_id=report.report_id, included_record_ids=[location_id],
                                                state='PERSISTED', has_persisted_at=True,
                                                persisted_at_ns=probe.get_clock().now().nanoseconds))
        found = wait(executor, lambda: probe.states and probe.states[-1].phase == 'TARGET_FOUND', 10,
                     probe.publish_status)
        checks['PERSISTED -> SystemState TARGET_FOUND'] = found
        saved = wait(executor, lambda: probe.snapshots and probe.snapshots[-1].targets
                     and probe.snapshots[-1].targets[0].persisted_report_id, 10, probe.publish_status)
        checks['TargetSnapshot names the saved target'] = bool(saved and report) and (
            probe.snapshots[-1].targets[0].target_id, probe.snapshots[-1].targets[0].persisted_report_id,
            probe.snapshots[-1].targets[0].mission_position.x) == (report.target_id, report.report_id, 2.5)
        if report:
            dispatch = call(executor, probe, SubmitOperatorCommand.Request(
                command_id=str(uuid4()), operation='DISPATCH_SUPPORT', operator_id='e3', target_id=report.target_id,
                expected_target_revision=0, issued_at_ns=probe.get_clock().now().nanoseconds))
            got_segment = wait(executor, lambda: probe.support_goals, 10, probe.publish_status)
            g = probe.support_goals[0] if got_segment else None
            checks['DISPATCH -> one MOVE_SEGMENT with route and ONE_SEGMENT approval'] = bool(g) and (
                dispatch.accepted, dispatch.created_task_id, g.kind, g.robot_id, g.has_route, g.has_approval,
                g.approval.scope, g.route.assessment.status, g.route.planner_evidence_id, len(g.route.planned_path.poses),
                g.route.approach_goal.pose.position.x, g.route.frame_ref.alignment_status) == (
                True, g.task_id, 'MOVE_SEGMENT', SUPPORT, True, True, 'ONE_SEGMENT', 'UNKNOWN', '', 0, 1.0, 'VALID')
            checks['MOVE_SEGMENT route carries the time it was made'] = bool(g) and (
                0 < g.route.generated_at_ns <= g.issued_at_ns < g.route.valid_until_ns)
            if g:
                wait(executor, lambda: False, 1.0, probe.publish_status)
                renewed = [l for l in probe.leases if l.lease_id == g.lease_id]
                probe.stop_reason[SUPPORT] = 'OPERATOR_STOP'
                wait(executor, lambda: False, 0.5, probe.publish_status)
                before = len([l for l in probe.leases if l.lease_id == g.lease_id])
                wait(executor, lambda: False, 1.0, probe.publish_status)
                after = len([l for l in probe.leases if l.lease_id == g.lease_id])
                checks['stop_reason in RobotStatus -> no further renewals'] = len(renewed) >= 2 and before == after
                probe.stop_reason[SUPPORT] = ''
            waiting = wait(executor, lambda: probe.states and probe.states[-1].phase == 'AWAITING_CONTINUE', 10,
                           probe.publish_status)
            wait(executor, lambda: False, 2, probe.publish_status)
            checks['segment done -> AWAITING_CONTINUE, no next segment by itself'] = waiting and (
                len(probe.support_goals), probe.states[-1].segment_phase, probe.states[-1].segment_index) == (
                1, 'AWAITING_APPROVAL', 1)
            recheck = call(executor, probe, SubmitOperatorCommand.Request(
                command_id=str(uuid4()), operation='REQUEST_RECHECK', operator_id='e3', target_id=report.target_id,
                issued_at_ns=probe.get_clock().now().nanoseconds))
            rechecks = lambda: [g for g in probe.goals if g.kind == 'REOBSERVE_TARGET']
            got = wait(executor, lambda: rechecks(), 10, probe.publish_status)
            first_recheck = rechecks()[0] if got else None
            wait(executor, lambda: False, 2.0, probe.publish_status)  # past admission_timeout_s
            count = lambda: len([l for l in probe.leases if first_recheck and l.lease_id == first_recheck.lease_id])
            stopped_at = count()
            reissued = wait(executor, lambda: len(rechecks()) >= 2, 10, probe.publish_status)
            checks['unadmitted recheck: renewals stop, reissued after STOPPED status'] = bool(
                recheck.accepted and first_recheck) and reissued and stopped_at == count() and (
                rechecks()[1].task_id != first_recheck.task_id)
            wait(executor, lambda: False, 1.5, probe.publish_status)  # let the reissued recheck finish
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()
        child.terminate()
        output, _ = child.communicate(timeout=10)
        print(output)  # the node log, also when a check raised
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    raise SystemExit(0 if checks and all(checks.values()) else 1)


if __name__ == '__main__':
    main()
