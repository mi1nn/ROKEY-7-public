"""E3 (partial): B-stage gate with the real coordinator and the real report_node on a local ROS graph.

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:<REPO>/src/rescue_operations:$PYTHONPATH \
      python3 tests/ros/e3_report_gate_probe.py <REPO> <SCRATCH_DIR>   # not -I: ROS needs PYTHONPATH

Two child processes: ``mission_coordinator`` (config/deployment.v2.example.json, robot_a SCOUT,
robot_b SUPPORT) and ``rescue_report_node`` (report DB in <SCRATCH_DIR>). This probe plays the
perception side (TargetLocation), both robots' status and a fake robot_b executor. It never
publishes a ReportReceipt itself. Checks:
1. a VALID TargetLocation -> RescueReport -> report_node PERSISTED -> TargetSnapshot names that
report as persisted_report_id (revision 0);
2. a nearby location -> revision 1 whose report carries only the new location_id, and its own
PERSISTED receipt names the revision-1 report;
3. DISPATCH_SUPPORT with revision 0 -> REJECTED INVALID_INPUT (stale), with revision 1 ->
ACCEPTED, and robot_b receives one MOVE_SEGMENT;
4. 40 more nearby locations -> every receipt PERSISTED, the report DB covers every location_id
and its largest stored report stays below 8 KiB (one or two locations, not the target history).
Exit code 0 = all held.
"""
import json
import os
import sqlite3
import subprocess
import sys
import time
from uuid import uuid4

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_interfaces.action import ExecuteTask
from rescue_interfaces.msg import (ReportReceipt, RescueReport, RobotStatus, SiteEvent, TargetLocation, TargetSnapshot,
                                   TaskAdmission)
from rescue_interfaces.srv import SubmitOperatorCommand

from e3_coordinator_probe import MAP, NODE_ARGS, SCOUT, SUPPORT, call, node_command, wait

REPORT_ARGS = ['-p', 'reports_topic:=/rescue/reports', '-p', 'receipts_topic:=/rescue/report_receipts',
               '-p', 'envelope_schema_version:=e3', '-p', 'publisher_id:=ops-e3']
MAX_REPORT_JSON = 8192


class Field(Node):
    def __init__(self):
        super().__init__('e3_report_gate_probe')
        self.reports, self.receipts, self.snapshots, self.support_goals = [], [], [], []
        self.status = {r: self.create_publisher(RobotStatus, f'/rescue/robots/{r}/status', 1) for r in (SCOUT, SUPPORT)}
        self.admission = self.create_publisher(TaskAdmission, f'/rescue/robots/{SUPPORT}/task_admission', 10)
        self.site = self.create_publisher(SiteEvent, '/rescue/site_events', 10)
        self.location = self.create_publisher(TargetLocation, '/rescue/target_locations', 10)
        self.command = self.create_client(SubmitOperatorCommand, '/rescue/operator_command')
        self.create_subscription(RescueReport, '/rescue/reports', self.reports.append, 10)
        self.create_subscription(ReportReceipt, '/rescue/report_receipts', self.receipts.append, 10)
        snapshot = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TargetSnapshot, '/rescue/targets', self.snapshots.append, snapshot)
        self.boot = {r: str(uuid4()) for r in (SCOUT, SUPPORT)}
        ActionServer(self, ExecuteTask, f'/rescue/robots/{SUPPORT}/execute_task', self.execute)

    def execute(self, handle):
        goal = handle.request
        self.support_goals.append(goal)
        self.admission.publish(TaskAdmission(
            command_id=goal.command_id, task_id=goal.task_id, robot_id=goal.robot_id,
            robot_boot_id=goal.expected_robot_boot_id, authority_epoch=goal.authority_epoch,
            generation=goal.generation, accepted=True, admission_id=str(uuid4())))
        time.sleep(.5)
        handle.succeed()
        result = ExecuteTask.Result()
        result.result.task_id, result.result.outcome, result.result.stop_confirmed = goal.task_id, 'COMPLETED', True
        return result

    def publish_status(self):
        for robot, pub in self.status.items():
            pub.publish(RobotStatus(robot_id=robot, robot_boot_id=self.boot[robot], readiness='READY',
                                    lease_valid=False, observed_at_ns=self.get_clock().now().nanoseconds,
                                    active_task_id='', stop_reason='', motion_state='STOPPED'))

    def publish_location(self, x):
        msg = TargetLocation(location_id=str(uuid4()), projection_status='VALID')
        f = msg.frame_ref
        f.robot_id, f.native_frame, f.mission_frame, f.map_session_id = SCOUT, 'map', 'mission', MAP
        f.map_revision, f.transform_revision, f.alignment_status = 0, 1, 'VALID'
        msg.mission_position.x, msg.mission_position.y = x, 0.5
        now = self.get_clock().now().nanoseconds
        msg.observation.time.observed_at_ns, msg.observation.time.valid_until_ns = now, now + 10**10
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

    def latest(self):
        return self.snapshots[-1].targets[0] if self.snapshots and self.snapshots[-1].targets else None

    def dispatch(self, executor, target_id, revision):
        return call(executor, self, SubmitOperatorCommand.Request(
            command_id=str(uuid4()), operation='DISPATCH_SUPPORT', operator_id='e3', target_id=target_id,
            expected_target_revision=revision, issued_at_ns=self.get_clock().now().nanoseconds))


def saved_as(field, revision, report_id):
    t = field.latest()
    return t is not None and (t.estimate_revision, t.persisted_report_id) == (revision, report_id)


def main():
    repo, scratch = sys.argv[1], sys.argv[2]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    db_path = os.path.join(scratch, f'reports-{uuid4()}.sqlite3')
    profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
    logs = {name: open(os.path.join(scratch, f'{name}.log'), 'w') for name in ('coordinator', 'report_node')}
    children = [
        subprocess.Popen(node_command(repo, ['-p', f'profile_path:={profile}'] + NODE_ARGS),
                         stdout=logs['coordinator'], stderr=subprocess.STDOUT, text=True),
        subprocess.Popen([sys.executable, '-c', 'from rescue_operations.report_node import main; main()',
                          '--ros-args', '-p', f'db_path:={db_path}'] + REPORT_ARGS,
                         stdout=logs['report_node'], stderr=subprocess.STDOUT, text=True)]
    checks = {}
    rclpy.init()
    field = Field()
    executor = MultiThreadedExecutor()
    executor.add_node(field)
    try:
        wait(executor, field.command.service_is_ready, 20)
        wait(executor, lambda: False, 2, field.publish_status)
        field.publish_disaster()
        wait(executor, lambda: False, 1, field.publish_status)

        first_id = field.publish_location(2.5)
        wait(executor, lambda: field.reports, 10, field.publish_status)
        first = field.reports[0] if field.reports else None
        ok = first is not None and wait(executor, lambda: saved_as(field, 0, first.report_id), 10, field.publish_status)
        receipt = next((r for r in field.receipts if first and r.report_id == first.report_id), None)
        checks['1 location -> report_node PERSISTED -> persisted_report_id (rev 0)'] = ok and bool(receipt) and (
            receipt.state, list(receipt.included_record_ids), list(first.record_ids)) == (
            'PERSISTED', [first_id], [first_id])
        target_id = first.target_id if first else ''

        second_id = field.publish_location(2.7)
        wait(executor, lambda: any(r.report_id != first.report_id for r in field.reports), 10, field.publish_status)
        second = next((r for r in field.reports if first and r.report_id != first.report_id), None)
        ok = second is not None and wait(executor, lambda: saved_as(field, 1, second.report_id), 10,
                                         field.publish_status)
        checks['2 nearby location -> rev 1 report carries only the new location, PERSISTED'] = ok and (
            second.target_id, list(second.record_ids), [l.location_id for l in second.locations]) == (
            target_id, [second_id], [second_id])

        stale = field.dispatch(executor, target_id, 0)
        reply = field.dispatch(executor, target_id, 1)
        got_goal = wait(executor, lambda: field.support_goals, 10, field.publish_status)
        checks['3 DISPATCH_SUPPORT stale rev REJECTED, current rev ACCEPTED -> MOVE_SEGMENT'] = (
            (stale.accepted, stale.reason_code, reply.accepted, reply.request_state) == (
                False, 'INVALID_INPUT', True, 'ACCEPTED')
            and got_goal and (field.support_goals[0].kind, field.support_goals[0].robot_id,
                              field.support_goals[0].target_id, field.support_goals[0].target_estimate_revision) == (
                'MOVE_SEGMENT', SUPPORT, target_id, 1))

        more = []
        for n in range(40):
            more.append(field.publish_location(2.7 + .002 * (n + 1)))
            wait(executor, lambda: False, .3, field.publish_status)
        wait(executor, lambda: field.latest() is not None and field.latest().estimate_revision == 41
             and field.latest().persisted_report_id != '', 15, field.publish_status)
        with sqlite3.connect(db_path) as db:
            rows = db.execute('SELECT record_ids_json, length(payload_json) FROM rescue_reports').fetchall()
        stored = {rid for ids, _ in rows for rid in json.loads(ids)}
        largest = max((size for _, size in rows), default=0)
        rejected = [r for r in field.receipts if r.state != 'PERSISTED']
        checks['4 40 more locations: all PERSISTED, all stored, largest report < 8 KiB'] = (
            field.latest() is not None and field.latest().estimate_revision == 41 and not rejected
            and {first_id, second_id, *more} <= stored and largest < MAX_REPORT_JSON)
        publishers = sorted({r.envelope.publisher_id for r in field.receipts})
        checks['5 every receipt comes from this run\'s report_node'] = publishers == ['ops-e3']
        print(f'reports={len(rows)} largest_payload_json={largest} receipts={len(field.receipts)} '
              f'rejected={len(rejected)} receipt_publishers={publishers}')
    finally:
        for child in children:
            child.terminate()
        codes = [child.wait(timeout=10) for child in children]
        field.destroy_node()
        rclpy.try_shutdown()
        for log in logs.values():
            log.close()
    checks['children exit cleanly'] = all(code in (0, -15) for code in codes)
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    print(f'PASS {sum(checks.values())} FAIL {len(checks) - sum(checks.values())} exit={codes}')
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
