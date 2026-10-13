"""E3 (partial): coordinator SetCameraWatch client against the webcam's own watch slot on a local ROS graph.

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:<REPO>/src/rescue_perception:<REPO>/tests/ros:$PYTHONPATH \
      python3 tests/ros/e3_camera_watch_probe.py <REPO>   # not -I: ROS needs PYTHONPATH

The coordinator runs as a child process (config/deployment.v2.example.json: cam_fixed has
TARGET_WATCH; no deployment ROI). This probe serves ``/rescue/cameras/cam_fixed/set_watch`` with
``rescue_perception.webcam_service.WatchSlot`` (the webcam node's acceptance rules, no camera or
model) and plays perception and the operator. Checks:
1. SET_CAMERA_WATCH before DISASTER -> REJECTED NOT_AUTHORIZED, no camera request (#124);
2. in DISASTER, without a ROI anywhere -> REJECTED PENDING_EVIDENCE, no camera request (no full-frame default);
3. with the operator's ROI -> ACCEPTED, one enable request (OPERATOR_ROI, evidence = command_id, a new
   watch_id) that the slot accepts at revision 1;
4. a TARGET_DISAPPEARED_CANDIDATE for that watch -> the coordinator releases it (enable=false,
   expected revision 1) and the slot is empty at revision 2.
Exit code 0 = all held.
"""
import os
import subprocess
import sys
import time
from uuid import uuid4

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rescue_interfaces.msg import SiteEvent, TargetLocation
from rescue_interfaces.srv import SetCameraWatch, SubmitOperatorCommand
from rescue_perception import SourceBinding
from rescue_perception.target_watch import TargetWatchPolicy
from rescue_perception.webcam_service import WatchRequest, WatchSlot

from e3_coordinator_probe import MAP, NODE_ARGS, SCOUT, call, fill_observation_time, node_command, wait

ROI = [0.2, 0.3, 0.6, 0.9]


class Field(Node):
    def __init__(self):
        super().__init__('e3_camera_watch_probe')
        binding = SourceBinding(None, 'cam_fixed', str(uuid4()), 'e3-clock', 'DRIVER_RECEIVE', 'UNKNOWN',
                                'cam-fixed-cal', 'UNKNOWN', None)
        policy = TargetWatchPolicy(score_min=.5, roi_overlap_min=.5, present_confirm_frames=2, absent_hold_ns=300,
                                   absent_min_frames=3, max_gap_ns=150, exit_roi_xyxy_normalized=(0., 0., 1., 1.),
                                   track_max_jump=.3)
        self.slot, self.requests, self.replies = WatchSlot(binding, policy), [], []
        self.create_service(SetCameraWatch, '/rescue/cameras/cam_fixed/set_watch', self.on_set_watch)
        self.site = self.create_publisher(SiteEvent, '/rescue/site_events', 10)
        self.location = self.create_publisher(TargetLocation, '/rescue/target_locations', 10)
        self.command = self.create_client(SubmitOperatorCommand, '/rescue/operator_command')

    def on_set_watch(self, request, response):
        self.requests.append(request)
        reply = self.slot.handle(WatchRequest(
            command_id=request.command_id, camera_id=request.camera_id, watch_id=request.watch_id,
            target_id=request.target_id, expected_watch_revision=int(request.expected_watch_revision),
            roi_xyxy_normalized=tuple(float(x) for x in request.roi_xyxy_normalized),
            association_method=request.association_method, association_evidence_id=request.association_evidence_id,
            enable=bool(request.enable), issued_at_ns=int(request.issued_at_ns), deadline_ns=int(request.deadline_ns)),
            now_ns=time.time_ns())
        self.replies.append(reply)
        response.accepted, response.watch_id = reply.accepted, reply.watch_id
        response.watch_revision, response.reason_code = reply.watch_revision, reply.reason_code
        return response

    def event(self, kind, **fields):
        msg = SiteEvent(event_id=str(uuid4()), camera_id='cam_fixed', kind=kind, confidence=1.,
                        visibility_state='VALID', **fields)
        msg.time.observed_at_ns = self.get_clock().now().nanoseconds
        msg.time.stamp_semantics, msg.time.clock_domain_id, msg.time.clock_state = 'CAPTURE', 'e3-clock', 'SYNCED'
        msg.time.has_clock_error_bound, msg.time.clock_error_bound_ns = True, 5_000_000
        msg.evidence.evidence_id, msg.evidence.sha256 = str(uuid4()), 'a' * 64
        self.site.publish(msg)

    def publish_location(self):
        msg = TargetLocation(location_id=str(uuid4()), projection_status='VALID')
        f = msg.frame_ref
        f.robot_id, f.native_frame, f.mission_frame, f.map_session_id = SCOUT, 'map', 'mission', MAP
        f.map_revision, f.transform_revision, f.alignment_status = 0, 1, 'VALID'
        msg.mission_position.x, msg.mission_position.y = 2.5, 0.5
        fill_observation_time(msg.observation.time, self.get_clock().now().nanoseconds)  # OPS-103 gate
        self.location.publish(msg)

    def watch(self, executor, target_id, roi):
        return call(executor, self, SubmitOperatorCommand.Request(
            command_id=str(uuid4()), operation='SET_CAMERA_WATCH', operator_id='e3', target_id=target_id,
            roi_xyxy_normalized=roi, issued_at_ns=self.get_clock().now().nanoseconds))


def target_of(log_path):
    with open(log_path) as log:
        for line in log:
            if 'TARGET_LOCATION' in line and '-> ' in line:
                return line.rsplit('-> ', 1)[1].strip()
    return None


def main():
    repo = sys.argv[1]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
    log_path = os.path.join(os.environ.get('E3_LOG_DIR', '.'), f'e3-camera-watch-{uuid4()}.log')
    log = open(log_path, 'w')
    child = subprocess.Popen(node_command(repo, ['-p', f'profile_path:={profile}'] + NODE_ARGS),
                             stdout=log, stderr=subprocess.STDOUT, text=True)
    checks = {}
    rclpy.init()
    field = Field()
    executor = MultiThreadedExecutor()
    executor.add_node(field)
    try:
        wait(executor, field.command.service_is_ready, 20)
        wait(executor, lambda: False, 1)
        early = field.watch(executor, str(uuid4()), ROI)
        wait(executor, lambda: False, .5)
        checks['1 before DISASTER -> REJECTED NOT_AUTHORIZED, no camera request'] = (
            early.accepted, early.reason_code, len(field.requests)) == (False, 'NOT_AUTHORIZED', 0)
        field.event('DISASTER_SIGN')
        field.publish_location()
        wait(executor, lambda: target_of(log_path), 10)
        target_id = target_of(log_path) or ''
        no_roi = field.watch(executor, target_id, [0.] * 4)
        wait(executor, lambda: False, .5)
        checks['2 no ROI anywhere -> REJECTED PENDING_EVIDENCE, no camera request'] = bool(target_id) and (
            no_roi.accepted, no_roi.reason_code, len(field.requests)) == (False, 'PENDING_EVIDENCE', 0)
        command = SubmitOperatorCommand.Request(
            command_id=str(uuid4()), operation='SET_CAMERA_WATCH', operator_id='e3', target_id=target_id,
            roi_xyxy_normalized=ROI, issued_at_ns=field.get_clock().now().nanoseconds)
        armed = call(executor, field, command)
        wait(executor, lambda: field.replies, 5)
        req = field.requests[0] if field.requests else None
        checks['3 operator ROI -> ACCEPTED, enable request accepted by the slot at revision 1'] = bool(req) and (
            armed.accepted, req.enable, req.target_id, [round(x, 6) for x in req.roi_xyxy_normalized],
            req.association_method, req.association_evidence_id, req.expected_watch_revision,
            field.replies[0].accepted, field.slot.revision) == (
            True, True, target_id, ROI, 'OPERATOR_ROI', command.command_id, 0, True, 1) and (
            req.watch_id not in (target_id, command.command_id))
        if req:
            field.event('TARGET_DISAPPEARED_CANDIDATE', target_id=target_id, watch_id=req.watch_id)
        wait(executor, lambda: len(field.replies) >= 2, 5)
        release = field.requests[1] if len(field.requests) >= 2 else None
        checks['4 candidate -> release (enable=false, rev 1), slot empty at rev 2'] = bool(release and req) and (
            release.enable, release.watch_id, release.expected_watch_revision, field.replies[1].accepted,
            field.slot.watch, field.slot.revision) == (False, req.watch_id, 1, True, None, 2)
    finally:
        child.terminate()
        code = child.wait(timeout=10)
        field.destroy_node()
        rclpy.try_shutdown()
        log.close()
    with open(log_path) as text:
        candidate_lines = [line.strip() for line in text if 'TARGET_DISAPPEARED_CANDIDATE' in line]
    print('\n'.join(candidate_lines[-1:]))
    checks['coordinator exits cleanly'] = code in (0, -15)
    for name, ok in checks.items():
        print(('PASS ' if ok else 'FAIL ') + name)
    print(f'PASS {sum(checks.values())} FAIL {len(checks) - sum(checks.values())} exit={code} log={log_path}')
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
