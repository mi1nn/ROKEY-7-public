"""E3 rehearsal: stages A, B and C end to end on a local ROS graph, no robot (2026-10-13).

Not collected by scripts/run_tests.py (needs ROS 2 Jazzy and a built rescue_interfaces).
Run on an isolated graph, never a robot's domain, with no Discovery Server:

    colcon build --packages-select rescue_interfaces   # in a scratch workspace
    source <scratch>/install/setup.bash
    unset ROS_DISCOVERY_SERVER
    ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      PYTHONPATH=<REPO>/src/rescue_contracts:<REPO>/src/rescue_coordination:<REPO>/src/rescue_bringup:\
<REPO>/src/rescue_operations:<REPO>/src/rescue_perception:<REPO>/src/rescue_mapping:<REPO>/src/rescue_console:\
<REPO>/tests/ros:$PYTHONPATH \
      python3 tests/ros/e3_rehearsal_abc.py <REPO> <SCRATCH_DIR>   # not -I: ROS needs PYTHONPATH

Real nodes as child processes on config/deployment.v2.example.json (TB-6 = robot_a SCOUT, TB-5 = robot_b
SUPPORT): mission_coordinator with ``support_dispatch=AUTO`` (owner decision 2026-10-13), target_locator
(robot_a), db_writer with the operator command API, report_node. The console view is built in this process
from the same site-state DB. Played by this probe (no code yet or hardware): the webcam's DISASTER_SIGN and
its exit-side TARGET_DISAPPEARED_CANDIDATE and its SetCameraWatch slot (rescue_perception WatchSlot), both
robots' executors and status, TB-5's RobotPose, TB-6's camera TargetObservation and TF, and TB-6's
RecheckObservation. Clock values are test values (SYNCED, 5 ms bounds), not measured clocks. The probe never
publishes a ReportReceipt, a TargetLocation, a TargetSnapshot or a SystemState.

A1 DISASTER_SIGN -> coordinator DISASTER -> DB mission_state
A2 console START_SEARCH enabled
A3 HTTP START_SEARCH -> TB-6 SEARCH_AREA
B1 TargetObservation + TF -> target_locator VALID -> coordinator target
B2 report_node PERSISTED -> DB persisted_report_id
B3 TB-5 gets APPROACH_TARGET by itself (AUTO); HTTP DISPATCH_SUPPORT is refused (NOT_AUTHORIZED)
C1 TB-5 arrives -> after support_exit_wait_s, RETURN_HOME home/robot_b (exit)
C2 HTTP SET_CAMERA_WATCH -> the camera slot accepts the watch (ROI from camera_watch_roi)
C3 candidate -> TB-6 REOBSERVE refused once -> TASK_REFUSED -> after the hold, a new REOBSERVE accepted
C4 RecheckObservation ABSENT/CLEAR/VALID -> DB target RESCUE_CONFIRMED
C5 TB-6 RETURN_HOME home/robot_a (docking approach) -> both home -> DB mission FINISHED

Then the same nodes again with fresh DBs and ``map_mode=SLAM_FREEZE`` (owner decision 2026-10-13, scope Q-29:
no reference map, TB-6 maps while it searches, the map is frozen before TB-5 goes). Played here: the freeze
service (/rescue/map/freeze), the frozen grid (/rescue/map/frozen) and both robots' MapState.
F1 target on the search revision R0 -> PERSISTED -> no freeze yet (MAP_FREEZE_WAITING_EXIT) -> the QR exit
   (/rescue/exit_location, exit_qr) -> one freeze call (TB-6 stopped, no task)
F2 frozen grid + TB-6 MapState R0+1 with the grid's snapshot id -> frozen; the R0 target and exit are re-based
   (TARGET_REBASED_ON_FREEZE until the re-based revision is saved, OPS-138)
F3 the target seen again on R0+1 -> PERSISTED -> TB-5 localized on R0+1 -> APPROACH_TARGET on R0+1
Each stage prints PASS/FAIL and its seconds. The fake executors are fakes: the real executor's acceptance
of APPROACH_TARGET/REOBSERVE_TARGET/RETURN_HOME is the robot lane's (C1, #166·#179). Exit 0 = all held.
"""
import json
import os
import subprocess
import sys
import time
import urllib.request
from uuid import uuid4

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rescue_interfaces.action import ExecuteTask
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_contracts.occupancy import map_snapshot_id, occupancy_sha256
from rescue_interfaces.msg import (MapState, ReportReceipt, RobotPose, SiteEvent, SystemState, TargetLocation,
                                   TaskAdmission)
from rescue_interfaces.srv import SetCameraWatch
from std_srvs.srv import Trigger

from e3_coordinator_probe import CLOCK_DOMAIN, MAP, SCOUT, SUPPORT, FakeExecutor, node_command
from e3_target_locator_probe import LOCATOR_ARGS, Probe, coordinator_args, locator_command, window_args

API_PORTS = {'SAVED_MAP': 18947, 'SLAM_FREEZE': 18948}
COMMAND_TIMEOUT_S = 3.0
ROI = [0.1, 0.2, 0.6, 0.9]
EXIT_WAIT_S = 1.0
MS = 1_000_000


def params_file(path, parameters):
    with open(path, 'w') as out:
        json.dump({'/**': {'ros__parameters': parameters}}, out)
    return path


def node(module, params, log):
    code = f'from rescue_operations.{module} import main; main()'
    return subprocess.Popen([sys.executable, '-c', code, '--ros-args', '--params-file', params],
                            stdout=log, stderr=subprocess.STDOUT, text=True)


def post(kind, port=API_PORTS['SAVED_MAP'], **fields):
    body = json.dumps(dict(command_id=str(uuid4()), kind=kind, operator_id='e3-rehearsal', **fields)).encode()
    req = urllib.request.Request(f'http://127.0.0.1:{port}/api/operator-commands', data=body,
                                 headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=COMMAND_TIMEOUT_S + 2) as res:
            return res.status, json.loads(res.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b'{}')


def declared_time(t, now_ns):
    t.observed_at_ns, t.valid_until_ns = now_ns, now_ns + 10**10
    t.stamp_semantics, t.clock_domain_id, t.clock_state = 'CAPTURE', CLOCK_DOMAIN, 'SYNCED'
    t.has_clock_error_bound, t.clock_error_bound_ns = True, 5 * MS


class Robots(FakeExecutor):
    """FakeExecutor plus TB-5's RobotPose, a refused first REOBSERVE and TB-6's RecheckObservation."""

    def __init__(self):
        super().__init__()
        self.pose_pub = self.create_publisher(RobotPose, f'/rescue/robots/{SUPPORT}/pose', 1)
        self.refusals = []
        self.reobserve_count = 0
        self.pose_revision = 0  # TB-5's map revision: R0+1 once it localizes on the frozen map
        self.create_timer(0.2, self.publish_pose)

    def publish_pose(self):
        msg = RobotPose(robot_id=SUPPORT, source_boot_id=self.boot[SUPPORT], mission_pose_status='VALID')
        f = msg.frame_ref
        f.robot_id, f.native_frame, f.mission_frame, f.map_session_id = SUPPORT, 'map', 'map', MAP
        f.map_revision, f.transform_revision, f.alignment_status = self.pose_revision, 1, 'VALID'
        declared_time(msg.time, self.get_clock().now().nanoseconds)
        msg.mission_pose.pose.pose.position.x, msg.mission_pose.pose.pose.position.y = -2.0, 3.0
        self.pose_pub.publish(msg)

    def admit(self, goal, accepted=True, reason=''):
        self.admission[goal.robot_id].publish(TaskAdmission(
            command_id=goal.command_id, task_id=goal.task_id, robot_id=goal.robot_id,
            robot_boot_id=goal.expected_robot_boot_id, authority_epoch=goal.authority_epoch,
            generation=goal.generation, accepted=accepted, reason_code=reason, admission_id=str(uuid4())))

    def execute(self, handle, goals):
        goal = handle.request
        goals.append(goal)
        if goal.kind == 'REOBSERVE_TARGET':
            self.reobserve_count += 1
            if self.reobserve_count == 1:
                # #180 / 작전 decision: a robot that cannot find the revision yet refuses; the coordinator re-issues.
                self.admit(goal, accepted=False, reason='STALE_OBSERVATION')
                self.refusals.append(goal.task_id)
                handle.abort()
                return ExecuteTask.Result()
        self.admit(goal)
        self.active[goal.robot_id] = goal.task_id
        time.sleep(1.)
        self.active[goal.robot_id] = ''
        handle.succeed()
        result = ExecuteTask.Result()
        r = result.result
        r.task_id, r.outcome, r.stop_confirmed = goal.task_id, 'COMPLETED', True
        if goal.kind == 'REOBSERVE_TARGET':
            r.has_recheck = True
            o = r.recheck
            o.observation_id, o.task_id, o.target_id, o.robot_id = str(uuid4()), goal.task_id, goal.target_id, SCOUT
            declared_time(o.time, self.get_clock().now().nanoseconds)
            o.frame_ref.robot_id, o.frame_ref.mission_frame, o.frame_ref.map_session_id = SCOUT, 'map', MAP
            o.frame_ref.map_revision, o.frame_ref.transform_revision, o.frame_ref.alignment_status = 0, 1, 'VALID'
            o.camera_state, o.lidar_state, o.coverage_state = 'ABSENT', 'CLEAR', 'VALID'
            o.sensor_window_start_ns = self.get_clock().now().nanoseconds - 200 * MS  # after the candidate
            o.sensor_window_end_ns = self.get_clock().now().nanoseconds
        return result


class Camera:
    """The webcam's SetCameraWatch slot (its real acceptance rules) and its candidate."""

    def __init__(self, node):
        from rescue_perception import SourceBinding
        from rescue_perception.target_watch import TargetWatchPolicy
        from rescue_perception.webcam_service import WatchSlot
        binding = SourceBinding(None, 'cam_fixed', str(uuid4()), CLOCK_DOMAIN, 'DRIVER_RECEIVE', 'UNKNOWN',
                                'cam-fixed-cal', 'UNKNOWN', None)
        policy = TargetWatchPolicy(score_min=.5, roi_overlap_min=.5, present_confirm_frames=2, absent_hold_ns=300,
                                   absent_min_frames=3, max_gap_ns=150, exit_roi_xyxy_normalized=(0., 0., 1., 1.),
                                   track_max_jump=.3)
        self.node, self.slot, self.requests = node, WatchSlot(binding, policy), []
        node.create_service(SetCameraWatch, '/rescue/cameras/cam_fixed/set_watch', self.on_set_watch)
        self.site = node.create_publisher(SiteEvent, '/rescue/site_events', 10)

    def on_set_watch(self, request, response):
        from rescue_perception.webcam_service import WatchRequest
        self.requests.append(request)
        reply = self.slot.handle(WatchRequest(
            command_id=request.command_id, camera_id=request.camera_id, watch_id=request.watch_id,
            target_id=request.target_id, expected_watch_revision=int(request.expected_watch_revision),
            roi_xyxy_normalized=tuple(float(x) for x in request.roi_xyxy_normalized),
            association_method=request.association_method, association_evidence_id=request.association_evidence_id,
            enable=bool(request.enable), issued_at_ns=int(request.issued_at_ns), deadline_ns=int(request.deadline_ns)),
            now_ns=time.time_ns())
        response.accepted, response.watch_id = reply.accepted, reply.watch_id
        response.watch_revision, response.reason_code = reply.watch_revision, reply.reason_code
        return response

    def candidate(self, target_id, watch_id):
        msg = SiteEvent(event_id=str(uuid4()), camera_id='cam_fixed', kind='TARGET_DISAPPEARED_CANDIDATE',
                        confidence=1., visibility_state='VALID', target_id=target_id, watch_id=watch_id)
        declared_time(msg.time, self.node.get_clock().now().nanoseconds - 1000 * MS)
        msg.evidence.evidence_id, msg.evidence.sha256 = str(uuid4()), 'a' * 64
        self.site.publish(msg)


def coordinator_auto_args(profile, map_mode):
    args = coordinator_args(profile)
    args[args.index('support_dispatch:=OPERATOR')] = 'support_dispatch:=AUTO'
    args[args.index(f'support_exit_wait_s:=1.0')] = f'support_exit_wait_s:={EXIT_WAIT_S}'
    args[args.index('map_mode:=SAVED_MAP')] = f'map_mode:={map_mode}'
    return args + ['-p', f'camera_watch_roi:={ROI}']


class FrozenMap:
    """The map lifecycle's side (통합-로봇): freeze service, frozen grid, MapState of both robots (test values)."""

    # 20 m x 20 m at 0.5 m, all free, around the mission origin. OccupancyGrid.resolution is float32: the hash
    # is of the received value, so the fake uses one float32 holds exactly (0.1 would hash as 0.10000000149).
    WIDTH = HEIGHT = 40
    RESOLUTION = .5

    def __init__(self, node):
        self.node, self.calls = node, []
        node.create_service(Trigger, '/rescue/map/freeze', self.on_freeze)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = node.create_publisher(OccupancyGrid, '/rescue/map/frozen', latched)
        self.exit_pub = node.create_publisher(TargetLocation, '/rescue/exit_location', 10)
        self.support_state = node.create_publisher(MapState, f'/rescue/robots/{SUPPORT}/map_state', latched)
        self.grid = dict(frame='map', width=self.WIDTH, height=self.HEIGHT, resolution=self.RESOLUTION, origin=(-10., -10., 0.),
                         data=[0] * (self.WIDTH * self.HEIGHT))
        self.sha256 = occupancy_sha256(**self.grid)

    def on_freeze(self, _request, response):
        self.calls.append(time.monotonic())
        response.success, response.message = True, 'frozen (rehearsal fake)'
        return response

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.width, msg.info.height, msg.info.resolution = self.WIDTH, self.HEIGHT, self.RESOLUTION
        msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.orientation.w = -10., -10., 1.
        msg.data = self.grid['data']
        self.grid_pub.publish(msg)

    def publish_exit(self, xy=(-5.0, 0.0)):
        """The QR exit seen by TB-6 on the search revision R0 (통합-인식 안 「나」; test values)."""
        msg = TargetLocation(location_id=str(uuid4()), projection_status='VALID')
        f = msg.frame_ref
        f.robot_id, f.native_frame, f.mission_frame, f.map_session_id = SCOUT, 'map', 'map', MAP
        f.map_revision, f.transform_revision, f.alignment_status = 0, 1, 'VALID'
        msg.mission_position.x, msg.mission_position.y = xy
        msg.observation.class_label = 'exit_qr'
        declared_time(msg.observation.time, self.node.get_clock().now().nanoseconds)
        self.exit_pub.publish(msg)

    def state(self, robot, revision, snapshot):
        return MapState(robot_id=robot, map_session_id=MAP, map_revision=revision, snapshot_id=snapshot,
                        generated_at_ns=self.node.get_clock().now().nanoseconds, localization_quality='VALID',
                        correction_detection_status='UNKNOWN', current_native_map_frame='map')

    def frozen_snapshot(self, revision):
        return map_snapshot_id(self.sha256, MAP, revision, 1)


def run(repo, scratch, map_mode):
    """One launch of the real nodes in ``map_mode``; returns ([(stage, ok)], children exited cleanly, run dir)."""
    from rescue_bringup.operations_launch import db_writer_parameters, report_node_parameters
    from rescue_bringup.profile import load_deployment
    from rescue_console.app import current_view
    from rescue_operations import site_state

    port = API_PORTS[map_mode]
    run_dir = os.path.join(scratch, f'rehearsal-{map_mode.lower()}-{uuid4()}')
    os.makedirs(run_dir)
    profile = os.path.join(repo, 'config', 'deployment.v2.example.json')
    db_path, report_db = os.path.join(run_dir, 'site_state.sqlite3'), os.path.join(run_dir, 'reports.sqlite3')
    deployment = load_deployment(profile)
    writer_params = params_file(os.path.join(run_dir, 'db_writer.json'), db_writer_parameters(
        deployment, db_path=db_path, api_port=port, command_timeout_s=COMMAND_TIMEOUT_S))
    report_params = params_file(os.path.join(run_dir, 'report_node.json'), report_node_parameters(
        db_path=db_path, report_db_path=report_db, envelope_schema_version='e3', publisher_id='ops-e3'))
    logs = {name: open(os.path.join(run_dir, f'{name}.log'), 'w')
            for name in ('coordinator', 'target_locator', 'db_writer', 'report_node')}
    children = {
        'coordinator': subprocess.Popen(node_command(repo, coordinator_auto_args(profile, map_mode)),
                                        stdout=logs['coordinator'], stderr=subprocess.STDOUT, text=True),
        'target_locator': subprocess.Popen(locator_command(LOCATOR_ARGS + window_args()), stdout=logs['target_locator'],
                                           stderr=subprocess.STDOUT, text=True),
        'db_writer': node('ros_node', writer_params, logs['db_writer']),
        'report_node': node('report_node', report_params, logs['report_node'])}
    rclpy.init()
    robots, tb6_camera = Robots(), Probe()
    webcam = Camera(robots)
    frozen_map = FrozenMap(robots) if map_mode == 'SLAM_FREEZE' else None
    receipts, states = [], []
    tb6_camera.create_subscription(ReportReceipt, '/rescue/report_receipts', receipts.append, 10)
    tb6_camera.create_subscription(SystemState, '/rescue/system_state', states.append, 10)
    executor = MultiThreadedExecutor()
    executor.add_node(robots)
    executor.add_node(tb6_camera)
    results, found = [], {}

    def spin(condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            robots.publish_status()
            executor.spin_once(timeout_sec=.1)
            if condition():
                return True
        return bool(condition())

    def stage(name, check):
        start = time.monotonic()
        try:
            ok, detail = check()
        except Exception as exc:  # a stage that crashes is a FAIL with its reason
            ok, detail = False, f'{type(exc).__name__}: {exc}'
        results.append((name, ok))
        print(f'{"PASS" if ok else "FAIL"} {name} ({time.monotonic() - start:.1f} s) {detail}', flush=True)
        return ok

    def read(fn, key, default):
        try:
            return getattr(site_state, fn)(db_path)[key]
        except Exception:  # the file appears when db_writer starts
            return default

    def mission():
        return read('read_mission_state', 'mission', None) or {}

    def target_row():
        return next((t for t in read('read_target_states', 'targets', []) if t['target_id'] == found.get('target')), None)

    def gate(name):
        return next(c for c in current_view(None, site_state_path=db_path)['commands'] if c['name'] == name)

    def kinds(goals, kind):
        return [g for g in goals if g.kind == kind]

    def observe(revision):
        """TB-6 MapState on ``revision`` (the frozen one carries the grid's snapshot) and one camera observation."""
        if frozen_map is not None and revision > 0:
            tb6_camera.map_state.publish(frozen_map.state(SCOUT, revision, frozen_map.frozen_snapshot(revision)))
        else:
            tb6_camera.publish_map_state(MAP, revision, tb6_camera.get_clock().now().nanoseconds - 5_000_000_000)
        # target_locator refuses a capture not after the revision by more than its clock bound (#207).
        spin(lambda: False, .5)
        tb6_camera.publish_static()
        stamp = tb6_camera.get_clock().now().nanoseconds
        tb6_camera.publish_map_odom(stamp)
        spin(lambda: False, .5)
        tb6_camera.publish_observation(stamp)

    def reason_seen(code, timeout):
        return spin(lambda: any(st.reason == code for st in states), timeout)

    try:
        ready = spin(lambda: robots.command.service_is_ready() and os.path.exists(db_path)
                     and tb6_camera.count_subscribers('/rescue/observations') >= 1, 30)
        print(f'[{map_mode}] graph ready={ready}', flush=True)

        def a1():
            robots.publish_disaster()
            return spin(lambda: mission().get('mode') == 'DISASTER', 10), f'mode={mission().get("mode")}'

        def a2():
            g = gate('START_SEARCH')
            return g['enabled'], f'reason={g["reason"]!r}'

        def a3():
            status, reply = post('START_SEARCH', port)
            got = spin(lambda: kinds(robots.goals, 'SEARCH_AREA'), 10)
            return status == 200 and reply.get('accepted') is True and got, f'http={status} {reply.get("reason")}'

        def b1():
            observe(0)
            ok = spin(lambda: tb6_camera.snapshots and tb6_camera.snapshots[-1].targets, 10)
            if ok:
                found['target'] = tb6_camera.snapshots[-1].targets[0].target_id
            return bool(ok), f'target={found.get("target")}'

        def b2():
            ok = spin(lambda: (target_row() or {}).get('persisted_report_id'), 15)
            return bool(ok), f'persisted_report_id={(target_row() or {}).get("persisted_report_id")}'

        def b3():
            got = spin(lambda: kinds(robots.support_goals, 'APPROACH_TARGET'), 10)
            goal = kinds(robots.support_goals, 'APPROACH_TARGET')[0] if got else None
            status, reply = post('DISPATCH_SUPPORT', port, target_id=found['target'], target_revision=0)
            a = goal.route.approach_goal.pose.position if goal else None
            return bool(goal) and (goal.has_approval, goal.target_id) == (False, found['target']) and (
                status, reply.get('accepted')) == (200, False), (
                f'approach_goal=({a.x:.2f}, {a.y:.2f})' if a else 'no goal') + f' http DISPATCH={status} {reply}'

        def c1():
            got = spin(lambda: kinds(robots.support_goals, 'RETURN_HOME'), 10)
            goal = kinds(robots.support_goals, 'RETURN_HOME')[0] if got else None
            return bool(goal) and goal.home_ref == f'home/{SUPPORT}', f'home_ref={goal.home_ref if goal else None}'

        def c2():
            status, reply = post('SET_CAMERA_WATCH', port, target_id=found['target'])
            ok = spin(lambda: webcam.slot.watch is not None, 10)
            req = webcam.requests[0] if webcam.requests else None
            return status == 200 and reply.get('accepted') is True and ok and [
                round(x, 6) for x in req.roi_xyxy_normalized] == ROI, f'http={status} slot_rev={webcam.slot.revision}'

        def c3():
            webcam.candidate(found['target'], webcam.slot.watch.watch_id)
            refused = spin(lambda: robots.refusals, 10)
            saw_hold = reason_seen('TASK_REFUSED', 5)
            again = spin(lambda: len(kinds(robots.goals, 'REOBSERVE_TARGET')) >= 2, 15)
            goals = kinds(robots.goals, 'REOBSERVE_TARGET')
            return bool(refused) and saw_hold and again and goals[0].task_id != goals[1].task_id, (
                f'reobserve goals={len(goals)} refused={len(robots.refusals)} TASK_REFUSED seen={saw_hold}')

        def c4():
            ok = spin(lambda: (target_row() or {}).get('rescue_state') == 'RESCUE_CONFIRMED', 15)
            return ok, f'rescue_state={(target_row() or {}).get("rescue_state")}'

        def c5():
            got = spin(lambda: kinds(robots.goals, 'RETURN_HOME'), 10)
            goal = kinds(robots.goals, 'RETURN_HOME')[0] if got else None
            done = spin(lambda: mission().get('phase') == 'FINISHED', 15)
            return bool(goal) and goal.home_ref == f'home/{SCOUT}' and done, (
                f'home_ref={goal.home_ref if goal else None} mission={mission().get("mode")}/{mission().get("phase")}')

        def f1():
            robots.publish_disaster()
            spin(lambda: mission().get('mode') == 'DISASTER', 10)
            observe(0)
            if spin(lambda: tb6_camera.snapshots and tb6_camera.snapshots[-1].targets, 10):
                found['target'] = tb6_camera.snapshots[-1].targets[0].target_id
            saved = spin(lambda: (target_row() or {}).get('persisted_report_id'), 15)
            waiting = reason_seen('MAP_FREEZE_WAITING_EXIT', 5) and not frozen_map.calls
            frozen_map.publish_exit()
            called = spin(lambda: frozen_map.calls, 10)
            spin(lambda: False, 1.)
            return bool(saved) and waiting and called and len(frozen_map.calls) == 1, (
                f'target={found.get("target")} saved={bool(saved)} MAP_FREEZE_WAITING_EXIT seen={waiting} '
                f'freeze calls={len(frozen_map.calls)}')

        def f2():
            frozen_map.publish_grid()
            tb6_camera.map_state.publish(frozen_map.state(SCOUT, 1, frozen_map.frozen_snapshot(1)))
            held = reason_seen('TARGET_REBASED_ON_FREEZE', 10)
            return held and not robots.support_goals, f'TARGET_REBASED_ON_FREEZE seen={held} sha256={frozen_map.sha256[:12]}'

        def f3():
            persisted_before = (target_row() or {}).get('persisted_report_id')
            observe(1)
            resaved = spin(lambda: (target_row() or {}).get('persisted_report_id') not in (None, persisted_before), 15)
            unlocalized = reason_seen('SUPPORT_NOT_LOCALIZED', 10)
            robots.pose_revision = 1
            frozen_map.support_state.publish(frozen_map.state(SUPPORT, 1, map_snapshot_id('unfrozen', MAP, 1, 1)))
            got = spin(lambda: kinds(robots.support_goals, 'APPROACH_TARGET'), 10)
            goal = kinds(robots.support_goals, 'APPROACH_TARGET')[0] if got else None
            revision = goal.route.frame_ref.map_revision if goal else None
            return bool(resaved) and unlocalized and revision == 1, (
                f'resaved={bool(resaved)} SUPPORT_NOT_LOCALIZED seen={unlocalized} approach map_revision={revision}')

        if map_mode == 'SAVED_MAP':
            plan = (('A1 DISASTER_SIGN -> DISASTER -> DB', a1), ('A2 console START_SEARCH enabled', a2),
                    ('A3 HTTP START_SEARCH -> TB-6 SEARCH_AREA', a3),
                    ('B1 observation + TF -> target_locator -> target', b1),
                    ('B2 report_node PERSISTED -> DB persisted_report_id', b2),
                    ('B3 TB-5 APPROACH_TARGET by itself; HTTP DISPATCH refused', b3),
                    ('C1 TB-5 arrived -> wait -> RETURN_HOME exit', c1),
                    ('C2 HTTP SET_CAMERA_WATCH -> camera slot armed', c2),
                    ('C3 candidate -> REOBSERVE refused -> TASK_REFUSED -> new REOBSERVE', c3),
                    ('C4 recheck ABSENT/CLEAR/VALID -> DB RESCUE_CONFIRMED', c4),
                    ('C5 TB-6 RETURN_HOME dock approach -> DB FINISHED', c5))
        else:
            plan = (('F1 SLAM_FREEZE: target on R0 PERSISTED -> wait for exit -> QR exit -> one freeze call', f1),
                    ('F2 frozen grid + TB-6 MapState R0+1 -> frozen; R0 target and exit re-based', f2),
                    ('F3 target seen on R0+1 PERSISTED, TB-5 localized -> APPROACH_TARGET on R0+1', f3))
        for name, check in plan:
            if not stage(name, check):
                break
        spin(lambda: not any(robots.active.values()), 8)
    finally:
        for child in children.values():
            child.terminate()
        codes = {}
        for name, child in children.items():
            try:
                codes[name] = child.wait(timeout=15)
            except subprocess.TimeoutExpired:  # a node that ignores SIGTERM is a FAIL, never left running
                child.kill()
                child.wait()
                codes[name] = 'NO_EXIT_15S_KILLED'
        robots.destroy_node()
        tb6_camera.destroy_node()
        rclpy.try_shutdown()
        for log in logs.values():
            log.close()
    clean = all(code in (0, -15) for code in codes.values())
    print(f'{"PASS" if clean else "FAIL"} [{map_mode}] children exit {codes}', flush=True)
    return results, clean, len(plan), run_dir


def main():
    repo, scratch = sys.argv[1], sys.argv[2]
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('unset ROS_DISCOVERY_SERVER first')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 10 or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('run with ROS_DOMAIN_ID>=10 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    ok, total, passed, dirs = True, 0, 0, []
    for map_mode in ('SAVED_MAP', 'SLAM_FREEZE'):
        results, clean, planned, run_dir = run(repo, scratch, map_mode)
        dirs.append(run_dir)
        total += planned + 1
        passed += sum(good for _, good in results) + clean
        ok = ok and clean and len(results) == planned and all(good for _, good in results)
    print(f'PASS {passed} FAIL {total - passed} of {total} (stages after a FAIL are not run) logs={dirs}')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
