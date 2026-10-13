"""ROS 2 boundary for the mission coordinator (SDD central.coordinator; IF-016/021/043/076/079).

ROS packages are imported inside functions, so this module stays importable and
``CoordinatorCore`` is testable without ROS. Thin version for the 10-13 run:

- ``/rescue/site_events`` DISASTER_SIGN -> ``MissionCoordinator`` mode gate (shown and
  stored); a ready SCOUT gets ``ExecuteTask(SEARCH_AREA)`` only after the operator's
  START_SEARCH (owner decision 2026-10-11, #85).
- ``/rescue/robots/{r}/task_admission`` is the only path to ACTIVE; the action
  result releases the reservation only with a confirmed stop.
- ``/rescue/operator_command`` (SubmitOperatorCommand) is judged by
  ``MissionCoordinator.handle_command``; a replayed command_id answers DUPLICATE
  with the first answer (C-11 row 10).

- Operator SET_CAMERA_WATCH -> ``SetCameraWatch`` to the profile's one TARGET_WATCH camera
  (``watch.CameraWatchBook``, C-11 rows 1-4); a TARGET_DISAPPEARED_CANDIDATE is judged against it
  in DISASTER only (#124) and the watch is released when its mission ends.
- ``home_poses`` (TB-6 docking approach point, TB-5 exit) -> HomePosition; ``/rescue/robots/{r}/map_state``
  (#176) VALID -> the robot's current map basis, which RETURN_HOME compares with its home.
- ``support_dispatch=AUTO`` (owner decision 2026-10-13, 1B·2B): once a target is CONFIRMED and its
  current revision PERSISTED, the SUPPORT gets one APPROACH_TARGET to ``approach_standoff_m`` from the
  target toward its current mission pose (``/rescue/robots/{SUPPORT}/pose``), facing the target, with no
  operator approval. ``OPERATOR`` keeps DISPATCH_SUPPORT / MOVE_SEGMENT.

  After it arrives (COMPLETED, stop confirmed) it waits ``support_exit_wait_s`` and gets one RETURN_HOME
  to its exit (home_ref ``home/<robot_id>``, Q1-b·5A).

Not yet (PR body): accepted candidate -> REOBSERVE (C-11 row 5), verdict linking and the TB-6 RETURN_HOME.
"""
from dataclasses import replace
import hashlib
import uuid
import json
import math
import subprocess
from pathlib import Path
from uuid import uuid4

from rescue_contracts.occupancy import compose_planar, planar_origin
from rescue_contracts import RescueError, SiteEvent, TaskAdmission, STAMP_SEMANTICS, duration_ns, seconds_to_ns, timestamp_ns

from .mission import ALL_COMMAND_FIELDS, HomePosition, MissionCoordinator, OperatorCommand, RecheckRequest
from .planning import Allocator, CoordinationError, GoalPoint, contract_value, finite, point, revision, uid
from .search import MissionSite, SearchCoordinator, SearchPolicy, mission_sites
from .verdict import Reference, judge
from .watch import CameraWatchBook, roi_value

SITE_EVENT_TOPIC = '/rescue/site_events'
TARGET_LOCATION_TOPIC = '/rescue/target_locations'
REPORT_TOPIC = '/rescue/reports'
REPORT_RECEIPT_TOPIC = '/rescue/report_receipts'
SYSTEM_STATE_TOPIC = '/rescue/system_state'
TARGETS_TOPIC = '/rescue/targets'
OPERATOR_COMMAND_SERVICE = '/rescue/operator_command'
ROBOT_PREFIX = '/rescue/robots/'  # + robot_id + '/execute_task' etc. (rescue_interfaces README)
# Mission refusal -> C-09 common code for SubmitOperatorCommand.reason_code (OPS-58, 2026-10-11).
# The REQUEST_RECHECK rows are C-11 row 10; the rest are [제안]. The mission reason is logged.
REASON_CODES = {
    'MODE_NOT_DISASTER': 'NOT_AUTHORIZED',
    'UNKNOWN_TARGET': 'NOT_FOUND', 'NO_HOME_POSITION': 'NOT_FOUND',
    'TARGET_NOT_CONFIRMED': 'PENDING_EVIDENCE', 'NO_ROUTE': 'PENDING_EVIDENCE', 'NO_VERDICT': 'PENDING_EVIDENCE',
    'ALREADY_DISPATCHED': 'TASK_BUSY', 'NOT_AWAITING_APPROVAL': 'TASK_BUSY', 'RECHECK_ALREADY_REQUESTED': 'TASK_BUSY',
    'ALREADY_RETURNING': 'TASK_BUSY', 'SEARCH_ALREADY_STARTED': 'TASK_BUSY',
    'TARGET_REVISION_STALE': 'INVALID_INPUT', 'ROUTE_REVISION_STALE': 'INVALID_INPUT',
    'SEGMENT_INDEX_MISMATCH': 'INVALID_INPUT', 'NOT_DISPATCHED_TARGET': 'INVALID_INPUT',
    'TARGET_ALREADY_JUDGED': 'INVALID_INPUT',
    'HOME_BASIS_MISMATCH': 'FRAME_REVISION_MISMATCH',
}
# The robot that a command may name (empty = not named).
COMMAND_ROLES = {'DISPATCH_SUPPORT': 'SUPPORT', 'CONTINUE_SEGMENT': 'SUPPORT', 'REQUEST_RECHECK': 'SCOUT',
                 'START_SEARCH': 'SCOUT'}
EXECUTOR_OUTCOMES = frozenset({'COMPLETED', 'CANCELED', 'EXPIRED', 'UNREACHABLE', 'FAILED'})
STOPPING_MOTION = frozenset({'STOPPING', 'STOP_UNCONFIRMED'})  # RobotStatus.motion_state
# Map mode (owner decision 2026-10-13, scope Q-29 ⑦): SLAM_FREEZE = no reference map, TB-6 maps while it searches
# and the map is frozen before TB-5 goes; SAVED_MAP = one launch basis for the whole mission (earlier runs).
MAP_MODES = ('SLAM_FREEZE', 'SAVED_MAP')
MAP_FREEZE_SERVICE = '/rescue/map/freeze'  # std_srvs/Trigger (통합-로봇); only ``success`` is used
FROZEN_MAP_TOPIC = '/rescue/map/frozen'    # nav_msgs/OccupancyGrid, RELIABLE / TRANSIENT_LOCAL
MAP_FREEZE_REASONS = ('MAP_FREEZE_WAITING_SCOUT_STOP', 'MAP_FREEZE_REQUESTED', 'MAP_FREEZE_FAILED', 'MAP_NOT_FROZEN',
                      'TARGET_NOT_ON_FROZEN_MAP', 'SCOUT_NOT_LOCALIZED', 'SUPPORT_NOT_LOCALIZED',
                      'SUPPORT_APPROACH_NOT_FREE', 'SUPPORT_EXIT_NOT_FREE', 'SCOUT_HOME_NOT_FREE',
                      # QR exit, re-basing at the freeze, TB-6 start pose (OPS-136·138·139·141·142)
                      'TARGET_REBASED_ON_FREEZE', 'EXIT_NOT_ON_FROZEN_MAP',
                      'SUPPORT_EXIT_UNREGISTERED', 'START_POSE_UNKNOWN', 'START_POSE_CONFLICT')
# TB-6 search (owner decision 2026-10-13 evening, OPS-136·139): PATROL = the field's patrol points, given in TB-6's
# start pose; AREAS = the profile search areas. Never both.
SEARCH_MODES = ('PATROL', 'AREAS')
PATROL_BASIS = 'PATROL_POINT'  # MissionSite.basis of a patrol point; must be in executable_bases
PATROL_NAMESPACE = uuid.uuid5(uuid.NAMESPACE_URL, 'rescue_coordination/patrol_point')
EXIT_LOCATION_TOPIC = '/rescue/exit_location'  # TargetLocation from the QR exit detector (통합-인식, 안 「나」)
EXIT_CLASS_LABEL = 'exit_qr'
START_POSE_TOPIC = '/rescue/map/start_pose/'  # + SCOUT robot_id; PoseWithCovarianceStamped, map, TRANSIENT_LOCAL


def start_pose_from_msg(msg, mission_frame):
    """PoseWithCovarianceStamped on START_POSE_TOPIC -> (x, y, yaw) in the mission frame (OPS-142).

    The yaw follows the shared planar rule (rescue_contracts.occupancy.planar_origin, the robot's own); another
    frame or a non-planar pose is refused, never projected."""
    if msg.header.frame_id != mission_frame:
        raise CoordinationError('FRAME_REVISION_MISMATCH', f'start pose frame {msg.header.frame_id!r} is not '
                                                           f'{mission_frame!r}')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    try:
        return planar_origin(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    except ValueError as exc:
        raise CoordinationError('INVALID_INPUT', f'start pose: {exc}') from exc


def patrol_site_id(map_session_id, index):
    """SEARCH_AREA ref of patrol point ``index`` in a map session: a UUID (the executor takes only UUID refs)."""
    return str(uuid.uuid5(PATROL_NAMESPACE, f'{map_session_id}/{index}'))


# C-10 (#88): internal segment phase -> TargetEstimate/SystemState segment_phase.
SEGMENT_PHASE_STATES = {'APPROVED': 'EXECUTING', 'MOVING': 'EXECUTING', 'AWAITING_APPROVAL': 'AWAITING_APPROVAL',
                        'ARRIVED': 'STOPPED'}
# Internal segment phase -> TargetEstimate.approach_state for the dispatched target [제안].
APPROACH_STATES = {'APPROVED': 'ASSIGNED', 'MOVING': 'IN_PROGRESS', 'AWAITING_APPROVAL': 'IN_PROGRESS',
                   'ARRIVED': 'APPROACH_COMPLETE'}
# Automatic approach phase -> TargetEstimate (approach_state, segment_phase) [제안, C-10 vocabulary].
AUTO_APPROACH_STATES = {'ISSUED': ('ASSIGNED', 'EXECUTING'), 'MOVING': ('IN_PROGRESS', 'EXECUTING'),
                        'ARRIVED': ('APPROACH_COMPLETE', 'STOPPED'), 'FAILED': ('FAILED', 'STOPPED')}
SUPPORT_DISPATCH_MODES = frozenset({'AUTO', 'OPERATOR'})
# TB-6 search start (owner decision 2026-10-13 evening, OPS-136; #85 changed): AUTO = the disaster sign starts it.
SEARCH_START_MODES = frozenset({'AUTO', 'OPERATOR'})


def system_mode(mode, phase):
    """SystemState.mode (C-10, #88): MISSION_FINISHED once the phase is FINISHED."""
    return 'MISSION_FINISHED' if phase == 'FINISHED' else mode


def git_sha():
    """Source checkout SHA, or 'UNKNOWN' (installed copies have no .git)."""
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def _invalid(detail):
    return CoordinationError('INVALID_INPUT', detail)


def site_event_from_msg(msg):
    """rescue_interfaces/SiteEvent -> rescue_contracts.SiteEvent (C-10).

    Only VALID visibility is an event; empty target/watch IDs are None; a missing
    error bound is None. Nothing is repaired: a bad message is INVALID_INPUT.
    """
    if msg.visibility_state != 'VALID':
        raise _invalid('site event visibility must be VALID')
    time = msg.time
    try:
        return SiteEvent(
            event_id=msg.event_id, camera_id=msg.camera_id, kind=msg.kind, observed_at_ns=time.observed_at_ns,
            evidence_id=msg.evidence.evidence_id, evidence_sha256=msg.evidence.sha256,
            confidence=float(msg.confidence), clock_domain_id=time.clock_domain_id,
            stamp_semantics=time.stamp_semantics, clock_state=time.clock_state,
            clock_error_bound_ns=time.clock_error_bound_ns if time.has_clock_error_bound else None,
            target_id=msg.target_id or None, watch_id=msg.watch_id or None)
    except RescueError as exc:
        raise _invalid(exc.detail) from exc


def admission_from_msg(msg):
    """rescue_interfaces/TaskAdmission -> rescue_contracts.TaskAdmission (the executor receipt)."""
    try:
        return TaskAdmission(msg.command_id, msg.task_id, msg.robot_id, msg.robot_boot_id, msg.authority_epoch,
                             msg.generation, bool(msg.accepted), 'ACCEPTED' if msg.accepted else msg.reason_code)
    except RescueError as exc:
        raise _invalid(exc.detail) from exc


def fill_execute_task_goal(goal, fields, *, coordinator_id):
    """``Decision.task_fields`` -> ExecuteTask goal (attribute assignment only).

    ``frame_ref.alignment_status`` stays empty for SEARCH_AREA: alignment is the
    mapping's call and the coordinator never asserts VALID. ``lease_id`` names the
    CommandLease this node renews for the task.
    """
    pose = fields['goal_pose']
    for name in ('command_id', 'task_id', 'robot_id', 'authority_epoch', 'generation', 'kind', 'issued_at_ns',
                 'deadline_ns'):
        setattr(goal, name, fields[name])
    goal.expected_robot_boot_id = fields['robot_boot_id']
    goal.coordinator_id = coordinator_id
    goal.policy_version = fields['policy_version']
    goal.target_id = fields['target_id'] or ''
    goal.target_estimate_revision = fields['target_revision'] or 0
    goal.search_area_ref = fields['search_area_ref'] or ''
    goal.home_ref = fields.get('home_ref') or ''  # RETURN_HOME: home/<robot_id>, the robot holds the pose
    goal.lease_id = fields['lease_id']
    frame = goal.frame_ref
    frame.robot_id = fields['robot_id']
    if pose is not None:
        frame.mission_frame = pose['frame_id']
        frame.map_session_id = pose['map_session_id']
        frame.map_revision = pose['map_revision']
        frame.transform_revision = pose['transform_revision']
    route, approval = fields.get('route'), fields.get('approval')
    goal.has_route, goal.has_approval = route is not None, approval is not None
    if route is not None:
        # Phase 1 rule (#88): approach_goal is the configured segment end; planned_path,
        # path_sha256, planner_evidence_id and the passage assessment stay empty.
        r, end = goal.route, route['approach_goal']
        for name in ('route_id', 'route_revision', 'robot_id', 'target_id', 'segment_index'):
            setattr(r, name, route[name])
        r.policy_version = fields['policy_version']
        r.generated_at_ns, r.valid_until_ns = route['generated_at_ns'], route['valid_until_ns']
        # frame_ref is the target's current one (alignment from mapping, never set here); the
        # top-level frame_ref carries the same basis for target tasks.
        for ref in (r.frame_ref, frame):
            for name in ('robot_id', 'native_frame', 'mission_frame', 'map_session_id', 'map_revision',
                         'transform_revision', 'alignment_status'):
                setattr(ref, name, getattr(route['frame_ref'], name))
        r.approach_goal.header.frame_id = route['frame_ref'].mission_frame
        a = r.assessment  # no passage evidence (#88): UNKNOWN, nothing known
        a.status, a.width_known, a.height_known, a.floor_geometry_known = 'UNKNOWN', False, False, False
        r.approach_goal.pose.position.x, r.approach_goal.pose.position.y = map(float, end.mission_xy)
        q = r.approach_goal.pose.orientation
        q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(end.yaw / 2.0), math.cos(end.yaw / 2.0)
    if approval is not None:
        for name, value in approval.items():
            setattr(goal.approval, name, value)
    return goal


def operator_command_from_request(req, roles):
    """SubmitOperatorCommand request -> OperatorCommand. Fields the operation does not use are not read."""
    if req.operation not in ALL_COMMAND_FIELDS:
        raise _invalid('operation not handled by the mission gate')
    if req.robot_id and req.robot_id != roles.get(COMMAND_ROLES.get(req.operation)):
        raise _invalid('robot_id must be empty or the role robot of this operation')
    used = ALL_COMMAND_FIELDS[req.operation]
    values = {'target_id': req.target_id or None, 'target_revision': req.expected_target_revision,
              'route_revision': req.route_revision, 'segment_index': req.segment_index}
    return OperatorCommand(command_id=req.command_id, kind=req.operation, operator_id=req.operator_id,
                           requested_at_ns=req.issued_at_ns,
                           **{name: values[name] if name in used else None for name in values})


def approach_point(target_xy, support_xy, standoff_m):
    """((x, y), yaw) ``standoff_m`` from the target toward the SUPPORT, facing the target; None on the target.

    A SUPPORT already within the standoff stays where it is and only turns to the target. Plain geometry
    in the mission frame; the executor and Nav2 check the path and the target's protected area.
    """
    dx, dy = support_xy[0] - target_xy[0], support_xy[1] - target_xy[1]
    distance = math.hypot(dx, dy)
    if distance == 0:
        return None
    if distance <= standoff_m:
        xy = (float(support_xy[0]), float(support_xy[1]))
    else:
        xy = (target_xy[0] + standoff_m * dx / distance, target_xy[1] + standoff_m * dy / distance)
    return xy, math.atan2(target_xy[1] - xy[1], target_xy[0] - xy[0])


def route_id(segment_goals):
    """sha256 of the configured segment-end list (작전 #88: route_id of the fixed route)."""
    text = json.dumps([[g.mission_xy[0], g.mission_xy[1], g.yaw, g.map_session_id, g.map_revision,
                        g.transform_revision] for g in segment_goals], separators=(',', ':'))
    return hashlib.sha256(text.encode()).hexdigest()


class CoordinatorCore:
    """Everything the node decides, without ROS. The node only moves messages.

    ``segment_goals`` (one ``GoalPoint`` per segment end, revision ``route_revision``)
    and ``observe_goal`` are deployment parameters (Phase 1 rule, #88). They are
    destinations only; the executor guards and Nav2 verify the path.
    """

    def __init__(self, mission, *, status_max_age_ns, segment_goals, route_revision, observe_goal, merge_distance_m,
                 report_retry_ns, lease_renew_period_ns, admission_timeout_ns, segment_passage_free,
                 lease_message_ttl_ns, camera_watch, target_clock_domain_id, target_max_age_ns,
                 target_max_future_ns, support_dispatch, approach_standoff_m, support_pose_max_age_ns,
                 support_exit_wait_ns, recheck_max_clock_error_ns, map_mode, freeze_clearance_m,
                 map_freeze_timeout_ns, map_snapshot_id, search_start, search_mode, patrol_points,
                 patrol_rounds, exit_standoff_m, new_id=None):
        if not isinstance(mission, MissionCoordinator):
            raise CoordinationError('INVALID_POLICY', 'mission coordinator type')
        contract_value(duration_ns, status_max_age_ns)
        contract_value(duration_ns, report_retry_ns)
        finite(merge_distance_m, 'merge_distance_m')
        if merge_distance_m <= 0 or report_retry_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'merge distance and report retry must be positive')
        # CommandLease length = the task lease the executor holds (SearchPolicy.lease_duration).
        lease_ns = seconds_to_ns(str(mission.search.policy.lease_duration))
        contract_value(duration_ns, lease_renew_period_ns)
        if not 0 < lease_renew_period_ns <= lease_ns // 2:
            raise CoordinationError('INVALID_POLICY', 'lease renewal period must be at most half the lease')
        contract_value(duration_ns, admission_timeout_ns)
        contract_value(duration_ns, lease_message_ttl_ns)
        if admission_timeout_ns <= 0 or lease_message_ttl_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'admission timeout and lease message TTL must be positive')
        segment_passage_free = tuple(segment_passage_free)
        if not all(type(value) is bool for value in segment_passage_free):
            raise CoordinationError('INVALID_POLICY', 'segment passage values must be booleans')
        segment_goals = tuple(segment_goals)
        if not segment_goals or not all(isinstance(g, GoalPoint) for g in segment_goals + (observe_goal,)):
            raise CoordinationError('INVALID_POLICY', 'segment goals and observation goal must be GoalPoint')
        revision(route_revision)
        if camera_watch is not None and not isinstance(camera_watch, CameraWatchBook):
            raise CoordinationError('INVALID_POLICY', 'camera_watch must be a CameraWatchBook or None')
        self.camera_watch = camera_watch  # None: the profile has no TARGET_WATCH camera
        if not isinstance(search_start, str) or search_start not in SEARCH_START_MODES:
            raise CoordinationError('INVALID_POLICY', 'search_start must be AUTO or OPERATOR')
        self.search_start = search_start
        if support_dispatch not in SUPPORT_DISPATCH_MODES:
            raise CoordinationError('INVALID_POLICY', 'support_dispatch must be AUTO or OPERATOR')
        finite(approach_standoff_m, 'approach_standoff_m')
        contract_value(duration_ns, support_pose_max_age_ns)
        if approach_standoff_m <= 0 or support_pose_max_age_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'approach standoff and pose age must be positive')
        self.support_dispatch, self.approach_standoff_m = support_dispatch, approach_standoff_m
        self.support_pose_max_age_ns = support_pose_max_age_ns
        self.support_pose = None   # (mission_xy, (session, revision, transform), mission_frame, status, observed_at_ns)
        self.approach_task = None  # the open APPROACH_TARGET task_id
        self.approach_failure = None  # executor terminal_reason (or outcome) of a failed approach, for the reason
        contract_value(duration_ns, support_exit_wait_ns)
        self.support_exit_wait_ns = support_exit_wait_ns
        # TB-5 to the exit after it reached the target (Q1-b): None, WAITING, ISSUED, MOVING, AT_EXIT, CANCELED, FAILED
        self.support_exit, self.exit_due_ns = None, None
        contract_value(duration_ns, recheck_max_clock_error_ns)
        self.recheck_max_clock_error_ns = recheck_max_clock_error_ns  # C-11 row 8 limit on the error sum
        if map_mode not in MAP_MODES:
            raise CoordinationError('INVALID_POLICY', 'map_mode must be SLAM_FREEZE or SAVED_MAP')
        finite(freeze_clearance_m, 'freeze_clearance_m')
        contract_value(duration_ns, map_freeze_timeout_ns)
        if freeze_clearance_m <= 0 or map_freeze_timeout_ns <= 0 or not callable(map_snapshot_id):
            raise CoordinationError('INVALID_POLICY', 'freeze clearance and timeout must be positive, '
                                                      'map_snapshot_id a function')
        self.map_mode, self.freeze_clearance_m = map_mode, freeze_clearance_m
        self.map_freeze_timeout_ns, self.map_snapshot_id = map_freeze_timeout_ns, map_snapshot_id
        # SLAM_FREEZE: None -> REQUESTED (sent) -> AWAITING_MAP (success) -> FROZEN, or FAILED (never asked again)
        self.freeze, self.freeze_due_ns, self.freeze_send = None, None, False
        self.frozen = None         # (map_session_id, map_revision, transform_revision) of the frozen map
        self.frozen_grid = None    # the grid of that basis: ``sha256`` and ``clear(x, y, radius)``
        self.map_grid = None       # latest grid on FROZEN_MAP_TOPIC (latched: may be an older run's until frozen)
        self.scout_map_state = None  # TB-6's latest (map_session_id, map_revision, snapshot_id)
        if search_mode not in SEARCH_MODES:
            raise CoordinationError('INVALID_POLICY', 'search_mode must be PATROL or AREAS')
        patrol_points = tuple(tuple(float(v) for v in p) for p in patrol_points)
        if not all(len(p) == 3 and all(math.isfinite(v) for v in p) for p in patrol_points):
            raise CoordinationError('INVALID_POLICY', 'patrol points are finite [x, y, yaw]')
        if search_mode == 'PATROL' and (not patrol_points
                                        or PATROL_BASIS not in mission.search.policy.executable_bases):
            raise CoordinationError('INVALID_POLICY', f'PATROL needs patrol points and {PATROL_BASIS} executable')
        if type(patrol_rounds) is not int or patrol_rounds < 1:
            raise CoordinationError('INVALID_POLICY', 'patrol_rounds must be an integer >= 1')
        finite(exit_standoff_m, 'exit_standoff_m')
        if exit_standoff_m <= 0:
            raise CoordinationError('INVALID_POLICY', 'exit_standoff_m must be positive')
        self.search_mode, self.patrol_points, self.patrol_rounds = search_mode, patrol_points, patrol_rounds
        self.exit_standoff_m = exit_standoff_m
        self.patrol_round, self.patrol_exhausted = 1, False
        if search_mode == 'PATROL':  # no area until the start pose places the points (OPS-141)
            mission.search.policy = replace(mission.search.policy, areas=())
        self.start_pose = None          # TB-6 start pose in the map, bound to its search basis (OPS-142)
        self.start_pose_pending = None  # received, waiting for TB-6's VALID MapState on the search revision
        self.start_pose_conflict = False
        # Registered exits, one per QR id (observation.track_id, OPS-157·158), in first-seen order:
        # dict(exit_id, basis, xy, revision, location_id). The latest position of an id counts.
        self.exits = []
        self.exit_locations = {}        # location_id -> (basis, xy): replays and conflicts
        self.rebased = set()            # targets re-based at the freeze, shown until that revision is saved
        self.rebase_reports = []        # (target_id, revision, location) reports to queue at the next reports_due
        self.verdict_reason = None  # 'RECHECK_NEEDED_<reason>' of the latest judgement, for SystemState
        self.scout_return = None    # TB-6 after RESCUE_CONFIRMED (5A): None, ISSUED, MOVING, AT_DOCK_APPROACH, FAILED
        self.at_home = set()        # role robots at their home/exit, for FINISHED
        self.notes = []             # one log line per verdict for the operator; the node logs and drains them
        self.candidate_results = {}  # candidate event_id -> its first reason, None = a row-3 conflict (replay = same)
        self.watch_rois = {}  # SET_CAMERA_WATCH command_id -> raw request ROI (replay content check)
        if not isinstance(target_clock_domain_id, str) or not target_clock_domain_id:
            raise CoordinationError('INVALID_POLICY', 'target_clock_domain_id required')
        contract_value(duration_ns, target_max_age_ns)
        contract_value(duration_ns, target_max_future_ns)
        if target_max_age_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'target_max_age must be positive')
        self.target_clock_domain_id = target_clock_domain_id
        self.target_max_age_ns, self.target_max_future_ns = target_max_age_ns, target_max_future_ns
        self.location_refusal = None  # why the latest location made no target; cleared by the next accepted one
        self.mission = mission
        self.search = mission.search
        self.roles = {role: robot_id for robot_id, role in self.search.policy.roles}
        self.status_max_age_ns = status_max_age_ns
        self.segment_goals, self.route_revision, self.observe_goal = segment_goals, route_revision, observe_goal
        self.route_id = route_id(segment_goals)
        self.observe_route_id = route_id((observe_goal,))
        self.merge_distance_m, self.report_retry_ns = merge_distance_m, report_retry_ns
        self.new_id = new_id or (lambda: str(uuid4()))
        self.locations = {}      # location_id -> (target_id, TargetLocation msg, content key)
        self.target_geo = {}     # target_id -> (basis, (x, y), [location msgs]) of the latest merge
        self.reports = {}        # report_id -> dict(report fields, next_send_ns, done)
        self.saved_records = set()  # location_ids some PERSISTED receipt included; never sent again
        self.issue_block = None  # why an approved/open task is held right now; recomputed every tick
        self.state_sequence, self.last_state = 0, None
        self.last_targets = None
        self.lease_ns, self.lease_renew_period_ns = lease_ns, lease_renew_period_ns
        self.leases = {}         # task_id -> lease dict; removed for good when renewal stops
        self.status_arrivals = 0
        self.admission_timeout_ns = admission_timeout_ns
        # AST88-01 (OPS-60): field-checked "no passage assessment needed" per segment end. A
        # list of another length marks nothing passage-free.
        self.segment_passage_free = segment_passage_free
        self.lease_message_ttl_ns = lease_message_ttl_ns  # AST88-02: CommandLease receive deadline
        self.issued_at = {}      # task_id -> issued_at_ns, for the admission timeout
        self.timed_out = {}      # task_id -> status arrival count when its admission timed out
        self.status_detail = {}  # robot_id -> (arrival, stop_reason, motion_state, lease_valid, active_task_id)
        self.status = {}         # robot_id -> latest RobotStatus fields used here
        self.route_made_at = {}  # target_id -> wall time its routes were made (target creation), resent as is
        self.replies = {}        # command_id -> (OperatorCommand, reply dict)
        self.accepted_at = {}    # command_id -> acceptance time (approval approved_at_ns)
        self.segment_tasks = {}  # approval command_id -> task_id (one approval, one task)
        self.recheck_tasks = {}  # recheck link_id -> task_id; dropped when a recheck did not complete
        self.task_meta = {}      # task_id -> ('MOVE_SEGMENT', SegmentApproval) | ('REOBSERVE_TARGET', link_id)
        self.outbox = []         # task fields reserved and not yet handed to the node
        self.last_reason = None  # mission reason behind the latest reply, for the node log
        # Refused before moving: approval command_id / recheck link_id -> hold-until ns (None until the next
        # tick sets it). The same approval or recheck goes again as a new task after admission_timeout_ns.
        self.refused = {}

    # -- inputs --------------------------------------------------------------
    def on_site_event(self, msg):
        """Mode gate for every event; a disappearance candidate is then judged against the watch (C-11 3-4)."""
        event = site_event_from_msg(msg)
        replay = event.event_id in self.mission.events
        result = self.mission.record_site_event(event)  # event_id idempotency and conflict first (row 3)
        if event.kind != 'TARGET_DISAPPEARED_CANDIDATE':
            return result
        if replay:
            reason = self.candidate_results[event.event_id]
            if reason is None:  # its first arrival was a row-3 conflict: the replay is the same conflict
                raise CoordinationError('ID_CONTENT_CONFLICT', 'another candidate event_id for the same watch_id')
            return reason
        if self.mission.mode != 'DISASTER' or self.camera_watch is None:
            reason = 'CANDIDATE_NOT_DISASTER' if self.mission.mode != 'DISASTER' else 'CANDIDATE_NOT_LINKED'
        else:
            try:
                reason = self.camera_watch.on_candidate(event)
                if reason == 'CANDIDATE_ACCEPTED':
                    self._open_camera_recheck(event)
            except CoordinationError:
                self.candidate_results[event.event_id] = None
                raise
        self.candidate_results[event.event_id] = reason
        return reason

    def watch_requests(self, now_ns):
        """SetCameraWatch request fields to send now (arm on command, release from ``tick``)."""
        contract_value(timestamp_ns, now_ns)
        return self.camera_watch.drain() if self.camera_watch is not None else []

    def on_watch_reply(self, **reply):
        """The camera's SetCameraWatch response (watch_id, accepted, watch_revision, reason_code)."""
        return self.camera_watch is not None and self.camera_watch.on_reply(**reply)

    def on_status(self, msg):
        if msg.robot_id not in self.roles.values():
            raise CoordinationError('WRONG_ROBOT', 'status from a robot outside the policy')
        self.status[msg.robot_id] = (msg.robot_boot_id, msg.readiness, bool(msg.lease_valid), msg.observed_at_ns)
        self.status_arrivals += 1  # arrival order at this node; no clock comparison
        self.status_detail[msg.robot_id] = (self.status_arrivals, msg.stop_reason, msg.motion_state,
                                            bool(msg.lease_valid), msg.active_task_id)

    def on_map_state(self, msg):
        """MapState of a role robot (#176): a VALID localization is its current map basis for RETURN_HOME.

        transform_revision is not in MapState; it is the profile map_basis value, which is this coordinator's
        goal basis. Anything but VALID withdraws the robot's basis (HOME_BASIS_MISMATCH), never keeps an old one.
        """
        if msg.robot_id not in self.roles.values():
            raise CoordinationError('WRONG_ROBOT', 'map state from a robot outside the policy')
        if self.map_mode == 'SLAM_FREEZE' and msg.robot_id == self.roles['SCOUT']:
            self.scout_map_state = (msg.map_session_id, msg.map_revision, msg.snapshot_id)
            self._try_freeze()
        if msg.localization_quality != 'VALID':
            self.mission.map_basis.pop(msg.robot_id, None)
            return False
        self.mission.record_map_basis(msg.robot_id, map_session_id=msg.map_session_id, map_revision=msg.map_revision,
                                      transform_revision=self.observe_goal.transform_revision)
        self._bind_start_pose()
        return True

    # -- TB-6 start pose and patrol (OPS-136·139·141·142) ---------------------------------------------------
    def on_start_pose(self, xy_yaw):
        """TB-6's first SLAM pose in the map (x, y, yaw). Bound once to TB-6's VALID MapState on the search basis;
        the same value again is a replay, another one is refused and shown (START_POSE_CONFLICT)."""
        pose = tuple(float(v) for v in xy_yaw)
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise CoordinationError('INVALID_INPUT', 'start pose is a finite (x, y, yaw)')
        known = self.start_pose or self.start_pose_pending
        if known is not None:
            if known != pose:
                self.start_pose_conflict = True
                raise CoordinationError('ID_CONTENT_CONFLICT', 'another TB-6 start pose in the same map session')
            return False
        self.start_pose_pending = pose
        return self._bind_start_pose()

    def _bind_start_pose(self):
        scout = self.roles['SCOUT']
        if (self.start_pose is not None or self.start_pose_pending is None
                or self.mission.map_basis.get(scout) != self._search_basis()):
            return False
        self.start_pose, self.start_pose_pending = self.start_pose_pending, None
        session, rev, transform = self._search_basis()
        if self.search_mode == 'PATROL':
            sites = []
            for i, point_ in enumerate(self.patrol_points):
                x, y, yaw = compose_planar(self.start_pose, point_)
                sites.append(MissionSite(patrol_site_id(session, i), (x, y), PATROL_BASIS, session, rev, transform,
                                         entry_yaw=yaw))
            self.search.policy = replace(self.search.policy, areas=tuple(sites))
        if self.map_mode == 'SLAM_FREEZE' and scout in self.mission.homes:
            # The docking approach point is measured from TB-6's start pose, not the SLAM origin (OPS-141).
            home = self.mission.homes[scout]
            x, y, yaw = compose_planar(self.start_pose, (*home.mission_xy, home.yaw))
            self.mission.homes = self.mission.homes | {scout: replace(home, mission_xy=(x, y), yaw=yaw)}
        self.notes.append(f'START_POSE bound {self.start_pose} session={session} revision={rev}')
        return True

    def _patrol_tick(self):
        """Next round once every point of this one ended; PATROL_EXHAUSTED after ``patrol_rounds``."""
        areas = self.search.policy.areas
        if self.search_mode != 'PATROL' or not areas or self.patrol_exhausted:
            return
        ids = {a.site_id for a in areas}
        if not ids <= self.search.completed_areas or self.search.assignment(self.roles['SCOUT']) is not None:
            return
        if self.patrol_round >= self.patrol_rounds:
            self.patrol_exhausted = True
            self.notes.append(f'PATROL_EXHAUSTED after {self.patrol_rounds} rounds')
            return
        self.search.completed_areas -= ids
        self.patrol_round += 1
        self.notes.append(f'PATROL_ROUND {self.patrol_round}')

    def _search_reason(self):
        if self.start_pose_conflict:
            return 'START_POSE_CONFLICT'
        if self.search_queued() and self.search_mode == 'PATROL' and self.start_pose is None:
            return 'START_POSE_UNKNOWN'
        if self.search_queued():
            return self._search_wait_reason()
        if self.search_mode == 'PATROL' and self.mission.search_started:
            return 'PATROL_EXHAUSTED' if self.patrol_exhausted else f'PATROL_ROUND_{self.patrol_round}'
        return ''

    # -- QR exit (안 「나」, OPS-136·138·139) ------------------------------------------------------------------
    def on_exit_location(self, msg, *, now_ns):
        """A VALID exit_qr TargetLocation with its QR id (track_id) registers or moves that exit: the search basis
        before the freeze, the frozen one after it. Never merged with a person target. The allow list of QR ids is
        perception's (OPS-158), not checked twice here."""
        contract_value(timestamp_ns, now_ns)
        if msg.observation.class_label != EXIT_CLASS_LABEL:
            raise CoordinationError('INVALID_INPUT', f'{EXIT_LOCATION_TOPIC} carries {EXIT_CLASS_LABEL} only')
        if msg.projection_status != 'VALID':
            raise CoordinationError('STALE_OBSERVATION', 'only VALID projections register an exit')
        if not msg.observation.has_track_id or not msg.observation.track_id:
            raise CoordinationError('INVALID_INPUT', 'an exit needs its QR id in observation.track_id')
        exit_id = msg.observation.track_id
        uid(msg.location_id)
        f = msg.frame_ref
        basis = (uid(f.map_session_id), revision(f.map_revision), revision(f.transform_revision))
        xy = point((float(msg.mission_position.x), float(msg.mission_position.y)))
        seen = self.exit_locations.get(msg.location_id)
        if seen is not None:
            if seen != (exit_id, basis, xy):
                raise CoordinationError('ID_CONTENT_CONFLICT', 'exit content differs for the same location_id')
            return False
        self._check_observation_time(msg.observation.time, now_ns)
        if (basis != self.basis() or f.mission_frame != self.search.policy.mission_frame_id
                or f.alignment_status != 'VALID'):
            raise CoordinationError('FRAME_REVISION_MISMATCH', 'exit is not on the current map basis')
        self.exit_locations[msg.location_id] = (exit_id, basis, xy)
        known = next((e for e in self.exits if e['exit_id'] == exit_id), None)
        entry = dict(exit_id=exit_id, basis=basis, xy=xy, revision=known['revision'] + 1 if known else 1,
                     location_id=msg.location_id)
        self.exits = [entry if e is known else e for e in self.exits] if known else self.exits + [entry]
        self.notes.append(f'EXIT_REGISTERED {exit_id} revision={entry["revision"]} xy={xy} basis={basis} '
                          f'location={msg.location_id}')
        return True

    def _chosen_exit(self, target_id):
        """The exit TB-5 leads out by: the nearest one on the frozen map to the doll. Straight-line distance for
        now; the grid path length replaces it with the second freeze (OPS-154 3)."""
        frozen = [e for e in self.exits if e['basis'] == self.frozen]
        xy = self.target_geo[target_id][1]
        return min(frozen, key=lambda e: (math.dist(e['xy'], xy), e['exit_id'])) if frozen else None

    def _exit_point(self, target_id, exit_):
        """(xy, yaw) exit_standoff_m before the exit on the side of the target TB-5 comes from, facing the exit."""
        return approach_point(exit_['xy'], self.target_geo[target_id][1], self.exit_standoff_m)

    def _exit_ready(self, target_id):
        """TB-5's exit move: an exit registered on the frozen map, its approach free. Held with the reason otherwise;
        the dispatch itself never waits for an exit (OPS-154·156)."""
        if not self.exits:
            self.issue_block = 'SUPPORT_EXIT_UNREGISTERED'
            return False
        exit_ = self._chosen_exit(target_id)
        if exit_ is None:
            self.issue_block = 'EXIT_NOT_ON_FROZEN_MAP'
            return False
        point_ = self._exit_point(target_id, exit_)
        if point_ is None:
            self.issue_block = 'SUPPORT_EXIT_NOT_FREE'
            return False
        return self._free_cell(point_[0], 'SUPPORT_EXIT_NOT_FREE')

    def live_destinations(self):
        """RETURN_HOME destinations this coordinator computes (SLAM_FREEZE, after the freeze): TB-5's exit approach
        and TB-6's docking approach in the map, sent in ~/task_inputs (live_task_inputs.destinations; the robot
        takes its own 'home/<id>' key, #232)."""
        if self.map_mode != 'SLAM_FREEZE' or self.frozen is None:
            return {}
        out, frame_id = {}, self.search.policy.mission_frame_id
        scout, support = self.roles['SCOUT'], self.roles['SUPPORT']
        home = self.mission.homes.get(scout)
        if home is not None and self.start_pose is not None:
            out[f'home/{scout}'] = dict(robot_id=scout, frame_id=frame_id, xy_yaw=[*home.mission_xy, home.yaw],
                                        map_session_id=self.frozen[0], map_revision=self.frozen[1],
                                        transform_revision=self.frozen[2])
        target = self.mission.dispatch_target
        exit_ = self._chosen_exit(target) if target in self.target_geo else None
        if exit_ is not None:
            point_ = self._exit_point(target, exit_)
            if point_ is not None:
                out[f'home/{support}'] = dict(robot_id=support, frame_id=frame_id, xy_yaw=[*point_[0], point_[1]],
                                              map_session_id=self.frozen[0], map_revision=self.frozen[1],
                                              transform_revision=self.frozen[2])
        return out

    # -- map freeze (SLAM_FREEZE; owner decision 2026-10-13, 작전 OPS-113..117) ------------------------------
    def basis(self):
        """The basis every goal carries: the frozen map once frozen, else the launch values (search, SAVED_MAP)."""
        return self.frozen or self._search_basis()

    def _on_basis(self, goal):
        """A configured goal on the current basis: same mission xy and yaw, the frozen map once frozen."""
        return goal if self.frozen is None else GoalPoint(goal.mission_xy, goal.yaw, *self.frozen)

    def _search_basis(self):
        o = self.observe_goal
        return (o.map_session_id, o.map_revision, o.transform_revision)

    def _target_saved(self):
        return any(confirmed and persisted for _, confirmed, persisted in self.mission.targets.values())

    def _freeze_tick(self, now_ns):
        """Ask for the freeze once: a target saved (PERSISTED), TB-6 READY, STOPPED and without a task.

        A goal moving on the search revision is cancelled by the executor when the revision changes, so TB-6 must
        stand still first (작전 OPS-117). No answer and no matching map before the timeout fails the freeze.
        """
        if self.map_mode != 'SLAM_FREEZE':
            return
        if self.freeze in ('REQUESTED', 'AWAITING_MAP') and now_ns > self.freeze_due_ns:
            self._freeze_failed(f'no frozen map matching TB-6 MapState within the timeout (state {self.freeze})')
        if self.freeze is not None or not self._target_saved():
            return  # the doll's freeze; it never waits for an exit (OPS-156)
        scout = self.roles['SCOUT']
        state, detail = self.ready(scout, now_ns), self.status_detail.get(scout)
        if (state is None or not state[1] or detail is None or detail[2] != 'STOPPED'
                or self.search.assignment(scout) is not None):
            return  # the approach gate shows MAP_FREEZE_WAITING_SCOUT_STOP
        self.freeze, self.freeze_due_ns, self.freeze_send = 'REQUESTED', now_ns + self.map_freeze_timeout_ns, True
        self.notes.append('MAP_FREEZE requested: target saved, TB-6 stopped')

    def take_freeze_request(self):
        """True once when the node must call MAP_FREEZE_SERVICE."""
        send, self.freeze_send = self.freeze_send, False
        return send

    def on_freeze_reply(self, success, message):
        """The Trigger answer: only ``success`` counts; ``message`` is for the log."""
        if self.freeze != 'REQUESTED':
            return False
        if not success:
            self._freeze_failed(f'freeze refused: {message}')
            return False
        self.freeze = 'AWAITING_MAP'
        self._try_freeze()
        return True

    def on_frozen_map(self, grid):
        """The grid on FROZEN_MAP_TOPIC (``sha256``, ``clear(x, y, radius)``); bound to a basis only by its hash."""
        self.map_grid = grid
        return self._try_freeze()

    def on_frozen_map_invalid(self, detail):
        """A grid on FROZEN_MAP_TOPIC the shared rule refuses (ValueError): fails a freeze being waited for."""
        if self.freeze in ('REQUESTED', 'AWAITING_MAP'):
            self._freeze_failed(f'invalid frozen map: {detail}')
            return True
        return False

    def _try_freeze(self):
        """FROZEN when TB-6's MapState on a revision other than the search one names this grid's snapshot.

        A mismatch is not failed at once: the latched grid or MapState may still be the previous one. The
        timeout in ``_freeze_tick`` fails a freeze that never matches.
        """
        if self.freeze != 'AWAITING_MAP' or self.map_grid is None or self.scout_map_state is None:
            return False
        session, rev, snapshot = self.scout_map_state
        transform = self.observe_goal.transform_revision
        if rev == self.observe_goal.map_revision or snapshot != self.map_snapshot_id(
                self.map_grid.sha256, session, rev, transform):
            return False
        self.frozen, self.frozen_grid, self.freeze = (session, rev, transform), self.map_grid, 'FROZEN'
        # Homes keep their measured xy; the frozen map is the same SLAM frame (Q-29 ⑦), checked free below.
        self.mission.homes = {r: replace(h, map_session_id=session, map_revision=rev)
                              for r, h in self.mission.homes.items()}
        self.notes.append(f'MAP_FROZEN session={session} revision={rev} snapshot={snapshot} '
                          f'sha256={self.map_grid.sha256}')
        self._rebase_on_freeze()
        return True

    def _rebase_on_freeze(self):
        """OPS-138·139: TB-6 stands in front of whichever it found last, so the other was seen on the search
        revision. Same map session only: the latest target and exit positions of the search revision move to the
        frozen revision. A target gets revision + 1, saved again with its latest location; recorded in the log and
        in SystemState (TARGET_REBASED_ON_FREEZE) until saved. Another session is never re-based."""
        search = self._search_basis()
        if self.frozen[0] != search[0]:
            return
        for target_id, (basis, xy, locations) in list(self.target_geo.items()):
            if basis != search:
                continue
            rev, confirmed, _ = self.mission.targets[target_id]
            self.target_geo[target_id] = (self.frozen, xy, locations)
            self.mission.record_target(target_id, rev + 1, detection_confirmed=confirmed, report_persisted=False)
            self.rebased.add(target_id)
            self.rebase_reports.append((target_id, rev + 1, locations[-1]))
            self.notes.append(f'TARGET_REBASED_ON_FREEZE target={target_id} revision={rev + 1} xy={xy} '
                              f'from={search} to={self.frozen}')
        for i, exit_ in enumerate(self.exits):
            if exit_['basis'] == search:
                self.exits[i] = exit_ | dict(basis=self.frozen)
                self.notes.append(f'EXIT_REBASED_ON_FREEZE {exit_["exit_id"]} xy={exit_["xy"]} from={search} '
                                  f'to={self.frozen}')

    def _freeze_failed(self, detail):
        self.freeze, self.freeze_send = 'FAILED', False
        self.notes.append(f'MAP_FREEZE_FAILED {detail}')

    def _frozen_gate(self, target_id):
        """TB-5 goes only on the frozen map: target observed there, both robots localized there (SLAM_FREEZE)."""
        if self.map_mode != 'SLAM_FREEZE':
            return True
        if self.freeze != 'FROZEN':
            self.issue_block = {None: 'MAP_FREEZE_WAITING_SCOUT_STOP', 'REQUESTED': 'MAP_FREEZE_REQUESTED',
                                'AWAITING_MAP': 'MAP_NOT_FROZEN', 'FAILED': 'MAP_FREEZE_FAILED'}[self.freeze]
            return False
        if self.target_geo[target_id][0] != self.frozen:
            self.issue_block = 'TARGET_NOT_ON_FROZEN_MAP'
            return False
        for role in ('SCOUT', 'SUPPORT'):
            if self.mission.map_basis.get(self.roles[role]) != self.frozen:
                self.issue_block = f'{role}_NOT_LOCALIZED'
                return False
        return True  # the exit is the exit move's matter (_home_free), not the dispatch's (OPS-156)

    def _free_cell(self, xy, reason):
        """True when ``xy`` is free on the frozen grid within freeze_clearance_m (always True in SAVED_MAP)."""
        if self.map_mode != 'SLAM_FREEZE':
            return True
        if self.frozen_grid is None:
            self.issue_block = 'MAP_NOT_FROZEN'
            return False
        if self.frozen_grid.clear(float(xy[0]), float(xy[1]), self.freeze_clearance_m) is not True:
            self.issue_block = reason
            return False
        return True

    def _home_free(self, role, reason):
        """The role robot's home (TB-6 docking approach point, TB-5 exit) is known and free on the frozen map."""
        if self.map_mode != 'SLAM_FREEZE':
            return True
        if role == 'SUPPORT':  # TB-5 ends at the registered exit (OPS-136 4), not a profile home
            return self._exit_ready(self.mission.dispatch_target)
        if self.start_pose is None:
            self.issue_block = 'START_POSE_UNKNOWN'  # the docking approach point is given in the start pose
            return False
        home = self.mission.homes.get(self.roles[role])
        if home is None:
            self.issue_block = 'NO_HOME_POSITION'  # nothing to check: never sent unchecked
            return False
        return self._free_cell(home.mission_xy, reason)

    def on_pose(self, msg):
        """RobotPose of the SUPPORT (its current mission pose); other robots are not used here."""
        if msg.robot_id not in self.roles.values():
            raise CoordinationError('WRONG_ROBOT', 'pose from a robot outside the policy')
        if msg.robot_id != self.roles['SUPPORT']:
            return False
        f, p = msg.frame_ref, msg.mission_pose.pose.pose.position
        self.support_pose = ((float(p.x), float(p.y)), (f.map_session_id, f.map_revision, f.transform_revision),
                             f.mission_frame, msg.mission_pose_status, msg.time.observed_at_ns)
        return True

    def on_admission(self, msg):
        admission = admission_from_msg(msg)
        admitted = self.search.record_admission(admission)
        meta = self.task_meta.get(admission.task_id)
        if admitted and admission.accepted and admission.task_id in self.leases:
            self.leases[admission.task_id]['admitted_after'] = self.status_arrivals
        if admitted and admission.accepted and meta and meta[0] == 'APPROACH_TARGET':
            self.mission.approach_started()
        if admitted and admission.accepted and meta and meta[0] == 'RETURN_HOME':
            if meta[1] == 'SUPPORT_EXIT':
                self.support_exit = 'MOVING'
            else:
                self.scout_return = 'MOVING'
        if admitted and admission.accepted and meta and meta[0] == 'MOVE_SEGMENT':
            self.mission.segment_started(meta[1].target_id, target_revision=meta[1].target_revision,
                                         route_revision=meta[1].route_revision, segment_index=meta[1].segment_index)
        elif not admission.accepted and meta and self.search.allocator.admissions.get(admission.task_id) == admission:
            # A refusal the Allocator recorded (it returns False for it, so ``admitted`` is never True here):
            # the task is released; its approval or recheck goes again after a hold.
            self.refused[meta[1].command_id if meta[0] == 'MOVE_SEGMENT' else meta[1]] = None
            self._forget(admission.task_id)
        return admitted

    def on_result(self, task_id, result):
        """Action result for one task; released only with a confirmed stop (Allocator rule)."""
        allocator = self.search.allocator
        item = allocator.active.get(task_id) or allocator.pending.get(task_id)
        if item is None or result.outcome not in EXECUTOR_OUTCOMES:
            return False
        stop = bool(result.stop_confirmed)
        if not self.search.finish(item, outcome=result.outcome, stop_confirmed=stop):
            return False
        if (self.search_mode == 'PATROL' and self.search.kind_of(item) == 'SEARCH_AREA'
                and result.outcome != 'COMPLETED'):
            self.search.completed_areas.add(item.target_id)  # passed: the patrol goes on to the next point
            self.notes.append(f'PATROL_POINT_PASSED {item.target_id} outcome={result.outcome}')
        if item.robot_id == self.roles['SCOUT'] and result.outcome == 'CANCELED':
            # The executor ends a stop (operator, lease, deadline) as CANCELED; a FAILED/UNREACHABLE
            # task is not a stop and leaves the waiting search in place.
            self._cancel_queued_search('SCOUT_TASK_CANCELED')
        meta = self.task_meta.get(task_id)
        if meta and meta[0] == 'MOVE_SEGMENT':
            approval = meta[1]
            self.mission.segment_finished(approval.target_id, target_revision=approval.target_revision,
                                          route_revision=approval.route_revision,
                                          segment_index=approval.segment_index,
                                          completed=result.outcome == 'COMPLETED', stop_confirmed=stop)
        elif meta and meta[0] == 'RETURN_HOME' and meta[1] == 'SUPPORT_EXIT':
            self.support_exit = 'AT_EXIT' if result.outcome == 'COMPLETED' else 'FAILED'  # FAILED: shown, not retried
            if self.support_exit == 'AT_EXIT':
                self.note_support_at_exit()
        elif meta and meta[0] == 'RETURN_HOME':
            # TB-6 stops at the docking approach point; docking itself is done by hand (supervised, 5A).
            self.scout_return = 'AT_DOCK_APPROACH' if result.outcome == 'COMPLETED' else 'FAILED'
            if self.scout_return == 'AT_DOCK_APPROACH':
                self._returned(self.roles['SCOUT'])
        elif meta and meta[0] == 'APPROACH_TARGET':
            self.mission.approach_finished(completed=result.outcome == 'COMPLETED', stop_confirmed=stop)
            self.approach_task = None  # ARRIVED or FAILED: never issued again
            if result.outcome != 'COMPLETED':
                # 작전 OPS-119: show why (e.g. PASSAGE_BLOCKED); an empty terminal reason shows the outcome.
                self.approach_failure = result.terminal_reason or result.outcome
        elif meta and meta[0] == 'REOBSERVE_TARGET':
            if result.outcome == 'COMPLETED':
                # Row 5: no new task after COMPLETED (only an operator REQUEST_RECHECK). Without an observation
                # there is nothing to judge: RECHECK_NEEDED, which closes the recheck for the operator.
                self._judge(task_id, meta[1], result.recheck if getattr(result, 'has_recheck', False) else None)
            else:
                self._forget(task_id)  # C-11 row 5: same recheck, new task_id and generation
        return True

    def on_target_location(self, msg, *, now_ns):
        """A VALID TargetLocation -> target_id (작전 2026-10-11, Q-13 merge by position, Q-21 distance).

        Same map basis (session, revision, transform revision) and within
        ``merge_distance_m`` of a target's latest position: same target, revision + 1.
        Farther: a new target. Another basis than an existing target: refused, never
        split into a new target (distances across bases mean nothing).
        """
        contract_value(timestamp_ns, now_ns)
        if msg.observation.class_label == EXIT_CLASS_LABEL:  # OPS-138: an exit is never a person target
            raise CoordinationError('INVALID_INPUT', f'{EXIT_CLASS_LABEL} belongs on {EXIT_LOCATION_TOPIC}')
        if msg.projection_status != 'VALID':
            raise CoordinationError('STALE_OBSERVATION', 'only VALID projections become targets')
        uid(msg.location_id)
        f = msg.frame_ref
        basis = (uid(f.map_session_id), revision(f.map_revision), revision(f.transform_revision))
        xy = point((float(msg.mission_position.x), float(msg.mission_position.y)))
        key = (basis, xy, f.alignment_status, f.mission_frame)
        seen = self.locations.get(msg.location_id)
        if seen is not None:
            if seen[2] != key:
                raise CoordinationError('ID_CONTENT_CONFLICT', 'location content differs for the same location_id')
            return seen[0]
        self._check_observation_time(msg.observation.time, now_ns)
        if self.frozen is not None and basis != self.frozen:
            raise CoordinationError('FRAME_REVISION_MISMATCH', 'location is not on the frozen map')
        # The frozen map is the search map's SLAM frame saved (Q-29 ⑦): a target seen before the freeze is
        # re-based by an observation on the frozen map within merge distance (revision + 1, saved again).
        bases = {basis} | ({self._search_basis()} if self.frozen is not None else set())
        if any(geo[0] not in bases for geo in self.target_geo.values()):
            raise CoordinationError('FRAME_REVISION_MISMATCH', 'location map basis differs from the known targets')
        near = sorted((math.dist(geo[1], xy), target_id) for target_id, geo in self.target_geo.items()
                      if math.dist(geo[1], xy) <= self.merge_distance_m)
        if near:
            target_id = near[0][1]
            locations = self.target_geo[target_id][2] + [msg]
            target_revision = self.mission.targets[target_id][0] + 1
        else:
            target_id, locations, target_revision = self.new_id(), [msg], 0
        # Thin version: the perception node publishes confirmed detections only (작전 decision 3).
        self.mission.record_target(target_id, target_revision, detection_confirmed=True, report_persisted=False)
        if not near:
            self.mission.record_route(target_id, route_revision=self.route_revision,
                                      segment_count=len(self.segment_goals))
            self.route_made_at[target_id] = now_ns
        self.target_geo[target_id] = (basis, xy, locations)
        self.locations[msg.location_id] = (target_id, msg, key)
        # SDD 391: records are deduplicated across reports. A report carries only the locations no
        # PERSISTED receipt included yet, so its size follows the unsaved backlog, not the target's
        # history (#137 measured all-locations reports hitting 64 KiB at the 34th location). An older
        # unsaved report of the target is covered by this one and stops being resent.
        for report in self.reports.values():
            if report['target_id'] == target_id and not report['done']:
                report['done'] = True
        self.rebase_reports = [r for r in self.rebase_reports if r[0] != target_id]  # this newer one covers it
        unsaved = [loc for loc in locations if loc.location_id not in self.saved_records]
        report_id = self.new_id()
        self.reports[report_id] = dict(report_id=report_id, target_id=target_id, target_revision=target_revision,
                                       record_ids=[loc.location_id for loc in unsaved], locations=unsaved,
                                       first_issued_at_ns=now_ns, next_send_ns=now_ns, done=False)
        return target_id

    def _check_observation_time(self, t, now_ns):
        """OPS-103 (#142): only an observation with a usable time becomes a target (C-03).

        Declared basis (stamp semantics known, SYNCED, an error bound), this coordinator's clock domain, not
        ahead of now by more than ``target_max_future_ns`` plus the observation's own declared error bound (PC
        clocks differ by hundreds of ms under timesyncd, measured 2026-10-13), not older than
        ``target_max_age_ns`` and before its own ``valid_until_ns``. The reason stays in SystemState.reason
        until a location is accepted.
        """
        checks = (
            (t.stamp_semantics in STAMP_SEMANTICS - {'UNKNOWN'} and t.clock_state == 'SYNCED'
             and bool(t.has_clock_error_bound),
             'CLOCK_UNCERTAIN', 'TARGET_TIME_BASIS_UNUSABLE'),
            (t.clock_domain_id == self.target_clock_domain_id, 'CLOCK_UNCERTAIN', 'TARGET_CLOCK_DOMAIN_MISMATCH'),
            (t.observed_at_ns - now_ns <= self.target_max_future_ns + int(t.clock_error_bound_ns),
             'STALE_OBSERVATION', 'TARGET_OBSERVATION_FUTURE'),
            (now_ns - t.observed_at_ns <= self.target_max_age_ns and now_ns < t.valid_until_ns,
             'STALE_OBSERVATION', 'TARGET_OBSERVATION_STALE'))
        for ok, code, reason in checks:
            if not ok:
                self.location_refusal = reason
                raise CoordinationError(code, reason)
        self.location_refusal = None

    def reports_due(self, now_ns):
        """RescueReports to (re)send now; resent every ``report_retry_ns`` until PERSISTED or REJECTED."""
        for target_id, target_revision, location in self.rebase_reports:
            # A re-based revision is saved again with its latest location (record ids may repeat across reports).
            for report in self.reports.values():
                if report['target_id'] == target_id and not report['done']:
                    report['done'] = True
            report_id = self.new_id()
            self.reports[report_id] = dict(report_id=report_id, target_id=target_id, target_revision=target_revision,
                                           record_ids=[location.location_id], locations=[location],
                                           first_issued_at_ns=now_ns, next_send_ns=now_ns, done=False)
        self.rebase_reports = []
        due = []
        for report in self.reports.values():
            if not report['done'] and report['next_send_ns'] <= now_ns:
                report['next_send_ns'] = now_ns + self.report_retry_ns
                due.append({k: v for k, v in report.items() if k not in ('next_send_ns', 'done')})
        return due

    def on_receipt(self, msg):
        """ReportReceipt: PERSISTED covering every record of our report saves that target revision."""
        report = self.reports.get(msg.report_id)
        if report is None or report['done']:
            return False
        if msg.state == 'REJECTED':
            report['done'] = True  # visible in the log; not resent, not saved (no silent fallback)
            return False
        if msg.state != 'PERSISTED' or not set(report['record_ids']) <= set(msg.included_record_ids):
            return False
        report['done'] = report['persisted'] = True
        self.saved_records.update(report['record_ids'])
        self.mission.record_target(report['target_id'], report['target_revision'], detection_confirmed=True,
                                   report_persisted=True)
        return True

    def system_state_change(self, now_ns):
        """SystemState fields (C-10, #88) when they changed since the last call, else None."""
        view = self.mission.view()
        allocator = self.search.allocator
        state = dict(
            mode=system_mode(view.mode, view.phase), phase=view.phase,
            cause_event_ids=[view.trigger_event_id] if view.trigger_event_id else [],
            active_task_ids=sorted(set(allocator.pending) | set(allocator.active)),
            policy_version=self.search.policy.version,
            reason=self.issue_block or self._rebase_reason() or self.mission_reason(now_ns) or self._search_reason() or (
                self.camera_watch.rearm if self.camera_watch is not None and self.camera_watch.rearm else '') or (
                self.location_refusal or ''),
            has_trigger=view.trigger_event_id is not None, trigger_event_id=view.trigger_event_id or '',
            trigger_observed_at_ns=view.trigger_observed_at_ns or 0,
            has_dispatch=view.target_id is not None, target_id=view.target_id or '',
            target_revision=view.target_revision or 0, route_revision=view.route_revision or 0,
            segment_index=view.segment_index or 0, segment_count=view.segment_count or 0,
            segment_phase=SEGMENT_PHASE_STATES.get(view.segment_phase, 'IDLE'))
        if state == self.last_state:
            return None
        self.last_state = state
        self.state_sequence += 1
        return state | dict(state_sequence=self.state_sequence, changed_at_ns=now_ns)

    def target_snapshot_change(self):
        """TargetSnapshot fields (IF-070) when the targets changed since the last call, else None.

        One TargetEstimate per target, ordered by target_id. ``persisted_report_id`` is
        the saved report of the current revision only (a newer revision is unsaved until
        its own receipt). Confirmation is CONFIRMED: the thin version takes confirmed
        detections only (작전 decision 3).
        """
        mission = self.mission
        saved = {(r['target_id'], r['target_revision']): r['report_id'] for r in self.reports.values()
                 if r['done'] and r.get('persisted')}
        targets = []
        for target_id in sorted(self.target_geo):
            basis, xy, locations = self.target_geo[target_id]
            target_revision = mission.targets[target_id][0]
            dispatched = target_id == mission.dispatch_target
            latest = locations[-1]
            if dispatched and mission.dispatch_kind == 'APPROACH':
                approach_state, segment_state = AUTO_APPROACH_STATES[mission.approach_phase]
            elif dispatched:
                approach_state = APPROACH_STATES[mission.segment_phase]
                segment_state = SEGMENT_PHASE_STATES[mission.segment_phase]
            else:
                approach_state, segment_state = 'UNASSIGNED', 'IDLE'
            targets.append(dict(
                target_id=target_id, estimate_revision=target_revision, locations=list(locations),
                latest_observed_at_ns=max(loc.observation.time.observed_at_ns for loc in locations),
                valid_until_ns=latest.observation.time.valid_until_ns, mission_position=xy,
                covariance=list(latest.covariance), confirmation_state='CONFIRMED',
                approach_state=approach_state, segment_phase=segment_state,
                rescue_state=mission.verdicts.get(target_id, 'NOT_CHECKED'),
                persisted_report_id=saved.get((target_id, target_revision), '')))
        if targets == self.last_targets:
            return None
        self.last_targets = targets
        return dict(snapshot_id=self.new_id(), targets=targets)

    def ready(self, robot_id, now_ns):
        """(robot_boot_id, ready) from the latest RobotStatus, or None before any status.

        Ready to take a new task: READY and fresh. lease_valid is not asked here: a
        lease belongs to a task, so an idle robot never has one (#100). It is watched
        only while a task runs (``_robot_stopped``).
        """
        status = self.status.get(robot_id)
        if status is None:
            return None
        boot, readiness, _, observed_at_ns = status
        fresh = observed_at_ns <= now_ns and now_ns - observed_at_ns <= self.status_max_age_ns
        return boot, readiness == 'READY' and fresh

    # -- decisions -------------------------------------------------------------
    def tick(self, now_ns):
        """Task fields to send now: approved segment, open rechecks, then the SCOUT's search once started."""
        contract_value(timestamp_ns, now_ns)
        self.issue_block = None  # only holds that still stand (OPS-70-01); the issuers below set them again
        self._expire_unadmitted(now_ns)
        self._watch_tick(now_ns)
        detail = self.status_detail.get(self.roles['SCOUT'])
        if self.mission.mode != 'DISASTER':
            self._cancel_queued_search('MODE_NOT_DISASTER')
        elif detail is not None and detail[2] in STOPPING_MOTION:
            self._cancel_queued_search('SCOUT_STOPPING')
        elif self.search_start == 'AUTO' and not self.mission.search_authorized:
            # The disaster sign is the start (OPS-136): the same authorization START_SEARCH gives; READY, lease,
            # clock and stop gates stay in mission.plan. Withdrawn by a stop or the end of DISASTER as before.
            self.mission.search_authorized = True
            self.notes.append('SEARCH_AUTO_STARTED on DISASTER')
        # Order matters: an open recheck takes a free SCOUT before a new search area does.
        # NORMAL needs no check here: no approval or recheck exists and mission.plan gates the search.
        self._freeze_tick(now_ns)
        self._issue_approach(now_ns)
        self._issue_support_exit(now_ns)
        self._issue_scout_return(now_ns)
        self._issue_segment(now_ns)
        self._issue_rechecks(now_ns)
        self._patrol_tick()
        self._issue_search(now_ns)
        sent, self.outbox = self.outbox, []
        return sent

    def on_command(self, req, *, now_ns):
        """SubmitOperatorCommand -> reply fields (accepted, request_state, reason_code, created_task_id)."""
        try:
            command = operator_command_from_request(req, self.roles)
        except RescueError as exc:
            self.last_reason = exc.detail
            return self._reply(False, 'REJECTED', exc.code)
        seen = self.replies.get(command.command_id)
        if command.kind == 'SET_CAMERA_WATCH':
            roi = tuple(req.roi_xyxy_normalized)
            if self.watch_rois.setdefault(command.command_id, roi) != roi:
                self.last_reason = 'ID_CONTENT_CONFLICT'
                return self._reply(False, 'REJECTED', 'ID_CONTENT_CONFLICT')
        if seen is not None:
            if seen[0] != command:
                self.last_reason = 'ID_CONTENT_CONFLICT'
                return self._reply(False, 'REJECTED', 'ID_CONTENT_CONFLICT')
            first = seen[1]
            self.last_reason = 'DUPLICATE'
            return self._reply(first['accepted'], 'DUPLICATE', first['reason_code'], first['created_task_id'])
        if command.kind == 'START_SEARCH' and self.search_start == 'AUTO':
            self.last_reason = 'AUTO_SEARCH_ENABLED'  # the disaster sign starts the search (OPS-136)
            reply = self._reply(False, 'REJECTED', 'NOT_AUTHORIZED')
            self.replies[command.command_id] = (command, reply)
            return reply
        if command.kind == 'DISPATCH_SUPPORT' and self.support_dispatch == 'AUTO':
            # 1B: the coordinator dispatches the SUPPORT itself; the operator path is for OPERATOR mode only.
            self.last_reason = 'AUTO_DISPATCH_ENABLED'
            reply = self._reply(False, 'REJECTED', 'NOT_AUTHORIZED')
            self.replies[command.command_id] = (command, reply)
            return reply
        if command.kind == 'SET_CAMERA_WATCH':
            refused = self._watch_precheck(req, command)
            if refused is not None:
                self.last_reason = refused[0]
                reply = self._reply(False, 'REJECTED', refused[1])
                self.replies[command.command_id] = (command, reply)
                return reply
        if (command.kind == 'START_SEARCH' and self.mission.mode == 'DISASTER' and not self.mission.search_authorized
                and not (self.ready(self.roles['SCOUT'], now_ns) or (None, False))[1]):
            # #98: a SCOUT of unknown or not-ready state refuses the start; it does not count as started,
            # so no robot leaves at an unknown moment after the button. The operator presses again when ready.
            self.last_reason = 'SCOUT_NOT_READY'
            reply = self._reply(False, 'REJECTED', 'NAV_NOT_READY')
            self.replies[command.command_id] = (command, reply)
            return reply
        result = self.mission.handle_command(command, now_ns=now_ns)
        self.last_reason = result.reason
        if result.accepted:
            self.accepted_at[command.command_id] = now_ns
            task_id = ''
            if command.kind in ('DISPATCH_SUPPORT', 'CONTINUE_SEGMENT'):
                task_id = self._issue_segment(now_ns) or ''
            elif command.kind == 'REQUEST_RECHECK':
                self._issue_rechecks(now_ns)
                task_id = self.recheck_tasks.get(command.command_id, '')
            elif command.kind == 'SET_CAMERA_WATCH':
                roi = roi_value(req.roi_xyxy_normalized) or self.camera_watch.roi
                watch = self.camera_watch.arm(command.command_id, command.target_id, roi, now_ns)
                self.last_reason = f'WATCH_REQUESTED:{watch.watch_id}'
            elif command.kind == 'START_SEARCH':
                queued = len(self.outbox)
                self._issue_search(now_ns)
                task_id = next((f['task_id'] for f in self.outbox[queued:] if f['kind'] == 'SEARCH_AREA'), '')
            # Empty created_task_id: accepted, the task follows when the robot is free and known.
            reply = self._reply(True, 'ACCEPTED', '', task_id)
        else:
            reply = self._reply(False, 'REJECTED', REASON_CODES[result.reason])
        self.replies[command.command_id] = (command, reply)
        return reply

    # -- camera watch --------------------------------------------------------------
    def _watch_precheck(self, req, command):
        """(mission-log reason, C-09 code) when the node itself cannot arm a watch, else None."""
        book = self.camera_watch
        if book is None:
            return 'NO_WATCH_CAMERA', 'NOT_FOUND'  # profile: no TARGET_WATCH camera
        try:
            roi = roi_value(req.roi_xyxy_normalized)
        except RescueError:
            return 'BAD_WATCH_ROI', 'INVALID_INPUT'
        if roi is None and book.roi is None:
            return 'NO_WATCH_ROI', 'PENDING_EVIDENCE'  # no full-frame default (Q-13 / #76 ROI source open)
        if book.pending is not None:
            return 'WATCH_PENDING', 'TASK_BUSY'
        return None

    def _watch_tick(self, now_ns):
        """Drop an unanswered request; release the active watch when its mission is over (#124, C-11 row 2)."""
        book = self.camera_watch
        if book is None:
            return
        book.expire(now_ns)
        active, mission = book.active, self.mission
        should_end = active is not None and (
            mission.mode != 'DISASTER' or mission.returning
            or mission.verdicts.get(active.target_id) == 'RESCUE_CONFIRMED')
        book.release_due(should_end, now_ns)

    # -- issuing -----------------------------------------------------------------
    def _free(self, role, now_ns):
        """robot_boot_id when the role robot is known, ready and has no task; else None."""
        robot_id = self.roles[role]
        state = self.ready(robot_id, now_ns)
        if state is None or not state[1] or self.search.assignment(robot_id) is not None:
            return None
        return state[0]

    def _passage_free(self, segment_index):
        flags = self.segment_passage_free
        return len(flags) == len(self.segment_goals) and flags[segment_index] is True

    def _route_frame(self, target_id, goal):
        """The target's current frame_ref when the goal shares its basis and alignment is VALID (#88)."""
        geo = self.target_geo.get(target_id)
        if geo is None:
            self.issue_block = 'TARGET_LOCATION_UNKNOWN'
            return None
        f = geo[2][-1].frame_ref
        if f.mission_frame != self.search.policy.mission_frame_id:
            self.issue_block = 'ROUTE_BASIS_MISMATCH'
            return None
        if f.alignment_status != 'VALID':
            self.issue_block = 'TARGET_ALIGNMENT_NOT_VALID'
            return None
        if geo[0] != (goal.map_session_id, goal.map_revision, goal.transform_revision):
            self.issue_block = 'ROUTE_BASIS_MISMATCH'
            return None
        return f

    def _route(self, fields, goal, frame, route_id, segment_index, *, route_revision=None, generated_at_ns=None):
        return dict(route_id=route_id,
                    route_revision=self.route_revision if route_revision is None else route_revision,
                    robot_id=fields['robot_id'], target_id=fields['target_id'], segment_index=segment_index,
                    approach_goal=goal, frame_ref=frame,
                    generated_at_ns=(self.route_made_at[fields['target_id']] if generated_at_ns is None
                                     else generated_at_ns),
                    valid_until_ns=fields['deadline_ns'])

    def _approach_target(self):
        """(target_id, revision) the automatic dispatch serves now, or None."""
        mission = self.mission
        if mission.dispatch_kind == 'APPROACH':
            if mission.approach_phase == 'FAILED':
                self.issue_block = f'APPROACH_FAILED_{self.approach_failure}'  # shown, never retried by itself
            if mission.approach_phase != 'ISSUED' or self.approach_task is not None:
                return None
            return mission.dispatch_target, mission.dispatch_target_revision
        if mission.dispatch_target is not None:
            return None
        # Targets in the order they were found (Phase 1: one SUPPORT dispatch per mission) [제안].
        for target_id in sorted(self.route_made_at, key=lambda t: (self.route_made_at[t], t)):
            rev, confirmed, persisted = mission.targets[target_id]
            if confirmed and persisted and mission.verdicts.get(target_id) != 'RESCUE_CONFIRMED':
                return target_id, rev
        return None

    def _issue_support_exit(self, now_ns):
        """Q1-b: support_exit_wait_ns after TB-5 arrived (COMPLETED, stop confirmed), one RETURN_HOME to its exit.

        The robot holds the exit pose (home_ref ``home/<robot_id>``, 5A). Only the end of disaster mode cancels
        the exit: a RESCUE_CONFIRMED target does not, TB-5 still leads out and ends at the exit (5A; 작전
        2026-10-13: after the verdict the TB-6 return and the TB-5 exit both follow). A failed approach never
        reaches here (ARRIVED only).
        """
        mission = self.mission
        if (mission.dispatch_kind != 'APPROACH' or mission.approach_phase != 'ARRIVED'
                or self.support_exit not in (None, 'WAITING')):
            return None
        if mission.mode != 'DISASTER':
            self.support_exit = 'CANCELED'
            return None
        if self.support_exit is None:
            self.support_exit, self.exit_due_ns = 'WAITING', now_ns + self.support_exit_wait_ns
        if now_ns < self.exit_due_ns or self._held_after_refusal('SUPPORT_EXIT', now_ns):
            return None
        if not self._home_free('SUPPORT', 'SUPPORT_EXIT_NOT_FREE'):
            return None
        boot = self._free('SUPPORT', now_ns)
        if boot is None:
            return None
        robot_id = self.roles['SUPPORT']
        decision = self.search.reserve_goal_task(robot_id, robot_boot_id=boot, kind='RETURN_HOME',
                                                 target_id=self.new_id(), target_revision=0, goal=None, now_ns=now_ns)
        if decision.action != 'RESERVED':
            return None
        fields = decision.task_fields | dict(route=None, approval=None, home_ref=f'home/{robot_id}')
        self.task_meta[fields['task_id']] = ('RETURN_HOME', 'SUPPORT_EXIT')
        self.support_exit = 'ISSUED'
        self._send(fields)
        return fields['task_id']

    # -- recheck -> verdict -> TB-6 return (C-11 rows 5, 7-11; owner decision 2026-10-13 4A·5A) ------------
    def _open_camera_recheck(self, event):
        """Row 5: an accepted candidate opens a CAMERA recheck of its target, unless one is already open (row 10:
        an open operator recheck keeps its link; the candidate is recorded only) or the target is judged."""
        mission, target_id = self.mission, event.target_id
        if mission.verdicts.get(target_id) == 'RESCUE_CONFIRMED' or target_id in mission.open_rechecks:
            return False
        mission.open_rechecks[target_id] = RecheckRequest(event.event_id, target_id, 'CAMERA', event.observed_at_ns)
        return True

    def _judge(self, task_id, link_id, observation):
        """Judge one completed REOBSERVE_TARGET with its RecheckObservation and close its recheck."""
        mission = self.mission
        recheck = next((r for r in mission.open_rechecks.values() if r.link_id == link_id), None)
        if recheck is None:
            return None  # closed meanwhile (judged by another path): nothing to record
        if recheck.source == 'CAMERA':
            event = mission.events[link_id][0]
            reference = Reference('CAMERA', event.observed_at_ns, event.clock_domain_id, event.clock_error_bound_ns,
                                  event.time_basis_declared)
        else:  # row 11 needs this coordinator's measured clock, not measured on this path
            reference = Reference('OPERATOR', recheck.accepted_at_ns, None, None, False)
        if observation is None:
            state, reason = 'RECHECK_NEEDED', 'RECHECK_OBSERVATION_MISSING'
        else:
            state, reason = judge(observation, reference, expected_task_id=task_id, target_id=recheck.target_id,
                                  scout_id=self.roles['SCOUT'], basis=self.basis(),
                                  mission_frame=self.search.policy.mission_frame_id,
                                  max_clock_error_ns=self.recheck_max_clock_error_ns)
        mission.record_rescue_verdict(recheck.target_id, state)
        self.verdict_reason = None if state == 'RESCUE_CONFIRMED' else f'RECHECK_NEEDED_{reason}'
        self.notes.append(f'VERDICT target={recheck.target_id} state={state} reason={reason} source={recheck.source} '
                          f'link={link_id} task={task_id}')
        return state

    def drain_notes(self):
        notes, self.notes = self.notes, []
        return notes

    def _issue_scout_return(self, now_ns):
        """5A: after a RESCUE_CONFIRMED, TB-6 goes once to its docking approach point (home_ref home/<robot_id>)."""
        mission = self.mission
        if (self.scout_return is not None or 'RESCUE_CONFIRMED' not in mission.verdicts.values()
                or mission.mode != 'DISASTER' or self._held_after_refusal('SCOUT_RETURN', now_ns)):
            return None
        if not self._home_free('SCOUT', 'SCOUT_HOME_NOT_FREE'):
            return None
        boot = self._free('SCOUT', now_ns)
        if boot is None:
            return None
        robot_id = self.roles['SCOUT']
        decision = self.search.reserve_goal_task(robot_id, robot_boot_id=boot, kind='RETURN_HOME',
                                                 target_id=self.new_id(), target_revision=0, goal=None, now_ns=now_ns)
        if decision.action != 'RESERVED':
            return None
        fields = decision.task_fields | dict(route=None, approval=None, home_ref=f'home/{robot_id}')
        self.task_meta[fields['task_id']] = ('RETURN_HOME', 'SCOUT_RETURN')
        self.scout_return = 'ISSUED'
        mission.returning = True
        for robot in self.at_home:  # TB-5 may already be at its exit
            mission.record_returned(robot)
        self._send(fields)
        return fields['task_id']

    def note_support_at_exit(self):
        """TB-5 reached its exit (5A); it counts toward FINISHED once the mission is returning."""
        self._returned(self.roles['SUPPORT'])

    def _returned(self, robot_id):
        self.at_home.add(robot_id)
        if self.mission.returning:
            self.mission.record_returned(robot_id)

    def mission_reason(self, now_ns):
        """The latest stage after dispatch for SystemState.reason: TB-6 return, the verdict, TB-5's exit."""
        scout = {'ISSUED': 'SCOUT_RETURNING', 'MOVING': 'SCOUT_RETURNING',
                 'AT_DOCK_APPROACH': 'SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND', 'FAILED': 'SCOUT_RETURN_FAILED'}
        return scout.get(self.scout_return) or self.verdict_reason or self.exit_reason(now_ns)

    def exit_reason(self, now_ns):
        """SystemState.reason for TB-5's exit: the countdown in whole seconds (rounded up), then its progress."""
        if self.support_exit == 'WAITING':
            remaining = max(0, self.exit_due_ns - now_ns)
            return f'SUPPORT_EXIT_IN_{-(-remaining // 1_000_000_000)}S'
        return {'ISSUED': 'SUPPORT_EXITING', 'MOVING': 'SUPPORT_EXITING', 'AT_EXIT': 'SUPPORT_AT_EXIT',
                'CANCELED': 'SUPPORT_EXIT_CANCELED', 'FAILED': 'SUPPORT_EXIT_FAILED'}.get(self.support_exit, '')

    def _support_xy(self, now_ns):
        """The SUPPORT's mission xy when its pose is fresh, VALID and on the coordinator's basis, else None."""
        pose = self.support_pose
        if pose is None or pose[3] != 'VALID' or not (
                pose[4] <= now_ns and now_ns - pose[4] <= self.support_pose_max_age_ns):
            self.issue_block = 'SUPPORT_POSE_UNKNOWN'
            return None
        if pose[1] != self.basis() or pose[2] != self.search.policy.mission_frame_id:
            self.issue_block = 'SUPPORT_POSE_BASIS_MISMATCH'
            return None
        return pose[0]

    def _issue_approach(self, now_ns):
        """AUTO: one APPROACH_TARGET to the standoff point of the saved target (owner decision 2026-10-13)."""
        if self.support_dispatch != 'AUTO' or self.mission.mode != 'DISASTER':
            return None
        chosen = self._approach_target()
        if chosen is None:
            return None
        target_id, target_revision = chosen
        if self._held_after_refusal(target_id, now_ns):  # refused before moving: wait, then a new task
            return None
        if not self._frozen_gate(target_id):
            return None
        state = self.ready(self.roles['SUPPORT'], now_ns)
        if state is None or not state[1]:
            # No fresh READY status: TB-5 reports READY only when armed (rescue_robot task_status.py:17).
            self.issue_block = 'SUPPORT_NOT_READY'
            return None
        boot = self._free('SUPPORT', now_ns)
        if boot is None:
            return None
        support_xy = self._support_xy(now_ns)
        if support_xy is None:
            return None
        point = approach_point(self.target_geo[target_id][1], support_xy, self.approach_standoff_m)
        if point is None:
            self.issue_block = 'SUPPORT_AT_TARGET'
            return None
        if not self._free_cell(point[0], 'SUPPORT_APPROACH_NOT_FREE'):
            return None
        goal = GoalPoint(point[0], point[1], *self.basis())
        frame = self._route_frame(target_id, goal)
        if frame is None:
            return None
        if self.mission.dispatch_kind is None:
            reason = self.mission.start_approach(target_id, target_revision)
            if reason != 'ACCEPTED':
                self.issue_block = reason
                return None
        decision = self.search.reserve_goal_task(
            self.roles['SUPPORT'], robot_boot_id=boot, kind='APPROACH_TARGET', target_id=target_id,
            target_revision=target_revision, goal=goal, now_ns=now_ns)
        if decision.action != 'RESERVED':
            return None
        fields = decision.task_fields
        fields = fields | dict(route=self._route(fields, goal, frame, route_id((goal,)), 0,
                                                 route_revision=target_revision, generated_at_ns=now_ns),
                               approval=None)
        self.approach_task = fields['task_id']
        self.task_meta[fields['task_id']] = ('APPROACH_TARGET', target_id)
        self._send(fields)
        return fields['task_id']

    def _held_after_refusal(self, key, now_ns):
        """True while a refused approval/recheck waits admission_timeout_ns before it is issued again."""
        if key not in self.refused:
            return False
        if self.refused[key] is None:
            self.refused[key] = now_ns + self.admission_timeout_ns
        if now_ns <= self.refused[key]:
            self.issue_block = 'TASK_REFUSED'
            return True
        del self.refused[key]
        return False

    def _issue_segment(self, now_ns):
        approval = self.mission.approval
        if (approval is None or self.mission.segment_phase != 'APPROVED'
                or approval.command_id in self.segment_tasks or approval.route_revision != self.route_revision):
            return self.segment_tasks.get(approval.command_id) if approval else None
        if self._held_after_refusal(approval.command_id, now_ns):
            return None
        if not self._passage_free(approval.segment_index):
            self.issue_block = 'PASSAGE_ASSESSMENT_REQUIRED'  # Q-25 default: hold, never guess
            return None
        boot = self._free('SUPPORT', now_ns)
        if boot is None:
            return None
        goal = self._on_basis(self.segment_goals[approval.segment_index])
        frame = self._route_frame(approval.target_id, goal)
        if frame is None:
            return None
        decision = self.search.reserve_goal_task(
            self.roles['SUPPORT'], robot_boot_id=boot, kind='MOVE_SEGMENT', target_id=approval.target_id,
            target_revision=approval.target_revision, goal=goal, approval_id=approval.command_id, now_ns=now_ns)
        if decision.action != 'RESERVED':
            return None
        fields = decision.task_fields
        command = self.mission.commands[approval.command_id][0]
        fields = fields | dict(
            route=self._route(fields, goal, frame, self.route_id, approval.segment_index),
            approval=dict(approval_id=approval.command_id, command_id=approval.command_id,
                          operator_id=command.operator_id, robot_id=fields['robot_id'], robot_boot_id=boot,
                          authority_epoch=fields['authority_epoch'], target_id=approval.target_id,
                          target_estimate_revision=approval.target_revision, route_id=self.route_id,
                          route_revision=approval.route_revision, segment_index=approval.segment_index,
                          approved_at_ns=self.accepted_at[approval.command_id], scope='ONE_SEGMENT',
                          valid_until_ns=fields['deadline_ns']))
        self.segment_tasks[approval.command_id] = fields['task_id']
        self.task_meta[fields['task_id']] = ('MOVE_SEGMENT', approval)
        self._send(fields)
        return fields['task_id']

    def _issue_rechecks(self, now_ns):
        for target_id, recheck in self.mission.open_rechecks.items():
            if recheck.link_id in self.recheck_tasks or self._held_after_refusal(recheck.link_id, now_ns):
                continue
            boot = self._free('SCOUT', now_ns)
            goal = self._on_basis(self.observe_goal)
            frame = self._route_frame(target_id, goal)
            if boot is None or frame is None:
                return
            decision = self.search.reserve_goal_task(
                self.roles['SCOUT'], robot_boot_id=boot, kind='REOBSERVE_TARGET', target_id=target_id,
                target_revision=self.mission.targets[target_id][0], goal=goal, now_ns=now_ns)
            if decision.action != 'RESERVED':
                return
            fields = decision.task_fields
            # REOBSERVE: the observation point as route, no operator approval (C-10, #88).
            fields = fields | dict(route=self._route(fields, goal, frame, self.observe_route_id, 0),
                                   approval=None)
            self.recheck_tasks[recheck.link_id] = fields['task_id']
            self.task_meta[fields['task_id']] = ('REOBSERVE_TARGET', recheck.link_id)
            self._send(fields)

    def _issue_search(self, now_ns):
        scout = self.roles['SCOUT']
        if self.map_mode == 'SLAM_FREEZE' and self._target_saved():
            return  # TB-6 stops for the doll's freeze (Q-29 ⑦); exploring on afterwards comes with OPS-154
        if self.patrol_exhausted:
            return
        state = self.ready(scout, now_ns)
        if state is None:
            return
        gate = self.mission.plan(scout, robot_boot_id=state[0], ready=state[1], now_ns=now_ns)
        if gate.decision is not None and gate.decision.action == 'RESERVED':
            self._send(gate.decision.task_fields | dict(route=None, approval=None))

    def _send(self, fields):
        """Queue task fields with a fresh lease_id and start renewing that lease."""
        lease_id = self.new_id()
        fields = fields | dict(lease_id=lease_id)
        self.issued_at[fields['task_id']] = fields['issued_at_ns']
        self.leases[fields['task_id']] = dict(
            lease_id=lease_id, robot_id=fields['robot_id'], robot_boot_id=fields['robot_boot_id'],
            authority_epoch=fields['authority_epoch'], generation=fields['generation'], renewal_sequence=0,
            next_ns=None, admitted_after=None)
        self.outbox.append(fields)

    def leases_due(self, now_ns):
        """CommandLease renewals to publish now (작전 2026-10-11).

        Renewal stops for good when the task left the Allocator (result, refusal),
        the mode is not DISASTER, the robot's status is stale or from another boot,
        or the reservation's authority/generation differs. The executor then stops
        itself with LEASE_EXPIRED; nothing extends a lease silently.
        """
        contract_value(timestamp_ns, now_ns)
        self._expire_unadmitted(now_ns)
        allocator, due = self.search.allocator, []
        for task_id, lease in list(self.leases.items()):
            item = allocator.active.get(task_id) or allocator.pending.get(task_id)
            state = self.status.get(lease['robot_id'])
            fresh = (state is not None and state[3] <= now_ns and now_ns - state[3] <= self.status_max_age_ns
                     and state[0] == lease['robot_boot_id'])
            if (item is None or self.mission.mode != 'DISASTER' or not fresh or self._robot_stopped(task_id, lease)
                    or (item.authority_epoch, item.generation) != (lease['authority_epoch'], lease['generation'])):
                del self.leases[task_id]
                continue
            if lease['next_ns'] is not None and now_ns < lease['next_ns']:
                continue
            lease['renewal_sequence'] += 1
            lease['next_ns'] = now_ns + self.lease_renew_period_ns
            due.append(dict(lease_id=lease['lease_id'], robot_id=lease['robot_id'],
                            robot_boot_id=lease['robot_boot_id'], authority_epoch=lease['authority_epoch'],
                            generation=lease['generation'], renewal_sequence=lease['renewal_sequence'],
                            issued_at_ns=now_ns, valid_until_ns=now_ns + self.lease_message_ttl_ns))
        return due

    def _expire_unadmitted(self, now_ns):
        """Admission timeout (작전 2026-10-11): a task still PENDING after ``admission_timeout_ns``.

        Its lease is never renewed again. The reservation is released only on stop
        evidence: a RobotStatus that arrived after the timeout and shows the robot
        STOPPED and not running this task (the admission may have been lost while
        the robot started). Released search areas and rechecks are issued again.
        """
        allocator = self.search.allocator
        for task_id, item in list(allocator.pending.items()):
            issued = self.issued_at.get(task_id)
            if issued is None or now_ns - issued <= self.admission_timeout_ns:
                continue
            if task_id not in self.timed_out:
                self.timed_out[task_id] = self.status_arrivals
                self.leases.pop(task_id, None)
            detail = self.status_detail.get(item.robot_id)
            if (detail is not None and detail[0] > self.timed_out[task_id] and detail[2] == 'STOPPED'
                    and detail[4] != task_id and self.search.finish(item, outcome='FAILED', stop_confirmed=True)):
                del self.timed_out[task_id]
                if task_id in self.task_meta:
                    self._forget(task_id)
        if self.timed_out:
            self.issue_block = 'ADMISSION_TIMEOUT'  # held until stop evidence releases the task

    def _rebase_reason(self):
        """TARGET_REBASED_ON_FREEZE while a target re-based at the freeze waits for that revision to be saved."""
        return 'TARGET_REBASED_ON_FREEZE' if any(
            not self.mission.targets[t][2] for t in self.rebased if t in self.mission.targets) else ''
    def _search_wait_reason(self):
        """SEARCH_QUEUED while an accepted start waits; SEARCH_GOAL_NOT_SELECTED when there is no area to send
        (the RViz pick is still missing, live_task_inputs.SelectedSearchGoal)."""
        if not self.search_queued():
            return ''
        return 'SEARCH_QUEUED' if self.search.policy.areas else 'SEARCH_GOAL_NOT_SELECTED'

    def search_queued(self):
        """START_SEARCH accepted but the first area not reserved yet (the SCOUT is busy); shown as SEARCH_QUEUED."""
        mission = self.mission
        return mission.mode == 'DISASTER' and mission.search_authorized and not mission.search_started

    def _cancel_queued_search(self, why):
        """A stop or the end of disaster mode withdraws a waiting search (작전 2026-10-11, #99)."""
        if self.mission.cancel_search_start():
            self.last_reason = f'SEARCH_START_CANCELED:{why}'

    def _robot_stopped(self, task_id, lease):
        """Supporting rule (작전 2026-10-11): never extend a stopped robot's lease; the stop is the executor latch.

        A status naming this task with a stop reason, a stopping motion or an
        invalid lease stops renewal; so does a status that arrived after the
        admission and names another task. Statuses before the admission (no
        active task yet, or a previous task's stop reason) do not cut a new lease.
        """
        detail = self.status_detail.get(lease['robot_id'])
        if detail is None:
            return False
        arrival, stop_reason, motion_state, lease_valid, active_task_id = detail
        if active_task_id == task_id:
            return bool(stop_reason) or motion_state in STOPPING_MOTION or not lease_valid
        return lease['admitted_after'] is not None and arrival > lease['admitted_after']

    def _forget(self, task_id):
        kind, key = self.task_meta.pop(task_id)
        if kind == 'APPROACH_TARGET':
            self.approach_task = None  # refused or timed out before moving: issued again on a later tick
            return
        if kind == 'RETURN_HOME':
            if key == 'SUPPORT_EXIT':
                self.support_exit = 'WAITING'  # the wait is over; issued again after the refusal hold
            else:
                self.scout_return = None
            return
        table = self.segment_tasks if kind == 'MOVE_SEGMENT' else self.recheck_tasks
        table.pop(key.command_id if kind == 'MOVE_SEGMENT' else key, None)

    @staticmethod
    def _reply(accepted, state, reason_code, created_task_id=''):
        return dict(accepted=accepted, request_state=state, reason_code=reason_code,
                    created_task_id=created_task_id)


def spin_until_shutdown(spin, context_ok, shutdown_error):
    """Run ``spin``; a ``shutdown_error`` raised after the context closed is the shutdown race, a normal stop.

    On SIGTERM rclpy shuts the context down, and a spin caught between callbacks raises
    RCLError("the given context is not valid") instead of ExternalShutdownException (A·B rehearsal
    2026-10-13: 2 of 3 runs ended with a traceback and exit 1). The same error while the context
    is valid, and every other error, is raised.
    """
    try:
        spin()
    except shutdown_error:
        if context_ok():
            raise


def _send_targets(core, out):
    targets = core.target_snapshot_change()
    if targets is not None:
        out.targets(targets)


def node_tick(core, now_ns, out):
    """One timer tick of the node, in send order; ``out`` turns each item into a ROS message.

    Changed targets go out before the goals (#179, 2026-10-13): a goal names its target revision, and the
    robot looks that revision up in the latched /rescue/targets snapshot. They go again after the goals, for
    the approach states those goals changed; ``target_snapshot_change`` is None when nothing changed.
    """
    _send_targets(core, out)
    for fields in core.tick(now_ns):
        out.task(fields)
    if core.take_freeze_request():
        out.freeze()
    for line in core.drain_notes():
        out.note(line)
    for fields in core.watch_requests(now_ns):
        out.watch(fields)
    for report in core.reports_due(now_ns):
        out.report(report)
    for lease in core.leases_due(now_ns):
        out.lease(lease)
    out.block(core.issue_block)
    _send_targets(core, out)
    out.state(now_ns)


class FrozenGrid:
    """The frozen OccupancyGrid for the core: its occupancy hash and the shared free-footprint rule.

    Both come from rescue_contracts.occupancy (통합-로봇 #208), the same functions the robot hashes and checks its
    goals with; ``planar_origin`` reads the origin pose as the robot does. An invalid grid raises ValueError.
    """

    def __init__(self, msg):
        from rescue_contracts.occupancy import occupancy_sha256, planar_origin
        info = msg.info
        p, q = info.origin.position, info.origin.orientation
        self.fields = dict(frame=msg.header.frame_id, width=int(info.width), height=int(info.height),
                           resolution=float(info.resolution), origin=planar_origin(p.x, p.y, p.z, q.x, q.y, q.z, q.w),
                           data=[int(v) for v in msg.data])
        self.sha256 = occupancy_sha256(**self.fields)

    def clear(self, x, y, radius):
        from rescue_contracts.occupancy import footprint_clear
        return footprint_clear(**self.fields, x=x, y=y, radius=radius)


# ---- ROS wiring (needs a sourced ROS 2 environment) --------------------------------------------
def declare_required(node, names, optional=(), unset_ok=()):
    """Typed parameters without defaults -> (values, missing). Jazzy rejects a typed
    ``declare_parameter`` with no value, so the type goes in as ``Parameter.Type`` (#35).
    ``optional`` may be empty; ``unset_ok`` may also be left out (None), e.g. an array a CLI cannot type empty."""
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    values = {}
    for name, kind in names.items():
        node.declare_parameter(name, Parameter.Type(kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, item in values.items()
                     if (item is None and name not in unset_ok) or (name not in optional and item in ('', [])))
    return values, missing


def build_core(values, *, map_snapshot_id):
    """Profile v2 + typed parameters -> CoordinatorCore (no defaults anywhere).

    ``map_snapshot_id(sha256, session, revision, transform_revision)``: the MapState snapshot id rule the robot
    uses (rescue_contracts, 통합-로봇); the node passes it, tests pass their own.
    """
    from rescue_bringup.profile import load_deployment
    deployment = load_deployment(values['profile_path'])
    if deployment.mission is None:
        raise CoordinationError('INVALID_POLICY', 'profile v2 mission section required')
    areas = deployment.mission.search_areas
    yaws = list(values['entry_yaws'])
    if len(yaws) != len(areas):
        raise CoordinationError('INVALID_POLICY', 'entry_yaws must give one heading per search area, in order')
    sites = mission_sites(areas, entry_yaws={area.id: yaw for area, yaw in zip(areas, yaws)})
    policy = SearchPolicy(version=values['policy_version'], roles=deployment.mission.roles, areas=sites,
                          executable_bases=frozenset(values['executable_bases']),
                          task_duration_ns=seconds_to_ns(str(values['task_duration_s'])),
                          lease_duration=values['lease_duration_s'],
                          executor_policy_version=values['executor_policy_version'],
                          mission_frame_id=values['mission_frame_id'])
    robot_ids = tuple(robot for robot, _ in deployment.mission.roles)
    search = SearchCoordinator(Allocator(values['authority_epoch'], allowed_robot_ids=robot_ids), policy)
    # home_poses: TB-6 (SCOUT) docking approach point, TB-5 (SUPPORT) exit; launch fills it from the profile
    # (owner decision 2026-10-13, 5A). Left out = no home: RETURN_HOME stays NO_HOME_POSITION.
    basis = (values['goal_map_session_id'], values['goal_map_revision'], values['goal_transform_revision'])
    roles = {role: robot_id for robot_id, role in deployment.mission.roles}
    mission = MissionCoordinator(search, home_positions=home_positions(values['home_poses'], roles, basis))
    cameras = [c.id for c in deployment.site_cameras if 'TARGET_WATCH' in c.roles]
    if len(cameras) > 1:
        raise CoordinationError('INVALID_POLICY', 'SET_CAMERA_WATCH needs exactly one TARGET_WATCH camera')
    roi = list(values['camera_watch_roi'] or [])
    camera_watch = CameraWatchBook(
        cameras[0], roi=tuple(roi) if roi else None,
        request_timeout_ns=seconds_to_ns(str(values['camera_watch_timeout_s'])),
        new_id=lambda: str(uuid4())) if cameras else None
    flat = list(values['route_segment_goals'])
    if len(flat) % 3 or len(values['observe_goal']) != 3:
        raise CoordinationError('INVALID_POLICY', 'goals are [x, y, yaw] triples in the mission frame')
    segments = tuple(GoalPoint((flat[i], flat[i + 1]), flat[i + 2], *basis) for i in range(0, len(flat), 3))
    x, y, yaw = values['observe_goal']
    return CoordinatorCore(mission, status_max_age_ns=seconds_to_ns(str(values['status_max_age_s'])),
                           segment_goals=segments, route_revision=values['route_revision'],
                           observe_goal=GoalPoint((x, y), yaw, *basis), merge_distance_m=values['merge_distance_m'],
                           report_retry_ns=seconds_to_ns(str(values['report_retry_s'])),
                           lease_renew_period_ns=seconds_to_ns(str(values['lease_renew_period_s'])),
                           admission_timeout_ns=seconds_to_ns(str(values['admission_timeout_s'])),
                           segment_passage_free=tuple(values['route_segment_passage_free']),
                           lease_message_ttl_ns=seconds_to_ns(str(values['lease_message_ttl_s'])),
                           camera_watch=camera_watch,
                           target_clock_domain_id=values['target_clock_domain_id'],
                           target_max_age_ns=seconds_to_ns(str(values['target_max_age_s'])),
                           target_max_future_ns=seconds_to_ns(str(values['target_max_future_s'])),
                           support_dispatch=values['support_dispatch'],
                           approach_standoff_m=values['approach_standoff_m'],
                           support_pose_max_age_ns=seconds_to_ns(str(values['support_pose_max_age_s'])),
                           support_exit_wait_ns=seconds_to_ns(str(values['support_exit_wait_s'])),
                           recheck_max_clock_error_ns=seconds_to_ns(str(values['recheck_max_clock_error_s'])),
                           map_mode=values['map_mode'], freeze_clearance_m=values['freeze_clearance_m'],
                           map_freeze_timeout_ns=seconds_to_ns(str(values['map_freeze_timeout_s'])),
                           map_snapshot_id=map_snapshot_id, search_start=values['search_start'],
                           search_mode=values['search_mode'],
                           patrol_points=patrol_points(values['patrol_points']),
                           patrol_rounds=values['patrol_rounds'], exit_standoff_m=values['exit_standoff_m'])


def patrol_points(flat):
    """``patrol_points`` [x, y, yaw]*N in TB-6's start pose -> tuple of triples. Left out = none (AREAS)."""
    flat = list(flat or [])
    if len(flat) % 3:
        raise CoordinationError('INVALID_POLICY', 'patrol_points must be [x, y, yaw] triples')
    return tuple(tuple(flat[i:i + 3]) for i in range(0, len(flat), 3))


def home_positions(flat, roles, basis):
    """``home_poses`` [x, y, yaw] per role robot (SCOUT, then SUPPORT) -> HomePosition tuple.

    Same mission frame and map basis as the other configured goals (goal_map_*). Left out or
    empty = no home: RETURN_HOME stays NO_HOME_POSITION. Any other length is refused at start.
    """
    flat = list(flat or [])
    if not flat:
        return ()
    order = [roles[role] for role in ('SCOUT', 'SUPPORT')]
    if len(flat) != 3 * len(order):
        raise CoordinationError('INVALID_POLICY', 'home_poses must be [x, y, yaw] for the SCOUT then the SUPPORT')
    return tuple(HomePosition(robot_id, (flat[3 * i], flat[3 * i + 1]), flat[3 * i + 2], *basis)
                 for i, robot_id in enumerate(order))


def camera_watch_service(camera_id):
    """IF-078 service name of a fixed camera (rescue_interfaces README)."""
    return f'/rescue/cameras/{camera_id}/set_watch'


PARAMETERS = {  # name -> rcl_interfaces ParameterType name; every one is required, none has a default
    'profile_path': 'STRING', 'entry_yaws': 'DOUBLE_ARRAY', 'policy_version': 'STRING',
    'executable_bases': 'STRING_ARRAY', 'task_duration_s': 'DOUBLE', 'lease_duration_s': 'DOUBLE',
    'executor_policy_version': 'STRING', 'mission_frame_id': 'STRING', 'authority_epoch': 'STRING',
    'coordinator_id': 'STRING', 'envelope_schema_version': 'STRING', 'status_max_age_s': 'DOUBLE',
    'tick_period_s': 'DOUBLE',
    # Phase 1 rule (#88): fixed segment ends [x, y, yaw]*N, the observation point [x, y, yaw], their map basis.
    'route_segment_goals': 'DOUBLE_ARRAY', 'route_revision': 'INTEGER', 'observe_goal': 'DOUBLE_ARRAY',
    'goal_map_session_id': 'STRING', 'goal_map_revision': 'INTEGER', 'goal_transform_revision': 'INTEGER',
    'merge_distance_m': 'DOUBLE', 'report_retry_s': 'DOUBLE',  # Q-21 merge distance; RescueReport resend period
    # CommandLease renewal period; at most half of lease_duration_s, which must equal the executor's
    # lease_duration (C-10 「명령 lease」, #88). Checked at start: otherwise FATAL exit 2.
    'lease_renew_period_s': 'DOUBLE',
    'admission_timeout_s': 'DOUBLE',  # a task still PENDING after this stops its lease (ACCEPT_TIMEOUT)
    'lease_message_ttl_s': 'DOUBLE',  # CommandLease.valid_until_ns = issued_at_ns + this (AST88-02)
    # AST88-01: one field-checked bool per segment end. A ROS parameter cannot carry an empty array (-p and
    # --params-file both arrive unset = missing, measured 2026-10-13); to hold every segment give all false.
    'route_segment_passage_free': 'BOOL_ARRAY',
    # SetCameraWatch (C-11 row 2) request deadline. The deployment ROI camera_watch_roi [x0, y0, x1, y1]
    # (normalized, used when the operator's request carries none; never the full frame) is in UNSET_OK_PARAMETERS.
    'camera_watch_timeout_s': 'DOUBLE',
    # OPS-103 (#142): a TargetLocation counts only on this clock domain, at most this old, and at most this far
    # ahead beyond its own declared clock error bound.
    'target_clock_domain_id': 'STRING', 'target_max_age_s': 'DOUBLE', 'target_max_future_s': 'DOUBLE',
    # TB-5 (SUPPORT) dispatch (owner decision 2026-10-13, 1B·2B): AUTO | OPERATOR; standoff from the target; max age of
    # the SUPPORT RobotPose used for the approach direction.
    'support_dispatch': 'STRING', 'approach_standoff_m': 'DOUBLE', 'support_pose_max_age_s': 'DOUBLE',
    # Q1-b: TB-5 waits this long at the target after arriving, then goes to its exit (RETURN_HOME).
    'support_exit_wait_s': 'DOUBLE',
    # C-11 row 8: the candidate + recheck clock error sum must be at most this for a RESCUE_CONFIRMED.
    'recheck_max_clock_error_s': 'DOUBLE',
    # Map (owner decision 2026-10-13, Q-29 ⑦): SLAM_FREEZE | SAVED_MAP; free radius around the approach point and
    # the homes on the frozen map (= the robots' goal_clear_radius_m); wait for the frozen map after the request.
    'map_mode': 'STRING', 'freeze_clearance_m': 'DOUBLE', 'map_freeze_timeout_s': 'DOUBLE',
    # TB-6 search start (owner decision 2026-10-13 evening, OPS-136): AUTO (the disaster sign) | OPERATOR (button).
    'search_start': 'STRING',
    # TB-6 search (OPS-136·139): PATROL | AREAS; rounds of the patrol points; TB-5's exit approach standoff.
    'search_mode': 'STRING', 'patrol_rounds': 'INTEGER', 'exit_standoff_m': 'DOUBLE'}
OPTIONAL_PARAMETERS = ('route_segment_passage_free',)
# Declared beside PARAMETERS (which launch passes in full) because they may be left out = none.
# camera_watch_roi: no deployment ROI, only the operator's. home_poses: [x, y, yaw] for the SCOUT then the SUPPORT
# on the goal_map_* basis (TB-6 docking approach point, TB-5 exit); none = RETURN_HOME refused.
# patrol_points: [x, y, yaw]*N in TB-6's start pose (search_mode PATROL); none with AREAS.
UNSET_OK_PARAMETERS = {'camera_watch_roi': 'DOUBLE_ARRAY', 'home_poses': 'DOUBLE_ARRAY',
                       'patrol_points': 'DOUBLE_ARRAY'}


def main(args=None):
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterType
    from rescue_interfaces.action import ExecuteTask
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import (CommandLease, MapState, ReportReceipt, RescueReport, RobotPose, RobotStatus,
                                       SiteEvent as SiteEventMsg, SystemState, TargetEstimate, TargetLocation,
                                       TargetSnapshot, TaskAdmission as AdmissionMsg)
    from rescue_interfaces.srv import SetCameraWatch, SubmitOperatorCommand

    rclpy.init(args=args)
    node = Node('mission_coordinator')
    log = node.get_logger()
    live_task_source = None
    try:
        values, missing = declare_required(
            node, {name: getattr(ParameterType, 'PARAMETER_' + kind)
                   for name, kind in (PARAMETERS | UNSET_OK_PARAMETERS).items()},
            optional=OPTIONAL_PARAMETERS + tuple(UNSET_OK_PARAMETERS), unset_ok=tuple(UNSET_OK_PARAMETERS))
        if missing:
            log.fatal(f'missing parameters: {missing}')
            raise SystemExit(2)
        try:
            from rescue_contracts.occupancy import map_snapshot_id
            core = build_core(values, map_snapshot_id=map_snapshot_id)
        except (RescueError, OSError, ValueError) as exc:
            log.fatal(f'configuration rejected: {exc}')
            raise SystemExit(2)
        boot_id, sequence = str(uuid4()), iter(range(1, 2 ** 63))
        from rescue_coordination.live_task_inputs import attach as attach_live_task_inputs
        live_task_source = attach_live_task_inputs(node,core,values,boot_id)
        log.info(f'git_sha={git_sha()} boot_id={boot_id} parameters={values} roles={core.roles}')

        def now_ns():
            return node.get_clock().now().nanoseconds

        def envelope(msg):
            """Fill ``msg.envelope`` (or ``msg`` itself when it is an Envelope) and return ``msg``."""
            env = getattr(msg, 'envelope', msg)
            env.schema_version = values['envelope_schema_version']
            env.publisher_id = values['coordinator_id']
            env.publisher_boot_id = boot_id
            env.sequence = next(sequence)
            env.published_at_ns = now_ns()
            return msg

        def guarded(label, callback):
            def run(*args):
                try:
                    return callback(*args)
                except RescueError as exc:  # visible, never a silent fallback (AGENTS 6)
                    log.error(f'{label}: {exc}')
            return run

        clients = {robot: ActionClient(node, ExecuteTask, ROBOT_PREFIX + robot + '/execute_task')
                   for robot in core.roles.values()}
        watch = core.camera_watch
        watch_client = node.create_client(SetCameraWatch, camera_watch_service(watch.camera_id)) if watch else None
        log.info(f'camera watch: {camera_watch_service(watch.camera_id) if watch else "none (no TARGET_WATCH camera)"}'
                 f' roi={watch.roi if watch else None}')

        def send_watch(fields):
            req = SetCameraWatch.Request(**{k: (list(v) if k == 'roi_xyxy_normalized' else v)
                                            for k, v in fields.items()})
            envelope(req.envelope)
            if not watch_client.service_is_ready():
                log.error(f'{camera_watch_service(watch.camera_id)} not ready; watch {fields["watch_id"]} '
                          'is dropped at its deadline unless the camera answers')
            log.info(f'SET_WATCH enable={fields["enable"]} watch={fields["watch_id"]} target={fields["target_id"]} '
                     f'roi={fields["roi_xyxy_normalized"]} expected_revision={fields["expected_watch_revision"]}')
            watch_client.call_async(req).add_done_callback(guarded('watch_reply', on_watch_reply))

        def on_watch_reply(future):
            res = future.result()
            recorded = core.on_watch_reply(watch_id=res.watch_id, accepted=bool(res.accepted),
                                           watch_revision=int(res.watch_revision), reason_code=res.reason_code)
            (log.info if res.accepted else log.error)(
                f'WATCH_REPLY {res.watch_id} accepted={res.accepted} revision={res.watch_revision} '
                f'reason={res.reason_code} recorded={recorded} active={core.camera_watch.active}')

        def send(fields):
            goal = fill_execute_task_goal(ExecuteTask.Goal(), fields, coordinator_id=values['coordinator_id'])
            envelope(goal.envelope)
            client = clients[fields['robot_id']]
            if not client.server_is_ready():
                log.error(f'execute_task server not ready for {fields["robot_id"]}; task {fields["task_id"]} '
                          'stays PENDING until admission or release')
            log.info(f'SEND {fields["kind"]} task={fields["task_id"]} robot={fields["robot_id"]} '
                     f'generation={fields["generation"]}')
            client.send_goal_async(goal).add_done_callback(lambda done: on_goal_response(fields['task_id'], done))

        def on_goal_response(task_id, future):
            handle = future.result()
            log.info(f'GOAL task={task_id} accepted={handle.accepted} (ACTIVE only from task_admission)')
            if handle.accepted:
                handle.get_result_async().add_done_callback(guarded('result', lambda done: on_task_result(
                    task_id, done.result().result.result)))

        def on_task_result(task_id, result):
            log.info(f'RESULT task={task_id} outcome={result.outcome} released={core.on_result(task_id, result)}')
            for note in core.drain_notes():
                log.info(note)

        snapshot = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)  # IF-071 latest snapshot
        state_pub = node.create_publisher(SystemState, SYSTEM_STATE_TOPIC, snapshot)
        from rescue_coordination.live_task_inputs import StatePublication
        state_channel = StatePublication(core, SystemState, envelope, state_pub.publish)
        if live_task_source is not None:
            from std_srvs.srv import Trigger
            def refresh_state(_request, response):
                measured = live_task_source[0].record
                now = now_ns()
                if (measured.get('state') != 'SYNCED'
                        or measured.get('valid_until_ns', 0) <= now + measured.get('error_bound_ns', 0)):
                    response.success = False
                    response.message = 'CURRENT_COORDINATOR_CLOCK_UNAVAILABLE'
                    return response
                fields = state_channel.send(now, refresh=True)
                response.success = fields is not None
                response.message = ('SYSTEM_STATE_REPUBLISHED; original changed_at_ns preserved'
                    if response.success else 'NO_PUBLISHED_SYSTEM_STATE')
                if response.success:
                    log.info('explicit SystemState refresh for session registration')
                return response
            node.create_service(Trigger, '~/refresh_state', refresh_state)
        targets_pub = node.create_publisher(TargetSnapshot, TARGETS_TOPIC, snapshot)  # IF-070 latest snapshot
        report_pub = node.create_publisher(RescueReport, REPORT_TOPIC, 10)
        lease_pubs = {robot: node.create_publisher(CommandLease, ROBOT_PREFIX + robot + '/command_lease', 1)
                      for robot in core.roles.values()}  # IF-014: RELIABLE / VOLATILE / KEEP_LAST 1
        last_block = [None]
        from std_srvs.srv import Trigger as FreezeTrigger
        freeze_client = node.create_client(FreezeTrigger, MAP_FREEZE_SERVICE) if core.map_mode == 'SLAM_FREEZE' else None
        log.info(f'map_mode={core.map_mode} freeze={MAP_FREEZE_SERVICE if freeze_client else "none"}')

        def call_freeze():
            if not freeze_client.service_is_ready():
                log.error(f'{MAP_FREEZE_SERVICE} not ready; the freeze fails at map_freeze_timeout_s unless it answers')
            log.info(f'MAP_FREEZE call {MAP_FREEZE_SERVICE}')
            freeze_client.call_async(FreezeTrigger.Request()).add_done_callback(guarded('freeze_reply', on_freeze_reply))

        def on_freeze_reply(future):
            res = future.result()
            recorded = core.on_freeze_reply(bool(res.success), res.message)
            (log.info if res.success else log.error)(
                f'MAP_FREEZE reply success={res.success} message={res.message!r} recorded={recorded}')
            tick()

        def on_frozen_map(msg):
            try:
                grid = FrozenGrid(msg)
            except ValueError as exc:
                log.error(f'FROZEN_MAP invalid: {exc} -> freeze failed={core.on_frozen_map_invalid(str(exc))}')
                return
            log.info(f'FROZEN_MAP {grid.fields["width"]}x{grid.fields["height"]} sha256={grid.sha256} '
                     f'-> frozen={core.on_frozen_map(grid)} basis={core.frozen}')
            tick()

        def publish_report(report):
            msg = RescueReport(report_id=report['report_id'], target_id=report['target_id'],
                               record_ids=report['record_ids'], locations=report['locations'],
                               confirmation_state='CONFIRMED', approach_state='UNASSIGNED', rescue_state='NOT_CHECKED',
                               first_issued_at_ns=report['first_issued_at_ns'], has_task_result=False)
            meaning = json.dumps([report['target_id'], report['target_revision'], report['record_ids']])
            msg.meaning_sha256 = hashlib.sha256(meaning.encode()).hexdigest()
            report_pub.publish(envelope(msg))
            log.info(f'REPORT {report["report_id"]} target={report["target_id"]} rev={report["target_revision"]}')

        class Out:
            """node_tick's sinks: ROS messages and log lines."""
            @staticmethod
            def task(fields):
                send(fields)

            @staticmethod
            def watch(fields):
                send_watch(fields)

            @staticmethod
            def report(report):
                publish_report(report)

            @staticmethod
            def lease(lease):
                msg = CommandLease(coordinator_id=values['coordinator_id'], **lease)
                lease_pubs[lease['robot_id']].publish(envelope(msg))

            @staticmethod
            def block(reason):
                if reason != last_block[0]:
                    last_block[0] = reason
                    if reason:
                        log.error(f'task not issued: {reason}')

            @staticmethod
            def targets(snapshot):
                estimates = []
                for item in snapshot['targets']:
                    estimate = TargetEstimate(**{k: v for k, v in item.items() if k != 'mission_position'})
                    estimate.mission_position.x, estimate.mission_position.y = map(float, item['mission_position'])
                    estimates.append(estimate)
                targets_pub.publish(envelope(TargetSnapshot(snapshot_id=snapshot['snapshot_id'], targets=estimates)))
                summary = [(t['target_id'], t['estimate_revision'], t['approach_state'], t['rescue_state'])
                           for t in snapshot['targets']]
                log.info(f'TARGETS {snapshot["snapshot_id"]} {summary}')

            @staticmethod
            def state(now):
                state = state_channel.send(now)
                if state is not None:
                    log.info(f'SYSTEM_STATE {state}')

            @staticmethod
            def freeze():
                call_freeze()

            @staticmethod
            def note(line):
                log.info(line)

        def tick():
            node_tick(core, now_ns(), Out)

        def on_site_event(msg):
            log.info(f'SITE_EVENT {msg.event_id} {msg.kind} -> {core.on_site_event(msg)}')
            tick()

        def on_command(req, res):
            selector = live_task_source[3] if live_task_source is not None else None
            if req.operation == 'START_SEARCH' and selector is not None and not selector.selected:
                core.last_reason = 'SEARCH_GOAL_NOT_SELECTED'
                reply = core._reply(False, 'REJECTED', 'PENDING_EVIDENCE')
            else:
                reply = core.on_command(req, now_ns=now_ns())
            envelope(res.envelope)
            res.command_id = req.command_id
            for name, value in reply.items():
                setattr(res, name, value)
            log.info(f'COMMAND {req.command_id} {req.operation} -> {reply} reason={core.last_reason}')
            tick()
            return res

        node.create_subscription(SiteEventMsg, SITE_EVENT_TOPIC, guarded('site_event', on_site_event), 10)
        for robot in core.roles.values():
            node.create_subscription(RobotStatus, ROBOT_PREFIX + robot + '/status',
                                     guarded('status', core.on_status), 1)
            node.create_subscription(AdmissionMsg, ROBOT_PREFIX + robot + '/task_admission', guarded(
                'admission', lambda msg: log.info(f'ADMISSION {msg.task_id} -> {core.on_admission(msg)}')), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # #176: map_state is latched
        for robot in core.roles.values():
            node.create_subscription(MapState, ROBOT_PREFIX + robot + '/map_state', guarded('map_state', lambda msg: log.info(
                f'MAP_STATE {msg.robot_id} {msg.map_session_id}/{msg.map_revision} {msg.localization_quality} '
                f'snapshot={msg.snapshot_id} -> recorded={core.on_map_state(msg)} frozen={core.frozen}')), latched)
        if core.map_mode == 'SLAM_FREEZE':
            from nav_msgs.msg import OccupancyGrid
            node.create_subscription(OccupancyGrid, FROZEN_MAP_TOPIC, guarded('frozen_map', on_frozen_map), latched)
        node.create_subscription(RobotPose, ROBOT_PREFIX + core.roles['SUPPORT'] + '/pose',
                                 guarded('pose', core.on_pose), 1)  # IF-011: RELIABLE / VOLATILE / KEEP_LAST 1
        node.create_subscription(TargetLocation, TARGET_LOCATION_TOPIC, guarded('target_location', lambda msg: log.info(
            f'TARGET_LOCATION {msg.location_id} -> {core.on_target_location(msg, now_ns=now_ns())}')), 10)
        # QR exit (안 「나」, OPS-136·138): same message as targets, class_label exit_qr only.
        node.create_subscription(TargetLocation, EXIT_LOCATION_TOPIC, guarded('exit_location', lambda msg: log.info(
            f'EXIT_LOCATION {msg.location_id} -> {core.on_exit_location(msg, now_ns=now_ns())} exits={[(e["exit_id"], e["xy"]) for e in core.exits]}')), 10)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        start_topic = START_POSE_TOPIC + core.roles['SCOUT']  # OPS-142: once per map session, latched
        node.create_subscription(PoseWithCovarianceStamped, start_topic, guarded('start_pose', lambda msg: log.info(
            f'START_POSE {start_topic} -> bound={core.on_start_pose(start_pose_from_msg(msg, core.search.policy.mission_frame_id))} '
            f'start_pose={core.start_pose}')), latched)
        node.create_subscription(ReportReceipt, REPORT_RECEIPT_TOPIC, guarded('receipt', lambda msg: log.info(
            f'RECEIPT {msg.report_id} {msg.state} -> saved={core.on_receipt(msg)}')), 10)
        node.create_service(SubmitOperatorCommand, OPERATOR_COMMAND_SERVICE, on_command)
        node.create_timer(values['tick_period_s'], guarded('tick', tick))
        spin_until_shutdown(lambda: rclpy.spin(node), node.context.ok, RCLError)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit) as exc:
        if isinstance(exc, SystemExit) and exc.code not in (None, 0):
            raise
    finally:
        if live_task_source is not None:
            live_task_source[0].close()
        node.destroy_node()
        rclpy.try_shutdown()
