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

Not yet (PR body): SetCameraWatch and candidate -> verdict linking, HomePosition
and RETURN_HOME assignment (Q-22), TargetSnapshot publishing.
"""
import hashlib
import json
import math
import subprocess
from pathlib import Path
from uuid import uuid4

from rescue_contracts import RescueError, SiteEvent, TaskAdmission, duration_ns, seconds_to_ns, timestamp_ns

from .mission import COMMAND_FIELDS, MissionCoordinator, OperatorCommand
from .planning import Allocator, CoordinationError, GoalPoint, contract_value, finite, point, revision, uid
from .search import SearchCoordinator, SearchPolicy, mission_sites

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
# C-10 (#88): internal segment phase -> TargetEstimate/SystemState segment_phase.
SEGMENT_PHASE_STATES = {'APPROVED': 'EXECUTING', 'MOVING': 'EXECUTING', 'AWAITING_APPROVAL': 'AWAITING_APPROVAL',
                        'ARRIVED': 'STOPPED'}
# Internal segment phase -> TargetEstimate.approach_state for the dispatched target [제안].
APPROACH_STATES = {'APPROVED': 'ASSIGNED', 'MOVING': 'IN_PROGRESS', 'AWAITING_APPROVAL': 'IN_PROGRESS',
                   'ARRIVED': 'APPROACH_COMPLETE'}


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
    if req.operation not in COMMAND_FIELDS:
        raise _invalid('operation not handled by the mission gate')
    if req.robot_id and req.robot_id != roles.get(COMMAND_ROLES.get(req.operation)):
        raise _invalid('robot_id must be empty or the role robot of this operation')
    used = COMMAND_FIELDS[req.operation]
    values = {'target_id': req.target_id or None, 'target_revision': req.expected_target_revision,
              'route_revision': req.route_revision, 'segment_index': req.segment_index}
    return OperatorCommand(command_id=req.command_id, kind=req.operation, operator_id=req.operator_id,
                           requested_at_ns=req.issued_at_ns,
                           **{name: values[name] if name in used else None for name in values})


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
                 lease_message_ttl_ns, new_id=None):
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

    # -- inputs --------------------------------------------------------------
    def on_site_event(self, msg):
        return self.mission.record_site_event(site_event_from_msg(msg))

    def on_status(self, msg):
        if msg.robot_id not in self.roles.values():
            raise CoordinationError('WRONG_ROBOT', 'status from a robot outside the policy')
        self.status[msg.robot_id] = (msg.robot_boot_id, msg.readiness, bool(msg.lease_valid), msg.observed_at_ns)
        self.status_arrivals += 1  # arrival order at this node; no clock comparison
        self.status_detail[msg.robot_id] = (self.status_arrivals, msg.stop_reason, msg.motion_state,
                                            bool(msg.lease_valid), msg.active_task_id)

    def on_admission(self, msg):
        admission = admission_from_msg(msg)
        admitted = self.search.record_admission(admission)
        meta = self.task_meta.get(admission.task_id)
        if admitted and admission.accepted and admission.task_id in self.leases:
            self.leases[admission.task_id]['admitted_after'] = self.status_arrivals
        if admitted and admission.accepted and meta and meta[0] == 'MOVE_SEGMENT':
            self.mission.segment_started(meta[1].target_id, target_revision=meta[1].target_revision,
                                         route_revision=meta[1].route_revision, segment_index=meta[1].segment_index)
        elif admitted and not admission.accepted and meta:
            self._forget(admission.task_id)  # refused before moving: issue again on the next tick
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
        elif meta and meta[0] == 'REOBSERVE_TARGET' and result.outcome != 'COMPLETED':
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
        if any(geo[0] != basis for geo in self.target_geo.values()):
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
        unsaved = [loc for loc in locations if loc.location_id not in self.saved_records]
        report_id = self.new_id()
        self.reports[report_id] = dict(report_id=report_id, target_id=target_id, target_revision=target_revision,
                                       record_ids=[loc.location_id for loc in unsaved], locations=unsaved,
                                       first_issued_at_ns=now_ns, next_send_ns=now_ns, done=False)
        return target_id

    def reports_due(self, now_ns):
        """RescueReports to (re)send now; resent every ``report_retry_ns`` until PERSISTED or REJECTED."""
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
            reason=self.issue_block or ('SEARCH_QUEUED' if self.search_queued() else ''),
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
            targets.append(dict(
                target_id=target_id, estimate_revision=target_revision, locations=list(locations),
                latest_observed_at_ns=max(loc.observation.time.observed_at_ns for loc in locations),
                valid_until_ns=latest.observation.time.valid_until_ns, mission_position=xy,
                covariance=list(latest.covariance), confirmation_state='CONFIRMED',
                approach_state=APPROACH_STATES[mission.segment_phase] if dispatched else 'UNASSIGNED',
                segment_phase=SEGMENT_PHASE_STATES[mission.segment_phase] if dispatched else 'IDLE',
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
        detail = self.status_detail.get(self.roles['SCOUT'])
        if self.mission.mode != 'DISASTER':
            self._cancel_queued_search('MODE_NOT_DISASTER')
        elif detail is not None and detail[2] in STOPPING_MOTION:
            self._cancel_queued_search('SCOUT_STOPPING')
        # Order matters: an open recheck takes a free SCOUT before a new search area does.
        # NORMAL needs no check here: no approval or recheck exists and mission.plan gates the search.
        self._issue_segment(now_ns)
        self._issue_rechecks(now_ns)
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
        if seen is not None:
            if seen[0] != command:
                self.last_reason = 'ID_CONTENT_CONFLICT'
                return self._reply(False, 'REJECTED', 'ID_CONTENT_CONFLICT')
            first = seen[1]
            self.last_reason = 'DUPLICATE'
            return self._reply(first['accepted'], 'DUPLICATE', first['reason_code'], first['created_task_id'])
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

    def _route(self, fields, goal, frame, route_id, segment_index):
        return dict(route_id=route_id, route_revision=self.route_revision, robot_id=fields['robot_id'],
                    target_id=fields['target_id'], segment_index=segment_index, approach_goal=goal,
                    frame_ref=frame, generated_at_ns=self.route_made_at[fields['target_id']],
                    valid_until_ns=fields['deadline_ns'])

    def _issue_segment(self, now_ns):
        approval = self.mission.approval
        if (approval is None or self.mission.segment_phase != 'APPROVED'
                or approval.command_id in self.segment_tasks or approval.route_revision != self.route_revision):
            return self.segment_tasks.get(approval.command_id) if approval else None
        if not self._passage_free(approval.segment_index):
            self.issue_block = 'PASSAGE_ASSESSMENT_REQUIRED'  # Q-25 default: hold, never guess
            return None
        boot = self._free('SUPPORT', now_ns)
        if boot is None:
            return None
        goal = self.segment_goals[approval.segment_index]
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
            if recheck.link_id in self.recheck_tasks:
                continue
            boot = self._free('SCOUT', now_ns)
            frame = self._route_frame(target_id, self.observe_goal)
            if boot is None or frame is None:
                return
            decision = self.search.reserve_goal_task(
                self.roles['SCOUT'], robot_boot_id=boot, kind='REOBSERVE_TARGET', target_id=target_id,
                target_revision=self.mission.targets[target_id][0], goal=self.observe_goal, now_ns=now_ns)
            if decision.action != 'RESERVED':
                return
            fields = decision.task_fields
            # REOBSERVE: the observation point as route, no operator approval (C-10, #88).
            fields = fields | dict(route=self._route(fields, self.observe_goal, frame, self.observe_route_id, 0),
                                   approval=None)
            self.recheck_tasks[recheck.link_id] = fields['task_id']
            self.task_meta[fields['task_id']] = ('REOBSERVE_TARGET', recheck.link_id)
            self._send(fields)

    def _issue_search(self, now_ns):
        scout = self.roles['SCOUT']
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
        table = self.segment_tasks if kind == 'MOVE_SEGMENT' else self.recheck_tasks
        table.pop(key.command_id if kind == 'MOVE_SEGMENT' else key, None)

    @staticmethod
    def _reply(accepted, state, reason_code, created_task_id=''):
        return dict(accepted=accepted, request_state=state, reason_code=reason_code,
                    created_task_id=created_task_id)


# ---- ROS wiring (needs a sourced ROS 2 environment) --------------------------------------------
def declare_required(node, names, optional=()):
    """Typed parameters without defaults -> (values, missing). Jazzy rejects a typed
    ``declare_parameter`` with no value, so the type goes in as ``Parameter.Type`` (#35)."""
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
                     if item is None or (name not in optional and item in ('', [])))
    return values, missing


def build_core(values):
    """Profile v2 + typed parameters -> CoordinatorCore (no defaults anywhere)."""
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
    # No profile home positions yet: RETURN_HOME stays NO_HOME_POSITION (assignment is Q-22, later).
    mission = MissionCoordinator(search, home_positions=())
    basis = (values['goal_map_session_id'], values['goal_map_revision'], values['goal_transform_revision'])
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
                           lease_message_ttl_ns=seconds_to_ns(str(values['lease_message_ttl_s'])))


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
    # AST88-01: one field-checked bool per segment end; may be empty (= nothing passage-free, hold).
    'route_segment_passage_free': 'BOOL_ARRAY'}
OPTIONAL_PARAMETERS = ('route_segment_passage_free',)


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterType
    from rescue_interfaces.action import ExecuteTask
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import (CommandLease, ReportReceipt, RescueReport, RobotStatus,
                                       SiteEvent as SiteEventMsg, SystemState, TargetEstimate, TargetLocation,
                                       TargetSnapshot, TaskAdmission as AdmissionMsg)
    from rescue_interfaces.srv import SubmitOperatorCommand

    rclpy.init(args=args)
    node = Node('mission_coordinator')
    log = node.get_logger()
    try:
        values, missing = declare_required(
            node, {name: getattr(ParameterType, 'PARAMETER_' + kind) for name, kind in PARAMETERS.items()},
            optional=OPTIONAL_PARAMETERS)
        if missing:
            log.fatal(f'missing parameters: {missing}')
            raise SystemExit(2)
        try:
            core = build_core(values)
        except (RescueError, OSError, ValueError) as exc:
            log.fatal(f'configuration rejected: {exc}')
            raise SystemExit(2)
        boot_id, sequence = str(uuid4()), iter(range(1, 2 ** 63))
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
                handle.get_result_async().add_done_callback(guarded('result', lambda done: log.info(
                    f'RESULT task={task_id} released={core.on_result(task_id, done.result().result.result)}')))

        snapshot = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)  # IF-071 latest snapshot
        state_pub = node.create_publisher(SystemState, SYSTEM_STATE_TOPIC, snapshot)
        targets_pub = node.create_publisher(TargetSnapshot, TARGETS_TOPIC, snapshot)  # IF-070 latest snapshot
        report_pub = node.create_publisher(RescueReport, REPORT_TOPIC, 10)
        lease_pubs = {robot: node.create_publisher(CommandLease, ROBOT_PREFIX + robot + '/command_lease', 1)
                      for robot in core.roles.values()}  # IF-014: RELIABLE / VOLATILE / KEEP_LAST 1
        last_block = [None]

        def publish_report(report):
            msg = RescueReport(report_id=report['report_id'], target_id=report['target_id'],
                               record_ids=report['record_ids'], locations=report['locations'],
                               confirmation_state='CONFIRMED', approach_state='UNASSIGNED', rescue_state='NOT_CHECKED',
                               first_issued_at_ns=report['first_issued_at_ns'], has_task_result=False)
            meaning = json.dumps([report['target_id'], report['target_revision'], report['record_ids']])
            msg.meaning_sha256 = hashlib.sha256(meaning.encode()).hexdigest()
            report_pub.publish(envelope(msg))
            log.info(f'REPORT {report["report_id"]} target={report["target_id"]} rev={report["target_revision"]}')

        def tick():
            now = now_ns()
            for fields in core.tick(now):
                send(fields)
            for report in core.reports_due(now):
                publish_report(report)
            for lease in core.leases_due(now):
                msg = CommandLease(coordinator_id=values['coordinator_id'], **lease)
                lease_pubs[lease['robot_id']].publish(envelope(msg))
            if core.issue_block != last_block[0]:
                last_block[0] = core.issue_block
                if core.issue_block:
                    log.error(f'task not issued: {core.issue_block}')
            targets = core.target_snapshot_change()
            if targets is not None:
                estimates = []
                for item in targets['targets']:
                    estimate = TargetEstimate(**{k: v for k, v in item.items() if k != 'mission_position'})
                    estimate.mission_position.x, estimate.mission_position.y = map(float, item['mission_position'])
                    estimates.append(estimate)
                targets_pub.publish(envelope(TargetSnapshot(snapshot_id=targets['snapshot_id'], targets=estimates)))
                summary = [(t['target_id'], t['estimate_revision'], t['approach_state'], t['rescue_state'])
                           for t in targets['targets']]
                log.info(f'TARGETS {targets["snapshot_id"]} {summary}')
            state = core.system_state_change(now)
            if state is not None:
                msg = SystemState(**state)
                envelope(msg)
                state_pub.publish(msg)
                log.info(f'SYSTEM_STATE {state}')

        def on_site_event(msg):
            log.info(f'SITE_EVENT {msg.event_id} {msg.kind} -> {core.on_site_event(msg)}')
            tick()

        def on_command(req, res):
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
        node.create_subscription(TargetLocation, TARGET_LOCATION_TOPIC, guarded('target_location', lambda msg: log.info(
            f'TARGET_LOCATION {msg.location_id} -> {core.on_target_location(msg, now_ns=now_ns())}')), 10)
        node.create_subscription(ReportReceipt, REPORT_RECEIPT_TOPIC, guarded('receipt', lambda msg: log.info(
            f'RECEIPT {msg.report_id} {msg.state} -> saved={core.on_receipt(msg)}')), 10)
        node.create_service(SubmitOperatorCommand, OPERATOR_COMMAND_SERVICE, on_command)
        node.create_timer(values['tick_period_s'], guarded('tick', tick))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit) as exc:
        if isinstance(exc, SystemExit) and exc.code not in (None, 0):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
