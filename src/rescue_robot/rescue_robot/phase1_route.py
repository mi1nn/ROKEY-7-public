"""#88 Phase 1 route endpoint with live #89 passage/protection checks.

Only planner path evidence is omitted. The injected passage_guard must verify
protected areas and actual planned/replanned path exclusion (e.g. a verified
Nav2 keepout configuration); UNKNOWN, unavailable or unloaded is never True.
It is not a boolean copied from the goal or an operator's approval.
"""
from copy import deepcopy
import math

from rescue_contracts import RescueError
from rescue_robot.executor import ContractError, GoalPose, generation, identifier, wall_time
from rescue_robot.segment_approval import ApprovedSegment
from rescue_robot.task_wire import validate_task_wire


def phase1_endpoint(goal):
    route, frame = goal.route, goal.frame_ref
    if (route.path_sha256 or route.planner_evidence_id or route.approval_corridor_sha256
            or route.planned_path.poses):
        raise ContractError('Phase 1 must not fabricate or mix planner evidence')
    assessment = route.assessment
    if (assessment.status != 'UNKNOWN' or any(getattr(assessment, name) is not False
            for name in ('width_known', 'height_known', 'floor_geometry_known'))):
        raise ContractError('Phase 1 passage assessment must be UNKNOWN')
    point = route.approach_goal
    if point.header.frame_id != frame.mission_frame:
        raise ContractError('endpoint frame differs', 'FRAME_REVISION_MISMATCH')
    p, q = point.pose.position, point.pose.orientation
    for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ContractError('finite planar endpoint required')
    if (p.z != 0 or q.x != 0 or q.y != 0
            or not math.isclose(q.z*q.z + q.w*q.w, 1., rel_tol=1e-6, abs_tol=1e-6)):
        raise ContractError('normalized planar endpoint quaternion required')
    return GoalPose(frame.mission_frame, p.x, p.y, 2 * math.atan2(q.z, q.w),
                    frame.map_session_id, frame.map_revision, frame.transform_revision)


class Phase1RouteGate:
    def __init__(self, adapter, *, registry, authorize_operator, current_target_revision, passage_guard,
                 clock_error_bound_ns):
        if not all(callable(c) for c in (authorize_operator, current_target_revision, passage_guard)):
            raise ContractError('operator, target and actual path protection providers required')
        if type(clock_error_bound_ns) is not int or clock_error_bound_ns <= 0:
            raise ContractError('positive measured clock bound required', 'CLOCK_UNCERTAIN')
        if not all(callable(getattr(registry, name, None)) for name in ('map_state', 'current_alignment')):
            raise ContractError('authoritative source and destination mapping registry required')
        self.adapter, self.clock_error_bound_ns = adapter, clock_error_bound_ns
        self.registry = registry
        self.authorize_operator = authorize_operator
        self.current_target_revision = current_target_revision
        self.passage_guard = passage_guard
        self.requests, self.approvals = {}, {}
        self.last_reason = ''

    def _endpoint(self, goal, now):
        """Verify the source coordinate basis, then capture the moving robot's basis.

        #88/#90 route.frame_ref is the target source's basis, not necessarily
        the SUPPORT robot. Never relabel it or apply the source robot's inverse
        to the moving robot. Both current alignments must share mission_frame.
        """
        frame = goal.frame_ref
        source_state = self.registry.map_state(frame.robot_id)
        source = self.registry.current_alignment(frame.robot_id, now_ns=now)
        if (frame.native_frame, frame.mission_frame, frame.map_session_id,
                frame.map_revision, frame.transform_revision) != (
                source.map_frame, source.mission_frame, source.map_session_id,
                source.map_revision, source.transform_revision):
            raise ContractError('target source alignment changed', 'FRAME_REVISION_MISMATCH')
        if (source_state.map_session_id, source_state.map_revision) != (
                frame.map_session_id, frame.map_revision):
            raise ContractError('target source map changed', 'FRAME_REVISION_MISMATCH')
        pose = phase1_endpoint(goal)
        own_state = self.registry.map_state(goal.robot_id)
        own = self.registry.current_alignment(goal.robot_id, now_ns=now)
        if (own.mission_frame != frame.mission_frame
                or (own_state.map_session_id, own_state.map_revision) != (own.map_session_id, own.map_revision)):
            raise ContractError('destination basis unavailable', 'FRAME_REVISION_MISMATCH')
        return GoalPose(pose.frame_id, pose.x, pose.y, pose.yaw, own.map_session_id,
                        own.map_revision, own.transform_revision)

    def validate(self, goal):
        with self.adapter.lock:
            try:
                if goal.kind not in ('MOVE_SEGMENT', 'REOBSERVE_TARGET') or goal.has_route is not True:
                    raise ContractError('Phase 1 endpoint kind/route required')
                now = wall_time(self.adapter.wall_clock_ns())
                validate_task_wire(goal, None, now_ns=now, clock_error_bound_ns=self.clock_error_bound_ns)
                executor = self.adapter.executor
                if (goal.robot_id, goal.expected_robot_boot_id, goal.authority_epoch, goal.policy_version) != (
                        executor.robot_id, executor.robot_boot_id, executor.authority_epoch, executor.policy.version):
                    raise ContractError('executor identity/policy mismatch', 'NOT_AUTHORIZED')
                if generation(self.current_target_revision(goal.target_id)) != goal.target_estimate_revision:
                    raise ContractError('target revision changed', 'FRAME_REVISION_MISMATCH')
                pose = self._endpoint(goal, now)
                approval_id = ''
                until = min(goal.deadline_ns, goal.route.valid_until_ns)
                if goal.kind == 'MOVE_SEGMENT':
                    approval = goal.approval
                    identifier(approval.command_id)  # operator command; different from task command
                    if approval.valid_until_ns != goal.route.valid_until_ns or goal.route.valid_until_ns != goal.deadline_ns:
                        raise ContractError('approval/route/task expiry differs', 'NOT_AUTHORIZED')
                    if self.authorize_operator(approval, goal) is not True:
                        raise ContractError('operator approval unavailable or revoked', 'NOT_AUTHORIZED')
                    old = self.approvals.get(approval.approval_id)
                    if old is not None and old != approval:
                        raise ContractError('approval ID content changed', 'ID_CONTENT_CONFLICT')
                    approval_id, until = approval.approval_id, min(until, approval.valid_until_ns)
                if self.passage_guard(goal) is not True:
                    raise ContractError('actual route protection missing/unknown or passage entered', 'NOT_AUTHORIZED')
                if generation(self.current_target_revision(goal.target_id)) != goal.target_estimate_revision:
                    raise ContractError('target changed during validation', 'FRAME_REVISION_MISMATCH')
                if goal.kind == 'MOVE_SEGMENT' and self.authorize_operator(goal.approval, goal) is not True:
                    raise ContractError('operator revoked during validation', 'NOT_AUTHORIZED')
                after = wall_time(self.adapter.wall_clock_ns())
                if after < now:
                    raise ContractError('clock regressed', 'CLOCK_UNCERTAIN')
                validate_task_wire(goal, None, now_ns=after, clock_error_bound_ns=self.clock_error_bound_ns)
                if self._endpoint(goal, after) != pose:
                    raise ContractError('coordinate basis changed during validation', 'FRAME_REVISION_MISMATCH')
                result = ApprovedSegment(goal.command_id, goal.task_id, goal.robot_id, goal.expected_robot_boot_id,
                    goal.authority_epoch, goal.generation, goal.target_id, goal.target_estimate_revision,
                    approval_id, goal.route.route_id, goal.route.route_revision, goal.route.segment_index, pose, until)
                old = self.requests.get(goal.command_id)
                if old is not None and old != (goal, result):
                    raise ContractError('existing route goal changed', 'ID_CONTENT_CONFLICT')
                self.requests[goal.command_id] = (deepcopy(goal), result)
                if approval_id: self.approvals[approval_id] = deepcopy(goal.approval)
                self.last_reason = ''
                return result
            except (AttributeError, TypeError) as exc:
                raise ContractError('malformed Phase 1 route') from exc

    def check(self, command_id):
        old = self.requests.get(command_id)
        if old is None:
            self.last_reason = 'NOT_AUTHORIZED'
            return False
        try:
            self.validate(old[0])
        except Exception as exc:
            self.last_reason = exc.code if isinstance(exc, RescueError) else 'INVALID_INPUT'
            return False
        return True
