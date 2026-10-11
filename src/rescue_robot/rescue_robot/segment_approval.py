"""Prepare a ONE_SEGMENT approval gate; never admit, arm or send a task.

TaskIngress uses this gate for MOVE_SEGMENT admission, and the task runtime
rechecks it before dispatch and on every step. This gate consumes authenticated operator decisions and
backend-verified route snapshots through mandatory local callbacks. Matching
UUIDs/hashes in an incoming message alone are not authorization or path proof.
It neither verifies a real Nav2 corridor nor enables automatic approach.
"""
from copy import deepcopy
from dataclasses import dataclass
import re

from rescue_contracts import RescueError, robot_id as validate_robot_id
from rescue_robot.executor import ContractError, GoalPose, generation, identifier, wall_time


def _name(value, field):
    if not isinstance(value, str) or not value or any(c.isspace() or ord(c) < 32 for c in value):
        raise ContractError('nonempty identifier required: ' + field)
    return value


def _segment_index(value):
    if type(value) is not int or not 0 <= value < 2**32:
        raise ContractError('uint32 segment_index required')
    return value


def _frame(frame):
    validate_robot_id(frame.robot_id)
    for field in ('native_frame', 'mission_frame'):
        _name(getattr(frame, field), field)
    identifier(frame.map_session_id)
    generation(frame.map_revision)
    generation(frame.transform_revision)
    if frame.alignment_status != 'VALID':
        raise ContractError('alignment not valid', 'FRAME_REVISION_MISMATCH')
    return (frame.robot_id, frame.native_frame, frame.mission_frame, frame.map_session_id,
            frame.map_revision, frame.transform_revision)


@dataclass(frozen=True)
class VerifiedSegmentRoute:
    """Trusted backend output, NOT a conversion of self-asserted wire fields.

    The backend callback must verify complete route/path/corridor content,
    passage/footprint policy, current mapping revisions and the selected segment pose.
    goal_pose is the validated selected segment endpoint, never the target centre
    or an inferred final RouteCandidate.approach_goal.
    """
    route_id: str
    route_revision: int
    segment_index: int
    robot_id: str
    target_id: str
    target_revision: int
    native_frame: str
    goal_pose: GoalPose
    path_sha256: str
    approval_corridor_sha256: str
    generated_at_ns: int
    valid_until_ns: int
    policy_version: str
    planner_evidence_id: str

    def __post_init__(self):
        _name(self.route_id, 'route_id')
        identifier(self.target_id)
        validate_robot_id(self.robot_id)
        for field in ('route_revision', 'target_revision'):
            generation(getattr(self, field))
        _segment_index(self.segment_index)
        if not isinstance(self.goal_pose, GoalPose):
            raise ContractError('backend must supply a validated segment GoalPose')
        for field in ('native_frame', 'policy_version', 'planner_evidence_id'):
            _name(getattr(self, field), field)
        for field in ('path_sha256', 'approval_corridor_sha256'):
            value = getattr(self, field)
            if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{64}', value):
                raise ContractError('canonical SHA-256 required: ' + field)
        wall_time(self.generated_at_ns)
        wall_time(self.valid_until_ns)
        if self.valid_until_ns <= self.generated_at_ns:
            raise ContractError('verified route lifetime')


@dataclass(frozen=True)
class ApprovedSegment:
    command_id: str
    task_id: str
    robot_id: str
    robot_boot_id: str
    authority_epoch: str
    generation: int
    target_id: str
    target_revision: int
    approval_id: str
    route_id: str
    route_revision: int
    segment_index: int
    goal_pose: GoalPose
    valid_until_ns: int


class SegmentApprovalGate:
    def __init__(self, adapter, *, authorize_operator, verify_route, current_target_revision,
                 clock_error_bound_ns):
        if not all(callable(callback) for callback in
                   (authorize_operator, verify_route, current_target_revision)):
            raise ContractError('operator authentication, backend route proof and current target callbacks required')
        if type(clock_error_bound_ns) is not int or clock_error_bound_ns <= 0:
            raise ContractError('positive measured clock_error_bound_ns required')
        self.adapter = adapter
        self.authorize_operator = authorize_operator
        self.verify_route = verify_route
        self.current_target_revision = current_target_revision
        self.clock_error_bound_ns = clock_error_bound_ns
        self.requests, self.approvals = {}, {}
        self.last_reason = ''
        self.last_wall_ns = None
        self.clock_fault = False

    def _now(self):
        now = wall_time(self.adapter.wall_clock_ns())
        if self.last_wall_ns is not None and now < self.last_wall_ns:
            self.clock_fault = True
        if self.clock_fault:
            raise ContractError('clock regressed; replace gate after clock recovery', 'CLOCK_UNCERTAIN')
        self.last_wall_ns = now
        return now

    def _current(self, goal):
        executor = self.adapter.executor
        if (goal.robot_id, goal.expected_robot_boot_id, goal.authority_epoch) != (
                executor.robot_id, executor.robot_boot_id, executor.authority_epoch):
            raise ContractError('robot boot/authority is no longer current', 'NOT_AUTHORIZED')
        if goal.policy_version != executor.policy.version:
            raise ContractError('execution policy differs', 'NOT_AUTHORIZED')
        current = self.current_target_revision(goal.target_id)
        generation(current)
        if current != goal.target_estimate_revision:
            raise ContractError('target revision changed', 'FRAME_REVISION_MISMATCH')
        if self.authorize_operator(goal.approval, goal) is not True:
            raise ContractError('operator decision not authenticated or revoked', 'NOT_AUTHORIZED')

    def _validate(self, goal):
        for field in ('command_id', 'task_id', 'expected_robot_boot_id', 'authority_epoch', 'target_id'):
            identifier(getattr(goal, field))
        generation(goal.generation)
        generation(goal.target_estimate_revision)
        if (goal.kind != 'MOVE_SEGMENT' or goal.has_route is not True or goal.has_approval is not True
                or goal.search_area_ref or goal.home_ref):
            raise ContractError('one target segment requires route and approval only')
        if goal.policy_version != self.adapter.executor.policy.version:
            raise ContractError('execution policy differs', 'NOT_AUTHORIZED')
        previous = self.requests.get(goal.command_id)
        if previous is not None and previous[0] != goal:
            raise ContractError('command content changed', 'ID_CONTENT_CONFLICT')
        approval, route = goal.approval, goal.route
        _name(route.route_id, 'route_id')
        generation(route.route_revision)
        _segment_index(route.segment_index)
        identifier(approval.approval_id)
        _name(approval.operator_id, 'operator_id')
        old_approval = self.approvals.get(approval.approval_id)
        if old_approval is not None and old_approval != approval:
            raise ContractError('approval ID content changed', 'ID_CONTENT_CONFLICT')
        identifier(approval.command_id)  # authenticated operator command, not ExecuteTask command (#90)
        expected = (goal.robot_id, goal.expected_robot_boot_id, goal.authority_epoch,
                    goal.target_id, goal.target_estimate_revision, route.route_id,
                    route.route_revision, route.segment_index)
        actual = (approval.robot_id, approval.robot_boot_id, approval.authority_epoch,
                  approval.target_id, approval.target_estimate_revision, approval.route_id,
                  approval.route_revision, approval.segment_index)
        _name(approval.route_id, 'approval route_id')
        generation(approval.route_revision)
        generation(approval.target_estimate_revision)
        _segment_index(approval.segment_index)
        if actual != expected or approval.scope != 'ONE_SEGMENT':
            raise ContractError('approval is not for this exact segment', 'NOT_AUTHORIZED')
        frame = _frame(goal.frame_ref)
        if frame != _frame(route.frame_ref):
            raise ContractError('route frame differs from command frame', 'FRAME_REVISION_MISMATCH')
        before = self._now()
        self._current(goal)
        proof = self.verify_route(goal)
        if not isinstance(proof, VerifiedSegmentRoute):
            raise ContractError('verified backend route unavailable', 'NOT_AUTHORIZED')
        if (route.robot_id, route.target_id, route.route_id, route.route_revision, route.segment_index,
                route.path_sha256, route.approval_corridor_sha256, route.generated_at_ns,
                route.policy_version, route.planner_evidence_id) != (
                proof.robot_id, proof.target_id, proof.route_id, proof.route_revision, proof.segment_index,
                proof.path_sha256, proof.approval_corridor_sha256, proof.generated_at_ns,
                proof.policy_version, proof.planner_evidence_id):
            raise ContractError('incoming route differs from backend evidence', 'ID_CONTENT_CONFLICT')
        pose = proof.goal_pose
        if (proof.robot_id != goal.robot_id or proof.target_id != goal.target_id
                or proof.target_revision != goal.target_estimate_revision
                or frame != (proof.robot_id, proof.native_frame, pose.frame_id,
                             pose.map_session_id, pose.map_revision, pose.transform_revision)
                or proof.policy_version != goal.policy_version):
            raise ContractError('backend route version differs', 'FRAME_REVISION_MISMATCH')
        if proof.valid_until_ns > wall_time(route.valid_until_ns):
            raise ContractError('backend proof cannot extend route lifetime')
        # Callbacks are local cache lookups, never RPC/planning. Recheck any time spent there.
        self._current(goal)
        now = self._now()
        for issued in (goal.issued_at_ns, approval.approved_at_ns, route.generated_at_ns):
            if wall_time(issued) > before:
                raise ContractError('future timestamp', 'CLOCK_UNCERTAIN')
        if (wall_time(goal.deadline_ns) <= goal.issued_at_ns
                or wall_time(approval.valid_until_ns) <= approval.approved_at_ns):
            raise ContractError('invalid command/approval lifetime')
        until = min(goal.deadline_ns, approval.valid_until_ns, proof.valid_until_ns, route.valid_until_ns)
        if until - now - self.clock_error_bound_ns <= 0:
            raise ContractError('segment approval expired', 'DEADLINE_EXPIRED')
        segment = ApprovedSegment(goal.command_id, goal.task_id, goal.robot_id,
            goal.expected_robot_boot_id, goal.authority_epoch, goal.generation,
            goal.target_id, goal.target_estimate_revision, approval.approval_id,
            proof.route_id, proof.route_revision, proof.segment_index, pose, until)
        if previous is not None:
            if previous[1] != segment:
                raise ContractError('backend proof changed for existing command', 'ID_CONTENT_CONFLICT')
            return previous[1]
        self.requests[goal.command_id] = (deepcopy(goal), segment)
        self.approvals[approval.approval_id] = deepcopy(approval)
        return segment

    def validate(self, goal):
        """Return immutable validation evidence only; never submit a task."""
        with self.adapter.lock:
            try:
                result = self._validate(goal)
            except (AttributeError, TypeError) as exc:
                raise ContractError('malformed segment input') from exc
            self.last_reason = ''
            return result

    def resolve(self, goal):
        return self.validate(goal).goal_pose

    def check(self, command_id):
        """Recheck revocation, current target/version and expiration for a cached command."""
        with self.adapter.lock:
            previous = self.requests.get(command_id)
            if previous is None:
                self.last_reason = 'NOT_AUTHORIZED'
                return False
            try:
                self.validate(previous[0])
            except RescueError as exc:
                self.last_reason = exc.code
                return False
            return True
