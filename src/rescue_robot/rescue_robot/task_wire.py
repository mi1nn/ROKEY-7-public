"""Structural ExecuteTask/CommandLease boundary, independent of rclpy.

These checks authenticate no publisher and choose no route or home coordinate.
The caller must authenticate the issuer and resolve a current, authorized goal
from its own records. The field integration must also re-check route/target
permission while driving (MissionGoalBinder.target_gate).
"""
from rescue_contracts import TASK_KINDS, TARGET_TASK_KINDS
import math

from rescue_robot.executor import SPIN_MAX_YAW_RAD, ContractError, GoalPose, generation, identifier, wall_time

COMPLETION_SCOPES = {'SEARCH_AREA': 'SEARCH', 'APPROACH_TARGET': 'TARGET_APPROACH',
                     'REOBSERVE_TARGET': 'RECHECK', 'HOLD': 'HOLD',
                     'MOVE_SEGMENT': 'SEGMENT', 'RETURN_HOME': 'RETURN', 'SPIN': 'SPIN'}


def validate_completion_scope(kind, result):
    """Keep one-segment arrival separate from approach or rescue completion."""
    if kind not in COMPLETION_SCOPES or result.completion_scope != COMPLETION_SCOPES[kind]:
        raise ContractError('result completion_scope does not match task kind')
    return result


def text(value, name):
    if not isinstance(value, str) or not value:
        raise ContractError(name)
    return value


def flag(value, name):
    if type(value) is not bool:
        raise ContractError(name)
    return value


def frame_key(ref):
    return (text(ref.robot_id, 'frame robot'), text(ref.native_frame, 'native frame'),
            text(ref.mission_frame, 'mission frame'), identifier(ref.map_session_id),
            generation(ref.map_revision), generation(ref.transform_revision), ref.alignment_status)


def window(start, end, now, bound, name):
    wall_time(start)
    wall_time(end)
    if end <= start:
        raise ContractError(name + ' interval')
    if start > now:
        raise ContractError(name + ' from future', 'CLOCK_UNCERTAIN')
    if now + bound >= end:
        raise ContractError(name + ' expired or within clock uncertainty', 'DEADLINE_EXPIRED')


def validate_task_wire(goal, lease, *, now_ns, clock_error_bound_ns):
    """Raise before admission; return remaining lease seconds without extending it.

Bounds are measured/supplied inputs, not default field policy. Integer ns are
preserved until the executor's monotonic lease-duration boundary.
"""
    wall_time(now_ns)
    if type(clock_error_bound_ns) is not int or not 0 <= clock_error_bound_ns < 2**63:
        raise ContractError('measured clock error bound required', 'CLOCK_UNCERTAIN')
    for name in ('command_id', 'task_id', 'expected_robot_boot_id', 'authority_epoch', 'lease_id'):
        identifier(getattr(goal, name))
    generation(goal.generation)
    text(goal.robot_id, 'robot_id')
    text(goal.coordinator_id, 'coordinator_id')
    text(goal.policy_version, 'policy_version')
    if goal.kind not in TASK_KINDS:
        raise ContractError('unsupported task kind')
    flag(goal.has_route, 'has_route')
    flag(goal.has_approval, 'has_approval')
    generation(goal.target_estimate_revision)
    if goal.kind in TARGET_TASK_KINDS:
        identifier(goal.target_id)
        generation(goal.target_estimate_revision)
    elif goal.target_id != '' or goal.target_estimate_revision != 0:
        raise ContractError('non-target kind carries target')
    if goal.kind == 'SEARCH_AREA':
        identifier(goal.search_area_ref)
    elif goal.search_area_ref != '':
        raise ContractError('unexpected search_area_ref')
    if goal.kind == 'RETURN_HOME':
        text(goal.home_ref, 'home_ref')
    elif goal.home_ref != '':
        raise ContractError('unexpected home_ref')
    if goal.kind == 'SPIN':   # C29: the turn in place; checked again on the Task
        for name in ('spin_yaw_rad', 'time_allowance_s'):
            value = getattr(goal, name)
            if type(value) is not float or not math.isfinite(value):
                raise ContractError(name)
        if not 0 < abs(goal.spin_yaw_rad) <= SPIN_MAX_YAW_RAD or goal.time_allowance_s <= 0:
            raise ContractError('spin_yaw_rad must turn at most one turn, time_allowance_s positive')
    elif goal.spin_yaw_rad != 0.0 or goal.time_allowance_s != 0.0:
        raise ContractError('only SPIN carries spin_yaw_rad and time_allowance_s')
    window(goal.issued_at_ns, goal.deadline_ns, now_ns, clock_error_bound_ns, 'task')
    expected = (goal.lease_id, goal.robot_id, goal.expected_robot_boot_id, goal.coordinator_id,
                goal.authority_epoch, goal.generation)
    if lease is not None:
        actual = (lease.lease_id, lease.robot_id, lease.robot_boot_id, lease.coordinator_id,
                  lease.authority_epoch, lease.generation)
        if expected != actual:
            raise ContractError('goal/lease identity mismatch', 'NOT_AUTHORIZED')
        generation(lease.renewal_sequence)
        generation(lease.generation)
        window(lease.issued_at_ns, lease.valid_until_ns, now_ns, clock_error_bound_ns, 'lease')
    if goal.kind == 'SPIN':
        # turning in place needs no map pose; the frame names this robot only
        if goal.frame_ref.robot_id != goal.robot_id:
            raise ContractError('goal frame robot mismatch', 'FRAME_REVISION_MISMATCH')
        if goal.has_route or goal.has_approval:
            raise ContractError('SPIN must not carry movement permissions')
    elif goal.kind != 'HOLD':
        if goal.kind == 'RETURN_HOME' and _no_basis(goal.frame_ref):
            # Decision 5A: the coordinator sends home_ref and no goal pose (search._goal_pose), so
            # the frame carries the robot only; the robot resolves home/<robot_id> on its own
            # current alignment (resolve_goal). Runs 6 and 8, 2026-10-16: refused on 'native
            # frame', then on 'mission frame'.
            if goal.frame_ref.robot_id != goal.robot_id:
                raise ContractError('goal frame robot mismatch', 'FRAME_REVISION_MISMATCH')
        elif goal.kind in ('SEARCH_AREA', 'RETURN_HOME'):
            # #90 supplies the area/basis but intentionally does not assert an
            # alignment. A live authoritative resolver/binder must supply it. A RETURN_HOME
            # that names a basis is held to it, all of it.
            ref = goal.frame_ref
            text(ref.mission_frame, 'mission frame')
            identifier(ref.map_session_id)
            generation(ref.map_revision)
            generation(ref.transform_revision)
            if ref.robot_id != goal.robot_id:
                raise ContractError('goal frame robot mismatch', 'FRAME_REVISION_MISMATCH')
        else:
            key = frame_key(goal.frame_ref)
            if key[-1] != 'VALID' or (goal.kind not in TARGET_TASK_KINDS and key[0] != goal.robot_id):
                raise ContractError('unusable goal frame', 'FRAME_REVISION_MISMATCH')
    elif goal.has_route or goal.has_approval:
        raise ContractError('HOLD must not carry movement permissions')
    if goal.has_route:
        route = goal.route
        text(route.route_id, 'route_id')  # #90 uses the fixed route's SHA-256, not a UUID
        generation(route.route_revision)
        if (type(route.segment_index) is not int or not 0 <= route.segment_index < 2**32
                or route.robot_id != goal.robot_id or route.target_id != goal.target_id
                or frame_key(route.frame_ref) != frame_key(goal.frame_ref)
                or route.policy_version != goal.policy_version):
            raise ContractError('route does not match task', 'FRAME_REVISION_MISMATCH')
        window(route.generated_at_ns, route.valid_until_ns, now_ns, clock_error_bound_ns, 'route')
    if goal.kind == 'MOVE_SEGMENT':
        if not goal.has_route or not goal.has_approval:
            raise ContractError('one-segment route and approval required', 'NOT_AUTHORIZED')
        approval = goal.approval
        identifier(approval.approval_id)
        identifier(approval.command_id)  # authenticated binding checked by SegmentApprovalGate
        text(approval.operator_id, 'operator_id')
        generation(approval.target_estimate_revision)
        generation(approval.route_revision)
        text(approval.route_id, 'approval route_id')
        if type(approval.segment_index) is not int or not 0 <= approval.segment_index < 2**32:
            raise ContractError('approval segment_index')
        expected = (goal.robot_id, goal.expected_robot_boot_id, goal.authority_epoch, goal.target_id,
                    goal.target_estimate_revision, goal.route.route_id, goal.route.route_revision,
                    goal.route.segment_index, 'ONE_SEGMENT')
        actual = (approval.robot_id, approval.robot_boot_id, approval.authority_epoch, approval.target_id,
                  approval.target_estimate_revision, approval.route_id, approval.route_revision,
                  approval.segment_index, approval.scope)
        if expected != actual:
            raise ContractError('approval does not match one segment', 'NOT_AUTHORIZED')
        window(approval.approved_at_ns, approval.valid_until_ns, now_ns, clock_error_bound_ns, 'approval')
    elif goal.has_approval:
        raise ContractError('approval only supported on MOVE_SEGMENT', 'NOT_AUTHORIZED')
    until = goal.deadline_ns if lease is None else min(lease.valid_until_ns, goal.deadline_ns)
    return (until - now_ns - clock_error_bound_ns) / 1e9


def _no_basis(ref):
    """A frame_ref with no mission frame and no map basis (only the robot): RETURN_HOME under decision 5A."""
    return (ref.mission_frame, ref.map_session_id, ref.map_revision, ref.transform_revision) == ('', '', 0, 0)


def validate_resolved_goal(goal, pose):
    """Only verify a resolver's output; never infer it from target/route coordinates.

Phase 1 MOVE_SEGMENT uses RouteCandidate.approach_goal as the segment end
(#88). Strict planner-backed routes use the backend's selected segment pose.
"""
    if goal.kind in ('HOLD', 'SPIN'):
        if pose is not None:
            raise ContractError(goal.kind + ' has no pose')
        return None
    if not isinstance(pose, GoalPose):
        raise ContractError('authoritative resolved GoalPose required')
    if goal.kind == 'RETURN_HOME' and _no_basis(goal.frame_ref):
        return pose   # 5A: the robot's resolver chose the frame and basis (its current alignment)
    if (pose.frame_id, pose.map_session_id, pose.map_revision, pose.transform_revision) != (
            goal.frame_ref.mission_frame, goal.frame_ref.map_session_id,
            goal.frame_ref.map_revision, goal.frame_ref.transform_revision):
        raise ContractError('resolved pose frame/revision mismatch', 'FRAME_REVISION_MISMATCH')
    return pose
