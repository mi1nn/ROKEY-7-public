"""Role-driven search/hold/approach proposals on top of the existing Allocator.

Scenario (BRD v0.1, 2026-10-08): the SCOUT robot searches configured areas in
order, the SUPPORT robot holds (stops in place), and each discovery event gets at most one
APPROACH_TARGET reservation for the SUPPORT robot. Roles are policy input; no
robot ID is fixed here.

This module never decides acceptance or keeps a generation ledger. Generations
come from ``Allocator.issue_generation``, PENDING from ``Allocator.reserve`` and
ACTIVE only from ``Allocator.record_admission`` with the executor receipt. It
returns proposals and executor ``Task`` field values; it sends no ROS command.
"""
from dataclasses import dataclass, field
import math
from uuid import uuid4

from rescue_contracts import timestamp_ns, duration_ns

from .planning import (Allocator, CoordinationError, PathCandidate, Reservation,
                       contract_value, finite, point, revision, robot, uid)

ROLES = ('SCOUT', 'SUPPORT')  # BRD v0.1 names: TB1 정찰 / TB2 구조 지원
ACTIONS = frozenset({'RESERVED', 'KEEP', 'WAIT', 'RELEASE_REQUIRED'})
# Closed set of decision reasons; no free text is returned (작전 Q5, 2026-10-08).
SEARCH_REJECT_REASONS = frozenset({
    'NOT_READY', 'HOLDING', 'PAUSED_FOR_APPROACH',
    'NO_EXECUTABLE_AREA', 'PROJECTION_NOT_VALID', 'NO_SUPPORT_CANDIDATE', 'CANDIDATE_EXPIRED',
    'SUPPORT_BUSY', 'ALLOCATOR_REJECTED', 'AREA_WITHDRAWN', 'SUPPORT_HOLDING',
    'EVENT_ALREADY_HAS_APPROACH'})
DECISION_REASONS = SEARCH_REJECT_REASONS | {'PENDING', 'ACTIVE', 'PENDING_EXECUTOR_ADMISSION'}
EXCLUSION_REASONS = frozenset({'BASIS_NOT_EXECUTABLE', 'NO_MAP_REVISION', 'WITHDRAWN'})
# mapping.MappingRegistry.projection(...).status that may feed an approach.
APPROACH_PROJECTION_STATUS = 'VALID'


@dataclass(frozen=True)
class MissionSite:
    """A configured search area in the mission frame.

    ``basis`` names where the coordinates came from. Only bases listed in
    ``SearchPolicy.executable_bases`` and bound to a map session/revision may be
    executed; anything else is reported as excluded, never guessed into a goal.
    ``entry_yaw`` (rad, mission frame) is the configured heading on arrival;
    required, keyword-only (profile v2.1 ``search_areas[].entry_yaw``, 작전).
    """
    site_id: str
    mission_xy: tuple
    basis: str
    map_session_id: str | None = None
    map_revision: int | None = None
    transform_revision: int | None = None
    entry_yaw: float = field(kw_only=True)

    def __post_init__(self):
        uid(self.site_id)
        point(self.mission_xy)
        finite(self.entry_yaw, 'entry_yaw', minimum=-math.inf)
        if not isinstance(self.basis, str) or not self.basis:
            raise CoordinationError('INVALID_INPUT', 'site basis tag')
        if self.map_session_id is not None:
            uid(self.map_session_id)
        for value in (self.map_revision, self.transform_revision):
            if value is not None:
                revision(value)

    def bound_to_map(self):
        return None not in (self.map_session_id, self.map_revision, self.transform_revision)


@dataclass(frozen=True)
class SearchPolicy:
    """Who searches, who waits and how tasks are timed.

    Decided by: 작전·총괄 instruction 2026-10-08 (BRD v0.1 role split). Numeric
    values are run inputs, not locked field thresholds (scope Q-06).

    - ``roles``: ((robot_id, 'SCOUT'|'SUPPORT'), ...), exactly one of each.
    - ``areas``: search order. Ties are impossible by construction: the first
      executable, unfinished, unreserved area in this order wins. No cost is
      compared, so a small cost change never switches a running assignment.
    - ``scout_continues_during_approach``: True keeps the SCOUT searching new
      areas while an approach is reserved/active; False stops proposing new
      areas (the current area is not cancelled by this flag).
    - ``mission_frame_id``: frame of area/candidate coordinates in goal poses;
      from the deployment profile, never a code constant (AGENTS 8).
    """
    version: str
    roles: tuple
    areas: tuple
    executable_bases: frozenset
    task_duration_ns: int
    lease_duration: float
    executor_policy_version: str
    mission_frame_id: str
    scout_continues_during_approach: bool = True

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise CoordinationError('INVALID_POLICY', 'policy version')
        if not isinstance(self.executor_policy_version, str) or not self.executor_policy_version:
            raise CoordinationError('INVALID_POLICY', 'executor policy version')
        if not isinstance(self.mission_frame_id, str) or not self.mission_frame_id:
            raise CoordinationError('INVALID_POLICY', 'mission frame')
        if not isinstance(self.roles, tuple) or not all(isinstance(r, tuple) and len(r) == 2 for r in self.roles):
            raise CoordinationError('INVALID_POLICY', 'roles must be ((robot_id, role), ...)')
        ids = [robot(r) for r, _ in self.roles]
        if len(set(ids)) != len(ids) or sorted(role for _, role in self.roles) != sorted(ROLES):
            raise CoordinationError('INVALID_POLICY', 'exactly one SCOUT and one SUPPORT robot')
        if not isinstance(self.areas, tuple) or not all(isinstance(a, MissionSite) for a in self.areas):
            raise CoordinationError('INVALID_POLICY', 'areas must be a tuple of MissionSite')
        site_ids = [a.site_id for a in self.areas]
        if len(set(site_ids)) != len(site_ids):
            raise CoordinationError('INVALID_POLICY', 'site IDs must be unique')
        if not isinstance(self.executable_bases, frozenset) or not all(
                isinstance(b, str) and b for b in self.executable_bases):
            raise CoordinationError('INVALID_POLICY', 'executable_bases')
        contract_value(duration_ns, self.task_duration_ns)
        if self.task_duration_ns <= 0:
            raise CoordinationError('INVALID_POLICY', 'task duration')
        finite(self.lease_duration, 'lease_duration')
        if self.lease_duration <= 0:
            raise CoordinationError('INVALID_POLICY', 'lease duration')
        if type(self.scout_continues_during_approach) is not bool:
            raise CoordinationError('INVALID_POLICY', 'scout_continues_during_approach')

    def role_of(self, role):
        return next(r for r, value in self.roles if value == role)

    def executable(self, site):
        return site.basis in self.executable_bases and site.bound_to_map()


# Re-observation (Part I 3.5.3) reasons; disjoint from the search/approach set.
RECHECK_REASONS = frozenset({
    'NO_NEW_EXIT_PASS', 'APPROACH_NOT_COMPLETED', 'EVENT_ALREADY_HAS_RECHECK', 'NO_SCOUT_CANDIDATE',
    'SCOUT_SEARCHING', 'SCOUT_BUSY'})


@dataclass(frozen=True)
class Decision:
    """``RESERVED`` means PENDING in the Allocator, never ACTIVE or moving.

    ``task_fields`` are keyword arguments for ``rescue_robot.executor.Task``;
    the robot lane builds and admits the Task and returns the TaskAdmission.
    ``goal_pose`` is a plain dict with GoalPose's fields (frame_id, x, y, yaw,
    map_session_id, map_revision, transform_revision) or None for HOLD. The robot lane converts it
    with ``GoalPose(**d)``; coordination does not import rescue_robot, keeping
    the contracts -> features dependency direction (작전 2026-10-08).
    """
    action: str
    reason: str
    reservation: Reservation | None = None
    task_fields: dict | None = None

    def __post_init__(self):
        if self.action not in ACTIONS or self.reason not in DECISION_REASONS | RECHECK_REASONS:
            raise CoordinationError('INVALID_INPUT', 'decision action/reason outside the closed set')


class SearchCoordinator:
    """Proposal layer; all acceptance/generation state stays in ``Allocator``."""

    def __init__(self, allocator, policy, *, new_id=None):
        if not isinstance(allocator, Allocator) or not isinstance(policy, SearchPolicy):
            raise CoordinationError('INVALID_POLICY', 'allocator/policy type')
        if {r for r, _ in policy.roles} != set(allocator.allowed_robot_ids):
            raise CoordinationError('INVALID_POLICY', 'roles must name the deployment robots')
        self.allocator = allocator
        self.policy = policy
        self.new_id = new_id or (lambda: str(uuid4()))
        self.completed_areas = set()
        self.withdrawn_areas = set()
        self.approach_tasks = {}  # event_id -> task_id; one approach per event (E-06)
        self.held = {}  # robot_id -> robot_boot_id whose HOLD finished with stop confirmed
        self.approached = set()  # event_ids whose APPROACH_TARGET finished COMPLETED
        self.recheck_tasks = {}  # event_id -> (task_id, exit count it was issued for)
        self.rechecked_at_count = {}  # event_id -> exit count of the last COMPLETED recheck
        self.exit_pass_count = 0  # highest cumulative exit count seen; never goes back

    # -- reading Allocator state (single source) --------------------------
    def assignment(self, robot_id):
        """('PENDING'|'ACTIVE', Reservation) for the robot, or None."""
        for state, table in (('ACTIVE', self.allocator.active), ('PENDING', self.allocator.pending)):
            for item in table.values():
                if item.robot_id == robot_id:
                    return state, item
        return None

    @staticmethod
    def kind_of(item):
        return item.task_kind

    def _reserved_targets(self):
        return {r.target_id for r in tuple(self.allocator.pending.values()) + tuple(self.allocator.active.values())}

    def _approach_open(self):
        open_tasks = set(self.allocator.pending) | set(self.allocator.active)
        return any(task_id in open_tasks for task_id in self.approach_tasks.values())

    def excluded_areas(self):
        """Areas that are never executed, with the reason (for the operator view)."""
        result = []
        for area in self.policy.areas:
            if area.basis not in self.policy.executable_bases:
                result.append((area.site_id, 'BASIS_NOT_EXECUTABLE'))
            elif not area.bound_to_map():
                result.append((area.site_id, 'NO_MAP_REVISION'))
            elif area.site_id in self.withdrawn_areas:
                result.append((area.site_id, 'WITHDRAWN'))
        return result

    # -- proposals ---------------------------------------------------------
    def plan(self, robot_id, *, robot_boot_id, ready, now_ns):
        """Next role task for one robot. ``ready`` is the caller's Readiness.valid()."""
        robot(robot_id)
        uid(robot_boot_id)
        contract_value(timestamp_ns, now_ns)
        if type(ready) is not bool:
            raise CoordinationError('INVALID_INPUT', 'ready must be the boolean Readiness result')
        role = dict(self.policy.roles).get(robot_id)
        if role is None:
            raise CoordinationError('WRONG_ROBOT', 'robot has no role in this policy')
        current = self.assignment(robot_id)
        if current is not None:
            state, item = current
            if self.kind_of(item) == 'SEARCH_AREA' and item.target_id in self.withdrawn_areas:
                # Reassignment condition 2: cancel, confirm stop, then finish().
                return Decision('RELEASE_REQUIRED', 'AREA_WITHDRAWN', item)
            return Decision('KEEP', state, item)
        if not ready:
            return Decision('WAIT', 'NOT_READY')
        if role == 'SUPPORT':
            if self.held.get(robot_id) == robot_boot_id:
                return Decision('WAIT', 'HOLDING')
            # HOLD = stop and stay where the robot is (executor semantics,
            # 작전 2026-10-08). The target ID is a fresh token so the Allocator's
            # one-reservation-per-target rule never couples it to an area/event.
            return self._reserve(robot_id, robot_boot_id, 'HOLD', self.new_id(), 0, None, now_ns)
        if not self.policy.scout_continues_during_approach and self._approach_open():
            return Decision('WAIT', 'PAUSED_FOR_APPROACH')
        taken = self._reserved_targets()
        for area in self.policy.areas:
            if (self.policy.executable(area) and area.site_id not in self.completed_areas
                    and area.site_id not in self.withdrawn_areas and area.site_id not in taken):
                return self._reserve(robot_id, robot_boot_id, 'SEARCH_AREA', area.site_id,
                                     area.map_revision, None, now_ns, area=area)
        return Decision('WAIT', 'NO_EXECUTABLE_AREA')

    def request_approach(self, event_id, target_revision, *, projection_status, candidate,
                         robot_boot_id, now_ns):
        """Reserve the SUPPORT robot for one discovery event.

        ``candidate`` must be the ``select_candidate`` result for this target;
        ``projection_status`` is the mapping projection status of the source
        observation. Neither one authorizes motion: the executor guards do.
        """
        uid(event_id)
        revision(target_revision)
        uid(robot_boot_id)
        contract_value(timestamp_ns, now_ns)
        support = self.policy.role_of('SUPPORT')
        if event_id in self.approach_tasks:
            return Decision('KEEP', 'EVENT_ALREADY_HAS_APPROACH')
        if projection_status != APPROACH_PROJECTION_STATUS:
            return Decision('WAIT', 'PROJECTION_NOT_VALID')
        if (not isinstance(candidate, PathCandidate) or candidate.robot_id != support
                or candidate.goal_xy == candidate.target_xy):  # no heading toward the target
            return Decision('WAIT', 'NO_SUPPORT_CANDIDATE')
        if candidate.valid_until_ns <= now_ns:
            return Decision('WAIT', 'CANDIDATE_EXPIRED')
        current = self.assignment(support)
        if current is not None:
            state, item = current
            if self.kind_of(item) == 'HOLD':
                return Decision('RELEASE_REQUIRED', 'SUPPORT_HOLDING', item)
            return Decision('WAIT', 'SUPPORT_BUSY', item)
        decision = self._reserve(support, robot_boot_id, 'APPROACH_TARGET', event_id,
                                 target_revision, candidate, now_ns)
        if decision.action == 'RESERVED':
            self.approach_tasks[event_id] = decision.reservation.task_id
        return decision

    def withdraw_area(self, area_id):
        """Stop proposing an area (e.g. its map basis became invalid)."""
        uid(area_id)
        if not any(a.site_id == area_id for a in self.policy.areas):
            raise CoordinationError('INVALID_INPUT', 'unknown area')
        self.withdrawn_areas.add(area_id)

    def request_recheck(self, event_id, target_revision, *, exit_pass_count, projection_status, candidate,
                        robot_boot_id, now_ns):
        """Reserve a SCOUT re-observation of one event after the exit count rose (Part I 3.5.3).

        ``exit_pass_count`` is the operations ledger's cumulative EXIT_PASS count
        (``read_site_events``); it never decreases. The event's SUPPORT approach
        must have finished COMPLETED here. Each COMPLETED recheck consumes the
        count it was issued for; the next one needs a higher count. A recheck
        that ends otherwise may be issued again without a new exit. The result
        is only observation input for the rescue verdict, never a confirmation.
        """
        uid(event_id)
        revision(target_revision)
        uid(robot_boot_id)
        contract_value(timestamp_ns, now_ns)
        if type(exit_pass_count) is not int or exit_pass_count < self.exit_pass_count:
            raise CoordinationError('INVALID_INPUT', 'exit count must be a non-decreasing int')
        self.exit_pass_count = exit_pass_count
        scout = self.policy.role_of('SCOUT')
        if event_id in self.recheck_tasks:
            return Decision('KEEP', 'EVENT_ALREADY_HAS_RECHECK')
        if event_id not in self.approached:
            return Decision('WAIT', 'APPROACH_NOT_COMPLETED')
        if exit_pass_count <= self.rechecked_at_count.get(event_id, 0):
            return Decision('WAIT', 'NO_NEW_EXIT_PASS')
        if projection_status != APPROACH_PROJECTION_STATUS:
            return Decision('WAIT', 'PROJECTION_NOT_VALID')
        if (not isinstance(candidate, PathCandidate) or candidate.robot_id != scout
                or candidate.goal_xy == candidate.target_xy):  # the camera must face the target
            return Decision('WAIT', 'NO_SCOUT_CANDIDATE')
        if candidate.valid_until_ns <= now_ns:
            return Decision('WAIT', 'CANDIDATE_EXPIRED')
        current = self.assignment(scout)
        if current is not None:
            state, item = current
            if self.kind_of(item) == 'SEARCH_AREA':
                # Recheck first [제안]: cancel, confirm stop, finish(), then ask again.
                return Decision('RELEASE_REQUIRED', 'SCOUT_SEARCHING', item)
            return Decision('WAIT', 'SCOUT_BUSY', item)
        decision = self._reserve(scout, robot_boot_id, 'REOBSERVE_TARGET', event_id, target_revision,
                                 candidate, now_ns)
        if decision.action == 'RESERVED':
            self.recheck_tasks[event_id] = (decision.reservation.task_id, exit_pass_count)
        return decision

    # -- executor feedback -------------------------------------------------
    def record_admission(self, receipt):
        """The only path to ACTIVE; delegates unchanged to the Allocator."""
        return self.allocator.record_admission(receipt)

    def finish(self, item, *, outcome, stop_confirmed):
        """Apply an executor Result for ``item``; release needs a confirmed stop.

        A SEARCH_AREA counts as searched only on COMPLETED; other outcomes leave
        it unsearched for a later proposal. Returns the Allocator release result.
        """
        if not isinstance(item, Reservation):
            raise CoordinationError('INVALID_RESERVATION', 'reservation type')
        if outcome not in ('COMPLETED', 'CANCELED', 'EXPIRED', 'UNREACHABLE', 'FAILED'):
            raise CoordinationError('INVALID_INPUT', 'executor outcome')
        released = self.allocator.release(item.task_id, robot_id=item.robot_id, generation=item.generation,
                                          authority_epoch=item.authority_epoch, stop_confirmed=stop_confirmed)
        if not released:
            return False
        kind = self.kind_of(item)
        if kind == 'SEARCH_AREA' and outcome == 'COMPLETED':
            self.completed_areas.add(item.target_id)
        elif kind == 'HOLD' and outcome == 'COMPLETED':
            self.held[item.robot_id] = item.robot_boot_id
        elif kind == 'APPROACH_TARGET':
            self.held.pop(item.robot_id, None)
            if outcome == 'COMPLETED':
                self.approached.add(item.target_id)
        elif kind == 'REOBSERVE_TARGET':
            task_id, count = self.recheck_tasks.pop(item.target_id, (None, None))
            if task_id == item.task_id and outcome == 'COMPLETED':
                self.rechecked_at_count[item.target_id] = count
        return True

    def _goal_pose(self, kind, area, candidate):
        """Goal for the robot lane; no heading literal (CUR-14).

        SEARCH_AREA uses the area's configured ``entry_yaw``. APPROACH_TARGET faces
        the target from the candidate point, so the camera can re-observe it.
        HOLD stays in place and has no goal.
        """
        if kind == 'HOLD':
            return None
        if kind == 'SEARCH_AREA':
            source, xy, yaw = area, area.mission_xy, area.entry_yaw
        else:
            source, xy = candidate, candidate.goal_xy
            yaw = math.atan2(candidate.target_xy[1] - xy[1], candidate.target_xy[0] - xy[0])
        return dict(frame_id=self.policy.mission_frame_id, x=float(xy[0]), y=float(xy[1]), yaw=float(yaw),
                    map_session_id=source.map_session_id, map_revision=source.map_revision,
                    transform_revision=source.transform_revision)

    def _reserve(self, robot_id, robot_boot_id, kind, target_id, target_revision, candidate, now_ns,
                 *, area=None):
        deadline = now_ns + self.policy.task_duration_ns
        contract_value(timestamp_ns, deadline)
        generation = self.allocator.issue_generation(robot_id)
        item = Reservation(self.new_id(), target_id, target_revision, robot_id, generation,
                           self.allocator.authority_epoch, candidate, command_id=self.new_id(),
                           robot_boot_id=robot_boot_id, task_kind=kind)
        if not self.allocator.reserve(item):
            # The issued generation is burned, never reused (Allocator rule).
            return Decision('WAIT', 'ALLOCATOR_REJECTED')
        is_target = kind in ('APPROACH_TARGET', 'REOBSERVE_TARGET')
        fields = dict(command_id=item.command_id, task_id=item.task_id, robot_id=robot_id,
                      robot_boot_id=robot_boot_id, authority_epoch=item.authority_epoch,
                      generation=generation, kind=kind, issued_at_ns=now_ns, deadline_ns=deadline,
                      lease_duration=float(self.policy.lease_duration),
                      policy_version=self.policy.executor_policy_version,
                      target_id=target_id if is_target else None,
                      target_revision=target_revision if is_target else None,
                      search_area_ref=area.site_id if kind == 'SEARCH_AREA' else None,
                      goal_pose=self._goal_pose(kind, area, candidate))
        return Decision('RESERVED', 'PENDING_EXECUTOR_ADMISSION', item, fields)
