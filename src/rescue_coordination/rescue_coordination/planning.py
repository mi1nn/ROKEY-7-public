"""Path-evidence based proposals and conservative, in-memory corridor leases.

No ROS call or movement authorization is provided by this module.
"""
from dataclasses import dataclass
import math
from uuid import UUID

from rescue_contracts import (RescueError, canonical_code, robot_id, robot_ids,
                              timestamp_ns, duration_ns, TaskAdmission)


class CoordinationError(RescueError):
    def __init__(self, reason, detail=''):
        code = canonical_code('rescue_coordination', reason)
        super().__init__(code, detail or reason, component='rescue_coordination', detail_code=reason)


def contract_value(check, value):
    try:
        return check(value)
    except RescueError as exc:
        raise CoordinationError('INVALID_INPUT', exc.detail) from exc


def finite(value, name, *, minimum=0):
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value >= minimum
    except OverflowError:
        valid = False
    if not valid:
        raise CoordinationError('INVALID_NUMBER', name)
    return value


def uid(value):
    if not isinstance(value, str):
        raise CoordinationError('INVALID_UUID', 'UUID')
    try:
        if str(UUID(value)) != value or UUID(value).int == 0:
            raise ValueError()
    except ValueError as exc:
        raise CoordinationError('INVALID_UUID', 'canonical UUID') from exc
    return value


def revision(value):
    if type(value) is not int or not 0 <= value < 2 ** 64:
        raise CoordinationError('INVALID_REVISION', 'revision')
    return value


def robot(value):
    return contract_value(robot_id, value)


def point(value):
    if not isinstance(value, tuple) or len(value) != 2:
        raise CoordinationError('INVALID_POINT', 'immutable XY point')
    for v in value:
        finite(v, 'coordinate', minimum=-math.inf)
    return value


@dataclass(frozen=True)
class ApproachPolicy:
    version: str
    min_distance: float
    max_distance: float
    clearance: float
    max_uncertainty: float
    short_loss_max: float
    confirmation_max_age_ns: int

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise CoordinationError('INVALID_POLICY', 'policy version')
        for key in ('min_distance', 'max_distance', 'clearance', 'max_uncertainty',
                    'short_loss_max'):
            finite(getattr(self, key), key)
        contract_value(duration_ns, self.confirmation_max_age_ns)
        if self.min_distance <= 0 or self.max_distance <= self.min_distance:
            raise CoordinationError('INVALID_POLICY', 'distance interval')


@dataclass(frozen=True)
class PathCandidate:
    robot_id: str
    goal_xy: tuple
    target_xy: tuple
    path_length: float
    footprint_clearance: float
    uncertainty: float
    valid_until_ns: int
    map_session_id: str
    map_revision: int
    transform_revision: int
    footprint_free: bool
    reachable: bool
    path_collision_free: bool

    def __post_init__(self):
        robot(self.robot_id)
        point(self.goal_xy)
        point(self.target_xy)
        uid(self.map_session_id)
        revision(self.map_revision)
        revision(self.transform_revision)
        contract_value(timestamp_ns, self.valid_until_ns)
        for key in ('path_length', 'footprint_clearance', 'uncertainty'):
            finite(getattr(self, key), key)
        for key in ('footprint_free', 'reachable', 'path_collision_free'):
            if type(getattr(self, key)) is not bool:
                raise CoordinationError('INVALID_CANDIDATE', 'boolean evidence required')


def select_candidate(candidates, *, now_ns, policy, available_robots):
    """Choose a proposal by verified path cost, never target/robot straight line.

    Flags/geometry are external planner evidence; this function cannot establish
    them itself. The result does not authorize a robot goal.
    """
    contract_value(timestamp_ns, now_ns)
    if not isinstance(policy, ApproachPolicy):
        raise CoordinationError('INVALID_POLICY', 'policy')
    available = {robot(r) for r in available_robots}
    accepted = []
    for candidate in candidates:
        if not isinstance(candidate, PathCandidate):
            raise CoordinationError('INVALID_CANDIDATE', 'candidate type')
        c = candidate
        distance = math.dist(c.goal_xy, c.target_xy)
        if (c.robot_id in available and c.valid_until_ns > now_ns and c.footprint_free
                and c.reachable and c.path_collision_free and c.uncertainty <= policy.max_uncertainty
                and c.footprint_clearance >= policy.clearance
                and distance - c.uncertainty >= policy.min_distance
                and distance + c.uncertainty <= policy.max_distance):
            accepted.append(c)
    return min(accepted, key=lambda c: (c.path_length, c.robot_id, c.goal_xy)) if accepted else None


@dataclass(frozen=True)
class Reservation:
    task_id: str
    target_id: str
    target_revision: int
    robot_id: str
    generation: int
    authority_epoch: str
    candidate: PathCandidate
    command_id: str
    robot_boot_id: str

    def __post_init__(self):
        for value in (self.task_id, self.target_id, self.authority_epoch, self.command_id, self.robot_boot_id):
            uid(value)
        for value in (self.target_revision, self.generation):
            revision(value)
        robot(self.robot_id)
        if not isinstance(self.candidate, PathCandidate) or self.candidate.robot_id != self.robot_id:
            raise CoordinationError('INVALID_RESERVATION', 'candidate owner')


def admission_identity(value):
    return (value.command_id, value.task_id, value.robot_id, value.robot_boot_id,
            value.authority_epoch, value.generation)


class Allocator:
    """Issue generations; only the executor admission receipt activates a task.

    Pending requests keep both resources reserved until rejection or verified
    stop. Local state is not durable and does not authenticate network receipts.
    """
    def __init__(self, authority_epoch, *, allowed_robot_ids):
        self.authority_epoch = uid(authority_epoch)
        self.allowed_robot_ids = contract_value(robot_ids, allowed_robot_ids)
        self.pending = {}
        self.active = {}
        self.seen = {}
        self.commands = {}
        self.admissions = {}
        self.used_generations = set()
        self.issued_generations = {r: -1 for r in self.allowed_robot_ids}
        self.accepted_generations = {r: -1 for r in self.allowed_robot_ids}

    def issue_generation(self, robot_id):
        robot(robot_id)
        if robot_id not in self.allowed_robot_ids:
            raise CoordinationError('WRONG_ROBOT', 'robot is outside this deployment')
        value = revision(self.issued_generations[robot_id] + 1)
        self.issued_generations[robot_id] = value
        return value

    def reserve(self, item):
        if not isinstance(item, Reservation):
            raise CoordinationError('INVALID_RESERVATION', 'reservation type')
        if item.task_id in self.seen:
            if self.seen[item.task_id] != item:
                raise CoordinationError('ID_CONTENT_CONFLICT')
            return item.task_id in self.pending or item.task_id in self.active
        if item.command_id in self.commands:
            raise CoordinationError('ID_CONTENT_CONFLICT', 'command_id already belongs to a different task')
        if item.authority_epoch != self.authority_epoch or item.robot_id not in self.allowed_robot_ids:
            return False
        key = (item.robot_id, item.generation)
        if (item.generation > self.issued_generations[item.robot_id]
                or item.generation <= self.accepted_generations[item.robot_id]
                or key in self.used_generations):
            return False
        occupied = tuple(self.pending.values()) + tuple(self.active.values())
        if any(r.robot_id == item.robot_id or r.target_id == item.target_id for r in occupied):
            return False
        self.pending[item.task_id] = item
        self.seen[item.task_id] = item
        self.commands[item.command_id] = item.task_id
        self.used_generations.add(key)
        return True

    def record_admission(self, receipt):
        if not isinstance(receipt, TaskAdmission):
            raise CoordinationError('INVALID_ADMISSION', 'admission receipt type')
        item = self.seen.get(receipt.task_id)
        if item is None or admission_identity(receipt) != admission_identity(item):
            return False
        prior = self.admissions.get(receipt.task_id)
        if prior is not None:
            if receipt != prior:
                raise CoordinationError('ID_CONTENT_CONFLICT', 'executor admission receipt changed')
            return receipt.task_id in self.active
        if receipt.task_id not in self.pending:
            # A verified stop released the pending request. A delayed receipt
            # cannot reactivate it or mutate accepted generation accounting.
            return False
        self.admissions[receipt.task_id] = receipt
        del self.pending[receipt.task_id]
        if not receipt.accepted:
            return False
        if receipt.generation <= self.accepted_generations[receipt.robot_id]:
            return False
        self.active[receipt.task_id] = item
        self.accepted_generations[receipt.robot_id] = receipt.generation
        return True

    def release(self, task_id, *, robot_id, generation, authority_epoch, stop_confirmed):
        uid(task_id)
        robot(robot_id)
        revision(generation)
        uid(authority_epoch)
        item = self.active.get(task_id) or self.pending.get(task_id)
        if (item is None or type(stop_confirmed) is not bool or not stop_confirmed
                or (item.robot_id, item.generation, item.authority_epoch)
                != (robot_id, generation, authority_epoch)):
            return False
        self.active.pop(task_id, None)
        self.pending.pop(task_id, None)
        return True


class ApproachTracker:
    """Target freshness/verification after assignment; no identity classifier."""
    def __init__(self, target_id, target_revision, policy):
        self.target_id = uid(target_id)
        self.target_revision = revision(target_revision)
        if not isinstance(policy, ApproachPolicy):
            raise CoordinationError('INVALID_POLICY', 'policy')
        self.policy = policy
        self.last_observed_ns = None
        self.last_received = None
        self.valid_until_ns = None
        self.observation_ids = set()
        self.last_local = None
        self.last_wall_ns = None
        self.canceled = False

    def _now(self, now):
        finite(now, 'steady time')
        if self.last_local is not None and now < self.last_local:
            self.canceled = True
            raise CoordinationError('CLOCK_UNCERTAIN', 'steady clock regressed')
        self.last_local = now

    def _wall(self, value):
        contract_value(timestamp_ns, value)
        if self.last_wall_ns is not None and value < self.last_wall_ns:
            self.canceled = True
            raise CoordinationError('CLOCK_UNCERTAIN', 'wall clock regressed')
        self.last_wall_ns = value

    def observe(self, observation_id, *, observed_at_ns, valid_until_ns, now_wall_ns, received_steady,
                target_id, target_revision, same_identity, freshness_verified):
        self._now(received_steady)
        uid(observation_id)
        uid(target_id)
        revision(target_revision)
        for value in (observed_at_ns, valid_until_ns):
            contract_value(timestamp_ns, value)
        self._wall(now_wall_ns)
        if (self.last_observed_ns is not None and
                (now_wall_ns >= self.valid_until_ns or received_steady - self.last_received > self.policy.short_loss_max)):
            self.canceled = True
        if (self.canceled or type(same_identity) is not bool or not same_identity
                or type(freshness_verified) is not bool or not freshness_verified
                or target_id != self.target_id or target_revision != self.target_revision
                or not observed_at_ns <= now_wall_ns < valid_until_ns
                or observation_id in self.observation_ids
                or (self.last_observed_ns is not None and observed_at_ns <= self.last_observed_ns)):
            return False
        self.observation_ids.add(observation_id)
        self.last_observed_ns, self.last_received, self.valid_until_ns = observed_at_ns, received_steady, valid_until_ns
        return True

    def guard(self, *, now_wall_ns, now_steady, local_guards_valid):
        self._now(now_steady)
        self._wall(now_wall_ns)
        valid = (not self.canceled and type(local_guards_valid) is bool and local_guards_valid
                 and self.last_observed_ns is not None and self.last_observed_ns <= now_wall_ns < self.valid_until_ns
                 and 0 <= now_steady - self.last_received <= self.policy.short_loss_max)
        if not valid:
            self.canceled = True
        return 'CONTINUE_EXISTING' if valid else 'CANCEL'

    def complete(self, *, now_wall_ns, now_steady, distance, uncertainty, nav_succeeded,
                 stop_confirmed, local_guards_valid, navigation_finished_at_ns):
        contract_value(timestamp_ns, navigation_finished_at_ns)
        finite(distance, 'distance')
        finite(uncertainty, 'uncertainty')
        valid = self.guard(now_wall_ns=now_wall_ns, now_steady=now_steady, local_guards_valid=local_guards_valid)
        return (valid == 'CONTINUE_EXISTING' and type(nav_succeeded) is bool and nav_succeeded
                and type(stop_confirmed) is bool and stop_confirmed
                and navigation_finished_at_ns < self.last_observed_ns <= now_wall_ns
                and now_wall_ns - self.last_observed_ns <= self.policy.confirmation_max_age_ns
                and uncertainty <= self.policy.max_uncertainty
                and distance - uncertainty >= self.policy.min_distance
                and distance + uncertainty <= self.policy.max_distance)


class Corridor:
    """Expiry removes authority but does not prove physical vacancy."""
    def __init__(self):
        self.state = 'OCCUPANCY_UNKNOWN'
        self.holder = None
        self.until = None
        self.revision = 0
        self.renewal = -1
        self.last_time = None

    def _time(self, now):
        finite(now, 'steady time')
        if self.last_time is not None and now < self.last_time:
            self.state = 'OCCUPANCY_UNKNOWN'
            raise CoordinationError('CLOCK_UNCERTAIN', 'steady clock regressed')
        self.last_time = now

    def confirm_vacant(self, *, holder, expected_revision, verified_exit, observation_fresh, now):
        self._time(now)
        if (type(expected_revision) is not int or expected_revision != self.revision
                or verified_exit is not True or observation_fresh is not True):
            return False
        if self.holder is None:
            if holder is not None:
                return False
        elif not isinstance(holder, Reservation) or admission_identity(holder) != admission_identity(self.holder):
            return False
        self.state, self.holder, self.until = 'FREE', None, None
        self.revision += 1
        return True

    def acquire(self, holder, *, now, duration):
        self._time(now)
        finite(duration, 'lease duration')
        if not isinstance(holder, Reservation):
            raise CoordinationError('INVALID_HOLDER', 'holder')
        if duration <= 0 or self.state != 'FREE':
            return False
        until = finite(now + duration, 'lease deadline')
        if until <= now:
            raise CoordinationError('LEASE_BELOW_RESOLUTION', 'lease duration below clock resolution')
        self.state, self.holder, self.until = 'HELD', holder, until
        self.revision += 1
        self.renewal = -1
        return True

    def check(self, *, now, holder_pose_fresh):
        self._time(now)
        if self.state == 'HELD' and (now >= self.until or holder_pose_fresh is not True):
            self.state = 'OCCUPANCY_UNKNOWN'
        return self.state

    def renew(self, holder, *, expected_revision, sequence, now, duration, holder_pose_fresh):
        revision(sequence)
        finite(duration, 'duration')
        self.check(now=now, holder_pose_fresh=holder_pose_fresh)
        if (self.state != 'HELD' or holder != self.holder or expected_revision != self.revision
                or type(expected_revision) is not int or sequence <= self.renewal or duration <= 0):
            return False
        until = finite(now + duration, 'lease deadline')
        if until <= now:
            raise CoordinationError('LEASE_BELOW_RESOLUTION', 'lease duration below clock resolution')
        self.until, self.renewal = until, sequence
        return True
