"""Path-evidence based proposals and conservative, in-memory corridor leases.

No ROS call or movement authorization is provided by this module.
"""
from dataclasses import dataclass
import math
from uuid import UUID


class CoordinationError(ValueError):
    pass


def finite(value, name, *, minimum=0):
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value >= minimum
    except OverflowError:
        valid = False
    if not valid:
        raise CoordinationError(name)
    return value


def uid(value):
    if not isinstance(value, str):
        raise CoordinationError('UUID')
    try:
        if str(UUID(value)) != value or UUID(value).int == 0:
            raise ValueError()
    except ValueError as exc:
        raise CoordinationError('canonical UUID') from exc
    return value


def revision(value):
    if type(value) is not int or not 0 <= value < 2 ** 64:
        raise CoordinationError('revision')
    return value


def robot(value):
    if value not in ('robot_a', 'robot_b'):
        raise CoordinationError('robot')
    return value


def point(value):
    if not isinstance(value, tuple) or len(value) != 2:
        raise CoordinationError('immutable XY point')
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
    confirmation_max_age: float

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise CoordinationError('policy version')
        for key in ('min_distance', 'max_distance', 'clearance', 'max_uncertainty',
                    'short_loss_max', 'confirmation_max_age'):
            finite(getattr(self, key), key)
        if self.min_distance <= 0 or self.max_distance <= self.min_distance:
            raise CoordinationError('distance interval')


@dataclass(frozen=True)
class PathCandidate:
    robot_id: str
    goal_xy: tuple
    target_xy: tuple
    path_length: float
    footprint_clearance: float
    uncertainty: float
    valid_until: float
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
        for key in ('path_length', 'footprint_clearance', 'uncertainty', 'valid_until'):
            finite(getattr(self, key), key)
        for key in ('footprint_free', 'reachable', 'path_collision_free'):
            if type(getattr(self, key)) is not bool:
                raise CoordinationError('boolean evidence required')


def select_candidate(candidates, *, now, policy, available_robots):
    """Choose a proposal by verified path cost, never target/robot straight line.

    Flags/geometry are external planner evidence; this function cannot establish
    them itself. The result does not authorize a robot goal.
    """
    finite(now, 'now')
    if not isinstance(policy, ApproachPolicy):
        raise CoordinationError('policy')
    available = {robot(r) for r in available_robots}
    accepted = []
    for candidate in candidates:
        if not isinstance(candidate, PathCandidate):
            raise CoordinationError('candidate type')
        c = candidate
        distance = math.dist(c.goal_xy, c.target_xy)
        if (c.robot_id in available and c.valid_until > now and c.footprint_free
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

    def __post_init__(self):
        for value in (self.task_id, self.target_id, self.authority_epoch):
            uid(value)
        for value in (self.target_revision, self.generation):
            revision(value)
        robot(self.robot_id)
        if not isinstance(self.candidate, PathCandidate) or self.candidate.robot_id != self.robot_id:
            raise CoordinationError('candidate owner')


class Allocator:
    """Logical assignment only. Single process; restart loses reservations."""
    def __init__(self, authority_epoch):
        self.authority_epoch = uid(authority_epoch)
        self.active = {}
        self.seen = {}
        self.generations = {'robot_a': -1, 'robot_b': -1}

    def reserve(self, item):
        if not isinstance(item, Reservation):
            raise CoordinationError('reservation type')
        if item.task_id in self.seen:
            if self.seen[item.task_id] != item:
                raise CoordinationError('ID_CONTENT_CONFLICT')
            return item.task_id in self.active
        if item.authority_epoch != self.authority_epoch:
            return False
        if item.generation <= self.generations[item.robot_id]:
            return False
        if any(r.robot_id == item.robot_id or r.target_id == item.target_id for r in self.active.values()):
            return False
        self.active[item.task_id] = item
        self.seen[item.task_id] = item
        self.generations[item.robot_id] = item.generation
        return True

    def release(self, task_id, *, robot_id, generation, authority_epoch, stop_confirmed):
        uid(task_id)
        robot(robot_id)
        revision(generation)
        uid(authority_epoch)
        item = self.active.get(task_id)
        if (item is None or type(stop_confirmed) is not bool or not stop_confirmed
                or (item.robot_id, item.generation, item.authority_epoch)
                != (robot_id, generation, authority_epoch)):
            return False
        del self.active[task_id]
        return True


class ApproachTracker:
    """Target freshness/verification after assignment; no identity classifier."""
    def __init__(self, target_id, target_revision, policy):
        self.target_id = uid(target_id)
        self.target_revision = revision(target_revision)
        if not isinstance(policy, ApproachPolicy):
            raise CoordinationError('policy')
        self.policy = policy
        self.last_observed = None
        self.last_received = None
        self.valid_until = None
        self.observation_ids = set()
        self.last_local = None
        self.canceled = False

    def _now(self, now):
        finite(now, 'steady time')
        if self.last_local is not None and now < self.last_local:
            self.canceled = True
            raise CoordinationError('steady clock regressed')
        self.last_local = now

    def observe(self, observation_id, *, observed_at, valid_until, now_wall, received_steady,
                target_id, target_revision, same_identity, freshness_verified):
        self._now(received_steady)
        uid(observation_id)
        uid(target_id)
        revision(target_revision)
        for value in (observed_at, valid_until, now_wall):
            finite(value, 'observation time')
        if (self.last_observed is not None and
                (now_wall >= self.valid_until or received_steady - self.last_received > self.policy.short_loss_max)):
            self.canceled = True
        if (self.canceled or type(same_identity) is not bool or not same_identity
                or type(freshness_verified) is not bool or not freshness_verified
                or target_id != self.target_id or target_revision != self.target_revision
                or observed_at <= 0 or not observed_at <= now_wall < valid_until
                or observation_id in self.observation_ids
                or (self.last_observed is not None and observed_at <= self.last_observed)):
            return False
        self.observation_ids.add(observation_id)
        self.last_observed, self.last_received, self.valid_until = observed_at, received_steady, valid_until
        return True

    def guard(self, *, now_wall, now_steady, local_guards_valid):
        self._now(now_steady)
        finite(now_wall, 'wall time')
        valid = (not self.canceled and type(local_guards_valid) is bool and local_guards_valid
                 and self.last_observed is not None and self.last_observed <= now_wall < self.valid_until
                 and 0 <= now_steady - self.last_received <= self.policy.short_loss_max)
        if not valid:
            self.canceled = True
        return 'CONTINUE_EXISTING' if valid else 'CANCEL'

    def complete(self, *, now_wall, now_steady, distance, uncertainty, nav_succeeded,
                 stop_confirmed, local_guards_valid, navigation_finished_at):
        finite(navigation_finished_at, 'navigation completion time')
        finite(distance, 'distance')
        finite(uncertainty, 'uncertainty')
        valid = self.guard(now_wall=now_wall, now_steady=now_steady, local_guards_valid=local_guards_valid)
        return (valid == 'CONTINUE_EXISTING' and type(nav_succeeded) is bool and nav_succeeded
                and type(stop_confirmed) is bool and stop_confirmed
                and 0 < navigation_finished_at < self.last_observed <= now_wall
                and now_wall - self.last_observed <= self.policy.confirmation_max_age
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
            raise CoordinationError('steady clock regressed')
        self.last_time = now

    def confirm_vacant(self, *, expected_revision, verified_exit, observation_fresh, now):
        self._time(now)
        if (type(expected_revision) is not int or expected_revision != self.revision
                or verified_exit is not True or observation_fresh is not True):
            return False
        self.state, self.holder, self.until = 'FREE', None, None
        self.revision += 1
        return True

    def acquire(self, holder, *, now, duration):
        self._time(now)
        finite(duration, 'lease duration')
        if not isinstance(holder, Reservation):
            raise CoordinationError('holder')
        if duration <= 0 or self.state != 'FREE':
            return False
        until = finite(now + duration, 'lease deadline')
        if until <= now:
            raise CoordinationError('lease duration below clock resolution')
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
            raise CoordinationError('lease duration below clock resolution')
        self.until, self.renewal = until, sequence
        return True
