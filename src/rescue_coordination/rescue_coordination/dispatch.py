"""Which confirmed target the SUPPORT robot approaches next (BRD Part I 3.4.3/3.4.5).

Q-19 is open: the distance is a policy value with no default. ``STRAIGHT_LINE``
(the proposed start) measures from the SUPPORT robot's mission-frame position to
the target; ``PATH_LENGTH`` uses the verified path cost of the SUPPORT candidate.
A straight line is compared only when the robot position and the candidate share
the same map session/revision/transform revision.

This only ranks. It reserves nothing: the caller passes the choice to
``SearchCoordinator.request_approach``, which keeps one approach per event.
"""
from dataclasses import dataclass
import math

from rescue_contracts import RESCUE_STATES, duration_ns, timestamp_ns

from .planning import CoordinationError, PathCandidate, contract_value, point, revision, robot, uid

DISPATCH_ORDERS = frozenset({'STRAIGHT_LINE', 'PATH_LENGTH'})
# Closed sets; no free text is returned (작전 2026-10-08).
DISPATCH_SKIP_REASONS = frozenset({
    'NOT_CONFIRMED', 'RESCUED', 'EVENT_ALREADY_HAS_APPROACH', 'PROJECTION_NOT_VALID',
    'NO_SUPPORT_CANDIDATE', 'CANDIDATE_EXPIRED', 'BASIS_MISMATCH'})
DISPATCH_RESULTS = frozenset({'SELECTED', 'NO_ELIGIBLE_TARGET', 'ORIGIN_UNUSABLE'})
# mapping.MappingRegistry.projection(...).status that may feed an approach.
APPROACH_PROJECTION_STATUS = 'VALID'


@dataclass(frozen=True)
class DispatchPolicy:
    """``order`` is required (Q-19); ``max_origin_age_ns`` bounds the robot position used for STRAIGHT_LINE."""
    version: str
    order: str
    max_origin_age_ns: int

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise CoordinationError('INVALID_POLICY', 'policy version')
        if not isinstance(self.order, str) or self.order not in DISPATCH_ORDERS:
            raise CoordinationError('INVALID_POLICY', 'dispatch order must be STRAIGHT_LINE or PATH_LENGTH')
        contract_value(duration_ns, self.max_origin_age_ns)


@dataclass(frozen=True)
class SupportOrigin:
    """The SUPPORT robot's position in the mission frame, with the map basis it was projected under."""
    robot_id: str
    xy: tuple
    map_session_id: str
    map_revision: int
    transform_revision: int
    stamp_ns: int

    def __post_init__(self):
        robot(self.robot_id)
        point(self.xy)
        uid(self.map_session_id)
        revision(self.map_revision)
        revision(self.transform_revision)
        contract_value(timestamp_ns, self.stamp_ns)


@dataclass(frozen=True)
class DispatchTarget:
    """One discovery event as the dispatcher sees it.

    ``detection_confirmed`` is the recognition confirmed/unconfirmed split
    (Part I 3.3.5: only confirmed targets are dispatched to). ``candidate`` is the
    SUPPORT ``select_candidate`` result for this target, or None.
    """
    event_id: str
    target_revision: int
    detection_confirmed: bool
    projection_status: str
    rescue_state: str
    candidate: PathCandidate | None

    def __post_init__(self):
        uid(self.event_id)
        revision(self.target_revision)
        if type(self.detection_confirmed) is not bool:
            raise CoordinationError('INVALID_INPUT', 'detection_confirmed')
        if not isinstance(self.projection_status, str) or not self.projection_status:
            raise CoordinationError('INVALID_INPUT', 'projection_status')
        if self.rescue_state not in RESCUE_STATES:
            raise CoordinationError('INVALID_INPUT', 'rescue_state')
        if self.candidate is not None and not isinstance(self.candidate, PathCandidate):
            raise CoordinationError('INVALID_INPUT', 'candidate type')


@dataclass(frozen=True)
class DispatchChoice:
    reason: str
    target: DispatchTarget | None
    ranked: tuple   # eligible event IDs, best first
    skipped: tuple  # ((event_id, reason), ...) in input order

    def __post_init__(self):
        if self.reason not in DISPATCH_RESULTS or not {r for _, r in self.skipped} <= DISPATCH_SKIP_REASONS:
            raise CoordinationError('INVALID_INPUT', 'dispatch reason outside the closed set')
        if (self.target is not None) != (self.reason == 'SELECTED'):
            raise CoordinationError('INVALID_INPUT', 'a target exactly when SELECTED')


def _origin_usable(origin, support_robot_id, policy, now_ns):
    return (isinstance(origin, SupportOrigin) and origin.robot_id == support_robot_id
            and origin.stamp_ns <= now_ns and now_ns - origin.stamp_ns <= policy.max_origin_age_ns)


def _basis(item):
    return item.map_session_id, item.map_revision, item.transform_revision


def select_dispatch_target(targets, *, policy, support_robot_id, assigned_events, origin, now_ns):
    """Rank eligible targets by the policy distance; ties go to the smaller event ID.

    ``assigned_events`` are events that already have an approach reservation
    (``SearchCoordinator.approach_tasks``); they are never chosen again.
    """
    if not isinstance(policy, DispatchPolicy):
        raise CoordinationError('INVALID_POLICY', 'dispatch policy')
    robot(support_robot_id)
    contract_value(timestamp_ns, now_ns)
    targets = tuple(targets)
    if not all(isinstance(t, DispatchTarget) for t in targets):
        raise CoordinationError('INVALID_INPUT', 'targets must be DispatchTarget')
    ids = [t.event_id for t in targets]
    if len(set(ids)) != len(ids):
        raise CoordinationError('INVALID_INPUT', 'duplicate event_id')
    straight = policy.order == 'STRAIGHT_LINE'
    if straight and not _origin_usable(origin, support_robot_id, policy, now_ns):
        return DispatchChoice('ORIGIN_UNUSABLE', None, (), ())
    skipped, scored = [], []
    for item in targets:
        c = item.candidate
        if not item.detection_confirmed:
            reason = 'NOT_CONFIRMED'
        elif item.rescue_state == 'RESCUE_CONFIRMED':
            reason = 'RESCUED'
        elif item.event_id in assigned_events:
            reason = 'EVENT_ALREADY_HAS_APPROACH'
        elif item.projection_status != APPROACH_PROJECTION_STATUS:
            reason = 'PROJECTION_NOT_VALID'
        elif c is None or c.robot_id != support_robot_id or c.goal_xy == c.target_xy:
            reason = 'NO_SUPPORT_CANDIDATE'
        elif c.valid_until_ns <= now_ns:
            reason = 'CANDIDATE_EXPIRED'
        elif straight and _basis(c) != _basis(origin):
            reason = 'BASIS_MISMATCH'
        else:
            reason = None
        if reason is not None:
            skipped.append((item.event_id, reason))
            continue
        metric = (math.hypot(c.target_xy[0] - origin.xy[0], c.target_xy[1] - origin.xy[1])
                  if straight else c.path_length)
        scored.append((metric, item.event_id, item))
    scored.sort(key=lambda entry: entry[:2])
    ranked = tuple(event_id for _, event_id, _ in scored)
    if not scored:
        return DispatchChoice('NO_ELIGIBLE_TARGET', None, ranked, tuple(skipped))
    return DispatchChoice('SELECTED', scored[0][2], ranked, tuple(skipped))
