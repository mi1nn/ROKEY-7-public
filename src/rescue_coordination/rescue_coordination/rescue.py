"""Rescue-completion verdicts (BRD v0.1 UC-07, FR-20~24, KPI-06, S-05 rescue axis).

RESCUE_CONFIRMED needs three positive pieces of evidence for one discovery
event: the SCOUT actually observed the target point, that observation saw no
person and LiDAR saw no obstacle there, and an exit EXIT_PASS is linked to the
event. Unknown (None) is never evidence. "Nothing detected" counts only together
with proof that the point was in view; absence alone never confirms (KPI-06).

This module only judges. It assigns no REOBSERVE_TARGET task (handoff PR) and
does not decide what the operator screen shows (operations, after PERSISTED).
"""
from dataclasses import dataclass
from uuid import uuid4

from rescue_contracts import RESCUE_STATES, TASK_KINDS, SiteEvent, duration_ns, timestamp_ns

from .planning import CoordinationError, contract_value, revision, robot, uid

# Closed set of reasons for RECHECK_NEEDED (작전 decision 2026-10-08).
RESCUE_HOLD_REASONS = frozenset({
    'TASK_NOT_COMPLETED', 'RECHECK_ROBOT_NOT_SCOUT', 'TARGET_NOT_OBSERVED',
    'PERSON_PRESENT_OR_UNKNOWN', 'OBSTACLE_PRESENT_OR_UNKNOWN', 'EVIDENCE_STALE',
    'MAP_BASIS_CHANGED', 'EXIT_NOT_LINKED', 'EXIT_TIME_BASIS_UNUSABLE', 'TIME_BASIS_INCOMPARABLE',
    'ORDER_VIOLATION', 'EXIT_COUNT_EXCEEDED'})
# Q-13 is open: a demo with one target may link the exit pass without an
# identity match, only if the policy lists this basis explicitly (Q-R3).
SINGLE_TARGET_SCENARIO = 'SINGLE_TARGET_SCENARIO'


def _tri(value, name):
    """True/False is evidence; None means unknown. Anything else is invalid."""
    if value is not None and type(value) is not bool:
        raise CoordinationError('INVALID_INPUT', name + ' must be bool or None')
    return value


def _time_basis(domain, error_bound_ns):
    if not isinstance(domain, str) or not domain:
        raise CoordinationError('INVALID_INPUT', 'clock_domain_id')
    if error_bound_ns is not None:
        contract_value(duration_ns, error_bound_ns)


@dataclass(frozen=True)
class RecheckEvidence:
    """SCOUT re-observation of the target point. 임시 DTO, 공통화 예정(rescue_contracts).

    Producers: perception (target_in_view, depth_valid_at_target,
    person_detected_at_target) and the robot lane (REOBSERVE task result).
    """
    target_id: str
    observation_id: str
    robot_id: str
    task_id: str
    generation: int
    task_kind: str
    task_outcome: str
    stop_confirmed: bool
    observed_at_ns: int
    map_session_id: str
    map_revision: int
    target_in_view: bool | None
    depth_valid_at_target: bool | None
    person_detected_at_target: bool | None
    clock_domain_id: str
    clock_error_bound_ns: int | None

    def __post_init__(self):
        _time_basis(self.clock_domain_id, self.clock_error_bound_ns)
        for value in (self.target_id, self.observation_id, self.task_id, self.map_session_id):
            uid(value)
        robot(self.robot_id)
        revision(self.generation)
        revision(self.map_revision)
        if self.task_kind not in TASK_KINDS:
            raise CoordinationError('INVALID_INPUT', 'task kind')
        if self.task_outcome not in ('COMPLETED', 'CANCELED', 'EXPIRED', 'UNREACHABLE', 'FAILED'):
            raise CoordinationError('INVALID_INPUT', 'executor outcome')
        if type(self.stop_confirmed) is not bool:
            raise CoordinationError('INVALID_INPUT', 'stop_confirmed')
        contract_value(timestamp_ns, self.observed_at_ns)
        for name in ('target_in_view', 'depth_valid_at_target', 'person_detected_at_target'):
            _tri(getattr(self, name), name)


@dataclass(frozen=True)
class LidarCheck:
    """LiDAR obstacle judgement at the target point. 임시 DTO, 공통화 예정(rescue_contracts)."""
    target_id: str
    robot_id: str
    scan_at_ns: int
    map_session_id: str
    map_revision: int
    target_in_scan_range: bool | None
    obstacle_at_target: bool | None
    clock_domain_id: str
    clock_error_bound_ns: int | None

    def __post_init__(self):
        _time_basis(self.clock_domain_id, self.clock_error_bound_ns)
        uid(self.target_id)
        uid(self.map_session_id)
        robot(self.robot_id)
        revision(self.map_revision)
        contract_value(timestamp_ns, self.scan_at_ns)
        _tri(self.target_in_scan_range, 'target_in_scan_range')
        _tri(self.obstacle_at_target, 'obstacle_at_target')


@dataclass(frozen=True)
class ExitLink:
    """An exit-camera EXIT_PASS tied to one discovery event. 임시 DTO, 공통화 예정(rescue_contracts)."""
    site_event: SiteEvent
    target_id: str
    link_basis: str

    def __post_init__(self):
        if not isinstance(self.site_event, SiteEvent) or self.site_event.kind != 'EXIT_PASS':
            raise CoordinationError('INVALID_INPUT', 'EXIT_PASS site event required')
        uid(self.target_id)
        if not isinstance(self.link_basis, str) or not self.link_basis:
            raise CoordinationError('INVALID_INPUT', 'link basis')


@dataclass(frozen=True)
class RescuePolicy:
    """Rescue confirmation gates. 임시 DTO, 공통화 예정(rescue_contracts).

    Decided by: 작전·총괄 2026-10-08 (Q-10, Q-R1~R4). Numbers are run inputs, not
    locked thresholds (Q-06). ``max_clock_error_sum_ns`` caps the summed clock
    error bounds of two stamps before they may be ordered (C-03). ``lidar_detects_target_measured`` is the TR-12
    measurement result: if LiDAR has not been shown to see the target object,
    "no obstacle" from LiDAR is not evidence and nothing is confirmed.
    ``accepted_link_bases`` has no default; SINGLE_TARGET_SCENARIO must be named.
    """
    version: str
    max_sensor_skew_ns: int
    max_recheck_age_ns: int
    max_clock_error_sum_ns: int
    min_exit_confidence: float
    accepted_link_bases: frozenset
    lidar_detects_target_measured: bool

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise CoordinationError('INVALID_POLICY', 'policy version')
        contract_value(duration_ns, self.max_sensor_skew_ns)
        contract_value(duration_ns, self.max_recheck_age_ns)
        contract_value(duration_ns, self.max_clock_error_sum_ns)
        if (type(self.min_exit_confidence) is not float
                or not 0.0 <= self.min_exit_confidence <= 1.0):
            raise CoordinationError('INVALID_POLICY', 'min_exit_confidence in [0, 1]')
        if not isinstance(self.accepted_link_bases, frozenset) or not all(
                isinstance(b, str) and b for b in self.accepted_link_bases):
            raise CoordinationError('INVALID_POLICY', 'accepted_link_bases')
        if type(self.lidar_detects_target_measured) is not bool:
            raise CoordinationError('INVALID_POLICY', 'lidar_detects_target_measured')


@dataclass(frozen=True)
class TargetBasis:
    """Facts about the event that come from elsewhere, read at judgement time.

    ``scout_robot_id`` is the policy role (never fixed here). Map session/revision
    and ``projection_valid`` come from the TargetEstimate/mapping projection.
    ``projection_valid`` gates only judgements before a confirmation: a rescue is
    a conclusion about a past fact, not a spatial projection, so a later map
    change does not take a RESCUE_CONFIRMED back (see RescueBoard.evaluate);
    ``approach_completed_at_ns`` from the SUPPORT approach result (None = not done).
    ``open_target_count`` is the number of open discovery events (Q-R3).
    ``approach_clock_*`` is the time basis of ``approach_completed_at_ns``.
    ``exit_pass_count`` is the cumulative EXIT_PASS SiteEvent count the operations
    ledger holds (recognition dedups tracks; it is not deduplicated again here,
    #21 TS-04). Confirmed rescues may never exceed it (BRD Part I 3.5.5).
    """
    target_id: str
    scout_robot_id: str
    map_session_id: str
    map_revision: int
    projection_valid: bool
    approach_completed_at_ns: int | None
    open_target_count: int
    approach_clock_domain_id: str
    approach_clock_error_bound_ns: int | None
    exit_pass_count: int

    def __post_init__(self):
        _time_basis(self.approach_clock_domain_id, self.approach_clock_error_bound_ns)
        uid(self.target_id)
        uid(self.map_session_id)
        robot(self.scout_robot_id)
        revision(self.map_revision)
        if type(self.projection_valid) is not bool:
            raise CoordinationError('INVALID_INPUT', 'projection_valid')
        if self.approach_completed_at_ns is not None:
            contract_value(timestamp_ns, self.approach_completed_at_ns)
        if type(self.open_target_count) is not int or self.open_target_count < 1:
            raise CoordinationError('INVALID_INPUT', 'open_target_count')
        if type(self.exit_pass_count) is not int or self.exit_pass_count < 0:
            raise CoordinationError('INVALID_INPUT', 'exit_pass_count')


@dataclass(frozen=True)
class RescueVerdict:
    """One recorded judgement. History is kept; nothing is overwritten (S-05).

    ``demoted_from_confirmed`` marks a CONFIRMED that later evidence took back;
    KPI-06 counts it as one misjudgement. The report for it must be a new
    report revision/ID, never an overwrite (operations).
    """
    verdict_id: str
    target_id: str
    state: str
    reasons: frozenset
    link_basis: str | None
    evaluated_at_ns: int
    evidence_version: int
    policy_version: str
    demoted_from_confirmed: bool = False
    previous_verdict_id: str | None = None
    previous_confirmed_at_ns: int | None = None

    def __post_init__(self):
        if self.state not in RESCUE_STATES or not self.reasons <= RESCUE_HOLD_REASONS:
            raise CoordinationError('INVALID_INPUT', 'verdict state/reason outside the closed set')
        if bool(self.reasons) != (self.state == 'RECHECK_NEEDED'):
            raise CoordinationError('INVALID_INPUT', 'hold reasons exactly when RECHECK_NEEDED')


class RescueBoard:
    """Per-event evidence and verdict history. In memory, not durable."""

    def __init__(self, policy, *, new_id=None):
        if not isinstance(policy, RescuePolicy):
            raise CoordinationError('INVALID_POLICY', 'policy')
        self.policy = policy
        self.new_id = new_id or (lambda: str(uuid4()))
        self.rechecks = {}  # target_id -> latest RecheckEvidence
        self.lidar = {}  # target_id -> latest LidarCheck
        self.exit_links = {}  # target_id -> ExitLink
        self.exit_owner = {}  # site event_id -> target_id (FR-19: one exit pass, one event)
        self.evidence_version = {}  # target_id -> count of recorded evidence changes
        self.history = {}  # target_id -> [RescueVerdict, ...]
        self.exit_pass_count = 0  # highest cumulative exit count seen; never goes back

    def _bump(self, target_id):
        self.evidence_version[target_id] = self.evidence_version.get(target_id, 0) + 1

    def record_recheck(self, evidence):
        if not isinstance(evidence, RecheckEvidence):
            raise CoordinationError('INVALID_INPUT', 'recheck evidence type')
        prior = self.rechecks.get(evidence.target_id)
        if prior == evidence:
            return False
        if prior is not None and evidence.observed_at_ns <= prior.observed_at_ns:
            return False  # an older or same-time recheck never replaces a newer one
        self.rechecks[evidence.target_id] = evidence
        self._bump(evidence.target_id)
        return True

    def record_lidar(self, check):
        if not isinstance(check, LidarCheck):
            raise CoordinationError('INVALID_INPUT', 'lidar check type')
        prior = self.lidar.get(check.target_id)
        if prior == check or (prior is not None and check.scan_at_ns <= prior.scan_at_ns):
            return False
        self.lidar[check.target_id] = check
        self._bump(check.target_id)
        return True

    def link_exit(self, link):
        if not isinstance(link, ExitLink):
            raise CoordinationError('INVALID_INPUT', 'exit link type')
        event_id = link.site_event.event_id
        if self.exit_links.get(link.target_id) == link:
            return False  # identical resend is idempotent
        if event_id in self.exit_owner or link.target_id in self.exit_links:
            raise CoordinationError('ID_CONTENT_CONFLICT', 'exit pass or event already linked')
        self.exit_links[link.target_id] = link
        self.exit_owner[event_id] = link.target_id
        self._bump(link.target_id)
        return True

    def latest(self, target_id):
        history = self.history.get(target_id)
        return history[-1] if history else None

    def evaluate(self, basis, *, now_ns):
        """Judge one event at ``now_ns`` (the caller's clock; no internal clock).

        A CONFIRMED verdict is re-judged only when new evidence was recorded after
        it; ageing or a later map revision alone does not take a rescue back.
        """
        if not isinstance(basis, TargetBasis):
            raise CoordinationError('INVALID_INPUT', 'target basis type')
        contract_value(timestamp_ns, now_ns)
        target = basis.target_id
        version = self.evidence_version.get(target, 0)
        last = self.latest(target)
        if last is not None and now_ns < last.evaluated_at_ns:
            raise CoordinationError('CLOCK_UNCERTAIN', 'judgement clock regressed')
        if basis.exit_pass_count < self.exit_pass_count:
            raise CoordinationError('INVALID_INPUT', 'exit count regressed')
        self.exit_pass_count = basis.exit_pass_count
        if last is not None and last.state == 'RESCUE_CONFIRMED' and last.evidence_version == version:
            return last
        recheck = self.rechecks.get(target)
        link = self.exit_links.get(target)
        if recheck is None:
            state, reasons = 'NOT_CHECKED', frozenset()
        else:
            reasons = set(self._hold_reasons(basis, recheck, self.lidar.get(target), link, now_ns))
            # Aggregate cap on top of the 1:1 exit link (Part I 3.5.5): this event
            # may join the confirmed set only while the exit count has room.
            confirmed = sum(1 for other, h in self.history.items()
                            if other != target and h[-1].state == 'RESCUE_CONFIRMED')
            if confirmed + 1 > basis.exit_pass_count:
                reasons.add('EXIT_COUNT_EXCEEDED')
            reasons = frozenset(reasons)
            state = 'RECHECK_NEEDED' if reasons else 'RESCUE_CONFIRMED'
        demoted = last is not None and last.state == 'RESCUE_CONFIRMED' and state != 'RESCUE_CONFIRMED'
        verdict = RescueVerdict(self.new_id(), target, state, reasons, link.link_basis if link else None,
                                now_ns, version, self.policy.version, demoted_from_confirmed=demoted,
                                previous_verdict_id=last.verdict_id if last else None,
                                previous_confirmed_at_ns=last.evaluated_at_ns if demoted else None)
        self.history.setdefault(target, []).append(verdict)
        return verdict

    def _hold_reasons(self, basis, recheck, lidar, link, now_ns):
        p = self.policy
        if recheck.observed_at_ns > now_ns or (lidar is not None and lidar.scan_at_ns > now_ns):
            raise CoordinationError('CLOCK_UNCERTAIN', 'evidence from the future')
        if (recheck.task_kind != 'REOBSERVE_TARGET' or recheck.task_outcome != 'COMPLETED'
                or recheck.stop_confirmed is not True):
            yield 'TASK_NOT_COMPLETED'
        if recheck.robot_id != basis.scout_robot_id or (lidar is not None and lidar.robot_id != basis.scout_robot_id):
            yield 'RECHECK_ROBOT_NOT_SCOUT'
        if recheck.target_in_view is not True or recheck.depth_valid_at_target is not True:
            yield 'TARGET_NOT_OBSERVED'
        if recheck.person_detected_at_target is not False:
            yield 'PERSON_PRESENT_OR_UNKNOWN'
        if (lidar is None or not p.lidar_detects_target_measured
                or lidar.target_in_scan_range is not True or lidar.obstacle_at_target is not False):
            yield 'OBSTACLE_PRESENT_OR_UNKNOWN'
        # Robot-side stamps are compared under the same gate as the exit stamp:
        # same clock domain, summed error under the cap, "after" beyond that sum.
        # No code assumes SCOUT, SUPPORT and their sensors share a domain.
        # Not comparable is its own reason, distinct from a wrong order (작전).
        approach = (basis.approach_clock_domain_id, basis.approach_clock_error_bound_ns)
        seen = (recheck.clock_domain_id, recheck.clock_error_bound_ns)
        scan = (lidar.clock_domain_id, lidar.clock_error_bound_ns) if lidar is not None else None
        done = basis.approach_completed_at_ns
        skew_margin = self._margin(*seen, *scan) if lidar is not None else 0
        seen_margin = self._margin(*seen, *approach) if done is not None else 0
        scan_margin = self._margin(*scan, *approach) if lidar is not None and done is not None else 0
        if None in (skew_margin, seen_margin, scan_margin):
            yield 'TIME_BASIS_INCOMPARABLE'
        if (now_ns - recheck.observed_at_ns > p.max_recheck_age_ns
                or (lidar is not None and skew_margin is not None
                    and abs(recheck.observed_at_ns - lidar.scan_at_ns) > p.max_sensor_skew_ns)):
            yield 'EVIDENCE_STALE'
        basis_ref = (basis.map_session_id, basis.map_revision)
        if (not basis.projection_valid or (recheck.map_session_id, recheck.map_revision) != basis_ref
                or (lidar is not None and (lidar.map_session_id, lidar.map_revision) != basis_ref)):
            yield 'MAP_BASIS_CHANGED'
        single = link is not None and link.link_basis == SINGLE_TARGET_SCENARIO
        if (link is None or link.link_basis not in p.accepted_link_bases
                or link.site_event.confidence < p.min_exit_confidence
                or (single and basis.open_target_count != 1)):
            yield 'EXIT_NOT_LINKED'
        # C-03: time_basis_declared is only a precondition. An exit stamp orders
        # against another stamp only in the same clock domain with the summed
        # error bounds under the policy cap, and "after" must exceed that sum.
        # Otherwise the comparison is withheld, never guessed.
        exit_after_recheck_gap = exit_after_done_gap = None
        if link is not None:
            event = link.site_event
            gap_recheck = self._exit_margin(event, recheck.clock_domain_id, recheck.clock_error_bound_ns)
            gap_done = (self._exit_margin(event, basis.approach_clock_domain_id, basis.approach_clock_error_bound_ns)
                        if single else 0)
            if gap_recheck is None or gap_done is None:
                yield 'EXIT_TIME_BASIS_UNUSABLE'
            else:
                exit_after_recheck_gap, exit_after_done_gap = gap_recheck, gap_done
        if (done is None
                or (seen_margin is not None and recheck.observed_at_ns - done <= seen_margin)
                or (lidar is not None and scan_margin is not None and lidar.scan_at_ns - done <= scan_margin)
                or (exit_after_recheck_gap is not None
                    and recheck.observed_at_ns - link.site_event.observed_at_ns <= exit_after_recheck_gap)
                or (exit_after_recheck_gap is not None and single
                    and link.site_event.observed_at_ns - done <= exit_after_done_gap)):
            yield 'ORDER_VIOLATION'

    def _margin(self, domain_a, error_a, domain_b, error_b):
        """Summed error bound if two stamps may be ordered (C-03), else None."""
        if error_a is None or error_b is None or domain_a != domain_b:
            return None
        total = error_a + error_b
        return total if total <= self.policy.max_clock_error_sum_ns else None

    def _exit_margin(self, event, other_domain, other_error_ns):
        """As _margin, and the exit event must first declare a usable time basis."""
        if not event.time_basis_declared:
            return None
        return self._margin(event.clock_domain_id, event.clock_error_bound_ns, other_domain, other_error_ns)
