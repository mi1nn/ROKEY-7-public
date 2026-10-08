"""Shared Python boundary contracts; no ROS IDL, middleware or runtime nodes."""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import re
from uuid import UUID

MAX_TIMESTAMP_NS = (1 << 63) - 1
CANONICAL_CODES = frozenset('''INVALID_INPUT INCOMPATIBLE_SCHEMA WRONG_ROBOT
STALE_OBSERVATION CLOCK_UNCERTAIN STALE_TRANSFORM FRAME_REVISION_MISMATCH
LOCALIZATION_UNCERTAIN SENSOR_UNAVAILABLE NAV_NOT_READY NOT_AUTHORIZED
STALE_GENERATION LEASE_EXPIRED STOP_LATCHED UNREACHABLE RETRY_EXHAUSTED
REPORT_NOT_PERSISTED STOP_UNCONFIRMED ID_CONTENT_CONFLICT TASK_BUSY
DEADLINE_EXPIRED ACCEPT_TIMEOUT GUARD_INVALID STOP_EVIDENCE_STALE
SESSION_FAULT GOAL_REJECTED ACTION_CANCELED ACTION_ABORTED ROBOT_DOCKED
DRIVER_UNAVAILABLE COMMAND_WATCHDOG_UNAVAILABLE POLICY_MISMATCH
NOT_FOUND PENDING_EVIDENCE'''.split())

# C-09's single executable dictionary. Component diagnostics remain namespaced,
# while consumers branch only on canonical .code (never exception class names).
DETAIL_CATEGORIES = {
    'rescue_mapping': {
        'INVALID_QUATERNION': 'INVALID_INPUT',
        'INVALID_TF_TIME_EVIDENCE': 'STALE_TRANSFORM',
        'UNKNOWN_ROBOT': 'WRONG_ROBOT',
        'RETIRED_SESSION': 'FRAME_REVISION_MISMATCH',
        'SESSION_CONTENT_CONFLICT': 'ID_CONTENT_CONFLICT',
        'MAP_SESSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'MAP_VERSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'NONINCREASING_MAP_REVISION': 'FRAME_REVISION_MISMATCH',
        'STALE_MAP_UPDATE': 'FRAME_REVISION_MISMATCH',
        'WRONG_ALIGNMENT_AUTHORITY': 'NOT_AUTHORIZED',
        'CLOCK_DOMAIN_MISMATCH': 'CLOCK_UNCERTAIN',
        'CLOCK_RESET_REQUIRED': 'CLOCK_UNCERTAIN',
        'FRAME_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'STALE_ALIGNMENT_REVISION': 'FRAME_REVISION_MISMATCH',
        'ALIGNMENT_REVISION_CONFLICT': 'ID_CONTENT_CONFLICT',
        'ALIGNMENT_UNAVAILABLE': 'LOCALIZATION_UNCERTAIN',
        'ALIGNMENT_INVALID': 'LOCALIZATION_UNCERTAIN',
        'ALIGNMENT_DEGRADED': 'LOCALIZATION_UNCERTAIN',
        'ALIGNMENT_EXPIRED_OR_NOT_YET_VALID': 'STALE_TRANSFORM',
        'ALIGNMENT_UNCERTAIN': 'LOCALIZATION_UNCERTAIN',
        'UNKNOWN_OBSERVATION': 'INVALID_INPUT',
        'OBSERVATION_EXPIRED': 'STALE_OBSERVATION',
        'FUTURE_OBSERVATION': 'CLOCK_UNCERTAIN',
        'TF_UNAVAILABLE': 'STALE_TRANSFORM',
        'TF_ROBOT_MISMATCH': 'WRONG_ROBOT',
        'TF_FRAME_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'TF_MAP_VERSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'TF_TIME_MISMATCH': 'STALE_TRANSFORM',
        'TF_CONTENT_CONFLICT': 'ID_CONTENT_CONFLICT',
        'PROJECTION_UNAVAILABLE': 'LOCALIZATION_UNCERTAIN',
        'DUPLICATE_OBSERVATION_REFERENCE': 'INVALID_INPUT',
        'SOURCE_PROJECTION_INVALID': 'FRAME_REVISION_MISMATCH',
        'INVALID_GOAL_SNAPSHOT': 'INVALID_INPUT',
        'UNREGISTERED_OR_MODIFIED_GOAL_SNAPSHOT': 'ID_CONTENT_CONFLICT',
        'GOAL_SNAPSHOT_EXPIRED': 'STALE_OBSERVATION',
        'GOAL_VERSION_INVALIDATED': 'FRAME_REVISION_MISMATCH',
        'DESTINATION_VERSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'SOURCE_VERSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'VERSION_MISMATCH': 'FRAME_REVISION_MISMATCH',
        'NEEDS_REOBSERVATION': 'FRAME_REVISION_MISMATCH',
        'BACKEND_NOT_INTEGRATED': 'LOCALIZATION_UNCERTAIN',
    },
    'rescue_perception': {
        'UNSUPPORTED_DEPTH': 'INVALID_INPUT', 'UNSYNCHRONIZED': 'CLOCK_UNCERTAIN',
        'ALIGNMENT_UNVERIFIED': 'LOCALIZATION_UNCERTAIN',
        'CALIBRATION_MISMATCH': 'LOCALIZATION_UNCERTAIN',
        'DIMENSION_MISMATCH': 'INVALID_INPUT',
        'DEPTH_INSUFFICIENT': 'SENSOR_UNAVAILABLE',
        'DEPTH_DISCONTINUITY': 'LOCALIZATION_UNCERTAIN',
        'MODEL_CLASS_MISMATCH': 'INVALID_INPUT',
        'FUTURE_OBSERVATION': 'CLOCK_UNCERTAIN',
    },
    'rescue_operations': {'STORAGE_ERROR': 'REPORT_NOT_PERSISTED',
                          'EVIDENCE_HASH_MISMATCH': 'ID_CONTENT_CONFLICT'},
    'rescue_coordination': {
        'INVALID_NUMBER': 'INVALID_INPUT',
        'INVALID_UUID': 'INVALID_INPUT',
        'INVALID_REVISION': 'INVALID_INPUT',
        'INVALID_POINT': 'INVALID_INPUT',
        'INVALID_POLICY': 'INVALID_INPUT',
        'INVALID_CANDIDATE': 'INVALID_INPUT',
        'INVALID_RESERVATION': 'INVALID_INPUT',
        'INVALID_ADMISSION': 'INVALID_INPUT',
        'INVALID_HOLDER': 'INVALID_INPUT',
        'LEASE_BELOW_RESOLUTION': 'INVALID_INPUT',
    }, 'rescue_robot': {
        'TARGET_STALE': 'STALE_OBSERVATION', 'CANCEL_USER': 'ACTION_CANCELED',
    }, 'rescue_contracts': {},
}


class RescueError(ValueError):
    def __init__(self, code, detail='', *, component='rescue_contracts', detail_code=None):
        if code not in CANONICAL_CODES or component not in DETAIL_CATEGORIES:
            raise ValueError('unregistered contract error or component')
        self.code, self.detail, self.component = code, str(detail), component
        reason = detail_code or code
        self.detail_code = reason if reason.startswith(component+'.') else component+'.'+reason
        super().__init__(f'{code} [{self.detail_code}]: {self.detail}')


def canonical_code(component, reason):
    if component not in DETAIL_CATEGORIES:
        raise ValueError('unregistered component')
    if reason in CANONICAL_CODES:
        return reason
    try:
        return DETAIL_CATEGORIES[component][reason]
    except KeyError as exc:
        raise ValueError(f'unregistered {component} diagnostic: {reason}') from exc


def _require(ok, detail):
    if not ok:
        raise RescueError('INVALID_INPUT', detail)


def robot_id(value):
    _require(isinstance(value, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', value),
             'robot ID must match profile identifier syntax')
    return value


def robot_ids(values):
    _require(isinstance(values, (list, tuple, set, frozenset)), 'explicit robot ID collection required')
    ids = tuple(robot_id(value) for value in values)
    _require(len(ids) == 2 and len(set(ids)) == 2, 'exactly two distinct configured robots required')
    return tuple(sorted(ids))


def uuid_string(value):
    _require(isinstance(value, str), 'UUID string required')
    try:
        parsed = UUID(value)
        valid = str(parsed) == value and parsed.int != 0
    except (ValueError, AttributeError):
        valid = False
    _require(valid, 'canonical nonzero UUID required')
    return value


def uint64(value):
    _require(type(value) is int and 0 <= value < 2**64, 'uint64 required')
    return value


def timestamp_ns(value):
    _require(type(value) is int and 0 < value <= MAX_TIMESTAMP_NS, 'positive int64 nanosecond timestamp required')
    return value


def duration_ns(value):
    _require(type(value) is int and 0 <= value <= MAX_TIMESTAMP_NS, 'nonnegative int64 nanosecond duration required')
    return value


def ros_time_to_ns(sec, nanosec):
    _require(type(sec) is int and 0 <= sec < 2**31, 'nonnegative ROS int32 sec required')
    _require(type(nanosec) is int and 0 <= nanosec < 10**9, 'ROS nanosec range')
    return timestamp_ns(sec * 10**9 + nanosec)


def ns_to_ros_time(value):
    value = timestamp_ns(value)
    sec, nanosec = divmod(value, 10**9)
    _require(sec < 2**31, 'timestamp exceeds ROS int32 sec')
    return sec, nanosec


def seconds_to_ns(value):
    """Exact decimal fixture/policy seconds only; NEVER recover a ROS stamp float.

    ROS sensor stamps must enter through ros_time_to_ns(int sec, int nanosec).
    A float that already lost epoch nanoseconds cannot be repaired by this API.
    """
    _require(type(value) in (int, float, str, Decimal), 'decimal seconds required')
    try:
        decimal = Decimal(str(value))
        _require(decimal.is_finite() and decimal >= 0, 'nonnegative finite seconds required')
        if decimal.is_zero():
            return 0
        # Decimal arithmetic obeys caller context precision; integer digit math
        # avoids silently rounding fractional ns or epoch nanoseconds.
        _require(-9 <= decimal.adjusted() <= 9, 'seconds outside nanosecond range')
        parts = decimal.as_tuple()
        digits = ''.join(str(digit) for digit in parts.digits)
        shift = parts.exponent + 9
        if shift < 0:
            _require(-shift < len(digits) and not any(c != '0' for c in digits[shift:]),
                     'seconds must denote exact nanoseconds')
            digits = digits[:shift]
        return duration_ns(int(digits) * 10**max(shift, 0))
    except (InvalidOperation, ValueError, OverflowError) as exc:
        if isinstance(exc, RescueError):
            raise
        raise RescueError('INVALID_INPUT', 'invalid decimal seconds') from exc


@dataclass(frozen=True)
class TaskAdmission:
    command_id: str
    task_id: str
    robot_id: str
    robot_boot_id: str
    authority_epoch: str
    generation: int
    accepted: bool
    reason: str
    detail_code: str = ''

    def __post_init__(self):
        for value in (self.command_id, self.task_id, self.robot_boot_id, self.authority_epoch):
            uuid_string(value)
        robot_id(self.robot_id)
        uint64(self.generation)
        _require(type(self.accepted) is bool, 'accepted must be boolean')
        _require(self.reason == 'ACCEPTED' if self.accepted else self.reason in CANONICAL_CODES,
                 'admission outcome/code mismatch')
        _require(isinstance(self.detail_code, str), 'detail_code')


# ---- Bundle 1 (2026-10-08, BRD v0.1): roles, site events, system modes, rescue states.
ROBOT_ROLES = frozenset({'SCOUT', 'SUPPORT'})
# C-06 task kinds — the single vocabulary for rescue_robot.Task and coordination reservations.
TASK_KINDS = frozenset({'SEARCH_AREA', 'REOBSERVE_TARGET', 'APPROACH_TARGET', 'HOLD'})
TARGET_TASK_KINDS = frozenset({'REOBSERVE_TARGET', 'APPROACH_TARGET'})
SITE_EVENT_KINDS = frozenset({'QUAKE_SIGN', 'EXIT_PASS'})
STAMP_SEMANTICS = frozenset({'CAPTURE', 'DRIVER_RECEIVE', 'UNKNOWN'})
CLOCK_STATES = frozenset({'SYNCED', 'UNKNOWN'})
SYSTEM_MODES = frozenset({'NORMAL', 'DISASTER', 'CLEARING'})
RESCUE_STATES = frozenset({'NOT_CHECKED', 'RECHECK_NEEDED', 'RESCUE_CONFIRMED'})


def robot_role(value):
    _require(isinstance(value, str) and value in ROBOT_ROLES, 'robot role must be SCOUT or SUPPORT')
    return value


def camera_id(value):
    """Fixed-camera identity (site camera A/B). Same syntax as robot_id; never a robot."""
    _require(isinstance(value, str) and re.fullmatch(r'cam_[A-Za-z0-9_]{1,60}', value),
             'camera ID must match cam_<identifier>')
    return value


@dataclass(frozen=True)
class SiteEvent:
    """An observation made by a fixed site camera, not by a robot.

    QUAKE_SIGN: image shake/occlusion sign from camera A. EXIT_PASS: a person
    passing the exit seen by camera B. It carries evidence, never a command.
    """
    event_id: str
    camera_id: str
    kind: str
    observed_at_ns: int
    evidence_id: str
    evidence_sha256: str
    confidence: float
    # C-03 time basis of the camera host (same vocabulary as Observation). An event
    # with UNKNOWN semantics/clock may be recorded but must not order decisions.
    clock_domain_id: str
    stamp_semantics: str
    clock_state: str
    clock_error_bound_ns: int | None
    detail: str = ''

    def __post_init__(self):
        uuid_string(self.event_id)
        camera_id(self.camera_id)
        _require(isinstance(self.kind, str) and self.kind in SITE_EVENT_KINDS, 'site event kind')
        timestamp_ns(self.observed_at_ns)
        uuid_string(self.evidence_id)
        _require(isinstance(self.evidence_sha256, str) and re.fullmatch(r'[0-9a-f]{64}', self.evidence_sha256),
                 'evidence_sha256 must be 64 lowercase hex digits')
        _require(type(self.confidence) is float and 0.0 <= self.confidence <= 1.0, 'confidence in [0, 1]')
        _require(isinstance(self.clock_domain_id, str) and self.clock_domain_id, 'clock_domain_id')
        _require(isinstance(self.stamp_semantics, str) and self.stamp_semantics in STAMP_SEMANTICS, 'stamp_semantics')
        _require(isinstance(self.clock_state, str) and self.clock_state in CLOCK_STATES, 'clock_state')
        if self.clock_error_bound_ns is not None:
            duration_ns(self.clock_error_bound_ns)
        _require(isinstance(self.detail, str), 'detail')

    @property
    def time_basis_declared(self):
        """Precondition only: the event *declares* a usable time basis.

        True means stamp_semantics != UNKNOWN, clock_state == SYNCED and an error
        bound is present. It does NOT compare this event with anything: ordering
        or freshness against a robot observation still requires the consumer to
        check the same clock_domain_id on both sides, the summed error bounds
        against its policy, and the age/future limits (C-03; see
        rescue_perception.validate_freshness). False means the event may be
        recorded but must never enter such a decision.
        """
        return (self.stamp_semantics != 'UNKNOWN' and self.clock_state == 'SYNCED'
                and self.clock_error_bound_ns is not None)
