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
    'rescue_coordination': {}, 'rescue_robot': {
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
