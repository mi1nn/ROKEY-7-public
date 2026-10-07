"""Deterministic task boundary. Emits intents; never publishes ROS commands.

All time limits are supplied by a run policy. External evidence and action events
must be provided by a future authenticated, single-owner robot adapter.
"""
from dataclasses import dataclass
import math
from uuid import UUID


class ContractError(ValueError):
    pass


def number(value, name, *, positive=False):
    try:
        valid = type(value) in (float, int) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ContractError(name)
    if value < 0 or (positive and value == 0):
        raise ContractError(name)
    return float(value)


def future_deadline(now, duration):
    deadline = number(now + duration, 'local deadline', positive=True)
    if deadline <= now:
        raise ContractError('duration below clock resolution')
    return deadline


def identifier(value):
    if not isinstance(value, str):
        raise ContractError('UUID required')
    try:
        if str(UUID(value)) != value or UUID(value).int == 0:
            raise ValueError()
    except ValueError as exc:
        raise ContractError('canonical UUID required') from exc
    return value


def generation(value):
    if type(value) is not int or not 0 <= value < 2 ** 64:
        raise ContractError('uint64 generation required')
    return value


@dataclass(frozen=True)
class Policy:
    version: str
    acceptance_timeout: float
    stop_timeout: float
    stop_window: float
    max_sample_gap: float
    evidence_max_age: float
    linear_stop_max: float
    angular_stop_max: float

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise ContractError('policy version')
        for name in ('acceptance_timeout', 'stop_timeout', 'stop_window',
                     'max_sample_gap', 'evidence_max_age', 'linear_stop_max', 'angular_stop_max'):
            number(getattr(self, name), name, positive=True)


@dataclass(frozen=True)
class Readiness:
    received_steady: float
    driver: bool
    sensors: bool
    slam: bool
    nav2: bool
    undocked: bool
    watchdog: bool

    def valid(self, now, policy):
        number(self.received_steady, 'readiness stamp')
        flags = (self.driver, self.sensors, self.slam, self.nav2, self.undocked, self.watchdog)
        return (all(type(flag) is bool and flag for flag in flags)
                and 0 <= now - self.received_steady <= policy.evidence_max_age)


@dataclass(frozen=True)
class Task:
    command_id: str
    task_id: str
    robot_id: str
    robot_boot_id: str
    authority_epoch: str
    generation: int
    kind: str
    issued_at: float
    deadline: float
    lease_duration: float
    policy_version: str
    target_id: str | None = None
    target_revision: int | None = None

    def __post_init__(self):
        for name in ('command_id', 'task_id', 'robot_boot_id', 'authority_epoch'):
            identifier(getattr(self, name))
        generation(self.generation)
        if self.robot_id not in ('robot_a', 'robot_b'):
            raise ContractError('robot identity')
        if self.kind not in ('SEARCH_AREA', 'REOBSERVE_TARGET', 'APPROACH_TARGET', 'HOLD'):
            raise ContractError('task kind')
        for name in ('issued_at', 'deadline', 'lease_duration'):
            number(getattr(self, name), name, positive=True)
        if self.deadline <= self.issued_at or not isinstance(self.policy_version, str) or not self.policy_version:
            raise ContractError('deadline or policy')
        if self.kind in ('REOBSERVE_TARGET', 'APPROACH_TARGET'):
            identifier(self.target_id)
            generation(self.target_revision)
        elif self.target_id is not None or self.target_revision is not None:
            raise ContractError('search/hold has no target')


@dataclass(frozen=True)
class Admission:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class Result:
    command_id: str
    outcome: str
    reason: str
    original_cause: str
    stop_confirmed: bool


class Executor:
    """One active movement intent; explicit stopped reset after every task.

    `drain_intents` is an adapter boundary, not evidence that a robot acted.
    `goal_terminal` is the action execution fence; odometry independently proves
    bounded motion. A cancel request/ack alone is never enough.
    """
    def __init__(self, robot_id, robot_boot_id, authority_epoch, policy):
        if robot_id not in ('robot_a', 'robot_b') or not isinstance(policy, Policy):
            raise ContractError('executor configuration')
        self.robot_id = robot_id
        self.robot_boot_id = identifier(robot_boot_id)
        self.authority_epoch = identifier(authority_epoch)
        self.policy = policy
        self.active = None
        self.state = 'IDLE'
        self.motion = 'STOP_UNCONFIRMED'
        self.latch = 'DISARMED'
        self.session_fault = False
        self.highest_generation = -1
        self.results = {}
        self.seen = {}
        self.intents = []
        self.pending = False
        self.fenced = True
        self.fenced_since = 0.0
        self.stop_start = None
        self.last_sample = None
        self.clock = None
        self.lease_until = 0.0
        self.lease_sequence = -1
        self.stop_deadline = None
        self.stop_cause = ''
        self.stop_outcome = ''

    def _time(self, now):
        now = number(now, 'local monotonic time')
        if self.clock is not None and now < self.clock:
            # No recovery through a forged/reversed steady clock.
            self.motion, self.latch = 'STOP_UNCONFIRMED', 'FAULT_LATCHED'
            self.session_fault = True
            if self.active:
                self.intents.append(('CANCEL', self.active.command_id))
            raise ContractError('monotonic clock regressed; recreate session')
        self.clock = now
        return now

    def observe_motion(self, *, sample_steady, received_steady, linear, angular):
        now = self._time(received_steady)
        stamp = number(sample_steady, 'sample time')
        for value in (linear, angular):
            if type(value) not in (int, float):
                raise ContractError('velocity')
            number(abs(value), 'velocity')
        if stamp > now or now - stamp > self.policy.evidence_max_age:
            self.stop_start = None
            self.motion = 'STOP_UNCONFIRMED'
            return False
        if self.last_sample is not None and stamp <= self.last_sample:
            return False  # duplicates cannot grow a stop window
        gap = self.last_sample is None or stamp - self.last_sample > self.policy.max_sample_gap
        self.last_sample = stamp
        bounded = abs(linear) <= self.policy.linear_stop_max and abs(angular) <= self.policy.angular_stop_max
        if not bounded or not self.fenced or stamp < self.fenced_since:
            self.stop_start = None
            self.motion = 'MOVING' if not bounded else 'STOP_REQUESTED'
            return False
        if gap or self.stop_start is None:
            self.stop_start = stamp
        if stamp - self.stop_start >= self.policy.stop_window:
            self.motion = 'STOPPED'
            if self.state == 'STOPPING':
                self._finish(self.stop_outcome, self.stop_cause, True)
            return True
        self.motion = 'STOP_UNCONFIRMED'
        return False

    def arm(self, authority_epoch, *, now, readiness):
        now = self._time(now)
        fresh_stop = self.last_sample is not None and 0 <= now - self.last_sample <= self.policy.evidence_max_age
        if (authority_epoch != self.authority_epoch or self.active is not None
                or not self.fenced or self.motion != 'STOPPED' or not fresh_stop
                or not readiness.valid(now, self.policy) or self.latch == 'FAULT_LATCHED' or self.session_fault):
            return False
        self.latch, self.state = 'ARMED', 'IDLE'
        return True

    def register_authority(self, previous_epoch, new_epoch, *, now, readiness):
        # Explicit revoke/register; never lexicographic UUID ordering.
        if previous_epoch != self.authority_epoch or not self.arm(previous_epoch, now=now, readiness=readiness):
            return False
        self.authority_epoch = identifier(new_epoch)
        self.latch = 'DISARMED'
        return True

    def admit(self, task, *, now_wall, now_steady, readiness):
        now = self._time(now_steady)
        wall = number(now_wall, 'wall time', positive=True)
        if not isinstance(task, Task):
            raise ContractError('task type')
        if task.command_id in self.seen:
            previous, receipt = self.seen[task.command_id]
            return receipt if previous == task else Admission(False, 'ID_CONTENT_CONFLICT')
        reason = None
        if (task.robot_id != self.robot_id or task.robot_boot_id != self.robot_boot_id
                or task.authority_epoch != self.authority_epoch):
            reason = 'AUTHORITY_OR_SESSION'
        elif task.policy_version != self.policy.version:
            reason = 'POLICY_VERSION'
        elif task.generation <= self.highest_generation:
            reason = 'OLD_GENERATION'
        elif self.active is not None or self.latch != 'ARMED' or self.motion != 'STOPPED':
            reason = 'BUSY_OR_LATCHED'
        elif (not readiness.valid(now, self.policy) or self.last_sample is None
              or not 0 <= now - self.last_sample <= self.policy.evidence_max_age):
            reason = 'NOT_READY'
        elif not task.issued_at <= wall < task.deadline:
            reason = 'INVALID_TIME'
        if reason is None:
            future_deadline(now, task.lease_duration)
        receipt = Admission(reason is None, reason or 'ACCEPTED')
        self.seen[task.command_id] = (task, receipt)
        if not receipt.accepted:
            return receipt
        lease_until = future_deadline(now, task.lease_duration)
        self.active = task
        self.highest_generation = task.generation
        self.lease_until = lease_until
        self.lease_sequence = -1
        self.state = 'PLANNING'
        return receipt

    def dispatch(self, *, now_wall, now_steady, readiness, guards_valid):
        self.tick(now_wall=now_wall, now_steady=now_steady, readiness=readiness, guards_valid=guards_valid)
        if self.state != 'PLANNING':
            return False
        if (self.motion != 'STOPPED' or self.last_sample is None
                or now_steady - self.last_sample > self.policy.evidence_max_age):
            self.request_stop('STOP_EVIDENCE_STALE', now=now_steady, outcome='FAILED')
            return False
        if self.active.kind == 'HOLD':
            self.request_stop('HOLD', now=now_steady, outcome='COMPLETED')
            return True
        accept_by = future_deadline(now_steady, self.policy.acceptance_timeout)
        self.pending, self.fenced = True, False
        self.stop_start = None
        self.motion, self.state = 'MOVING', 'AWAITING_ACCEPTANCE'
        self.accept_by = accept_by
        self.intents.append(('SEND', self.active.command_id))
        return True

    def goal_response(self, command_id, *, accepted, now):
        now = self._time(now)
        identifier(command_id)
        if type(accepted) is not bool:
            raise ContractError('accepted flag')
        if self.active and command_id == self.active.command_id and self.state == 'NAVIGATING' and accepted:
            return  # repeated acceptance cannot cancel the active execution
        if self.active is None or command_id != self.active.command_id or self.state != 'AWAITING_ACCEPTANCE':
            if accepted:
                self.intents.append(('CANCEL', command_id))
            if self.active and command_id == self.active.command_id:
                self.pending = False
                if not accepted and not self.fenced:
                    self.fenced = True
                    self.fenced_since, self.stop_start = now, None
            return
        self.pending = False
        if accepted:
            self.state = 'NAVIGATING'
        else:
            self.fenced = True
            self.fenced_since, self.stop_start = now, None
            self.request_stop('GOAL_REJECTED', now=self.clock, outcome='FAILED')

    def goal_terminal(self, command_id, *, succeeded, now):
        self._time(now)
        if type(succeeded) is not bool:
            raise ContractError('success flag')
        if (not self.active or self.active.command_id != command_id
                or self.state not in ('AWAITING_ACCEPTANCE', 'NAVIGATING', 'STOPPING')):
            return False
        # A result implies acceptance, even if its callback arrived first.
        self.pending = False
        if not self.fenced:
            self.fenced_since, self.stop_start = now, None
        self.fenced = True
        if self.state != 'STOPPING':
            self.request_stop('NAVIGATION_SUCCEEDED' if succeeded else 'NAVIGATION_FAILED',
                              now=now, outcome='COMPLETED' if succeeded else 'FAILED')
        return True

    def renew(self, command_id, authority_epoch, generation_value, sequence, *, now):
        now = self._time(now)
        generation(sequence)
        generation(generation_value)
        if (not self.active or self.active.command_id != command_id
                or authority_epoch != self.authority_epoch or generation_value != self.active.generation
                or now >= self.lease_until or self.state == 'STOPPING' or sequence <= self.lease_sequence):
            return False
        lease_until = future_deadline(now, self.active.lease_duration)
        self.lease_sequence, self.lease_until = sequence, lease_until
        return True

    def request_stop(self, cause, *, now, outcome='CANCELED'):
        now = self._time(now)
        if outcome not in ('COMPLETED', 'CANCELED', 'EXPIRED', 'UNREACHABLE', 'FAILED'):
            raise ContractError('outcome')
        if self.active is None or self.state == 'STOPPING':
            return
        self.state, self.motion, self.latch = 'STOPPING', 'STOP_REQUESTED', 'STOP_LATCHED'
        self.stop_cause, self.stop_outcome = cause, outcome
        self.stop_start = None
        if not self.fenced:
            self.intents.append(('CANCEL', self.active.command_id))
        try:
            self.stop_deadline = future_deadline(now, self.policy.stop_timeout)
        except ContractError:
            # A broken timer cannot suppress cancellation or leave movement armed.
            self.motion, self.latch = 'STOP_UNCONFIRMED', 'FAULT_LATCHED'
            self._finish('FAILED', 'STOP_UNCONFIRMED', False)

    def tick(self, *, now_wall, now_steady, readiness, guards_valid):
        now = self._time(now_steady)
        wall = number(now_wall, 'wall time', positive=True)
        if self.active is None:
            return
        if self.state == 'STOPPING':
            if now >= self.stop_deadline:
                self.motion, self.latch = 'STOP_UNCONFIRMED', 'FAULT_LATCHED'
                self._finish('FAILED', 'STOP_UNCONFIRMED', False)
            return
        if self.latch == 'FAULT_LATCHED':
            self.request_stop('SESSION_FAULT', now=now, outcome='FAILED')
        elif type(guards_valid) is not bool or not guards_valid or not readiness.valid(now, self.policy):
            self.request_stop('GUARD_INVALID', now=now, outcome='FAILED')
        elif wall < self.active.issued_at:
            self.request_stop('CLOCK_INVALID', now=now, outcome='FAILED')
        elif wall >= self.active.deadline or now >= self.lease_until:
            self.request_stop('DEADLINE_OR_LEASE', now=now, outcome='EXPIRED')
        elif self.state == 'AWAITING_ACCEPTANCE' and now >= self.accept_by:
            self.request_stop('ACCEPT_TIMEOUT', now=now, outcome='FAILED')

    def _finish(self, outcome, reason, confirmed):
        command_id = self.active.command_id
        self.results[command_id] = Result(command_id, outcome, reason, self.stop_cause, confirmed)
        # Approach verification is separate: COMPLETED means action+stop only.
        self.state = 'FINISHED'
        self.active = None
        self.latch = 'STOP_LATCHED' if confirmed and not self.session_fault else 'FAULT_LATCHED'

    def drain_intents(self):
        result, self.intents = tuple(self.intents), []
        return result
