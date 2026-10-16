"""Robot-side final command watchdog core (C-01, TR-11). Imports no ROS package.

It sits on the last velocity path of one robot: Nav2 (or any mover) writes to its
input, and only this watchdog writes the robot's final command topic.

- An input command is forwarded only while ARMED and the executor heartbeat (the
  command adapter process on the PC) is fresh. Anything else becomes a zero
  command; it is never clamped or delayed.
- A stale executor heartbeat, an out-of-limit or non-finite command, or local time
  going backwards latches STOP: a zero on the transition, then every input
  command becomes a zero (no forwarding), and the watchdog's own heartbeat
  stops (the adapter's `watchdog` readiness flag then expires, so no new goal
  is sent).
- A heartbeat coming back never releases the latch (C-06: new connection or
  heartbeat alone does not unlatch). Only `reset(now)` with a fresh heartbeat does;
  the executor's own STOP latch and new-generation rule are separate and stay.
  One exception (재범 decision 2026-10-16 11:0x 「B로 가자」, grade 4): in the start-up phase (every heartbeat so
  far said HEARTBEAT_STARTUP: the executor has never been ARMED and has no task) an EXECUTOR_HEARTBEAT_STALE
  latch releases itself on the next fresh start-up heartbeat, at most AUTO_RESET_MAX times
  (WATCHDOG_AUTO_RESET; run 18 needed a person's reset before TB-6 could be armed).
- Input silence after a moving command sends one zero without latching (Nav2 goes
  quiet between goals).

All thresholds come from a WatchdogPolicy; none is defined here [미확인].
This is a software watchdog in a ROS process. Whether it runs on the robot or a
PC, and how it relates to the Create 3 cmd_vel timeout, is a deployment
question to measure (T-05); it is not a physical safety certificate.
"""
from dataclasses import dataclass
import json
import math

from rescue_robot.executor import ContractError

WATCHDOG_STATES = ('WAITING', 'ARMED', 'STOP_LATCHED')
ZERO = ('ZERO',)
# executor heartbeat Header.frame_id: start-up phase or not (anything else, '' from an older executor, is not)
HEARTBEAT_STARTUP, HEARTBEAT_RUN = 'startup', 'run'
AUTO_RESET_MAX = 3   # start-up auto resets per watchdog run [제안]; past it a person's reset as before


def heartbeat_phase(executor):
    """HEARTBEAT_STARTUP while the executor has never been ARMED and has no task, else HEARTBEAT_RUN."""
    return HEARTBEAT_STARTUP if not executor.ever_armed and executor.active is None else HEARTBEAT_RUN


@dataclass(frozen=True)
class WatchdogPolicy:
    version: str
    heartbeat_max_age_s: float
    command_max_age_s: float
    max_linear_mps: float
    max_angular_rps: float

    def __post_init__(self):
        if not isinstance(self.version, str) or not self.version:
            raise ContractError('watchdog policy version')
        for field in ('heartbeat_max_age_s', 'command_max_age_s', 'max_linear_mps', 'max_angular_rps'):
            value = getattr(self, field)
            if type(value) not in (int, float) or not 0 < value < math.inf:
                raise ContractError('watchdog ' + field)


def _norm(vector):
    return math.sqrt(sum(component * component for component in vector))


class CommandWatchdog:
    def __init__(self, policy):
        if not isinstance(policy, WatchdogPolicy):
            raise ContractError('watchdog policy type')
        self.policy = policy
        self.state = 'WAITING'
        self.reason = ''
        self.events = []          # (state, reason, local time) per transition
        self.last_heartbeat = None
        self.last_command = None  # local time of the last forwarded nonzero command
        self.last_time = None
        self.ran = False          # a heartbeat has said the executor is past start-up
        self.auto_resets = 0
        self.limit_said = False

    def _advance(self, now):
        """None if local time did not go backwards; else latch and return its outputs."""
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ContractError('watchdog time')
        if self.last_time is not None and now < self.last_time:
            return self._latch('CLOCK_REGRESSION', now)
        self.last_time = now
        return None

    def _set(self, state, reason, now, event_reason=None):
        self.state, self.reason = state, reason
        self.events.append((state, reason if event_reason is None else event_reason, now))

    def _latch(self, reason, now):
        """Returns [ZERO] on the transition only."""
        self.last_command = None
        if self.state == 'STOP_LATCHED':
            return []
        self._set('STOP_LATCHED', reason, now)
        return [ZERO]

    def _heartbeat_fresh(self, now):
        return (self.last_heartbeat is not None
                and 0 <= now - self.last_heartbeat <= self.policy.heartbeat_max_age_s)

    def heartbeat(self, received_steady, phase=None):
        """An executor heartbeat; ``phase`` is its HEARTBEAT_STARTUP / HEARTBEAT_RUN mark (None: unmarked)."""
        if self._advance(received_steady) is not None:
            return
        previous, self.last_heartbeat = self.last_heartbeat, received_steady
        if phase != HEARTBEAT_STARTUP:
            self.ran = True
        if self.state == 'WAITING':
            self._set('ARMED', '', received_steady)
        elif (self.state == 'STOP_LATCHED' and self.reason == 'EXECUTOR_HEARTBEAT_STALE' and not self.ran
              and previous is not None):
            stale = (f'startup reason=EXECUTOR_HEARTBEAT_STALE stale_for={received_steady - previous:.3f}s')
            if self.auto_resets < AUTO_RESET_MAX:
                self.auto_resets += 1
                self._set('ARMED', '', received_steady, event_reason=f'WATCHDOG_AUTO_RESET {stale} '
                                                                     f'count={self.auto_resets}/{AUTO_RESET_MAX}')
            elif not self.limit_said:   # once: stays latched for a person
                self.limit_said = True
                self.events.append(('STOP_LATCHED', f'WATCHDOG_AUTO_RESET_LIMIT {AUTO_RESET_MAX} reached ({stale}): '
                                                    'reset by a person', received_steady))

    def command(self, linear, angular, received_steady):
        """('FORWARD', linear, angular) or ('ZERO',) for one input command."""
        if self._advance(received_steady) is not None or self.state != 'ARMED':
            return ZERO
        if not self._heartbeat_fresh(received_steady):
            self._latch('EXECUTOR_HEARTBEAT_STALE', received_steady)
            return ZERO
        values = tuple(linear) + tuple(angular)
        if len(values) != 6 or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            self._latch('COMMAND_NOT_FINITE', received_steady)
            return ZERO
        if _norm(linear) > self.policy.max_linear_mps or _norm(angular) > self.policy.max_angular_rps:
            self._latch('COMMAND_OUT_OF_LIMIT', received_steady)
            return ZERO
        self.last_command = received_steady if any(values) else None
        return ('FORWARD', tuple(linear), tuple(angular))

    def step(self, now):
        """Periodic check; returns the outputs to publish (at most one ZERO)."""
        regressed = self._advance(now)
        if regressed is not None:
            return regressed
        if self.state == 'ARMED' and not self._heartbeat_fresh(now):
            return self._latch('EXECUTOR_HEARTBEAT_STALE', now)
        if self.last_command is not None and now - self.last_command > self.policy.command_max_age_s:
            self.last_command = None
            return [ZERO]
        return []

    def alive(self, now):
        """Publish the watchdog's own heartbeat now? Only while ARMED and fed."""
        return self.state == 'ARMED' and self._heartbeat_fresh(now)

    def reset(self, now):
        """Explicit operator/launch reset of STOP_LATCHED; needs a fresh heartbeat."""
        if self._advance(now) is not None or self.state != 'STOP_LATCHED' or not self._heartbeat_fresh(now):
            return False
        self._set('ARMED', '', now, event_reason='RESET')
        return True


def watchdog_state_topic(heartbeat_topic):
    """<ns>/rescue/watchdog_state next to <ns>/rescue/watchdog_heartbeat: the watchdog's state, latched.
    Reporting only (run 7, 2026-10-16): a latched watchdog sends no heartbeat, so without this it looked absent."""
    return heartbeat_topic.rsplit('/', 1)[0] + '/watchdog_state'


def state_text(state, reason, reset_service):
    """The JSON the watchdog publishes on every state change."""
    return json.dumps({'state': state, 'reason': reason, 'reset_service': reset_service}, sort_keys=True)


def latched_hint(text):
    """'watchdog STOP_LATCHED <reason> - reset needed: <command>' for a latched state text, else ''."""
    try:
        state = json.loads(text)
        if state['state'] != 'STOP_LATCHED':
            return ''
        return (f"watchdog STOP_LATCHED {state['reason'] or '-'} - reset needed: "
                f"ros2 service call {state['reset_service']} std_srvs/srv/Trigger {{}}")
    except (TypeError, ValueError, KeyError):
        return ''
