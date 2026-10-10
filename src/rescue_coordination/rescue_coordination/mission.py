"""Mission mode gate, operator command gate and the displayed mission phase.

Scenario (scope Q-12, 2026-10-10; console-and-db-plan 1): a verified
``DISASTER_SIGN`` switches NORMAL -> DISASTER, and only then does the SCOUT get
``SEARCH_AREA`` (no operator approval). The SUPPORT moves only on operator
approval: ``DISPATCH_SUPPORT`` authorizes the first segment of a route, and each
further segment needs ``CONTINUE_SEGMENT`` after the previous one stopped. The
coordinator never authorizes the next segment by itself.

This sits on ``SearchCoordinator``; it keeps no generation ledger. MOVE_SEGMENT
and RETURN_HOME are not in ``rescue_contracts.TASK_KINDS`` yet (#65), so an
approval here is state only and issuing the Task is a follow-up. Every result
uses a closed reason set; no free text is returned.
"""
from dataclasses import dataclass

from rescue_contracts import RESCUE_STATES, SiteEvent, timestamp_ns

from .planning import CoordinationError, contract_value, revision, robot, uid
from .search import Decision, SearchCoordinator

MODE_REJECT_REASONS = frozenset({'NOT_DISASTER_SIGN', 'TIME_BASIS_UNUSABLE'})
MODE_EVENT_REASONS = MODE_REJECT_REASONS | {'ENTERED_DISASTER', 'ALREADY_DISASTER'}
PLAN_GATE_REASONS = frozenset({'PASSED', 'MODE_NOT_DISASTER'})
# Commands this gate judges (console-and-db-plan 4). STOP/PAUSE/RESUME/CANCEL and
# the manual START_SEARCH go elsewhere and are rejected here as invalid input.
COMMAND_FIELDS = {
    'DISPATCH_SUPPORT': ('target_id', 'target_revision'),
    'CONTINUE_SEGMENT': ('target_id', 'route_revision', 'segment_index'),
    'REQUEST_RECHECK': ('target_id',),
    'RETURN_HOME': (),
}
OPTIONAL_FIELDS = ('target_id', 'target_revision', 'route_revision', 'segment_index')
COMMAND_REJECT_REASONS = frozenset({
    'MODE_NOT_DISASTER', 'UNKNOWN_TARGET', 'TARGET_NOT_CONFIRMED', 'TARGET_REVISION_STALE', 'NO_ROUTE',
    'ALREADY_DISPATCHED', 'NOT_DISPATCHED_TARGET', 'NOT_AWAITING_APPROVAL', 'ROUTE_REVISION_STALE',
    'SEGMENT_INDEX_MISMATCH', 'NOT_ARRIVED', 'RECHECK_ALREADY_REQUESTED', 'NO_VERDICT', 'ALREADY_RETURNING'})
# APPROVED: the operator authorized segment ``segment_index``; MOVING: the executor admitted it;
# AWAITING_APPROVAL: stopped, the next segment needs CONTINUE_SEGMENT; ARRIVED: route done.
SEGMENT_PHASES = frozenset({'APPROVED', 'MOVING', 'AWAITING_APPROVAL', 'ARRIVED'})
# console-and-db-plan 1. CANDIDATE needs TARGET_DISAPPEARED_CANDIDATE (#48) and is not reached yet.
MISSION_PHASES = frozenset({
    'IDLE', 'DISASTER', 'SEARCHING', 'TARGET_FOUND', 'DISPATCH_APPROVED', 'SEGMENT_MOVING',
    'AWAITING_CONTINUE', 'ARRIVED', 'CANDIDATE', 'RECHECKING', 'RESCUE_CONFIRMED', 'RECHECK_NEEDED',
    'RETURNING', 'FINISHED'})
VERDICTS = RESCUE_STATES - {'NOT_CHECKED'}


@dataclass(frozen=True)
class ModeResult:
    event_id: str
    reason: str
    mode: str

    def __post_init__(self):
        if self.reason not in MODE_EVENT_REASONS:
            raise CoordinationError('INVALID_INPUT', 'mode reason outside the closed set')


@dataclass(frozen=True)
class PlanGate:
    """``decision`` is the ``SearchCoordinator.plan`` result exactly when PASSED."""
    reason: str
    decision: Decision | None

    def __post_init__(self):
        if self.reason not in PLAN_GATE_REASONS or (self.decision is None) == (self.reason == 'PASSED'):
            raise CoordinationError('INVALID_INPUT', 'plan gate reason/decision')


@dataclass(frozen=True)
class OperatorCommand:
    """One operator POST (console-and-db-plan 4). Fields a kind does not use must be None."""
    command_id: str
    kind: str
    operator_id: str
    requested_at_ns: int
    target_id: str | None = None
    target_revision: int | None = None
    route_revision: int | None = None
    segment_index: int | None = None

    def __post_init__(self):
        uid(self.command_id)
        contract_value(timestamp_ns, self.requested_at_ns)
        if not isinstance(self.operator_id, str) or not self.operator_id:
            raise CoordinationError('INVALID_INPUT', 'operator_id')
        required = COMMAND_FIELDS.get(self.kind) if isinstance(self.kind, str) else None
        if required is None:
            raise CoordinationError('INVALID_INPUT', 'command kind')
        for name in OPTIONAL_FIELDS:
            value = getattr(self, name)
            if (value is None) == (name in required):
                raise CoordinationError('INVALID_INPUT', f'{name} required exactly for {self.kind}')
            if value is not None:
                (uid if name == 'target_id' else revision)(value)


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    accepted: bool
    reason: str

    def __post_init__(self):
        if self.reason != 'ACCEPTED' and self.reason not in COMMAND_REJECT_REASONS:
            raise CoordinationError('INVALID_INPUT', 'command reason outside the closed set')
        if self.accepted != (self.reason == 'ACCEPTED'):
            raise CoordinationError('INVALID_INPUT', 'accepted exactly when ACCEPTED')


@dataclass(frozen=True)
class MissionView:
    """What the screen reads; it never judges (console-and-db-plan 1)."""
    mode: str
    phase: str
    trigger_event_id: str | None
    trigger_observed_at_ns: int | None
    target_id: str | None
    route_revision: int | None
    segment_index: int | None
    segment_count: int | None
    segment_phase: str | None


class MissionCoordinator:
    """One mission: one trigger, one SUPPORT dispatch (Phase 1)."""

    def __init__(self, search):
        if not isinstance(search, SearchCoordinator):
            raise CoordinationError('INVALID_POLICY', 'search coordinator type')
        self.search = search
        self.mode = 'NORMAL'
        self.trigger_event_id = None
        self.trigger_observed_at_ns = None
        self.events = {}    # event_id -> (SiteEvent, ModeResult)
        self.commands = {}  # command_id -> (OperatorCommand, CommandResult)
        self.targets = {}   # target_id -> (target_revision, detection_confirmed, report_persisted)
        self.routes = {}    # target_id -> (route_revision, segment_count)
        self.search_started = False
        self.dispatch_target = None
        self.segment_index = None
        self.segment_phase = None
        self.recheck_pending = False
        self.verdict = None
        self.returning = False
        self.returned = set()

    @property
    def route_revision(self):
        return self.routes[self.dispatch_target][0] if self.dispatch_target else None

    @property
    def segment_count(self):
        return self.routes[self.dispatch_target][1] if self.dispatch_target else None

    # -- mode gate ---------------------------------------------------------
    def record_site_event(self, event):
        """NORMAL -> DISASTER on a DISASTER_SIGN with a declared time basis (C-03).

        The same event_id returns its first result unchanged; the same ID with
        other content is ``ID_CONTENT_CONFLICT``. Rejected events are remembered
        too, so a replay gets the same answer.
        """
        if not isinstance(event, SiteEvent):
            raise CoordinationError('INVALID_INPUT', 'SiteEvent required')
        seen = self.events.get(event.event_id)
        if seen is not None:
            if seen[0] != event:
                raise CoordinationError('ID_CONTENT_CONFLICT', 'site event content differs for the same event_id')
            return seen[1]
        if event.kind != 'DISASTER_SIGN':
            reason = 'NOT_DISASTER_SIGN'
        elif not event.time_basis_declared:
            reason = 'TIME_BASIS_UNUSABLE'
        elif self.mode == 'DISASTER':
            reason = 'ALREADY_DISASTER'
        else:
            reason = 'ENTERED_DISASTER'
            self.mode = 'DISASTER'
            self.trigger_event_id, self.trigger_observed_at_ns = event.event_id, event.observed_at_ns
        result = ModeResult(event.event_id, reason, self.mode)
        self.events[event.event_id] = (event, result)
        return result

    def plan(self, robot_id, *, robot_boot_id, ready, now_ns):
        """``SearchCoordinator.plan`` behind the mode gate: no SEARCH_AREA before DISASTER.

        Only the SCOUT is gated; the SUPPORT's HOLD (stop in place) passes.
        """
        role = dict(self.search.policy.roles).get(robot(robot_id))
        if role == 'SCOUT' and self.mode != 'DISASTER':
            return PlanGate('MODE_NOT_DISASTER', None)
        decision = self.search.plan(robot_id, robot_boot_id=robot_boot_id, ready=ready, now_ns=now_ns)
        if decision.reservation is not None and decision.reservation.task_kind == 'SEARCH_AREA':
            self.search_started = True
        return PlanGate('PASSED', decision)

    # -- inputs ------------------------------------------------------------
    def record_target(self, target_id, target_revision, *, detection_confirmed, report_persisted):
        """Latest revision wins; an older one is ignored (False), same revision with other content conflicts.

        A target may be dispatched only when the detection is CONFIRMED and the
        report receipt is PERSISTED (S-00).
        """
        uid(target_id)
        revision(target_revision)
        if type(detection_confirmed) is not bool or type(report_persisted) is not bool:
            raise CoordinationError('INVALID_INPUT', 'target confirmation flags must be boolean')
        value = (target_revision, detection_confirmed, report_persisted)
        return self._record(self.targets, target_id, value, older='ignore')

    def record_route(self, target_id, *, route_revision, segment_count):
        """The SUPPORT route as (revision, number of segments).

        A newer revision may replace the route only while the SUPPORT is not
        authorized to move (before dispatch or AWAITING_APPROVAL); the count then
        restarts at segment 0 from the stopped position. An older revision is
        rejected: a route is never silently rolled back.
        """
        uid(target_id)
        revision(route_revision)
        if type(segment_count) is not int or segment_count < 1:
            raise CoordinationError('INVALID_INPUT', 'segment_count must be a positive int')
        dispatched = target_id == self.dispatch_target
        current = self.routes.get(target_id)
        if (dispatched and current and current != (route_revision, segment_count)
                and self.segment_phase != 'AWAITING_APPROVAL'):
            raise CoordinationError('INVALID_INPUT', 'route is locked while a segment is approved or moving')
        changed = self._record(self.routes, target_id, (route_revision, segment_count), older='reject')
        if changed and dispatched:
            self.segment_index = 0
        return changed

    def segment_started(self, target_id, *, route_revision, segment_index):
        """The executor admitted the approved segment: APPROVED -> MOVING. False if it is not that segment."""
        if not self._is_segment(target_id, route_revision, segment_index) or self.segment_phase != 'APPROVED':
            return False
        self.segment_phase = 'MOVING'
        return True

    def segment_finished(self, target_id, *, route_revision, segment_index, completed, stop_confirmed):
        """A segment ended. Needs a confirmed stop; never authorizes the next segment.

        Completed: the next segment awaits approval, or ARRIVED after the last.
        Not completed: the same segment awaits approval again (the operator decides).
        """
        if type(completed) is not bool or type(stop_confirmed) is not bool:
            raise CoordinationError('INVALID_INPUT', 'completed/stop_confirmed must be boolean')
        if (not stop_confirmed or not self._is_segment(target_id, route_revision, segment_index)
                or self.segment_phase != 'MOVING'):
            return False
        if completed:
            self.segment_index += 1
        self.segment_phase = 'ARRIVED' if self.segment_index == self.segment_count else 'AWAITING_APPROVAL'
        return True

    def record_rescue_verdict(self, target_id, state):
        """The rescue judgement for the pending recheck (RESCUE_CONFIRMED or RECHECK_NEEDED)."""
        if target_id != self.dispatch_target or not self.recheck_pending:
            raise CoordinationError('INVALID_INPUT', 'no recheck pending for this target')
        if state not in VERDICTS:
            raise CoordinationError('INVALID_INPUT', 'verdict must be RESCUE_CONFIRMED or RECHECK_NEEDED')
        self.verdict, self.recheck_pending = state, False

    def record_returned(self, robot_id):
        """A role robot is back after RETURN_HOME; FINISHED when every role robot is."""
        robot(robot_id)
        if not self.returning or robot_id not in dict(self.search.policy.roles):
            raise CoordinationError('INVALID_INPUT', 'no return accepted for this robot')
        self.returned.add(robot_id)

    # -- operator command gate --------------------------------------------
    def handle_command(self, command):
        """Judge one operator command on the server side (console-and-db-plan 4).

        A replayed command_id returns its first result unchanged; the same ID
        with other content is ``ID_CONTENT_CONFLICT``.
        """
        if not isinstance(command, OperatorCommand):
            raise CoordinationError('INVALID_INPUT', 'OperatorCommand required')
        seen = self.commands.get(command.command_id)
        if seen is not None:
            if seen[0] != command:
                raise CoordinationError('ID_CONTENT_CONFLICT', 'command content differs for the same command_id')
            return seen[1]
        reason = 'MODE_NOT_DISASTER' if self.mode != 'DISASTER' else getattr(self, '_' + command.kind.lower())(command)
        result = CommandResult(command.command_id, reason == 'ACCEPTED', reason)
        self.commands[command.command_id] = (command, result)
        return result

    def _dispatch_support(self, command):
        target = self.targets.get(command.target_id)
        if self.dispatch_target is not None:
            return 'ALREADY_DISPATCHED'
        if target is None:
            return 'UNKNOWN_TARGET'
        current_revision, detection_confirmed, report_persisted = target
        if not (detection_confirmed and report_persisted):
            return 'TARGET_NOT_CONFIRMED'
        if command.target_revision != current_revision:
            return 'TARGET_REVISION_STALE'
        if command.target_id not in self.routes:
            return 'NO_ROUTE'
        self.dispatch_target, self.segment_index, self.segment_phase = command.target_id, 0, 'APPROVED'
        return 'ACCEPTED'

    def _continue_segment(self, command):
        if command.target_id != self.dispatch_target:
            return 'NOT_DISPATCHED_TARGET'
        if self.segment_phase != 'AWAITING_APPROVAL':
            return 'NOT_AWAITING_APPROVAL'
        if command.route_revision != self.route_revision:
            return 'ROUTE_REVISION_STALE'
        if command.segment_index != self.segment_index:
            return 'SEGMENT_INDEX_MISMATCH'
        self.segment_phase = 'APPROVED'
        return 'ACCEPTED'

    def _request_recheck(self, command):
        if command.target_id != self.dispatch_target:
            return 'NOT_DISPATCHED_TARGET'
        if self.segment_phase != 'ARRIVED':
            return 'NOT_ARRIVED'
        if self.recheck_pending:
            return 'RECHECK_ALREADY_REQUESTED'
        self.recheck_pending, self.verdict = True, None
        return 'ACCEPTED'

    def _return_home(self, command):
        if self.returning:
            return 'ALREADY_RETURNING'
        if self.verdict is None:
            return 'NO_VERDICT'
        self.returning = True
        return 'ACCEPTED'

    # -- display -----------------------------------------------------------
    def phase(self):
        """console-and-db-plan 1 phase; the latest reached stage wins."""
        if self.mode != 'DISASTER':
            return 'IDLE'
        if self.returning:
            roles = {r for r, _ in self.search.policy.roles}
            return 'FINISHED' if self.returned >= roles else 'RETURNING'
        if self.dispatch_target is not None:
            if self.recheck_pending:
                return 'RECHECKING'
            if self.verdict is not None:
                return self.verdict
            return {'ARRIVED': 'ARRIVED', 'AWAITING_APPROVAL': 'AWAITING_CONTINUE', 'MOVING': 'SEGMENT_MOVING',
                    'APPROVED': 'DISPATCH_APPROVED' if self.segment_index == 0 else 'SEGMENT_MOVING'
                    }[self.segment_phase]
        if any(confirmed and persisted for _, confirmed, persisted in self.targets.values()):
            return 'TARGET_FOUND'
        return 'SEARCHING' if self.search_started else 'DISASTER'

    def view(self):
        return MissionView(self.mode, self.phase(), self.trigger_event_id, self.trigger_observed_at_ns,
                           self.dispatch_target, self.route_revision, self.segment_index, self.segment_count,
                           self.segment_phase)

    # -- helpers -----------------------------------------------------------
    def _is_segment(self, target_id, route_revision, segment_index):
        return (target_id == self.dispatch_target and route_revision == self.route_revision
                and segment_index == self.segment_index)

    @staticmethod
    def _record(table, key, value, *, older):
        """Revision-ordered record; ``value[0]`` is the revision. True when stored."""
        current = table.get(key)
        if current is not None:
            if value[0] == current[0]:
                if value != current:
                    raise CoordinationError('ID_CONTENT_CONFLICT', 'same revision with other content')
                return False
            if value[0] < current[0]:
                if older == 'reject':
                    raise CoordinationError('INVALID_INPUT', 'revision went back')
                return False
        table[key] = value
        return True
