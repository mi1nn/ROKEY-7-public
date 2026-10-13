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
import math

from rescue_contracts import RESCUE_STATES, SiteEvent, timestamp_ns

from .planning import CoordinationError, contract_value, finite, point, revision, robot, uid
from .search import Decision, SearchCoordinator

MODE_REJECT_REASONS = frozenset({'NOT_DISASTER_SIGN', 'TIME_BASIS_UNUSABLE'})
MODE_EVENT_REASONS = MODE_REJECT_REASONS | {'ENTERED_DISASTER', 'ALREADY_DISASTER'}
PLAN_GATE_REASONS = frozenset({'PASSED', 'MODE_NOT_DISASTER', 'SEARCH_NOT_STARTED'})
# Commands this gate judges (console-and-db-plan 4). STOP/PAUSE/RESUME/CANCEL go
# elsewhere and are rejected here as invalid input. START_SEARCH is how AMR1 starts
# (owner decision 2026-10-11, #85): the disaster event alone only shows and stores.
COMMAND_FIELDS = {
    'START_SEARCH': (),
    'DISPATCH_SUPPORT': ('target_id', 'target_revision'),
    'CONTINUE_SEGMENT': ('target_id', 'target_revision', 'route_revision', 'segment_index'),
    'REQUEST_RECHECK': ('target_id',),
    'RETURN_HOME': (),
}
# Commands the console has no button for yet (its tests mirror COMMAND_FIELDS, which drives its buttons).
# SET_CAMERA_WATCH: C-11 row 2; the ROI and camera are bound by the node core.
CAMERA_COMMAND_FIELDS = {'SET_CAMERA_WATCH': ('target_id',)}
ALL_COMMAND_FIELDS = COMMAND_FIELDS | CAMERA_COMMAND_FIELDS
OPTIONAL_FIELDS = ('target_id', 'target_revision', 'route_revision', 'segment_index')
COMMAND_REJECT_REASONS = frozenset({
    'MODE_NOT_DISASTER', 'UNKNOWN_TARGET', 'TARGET_NOT_CONFIRMED', 'TARGET_REVISION_STALE', 'NO_ROUTE',
    'ALREADY_DISPATCHED', 'NOT_DISPATCHED_TARGET', 'NOT_AWAITING_APPROVAL', 'ROUTE_REVISION_STALE',
    'SEGMENT_INDEX_MISMATCH', 'TARGET_ALREADY_JUDGED', 'RECHECK_ALREADY_REQUESTED', 'NO_VERDICT', 'ALREADY_RETURNING',
    'NO_HOME_POSITION', 'HOME_BASIS_MISMATCH', 'SEARCH_ALREADY_STARTED'})
# APPROVED: the operator authorized segment ``segment_index``; MOVING: the executor admitted it;
# AWAITING_APPROVAL: stopped, the next segment needs CONTINUE_SEGMENT; ARRIVED: route done.
SEGMENT_PHASES = frozenset({'APPROVED', 'MOVING', 'AWAITING_APPROVAL', 'ARRIVED'})
# AMR2 automatic dispatch (owner decision 2026-10-13, 1B): ISSUED = the coordinator dispatched the
# saved target; MOVING = the executor admitted the APPROACH_TARGET; ARRIVED = completed with a confirmed
# stop; FAILED = ended otherwise with a confirmed stop (not retried).
APPROACH_PHASES = frozenset({'ISSUED', 'MOVING', 'ARRIVED', 'FAILED'})
APPROACH_REJECT_REASONS = frozenset({'MODE_NOT_DISASTER', 'ALREADY_DISPATCHED', 'UNKNOWN_TARGET',
                                     'TARGET_NOT_CONFIRMED', 'TARGET_REVISION_STALE', 'TARGET_ALREADY_JUDGED'})
# console-and-db-plan 1. CANDIDATE needs TARGET_DISAPPEARED_CANDIDATE (#48) and is not reached yet.
MISSION_PHASES = frozenset({
    'IDLE', 'DISASTER', 'SEARCHING', 'TARGET_FOUND', 'DISPATCH_APPROVED', 'SEGMENT_MOVING',
    'AWAITING_CONTINUE', 'ARRIVED', 'CANDIDATE', 'RECHECKING', 'RESCUE_CONFIRMED', 'RECHECK_NEEDED',
    'RETURNING', 'FINISHED'})
VERDICTS = RESCUE_STATES - {'NOT_CHECKED'}


@dataclass(frozen=True)
class HomePosition:
    """Where a role robot returns to, with the map basis it was set on.

    Not in profile v2 yet (only the SUPPORT's ``hold_positions``); the caller
    passes it and a missing one is refused, never defaulted (작전 2026-10-10).
    """
    robot_id: str
    mission_xy: tuple
    yaw: float
    map_session_id: str
    map_revision: int
    transform_revision: int

    def __post_init__(self):
        robot(self.robot_id)
        point(self.mission_xy)
        finite(self.yaw, 'yaw', minimum=-math.inf)
        uid(self.map_session_id)
        revision(self.map_revision)
        revision(self.transform_revision)

    @property
    def basis(self):
        return self.map_session_id, self.map_revision, self.transform_revision


@dataclass(frozen=True)
class SegmentApproval:
    """One operator approval = one segment (ONE_SEGMENT). A MOVE_SEGMENT Task must carry the same keys."""
    command_id: str
    target_id: str
    target_revision: int
    route_revision: int
    segment_index: int

    def key(self):
        return self.target_id, self.target_revision, self.route_revision, self.segment_index


@dataclass(frozen=True)
class RecheckRequest:
    """An open SCOUT recheck of one target (C-11 rows 10-11).

    ``source`` is OPERATOR: camera candidates (row 4-5) are not taken in yet, so
    ``link_id`` is the command_id sitting in the candidate event_id slot, and
    ``accepted_at_ns`` is when this coordinator accepted it (row 11 ordering).
    """
    link_id: str
    target_id: str
    source: str
    accepted_at_ns: int


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
        required = ALL_COMMAND_FIELDS.get(self.kind) if isinstance(self.kind, str) else None
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
    target_revision: int | None
    route_revision: int | None
    segment_index: int | None
    segment_count: int | None
    segment_phase: str | None


class MissionCoordinator:
    """One mission: one trigger, one SUPPORT dispatch (Phase 1)."""

    def __init__(self, search, *, home_positions):
        if not isinstance(search, SearchCoordinator):
            raise CoordinationError('INVALID_POLICY', 'search coordinator type')
        roles = dict(search.policy.roles)
        homes = tuple(home_positions)
        if not all(isinstance(h, HomePosition) and h.robot_id in roles for h in homes) or (
                len({h.robot_id for h in homes}) != len(homes)):
            raise CoordinationError('INVALID_POLICY', 'home positions must name role robots once')
        self.search = search
        self.homes = {h.robot_id: h for h in homes}
        self.map_basis = {}  # robot_id -> (map_session_id, map_revision, transform_revision) now in use
        self.mode = 'NORMAL'
        self.trigger_event_id = None
        self.trigger_observed_at_ns = None
        self.events = {}    # event_id -> (SiteEvent, ModeResult)
        self.commands = {}  # command_id -> (OperatorCommand, CommandResult)
        self.targets = {}   # target_id -> (target_revision, detection_confirmed, report_persisted)
        self.routes = {}    # target_id -> (route_revision, segment_count)
        self.search_started = False
        self.search_authorized = False  # operator START_SEARCH accepted (#85)
        self.dispatch_target = None
        self.dispatch_target_revision = None
        self.dispatch_kind = None    # 'SEGMENTS' (operator DISPATCH_SUPPORT) or 'APPROACH' (automatic)
        self.approach_phase = None   # APPROACH_PHASES while dispatch_kind == 'APPROACH'
        self.approval = None  # SegmentApproval while APPROVED/MOVING
        self.segment_index = None
        self.segment_phase = None
        self.open_rechecks = {}  # target_id -> RecheckRequest; one open recheck per target
        self.verdicts = {}       # target_id -> RESCUE_CONFIRMED | RECHECK_NEEDED
        self.verdict = None      # the latest verdict, for the displayed phase
        self.returning = False
        self.returned = set()

    @property
    def route_revision(self):
        if self.dispatch_kind == 'APPROACH':
            return 0  # one computed goal, no configured route
        return self.routes[self.dispatch_target][0] if self.dispatch_target else None

    @property
    def segment_count(self):
        if self.dispatch_kind == 'APPROACH':
            return 1
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
        if role == 'SCOUT' and not self.search_authorized:
            return PlanGate('SEARCH_NOT_STARTED', None)
        decision = self.search.plan(robot_id, robot_boot_id=robot_boot_id, ready=ready, now_ns=now_ns)
        if decision.reservation is not None and decision.reservation.task_kind == 'SEARCH_AREA':
            self.search_started = True
        return PlanGate('PASSED', decision)

    # -- inputs ------------------------------------------------------------
    def record_target(self, target_id, target_revision, *, detection_confirmed, report_persisted):
        """Latest revision wins; an older one is ignored (False), same revision with other content conflicts,
        except ``report_persisted`` turning True later for the same revision.

        A target may be dispatched only when the detection is CONFIRMED and the
        report receipt is PERSISTED (S-00).
        """
        uid(target_id)
        revision(target_revision)
        if type(detection_confirmed) is not bool or type(report_persisted) is not bool:
            raise CoordinationError('INVALID_INPUT', 'target confirmation flags must be boolean')
        value = (target_revision, detection_confirmed, report_persisted)
        if self.targets.get(target_id) == (target_revision, detection_confirmed, False) and report_persisted:
            self.targets[target_id] = value  # the save receipt arrives after the revision; never the reverse
            return True
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

    def record_map_basis(self, robot_id, *, map_session_id, map_revision, transform_revision):
        """The map basis a role robot runs on now (mapping); RETURN_HOME compares homes with it."""
        robot(robot_id)
        uid(map_session_id)
        revision(map_revision)
        revision(transform_revision)
        if robot_id not in dict(self.search.policy.roles):
            raise CoordinationError('WRONG_ROBOT', 'robot has no role in this policy')
        self.map_basis[robot_id] = (map_session_id, map_revision, transform_revision)

    def segment_started(self, target_id, *, target_revision, route_revision, segment_index):
        """The executor admitted the approved segment: APPROVED -> MOVING. False if it is not that segment."""
        if not self._is_segment(target_id, target_revision, route_revision, segment_index) or (
                self.segment_phase != 'APPROVED'):
            return False
        self.segment_phase = 'MOVING'
        return True

    def segment_finished(self, target_id, *, target_revision, route_revision, segment_index, completed,
                         stop_confirmed):
        """A segment ended. Needs a confirmed stop; never authorizes the next segment.

        Completed: the next segment awaits approval, or ARRIVED after the last.
        Not completed: the same segment awaits approval again (the operator decides).
        """
        if type(completed) is not bool or type(stop_confirmed) is not bool:
            raise CoordinationError('INVALID_INPUT', 'completed/stop_confirmed must be boolean')
        if (not stop_confirmed or not self._is_segment(target_id, target_revision, route_revision, segment_index)
                or self.segment_phase != 'MOVING'):
            return False
        self.approval = None
        if completed:
            self.segment_index += 1
        self.segment_phase = 'ARRIVED' if self.segment_index == self.segment_count else 'AWAITING_APPROVAL'
        return True

    # -- automatic dispatch (owner decision 2026-10-13, 1B) ---------------------
    def start_approach(self, target_id, target_revision):
        """Dispatch the SUPPORT to a saved target without an operator: the same gate as DISPATCH_SUPPORT.

        Once per mission: CONFIRMED, its current revision PERSISTED, not judged. Returns 'ACCEPTED' or a
        reason from ``APPROACH_REJECT_REASONS``.
        """
        uid(target_id)
        revision(target_revision)
        if self.mode != 'DISASTER':
            return 'MODE_NOT_DISASTER'
        if self.dispatch_target is not None:
            return 'ALREADY_DISPATCHED'
        target = self.targets.get(target_id)
        if target is None:
            return 'UNKNOWN_TARGET'
        if not (target[1] and target[2]):
            return 'TARGET_NOT_CONFIRMED'
        if target_revision != target[0]:
            return 'TARGET_REVISION_STALE'
        if self.verdicts.get(target_id) == 'RESCUE_CONFIRMED':
            return 'TARGET_ALREADY_JUDGED'
        self.dispatch_target, self.dispatch_target_revision = target_id, target_revision
        self.dispatch_kind, self.approach_phase, self.segment_index = 'APPROACH', 'ISSUED', 0
        return 'ACCEPTED'

    def approach_started(self):
        """The executor admitted the APPROACH_TARGET: ISSUED -> MOVING. False otherwise."""
        if self.dispatch_kind != 'APPROACH' or self.approach_phase != 'ISSUED':
            return False
        self.approach_phase = 'MOVING'
        return True

    def approach_finished(self, *, completed, stop_confirmed):
        """The approach ended. Needs a confirmed stop; COMPLETED -> ARRIVED, anything else -> FAILED."""
        if type(completed) is not bool or type(stop_confirmed) is not bool:
            raise CoordinationError('INVALID_INPUT', 'completed/stop_confirmed must be boolean')
        if not stop_confirmed or self.dispatch_kind != 'APPROACH' or self.approach_phase not in ('ISSUED', 'MOVING'):
            return False
        self.approach_phase = 'ARRIVED' if completed else 'FAILED'
        return True

    def record_rescue_verdict(self, target_id, state):
        """The rescue judgement that closes the target's open recheck (RESCUE_CONFIRMED or RECHECK_NEEDED)."""
        if target_id not in self.open_rechecks:
            raise CoordinationError('INVALID_INPUT', 'no recheck open for this target')
        if state not in VERDICTS:
            raise CoordinationError('INVALID_INPUT', 'verdict must be RESCUE_CONFIRMED or RECHECK_NEEDED')
        del self.open_rechecks[target_id]
        self.verdicts[target_id] = self.verdict = state

    def record_returned(self, robot_id):
        """A role robot is back after RETURN_HOME; FINISHED when every role robot is."""
        robot(robot_id)
        if not self.returning or robot_id not in dict(self.search.policy.roles):
            raise CoordinationError('INVALID_INPUT', 'no return accepted for this robot')
        self.returned.add(robot_id)

    # -- operator command gate --------------------------------------------
    def handle_command(self, command, *, now_ns):
        """Judge one operator command on the server side (console-and-db-plan 4).

        A replayed command_id returns its first result unchanged, even if the
        target was judged since (C-11 row 10 ①); the same ID with other content
        is ``ID_CONTENT_CONFLICT``. ``now_ns`` is this coordinator's acceptance time.
        """
        contract_value(timestamp_ns, now_ns)
        if not isinstance(command, OperatorCommand):
            raise CoordinationError('INVALID_INPUT', 'OperatorCommand required')
        seen = self.commands.get(command.command_id)
        if seen is not None:
            if seen[0] != command:
                raise CoordinationError('ID_CONTENT_CONFLICT', 'command content differs for the same command_id')
            return seen[1]
        reason = 'MODE_NOT_DISASTER' if self.mode != 'DISASTER' else (
            getattr(self, '_' + command.kind.lower())(command, now_ns))
        result = CommandResult(command.command_id, reason == 'ACCEPTED', reason)
        self.commands[command.command_id] = (command, result)
        return result

    def _start_search(self, command, now_ns):
        """The operator's START_SEARCH (#85): once per mission, in DISASTER only."""
        if self.search_authorized:
            return 'SEARCH_ALREADY_STARTED'
        self.search_authorized = True
        return 'ACCEPTED'

    def cancel_search_start(self):
        """Withdraw an accepted START_SEARCH whose first area was not reserved yet. True when withdrawn."""
        if not self.search_authorized or self.search_started:
            return False
        self.search_authorized = False
        return True

    def _dispatch_support(self, command, now_ns):
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
        self.dispatch_target, self.segment_index, self.dispatch_kind = command.target_id, 0, 'SEGMENTS'
        self._approve(command)
        return 'ACCEPTED'

    def _continue_segment(self, command, now_ns):
        if command.target_id != self.dispatch_target:
            return 'NOT_DISPATCHED_TARGET'
        if self.segment_phase != 'AWAITING_APPROVAL':
            return 'NOT_AWAITING_APPROVAL'
        if command.target_revision != self.targets[command.target_id][0]:
            return 'TARGET_REVISION_STALE'
        if command.route_revision != self.route_revision:
            return 'ROUTE_REVISION_STALE'
        if command.segment_index != self.segment_index:
            return 'SEGMENT_INDEX_MISMATCH'
        self._approve(command)
        return 'ACCEPTED'

    def _approve(self, command):
        self.dispatch_target_revision = command.target_revision
        self.approval = SegmentApproval(command.command_id, command.target_id, command.target_revision,
                                        self.route_revision, self.segment_index)
        self.segment_phase = 'APPROVED'

    def _request_recheck(self, command, now_ns):
        """C-11 row 10 (Q-26): any recorded target; arrival and dispatch are not conditions."""
        if command.target_id not in self.targets:
            return 'UNKNOWN_TARGET'
        if self.verdicts.get(command.target_id) == 'RESCUE_CONFIRMED':
            return 'TARGET_ALREADY_JUDGED'
        if command.target_id in self.open_rechecks:
            return 'RECHECK_ALREADY_REQUESTED'
        self.open_rechecks[command.target_id] = RecheckRequest(command.command_id, command.target_id, 'OPERATOR',
                                                               now_ns)
        return 'ACCEPTED'

    def _set_camera_watch(self, command, now_ns):
        """C-11 row 2 / #124: a recorded, confirmed, not yet judged target, in DISASTER only."""
        target = self.targets.get(command.target_id)
        if target is None:
            return 'UNKNOWN_TARGET'
        if not target[1]:
            return 'TARGET_NOT_CONFIRMED'
        if self.verdicts.get(command.target_id) == 'RESCUE_CONFIRMED':
            return 'TARGET_ALREADY_JUDGED'
        return 'ACCEPTED'

    def _return_home(self, command, now_ns):
        if self.returning:
            return 'ALREADY_RETURNING'
        if self.verdict is None:
            return 'NO_VERDICT'
        robots = [r for r, _ in self.search.policy.roles]
        if any(r not in self.homes for r in robots):
            return 'NO_HOME_POSITION'
        if any(self.map_basis.get(r) != self.homes[r].basis for r in robots):
            return 'HOME_BASIS_MISMATCH'
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
        if self.open_rechecks:
            return 'RECHECKING'
        if self.verdict is not None:
            return self.verdict
        if self.dispatch_kind == 'APPROACH':
            # A failed approach leaves the target found and undispatched on screen (SystemState.reason says why).
            return {'ISSUED': 'DISPATCH_APPROVED', 'MOVING': 'SEGMENT_MOVING', 'ARRIVED': 'ARRIVED',
                    'FAILED': 'TARGET_FOUND'}[self.approach_phase]
        if self.dispatch_target is not None:
            return {'ARRIVED': 'ARRIVED', 'AWAITING_APPROVAL': 'AWAITING_CONTINUE', 'MOVING': 'SEGMENT_MOVING',
                    'APPROVED': 'DISPATCH_APPROVED' if self.segment_index == 0 else 'SEGMENT_MOVING'
                    }[self.segment_phase]
        if any(confirmed and persisted for _, confirmed, persisted in self.targets.values()):
            return 'TARGET_FOUND'
        return 'SEARCHING' if self.search_started else 'DISASTER'

    def view(self):
        segment_phase = self.segment_phase
        if self.dispatch_kind == 'APPROACH':  # shown with the segment vocabulary: one segment, no approval
            segment_phase = {'ISSUED': 'APPROVED', 'MOVING': 'MOVING', 'ARRIVED': 'ARRIVED', 'FAILED': None
                             }[self.approach_phase]
        return MissionView(self.mode, self.phase(), self.trigger_event_id, self.trigger_observed_at_ns,
                           self.dispatch_target, self.dispatch_target_revision, self.route_revision,
                           self.segment_index, self.segment_count, segment_phase)

    # -- helpers -----------------------------------------------------------
    def _is_segment(self, *key):
        return self.approval is not None and self.approval.key() == key

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
