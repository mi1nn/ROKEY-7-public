"""ROS 2 boundary for the mission coordinator (SDD central.coordinator; IF-016/021/043/076/079).

ROS packages are imported inside functions, so this module stays importable and
``CoordinatorCore`` is testable without ROS. Thin version for the 10-13 run:

- ``/rescue/site_events`` DISASTER_SIGN -> ``MissionCoordinator`` mode gate; once in
  DISASTER a ready SCOUT gets ``ExecuteTask(SEARCH_AREA)`` with no operator approval.
- ``/rescue/robots/{r}/task_admission`` is the only path to ACTIVE; the action
  result releases the reservation only with a confirmed stop.
- ``/rescue/operator_command`` (SubmitOperatorCommand) is judged by
  ``MissionCoordinator.handle_command``; a replayed command_id answers DUPLICATE
  with the first answer (C-11 row 10).

Not yet (PR body): MOVE_SEGMENT/REOBSERVE_TARGET issuance (no path evidence
source, 작전 Q7), CommandLease, SystemState publishing (2차 IDL), target intake,
SetCameraWatch, RETURN_HOME assignment.
"""
import subprocess
from pathlib import Path
from uuid import uuid4

from rescue_contracts import RescueError, SiteEvent, TaskAdmission, duration_ns, seconds_to_ns, timestamp_ns

from .mission import COMMAND_FIELDS, MissionCoordinator, OperatorCommand
from .planning import Allocator, CoordinationError, contract_value
from .search import SearchCoordinator, SearchPolicy, mission_sites

SITE_EVENT_TOPIC = '/rescue/site_events'
OPERATOR_COMMAND_SERVICE = '/rescue/operator_command'
ROBOT_PREFIX = '/rescue/robots/'  # + robot_id + '/execute_task' etc. (rescue_interfaces README)
# Mission refusal -> C-09 common code for SubmitOperatorCommand.reason_code (OPS-58, 2026-10-11).
# The REQUEST_RECHECK rows are C-11 row 10; the rest are [제안]. The mission reason is logged.
REASON_CODES = {
    'MODE_NOT_DISASTER': 'NOT_AUTHORIZED',
    'UNKNOWN_TARGET': 'NOT_FOUND', 'NO_HOME_POSITION': 'NOT_FOUND',
    'TARGET_NOT_CONFIRMED': 'PENDING_EVIDENCE', 'NO_ROUTE': 'PENDING_EVIDENCE', 'NO_VERDICT': 'PENDING_EVIDENCE',
    'ALREADY_DISPATCHED': 'TASK_BUSY', 'NOT_AWAITING_APPROVAL': 'TASK_BUSY', 'RECHECK_ALREADY_REQUESTED': 'TASK_BUSY',
    'ALREADY_RETURNING': 'TASK_BUSY',
    'TARGET_REVISION_STALE': 'INVALID_INPUT', 'ROUTE_REVISION_STALE': 'INVALID_INPUT',
    'SEGMENT_INDEX_MISMATCH': 'INVALID_INPUT', 'NOT_DISPATCHED_TARGET': 'INVALID_INPUT',
    'TARGET_ALREADY_JUDGED': 'INVALID_INPUT',
    'HOME_BASIS_MISMATCH': 'FRAME_REVISION_MISMATCH',
}
# The robot that a command may name (empty = not named).
COMMAND_ROLES = {'DISPATCH_SUPPORT': 'SUPPORT', 'CONTINUE_SEGMENT': 'SUPPORT', 'REQUEST_RECHECK': 'SCOUT'}
EXECUTOR_OUTCOMES = frozenset({'COMPLETED', 'CANCELED', 'EXPIRED', 'UNREACHABLE', 'FAILED'})


def git_sha():
    """Source checkout SHA, or 'UNKNOWN' (installed copies have no .git)."""
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def _invalid(detail):
    return CoordinationError('INVALID_INPUT', detail)


def site_event_from_msg(msg):
    """rescue_interfaces/SiteEvent -> rescue_contracts.SiteEvent (C-10).

    Only VALID visibility is an event; empty target/watch IDs are None; a missing
    error bound is None. Nothing is repaired: a bad message is INVALID_INPUT.
    """
    if msg.visibility_state != 'VALID':
        raise _invalid('site event visibility must be VALID')
    time = msg.time
    try:
        return SiteEvent(
            event_id=msg.event_id, camera_id=msg.camera_id, kind=msg.kind, observed_at_ns=time.observed_at_ns,
            evidence_id=msg.evidence.evidence_id, evidence_sha256=msg.evidence.sha256,
            confidence=float(msg.confidence), clock_domain_id=time.clock_domain_id,
            stamp_semantics=time.stamp_semantics, clock_state=time.clock_state,
            clock_error_bound_ns=time.clock_error_bound_ns if time.has_clock_error_bound else None,
            target_id=msg.target_id or None, watch_id=msg.watch_id or None)
    except RescueError as exc:
        raise _invalid(exc.detail) from exc


def admission_from_msg(msg):
    """rescue_interfaces/TaskAdmission -> rescue_contracts.TaskAdmission (the executor receipt)."""
    try:
        return TaskAdmission(msg.command_id, msg.task_id, msg.robot_id, msg.robot_boot_id, msg.authority_epoch,
                             msg.generation, bool(msg.accepted), 'ACCEPTED' if msg.accepted else msg.reason_code)
    except RescueError as exc:
        raise _invalid(exc.detail) from exc


def fill_execute_task_goal(goal, fields, *, coordinator_id):
    """``Decision.task_fields`` -> ExecuteTask goal (attribute assignment only).

    ``frame_ref.alignment_status`` stays empty: alignment is the mapping's call
    and the coordinator never asserts VALID. ``lease_id`` stays empty until
    CommandLease publishing exists; the executor must treat both as not given.
    """
    pose = fields['goal_pose']
    for name in ('command_id', 'task_id', 'robot_id', 'authority_epoch', 'generation', 'kind', 'issued_at_ns',
                 'deadline_ns'):
        setattr(goal, name, fields[name])
    goal.expected_robot_boot_id = fields['robot_boot_id']
    goal.coordinator_id = coordinator_id
    goal.policy_version = fields['policy_version']
    goal.target_id = fields['target_id'] or ''
    goal.target_estimate_revision = fields['target_revision'] or 0
    goal.search_area_ref = fields['search_area_ref'] or ''
    frame = goal.frame_ref
    frame.robot_id = fields['robot_id']
    if pose is not None:
        frame.mission_frame = pose['frame_id']
        frame.map_session_id = pose['map_session_id']
        frame.map_revision = pose['map_revision']
        frame.transform_revision = pose['transform_revision']
    goal.has_route = goal.has_approval = False
    return goal


def operator_command_from_request(req, roles):
    """SubmitOperatorCommand request -> OperatorCommand. Fields the operation does not use are not read."""
    if req.operation not in COMMAND_FIELDS:
        raise _invalid('operation not handled by the mission gate')
    if req.robot_id and req.robot_id != roles.get(COMMAND_ROLES.get(req.operation)):
        raise _invalid('robot_id must be empty or the role robot of this operation')
    used = COMMAND_FIELDS[req.operation]
    values = {'target_id': req.target_id or None, 'target_revision': req.expected_target_revision,
              'route_revision': req.route_revision, 'segment_index': req.segment_index}
    return OperatorCommand(command_id=req.command_id, kind=req.operation, operator_id=req.operator_id,
                           requested_at_ns=req.issued_at_ns,
                           **{name: values[name] if name in used else None for name in values})


class CoordinatorCore:
    """Everything the node decides, without ROS. The node only moves messages."""

    def __init__(self, mission, *, status_max_age_ns):
        if not isinstance(mission, MissionCoordinator):
            raise CoordinationError('INVALID_POLICY', 'mission coordinator type')
        contract_value(duration_ns, status_max_age_ns)
        self.mission = mission
        self.search = mission.search
        self.roles = {role: robot_id for robot_id, role in self.search.policy.roles}
        self.status_max_age_ns = status_max_age_ns
        self.status = {}     # robot_id -> latest RobotStatus fields used here
        self.replies = {}    # command_id -> (OperatorCommand, reply dict)
        self.last_phase = None
        self.last_reason = None  # mission reason behind the latest reply, for the node log

    # -- inputs --------------------------------------------------------------
    def on_site_event(self, msg):
        return self.mission.record_site_event(site_event_from_msg(msg))

    def on_status(self, msg):
        if msg.robot_id not in self.roles.values():
            raise CoordinationError('WRONG_ROBOT', 'status from a robot outside the policy')
        self.status[msg.robot_id] = (msg.robot_boot_id, msg.readiness, bool(msg.lease_valid), msg.observed_at_ns)

    def on_admission(self, msg):
        return self.search.record_admission(admission_from_msg(msg))

    def on_result(self, task_id, result):
        """Action result for one task; released only with a confirmed stop (Allocator rule)."""
        allocator = self.search.allocator
        item = allocator.active.get(task_id) or allocator.pending.get(task_id)
        if item is None or result.outcome not in EXECUTOR_OUTCOMES:
            return False
        return self.search.finish(item, outcome=result.outcome, stop_confirmed=bool(result.stop_confirmed))

    def ready(self, robot_id, now_ns):
        """(robot_boot_id, ready) from the latest RobotStatus, or None before any status."""
        status = self.status.get(robot_id)
        if status is None:
            return None
        boot, readiness, lease_valid, observed_at_ns = status
        fresh = observed_at_ns <= now_ns and now_ns - observed_at_ns <= self.status_max_age_ns
        return boot, readiness == 'READY' and lease_valid and fresh

    # -- decisions -------------------------------------------------------------
    def tick(self, now_ns):
        """Task fields to send now. Only the SCOUT is driven automatically (Q-12)."""
        contract_value(timestamp_ns, now_ns)
        scout = self.roles['SCOUT']
        state = self.ready(scout, now_ns)
        if state is None or self.mission.mode != 'DISASTER':
            return []
        boot, ready = state
        gate = self.mission.plan(scout, robot_boot_id=boot, ready=ready, now_ns=now_ns)
        decision = gate.decision
        return [decision.task_fields] if decision is not None and decision.action == 'RESERVED' else []

    def on_command(self, req, *, now_ns):
        """SubmitOperatorCommand -> reply fields (accepted, request_state, reason_code, created_task_id)."""
        try:
            command = operator_command_from_request(req, self.roles)
        except RescueError as exc:
            self.last_reason = exc.detail
            return self._reply(False, 'REJECTED', exc.code)
        seen = self.replies.get(command.command_id)
        if seen is not None:
            if seen[0] != command:
                return self._reply(False, 'REJECTED', 'ID_CONTENT_CONFLICT')
            first = seen[1]
            return self._reply(first['accepted'], 'DUPLICATE', first['reason_code'], first['created_task_id'])
        result = self.mission.handle_command(command, now_ns=now_ns)
        self.last_reason = result.reason
        if result.accepted:
            # No MOVE_SEGMENT/REOBSERVE_TARGET Task yet (작전 Q7): accepted = received, no created task.
            reply = self._reply(True, 'ACCEPTED', '')
        else:
            reply = self._reply(False, 'REJECTED', REASON_CODES[result.reason])
        self.replies[command.command_id] = (command, reply)
        return reply

    def phase_change(self):
        """The new phase when it changed since the last call, else None (for the log)."""
        phase = self.mission.phase()
        if phase == self.last_phase:
            return None
        self.last_phase = phase
        return phase

    @staticmethod
    def _reply(accepted, state, reason_code, created_task_id=''):
        return dict(accepted=accepted, request_state=state, reason_code=reason_code,
                    created_task_id=created_task_id)


# ---- ROS wiring (needs a sourced ROS 2 environment) --------------------------------------------
def declare_required(node, names, optional=()):
    """Typed parameters without defaults -> (values, missing). Jazzy rejects a typed
    ``declare_parameter`` with no value, so the type goes in as ``Parameter.Type`` (#35)."""
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    values = {}
    for name, kind in names.items():
        node.declare_parameter(name, Parameter.Type(kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, item in values.items()
                     if item is None or (name not in optional and item in ('', [])))
    return values, missing


def build_core(values):
    """Profile v2 + typed parameters -> CoordinatorCore (no defaults anywhere)."""
    from rescue_bringup.profile import load_deployment
    deployment = load_deployment(values['profile_path'])
    if deployment.mission is None:
        raise CoordinationError('INVALID_POLICY', 'profile v2 mission section required')
    areas = deployment.mission.search_areas
    yaws = list(values['entry_yaws'])
    if len(yaws) != len(areas):
        raise CoordinationError('INVALID_POLICY', 'entry_yaws must give one heading per search area, in order')
    sites = mission_sites(areas, entry_yaws={area.id: yaw for area, yaw in zip(areas, yaws)})
    policy = SearchPolicy(version=values['policy_version'], roles=deployment.mission.roles, areas=sites,
                          executable_bases=frozenset(values['executable_bases']),
                          task_duration_ns=seconds_to_ns(str(values['task_duration_s'])),
                          lease_duration=values['lease_duration_s'],
                          executor_policy_version=values['executor_policy_version'],
                          mission_frame_id=values['mission_frame_id'])
    robot_ids = tuple(robot for robot, _ in deployment.mission.roles)
    search = SearchCoordinator(Allocator(values['authority_epoch'], allowed_robot_ids=robot_ids), policy)
    # No profile home positions yet: RETURN_HOME stays NO_HOME_POSITION (assignment is Q-22, later).
    mission = MissionCoordinator(search, home_positions=())
    return CoordinatorCore(mission, status_max_age_ns=seconds_to_ns(str(values['status_max_age_s'])))


PARAMETERS = {  # name -> rcl_interfaces ParameterType name; every one is required, none has a default
    'profile_path': 'STRING', 'entry_yaws': 'DOUBLE_ARRAY', 'policy_version': 'STRING',
    'executable_bases': 'STRING_ARRAY', 'task_duration_s': 'DOUBLE', 'lease_duration_s': 'DOUBLE',
    'executor_policy_version': 'STRING', 'mission_frame_id': 'STRING', 'authority_epoch': 'STRING',
    'coordinator_id': 'STRING', 'envelope_schema_version': 'STRING', 'status_max_age_s': 'DOUBLE',
    'tick_period_s': 'DOUBLE'}


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterType
    from rescue_interfaces.action import ExecuteTask
    from rescue_interfaces.msg import RobotStatus, SiteEvent as SiteEventMsg, TaskAdmission as AdmissionMsg
    from rescue_interfaces.srv import SubmitOperatorCommand

    rclpy.init(args=args)
    node = Node('mission_coordinator')
    log = node.get_logger()
    try:
        values, missing = declare_required(
            node, {name: getattr(ParameterType, 'PARAMETER_' + kind) for name, kind in PARAMETERS.items()})
        if missing:
            log.fatal(f'missing parameters: {missing}')
            raise SystemExit(2)
        try:
            core = build_core(values)
        except (RescueError, OSError, ValueError) as exc:
            log.fatal(f'configuration rejected: {exc}')
            raise SystemExit(2)
        boot_id, sequence = str(uuid4()), iter(range(1, 2 ** 63))
        log.info(f'git_sha={git_sha()} boot_id={boot_id} parameters={values} roles={core.roles}')

        def now_ns():
            return node.get_clock().now().nanoseconds

        def envelope(msg):
            msg.schema_version = values['envelope_schema_version']
            msg.publisher_id = values['coordinator_id']
            msg.publisher_boot_id = boot_id
            msg.sequence = next(sequence)
            msg.published_at_ns = now_ns()
            return msg

        def guarded(label, callback):
            def run(*args):
                try:
                    return callback(*args)
                except RescueError as exc:  # visible, never a silent fallback (AGENTS 6)
                    log.error(f'{label}: {exc}')
            return run

        clients = {robot: ActionClient(node, ExecuteTask, ROBOT_PREFIX + robot + '/execute_task')
                   for robot in core.roles.values()}

        def send(fields):
            goal = fill_execute_task_goal(ExecuteTask.Goal(), fields, coordinator_id=values['coordinator_id'])
            envelope(goal.envelope)
            client = clients[fields['robot_id']]
            if not client.server_is_ready():
                log.error(f'execute_task server not ready for {fields["robot_id"]}; task {fields["task_id"]} '
                          'stays PENDING until admission or release')
            log.info(f'SEND {fields["kind"]} task={fields["task_id"]} robot={fields["robot_id"]} '
                     f'generation={fields["generation"]}')
            client.send_goal_async(goal).add_done_callback(lambda done: on_goal_response(fields['task_id'], done))

        def on_goal_response(task_id, future):
            handle = future.result()
            log.info(f'GOAL task={task_id} accepted={handle.accepted} (ACTIVE only from task_admission)')
            if handle.accepted:
                handle.get_result_async().add_done_callback(guarded('result', lambda done: log.info(
                    f'RESULT task={task_id} released={core.on_result(task_id, done.result().result.result)}')))

        def tick():
            for fields in core.tick(now_ns()):
                send(fields)
            phase = core.phase_change()
            if phase is not None:  # SystemState is 2차 IDL: logged, not published (작전 Q1)
                log.info(f'PHASE {phase} view={core.mission.view()}')

        def on_site_event(msg):
            log.info(f'SITE_EVENT {msg.event_id} {msg.kind} -> {core.on_site_event(msg)}')
            tick()

        def on_command(req, res):
            reply = core.on_command(req, now_ns=now_ns())
            envelope(res.envelope)
            res.command_id = req.command_id
            for name, value in reply.items():
                setattr(res, name, value)
            log.info(f'COMMAND {req.command_id} {req.operation} -> {reply} reason={core.last_reason}')
            tick()
            return res

        node.create_subscription(SiteEventMsg, SITE_EVENT_TOPIC, guarded('site_event', on_site_event), 10)
        for robot in core.roles.values():
            node.create_subscription(RobotStatus, ROBOT_PREFIX + robot + '/status',
                                     guarded('status', core.on_status), 1)
            node.create_subscription(AdmissionMsg, ROBOT_PREFIX + robot + '/task_admission', guarded(
                'admission', lambda msg: log.info(f'ADMISSION {msg.task_id} -> {core.on_admission(msg)}')), 10)
        node.create_service(SubmitOperatorCommand, OPERATOR_COMMAND_SERVICE, on_command)
        node.create_timer(values['tick_period_s'], guarded('tick', tick))
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        if isinstance(exc, SystemExit) and exc.code not in (None, 0):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
