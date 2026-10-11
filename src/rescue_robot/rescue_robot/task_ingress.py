"""ROS-shaped ExecuteTask goals and CommandLease messages -> CommandAdapter.

No ROS imports. The caller MUST authenticate the publisher/authority in
authorize(goal, lease) and resolve goals from verified mapping/target records.
This boundary does not infer authority from message fields, arm an executor,
approve a target, or provide a field policy. Wire subscriptions/action-server
callbacks remain a separate transport boundary.
"""
from copy import deepcopy
from dataclasses import dataclass
from rescue_contracts import RescueError

from rescue_robot.executor import ContractError, Task, identifier, generation, wall_time
from rescue_robot.task_wire import validate_task_wire, validate_resolved_goal
from rescue_robot.segment_approval import SegmentApprovalGate
from rescue_robot.phase1_route import Phase1RouteGate
from rescue_robot.task_policy import LeasePolicy, validate_clock_basis, validate_lease_reception


@dataclass(frozen=True)
class ExecutionReport:
    """Result plus original command identity; no invented pose/evidence."""
    task_id: str
    command_id: str
    robot_id: str
    robot_boot_id: str
    authority_epoch: str
    generation: int
    result: object


class TaskIngress:
    def __init__(self, adapter, *, authorize, resolve_goal, clock_error_bound_ns,
                 lease_policy, clock_basis, segment_gate=None, phase1_gate=None):
        if not callable(authorize) or not callable(resolve_goal):
            raise ContractError('authorization and goal resolver required')
        if type(clock_error_bound_ns) is not int or clock_error_bound_ns <= 0:
            raise ContractError('clock_error_bound_ns must be a positive integer')
        if (not isinstance(lease_policy, LeasePolicy) or not callable(clock_basis)
                or clock_error_bound_ns > lease_policy.max_error_ns):
            raise ContractError('explicit lease policy and verified clock provider required', 'CLOCK_UNCERTAIN')
        self.adapter = adapter
        self.authorize = authorize
        self.resolve_goal = resolve_goal
        self.clock_error_bound_ns = clock_error_bound_ns
        self.requests = {}
        self.segment_gate = segment_gate
        self.phase1_gate = phase1_gate
        self.lease_policy, self.clock_basis = lease_policy, clock_basis
        self.last_wall_ns = None
        self.last_steady = None
        self.clock_epoch = None
        self.clock_fault = False
        self.last_reason = ''

    def _now(self):
        now = wall_time(self.adapter.wall_clock_ns())
        steady = self.adapter.clock()
        if self.last_wall_ns is not None:
            delta = now - self.last_wall_ns
            expected = round((steady - self.last_steady) * 1e9)
            if delta < 0 or expected < 0 or abs(delta - expected) > 2 * self.clock_error_bound_ns:
                self.clock_fault = True
        if self.clock_fault:
            raise ContractError('clock regressed; recreate runtime after recovery', 'CLOCK_UNCERTAIN')
        self.last_wall_ns, self.last_steady = now, steady
        return now

    def _clock(self, now):
        basis = self.clock_basis()
        error = validate_clock_basis(basis, policy=self.lease_policy, now_ns=now)
        if error > self.clock_error_bound_ns:
            raise ContractError('measurement exceeds configured task clock bound', 'CLOCK_UNCERTAIN')
        if self.clock_epoch is not None and basis.epoch_id != self.clock_epoch:
            self.clock_fault = True
            raise ContractError('clock session changed; recreate runtime', 'CLOCK_UNCERTAIN')
        self.clock_epoch = basis.epoch_id
        return basis

    def validate_lease(self, lease):
        now = self._now()
        return validate_lease_reception(lease, policy=self.lease_policy,
                                       basis=self._clock(now), now_ns=now)

    def _segment(self, goal):
        gate = self.segment_gate
        if (not isinstance(gate, (SegmentApprovalGate, Phase1RouteGate)) or gate.adapter is not self.adapter
                or gate.clock_error_bound_ns != self.clock_error_bound_ns):
            raise ContractError('matching authenticated SegmentApprovalGate required', 'NOT_AUTHORIZED')
        return gate.validate(goal)

    def check(self, task):
        """Continuous guard, composed with the mapping/target guard by the runtime."""
        try:
            self._now()
            self._clock(self.last_wall_ns)
            if task.kind == 'MOVE_SEGMENT':
                gate = self.segment_gate
                if (not isinstance(gate, (SegmentApprovalGate, Phase1RouteGate)) or gate.adapter is not self.adapter
                        or gate.clock_error_bound_ns != self.clock_error_bound_ns):
                    raise ContractError('matching segment gate unavailable', 'NOT_AUTHORIZED')
                if not gate.check(task.command_id):
                    self.last_reason = gate.last_reason
                    return False
            elif task.kind == 'REOBSERVE_TARGET':
                request = self.requests.get(task.command_id)
                if request and self._uses_phase1(request[0]): self._phase1_check(task)
            elif task.kind == 'RETURN_HOME':
                request = self.requests.get(task.command_id)
                if request is None:
                    raise ContractError('home request unavailable', 'NOT_AUTHORIZED')
                goal = request[0]
                pose = validate_resolved_goal(goal, self.resolve_goal(goal))
                if pose != task.goal_pose:
                    raise ContractError('home record changed; require a new task', 'FRAME_REVISION_MISMATCH')
        except Exception as exc:
            self.last_reason = (exc.code if isinstance(exc, RescueError) else 'INVALID_INPUT') + ': ' + str(exc)
            return False
        self.last_reason = ''
        return True

    def _phase1(self, goal):
        gate = self.phase1_gate
        if (not isinstance(gate, Phase1RouteGate) or gate.adapter is not self.adapter
                or gate.clock_error_bound_ns != self.clock_error_bound_ns):
            raise ContractError('matching Phase1RouteGate required', 'NOT_AUTHORIZED')
        return gate.validate(goal)

    @staticmethod
    def _uses_phase1(goal):
        return (goal.kind == 'REOBSERVE_TARGET' and goal.has_route
                and not getattr(goal.route, 'planner_evidence_id', ''))

    def _phase1_check(self, task):
        request = self.requests.get(task.command_id)
        if request is None:
            raise ContractError('phase1 request unavailable', 'NOT_AUTHORIZED')
        self._phase1(request[0])
        return True

    def _authorize(self, goal, lease):
        if self.authorize(goal, lease) is not True:
            raise ContractError('publisher/authority not authorized', 'NOT_AUTHORIZED')

    def _identity(self, goal, lease):
        for name in ('lease_id', 'robot_boot_id', 'authority_epoch'):
            identifier(getattr(lease, name))
        generation(lease.generation)
        generation(lease.renewal_sequence)
        expected = (goal.lease_id, goal.robot_id, goal.expected_robot_boot_id,
                    goal.coordinator_id, goal.authority_epoch, goal.generation)
        actual = (lease.lease_id, lease.robot_id, lease.robot_boot_id,
                  lease.coordinator_id, lease.authority_epoch, lease.generation)
        if actual != expected:
            raise ContractError('lease does not belong to this command', 'NOT_AUTHORIZED')

    def submit(self, goal, lease):
        """Return TaskAdmission, or reject malformed/untrusted wire input.

        Retries preserve the original admission and monotonic expiry; replaying
        a goal is never a lease renewal. A transport maps ContractError to its
        rejection path, rather than creating an accepted action handle.
        """
        with self.adapter.lock:
            self._authorize(goal, lease)
            identifier(goal.command_id)
            if lease is not None:
                self._identity(goal, lease)
            previous = self.requests.get(goal.command_id)
            if previous is not None:
                request, receipt = previous
                if request != goal:
                    raise ContractError('command content changed', 'ID_CONTENT_CONFLICT')
                return receipt
            now = self._now()
            self._clock(now)
            if lease is not None:
                self.validate_lease(lease)
            validate_task_wire(goal, None, now_ns=now,
                               clock_error_bound_ns=self.clock_error_bound_ns)
            if goal.kind == 'MOVE_SEGMENT':
                pose = self._segment(goal).goal_pose
            elif self._uses_phase1(goal):
                pose = self._phase1(goal).goal_pose
            else:
                pose = None if goal.kind == 'HOLD' else self.resolve_goal(goal)
            if not (goal.kind == 'MOVE_SEGMENT' and isinstance(self.segment_gate, Phase1RouteGate)
                    or self._uses_phase1(goal)):
                validate_resolved_goal(goal, pose)
            # Phase1RouteGate already verifies the source and destination
            # versions and returns a GoalPose in the moving robot's basis.
            # Resolution may be slow: use the lifetime left after it finishes.
            now = self._now()
            self._clock(now)
            validate_task_wire(goal, None, now_ns=now, clock_error_bound_ns=self.clock_error_bound_ns)
            if lease is not None:
                self.validate_lease(lease)
            if goal.kind == 'MOVE_SEGMENT': self._segment(goal)
            elif self._uses_phase1(goal): self._phase1(goal)
            duration = self.lease_policy.duration_s
            self._authorize(goal, lease)
            task = Task(goal.command_id, goal.task_id, goal.robot_id,
                        goal.expected_robot_boot_id, goal.authority_epoch,
                        goal.generation, goal.kind, goal.issued_at_ns, goal.deadline_ns,
                        duration, goal.policy_version,
                        target_id=goal.target_id or None,
                        target_revision=goal.target_estimate_revision if goal.target_id else None,
                        search_area_ref=goal.search_area_ref or None, goal_pose=pose)
            receipt = self.adapter.submit(task)
            self.requests[goal.command_id] = (deepcopy(goal), receipt)
            if receipt.accepted and lease is not None:
                # The initial sequence is already consumed. An equal sequence
                # received again must not extend the admission's lease.
                self.adapter.executor.lease_sequence = lease.renewal_sequence
            return receipt

    def renew(self, lease):
        """Check message eligibility; renew the active LOCAL lease by policy duration."""
        with self.adapter.lock:
            active = self.adapter.executor.active
            if active is None or active.command_id not in self.requests:
                return False
            goal, _ = self.requests[active.command_id]
            self._authorize(goal, lease)
            self._identity(goal, lease)
            self.validate_lease(lease)
            validate_task_wire(goal, None, now_ns=self._now(),
                                          clock_error_bound_ns=self.clock_error_bound_ns)
            if goal.kind == 'MOVE_SEGMENT':
                self._segment(goal)
            elif self._uses_phase1(goal):
                self._phase1(goal)
            self._authorize(goal, lease)
            return self.adapter.renew(active.command_id, lease.authority_epoch,
                                      lease.generation, lease.renewal_sequence,
                                      lease_duration=self.lease_policy.duration_s)

    def report(self, command_id):
        """None until the executor's terminal+odometry stop gate has finished.

        Failure with stop_confirmed=False is also a terminal report; callers
        must preserve that value rather than announcing a successful stop.
        """
        with self.adapter.lock:
            request = self.requests.get(command_id)
            result = self.adapter.result(command_id)
            if request is None or not request[1].accepted or result is None:
                return None
            goal = request[0]
            return ExecutionReport(goal.task_id, goal.command_id, goal.robot_id,
                                   goal.expected_robot_boot_id, goal.authority_epoch,
                                   goal.generation, result)
