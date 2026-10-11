"""ExecuteTask action transport for one TaskIngress (Jazzy rclpy boundary).

Runtime integration must supply authenticated lease storage and a result
encoder with actual observation/evidence records. Nothing here arms the robot,
provides an alignment, or chooses a production endpoint/policy.
"""
from copy import deepcopy
from types import SimpleNamespace
from uuid import uuid4

from rescue_contracts import RescueError
from rescue_robot.executor import ContractError, number
from rescue_robot.goal_binding import MissionGoalBinder
from rescue_robot.task_ingress import TaskIngress
from rescue_robot.task_wire import validate_completion_scope
from rescue_robot.task_status import fill_robot_status
from rescue_robot.phase1_route import Phase1RouteGate


def attach_task_runtime(node, adapter, *, registry, target_gate, authorize, resolve_goal,
                        clock_error_bound_ns, lease_policy, clock_basis,
                        segment_gate=None, phase1_gate=None, **server_settings):
    """Explicit integration hook for ros_node.main(task_runtime_factory=...).

    The registry must receive authoritative alignment updates continuously.
    It is not replaced with a fixed identity transform or destination pose.
    Install before arm, never by reconnecting to the action server.
    """
    with adapter.lock:
        if adapter.executor.active is not None or adapter.executor.latch != 'DISARMED':
            raise ContractError('install task runtime before explicit arm')
        if (not callable(getattr(registry, 'map_state', None))
                or not callable(getattr(registry, 'current_alignment', None))
                or not isinstance(getattr(registry, 'mission_frame', None), str)
                or not registry.mission_frame):
            raise ContractError('authoritative mapping registry required')
        binder = MissionGoalBinder(registry, adapter.robot_id, target_gate=target_gate,
                                   wall_clock_ns=adapter.wall_clock_ns)
        for gate in (segment_gate, phase1_gate):
            if isinstance(gate, Phase1RouteGate) and gate.registry is not registry:
                raise ContractError('route gate and motion binder must share authoritative mapping input')
        ingress = TaskIngress(adapter, authorize=authorize, resolve_goal=resolve_goal,
                              clock_error_bound_ns=clock_error_bound_ns, segment_gate=segment_gate,
                              phase1_gate=phase1_gate, lease_policy=lease_policy, clock_basis=clock_basis)
        server = TaskActionServer(node, ingress, **server_settings)
        def guards(task):
            if not ingress.check(task):
                node.get_logger().error(f'task guard rejected {task.command_id}: {ingress.last_reason}')
                return False
            return binder.check(task)
        adapter.guards = guards
        adapter.bind_goal = binder.bind
        return server


def ros_bindings():
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.task import Future
    from rescue_interfaces.action import ExecuteTask
    from rescue_interfaces.msg import CommandLease, TaskAdmission, RobotStatus
    return SimpleNamespace(ActionServer=ActionServer, CancelResponse=CancelResponse,
                           GoalResponse=GoalResponse, ReentrantCallbackGroup=ReentrantCallbackGroup,
                           Future=Future, ExecuteTask=ExecuteTask,
                           CommandLease=CommandLease, TaskAdmission=TaskAdmission, RobotStatus=RobotStatus)


class TaskActionServer:
    def __init__(self, node, ingress, *, action_name, lease_topic, admission_topic, status_topic,
                 poll_period_s, schema_version, publisher_id, lookup_lease,
                 accept_lease, encode_result, bindings=None):
        for value in (action_name, lease_topic, admission_topic, status_topic, schema_version, publisher_id):
            if not isinstance(value, str) or not value:
                raise ContractError('explicit endpoint and envelope settings required')
        if len({action_name, lease_topic, admission_topic, status_topic}) != 4:
            raise ContractError('task/lease/admission/status endpoints must differ')
        for callback in (lookup_lease, accept_lease, encode_result):
            if not callable(callback):
                raise ContractError('lease storage/authentication and result encoder required')
        number(poll_period_s, 'poll_period_s', positive=True)
        self.node, self.ingress = node, ingress
        self.lookup_lease, self.accept_lease = lookup_lease, accept_lease
        self.encode_result = encode_result
        self.schema_version, self.publisher_id = schema_version, publisher_id
        self.sequence = 0
        self.status_sequence = 0
        self.bindings = bindings or ros_bindings()
        self.waiting, self.admissions = {}, {}
        group = self.bindings.ReentrantCallbackGroup()
        self.publisher = node.create_publisher(self.bindings.TaskAdmission, admission_topic, 10)
        self.status_publisher = node.create_publisher(self.bindings.RobotStatus, status_topic, 10)
        self.subscription = node.create_subscription(self.bindings.CommandLease, lease_topic,
                                                     self.on_lease, 10, callback_group=group)
        self.server = self.bindings.ActionServer(
            node, self.bindings.ExecuteTask, action_name, execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel, callback_group=group)
        self.timer = node.create_timer(float(poll_period_s), self.poll, callback_group=group)

    def stamp_envelope(self, envelope):
        # All callers serialize with the adapter lock, including the encoder.
        envelope.schema_version = self.schema_version
        envelope.publisher_id = self.publisher_id
        envelope.publisher_boot_id = self.ingress.adapter.executor.robot_boot_id
        envelope.sequence = self.sequence
        envelope.published_at_ns = self.ingress.adapter.wall_clock_ns()
        self.sequence += 1

    def publish_admission(self, goal, accepted, reason):
        fields = (goal.command_id, goal.task_id, goal.robot_id, goal.expected_robot_boot_id,
                  goal.authority_epoch, goal.generation, accepted, reason)
        if fields not in self.admissions:
            self.admissions[fields] = (str(uuid4()), self.ingress.adapter.wall_clock_ns())
        admission_id, observed = self.admissions[fields]
        msg = self.bindings.TaskAdmission()
        msg.admission_id, msg.observed_at_ns = admission_id, observed
        (msg.command_id, msg.task_id, msg.robot_id, msg.robot_boot_id,
         msg.authority_epoch, msg.generation, msg.accepted, msg.reason_code) = fields
        self.stamp_envelope(msg.envelope)
        self.publisher.publish(msg)

    def on_goal(self, goal):
        with self.ingress.adapter.lock:
            accepted = False
            try:
                lease = self.lookup_lease(goal)
                # #90 sends goal before its first CommandLease. Admission uses
                # explicit local lease policy + verified clock/goal authority;
                # never fabricate an initial wire lease or require its arrival.
                receipt = self.ingress.submit(goal, lease)
                accepted = receipt.accepted
                self.publish_admission(goal, accepted, receipt.reason)
                return (self.bindings.GoalResponse.ACCEPT if accepted
                        else self.bindings.GoalResponse.REJECT)
            except Exception as exc:
                # A resolver/storage/publisher failure must not become an
                # unobserved admitted movement. No step can interleave here.
                if accepted:
                    self.ingress.adapter.request_stop('ACTION_ABORTED', outcome='FAILED')
                reason = exc.code if isinstance(exc, RescueError) else 'INVALID_INPUT'
                self.node.get_logger().error(f'ExecuteTask rejected: {reason}: {exc}')
                try:
                    self.publish_admission(goal, False, reason)
                except Exception as publish_error:
                    self.node.get_logger().error(f'TaskAdmission publish failed: {publish_error}')
                return self.bindings.GoalResponse.REJECT

    def on_lease(self, lease):
        with self.ingress.adapter.lock:
            try:
                # This callback authenticates and stores the wire lease.
                # Initial leases may arrive before there is an active task.
                self.ingress.validate_lease(lease)
                if self.accept_lease(lease) is not True:
                    return False
                active = self.ingress.adapter.executor.active
                if active is None:
                    return True
                request = self.ingress.requests.get(active.command_id)
                if request is None or request[0].lease_id != lease.lease_id:
                    return False
                return self.ingress.renew(lease)
            except Exception as exc:
                self.node.get_logger().error(f'CommandLease rejected: {exc}')
                return False

    def on_cancel(self, handle):
        with self.ingress.adapter.lock:
            active = self.ingress.adapter.executor.active
            if active is None or active.command_id != handle.request.command_id:
                return self.bindings.CancelResponse.REJECT
            self.ingress.adapter.request_stop('CANCEL_USER')
            return self.bindings.CancelResponse.ACCEPT

    async def execute(self, handle):
        command_id = handle.request.command_id
        with self.ingress.adapter.lock:
            report = self.ingress.report(command_id)
            if report is None:
                future = self.bindings.Future(executor=getattr(self.node, 'executor', None))
                self.waiting.setdefault(command_id, []).append((handle, future))
        if report is None:
            report = await future
        with self.ingress.adapter.lock:
            response = self.encode_result(report, handle.request)
            wire = response.result
            validate_completion_scope(handle.request.kind, wire)
            expected = (report.task_id, report.command_id, report.robot_id, report.generation,
                        report.result.outcome, report.result.reason, report.result.original_cause,
                        report.result.stop_confirmed)
            actual = (wire.task_id, wire.command_id, wire.robot_id, wire.generation,
                      wire.outcome, wire.terminal_reason, wire.original_cause, wire.stop_confirmed)
            if actual != expected or wire.motion_state != (
                    'STOPPED' if report.result.stop_confirmed else 'STOP_UNCONFIRMED'):
                raise ContractError('result encoder changed executor identity/stop outcome')
            self.stamp_envelope(response.envelope)
            if report.result.outcome == 'COMPLETED' and report.result.stop_confirmed:
                handle.succeed()
            elif (report.result.outcome == 'CANCELED' and report.result.stop_confirmed
                  and handle.is_cancel_requested):
                handle.canceled()
            else:
                handle.abort()
            return response

    def poll(self):
        """Report polling is independent from Nav2's adapter.step timer."""
        with self.ingress.adapter.lock:
            try:
                status = fill_robot_status(self.bindings.RobotStatus(), self.ingress.adapter,
                                           self.ingress, state_sequence=self.status_sequence)
                self.status_sequence += 1
                self.stamp_envelope(status.envelope)
                self.status_publisher.publish(status)
            except Exception as exc:
                self.node.get_logger().error(f'RobotStatus publication failed: {exc}')
            for command_id, waiters in list(self.waiting.items()):
                report = self.ingress.report(command_id)
                if report is not None:
                    for _, future in waiters:
                        if not future.done():
                            future.set_result(report)
                    del self.waiting[command_id]
                    continue
                executor = self.ingress.adapter.executor
                state = {'PLANNING': 'PLANNING', 'AWAITING_ACCEPTANCE': 'NAVIGATING',
                         'NAVIGATING': 'NAVIGATING', 'STOPPING': 'STOPPING'}.get(
                             executor.state, 'VALIDATING')
                for handle, _ in waiters:
                    feedback = self.bindings.ExecuteTask.Feedback()
                    feedback.task_id, feedback.generation = handle.request.task_id, handle.request.generation
                    feedback.state = state
                    feedback.segment_index = (handle.request.route.segment_index
                                              if getattr(handle.request, 'has_route', False) else 0)
                    feedback.distance_remaining_known = False
                    feedback.reason_code = ''
                    pose = feedback.current_pose
                    pose.robot_id, pose.source_boot_id = executor.robot_id, executor.robot_boot_id
                    pose.mission_pose_status = 'UNKNOWN'
                    pose.time.stamp_semantics, pose.time.clock_state = 'UNKNOWN', 'UNKNOWN'
                    self.stamp_envelope(pose.envelope)
                    self.stamp_envelope(feedback.envelope)
                    # The pose is unknown; no native/mission coordinate or
                    # clock-error bound is fabricated from the destination.
                    try:
                        handle.publish_feedback(feedback)
                    except Exception as exc:
                        self.node.get_logger().error(f'feedback publication failed: {exc}')

    def destroy(self):
        # Caller must keep the adapter/watchdog stop monitoring alive until
        # cancellation is fenced and fresh odometry proves the stop.
        with self.ingress.adapter.lock:
            if self.ingress.adapter.executor.active is not None:
                self.ingress.adapter.request_stop('CANCEL_USER')
            self.node.destroy_timer(self.timer)
            self.node.destroy_subscription(self.subscription)
            self.node.destroy_publisher(self.publisher)
            self.node.destroy_publisher(self.status_publisher)
            self.server.destroy()
