"""ROS action callbacks tested with fake bindings; no ROS or robot required."""
from types import SimpleNamespace as NS
import unittest

import test_task_ingress as ingress_tests
from rescue_robot.task_server import TaskActionServer
from rescue_robot.executor import ContractError
from robot_task_policy_fixtures import POLICY, clock_basis


class Future:
    def __init__(self, **kwargs):
        self.value, self.finished = None, False

    def done(self):
        return self.finished

    def set_result(self, value):
        self.value, self.finished = value, True

    def __await__(self):
        while not self.finished:
            yield self
        return self.value


def envelope():
    return NS()


class Action:
    @staticmethod
    def Feedback():
        return NS(envelope=envelope(), current_pose=NS(envelope=envelope(), time=NS()))


class Node:
    def __init__(self):
        self.messages, self.errors = [], []
        self.warnings, self.debug_messages = [], []

    def create_publisher(self, *args):
        return NS(publish=self.messages.append)

    def create_subscription(self, *args, **kwargs):
        return NS()

    def create_timer(self, *args, **kwargs):
        return NS()

    def destroy_timer(self, timer):
        pass

    def get_logger(self):
        return NS(error=self.errors.append, warning=self.warnings.append,
                  debug=self.debug_messages.append, info=self.debug_messages.append)


class Handle:
    def __init__(self, request):
        self.request, self.is_cancel_requested = request, False
        self.state, self.feedback = None, []

    def succeed(self):
        self.state = 'succeeded'

    def abort(self):
        self.state = 'aborted'

    def canceled(self):
        self.state = 'canceled'

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)


class TaskServerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ingress_tests.TaskIngressTests()
        self.fixture.setUp()
        self.goal, self.lease = self.fixture.goal, self.fixture.lease
        self.node = Node()
        self.lease_allowed = True
        bindings = NS(ActionServer=lambda *args, **kwargs: NS(destroy=lambda: None),
                      GoalResponse=NS(ACCEPT='accept', REJECT='reject'),
                      CancelResponse=NS(ACCEPT='accept', REJECT='reject'),
                      Future=Future, ReentrantCallbackGroup=lambda: None,
                      ExecuteTask=Action, CommandLease=NS,
                      TaskAdmission=lambda: NS(envelope=envelope()), RobotStatus=lambda: NS(envelope=envelope()))
        self.server = TaskActionServer(
            self.node, self.fixture.ingress, action_name='/test/execute_task',
            lease_topic='/test/lease', admission_topic='/test/admission', status_topic='/test/status', poll_period_s=.05,
            schema_version='test-only', publisher_id='test-executor',
            lookup_lease=lambda goal: self.lease,
            accept_lease=lambda lease: self.lease_allowed,
            encode_result=self.encode,
            bindings=bindings)

    @staticmethod
    def encode(report, goal):
        result = NS(task_id=report.task_id, command_id=report.command_id, robot_id=report.robot_id,
                    generation=report.generation, outcome=report.result.outcome,
                    terminal_reason=report.result.reason, original_cause=report.result.original_cause,
                    stop_confirmed=report.result.stop_confirmed,
                    motion_state='STOPPED' if report.result.stop_confirmed else 'STOP_UNCONFIRMED',
                    completion_scope={'SEARCH_AREA':'SEARCH','MOVE_SEGMENT':'SEGMENT','RETURN_HOME':'RETURN',
                                      'HOLD':'HOLD','APPROACH_TARGET':'TARGET_APPROACH',
                                      'REOBSERVE_TARGET':'RECHECK'}[goal.kind])
        return NS(envelope=envelope(), result=result)

    def finish(self):
        adapter = self.fixture.adapter
        adapter.step()
        adapter.on_goal_response(self.goal.command_id, True)
        adapter.on_terminal(self.goal.command_id, 4)
        self.fixture.fixture.still(adapter)

    def test_accept_publishes_admission_without_dispatching_nav2(self):
        self.server.lookup_lease = lambda goal: None  # central sends goal first
        self.assertEqual(self.server.on_goal(self.goal), 'accept')
        self.assertTrue(self.node.messages[-1].accepted)
        self.assertEqual(self.fixture.fixture.nav_b.goals, [])
        self.assertTrue(self.server.on_lease(self.lease))

    def test_untrusted_goal_is_rejected_with_admission_reason(self):
        self.fixture.allowed = False
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertFalse(self.node.messages[-1].accepted)
        self.assertEqual(self.node.messages[-1].reason_code, 'NOT_AUTHORIZED')

    def test_retry_preserves_admission_id_and_observed_time(self):
        self.server.on_goal(self.goal)
        first = self.node.messages[-1]
        self.fixture.fixture.advance(.1)
        self.server.on_goal(self.goal)
        second = self.node.messages[-1]
        self.assertEqual((first.admission_id, first.observed_at_ns),
                         (second.admission_id, second.observed_at_ns))
        self.assertGreater(second.envelope.sequence, first.envelope.sequence)

    def test_execute_waits_for_stop_and_reports_unknown_feedback_pose(self):
        self.server.on_goal(self.goal)
        handle = Handle(self.goal)
        coroutine = self.server.execute(handle)
        future = coroutine.send(None)
        self.server.poll()
        self.assertFalse(future.done())
        self.assertEqual(handle.feedback[-1].current_pose.mission_pose_status, 'UNKNOWN')
        self.finish()
        self.server.poll()
        with self.assertRaises(StopIteration) as done:
            coroutine.send(None)
        self.assertTrue(done.exception.value.result.stop_confirmed)
        self.assertEqual(handle.state, 'succeeded')

    def test_cancel_foreign_handle_does_not_stop_current_task(self):
        self.server.on_goal(self.goal)
        from copy import deepcopy
        other = deepcopy(self.goal)
        other.command_id = ingress_tests.uid(99)
        self.assertEqual(self.server.on_cancel(Handle(other)), 'reject')
        self.assertEqual(self.fixture.adapter.executor.state, 'PLANNING')

    def test_cancel_is_not_reported_as_stopped_without_evidence(self):
        self.server.on_goal(self.goal)
        handle = Handle(self.goal)
        coroutine = self.server.execute(handle)
        future = coroutine.send(None)
        self.fixture.adapter.step()
        self.fixture.adapter.on_goal_response(self.goal.command_id, True)
        self.assertEqual(self.server.on_cancel(handle), 'accept')
        self.fixture.adapter.on_cancel_response(self.goal.command_id, True)
        self.server.poll()
        self.assertFalse(future.done())
        self.fixture.adapter.on_terminal(self.goal.command_id, 5)
        self.fixture.fixture.still(self.fixture.adapter)
        handle.is_cancel_requested = True
        self.server.poll()
        with self.assertRaises(StopIteration):
            coroutine.send(None)
        self.assertEqual(handle.state, 'canceled')

    def test_authentication_rejection_logs_without_extending_lease(self):
        self.server.on_goal(self.goal)
        self.lease.renewal_sequence += 1
        self.lease_allowed = False
        until = self.fixture.adapter.executor.lease_until
        self.assertFalse(self.server.on_lease(self.lease))
        self.assertIn('LEASE_AUTHENTICATION_OR_STORAGE_REJECTED', '\n'.join(self.node.warnings))
        self.assertEqual(self.fixture.adapter.executor.lease_until, until)

    def test_other_lease_logs_debug_without_extending_active_task(self):
        self.server.on_goal(self.goal)
        self.lease.lease_id = ingress_tests.uid(99)
        until = self.fixture.adapter.executor.lease_until
        self.assertFalse(self.server.on_lease(self.lease))
        self.assertIn('OTHER_LEASE_ID', '\n'.join(self.node.debug_messages))
        self.assertFalse(self.node.warnings)
        self.assertEqual(self.fixture.adapter.executor.lease_until, until)

    def test_pre_goal_lease_is_stored_with_debug_and_no_dispatch(self):
        self.assertTrue(self.server.on_lease(self.lease))
        self.assertIn('STORED_BEFORE_TASK', '\n'.join(self.node.debug_messages))
        self.assertIsNone(self.fixture.adapter.executor.active)
        self.assertFalse(self.fixture.fixture.nav_b.goals)

    def test_executor_rejection_logs_expiry_stopping_and_sequence(self):
        for reason in ('NONINCREASING_SEQUENCE', 'STOPPING', 'LOCAL_LEASE_EXPIRED'):
            with self.subTest(reason=reason):
                self.setUp()
                self.server.on_goal(self.goal)
                if reason == 'STOPPING':
                    self.lease.renewal_sequence += 1
                    self.fixture.adapter.request_stop('CANCEL_USER')
                elif reason == 'LOCAL_LEASE_EXPIRED':
                    self.lease.renewal_sequence += 1
                    self.fixture.fixture.advance(POLICY.duration_s)
                    self.lease.issued_at_ns = self.fixture.adapter.wall_clock_ns()
                    self.lease.valid_until_ns = self.lease.issued_at_ns + 2_000_000_000
                until = self.fixture.adapter.executor.lease_until
                self.assertFalse(self.server.on_lease(self.lease))
                self.assertIn(reason, '\n'.join(self.node.warnings))
                self.assertEqual(self.fixture.adapter.executor.lease_until, until)
                self.assertFalse(self.fixture.fixture.nav_b.goals)

    def test_stop_unconfirmed_result_aborts_action(self):
        self.server.on_goal(self.goal)
        handle = Handle(self.goal)
        coroutine = self.server.execute(handle)
        coroutine.send(None)
        self.fixture.adapter.step()
        self.fixture.adapter.on_goal_response(self.goal.command_id, True)
        self.server.on_cancel(handle)
        self.fixture.adapter.on_terminal(self.goal.command_id, 5)
        self.fixture.fixture.advance(3.5)
        self.fixture.fixture.ready(self.fixture.adapter)
        self.fixture.adapter.step()
        self.server.poll()
        with self.assertRaises(StopIteration) as done:
            coroutine.send(None)
        self.assertFalse(done.exception.value.result.stop_confirmed)
        self.assertEqual(handle.state, 'aborted')

    def test_admission_publish_failure_stops_before_any_dispatch(self):
        def fail(msg):
            raise RuntimeError('publisher unavailable')
        self.server.publisher.publish = fail
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertEqual(self.fixture.adapter.executor.state, 'STOPPING')
        self.fixture.adapter.step()
        self.assertEqual(self.fixture.fixture.nav_b.goals, [])

    def test_encoder_cannot_turn_failure_into_success(self):
        self.server.on_goal(self.goal)
        handle = Handle(self.goal)
        coroutine = self.server.execute(handle)
        coroutine.send(None)
        self.finish()
        self.server.poll()
        def invalid(report, goal):
            response = self.encode(report, goal)
            response.result.stop_confirmed = False
            return response
        self.server.encode_result = invalid
        with self.assertRaises(ContractError):
            coroutine.send(None)
        self.assertIsNone(handle.state)

    def test_runtime_install_does_not_arm_and_missing_alignment_blocks_dispatch(self):
        from rescue_robot.task_server import attach_task_runtime
        adapter, nav = self.fixture.fixture.make(self.goal.robot_id, boot=11)
        def missing(**kwargs):
            raise ContractError('no measured alignment', 'FRAME_REVISION_MISMATCH')
        registry = NS(mission_frame='map',
                      map_state=lambda robot: NS(map_session_id=self.fixture.fixture.pose.map_session_id,
                                                map_revision=0),
                      current_alignment=lambda robot, **kwargs: missing(**kwargs))
        server = attach_task_runtime(
            self.node, adapter, registry=registry, target_gate=lambda task: False,
            authorize=lambda goal, lease: True, resolve_goal=lambda goal: self.fixture.fixture.pose,
            clock_error_bound_ns=10_000_000,
            lease_policy=POLICY, clock_basis=lambda: clock_basis(adapter),
            action_name='/test2/action', lease_topic='/test2/lease', admission_topic='/test2/admission', status_topic='/test2/status',
            poll_period_s=.05, schema_version='test-only', publisher_id='test-executor',
            lookup_lease=lambda goal: self.lease, accept_lease=lambda lease: True,
            encode_result=self.encode, bindings=self.server.bindings)
        self.assertEqual(adapter.executor.latch, 'DISARMED')
        self.fixture.fixture.arm(adapter, 11)
        self.assertEqual(server.on_goal(self.goal), 'accept')
        adapter.step()
        self.assertEqual(nav.goals, [])
        self.assertEqual(adapter.executor.state, 'STOPPING')

    def test_installing_runtime_on_armed_executor_is_rejected(self):
        from rescue_robot.task_server import attach_task_runtime
        with self.assertRaises(ContractError):
            attach_task_runtime(self.node, self.fixture.adapter, registry=None,
                                target_gate=lambda task: True, authorize=lambda goal, lease: True,
                                resolve_goal=lambda goal: None, clock_error_bound_ns=1,
                                lease_policy=POLICY, clock_basis=lambda: clock_basis(self.fixture.adapter))
