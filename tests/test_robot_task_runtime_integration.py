"""Full task boundary + mapping + approval guard using fake Nav2 only."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

import test_mapping_guard as mapping
import test_task_server as transport
from test_robot_task_wire import wire
from rescue_robot.executor import ContractError, GoalPose
from rescue_robot.segment_approval import SegmentApprovalGate, VerifiedSegmentRoute
from rescue_robot.task_server import attach_task_runtime
from robot_task_policy_fixtures import POLICY, clock_basis


class RuntimeIntegrationTests(unittest.TestCase):
    def setUp(self):
        f = mapping.MappingGuardTests(); f.setUp()
        self.f, self.adapter = f, f.adapter
        # Installation is before arm, as on the actual entrypoint.
        self.adapter.executor.latch = 'DISARMED'
        self.goal, self.lease = wire()
        g, l = self.goal, self.lease
        g.expected_robot_boot_id, g.authority_epoch = mapping.uid(1), mapping.uid(2)
        l.robot_boot_id, l.authority_epoch = g.expected_robot_boot_id, g.authority_epoch
        g.coordinator_id = l.coordinator_id = mapping.uid(500)
        self.now = f.clock.wall()
        g.issued_at_ns = l.issued_at_ns = self.now
        g.deadline_ns = self.now + 10_000_000_000
        l.valid_until_ns = self.now + 5_000_000_000
        for frame in (g.frame_ref, g.route.frame_ref):
            frame.map_session_id = mapping.SESSION
        g.route.generated_at_ns = g.approval.approved_at_ns = self.now
        g.route.valid_until_ns = g.approval.valid_until_ns = self.now + 5_000_000_000
        g.approval.command_id = g.command_id  # existing exact-command gate binding
        g.approval.robot_boot_id, g.approval.authority_epoch = l.robot_boot_id, l.authority_epoch
        g.route.path_sha256, g.route.approval_corridor_sha256 = 'a' * 64, 'b' * 64
        g.route.planner_evidence_id = 'verified-test-record'
        self.pose = GoalPose('mission', 3., 0., 0., mapping.SESSION, 0, 1)
        self.proof = VerifiedSegmentRoute(g.route.route_id, g.route.route_revision,
            g.route.segment_index, g.robot_id, g.target_id, g.target_estimate_revision,
            g.frame_ref.native_frame, self.pose, g.route.path_sha256,
            g.route.approval_corridor_sha256, self.now, g.route.valid_until_ns,
            g.policy_version, g.route.planner_evidence_id)
        self.operator_ok, self.target_revision = True, g.target_estimate_revision
        self.segment_gate = SegmentApprovalGate(self.adapter,
            authorize_operator=lambda approval, goal: self.operator_ok,
            verify_route=lambda goal: self.proof,
            current_target_revision=lambda target: self.target_revision,
            clock_error_bound_ns=10_000_000)
        t = transport.TaskServerTests(); t.setUp()
        self.node = transport.Node()
        self.server = attach_task_runtime(self.node, self.adapter, registry=f.registry,
            target_gate=lambda task: True, authorize=lambda goal, lease: True,
            resolve_goal=lambda goal: self.pose, segment_gate=self.segment_gate,
            clock_error_bound_ns=10_000_000, action_name='/local/task', lease_topic='/local/lease',
            admission_topic='/local/admission', status_topic='/local/status', poll_period_s=.05, schema_version='test-only',
            lease_policy=POLICY, clock_basis=lambda: clock_basis(self.adapter),
            publisher_id='test-executor', lookup_lease=lambda goal: self.lease,
            accept_lease=lambda lease: True, encode_result=t.encode, bindings=t.server.bindings)
        self.assertTrue(self.adapter.executor.arm(mapping.uid(2), now=f.clock(),
            readiness=self.adapter.readiness(f.clock())))

    def accept(self):
        self.assertEqual(self.server.on_goal(self.goal), 'accept', self.node.errors)

    def test_segment_reaches_transformed_selected_endpoint_only_once(self):
        self.accept(); self.accept(); self.adapter.step()
        self.assertEqual(len(self.f.port.goals), 1)
        sent = self.f.port.goals[0][1]
        self.assertEqual((sent.frame_id, sent.x), ('robot_a/map', 1.))
        self.assertEqual(self.adapter.executor.active.kind, 'MOVE_SEGMENT')

    def test_segment_without_authenticated_gate_is_rejected(self):
        self.server.ingress.segment_gate = None
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertIsNone(self.adapter.executor.active)

    def test_unauthorized_operator_or_changed_backend_route_rejected_before_admission(self):
        self.operator_ok = False
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.operator_ok = True
        self.goal.route.path_sha256 = 'c' * 64
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertEqual(self.f.port.goals, [])

    def test_proof_for_another_adapter_is_rejected(self):
        self.segment_gate.adapter = NS()
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertIsNone(self.adapter.executor.active)

    def test_live_revocation_cancels_then_waits_for_stop(self):
        self.accept(); self.adapter.step()
        self.adapter.on_goal_response(self.goal.command_id, True)
        self.operator_ok = False; self.f.tick(linear=.3); self.adapter.step()
        self.assertEqual(self.f.port.cancels, [self.goal.command_id])
        self.assertIsNone(self.server.ingress.report(self.goal.command_id))
        self.adapter.on_terminal(self.goal.command_id, 5)
        for _ in range(4): self.f.tick()
        self.assertTrue(self.server.ingress.report(self.goal.command_id).result.stop_confirmed)

    def test_target_revision_change_cancels(self):
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(self.goal.command_id, True)
        self.target_revision += 1; self.adapter.step()
        self.assertEqual(self.f.port.cancels, [self.goal.command_id])

    def test_approval_expiry_cancels_even_with_long_renewed_lease(self):
        self.goal.approval.valid_until_ns = self.now + 400_000_000
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(self.goal.command_id, True)
        for _ in range(4): self.f.tick()
        self.adapter.step()
        self.assertEqual(self.f.port.cancels, [self.goal.command_id])

    def test_return_resolves_home_record_without_target_or_approval(self):
        g = self.goal
        g.kind, g.target_id, g.target_estimate_revision = 'RETURN_HOME', '', 0
        g.home_ref, g.has_route, g.has_approval = mapping.uid(999), False, False
        self.accept(); self.adapter.step()
        self.assertEqual(len(self.f.port.goals), 1)
        self.assertIsNone(self.adapter.executor.active.target_id)

    def test_return_cannot_infer_missing_home_pose(self):
        self.goal.kind, self.goal.target_id = 'RETURN_HOME', ''
        self.goal.home_ref = mapping.uid(999)
        self.goal.has_route = self.goal.has_approval = False
        self.server.ingress.resolve_goal = lambda goal: None
        self.assertEqual(self.server.on_goal(self.goal), 'reject')
        self.assertIsNone(self.adapter.executor.active)

    def test_changed_home_record_during_movement_cancels(self):
        g = self.goal
        g.kind, g.target_id, g.target_estimate_revision = 'RETURN_HOME', '', 0
        g.home_ref, g.has_route, g.has_approval = mapping.uid(999), False, False
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(g.command_id, True)
        self.pose = GoalPose('mission', 4., 0., 0., mapping.SESSION, 0, 1)
        self.adapter.step()
        self.assertEqual(self.f.port.cancels, [g.command_id])

    def test_home_lookup_failure_blocks_dispatch_without_inventing_pose(self):
        g = self.goal
        g.kind, g.target_id, g.target_estimate_revision = 'RETURN_HOME', '', 0
        g.home_ref, g.has_route, g.has_approval = mapping.uid(999), False, False
        self.accept()
        def unavailable(goal): raise RuntimeError('home cache unavailable')
        self.server.ingress.resolve_goal = unavailable
        self.adapter.step()
        self.assertEqual(self.f.port.goals, [])
        self.assertTrue(self.node.errors)

    def test_route_cache_failure_during_movement_cancels_and_logs(self):
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(self.goal.command_id, True)
        def unavailable(goal): raise RuntimeError('route cache unavailable')
        self.segment_gate.verify_route = unavailable
        self.adapter.step()
        self.assertEqual(self.f.port.cancels, [self.goal.command_id])
        self.assertTrue(self.node.errors)

    def test_segment_result_is_not_target_approach_completion(self):
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(self.goal.command_id, True)
        self.adapter.on_terminal(self.goal.command_id, 4)
        self.assertIsNone(self.server.ingress.report(self.goal.command_id))
        for _ in range(4): self.f.tick()
        handle = transport.Handle(self.goal)
        coroutine = self.server.execute(handle)
        with self.assertRaises(StopIteration) as done: coroutine.send(None)
        self.assertEqual(done.exception.value.result.completion_scope, 'SEGMENT')
        self.assertEqual(handle.state, 'succeeded')

    def test_bad_completion_scope_is_rejected_before_success(self):
        self.accept(); self.adapter.step(); self.adapter.on_goal_response(self.goal.command_id, True)
        self.adapter.on_terminal(self.goal.command_id, 4)
        for _ in range(4): self.f.tick()
        original = self.server.encode_result
        def bad(report, goal):
            response = original(report, goal); response.result.completion_scope = 'TARGET_APPROACH'
            return response
        self.server.encode_result = bad
        handle = transport.Handle(self.goal)
        with self.assertRaises(ContractError): self.server.execute(handle).send(None)
        self.assertIsNone(handle.state)

    def test_changed_approval_is_not_rebound_on_retry(self):
        self.accept(); changed = deepcopy(self.goal); changed.approval.operator_id = 'other'
        self.assertEqual(self.server.on_goal(changed), 'reject')


if __name__ == '__main__': unittest.main()
