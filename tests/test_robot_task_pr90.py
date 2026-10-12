"""Regression checks for #88/#90 wire semantics and proposed #89 lease rules.

All clocks, route protection and publisher authentication are synthetic E2 data.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import unittest

import test_task_ingress as ingress_tests
import test_task_server as transport
import test_robot_task_runtime_integration as runtime
from rescue_robot.executor import ContractError
from rescue_robot.task_policy import validate_lease_reception
from rescue_robot.phase1_route import Phase1RouteGate
from robot_task_policy_fixtures import POLICY, clock_basis

BOUND = 10_000_000


class LeaseReceptionTests(unittest.TestCase):
    def setUp(self):
        f = ingress_tests.TaskIngressTests(); f.setUp()
        self.f, self.ingress, self.goal, self.lease = f, f.ingress, f.goal, f.lease
        self.now = f.fixture.clock.wall()
        self.basis = clock_basis(f.adapter)

    def check(self, lease=None, basis=None, now=None):
        return validate_lease_reception(lease or self.lease, policy=POLICY,
            basis=basis or self.basis, now_ns=self.now if now is None else now)

    def test_fresh_lease_allows_bounded_future_timestamp(self):
        self.lease.issued_at_ns = self.now + BOUND
        self.check()
        self.lease.issued_at_ns += 1
        with self.assertRaises(ContractError): self.check()

    def test_expired_old_and_uncertain_lease_never_extend_local_deadline(self):
        self.assertTrue(self.ingress.submit(self.goal, self.lease).accepted)
        until = self.f.adapter.executor.lease_until
        for changes in (dict(valid_until_ns=self.now),
                        dict(issued_at_ns=self.now - POLICY.max_age_ns),
                        dict(valid_until_ns=self.now + BOUND)):
            lease = deepcopy(self.lease)
            lease.renewal_sequence = 99
            for name, value in changes.items(): setattr(lease, name, value)
            with self.assertRaises(ContractError): self.ingress.renew(lease)
            self.assertEqual(self.f.adapter.executor.lease_until, until)

    def test_unknown_domain_unsynced_missing_bound_and_over_policy_bound_rejected(self):
        for changes in (dict(domain_id='other'), dict(state='UNKNOWN'),
                        dict(error_bound_ns=None), dict(error_bound_ns=POLICY.max_error_ns + 1),
                        dict(valid_until_ns=self.now)):
            with self.subTest(changes=changes), self.assertRaises(ContractError):
                self.check(basis=replace(self.basis, **changes))

    def test_clock_session_change_and_forward_step_latch_runtime(self):
        self.ingress.submit(self.goal, self.lease)
        self.ingress.clock_basis = lambda: replace(clock_basis(self.f.adapter), epoch_id='restarted')
        self.assertFalse(self.ingress.check(self.f.adapter.executor.active))
        self.ingress.clock_basis = lambda: clock_basis(self.f.adapter)
        self.assertFalse(self.ingress.check(self.f.adapter.executor.active))

    def test_forward_wall_step_without_steady_progress_blocks_renew(self):
        self.ingress.submit(self.goal, self.lease)
        self.f.adapter.wall_clock_ns = lambda: self.now + 1_000_000_000
        self.lease.renewal_sequence = 1
        with self.assertRaises(ContractError): self.ingress.renew(self.lease)

    def test_goal_admitted_without_first_lease_and_initial_sequence_not_consumed(self):
        self.assertTrue(self.ingress.submit(self.goal, None).accepted)
        self.assertEqual(self.f.adapter.executor.lease_sequence, -1)
        self.lease.renewal_sequence = 1
        self.assertTrue(self.ingress.renew(self.lease))

    def test_goal_before_lease_still_requires_authorization_and_clock_proof(self):
        self.f.allowed = False
        with self.assertRaises(ContractError): self.ingress.submit(self.goal, None)
        self.f.allowed = True
        self.ingress.clock_basis = lambda: None
        with self.assertRaises(ContractError): self.ingress.submit(self.goal, None)


class Phase1RouteTests(unittest.TestCase):
    def setUp(self):
        r = runtime.RuntimeIntegrationTests(); r.setUp()
        self.r, self.g, self.l = r, r.goal, r.lease
        self.passage_ok = True
        g = self.g
        g.route.valid_until_ns = g.approval.valid_until_ns = g.deadline_ns
        g.route.path_sha256 = g.route.approval_corridor_sha256 = g.route.planner_evidence_id = ''
        g.route.planned_path = NS(poses=[])
        g.route.assessment = NS(status='UNKNOWN', width_known=False, height_known=False,
                                floor_geometry_known=False)
        g.route.approach_goal = NS(header=NS(frame_id='mission'), pose=NS(
            position=NS(x=3., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        g.approval.command_id = runtime.mapping.uid(777)  # operator ID != ExecuteTask command
        self.gate = Phase1RouteGate(r.adapter, registry=r.f.registry,
            authorize_operator=lambda approval, goal: r.operator_ok,
            current_target_revision=lambda target: r.target_revision,
            passage_guard=lambda goal: self.passage_ok, clock_error_bound_ns=BOUND)
        r.server.ingress.segment_gate = self.gate
        r.server.ingress.phase1_gate = self.gate

    def test_approved_endpoint_without_planner_values_is_admitted_and_transformed(self):
        self.g.route.route_id = self.g.approval.route_id = 'a' * 64  # #90 fixed route identity
        self.r.accept(); self.r.adapter.step()
        self.assertEqual(len(self.r.f.port.goals), 1)
        self.assertEqual(self.r.f.port.goals[0][1].x, 1.)
        self.assertNotEqual(self.g.approval.command_id, self.g.command_id)

    def test_target_source_basis_is_validated_then_bound_to_moving_robot_basis(self):
        from rescue_mapping import FrameAlignment, RigidTransform
        session = runtime.mapping.uid(810)
        self.r.f.registry.start_session('robot_b', 'robot_b/map', session)
        self.r.f.registry.set_alignment(FrameAlignment('robot_b', 'robot_b/map', 'mission', session,
            0, 4, 'test-mapping', 'test-clock', RigidTransform.planar(8., 0., 0.),
            runtime.mapping.WALL0, runtime.mapping.WALL0 + 10**12, .01, .01))
        for frame in (self.g.frame_ref, self.g.route.frame_ref):
            frame.robot_id, frame.native_frame, frame.map_session_id = 'robot_b', 'robot_b/map', session
            frame.transform_revision = 4
        self.r.accept(); self.r.adapter.step()
        task = self.r.adapter.executor.active
        self.assertEqual((task.goal_pose.map_session_id, task.goal_pose.transform_revision),
                         (runtime.mapping.SESSION, 1))
        self.assertEqual(self.r.f.port.goals[0][1].x, 1.)  # own map transform, not source robot's
        self.r.adapter.on_goal_response(self.g.command_id, True)
        self.r.f.registry.start_session('robot_b', 'robot_b/map', runtime.mapping.uid(811))
        self.r.adapter.step()
        self.assertEqual(self.r.f.port.cancels, [self.g.command_id])

    def test_phase1_reobserve_without_guard_cannot_use_generic_resolver(self):
        self.g.kind, self.g.has_approval = 'REOBSERVE_TARGET', False
        self.r.server.ingress.phase1_gate = None
        self.assertEqual(self.r.server.on_goal(self.g), 'reject')

    def test_unknown_passage_and_replan_entry_block_or_cancel(self):
        for guard in (False, None):
            self.passage_ok = guard
            self.assertEqual(self.r.server.on_goal(self.g), 'reject')
            self.assertEqual(self.r.f.port.goals, [])
        self.passage_ok = True
        self.r.accept(); self.r.adapter.step()
        self.r.adapter.on_goal_response(self.g.command_id, True)
        self.passage_ok = False
        self.r.adapter.step()
        self.assertEqual(self.r.f.port.cancels, [self.g.command_id])
        self.assertIsNone(self.r.server.ingress.report(self.g.command_id))

    def test_missing_approval_changed_revision_expiry_and_invalid_pose_rejected(self):
        for mutation in ('approval', 'revision', 'expiry', 'quaternion', 'frame', 'generated'):
            goal = deepcopy(self.g)
            if mutation == 'approval': goal.has_approval = False
            elif mutation == 'revision': goal.approval.route_revision += 1
            elif mutation == 'expiry': goal.approval.valid_until_ns = self.r.now
            elif mutation == 'quaternion': goal.route.approach_goal.pose.orientation.w = 0.
            elif mutation == 'frame': goal.route.approach_goal.header.frame_id = 'other'
            else: goal.route.generated_at_ns = 0
            with self.subTest(mutation=mutation), self.assertRaises(ContractError): self.gate.validate(goal)

    def test_phase1_approval_and_route_expiry_must_bind_to_task_deadline(self):
        self.g.approval.valid_until_ns -= 1
        with self.assertRaises(ContractError): self.gate.validate(self.g)

    def test_reobserve_uses_fixed_endpoint_without_operator_approval(self):
        self.g.kind, self.g.has_approval = 'REOBSERVE_TARGET', False
        self.r.operator_ok = False  # not an approval-based task
        self.r.accept(); self.r.adapter.step()
        self.assertEqual(len(self.r.f.port.goals), 1)
        self.assertEqual(self.r.adapter.executor.active.kind, 'REOBSERVE_TARGET')

    def test_revocation_and_target_change_are_live_guards(self):
        self.r.accept(); self.r.adapter.step()
        self.r.adapter.on_goal_response(self.g.command_id, True)
        self.r.operator_ok = False
        self.r.adapter.step()
        self.assertEqual(self.r.f.port.cancels, [self.g.command_id])


class TransportContractTests(unittest.TestCase):
    def test_real_robot_status_uses_stop_observation_not_cancel_ack(self):
        f = transport.TaskServerTests(); f.setUp()
        f.server.on_goal(f.goal)
        f.fixture.adapter.step()
        f.fixture.adapter.on_goal_response(f.goal.command_id, True)
        f.fixture.adapter.request_stop('CANCEL_USER')
        f.server.poll()
        status = f.node.messages[-1]
        self.assertEqual(status.motion_state, 'STOPPING')
        self.assertEqual(status.active_task_id, f.goal.task_id)
        self.assertFalse(status.lease_valid)
        self.assertEqual(status.robot_boot_id, f.goal.expected_robot_boot_id)
        self.assertEqual(status.stop_evidence, [])
        self.assertFalse(status.battery_known)

    def test_idle_status_does_not_invent_command_lease(self):
        f = transport.TaskServerTests(); f.setUp()
        f.server.poll()
        status = f.node.messages[-1]
        self.assertEqual(status.readiness, 'READY')
        self.assertEqual(status.active_task_id, '')
        self.assertFalse(status.lease_valid)  # #90 ready() needs an owner-side bootstrap correction


if __name__ == '__main__': unittest.main()
