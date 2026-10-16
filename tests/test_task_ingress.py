"""ExecuteTask/CommandLease boundary with fake ROS-shaped messages; no hardware."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

import test_nav2_adapter as fixtures
from rescue_robot.executor import ContractError
from rescue_robot.task_ingress import TaskIngress
from robot_task_policy_fixtures import POLICY, clock_basis

uid = fixtures.uid


class TaskIngressTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.AdapterTests()
        fixture.setUp()
        self.fixture = fixture
        self.adapter = fixture.b
        fixture.arm(self.adapter, 11)
        self.allowed = True
        self.ingress = TaskIngress(self.adapter, authorize=lambda goal, lease: self.allowed,
                                  resolve_goal=lambda goal: fixture.pose,
                                  clock_error_bound_ns=10_000_000, lease_policy=POLICY,
                                  clock_basis=lambda: clock_basis(self.adapter))
        wall = fixture.clock.wall()
        self.goal = NS(command_id=uid(30), task_id=uid(31), robot_id=self.adapter.robot_id,
                       expected_robot_boot_id=uid(11), coordinator_id=uid(32),
                       authority_epoch=uid(12), generation=1, kind='SEARCH_AREA',
                       issued_at_ns=wall, deadline_ns=wall + 60_000_000_000,
                       lease_id=uid(33), target_id='', target_estimate_revision=0,
                       search_area_ref=uid(34), home_ref='', policy_version='test-only',
                       has_route=False, has_approval=False, spin_yaw_rad=0.0, time_allowance_s=0.0,
                       frame_ref=NS(robot_id=self.adapter.robot_id, native_frame='robot_b/map', mission_frame='map',
                                    map_session_id=fixture.pose.map_session_id,
                                    map_revision=0, transform_revision=0, alignment_status='VALID'))
        self.lease = NS(lease_id=uid(33), robot_id=self.adapter.robot_id, robot_boot_id=uid(11),
                        coordinator_id=uid(32), authority_epoch=uid(12), generation=1,
                        renewal_sequence=0, issued_at_ns=wall, valid_until_ns=wall + 2_000_000_000)

    def test_wire_fields_submit_and_dispatch(self):
        receipt = self.ingress.submit(self.goal, self.lease)
        self.assertTrue(receipt.accepted)
        self.adapter.step()
        self.assertEqual(len(self.fixture.nav_b.goals), 1)
        self.assertAlmostEqual(self.adapter.executor.active.lease_duration, POLICY.duration_s)

    def test_retry_does_not_extend_lease_or_dispatch_twice(self):
        first = self.ingress.submit(self.goal, self.lease)
        until = self.adapter.executor.lease_until
        self.fixture.advance(.1)
        self.assertEqual(self.ingress.submit(deepcopy(self.goal), deepcopy(self.lease)), first)
        self.assertEqual(self.adapter.executor.lease_until, until)
        self.adapter.step()
        self.adapter.step()
        self.assertEqual(len(self.fixture.nav_b.goals), 1)

    def test_changed_content_on_retry_is_rejected(self):
        self.ingress.submit(self.goal, self.lease)
        changed = deepcopy(self.goal)
        changed.search_area_ref = uid(40)
        with self.assertRaises(ContractError) as context:
            self.ingress.submit(changed, self.lease)
        self.assertEqual(context.exception.code, 'ID_CONTENT_CONFLICT')

    def test_retry_with_foreign_lease_is_not_an_admission(self):
        self.ingress.submit(self.goal, self.lease)
        lease = deepcopy(self.lease)
        lease.lease_id = uid(40)
        with self.assertRaises(ContractError):
            self.ingress.submit(self.goal, lease)

    def test_lease_identity_and_time_fail_before_admission(self):
        for name, value in (('robot_boot_id', uid(40)), ('coordinator_id', uid(40)),
                            ('lease_id', uid(40)), ('generation', 2),
                            ('valid_until_ns', self.fixture.clock.wall()),
                            ('issued_at_ns', self.fixture.clock.wall() + 10_000_001)):
            lease = deepcopy(self.lease)
            setattr(lease, name, value)
            with self.assertRaises(ContractError, msg=name):
                self.ingress.submit(self.goal, lease)
            self.assertIsNone(self.adapter.executor.active)

    def test_authorization_and_frame_revision_are_required(self):
        self.allowed = False
        with self.assertRaises(ContractError):
            self.ingress.submit(self.goal, self.lease)
        self.allowed = True
        self.goal.frame_ref.map_revision = 1
        with self.assertRaises(ContractError):
            self.ingress.submit(self.goal, self.lease)
        self.assertEqual(self.fixture.nav_b.goals, [])

    def test_slow_goal_resolution_does_not_extend_an_expired_lease(self):
        def resolve(goal):
            self.fixture.advance(2)
            return self.fixture.pose
        self.ingress.resolve_goal = resolve
        with self.assertRaises(ContractError):
            self.ingress.submit(self.goal, self.lease)
        self.assertIsNone(self.adapter.executor.active)

    def test_renewal_reception_expiry_does_not_shorten_policy_steady_lifetime(self):
        self.ingress.submit(self.goal, self.lease)
        self.fixture.advance(.5)
        lease = deepcopy(self.lease)
        lease.renewal_sequence = 1
        lease.issued_at_ns = self.fixture.clock.wall()
        lease.valid_until_ns = lease.issued_at_ns + 300_000_000
        self.assertTrue(self.ingress.renew(lease))
        self.assertAlmostEqual(self.adapter.executor.lease_until, self.fixture.clock() + POLICY.duration_s)
        self.assertFalse(self.ingress.renew(lease))

    def test_revocation_during_resolution_is_rejected_before_admission(self):
        def resolve(goal):
            self.allowed = False
            return self.fixture.pose
        self.ingress.resolve_goal = resolve
        with self.assertRaises(ContractError): self.ingress.submit(self.goal, self.lease)
        self.assertIsNone(self.adapter.executor.active)

    def test_coordinator_string_identifier_is_not_forced_to_uuid(self):
        self.goal.coordinator_id = self.lease.coordinator_id = 'authenticated-coordinator'
        self.assertTrue(self.ingress.submit(self.goal, self.lease).accepted)

    def test_clock_regression_blocks_renewal_and_latches_runtime_guard(self):
        self.ingress.submit(self.goal, self.lease)
        self.fixture.advance(.5)
        lease = deepcopy(self.lease); lease.renewal_sequence = 1
        self.assertTrue(self.ingress.renew(lease))
        self.fixture.advance(-.1)
        lease.renewal_sequence = 2
        with self.assertRaises(ContractError): self.ingress.renew(lease)
        self.assertFalse(self.ingress.check(self.adapter.executor.active))

    def test_late_renewal_cannot_resurrect_expired_lease(self):
        self.ingress.submit(self.goal, self.lease)
        self.fixture.advance(2)
        lease = deepcopy(self.lease)
        lease.renewal_sequence = 1
        lease.issued_at_ns = self.fixture.clock.wall()
        lease.valid_until_ns = lease.issued_at_ns + 2_000_000_000
        self.assertFalse(self.ingress.renew(lease))

    def test_hold_does_not_resolve_or_send_a_pose(self):
        self.goal.kind, self.goal.search_area_ref = 'HOLD', ''
        self.ingress.resolve_goal = lambda goal: self.fail('HOLD must not resolve a position')
        self.assertTrue(self.ingress.submit(self.goal, self.lease).accepted)
        self.adapter.step()
        self.fixture.still(self.adapter)
        self.assertEqual(self.fixture.nav_b.goals, [])
        self.assertTrue(self.adapter.result(self.goal.command_id).stop_confirmed)

    def test_malformed_segment_and_return_are_rejected(self):
        for kind in ('MOVE_SEGMENT', 'RETURN_HOME'):
            self.goal.kind = kind
            with self.assertRaises(ContractError):
                self.ingress.submit(self.goal, self.lease)
        self.assertIsNone(self.adapter.executor.active)

    def test_terminal_report_waits_for_continuous_stop_evidence(self):
        self.ingress.submit(self.goal, self.lease)
        self.adapter.step()
        self.adapter.on_goal_response(self.goal.command_id, True)
        self.adapter.on_terminal(self.goal.command_id, 4)
        self.assertIsNone(self.ingress.report(self.goal.command_id))
        self.fixture.still(self.adapter)
        report = self.ingress.report(self.goal.command_id)
        self.assertEqual((report.task_id, report.robot_id, report.generation),
                         (self.goal.task_id, self.goal.robot_id, self.goal.generation))
        self.assertTrue(report.result.stop_confirmed)
        self.assertEqual(report.result.outcome, 'COMPLETED')

    def test_stop_without_odometry_is_reported_as_unconfirmed_failure(self):
        self.ingress.submit(self.goal, self.lease)
        self.adapter.step()
        self.adapter.on_goal_response(self.goal.command_id, True)
        self.adapter.request_stop('CANCEL_USER')
        self.adapter.on_terminal(self.goal.command_id, 5)
        self.fixture.advance(3.5)
        self.fixture.ready(self.adapter)
        self.adapter.step()
        report = self.ingress.report(self.goal.command_id)
        self.assertEqual(report.result.outcome, 'FAILED')
        self.assertFalse(report.result.stop_confirmed)
