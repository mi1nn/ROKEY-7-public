"""SEARCH-only runtime with synthetic records and fake Nav2 (E2, no hardware)."""
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import test_mapping_guard as mapping
import test_task_server as transport
from test_robot_task_wire import wire
from robot_task_policy_fixtures import POLICY, clock_basis
from rescue_robot.executor import ContractError
from rescue_robot import task_input_node as search


class SearchRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(search, 'SearchInputStore'), 'built-in SEARCH input reader missing')
        self.f = mapping.MappingGuardTests(); self.f.setUp()
        self.adapter = self.f.adapter
        self.adapter.executor.latch = 'DISARMED'
        self.goal, self.lease = wire('SEARCH_AREA')
        g, l, now = self.goal, self.lease, self.adapter.wall_clock_ns()
        g.coordinator_id = l.coordinator_id = mapping.uid(500)
        g.issued_at_ns = l.issued_at_ns = now
        g.deadline_ns = now + 30_000_000_000
        l.valid_until_ns = now + 1_000_000_000
        g.frame_ref.map_session_id = mapping.SESSION
        g.frame_ref.native_frame = g.frame_ref.alignment_status = ''
        g.envelope = NS(schema_version='test-only', publisher_id='test-central', publisher_boot_id='test-boot')
        l.envelope = deepcopy(g.envelope)
        self.config = dict(format='rescue-robot-runtime/1', robot_id=g.robot_id, mission_frame='mission',
            schema_version='test-only', publisher_id='test-executor', clock_error_bound_ns=10_000_000,
            lease=asdict(POLICY), mapping_authority='test-mapping', max_position_error_m=.2,
            max_rotation_error_rad=.1, input_max_age_ns=60_000_000_000, poll_period_s=.05,
            input_file='/TEST_ONLY/inputs.json', endpoints=dict(action_name='/test/task', lease_topic='/test/lease',
            admission_topic='/test/admission', status_topic='/test/status'))
        self.data = dict(format='rescue-robot-inputs/1', generated_at_ns=now, valid_until_ns=now+60_000_000_000,
            clock=asdict(clock_basis(self.adapter)), authority=dict(coordinator_id=g.coordinator_id,
            authority_epoch=g.authority_epoch, robot_boot_id=g.expected_robot_boot_id,
            publisher_id='test-central', publisher_boot_id='test-boot'),
            maps={g.robot_id: dict(map_frame='robot_a/map', map_session_id=mapping.SESSION,
            map_revision=0, map_update_sequence=0, observed_at_ns=now, valid_until_ns=now+60_000_000_000,
            localization_status='VALID', transform_revision=1, authority_id='test-mapping',
            clock_domain_id='test-clock', valid_from_ns=now-1_000_000_000,
            transform_valid_until_ns=now+60_000_000_000, transform_xy_yaw=[2., 0., 0.],
            position_error_m=.01, rotation_error_rad=.01, evidence_id='TEST_ONLY_calibration')},
            destinations={g.search_area_ref: dict(robot_id=g.robot_id, map_session_id=mapping.SESSION,
            map_revision=0, transform_revision=1, xy_yaw=[4., 2., 0.], observed_at_ns=now,
            valid_until_ns=now+60_000_000_000)})
        self.node = transport.Node()
        t = transport.TaskServerTests(); t.setUp()
        bindings = NS(**vars(t.server.bindings))
        bindings.ExecuteTask = NS(Feedback=transport.Action.Feedback,
            Result=lambda: NS(envelope=NS(), result=NS(final_pose=NS(envelope=NS(), time=NS()))))
        self.inputs = search.SearchInputStore(self.config, wall_clock_ns=self.adapter.wall_clock_ns,
            read=lambda: deepcopy(self.data))
        self.runtime = search.SearchRuntime(self.node, self.adapter, self.config, self.inputs, bindings=bindings)

    def accept(self):
        self.assertTrue(self.runtime.arm())
        self.assertEqual(self.runtime.server.on_goal(self.goal), 'accept', self.node.errors)

    def start(self):
        self.accept(); self.adapter.step()
        self.adapter.on_goal_response(self.goal.command_id, True)

    def test_installation_stays_disarmed_and_needs_no_target_or_route_records(self):
        self.assertEqual(self.adapter.executor.latch, 'DISARMED')
        self.assertEqual(self.f.port.goals, [])
        self.accept()

    def test_goal_first_dispatches_transformed_endpoint_only_once(self):
        self.accept(); self.runtime.server.on_goal(deepcopy(self.goal))
        self.adapter.step(); self.adapter.step()
        (command, pose), = self.f.port.goals
        self.assertEqual((command, pose.frame_id, pose.x, pose.y),
            (self.goal.command_id, 'robot_a/map', 2., 2.))

    def test_renewal_extends_initial_lease_without_redispatch(self):
        self.start(); until = self.adapter.executor.lease_until
        for sequence in range(6):
            self.f.clock.now += .4; self.f.tick()
            self.lease.renewal_sequence = sequence
            self.lease.issued_at_ns = self.adapter.wall_clock_ns()
            self.lease.valid_until_ns = self.lease.issued_at_ns + 1_000_000_000
            self.assertTrue(self.runtime.server.on_lease(self.lease), self.node.warnings)
        self.assertGreater(self.adapter.executor.lease_until, until)
        self.assertEqual(len(self.f.port.goals), 1)
        self.assertEqual(self.adapter.executor.state, 'NAVIGATING')

    def test_completion_waits_for_fresh_stop_after_nav2_success(self):
        self.start(); self.f.tick(linear=.3)
        self.adapter.on_terminal(self.goal.command_id, 4)
        self.assertIsNone(self.runtime.server.ingress.report(self.goal.command_id))
        for _ in range(4): self.f.tick()
        report = self.runtime.server.ingress.report(self.goal.command_id)
        result = self.runtime.encode_result(report, self.goal).result
        self.assertEqual((result.outcome, result.completion_scope, result.stop_confirmed, result.motion_state),
            ('COMPLETED', 'SEARCH', True, 'STOPPED'))
        self.assertEqual(result.final_pose.mission_pose_status, 'UNKNOWN')

    def test_other_task_kinds_are_rejected_without_nav2_goal(self):
        self.assertTrue(self.runtime.arm())
        for kind in ('MOVE_SEGMENT', 'REOBSERVE_TARGET', 'APPROACH_TARGET', 'RETURN_HOME', 'HOLD'):
            with self.subTest(kind=kind):
                goal = deepcopy(self.goal); goal.kind = kind
                self.assertEqual(self.runtime.server.on_goal(goal), 'reject')
        self.assertEqual(self.f.port.goals, [])

    def test_unknown_destination_is_rejected(self):
        self.data['destinations'] = {}; self.assertTrue(self.runtime.arm())
        self.assertEqual(self.runtime.server.on_goal(self.goal), 'reject')
        self.assertEqual(self.f.port.goals, [])

    def test_destination_change_cancels_active_goal(self):
        self.start(); self.data['destinations'][self.goal.search_area_ref]['xy_yaw'][0] += 1.
        self.adapter.step(); self.assertEqual(self.f.port.cancels, [self.goal.command_id])

    def test_live_authority_revocation_cancels_active_goal(self):
        self.start(); self.data['authority']['authority_epoch'] = mapping.uid(999)
        self.adapter.step(); self.assertEqual(self.f.port.cancels, [self.goal.command_id])

    def test_expired_inputs_cancel_without_refreshing_observation(self):
        self.start(); original = self.data['generated_at_ns']
        self.data['valid_until_ns'] = self.adapter.wall_clock_ns()
        self.adapter.step(); self.assertEqual(self.f.port.cancels, [self.goal.command_id])
        self.assertEqual(self.data['generated_at_ns'], original)

    def test_missing_live_map_cancels_instead_of_breaking_step(self):
        self.start(); self.data['maps'] = {}
        self.adapter.step(); self.assertEqual(self.f.port.cancels, [self.goal.command_id])

    def test_localization_and_alignment_evidence_required_before_dispatch(self):
        self.assertTrue(self.runtime.arm())
        m = self.data['maps'][self.goal.robot_id]
        for field, value in (('localization_status', 'UNKNOWN'), ('evidence_id', ''),
                             ('position_error_m', .3), ('rotation_error_rad', .2)):
            with self.subTest(field=field):
                old = m[field]; m[field] = value
                self.assertEqual(self.runtime.server.on_goal(self.goal), 'reject')
                m[field] = old
        self.assertEqual(self.f.port.goals, [])

    def test_identity_alignment_uses_same_map_coordinates(self):
        self.data['maps'][self.goal.robot_id]['transform_xy_yaw'] = [0., 0., 0.]
        self.accept(); self.adapter.step()
        self.assertEqual((self.f.port.goals[0][1].x, self.f.port.goals[0][1].y), (4., 2.))

    def test_unregistered_lease_publisher_cannot_extend_deadline(self):
        self.start(); until = self.adapter.executor.lease_until
        self.lease.envelope.publisher_id = 'other'
        self.assertFalse(self.runtime.server.on_lease(self.lease))
        self.assertEqual(self.adapter.executor.lease_until, until)
        self.assertIn('PUBLISHER_ID_MISMATCH', '\n'.join(self.node.warnings))

    def test_map_revision_rollback_is_rejected(self):
        m = self.data['maps'][self.goal.robot_id]
        m['map_revision'] = 1; self.inputs.map_state(self.goal.robot_id)
        m['map_revision'] = 0
        with self.assertRaises(ContractError): self.inputs.map_state(self.goal.robot_id)

    def test_same_transform_revision_cannot_change_meaning(self):
        self.inputs.current_alignment(self.goal.robot_id, now_ns=self.adapter.wall_clock_ns())
        self.data['maps'][self.goal.robot_id]['transform_xy_yaw'][0] += 1.
        with self.assertRaises(ContractError):
            self.inputs.current_alignment(self.goal.robot_id, now_ns=self.adapter.wall_clock_ns())

    def test_uncertain_clock_blocks_arm(self):
        self.data['clock']['state'] = 'UNKNOWN'
        with self.assertRaises(ContractError): self.runtime.arm()

    def test_boot_mismatch_blocks_arm(self):
        self.data['authority']['robot_boot_id'] = mapping.uid(999)
        self.assertFalse(self.runtime.arm())


class JsonBoundaryTests(unittest.TestCase):
    def test_ambiguous_and_nonfinite_json_is_rejected(self):
        self.assertTrue(hasattr(search, 'load_json'), 'strict runtime JSON reader missing')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'input.json'
            for content in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{"x":1e999}', '<html>'):
                with self.subTest(content=content):
                    path.write_text(content)
                    with self.assertRaises(ContractError): search.load_json(path)


if __name__ == '__main__':
    unittest.main()
