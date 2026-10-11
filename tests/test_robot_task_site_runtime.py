"""Test the installed runtime with synthetic authoritative records, no ROS."""
from copy import deepcopy
from dataclasses import asdict
from types import SimpleNamespace as NS
import tempfile
from pathlib import Path
import unittest

import test_robot_task_pr90 as phase
import test_task_server as transport
from rescue_robot.task_site_runtime import SiteRuntime, InputStore, atomic_json


def records(f):
    g, a, now = f.g, f.g.approval, f.r.now
    g.envelope=NS(schema_version='test-only',publisher_id='test-central',publisher_boot_id='test-central-boot')
    f.l.envelope=deepcopy(g.envelope)
    return dict(format='rescue-robot-inputs/1', generated_at_ns=now,
        valid_until_ns=now+60_000_000_000, clock=asdict(phase.clock_basis(f.r.adapter)),
        authority=dict(coordinator_id=g.coordinator_id, authority_epoch=g.authority_epoch,
            robot_boot_id=g.expected_robot_boot_id, publisher_id='test-central',
            publisher_boot_id=g.envelope.publisher_boot_id),
        maps={g.robot_id:dict(map_frame=g.frame_ref.native_frame,
            map_session_id=g.frame_ref.map_session_id, map_revision=0, map_update_sequence=0,
            observed_at_ns=now, valid_until_ns=now+60_000_000_000,
            localization_status='VALID', transform_revision=1,
            authority_id='test-mapping', clock_domain_id='test-clock',
            valid_from_ns=now-1_000_000_000, transform_valid_until_ns=now+60_000_000_000,
            transform_xy_yaw=[2.,0.,0.], position_error_m=.01, rotation_error_rad=.01,
            evidence_id='synthetic-calibration')},
        targets={g.target_id:dict(revision=g.target_estimate_revision)},
        approvals={a.approval_id:vars(a).copy()},
        protected_routes={g.route.route_id:dict(route_revision=g.route.route_revision,
            segment_index=g.route.segment_index, robot_id=g.robot_id,
            target_id=g.target_id, target_revision=g.target_estimate_revision,
            endpoint=[3.,0.,0.], map_session_id=g.frame_ref.map_session_id,
            map_revision=0, transform_revision=1, valid_until_ns=g.deadline_ns,
            observed_at_ns=now, protection_loaded=True, passage_free=True,
            evidence_id='synthetic-protected-route')},
        destinations={})


def config(f):
    g=f.g
    return dict(format='rescue-robot-runtime/1', robot_id=g.robot_id,
        mission_frame='mission', schema_version=g.envelope.schema_version,
        publisher_id='test-executor', clock_error_bound_ns=phase.BOUND,
        lease=asdict(phase.POLICY), mapping_authority='test-mapping',
        max_position_error_m=.2, max_rotation_error_rad=.1,
        input_max_age_ns=60_000_000_000, poll_period_s=.05,
        endpoints=dict(action_name='/test/task', lease_topic='/test/lease',
            admission_topic='/test/admission', status_topic='/test/status'),
        input_file='/explicit/test-inputs.json')


class SiteRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.f=phase.Phase1RouteTests(); self.f.setUp()
        self.data=records(self.f); self.cfg=config(self.f)
        self.adapter=self.f.r.adapter; self.adapter.executor.latch='DISARMED'
        self.node=transport.Node()
        bindings=NS(**vars(self.f.r.server.bindings))
        def response():
            return NS(envelope=NS(),result=NS(final_pose=NS(envelope=NS(),time=NS())))
        bindings.ExecuteTask=NS(Result=response,Feedback=self.f.r.server.bindings.ExecuteTask.Feedback)
        self.store=InputStore(self.cfg, wall_clock_ns=self.adapter.wall_clock_ns,
                              read=lambda: deepcopy(self.data))
        self.runtime=SiteRuntime(self.node,self.adapter,self.cfg,self.store,
            bindings=bindings)
        self.adapter.executor.arm(self.f.g.authority_epoch, now=self.adapter.clock(),
                                  readiness=self.adapter.readiness(self.adapter.clock()))

    def test_installed_runtime_goal_first_transforms_endpoint_and_returns_segment(self):
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'accept',self.node.errors)
        self.adapter.step()
        self.assertEqual(self.f.r.f.port.goals[0][1].x,1.)
        self.adapter.on_goal_response(self.f.g.command_id,True)
        self.adapter.on_terminal(self.f.g.command_id,4)
        for _ in range(4): self.f.r.f.tick()
        report=self.runtime.server.ingress.report(self.f.g.command_id)
        response=self.runtime.encode_result(report,self.f.g)
        self.assertTrue(response.result.stop_confirmed)
        self.assertEqual(response.result.completion_scope,'SEGMENT')
        self.assertEqual(response.result.final_pose.mission_pose_status,'UNKNOWN')

    def test_registered_publisher_required_even_when_message_claims_authority(self):
        self.data['authority']['publisher_boot_id']='other-boot'
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'reject')
        self.assertFalse(self.f.r.f.port.goals)

    def test_missing_or_stale_inputs_reject_and_do_not_refresh_timestamp(self):
        self.data['valid_until_ns']=self.adapter.wall_clock_ns()
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'reject')
        self.assertEqual(self.data['generated_at_ns'],self.f.r.now)

    def test_missing_protection_and_operator_records_reject(self):
        self.data['protected_routes']={}
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'reject')
        self.data=records(self.f); self.data['approvals']={}
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'reject')

    def test_changed_source_map_or_revoked_authority_cancels_active_task(self):
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'accept')
        self.adapter.step(); self.adapter.on_goal_response(self.f.g.command_id,True)
        self.data['authority']['authority_epoch']='revoked'
        self.adapter.step()
        self.assertEqual(self.f.r.f.port.cancels,[self.f.g.command_id])

    def test_renewal_checks_registered_publisher_and_active_lease_binding(self):
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'accept')
        self.f.l.envelope.publisher_id='test-central'
        self.assertTrue(self.runtime.server.on_lease(self.f.l))
        before=self.adapter.executor.lease_until
        self.f.l.renewal_sequence+=1
        self.data['authority']['coordinator_id']='revoked'
        self.assertFalse(self.runtime.server.on_lease(self.f.l))
        self.assertEqual(before,self.adapter.executor.lease_until)

    def test_node_runtime_installs_without_watchdog_and_stays_disarmed(self):
        self.adapter.executor.latch='DISARMED'
        self.adapter.status.pop('watchdog',None)
        self.runtime.server.poll()
        self.assertEqual(self.node.messages[-1].readiness,'NOT_READY')
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'reject')
        self.assertFalse(self.f.r.f.port.goals)

    def test_input_atomic_write_and_explicit_policy_no_defaults(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'inputs.json'; atomic_json(p,self.data)
            store=InputStore(self.cfg,wall_clock_ns=self.adapter.wall_clock_ns,
                             read=lambda: __import__('json').loads(p.read_text()))
            self.assertEqual(store.current()['format'],'rescue-robot-inputs/1')
        bad=deepcopy(self.cfg); del bad['lease']['duration_s']
        with self.assertRaises(Exception): InputStore(bad,wall_clock_ns=self.adapter.wall_clock_ns)

    def test_search_area_resolves_configured_point_with_empty_wire_alignment(self):
        g=self.f.g; g.kind='SEARCH_AREA'; g.target_id=''; g.target_estimate_revision=0
        g.has_route=g.has_approval=False; g.search_area_ref=phase.runtime.mapping.uid(998)
        self.data['destinations'][g.search_area_ref]=dict(robot_id=g.robot_id,
            map_session_id=g.frame_ref.map_session_id,map_revision=0,transform_revision=1,
            xy_yaw=[4.,0.,0.],observed_at_ns=self.f.r.now,valid_until_ns=g.deadline_ns)
        g.frame_ref.native_frame=g.frame_ref.alignment_status=''
        self.assertEqual(self.runtime.server.on_goal(g),'accept',self.node.errors)
        self.adapter.step(); self.assertEqual(self.f.r.f.port.goals[0][1].x,2.)

    def test_reobserve_phase1_without_operator_approval(self):
        g=self.f.g; g.kind='REOBSERVE_TARGET'; g.has_approval=False
        self.data['approvals']={}
        self.assertEqual(self.runtime.server.on_goal(g),'accept',self.node.errors)
        self.adapter.step(); self.assertEqual(self.f.r.f.port.goals[0][1].x,1.)

    def test_expired_inputs_after_dispatch_cancel_without_fake_stopped_result(self):
        self.assertEqual(self.runtime.server.on_goal(self.f.g),'accept')
        self.adapter.step(); self.adapter.on_goal_response(self.f.g.command_id,True)
        self.data['valid_until_ns']=self.adapter.wall_clock_ns()
        self.adapter.step()
        self.assertEqual(self.f.r.f.port.cancels,[self.f.g.command_id])
        self.assertIsNone(self.runtime.server.ingress.report(self.f.g.command_id))

    def test_malformed_live_mapping_input_stops_search_instead_of_breaking_timer(self):
        self.test_search_area_resolves_configured_point_with_empty_wire_alignment()
        self.adapter.on_goal_response(self.f.g.command_id,True)
        self.data['maps']={}
        self.adapter.step()
        self.assertEqual(self.f.r.f.port.cancels,[self.f.g.command_id])

    def test_search_destination_change_requires_new_command(self):
        self.test_search_area_resolves_configured_point_with_empty_wire_alignment()
        self.adapter.on_goal_response(self.f.g.command_id,True)
        self.data['destinations'][self.f.g.search_area_ref]['xy_yaw'][0]+=1.
        self.adapter.step()
        self.assertEqual(self.f.r.f.port.cancels,[self.f.g.command_id])


if __name__=='__main__': unittest.main()
