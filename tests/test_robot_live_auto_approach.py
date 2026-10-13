"""LIVE_ROS APPROACH_TARGET without operator approval (C1-a, user decision 1B/2B 2026-10-13), no ROS.

The SUPPORT robot accepts the coordinator's automatic approach only under an explicit
policy (task.live.auto_target.*, no defaults; off = rejected as before). Path basis: the
approach goal's footprint must be free in the saved map this robot loaded and hashed;
Nav2 then plans the real path (live costmap) and an unplannable goal ends UNREACHABLE.
No operator approval record is created, and the stop / lease / watchdog gates are unchanged.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

import test_robot_task_site_runtime as site
from rescue_robot.executor import ContractError
from rescue_robot.live_sources import Grid
from rescue_robot.task_site_runtime import InputStore, SiteRuntime

POLICY = dict(enabled=True, kinds=['APPROACH_TARGET'], goal_clear_radius_m=0.2, policy_version='test-auto-target')


def grid(cells=None, *, width=60, height=40, resolution=0.05, origin=(-1.0, -1.0)):
    """A saved map in the robot's native frame 'map'; all free unless cells says otherwise."""
    data = [0] * (width * height)
    for (ix, iy), value in (cells or {}).items():
        data[iy * width + ix] = value
    message = NS(header=NS(frame_id='map'), info=NS(width=width, height=height, resolution=resolution,
        origin=NS(position=NS(x=origin[0], y=origin[1], z=0.), orientation=NS(x=0., y=0., z=0., w=1.))), data=data)
    return Grid(message)


def loaded_maps(maps):
    """SavedMapSources stand-in: these maps loaded, no newer frozen revision pending (OPS-159 D2)."""
    return NS(maps=maps, check_new_goal=lambda robot: None)


class LiveAutoApproach(unittest.TestCase):
    def setUp(self):
        self.base = site.SiteRuntimeTests('test_installed_runtime_goal_first_transforms_endpoint_and_returns_segment')
        self.base.setUp()
        b = self.base
        self.f, self.adapter, self.node = b.f, b.adapter, b.node
        self.data = b.data
        g = self.g = b.f.g
        # The coordinator's approach (contract 2026-10-13): Phase 1 shape, no operator approval.
        g.kind = 'APPROACH_TARGET'
        g.has_approval = False
        for key in ('approvals', 'protected_routes'):
            self.data.pop(key)
        self.build(POLICY)

    def build(self, policy, **changes):
        b = self.base
        self.adapter.executor.latch = 'DISARMED'
        self.cfg = dict(b.cfg, input_mode='LIVE_ROS', **changes)
        if policy is not None:
            self.cfg['auto_target'] = deepcopy(policy)
        self.store = InputStore(self.cfg, wall_clock_ns=self.adapter.wall_clock_ns, read=lambda: deepcopy(self.data))
        self.runtime = SiteRuntime(self.node, self.adapter, self.cfg, self.store, bindings=b.runtime.server.bindings)
        # The live saved-map source (SavedMapSources.maps): this robot's loaded, hash-checked map.
        self.runtime.saved_maps = loaded_maps({self.g.robot_id: grid()})
        self.adapter.executor.arm(self.g.authority_epoch, now=self.adapter.clock(),
                                  readiness=self.adapter.readiness(self.adapter.clock()))

    def submit(self):
        return self.runtime.server.on_goal(self.g)

    def test_policy_on_accepts_and_drives_to_the_approach_goal_in_the_native_frame(self):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        # mission (3, 0) with the declared alignment x+2 -> native (1, 0), as for SEARCH_AREA.
        self.assertEqual(1., self.f.r.f.port.goals[0][1].x)
        self.assertEqual({}, self.runtime.gate.approvals)  # no operator approval made up

    def test_completed_approach_reports_target_approach_scope(self):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)
        self.adapter.on_terminal(self.g.command_id, 4)
        for _ in range(4):
            self.f.r.f.tick()
        report = self.runtime.server.ingress.report(self.g.command_id)
        response = self.runtime.encode_result(report, self.g)
        self.assertTrue(response.result.stop_confirmed)
        self.assertEqual('TARGET_APPROACH', response.result.completion_scope)

    def test_policy_off_or_absent_rejects_as_before(self):
        for policy in (dict(POLICY, enabled=False), None):
            with self.subTest(policy=policy):
                self.build(policy)
                self.assertEqual('reject', self.submit())
                self.assertEqual([], self.f.r.f.port.goals)

    def test_occupied_or_unknown_footprint_is_rejected(self):
        # native (1, 0) -> cell ((1 - -1) / 0.05, (0 - -1) / 0.05) = (40, 20)
        for value in (100, -1):
            with self.subTest(value=value):
                self.build(POLICY)
                self.runtime.saved_maps = loaded_maps({self.g.robot_id: grid({(41, 21): value})})
                self.assertEqual('reject', self.submit())
                self.assertEqual([], self.f.r.f.port.goals)

    def test_footprint_must_be_inside_the_map(self):
        self.g.route.approach_goal.pose.position.x = 50.
        self.assertEqual('reject', self.submit())

    def test_no_loaded_saved_map_rejects(self):
        self.runtime.saved_maps = loaded_maps({})
        self.assertEqual('reject', self.submit())
        self.runtime.saved_maps = None
        self.assertEqual('reject', self.submit())

    def test_goal_basis_must_be_this_robots_current_map_basis(self):
        """One shared map (map_basis): the target source's basis equals the mover's declared alignment."""
        for name, value in (('transform_revision', 2), ('map_revision', 1), ('map_session_id', 'other-session')):
            with self.subTest(name=name):
                self.build(POLICY)
                g = deepcopy(self.g)
                setattr(g.frame_ref, name, value)
                g.route.frame_ref = g.frame_ref
                self.assertEqual('reject', self.runtime.server.on_goal(g))

    def test_planner_evidence_or_operator_approval_on_the_wire_is_refused(self):
        for change in (lambda g: setattr(g.route, 'planner_evidence_id', 'made-up'),
                       lambda g: setattr(g, 'has_approval', True)):
            with self.subTest(change=change):
                self.build(POLICY)
                g = deepcopy(self.g)
                change(g)
                self.assertEqual('reject', self.runtime.server.on_goal(g))

    def test_losing_the_saved_map_while_moving_cancels(self):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)
        self.runtime.saved_maps.maps.clear()
        self.adapter.step()
        self.assertEqual([self.g.command_id], self.f.r.f.port.cancels)

    def test_changed_approach_goal_needs_a_new_command(self):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)
        self.runtime.saved_maps.maps[self.g.robot_id] = grid({(41, 21): 100})
        self.adapter.step()
        self.assertEqual([self.g.command_id], self.f.r.f.port.cancels)

    def test_policy_values_are_checked(self):
        for bad in (dict(POLICY, enabled='yes'), dict(POLICY, kinds=[]), dict(POLICY, kinds=['MOVE_SEGMENT']),
                    dict(POLICY, kinds=['APPROACH_TARGET', 'APPROACH_TARGET']),
                    dict(POLICY, goal_clear_radius_m=0.), dict(POLICY, goal_clear_radius_m=float('nan')),
                    dict(POLICY, policy_version=''), {k: v for k, v in POLICY.items() if k != 'kinds'}):
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                self.build(bad)

    def test_search_area_is_unchanged_by_the_policy(self):
        g = self.g
        g.kind = 'SEARCH_AREA'; g.target_id = ''; g.target_estimate_revision = 0
        g.has_route = g.has_approval = False
        g.search_area_ref = site.phase.runtime.mapping.uid(998)
        self.store.config['configured_destinations'] = {g.search_area_ref: dict(frame_id='mission',
            robot_id=g.robot_id, map_session_id=g.frame_ref.map_session_id, map_revision=0, transform_revision=1,
            xy_yaw=[4., 0., 0.])}
        g.frame_ref.native_frame = g.frame_ref.alignment_status = ''
        self.assertEqual('accept', self.submit(), self.node.errors)


if __name__ == '__main__':
    unittest.main()
