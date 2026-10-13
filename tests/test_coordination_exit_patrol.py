"""TB-6 patrol, the QR exit and the start pose (owner decisions 2026-10-13 evening; 작전 OPS-136·138·139·141; E2).

Patrol (``search_mode=PATROL``): the field gives patrol points [x, y, yaw]*N in TB-6's start pose (the SLAM map
origin is not that pose, OPS-141). Once /rescue/map/start_pose/robot_a arrives the coordinator turns them into
SEARCH_AREA sites ``patrol/<i>`` on the search basis and sends them in order; a point that does not complete is
passed. After the last point the next round starts, ``patrol_rounds`` rounds in all (PATROL_ROUND_<n>), then
PATROL_EXHAUSTED. No start pose: no patrol (START_POSE_UNKNOWN). ``AREAS`` keeps the profile search areas.

Exit (안 「나」): /rescue/exit_location (TargetLocation, class_label exit_qr) registers TB-5's exit; a person
target never comes from it and an exit_qr is never a person target. With SLAM_FREEZE the freeze waits for the
exit (MAP_FREEZE_WAITING_EXIT) and TB-6 keeps patrolling meanwhile. Before the freeze only the search basis is
accepted, after it only the frozen one. At the freeze the target and exit of the search revision are re-based
only when the frozen map is the same map session; another session is refused as before.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from test_coordination_map_freeze import (FROZEN, FROZEN_REV, MAP, SEARCH_REV, TRANSFORM, FakeGrid, exit_msg,
                                          freeze_core, map_state, snapshot_id)
from test_coordination_ros_core import (SCOUT, SUPPORT, admission_msg, location, make_core, receipt, request,
                                        site_msg, status_msg)
from test_coordination_search import NOW, uid

OTHER_MAP = '33333333-3333-4333-8333-333333333333'
POINTS = ((1., 0., 0.), (2., 0., 0.), (2., 1., math.pi / 2))


def ticks(core, kind=None, now=NOW):
    core.on_status(status_msg(SCOUT, observed_at_ns=now - 10**8))
    core.on_status(status_msg(SUPPORT, observed_at_ns=now - 10**8))
    return [f for f in core.tick(now) if kind is None or f['kind'] == kind]


def reason(core, now=NOW):
    core.system_state_change(now)
    return core.last_state['reason']


class ExitRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.core = freeze_core()
        self.core.on_site_event(site_msg())

    def save_target(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        return target

    def test_exit_and_person_never_mix(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(exit_msg(1), now_ns=NOW)
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        with self.assertRaises(RescueError) as caught:
            self.core.on_exit_location(exit_msg(2, label='survivor'), now_ns=NOW)
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        self.assertIsNone(self.core.exit)

    def test_an_exit_registers_once_per_location(self):
        msg = exit_msg(1)
        self.core.on_exit_location(msg, now_ns=NOW)
        self.core.on_exit_location(msg, now_ns=NOW)  # replay
        self.assertEqual(((MAP, SEARCH_REV, TRANSFORM), (0., 6.), 1),
                         (self.core.exit['basis'], self.core.exit['xy'], self.core.exit['revision']))
        self.assertIn('EXIT_REGISTERED', ' '.join(self.core.drain_notes()))

    def test_an_exit_on_another_basis_before_the_freeze_is_refused(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_exit_location(exit_msg(1, revision=SEARCH_REV + 7), now_ns=NOW)
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)

    def test_the_freeze_waits_for_the_exit_and_the_patrol_goes_on(self):
        self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        self.save_target()
        self.assertEqual(1, len(ticks(self.core, 'SEARCH_AREA')))
        self.assertFalse(self.core.take_freeze_request())
        self.assertEqual('MAP_FREEZE_WAITING_EXIT', reason(self.core))

    def test_the_exit_lets_the_freeze_go(self):
        self.save_target()
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)
        ticks(self.core)
        self.assertTrue(self.core.take_freeze_request())

    def test_after_the_freeze_only_frozen_exits_count(self):
        self.save_target()
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)
        ticks(self.core)
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        self.core.on_map_state(map_state(SCOUT))
        with self.assertRaises(RescueError):
            self.core.on_exit_location(exit_msg(2, (0., 6.5)), now_ns=NOW)
        self.core.on_exit_location(exit_msg(3, (0., 6.5), revision=FROZEN_REV), now_ns=NOW)
        self.assertEqual((FROZEN, (0., 6.5)), (self.core.exit['basis'], self.core.exit['xy']))

    def test_a_frozen_map_of_another_session_rebases_nothing(self):
        self.save_target()
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)
        ticks(self.core)
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        self.core.on_map_state(map_state(SCOUT, map_session_id=OTHER_MAP,
                                         snapshot_id=snapshot_id('c' * 64, OTHER_MAP, FROZEN_REV, TRANSFORM)))
        self.assertEqual((OTHER_MAP, FROZEN_REV, TRANSFORM), self.core.frozen)
        self.assertEqual((MAP, SEARCH_REV, TRANSFORM), self.core.exit['basis'])
        self.assertEqual([], ticks(self.core, 'APPROACH_TARGET'))
        self.assertEqual('TARGET_NOT_ON_FROZEN_MAP', reason(self.core))


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core(search_mode='PATROL', patrol_points=POINTS, patrol_rounds=2,
                              policy_changes=dict(executable_bases=frozenset({'PATROL_POINT'})))
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SCOUT))
        self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)

    def start(self, pose):
        self.core.on_map_state(map_state(SCOUT, revision=3))  # TB-6 on the search basis (MAP, 3, 1)
        self.core.on_start_pose(pose)

    def run_point(self, outcome='COMPLETED'):
        (fields,) = ticks(self.core, 'SEARCH_AREA')
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True, terminal_reason=''))
        return fields['search_area_ref']

    def test_no_start_pose_no_patrol(self):
        self.assertEqual([], ticks(self.core, 'SEARCH_AREA'))
        self.assertEqual('START_POSE_UNKNOWN', reason(self.core))

    def test_points_are_turned_by_the_start_pose(self):
        self.start((1., 2., math.pi / 2))
        sites = self.core.search.policy.areas
        self.assertEqual(['patrol/0', 'patrol/1', 'patrol/2'], [s.site_id for s in sites])
        self.assertEqual([(1., 3.), (1., 4.), (0., 4.)],
                         [tuple(round(v, 9) for v in s.mission_xy) for s in sites])
        self.assertAlmostEqual(math.pi, sites[2].entry_yaw)
        self.assertEqual((MAP, 3, 1), (sites[0].map_session_id, sites[0].map_revision, sites[0].transform_revision))

    def test_rounds_in_order_then_exhausted(self):
        self.start((0., 0., 0.))
        order = [self.run_point() for _ in range(3)]
        self.assertEqual(['patrol/0', 'patrol/1', 'patrol/2'], order)
        self.assertEqual('patrol/0', self.run_point())
        self.assertEqual('PATROL_ROUND_2', reason(self.core))
        self.run_point()
        self.run_point()
        self.assertEqual([], ticks(self.core, 'SEARCH_AREA'))
        self.assertEqual('PATROL_EXHAUSTED', reason(self.core))

    def test_a_point_that_does_not_complete_is_passed(self):
        self.start((0., 0., 0.))
        self.assertEqual('patrol/0', self.run_point('UNREACHABLE'))
        self.assertEqual('patrol/1', self.run_point())


class StartPoseTests(unittest.TestCase):
    """OPS-142: the start pose carries no basis; it is bound to TB-6's VALID MapState on the search revision, once
    per map session. Before that it waits; another value in the same session is refused and shown."""

    def setUp(self):
        self.core = make_core(search_mode='PATROL', patrol_points=POINTS, patrol_rounds=1,
                              policy_changes=dict(executable_bases=frozenset({'PATROL_POINT'})))

    def test_waits_for_tb6_on_the_search_revision(self):
        self.core.on_start_pose((0., 0., 0.))
        self.assertEqual((), self.core.search.policy.areas)
        self.core.on_map_state(map_state(SCOUT, revision=3, localization_quality='UNKNOWN'))
        self.assertEqual((), self.core.search.policy.areas)
        self.core.on_map_state(map_state(SCOUT, revision=3))
        self.assertEqual(3, len(self.core.search.policy.areas))

    def test_another_value_in_the_same_session_is_refused(self):
        self.core.on_map_state(map_state(SCOUT, revision=3))
        self.core.on_start_pose((0., 0., 0.))
        self.core.on_start_pose((0., 0., 0.))  # the same again is fine (latched replay)
        with self.assertRaises(RescueError) as caught:
            self.core.on_start_pose((0.5, 0., 0.))
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)
        self.assertEqual('START_POSE_CONFLICT', reason(self.core))
        self.assertEqual((1., 0.), tuple(self.core.search.policy.areas[0].mission_xy))


class ParameterTests(unittest.TestCase):
    def test_typed_parameters(self):
        self.assertEqual(('STRING', 'INTEGER', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[k] for k in ('search_mode', 'patrol_rounds', 'exit_standoff_m')))
        self.assertEqual('DOUBLE_ARRAY', ros_node.UNSET_OK_PARAMETERS['patrol_points'])

    def test_bad_values_are_refused(self):
        for changes in (dict(search_mode='LOOP'), dict(search_mode='PATROL', patrol_points=()),
                        dict(search_mode='PATROL', patrol_points=POINTS),  # PATROL_POINT not executable
                        dict(patrol_rounds=0), dict(exit_standoff_m=0.)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                make_core(**changes)

    def test_reasons_are_listed(self):
        self.assertTrue({'MAP_FREEZE_WAITING_EXIT', 'TARGET_REBASED_ON_FREEZE', 'EXIT_NOT_ON_FROZEN_MAP',
                         'SUPPORT_EXIT_UNREGISTERED', 'START_POSE_UNKNOWN', 'START_POSE_CONFLICT'}
                        <= set(ros_node.MAP_FREEZE_REASONS))


if __name__ == '__main__':
    unittest.main()
