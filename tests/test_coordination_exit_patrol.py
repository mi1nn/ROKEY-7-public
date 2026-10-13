"""TB-6 patrol, the QR exit and the start pose (owner decisions 2026-10-13 evening; 작전 OPS-136·138·139·141; E2).

Patrol (``search_mode=PATROL``): the field gives patrol points [x, y, yaw]*N in TB-6's start pose (the SLAM map
origin is not that pose, OPS-141). Once /rescue/map/start_pose/robot_a arrives the coordinator turns them into
SEARCH_AREA sites on the search basis (site ids are UUIDs: the executor takes only canonical UUID refs) and sends them in order; a point that does not complete is
passed. After the last point the next round starts, ``patrol_rounds`` rounds in all (PATROL_ROUND_<n>), then
PATROL_EXHAUSTED. No start pose: no patrol (START_POSE_UNKNOWN). ``AREAS`` keeps the profile search areas.

Exit (안 「나」): /rescue/exit_location (TargetLocation, class_label exit_qr) registers exits; a person target
never comes from it and an exit_qr is never a person target. Several exits are kept, one per QR id
(observation.track_id, OPS-157·158; the allow list is checked by perception only): the latest position counts. The freeze does not wait for an
exit (OPS-156: the first freeze is the doll's); TB-5 leaves without one and only its exit move needs one. Before the freeze only the search basis is
accepted, after it only the frozen one. At the freeze the target and exit of the search revision are re-based
only when the frozen map is the same map session; another session is refused as before.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from test_coordination_map_freeze import (FROZEN, FROZEN_REV, MAP, SEARCH_REV, TRANSFORM, FakeGrid, exit_msg,
                                          freeze_core, map_state, revision_info)
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
        for bad in (exit_msg(2, label='survivor'), exit_msg(4, track=None)):  # no QR id is no exit
            with self.subTest(msg=bad), self.assertRaises(RescueError) as caught:
                self.core.on_exit_location(bad, now_ns=NOW)
            self.assertEqual('INVALID_INPUT', caught.exception.code)
        self.assertEqual([], self.core.exits)

    def test_one_exit_per_qr_id_and_several_are_kept(self):
        msg = exit_msg(1)
        self.core.on_exit_location(msg, now_ns=NOW)
        self.core.on_exit_location(msg, now_ns=NOW)  # replay
        self.core.on_exit_location(exit_msg(2, (0., 6.3)), now_ns=NOW)                 # EXIT-1 seen again
        self.core.on_exit_location(exit_msg(3, (5., 5.), track='EXIT-2'), now_ns=NOW)  # another exit
        self.assertEqual([('EXIT-1', (MAP, SEARCH_REV, TRANSFORM), (0., 6.3), 2),
                          ('EXIT-2', (MAP, SEARCH_REV, TRANSFORM), (5., 5.), 1)],
                         [(e['exit_id'], e['basis'], e['xy'], e['revision']) for e in self.core.exits])
        self.assertIn('EXIT_REGISTERED', ' '.join(self.core.drain_notes()))

    def test_an_exit_on_another_basis_before_the_freeze_is_refused(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_exit_location(exit_msg(1, revision=SEARCH_REV + 7), now_ns=NOW)
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)

    def test_the_freeze_does_not_wait_for_an_exit(self):
        # OPS-156 (main 477eac2 waited): without a QR exit seen, the doll's freeze must still go.
        self.save_target()
        ticks(self.core)
        self.assertTrue(self.core.take_freeze_request())

    def freeze(self):
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        self.core.on_frozen_revision(revision_info())

    def test_between_the_freezes_exits_keep_coming_on_the_search_revision(self):
        # OPS-154·159: after the doll's freeze TB-6 keeps mapping (SLAM, search revision); its exits stay there and
        # move to the final map at the second freeze. TB-6 never sees the first frozen revision.
        self.save_target()
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)
        ticks(self.core)
        self.freeze()
        self.core.on_exit_location(exit_msg(2, (5., 5.), track='EXIT-2'), now_ns=NOW)
        with self.assertRaises(RescueError):
            self.core.on_exit_location(exit_msg(3, (0., 6.5), revision=FROZEN_REV), now_ns=NOW)
        self.assertEqual([('EXIT-1', FROZEN), ('EXIT-2', (MAP, SEARCH_REV, TRANSFORM))],
                         [(e['exit_id'], e['basis']) for e in self.core.exits])

    def test_another_run_s_frozen_revision_is_no_freeze_of_this_one(self):
        self.save_target()
        ticks(self.core)
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        self.core.on_frozen_revision(revision_info(session=OTHER_MAP))
        self.assertIsNone(self.core.frozen)
        self.assertEqual([], ticks(self.core, 'APPROACH_TARGET'))
        self.assertEqual('MAP_NOT_FROZEN', reason(self.core))


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
        self.assertEqual([ros_node.patrol_site_id(MAP, i) for i in range(3)], [s.site_id for s in sites])
        self.assertEqual([(1., 3.), (1., 4.), (0., 4.)],
                         [tuple(round(v, 9) for v in s.mission_xy) for s in sites])
        self.assertAlmostEqual(math.pi, sites[2].entry_yaw)
        self.assertEqual((MAP, 3, 1), (sites[0].map_session_id, sites[0].map_revision, sites[0].transform_revision))

    def test_rounds_in_order_then_exhausted(self):
        self.start((0., 0., 0.))
        order = [self.run_point() for _ in range(3)]
        self.assertEqual([ros_node.patrol_site_id(MAP, i) for i in range(3)], order)
        self.assertEqual(ros_node.patrol_site_id(MAP, 0), self.run_point())
        self.assertEqual('PATROL_ROUND_2', reason(self.core))
        self.run_point()
        self.run_point()
        self.assertEqual([], ticks(self.core, 'SEARCH_AREA'))
        self.assertEqual('PATROL_EXHAUSTED', reason(self.core))

    def test_a_point_that_does_not_complete_is_passed(self):
        self.start((0., 0., 0.))
        self.assertEqual(ros_node.patrol_site_id(MAP, 0), self.run_point('UNREACHABLE'))
        self.assertEqual(ros_node.patrol_site_id(MAP, 1), self.run_point())


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


class StartPoseMessageTests(unittest.TestCase):
    """/rescue/map/start_pose/<TB-6> is a PoseWithCovarianceStamped in the map (OPS-142); yaw by the shared
    planar_origin rule (rescue_contracts.occupancy), never computed here."""

    def msg(self, frame='map', z=0., qz=math.sin(math.pi / 8), qw=math.cos(math.pi / 8)):
        pose = NS(position=NS(x=1., y=2., z=z), orientation=NS(x=0., y=0., z=qz, w=qw))
        return NS(header=NS(frame_id=frame), pose=NS(pose=pose))

    def test_planar_pose_in_the_mission_frame(self):
        x, y, yaw = ros_node.start_pose_from_msg(self.msg(), 'map')
        self.assertEqual((1., 2.), (x, y))
        self.assertAlmostEqual(math.pi / 4, yaw)

    def test_other_frame_or_not_planar_is_refused(self):
        for msg in (self.msg(frame='odom'), self.msg(z=.1)):
            with self.subTest(msg=msg), self.assertRaises(RescueError):
                ros_node.start_pose_from_msg(msg, 'map')


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
        self.assertNotIn('MAP_FREEZE_WAITING_EXIT', ros_node.MAP_FREEZE_REASONS)  # OPS-156: no longer a wait
        self.assertTrue({'TARGET_REBASED_ON_FREEZE', 'EXIT_NOT_ON_FROZEN_MAP',
                         'SUPPORT_EXIT_UNREGISTERED', 'START_POSE_UNKNOWN', 'START_POSE_CONFLICT'}
                        <= set(ros_node.MAP_FREEZE_REASONS))


if __name__ == '__main__':
    unittest.main()
