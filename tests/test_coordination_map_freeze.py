"""SLAM map freeze before the TB-5 dispatch (owner decision 2026-10-13, scope Q-29 ⑦; 작전 OPS-113/114/116/117; E2).

No reference map: only TB-6 (SCOUT) runs SLAM while it searches. With ``map_mode=SLAM_FREEZE`` the coordinator
asks for the freeze (/rescue/map/freeze) once a target is confirmed and its report PERSISTED and TB-6 is stopped
with no task; no new SEARCH_AREA goes after that. The frozen basis is TB-6's MapState only when its snapshot_id
is the one computed from the frozen grid's occupancy hash, its session/revision and the profile transform
revision, on a revision other than the search one. After the freeze:

- every goal carries the frozen basis; a location on another basis is refused. At the freeze the latest target
  and exit positions of the search revision in the same map session are re-based to the frozen revision (작전
  OPS-138·139: TB-6 stands in front of whichever it found last); the target then needs a new revision saved
  (TARGET_REBASED_ON_FREEZE until PERSISTED). A later frozen-map observation within merge distance merges;
- TB-5 goes only with the target observed on the frozen map and saved again, and both robots localized (VALID)
  on the frozen revision; otherwise SystemState.reason says which;
- the approach point, TB-5's exit and TB-6's docking approach point must be free on the frozen grid within
  ``freeze_clearance_m``; otherwise nothing moves and the reason says which (no other point is made up).

``map_mode=SAVED_MAP`` keeps the earlier behaviour (one basis from the launch values, no freeze).
Hash, snapshot id and the free-cell rule belong to rescue_contracts (통합-로봇); the core takes the frozen grid
as an object with ``sha256`` and ``clear(x, y, radius)`` and the snapshot id function as a parameter.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import mission as mission_module, ros_node
from rescue_coordination.mission import HomePosition
from test_coordination_approach import POSE_AGE, pose_msg
from test_coordination_ros_core import (SCOUT, SUPPORT, admission_msg, frame, location, make_core, obs_time, receipt,
                                        site_msg, status_msg)
from test_coordination_search import MAP, NOW

S = 1_000_000_000
TIMEOUT = 5 * S
SEARCH_REV, FROZEN_REV, TRANSFORM = 3, 4, 1
FROZEN = (MAP, FROZEN_REV, TRANSFORM)
SHA = 'c' * 64
CLEARANCE = 0.2
EXIT_XY, DOCK_XY = (0., 6.), (1., 0.)
EXIT_STANDOFF = 0.5


def exit_msg(n, xy=EXIT_XY, revision=SEARCH_REV, session=MAP, label='exit_qr', track='EXIT-1'):
    """A TargetLocation on /rescue/exit_location (통합-인식, class_label exit_qr, the QR id in track_id; OPS-157)."""
    return location(900 + n, xy, frame_ref=frame(map_revision=revision, map_session_id=session),
                    observation=NS(time=obs_time(), class_label=label, has_track_id=bool(track),
                                   track_id=track or ''))


def exit_point(target_xy=(4., 4.)):
    """TB-5's exit approach: EXIT_STANDOFF before the exit on the side of the target it comes from."""
    return ros_node.approach_point(EXIT_XY, target_xy, EXIT_STANDOFF)[0]


def revision_info(revision=FROZEN_REV, sha256=SHA, session=MAP):
    """A parsed /rescue/map/frozen_revision (통합-로봇 #242: which revision the frozen grid is, by its hash)."""
    return dict(map_session_id=session, map_revision=revision, occupancy_sha256=sha256)


class FakeGrid:
    """The frozen grid as the core sees it: occupancy hash and the free-cell rule (blocked points here)."""

    def __init__(self, sha256=SHA, blocked=()):
        self.sha256, self.blocked = sha256, tuple(blocked)

    def clear(self, x, y, radius):
        return all(math.dist((x, y), b) > radius for b in self.blocked)


def map_state(robot, revision=FROZEN_REV, quality='VALID', **changes):
    values = dict(robot_id=robot, map_session_id=MAP, map_revision=revision, localization_quality=quality,
                  snapshot_id='')
    return NS(**(values | changes))


def freeze_core(**changes):
    core = make_core(**(dict(support_dispatch='AUTO', approach_standoff_m=1.0, support_pose_max_age_ns=POSE_AGE,
                             map_mode='SLAM_FREEZE', freeze_clearance_m=CLEARANCE, map_freeze_timeout_ns=TIMEOUT,
                             exit_standoff_m=EXIT_STANDOFF) | changes))
    homes = (HomePosition(SCOUT, DOCK_XY, 0., MAP, SEARCH_REV, TRANSFORM),
             HomePosition(SUPPORT, EXIT_XY, 0., MAP, SEARCH_REV, TRANSFORM))
    core.mission.homes = {h.robot_id: h for h in homes}
    return core


class Base(unittest.TestCase):
    START = (0., 0., 0.)  # TB-6 start pose in the map (OPS-141); the docking approach point is given in it

    def setUp(self):
        self.core = freeze_core()
        if self.START is not None:
            self.core.on_map_state(map_state(SCOUT, revision=SEARCH_REV))  # TB-6 SLAM on the search revision
            self.core.on_start_pose(self.START)
        self.core.on_site_event(site_msg())
        self.statuses(NOW)
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)  # the freeze needs a registered exit (OPS-136)

    def statuses(self, now, **scout):
        self.core.on_status(status_msg(SCOUT, observed_at_ns=now - 10**8, **scout))
        self.core.on_status(status_msg(SUPPORT, observed_at_ns=now - 10**8))

    def save(self, now=NOW):
        (report,) = [r for r in self.core.reports_due(now) if not r.get('persisted')]
        self.assertTrue(self.core.on_receipt(receipt(report)))

    def tick(self, now=NOW, kind=None):
        self.statuses(now)
        sent = self.core.tick(now)
        return [f for f in sent if kind is None or f['kind'] == kind]

    def reason(self, now=NOW):
        self.core.system_state_change(now)
        return self.core.last_state['reason']

    def freeze(self, now=NOW, grid=None):
        """Saved target -> request -> success -> grid -> TB-6 MapState on the frozen revision."""
        self.save(now)
        self.tick(now)
        self.assertTrue(self.core.take_freeze_request())
        self.core.on_freeze_reply(True, 'frozen')
        grid = grid or FakeGrid()
        self.core.on_frozen_map(grid)
        self.core.on_frozen_revision(revision_info(sha256=grid.sha256))
        self.assertEqual(FROZEN, self.core.frozen)


class FreezeRequestTests(Base):
    def test_one_request_once_the_target_is_saved_and_tb6_is_stopped(self):
        self.tick()
        self.assertFalse(self.core.take_freeze_request())
        self.save()
        self.tick()
        self.assertTrue(self.core.take_freeze_request())
        self.assertFalse(self.core.take_freeze_request())
        self.tick(NOW + S)
        self.assertFalse(self.core.take_freeze_request())
        self.assertEqual('MAP_FREEZE_REQUESTED', self.reason(NOW + S))

    def test_waits_for_tb6_to_stop_and_finish_its_task(self):
        self.save()
        self.core.on_status(status_msg(SCOUT, observed_at_ns=NOW - 10**8, motion_state='MOVING'))
        self.core.on_status(status_msg(SUPPORT, observed_at_ns=NOW - 10**8))
        self.core.tick(NOW)
        self.assertFalse(self.core.take_freeze_request())
        self.assertEqual('MAP_FREEZE_WAITING_SCOUT_STOP', self.reason())

    def test_no_new_search_area_once_a_target_is_saved(self):
        self.core.mission.search_authorized = True
        self.save()
        self.assertEqual([], self.tick(kind='SEARCH_AREA'))

    def test_refused_or_timed_out_freeze_fails_and_is_not_asked_again(self):
        for case in ('refused', 'timeout'):
            with self.subTest(case=case):
                self.setUp()
                self.save()
                self.tick()
                self.assertTrue(self.core.take_freeze_request())
                if case == 'refused':
                    self.core.on_freeze_reply(False, 'no map')
                    now = NOW + S
                else:
                    self.core.on_freeze_reply(True, 'frozen')
                    now = NOW + TIMEOUT + 1
                self.assertEqual([], self.tick(now, 'APPROACH_TARGET'))
                self.assertFalse(self.core.take_freeze_request())
                self.assertEqual('MAP_FREEZE_FAILED', self.reason(now))
                self.assertIsNone(self.core.frozen)

    def test_frozen_only_when_the_revision_text_pairs_the_grid(self):
        # 통합-로봇 #242 (OPS-159): /rescue/map/frozen_revision names the grid by its hash; TB-6's MapState stays on
        # the SLAM revision after the first freeze, so it no longer tells the frozen revision.
        self.save()
        self.tick()
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        for info in (revision_info(sha256='d' * 64),              # another grid (a latched old one)
                     revision_info(revision=SEARCH_REV + 2),      # not the next revision
                     revision_info(session='33333333-3333-4333-8333-333333333333')):  # another run
            with self.subTest(info=info):
                self.core.on_frozen_revision(info)
                self.assertIsNone(self.core.frozen)
        self.core.on_map_state(map_state(SCOUT))  # a MapState never freezes
        self.assertIsNone(self.core.frozen)
        self.tick()
        self.assertEqual('MAP_NOT_FROZEN', self.reason())
        self.core.on_frozen_revision(revision_info())
        self.assertEqual(FROZEN, self.core.frozen)

    def test_a_new_unsaved_revision_does_not_step_the_phase_back(self):
        # Same rule outside the freeze (OPS-147): a later observation raising the revision keeps TARGET_FOUND.
        core = make_core()
        core.on_site_event(site_msg())
        core.on_target_location(location(1), now_ns=NOW)
        (report,) = core.reports_due(NOW)
        core.on_receipt(receipt(report))
        self.assertEqual('TARGET_FOUND', core.mission.phase())
        core.on_target_location(location(2, (4.2, 4.)), now_ns=NOW)
        self.assertEqual('TARGET_FOUND', core.mission.phase())

    def test_saved_map_mode_asks_for_no_freeze(self):
        core = make_core(support_dispatch='AUTO')
        core.on_site_event(site_msg())
        core.on_status(status_msg(SUPPORT))
        core.on_pose(pose_msg())
        core.on_target_location(location(1), now_ns=NOW)
        (report,) = core.reports_due(NOW)
        core.on_receipt(receipt(report))
        self.assertEqual(['APPROACH_TARGET'], [f['kind'] for f in core.tick(NOW)])
        self.assertFalse(core.take_freeze_request())


class AfterFreezeTests(Base):
    def setUp(self):
        super().setUp()
        self.freeze()

    def frozen_location(self, n=2, xy=(4.2, 4.)):
        return self.core.on_target_location(location(n, xy, frame_ref=frame(map_revision=FROZEN_REV)), now_ns=NOW)

    def ready_to_go(self):
        """Target observed on the frozen map and saved; both robots localized there; TB-5 pose there."""
        self.assertEqual(self.target, self.frozen_location())
        self.save()
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))

    def test_the_search_revision_target_and_exit_are_rebased_at_the_freeze(self):
        self.assertEqual(FROZEN, self.core.target_geo[self.target][0])
        self.assertEqual((1, True, False), self.core.mission.targets[self.target])
        self.assertEqual([FROZEN], [e['basis'] for e in self.core.exits])
        (report,) = self.core.reports_due(NOW)  # the rebased revision is saved again, with its latest location
        self.assertEqual((self.target, 1, [location(1).location_id]),
                         (report['target_id'], report['target_revision'], report['record_ids']))
        self.assertEqual([], self.tick(kind='APPROACH_TARGET'))
        self.assertEqual('TARGET_REBASED_ON_FREEZE', self.reason())
        self.assertIn('TARGET_REBASED_ON_FREEZE', ' '.join(self.core.drain_notes()))
        self.core.on_receipt(receipt(report))
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        (fields,) = self.tick(kind='APPROACH_TARGET')
        goal = fields['route']['approach_goal']
        self.assertEqual(FROZEN, (goal.map_session_id, goal.map_revision, goal.transform_revision))

    def test_the_phase_stays_target_found_while_the_rebased_revision_waits(self):
        # 작전 OPS-147: the doll was found; the screen must not step back to DISASTER while the re-based revision
        # waits for its receipt. The wait is the reason, not the phase.
        self.tick()
        self.core.system_state_change(NOW)
        self.assertEqual(('TARGET_FOUND', 'TARGET_REBASED_ON_FREEZE'),
                         (self.core.last_state['phase'], self.core.last_state['reason']))

    def test_a_frozen_map_observation_merges_and_needs_a_new_save(self):
        self.assertEqual(self.target, self.frozen_location())
        self.assertEqual((2, True, False), self.core.mission.targets[self.target])
        self.assertEqual(FROZEN, self.core.target_geo[self.target][0])
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        self.assertEqual([], self.tick(kind='APPROACH_TARGET'))  # revision 2 not saved yet

    def test_a_location_on_another_basis_is_refused_after_the_freeze(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(location(3, (4.1, 4.)), now_ns=NOW)
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)

    def test_tb5_must_be_localized_on_the_frozen_revision_tb6_stays_on_slam(self):
        # OPS-154·159: after the first freeze TB-6 keeps mapping (SLAM, search revision); only TB-5 moves to it.
        self.save()
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        self.core.on_map_state(map_state(SCOUT, revision=SEARCH_REV))
        self.assertEqual(([], 'SUPPORT_NOT_LOCALIZED'), (self.tick(kind='APPROACH_TARGET'), self.reason()))
        self.core.on_map_state(map_state(SUPPORT, revision=SEARCH_REV))
        self.assertEqual(([], 'SUPPORT_NOT_LOCALIZED'), (self.tick(kind='APPROACH_TARGET'), self.reason()))
        self.core.on_map_state(map_state(SUPPORT))
        self.assertEqual(1, len(self.tick(kind='APPROACH_TARGET')))

    def test_the_approach_goes_on_the_frozen_basis(self):
        self.ready_to_go()
        (fields,) = self.tick(kind='APPROACH_TARGET')
        goal = fields['route']['approach_goal']
        self.assertEqual(FROZEN, (goal.map_session_id, goal.map_revision, goal.transform_revision))
        self.assertEqual((3.2, 4.0), tuple(round(v, 9) for v in goal.mission_xy))

    def test_a_support_pose_on_the_search_revision_is_not_used(self):
        self.ready_to_go()
        self.core.on_pose(pose_msg())
        self.assertEqual(([], 'SUPPORT_POSE_BASIS_MISMATCH'), (self.tick(kind='APPROACH_TARGET'), self.reason()))


class FreeCellTests(Base):
    def go(self, grid):
        self.freeze(grid=grid)
        self.core.on_target_location(location(2, (4.2, 4.), frame_ref=frame(map_revision=FROZEN_REV)), now_ns=NOW)
        self.save()
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))

    def test_approach_point_on_a_wall_or_unknown_cell_holds(self):
        self.go(FakeGrid(blocked=[(3.25, 4.)]))
        self.assertEqual(([], 'SUPPORT_APPROACH_NOT_FREE'), (self.tick(kind='APPROACH_TARGET'), self.reason()))

    def arrive(self):
        (approach,) = self.tick(kind='APPROACH_TARGET')
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))

    def test_exit_approach_not_free_holds_only_the_exit_move(self):
        # OPS-154·156: TB-5 leaves for the doll at once; a blocked exit holds its way out, not the dispatch.
        self.go(FakeGrid(blocked=[exit_point((4.2, 4.))]))
        self.arrive()
        self.assertEqual(([], 'SUPPORT_EXIT_NOT_FREE'), (self.tick(NOW, 'RETURN_HOME'), self.reason()))

    def test_no_exit_seen_tb5_still_goes_and_its_exit_move_waits(self):
        self.core.exits = []
        self.go(FakeGrid())
        self.arrive()
        self.assertEqual(([], 'SUPPORT_EXIT_UNREGISTERED'), (self.tick(NOW, 'RETURN_HOME'), self.reason()))

    def test_tb5_takes_the_nearer_of_two_exits(self):
        # Interim rule until the grid path length (OPS-154 3): straight-line distance from the doll.
        self.go(FakeGrid())
        self.core.on_exit_location(exit_msg(5, (5., 4.), revision=FROZEN_REV, track='EXIT-2'), now_ns=NOW)
        self.arrive()
        self.tick(NOW, 'RETURN_HOME')
        home = self.core.live_destinations()[f'home/{SUPPORT}']
        self.assertEqual([round(v, 9) for v in ros_node.approach_point((5., 4.), (4.2, 4.), EXIT_STANDOFF)[0]],
                         [round(v, 9) for v in home['xy_yaw'][:2]])

    def test_tb5_exit_is_the_registered_exit_on_the_frozen_map(self):
        # OPS-136 4: home/<TB-5> is the registered exit approach in ~/task_inputs, not the profile home.
        self.go(FakeGrid())
        (approach,) = self.tick(kind='APPROACH_TARGET')
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        (fields,) = self.tick(NOW, 'RETURN_HOME')
        self.assertEqual(f'home/{SUPPORT}', fields['home_ref'])
        home = self.core.live_destinations()[f'home/{SUPPORT}']
        xy = exit_point((4.2, 4.))
        self.assertEqual((SUPPORT, MAP, FROZEN_REV, TRANSFORM), (home['robot_id'], home['map_session_id'],
                                                                 home['map_revision'], home['transform_revision']))
        self.assertEqual([round(v, 9) for v in xy], [round(v, 9) for v in home['xy_yaw'][:2]])

    def test_docking_approach_not_free_holds_tb6(self):
        self.freeze(grid=FakeGrid(blocked=[DOCK_XY]))
        self.core.mission.verdicts[self.target] = 'RESCUE_CONFIRMED'
        self.assertEqual(([], 'SCOUT_HOME_NOT_FREE'), (self.tick(kind='RETURN_HOME'), self.reason()))

    def test_homes_move_to_the_frozen_basis(self):
        self.freeze()
        self.assertEqual({(MAP, FROZEN_REV, TRANSFORM)},
                         {(h.map_session_id, h.map_revision, h.transform_revision)
                          for h in self.core.mission.homes.values()})


class TaskInputsDestinationsTests(Base):
    """~/task_inputs carries the coordinator's home/<id> destinations next to the search areas (robot #232 accepts
    its own 'home/<id>' key; OPS-144)."""

    def test_tb5_exit_and_tb6_dock_go_to_the_robots(self):
        from rescue_coordination.live_task_inputs import destinations
        self.assertNotIn(f'home/{SUPPORT}', destinations(self.core))  # nothing before the freeze
        self.freeze()
        self.core.on_target_location(location(2, (4.2, 4.), frame_ref=frame(map_revision=FROZEN_REV)), now_ns=NOW)
        self.save()
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        self.tick(kind='APPROACH_TARGET')
        record = destinations(self.core)
        self.assertEqual({f'home/{SCOUT}', f'home/{SUPPORT}'}, {k for k in record if k.startswith('home/')})
        self.assertEqual(self.core.live_destinations()[f'home/{SUPPORT}'], record[f'home/{SUPPORT}'])


class DockStartPoseTests(Base):
    """OPS-141: the TB-6 docking approach point is measured from TB-6's start pose, not the SLAM map origin."""
    START = (1., 1., 0.)

    def test_dock_approach_moves_with_the_start_pose(self):
        self.freeze(grid=FakeGrid(blocked=[(DOCK_XY[0] + 1., DOCK_XY[1] + 1.)]))
        self.core.mission.verdicts[self.target] = 'RESCUE_CONFIRMED'
        self.assertEqual(([], 'SCOUT_HOME_NOT_FREE'), (self.tick(kind='RETURN_HOME'), self.reason()))
        home = self.core.live_destinations()[f'home/{SCOUT}']
        self.assertEqual(([2., 1.], FROZEN_REV), (home['xy_yaw'][:2], home['map_revision']))


class NoStartPoseTests(Base):
    START = None

    def test_tb6_return_needs_the_start_pose(self):
        self.freeze()
        self.core.mission.verdicts[self.target] = 'RESCUE_CONFIRMED'
        self.assertEqual(([], 'START_POSE_UNKNOWN'), (self.tick(kind='RETURN_HOME'), self.reason()))


class NodeWiringTests(Base):
    def test_the_node_tick_calls_the_freeze_once_and_logs_it(self):
        from test_coordination_ros_tick import Recorder
        self.save()
        out = Recorder()
        ros_node.node_tick(self.core, NOW, out)
        self.assertEqual(1, out.events.count(('freeze',)))
        self.assertIn(('note', 'MAP_FREEZE'), out.events)
        out = Recorder()
        ros_node.node_tick(self.core, NOW + 1, out)
        self.assertNotIn(('freeze',), out.events)

    def test_an_invalid_frozen_grid_while_waiting_fails_the_freeze(self):
        self.save()
        self.tick()
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map_invalid('invalid occupancy grid')
        self.tick()
        self.assertEqual('MAP_FREEZE_FAILED', self.reason())

    def test_an_invalid_grid_before_any_request_is_only_logged(self):
        self.core.on_frozen_map_invalid('invalid occupancy grid')
        self.assertIsNone(self.core.freeze)


class FrozenGridTests(unittest.TestCase):
    """ros_node.FrozenGrid reads an OccupancyGrid with the shared rescue_contracts.occupancy functions (#208)."""

    def msg(self, data=None, qz=0., qw=1.):
        origin = NS(position=NS(x=-1., y=-2., z=0.), orientation=NS(x=0., y=0., z=qz, w=qw))
        return NS(header=NS(frame_id='map'), info=NS(width=4, height=3, resolution=.5, origin=origin),
                  data=data if data is not None else [0] * 12)

    def test_hash_and_free_cells_are_the_shared_rule(self):
        from rescue_contracts.occupancy import footprint_clear, occupancy_sha256
        data = [0] * 11 + [100]
        grid = ros_node.FrozenGrid(self.msg(data))
        fields = dict(frame='map', width=4, height=3, resolution=.5, origin=(-1., -2., 0.), data=data)
        self.assertEqual(occupancy_sha256(**fields), grid.sha256)
        for xy in ((-.5, -1.5), (.75, -.75)):
            with self.subTest(xy=xy):
                self.assertEqual(footprint_clear(**fields, x=xy[0], y=xy[1], radius=.1), grid.clear(*xy, .1))
        self.assertFalse(grid.clear(.75, -.75, .1))  # the occupied corner cell

    def test_an_invalid_grid_or_origin_is_a_value_error(self):
        for msg in (self.msg(data=[0] * 11), self.msg(qz=.5, qw=.5)):
            with self.subTest(msg=msg), self.assertRaises(ValueError):
                ros_node.FrozenGrid(msg)


class ParameterTests(unittest.TestCase):
    def test_typed_required_parameters(self):
        self.assertEqual(('STRING', 'DOUBLE', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[k] for k in ('map_mode', 'freeze_clearance_m',
                                                                'map_freeze_timeout_s')))

    def test_bad_values_are_refused_at_start(self):
        for changes in (dict(map_mode='SLAM'), dict(freeze_clearance_m=0.), dict(freeze_clearance_m=math.nan),
                        dict(map_freeze_timeout_ns=0)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                freeze_core(**changes)

    def test_reasons_are_listed(self):
        self.assertEqual({'MAP_FREEZE_WAITING_SCOUT_STOP', 'MAP_FREEZE_REQUESTED', 'MAP_FREEZE_FAILED',
                          'MAP_NOT_FROZEN', 'TARGET_NOT_ON_FROZEN_MAP', 'SCOUT_NOT_LOCALIZED',
                          'SUPPORT_NOT_LOCALIZED', 'SUPPORT_APPROACH_NOT_FREE', 'SUPPORT_EXIT_NOT_FREE',
                          'SCOUT_HOME_NOT_FREE', 'TARGET_REBASED_ON_FREEZE',
                          'EXIT_NOT_ON_FROZEN_MAP', 'SUPPORT_EXIT_UNREGISTERED', 'START_POSE_UNKNOWN',
                          'START_POSE_CONFLICT'}, set(ros_node.MAP_FREEZE_REASONS))


if __name__ == '__main__':
    unittest.main()
