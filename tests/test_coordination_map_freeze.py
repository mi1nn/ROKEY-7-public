"""SLAM map freeze before the TB-5 dispatch (owner decision 2026-10-13, scope Q-29 ⑦; 작전 OPS-113/114/116/117; E2).

No reference map: only TB-6 (SCOUT) runs SLAM while it searches. With ``map_mode=SLAM_FREEZE`` the coordinator
asks for the freeze (/rescue/map/freeze) once a target is confirmed and its report PERSISTED and TB-6 is stopped
with no task; no new SEARCH_AREA goes after that. The frozen basis is TB-6's MapState only when its snapshot_id
is the one computed from the frozen grid's occupancy hash, its session/revision and the profile transform
revision, on a revision other than the search one. After the freeze:

- every goal carries the frozen basis; a location on another basis is refused, and one within merge distance
  of a target observed before the freeze re-bases that target (revision + 1, a new report to save);
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
from test_coordination_ros_core import (SCOUT, SUPPORT, admission_msg, frame, location, make_core, receipt, site_msg,
                                        status_msg)
from test_coordination_search import MAP, NOW

S = 1_000_000_000
TIMEOUT = 5 * S
SEARCH_REV, FROZEN_REV, TRANSFORM = 3, 4, 1
FROZEN = (MAP, FROZEN_REV, TRANSFORM)
SHA = 'c' * 64
CLEARANCE = 0.2
EXIT_XY, DOCK_XY = (0., 6.), (1., 0.)


def snapshot_id(sha256, session, revision, transform_revision):
    """Stand-in for rescue_contracts.map_snapshot_id: any injective function of its four inputs."""
    return f'snap:{sha256}:{session}:{revision}:{transform_revision}'


class FakeGrid:
    """The frozen grid as the core sees it: occupancy hash and the free-cell rule (blocked points here)."""

    def __init__(self, sha256=SHA, blocked=()):
        self.sha256, self.blocked = sha256, tuple(blocked)

    def clear(self, x, y, radius):
        return all(math.dist((x, y), b) > radius for b in self.blocked)


def map_state(robot, revision=FROZEN_REV, quality='VALID', sha256=SHA, **changes):
    values = dict(robot_id=robot, map_session_id=MAP, map_revision=revision, localization_quality=quality,
                  snapshot_id=snapshot_id(sha256, MAP, revision, TRANSFORM))
    return NS(**(values | changes))


def freeze_core(**changes):
    core = make_core(**(dict(support_dispatch='AUTO', approach_standoff_m=1.0, support_pose_max_age_ns=POSE_AGE,
                             map_mode='SLAM_FREEZE', freeze_clearance_m=CLEARANCE, map_freeze_timeout_ns=TIMEOUT,
                             map_snapshot_id=snapshot_id) | changes))
    homes = (HomePosition(SCOUT, DOCK_XY, 0., MAP, SEARCH_REV, TRANSFORM),
             HomePosition(SUPPORT, EXIT_XY, 0., MAP, SEARCH_REV, TRANSFORM))
    core.mission.homes = {h.robot_id: h for h in homes}
    return core


class Base(unittest.TestCase):
    def setUp(self):
        self.core = freeze_core()
        self.core.on_site_event(site_msg())
        self.statuses(NOW)
        self.target = self.core.on_target_location(location(1), now_ns=NOW)

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
        self.core.on_frozen_map(grid or FakeGrid())
        self.core.on_map_state(map_state(SCOUT))
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

    def test_frozen_only_when_the_snapshot_matches_the_grid_on_a_new_revision(self):
        self.save()
        self.tick()
        self.core.take_freeze_request()
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid())
        for msg in (map_state(SCOUT, sha256='d' * 64),           # another grid (a latched old one)
                    map_state(SCOUT, revision=SEARCH_REV),       # still the search revision
                    map_state(SUPPORT)):                         # not TB-6
            with self.subTest(msg=msg):
                self.core.on_map_state(msg)
                self.assertIsNone(self.core.frozen)
        self.tick()
        self.assertEqual('MAP_NOT_FROZEN', self.reason())
        self.core.on_map_state(map_state(SCOUT))
        self.assertEqual(FROZEN, self.core.frozen)

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

    def test_target_seen_only_before_the_freeze_holds(self):
        self.assertEqual([], self.tick(kind='APPROACH_TARGET'))
        self.assertEqual('TARGET_NOT_ON_FROZEN_MAP', self.reason())

    def test_a_frozen_map_observation_rebases_the_target_and_needs_a_new_save(self):
        self.assertEqual(self.target, self.frozen_location())
        self.assertEqual((1, True, False), self.core.mission.targets[self.target])
        self.assertEqual(FROZEN, self.core.target_geo[self.target][0])
        self.core.on_map_state(map_state(SUPPORT))
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        self.assertEqual([], self.tick(kind='APPROACH_TARGET'))  # revision 1 not saved yet

    def test_a_location_on_another_basis_is_refused_after_the_freeze(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(location(3, (4.1, 4.)), now_ns=NOW)
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)

    def test_both_robots_must_be_localized_on_the_frozen_revision(self):
        self.assertEqual(self.target, self.frozen_location())
        self.save()
        self.core.on_pose(pose_msg(map_revision=FROZEN_REV))
        self.core.on_map_state(map_state(SCOUT, quality='UNKNOWN'))
        self.assertEqual(([], 'SCOUT_NOT_LOCALIZED'), (self.tick(kind='APPROACH_TARGET'), self.reason()))
        self.core.on_map_state(map_state(SCOUT))
        self.assertEqual(([], 'SUPPORT_NOT_LOCALIZED'), (self.tick(kind='APPROACH_TARGET'), self.reason()))
        self.core.on_map_state(map_state(SUPPORT, revision=SEARCH_REV))
        self.assertEqual(([], 'SUPPORT_NOT_LOCALIZED'), (self.tick(kind='APPROACH_TARGET'), self.reason()))

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

    def test_exit_not_free_holds_tb5_at_the_target(self):
        self.go(FakeGrid(blocked=[(EXIT_XY[0] + .1, EXIT_XY[1])]))
        (approach,) = self.tick(kind='APPROACH_TARGET')
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual([], self.tick(NOW, 'RETURN_HOME'))  # support_exit_wait 0
        self.assertEqual('SUPPORT_EXIT_NOT_FREE', self.reason())

    def test_no_exit_position_cannot_be_checked_and_holds(self):
        self.go(FakeGrid())
        del self.core.mission.homes[SUPPORT]
        (approach,) = self.tick(kind='APPROACH_TARGET')
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual(([], 'NO_HOME_POSITION'), (self.tick(NOW, 'RETURN_HOME'), self.reason()))

    def test_docking_approach_not_free_holds_tb6(self):
        self.freeze(grid=FakeGrid(blocked=[DOCK_XY]))
        self.core.mission.verdicts[self.target] = 'RESCUE_CONFIRMED'
        self.assertEqual(([], 'SCOUT_HOME_NOT_FREE'), (self.tick(kind='RETURN_HOME'), self.reason()))

    def test_homes_move_to_the_frozen_basis(self):
        self.freeze()
        self.assertEqual({(MAP, FROZEN_REV, TRANSFORM)},
                         {(h.map_session_id, h.map_revision, h.transform_revision)
                          for h in self.core.mission.homes.values()})


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


class ParameterTests(unittest.TestCase):
    def test_typed_required_parameters(self):
        self.assertEqual(('STRING', 'DOUBLE', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[k] for k in ('map_mode', 'freeze_clearance_m',
                                                                'map_freeze_timeout_s')))

    def test_bad_values_are_refused_at_start(self):
        for changes in (dict(map_mode='SLAM'), dict(freeze_clearance_m=0.), dict(freeze_clearance_m=math.nan),
                        dict(map_freeze_timeout_ns=0), dict(map_snapshot_id=None)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                freeze_core(**changes)

    def test_reasons_are_listed(self):
        self.assertEqual({'MAP_FREEZE_WAITING_SCOUT_STOP', 'MAP_FREEZE_REQUESTED', 'MAP_FREEZE_FAILED',
                          'MAP_NOT_FROZEN', 'TARGET_NOT_ON_FROZEN_MAP', 'SCOUT_NOT_LOCALIZED',
                          'SUPPORT_NOT_LOCALIZED', 'SUPPORT_APPROACH_NOT_FREE', 'SUPPORT_EXIT_NOT_FREE',
                          'SCOUT_HOME_NOT_FREE'}, set(ros_node.MAP_FREEZE_REASONS))


if __name__ == '__main__':
    unittest.main()
