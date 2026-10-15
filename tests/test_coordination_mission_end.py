"""How a FRONTIER run ends (owner decisions 2026-10-13 night; 작전 OPS-154·155·156·173; E2).

- TB-5 waits at the doll until the final map (MAP_NOT_FINAL), then leads out by the exit nearest along the final
  map's free cells (grid path length, not the straight line), stopping ``exit_standoff_m`` before it on the doll's
  side. No QR exit seen: TB-5's start spot (``support_start_pose``, in TB-6's start pose; SUPPORT_START_UNKNOWN when
  not measured). Exits seen but none reachable: SUPPORT_EXIT_UNREACHABLE, nothing made up.
- ``completion_mode=EXIT_ARRIVAL``: no TB-6 recheck (a webcam candidate is recorded only); TB-6 goes to its docking
  approach once the final map is there; TB-5 at the exit is RESCUE_CONFIRMED (source EXIT_ARRIVAL); both back is
  FINISHED. ``RECHECK`` keeps the C-11 recheck flow.
- The whole map explored and no doll: both freezes still run (TB-6 changes to the final map), TB-5 never leaves,
  TB-6 returns, FINISHED with NO_SURVIVOR_FOUND.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_contracts.occupancy import footprint_clear, occupancy_sha256
from rescue_coordination import live_task_inputs, ros_node
from test_coordination_approach import pose_msg
from test_coordination_frontier import grid, scout_pose
from test_coordination_map_freeze import EXIT_STANDOFF, exit_msg, map_state, revision_info
from test_coordination_ros_core import SCOUT, SUPPORT, admission_msg, make_core, request, site_msg, status_msg
from test_coordination_search import NOW, uid
from test_coordination_two_freezes import B2, Base, R2, core_

S = 1_000_000_000
RES, W, H, ORIGIN = 0.2, 50, 50, (-1., -1., 0.)  # 10 m x 10 m around the doll at (4, 4)


class RealGrid:
    """A frozen grid with the shared rules: occupancy hash and footprint_clear (rescue_contracts)."""

    def __init__(self, walls=()):
        data = [0] * (W * H)
        for (x0, y0, x1, y1) in walls:  # occupied boxes in map metres
            for iy in range(H):
                for ix in range(W):
                    x, y = ORIGIN[0] + (ix + .5) * RES, ORIGIN[1] + (iy + .5) * RES
                    if x0 <= x <= x1 and y0 <= y <= y1:
                        data[iy * W + ix] = 100
        self.fields = dict(frame='map', width=W, height=H, resolution=RES, origin=ORIGIN, data=data)
        self.sha256 = occupancy_sha256(**self.fields)

    def clear(self, x, y, radius):
        return footprint_clear(**self.fields, x=x, y=y, radius=radius)


WALL = (-1., 4.9, 6.0, 5.1)  # between the doll (4, 4) and EXIT-1 (0, 6): around it is long


class ExitChoiceTests(Base):
    def arrive_and_finish_exploring(self, grid_=None, exits=()):
        approach = self.dispatch()
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        for n, (xy, track) in enumerate(exits):
            self.core.on_exit_location(exit_msg(10 + n, xy, track=track), now_ns=NOW)
        self.assertEqual([], self.tick(NOW, 'RETURN_HOME'))  # TB-5 waits at the doll for the whole map
        self.assertEqual('WAITING', self.core.support_exit)
        self.explore_to_the_end(NOW)
        self.tick(NOW + 2)
        self.freeze(R2, grid=grid_ or RealGrid([WALL]))
        self.core.on_map_state(map_state(SUPPORT, revision=R2))
        self.core.on_pose(pose_msg((3., 4.), map_revision=R2, age_ns=-3 * S + 10**8))

    def support_home(self, now=NOW + 3 * S):
        fields = self.tick(now, 'RETURN_HOME')
        return fields, self.core.live_destinations().get(f'home/{SUPPORT}')

    def test_the_exit_nearest_along_free_cells_not_in_a_straight_line(self):
        # EXIT-1 (0, 6) is nearer in a straight line but behind the wall; EXIT-2 (8.5, 4) is nearer to walk to.
        self.core.exits = []
        self.arrive_and_finish_exploring(exits=(((0., 6.), 'EXIT-1'), ((8.5, 4.), 'EXIT-2')))
        fields, home = self.support_home()
        self.assertEqual([SUPPORT], [f['robot_id'] for f in fields])
        self.assertEqual((B2[1], [8.0, 4.0]), (home['map_revision'], [round(v, 6) for v in home['xy_yaw'][:2]]))
        self.assertIn('EXIT_CHOSEN EXIT-2', ' '.join(self.core.drain_notes()))

    def test_no_exit_seen_goes_to_tb5_s_start_spot(self):
        self.core.exits = []
        self.core.support_start_pose = (2., .5, math.pi)
        self.arrive_and_finish_exploring()
        _, home = self.support_home()
        self.assertEqual([2., .5], [round(v, 6) for v in home['xy_yaw'][:2]])
        self.assertAlmostEqual(math.pi, abs(home['xy_yaw'][2]))

    def test_no_exit_and_no_measured_start_spot_holds_with_its_reason(self):
        self.core.exits = []
        self.core.support_start_pose = None
        self.arrive_and_finish_exploring()
        self.assertEqual(([], None), self.support_home())
        self.assertEqual('SUPPORT_START_UNKNOWN', self.reason(NOW + 3 * S))

    def test_exits_seen_but_none_reachable_hold_with_their_reason(self):
        self.core.exits = []
        boxed = RealGrid([WALL, (-1., 4.9, 9.0, 5.1)])  # the wall across the whole map
        self.arrive_and_finish_exploring(boxed, exits=(((0., 6.), 'EXIT-1'),))
        self.assertEqual(([], None), self.support_home())
        self.assertEqual('SUPPORT_EXIT_UNREACHABLE', self.reason(NOW + 3 * S))


class ExitArrivalCompletionTests(Base):
    def test_no_recheck_tb6_returns_on_the_final_map_tb5_at_the_exit_confirms_and_finishes(self):
        self.core.completion_mode = 'EXIT_ARRIVAL'
        approach = self.dispatch()
        self.core.on_admission(admission_msg(approach))
        self.core.on_result(approach['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        self.explore_to_the_end(NOW)
        self.tick(NOW + 2)
        self.freeze(R2, grid=RealGrid([WALL]))
        self.core.on_map_state(map_state(SUPPORT, revision=R2))
        self.core.on_map_state(map_state(SCOUT, revision=R2))
        self.core.on_pose(pose_msg((3., 4.), map_revision=R2, age_ns=-3 * S + 10**8))
        sent = self.tick(NOW + 3 * S, 'RETURN_HOME')
        self.assertEqual({SCOUT, SUPPORT}, {f['robot_id'] for f in sent})  # no verdict needed for TB-6 now
        self.assertEqual({}, self.core.mission.open_rechecks)
        for f in sent:
            self.core.on_admission(admission_msg(f))
            self.core.on_result(f['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        self.assertEqual('RESCUE_CONFIRMED', self.core.mission.verdicts.get(self.target))
        self.assertEqual('FINISHED', self.core.mission.phase())
        self.assertIn('source=EXIT_ARRIVAL', ' '.join(self.core.drain_notes()))


class NoSurvivorTests(unittest.TestCase):
    def returning(self):
        """Whole map, no doll: both freezes done, TB-6's RETURN_HOME sent (core, its fields)."""
        core = core_()
        core.completion_mode = 'EXIT_ARRIVAL'
        core.on_map_state(map_state(SCOUT, revision=3))
        core.on_start_pose((0., 0., 0.))
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_status(status_msg(SUPPORT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        for k in range(2):
            core.on_scout_map(grid(known_cols=40))
            core.on_pose(scout_pose(at=NOW + k))
            core.on_status(status_msg(SCOUT, observed_at_ns=NOW + k))
            core.tick(NOW + k)
        self.assertTrue(core.exploration_complete)
        for revision in (4, 5):  # both freezes still run: TB-6 changes to the final map
            core.on_status(status_msg(SCOUT, observed_at_ns=NOW + 2))
            core.on_status(status_msg(SUPPORT, observed_at_ns=NOW + 2))
            core.tick(NOW + 2)
            self.assertTrue(core.take_freeze_request(), revision)
            core.on_freeze_reply(True, 'frozen')
            g = RealGrid()
            core.on_frozen_map(g)
            core.on_frozen_revision(revision_info(revision, g.sha256))
        core.on_map_state(map_state(SCOUT, revision=5))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW + 3))
        core.on_status(status_msg(SUPPORT, observed_at_ns=NOW + 3))
        (ret,) = core.tick(NOW + 3)
        self.assertEqual((SCOUT, 'RETURN_HOME'), (ret['robot_id'], ret['kind']))
        core.on_admission(admission_msg(ret))
        return core, ret

    def test_whole_map_and_no_doll_tb6_returns_and_the_mission_finishes(self):
        core, ret = self.returning()
        core.on_result(ret['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        core.system_state_change(NOW + 3)
        self.assertEqual(('FINISHED', 'NO_SURVIVOR_FOUND'), (core.last_state['phase'], core.last_state['reason']))

    def test_a_failed_return_is_shown_over_no_survivor(self):
        # 11회차 2026-10-16: RETURN_HOME ended FAILED (ACTION_ABORTED, a Nav2 TF error); SystemState kept
        # NO_SURVIVOR_FOUND and the log had only the RESULT line, so nobody saw that TB-6 would not be sent again
        core, ret = self.returning()
        core.drain_notes()
        core.on_result(ret['task_id'], NS(outcome='FAILED', stop_confirmed=True, terminal_reason='ACTION_ABORTED'))
        self.assertIn(f'SCOUT_RETURN_FAILED task={ret["task_id"]} outcome=FAILED reason=ACTION_ABORTED',
                      core.drain_notes())
        core.system_state_change(NOW + 4)
        self.assertEqual(('RETURNING', 'SCOUT_RETURN_FAILED'), (core.last_state['phase'], core.last_state['reason']))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW + 20 * 10**9))
        self.assertEqual([], [f for f in core.tick(NOW + 20 * 10**9) if f['kind'] == 'RETURN_HOME'])  # #194


class ParameterTests(unittest.TestCase):
    def test_typed_parameters(self):
        self.assertEqual('STRING', ros_node.PARAMETERS['completion_mode'])
        self.assertEqual('DOUBLE_ARRAY', ros_node.UNSET_OK_PARAMETERS['support_start_pose'])

    def test_bad_values_are_refused(self):
        for changes in (dict(completion_mode='ARRIVAL'), dict(support_start_pose=(1., 2.))):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                make_core(**changes)


if __name__ == '__main__':
    unittest.main()
