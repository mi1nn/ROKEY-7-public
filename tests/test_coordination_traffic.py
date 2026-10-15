"""Traffic control, stage 1 (owner decision 2026-10-14, scope Q-29 ⑤; 작전 OPS-204·205; E2).

TB-5 always goes first: it leads the rescuer, who follows it on foot. With ``traffic_control=TB5_PRIORITY``
(SLAM_FREEZE only, the two robots on one map):

- Before a TB-5 command, its path is the clear grid path on the frozen map (rescue_contracts.grid_search). TB-6
  within ``traffic_path_clearance_m`` of that path is sent aside first: to the clear cell outside the path (and
  TB-5) with the shortest grid path on TB-6's live map, no more than ``traffic_yield_max_m`` away, facing the way
  it goes (turn in place, then forward; never backwards). TB-5 waits until it stepped aside. No such cell: TB-6
  stays stopped, TB-5 waits, TRAFFIC_SCOUT_YIELD_NO_SPACE (no other move is made up).
- TB-6 moving near that path is canceled (stopped, the safe side) and then sent aside; TB-5 waits for the stop
  (TRAFFIC_SUPPORT_WAITING_SCOUT_STOP) up to ``traffic_stop_timeout_ns``, then TRAFFIC_SCOUT_STOP_TIMEOUT.
- While TB-5 moves, a new TB-6 goal near its path is held (TRAFFIC_SCOUT_HELD_FOR_SUPPORT); elsewhere TB-6 moves.
- Robots closer than ``traffic_min_separation_m``, or a new goal that close to the other robot: the new command is
  held (TRAFFIC_ROBOTS_TOO_CLOSE); a command already moving is not canceled for it. The one exception is TB-6
  stepping aside, away from TB-5 (so the two never wait on each other).
"""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_contracts.occupancy import occupancy_sha256
from rescue_coordination import live_task_inputs, ros_node
from test_coordination_approach import pose_msg
from test_coordination_frontier import saved_map, scout_pose
from test_coordination_map_freeze import map_state
from test_coordination_mission_end import ORIGIN, RES, H, W, RealGrid
from test_coordination_ros_core import SCOUT, SUPPORT, admission_msg, make_core, request, site_msg, status_msg
from test_coordination_search import NOW, uid
from test_coordination_two_freezes import R1, Base, core_

S = 1_000_000_000
MIN_SEP, CLEAR, YIELD_MAX, STOP_TIMEOUT = 0.6, 0.6, 2.0, 3 * S
TB5_START = (0., 4.)  # the doll at (4, 4): TB-5's approach point is (3, 4), its path along y = 4
CORRIDOR = [(-1., -1., 9.0, 3.6), (-1., 4.4, 9.0, 9.0)]  # free only for 3.6 < y < 4.4


def scout_grid(unknown=lambda x, y: x > 8., walls=()):
    """TB-6's live map: free but for ``walls`` (occupied) and ``unknown`` (-1), same cells as RealGrid."""
    fields = dict(RealGrid(walls).fields)
    data = list(fields['data'])
    for iy in range(H):
        for ix in range(W):
            x, y = ORIGIN[0] + (ix + .5) * RES, ORIGIN[1] + (iy + .5) * RES
            if data[iy * W + ix] == 0 and unknown(x, y):
                data[iy * W + ix] = -1
    fields['data'] = data
    return NS(fields=fields, sha256=occupancy_sha256(**fields))


def path_distance(xy):
    """Distance from ``xy`` to TB-5's path (0, 4) -> (3, 4)."""
    return math.dist(xy, (min(max(xy[0], 0.), 3.), 4.))


class TrafficBase(Base):
    CORE = dict(traffic_control='TB5_PRIORITY', traffic_min_separation_m=MIN_SEP, traffic_path_clearance_m=CLEAR,
                traffic_yield_max_m=YIELD_MAX, traffic_stop_timeout_ns=STOP_TIMEOUT,
                support_start_pose=(*TB5_START, 0.))
    FROZEN = RealGrid()

    def freeze(self, revision, sha=None, grid=None):
        super().freeze(revision, grid=grid or self.FROZEN)

    def scout_at(self, xy, now=NOW, grid=None):
        self.core.on_scout_map(grid or scout_grid())
        self.core.on_pose(scout_pose(xy, at=now))

    def support_at(self, xy, now=NOW):
        self.core.on_map_state(map_state(SUPPORT, revision=R1))
        self.core.on_pose(pose_msg(xy, map_revision=R1, age_ns=-(now - NOW) + 10**8))

    def sent(self, now):
        return {(f['robot_id'], f['kind']): f for f in self.tick(now)}

    def finish(self, fields, outcome='COMPLETED'):
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True, terminal_reason=''))

    def destination(self, fields):
        return live_task_inputs.destinations(self.core)[fields['search_area_ref']]['xy_yaw']


class StepAsideTests(TrafficBase):
    def test_tb6_on_tb5_s_path_steps_aside_first_then_tb5_goes(self):
        self.scout_at((2., 4.2))
        self.support_at(TB5_START)
        sent = self.sent(NOW)
        self.assertEqual({(SCOUT, 'SEARCH_AREA')}, set(sent))  # TB-6 aside; TB-5 not yet
        self.assertEqual('TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD', self.reason(NOW))
        x, y, yaw = self.destination(sent[(SCOUT, 'SEARCH_AREA')])
        self.assertGreater(path_distance((x, y)), CLEAR)
        self.assertLessEqual(math.dist((2., 4.2), (x, y)), YIELD_MAX)
        self.assertAlmostEqual(math.atan2(y - 4.2, x - 2.), yaw, places=6)  # turn, then forward
        self.assertIn('TRAFFIC_YIELD', ' '.join(self.core.drain_notes()))
        self.assertEqual({}, {k: v for k, v in self.sent(NOW + 1).items() if k[0] == SUPPORT})  # still aside
        self.finish(sent[(SCOUT, 'SEARCH_AREA')])
        self.scout_at((x, y), NOW + 2)
        self.support_at(TB5_START, NOW + 2)
        self.assertIn((SUPPORT, 'APPROACH_TARGET'), self.sent(NOW + 2))
        self.assertNotIn((x, y), [tuple(p) for p in self.core.frontier_tried])  # a step aside is no frontier
        self.assertIn('TRAFFIC_YIELD_DONE', ' '.join(self.core.drain_notes()))

    def test_no_room_to_step_aside_keeps_both_stopped_with_the_reason(self):
        self.core.traffic_yield_max_m = 1.0  # in the corridor the nearest spot off the path is 1.6 m ahead
        self.scout_at((2., 4.), grid=scout_grid(walls=CORRIDOR))
        self.support_at(TB5_START)
        self.assertEqual({}, self.sent(NOW))
        self.assertEqual('TRAFFIC_SCOUT_YIELD_NO_SPACE', self.reason(NOW))

    def test_tb6_pose_unknown_holds_tb5(self):
        self.support_at(TB5_START)
        self.assertEqual({}, {k: v for k, v in self.sent(NOW).items() if k[0] == SUPPORT})
        self.assertEqual('TRAFFIC_SCOUT_POSE_UNKNOWN', self.core.traffic_hold.get('SUPPORT'))


class UnmeasuredTb5StartTests(TrafficBase):
    CORE = TrafficBase.CORE | dict(support_start_pose=None)

    def test_tb6_still_explores_before_tb5_ever_moved_and_says_the_check_is_off(self):
        # A field file without robots.<TB-5>.initial_pose_xy_yaw (not measured yet): holding TB-6 then would
        # also hold the doll's freeze and so TB-5 itself. TB-5 has not moved and has no path: only the separation
        # check is off. Never silent (team rule 6): a note for every TB-6 goal sent so, and SystemState shows it.
        self.scout_at((1., 4.2))
        first = self.sent(NOW)[(SCOUT, 'SEARCH_AREA')]
        self.assertIn(f'TRAFFIC_SEPARATION_UNCHECKED task={first["task_id"]}', ' '.join(self.core.drain_notes()))
        self.finish(first)
        self.scout_at((2., 4.2), NOW + 1, grid=scout_grid(unknown=lambda x, y: y < 1.))  # another frontier
        second = self.sent(NOW + 1)[(SCOUT, 'SEARCH_AREA')]
        self.assertIn(f'TRAFFIC_SEPARATION_UNCHECKED task={second["task_id"]}', ' '.join(self.core.drain_notes()))

    def test_system_state_shows_it_when_nothing_else_holds(self):
        # Before the doll's freeze nothing else is held: SystemState.reason shows the check is off.
        core = core_(**self.CORE)
        core.on_map_state(map_state(SCOUT, revision=3))
        core.on_start_pose((0., 0., 0.))
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_status(status_msg(SUPPORT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(scout_grid())
        core.on_pose(scout_pose((1., 4.2)))
        self.assertIn('SEARCH_AREA', [f['kind'] for f in core.tick(NOW)])
        core.system_state_change(NOW)
        self.assertEqual('TRAFFIC_SEPARATION_UNCHECKED', core.last_state['reason'])


class StopThenStepAsideTests(TrafficBase):
    def test_tb6_moving_near_tb5_s_path_is_canceled_then_sent_aside(self):
        self.scout_at((1., 4.2))  # TB-5 not localized yet: TB-6 explores (TB-5 at its start spot)
        frontier = self.sent(NOW)[(SCOUT, 'SEARCH_AREA')]
        self.core.on_admission(admission_msg(frontier))
        self.support_at(TB5_START, NOW + 1)
        self.scout_at((1.5, 4.2), NOW + 1)
        self.assertEqual({}, self.sent(NOW + 1))
        self.assertEqual([frontier['task_id']], self.core.take_cancels())  # once
        self.assertEqual([], self.core.take_cancels())
        self.assertEqual('TRAFFIC_SUPPORT_WAITING_SCOUT_STOP', self.reason(NOW + 1))
        self.support_at(TB5_START, NOW + STOP_TIMEOUT + 2)
        self.scout_at((1.5, 4.2), NOW + STOP_TIMEOUT + 2)
        self.assertEqual({}, self.sent(NOW + STOP_TIMEOUT + 2))
        self.assertEqual('TRAFFIC_SCOUT_STOP_TIMEOUT', self.reason(NOW + STOP_TIMEOUT + 2))
        self.core.on_result(frontier['task_id'], NS(outcome='CANCELED', stop_confirmed=True, terminal_reason=''))
        self.assertEqual([], self.core.frontier_tried)  # stopped for TB-5, not tried
        now = NOW + STOP_TIMEOUT + 3
        self.support_at(TB5_START, now)
        self.scout_at((1.5, 4.2), now)
        sent = self.sent(now)
        self.assertEqual({(SCOUT, 'SEARCH_AREA')}, set(sent))
        self.assertGreater(path_distance(self.destination(sent[(SCOUT, 'SEARCH_AREA')])[:2]), CLEAR)
        self.assertEqual('TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD', self.reason(now))

    def test_node_tick_sends_the_cancel(self):
        out = NS(calls=[])
        for name in ('task', 'watch', 'report', 'lease', 'block', 'targets', 'state', 'freeze', 'note', 'cancel'):
            setattr(out, name, lambda *a, n=name: out.calls.append((n, a)))
        self.scout_at((1., 4.2))
        frontier = self.sent(NOW)[(SCOUT, 'SEARCH_AREA')]
        self.core.on_admission(admission_msg(frontier))
        self.support_at(TB5_START, NOW + 1)
        self.scout_at((1.5, 4.2), NOW + 1)
        self.status(NOW + 1)
        ros_node.node_tick(self.core, NOW + 1, out)
        self.assertIn(('cancel', (frontier['task_id'],)), out.calls)


class WhileTb5MovesTests(TrafficBase):
    def test_a_tb6_goal_on_tb5_s_path_waits_until_tb5_arrived(self):
        self.scout_at((2., 5.4), grid=scout_grid(unknown=lambda x, y: y < 1.))  # the next frontier is down
        self.support_at(TB5_START)
        sent = self.sent(NOW)
        self.assertEqual({(SUPPORT, 'APPROACH_TARGET')}, set(sent))
        self.assertEqual('TRAFFIC_SCOUT_HELD_FOR_SUPPORT', self.reason(NOW))
        self.finish(sent[(SUPPORT, 'APPROACH_TARGET')])
        self.support_at((3., 4.), NOW + 1)
        self.scout_at((2., 5.4), NOW + 1, grid=scout_grid(unknown=lambda x, y: y < 1.))
        self.assertIn((SCOUT, 'SEARCH_AREA'), self.sent(NOW + 1))

    def test_a_tb6_goal_elsewhere_goes_while_tb5_moves(self):
        self.scout_at((2., 6.))
        self.support_at(TB5_START)
        self.assertEqual({(SUPPORT, 'APPROACH_TARGET'), (SCOUT, 'SEARCH_AREA')}, set(self.sent(NOW)))


class SeparationTests(TrafficBase):
    def arrived(self):
        self.scout_at((2., 6.))
        self.support_at(TB5_START)
        sent = self.sent(NOW)
        self.finish(sent[(SUPPORT, 'APPROACH_TARGET')])
        self.finish(sent[(SCOUT, 'SEARCH_AREA')])
        self.support_at((3., 4.), NOW + 1)

    def test_too_close_tb6_steps_away_from_tb5(self):
        self.arrived()
        self.scout_at((3.3, 4.45), NOW + 1, grid=scout_grid(unknown=lambda x, y: y < 1.))  # 0.54 m from TB-5
        sent = self.sent(NOW + 1)
        self.assertEqual({(SCOUT, 'SEARCH_AREA')}, set(sent))
        self.assertGreaterEqual(math.dist(self.destination(sent[(SCOUT, 'SEARCH_AREA')])[:2], (3., 4.)), MIN_SEP)
        self.assertEqual('TRAFFIC_ROBOTS_TOO_CLOSE', self.core.traffic_hold.get('SCOUT'))

    def test_a_new_goal_past_a_standing_tb5_goes_around_it(self):
        # C10 (2026-10-16): the frontier pick keeps traffic_min_separation_m off a standing TB-5, so the goal cut
        # along the path is no longer the one beside TB-5 that would be held for as long as TB-5 stands.
        self.arrived()
        self.scout_at((3., 5.3), NOW + 1, grid=scout_grid(unknown=lambda x, y: y < 1.))  # next frontier: past TB-5
        sent = self.sent(NOW + 1)
        self.assertEqual({(SCOUT, 'SEARCH_AREA')}, set(sent))
        self.assertGreaterEqual(math.dist(self.destination(sent[(SCOUT, 'SEARCH_AREA')])[:2], (3., 4.)), MIN_SEP)

    def test_a_new_goal_that_close_to_tb5_is_held(self):
        # the gate itself stays (a goal that close from any other source)
        self.arrived()
        self.scout_at((3., 5.3), NOW + 1)
        self.status(NOW + 1)
        self.assertFalse(self.core._traffic_scout((3., 4.3), NOW + 1))
        self.assertEqual('TRAFFIC_ROBOTS_TOO_CLOSE', self.core.traffic_hold.get('SCOUT'))


class ParameterTests(unittest.TestCase):
    def test_typed_parameters(self):
        self.assertEqual(('STRING', 'DOUBLE', 'DOUBLE', 'DOUBLE', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[k] for k in (
                             'traffic_control', 'traffic_min_separation_m', 'traffic_path_clearance_m',
                             'traffic_yield_max_m', 'traffic_stop_timeout_s')))
        self.assertEqual(('TB5_PRIORITY', 'DISABLED'), ros_node.TRAFFIC_CONTROLS)

    def test_values_stay_text_on_the_ros_command_line(self):
        # E3 rehearsal 2026-10-14: `-p traffic_control:=OFF` is read as YAML, OFF is the boolean false, and the
        # coordinator refused to start (InvalidParameterTypeException: BOOL, expecting STRING).
        # YAML 1.1 booleans (what the ROS command line reads), checked without PyYAML: CI has no yaml module.
        yaml_booleans = {'y', 'n', 'yes', 'no', 'true', 'false', 'on', 'off'}
        for value in ros_node.TRAFFIC_CONTROLS:
            with self.subTest(value=value):
                self.assertNotIn(value.lower(), yaml_booleans)
        self.assertIn('off', yaml_booleans)   # the old name OFF, the one that failed in the rehearsal
        self.assertLessEqual({'TRAFFIC_SUPPORT_WAITING_SCOUT_STOP', 'TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD',
                              'TRAFFIC_SCOUT_YIELD_NO_SPACE', 'TRAFFIC_SCOUT_HELD_FOR_SUPPORT',
                              'TRAFFIC_ROBOTS_TOO_CLOSE', 'TRAFFIC_SUPPORT_PATH_UNKNOWN', 'TRAFFIC_SCOUT_POSE_UNKNOWN',
                              'TRAFFIC_SUPPORT_POSE_UNKNOWN', 'TRAFFIC_SCOUT_STOP_TIMEOUT'},
                             set(ros_node.TRAFFIC_REASONS))

    def test_bad_values_are_refused(self):
        frontier = dict(search_mode='FRONTIER', policy_changes=dict(executable_bases=frozenset({'FRONTIER'})))
        for changes in (dict(traffic_control='ON'), dict(traffic_min_separation_m=0.),
                        dict(traffic_path_clearance_m=0.5),  # under the minimum separation
                        dict(traffic_yield_max_m=0.), dict(traffic_stop_timeout_ns=0),
                        dict(traffic_control='TB5_PRIORITY', search_mode='AREAS', policy_changes=None)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                make_core(**(frontier | dict(traffic_control='TB5_PRIORITY', traffic_min_separation_m=MIN_SEP,
                                             traffic_path_clearance_m=CLEAR, traffic_yield_max_m=YIELD_MAX,
                                             traffic_stop_timeout_ns=STOP_TIMEOUT) | changes))


if __name__ == '__main__':
    unittest.main()


class FrontierBesideTb5Tests(unittest.TestCase):
    """A frontier whose goal TB-6 could never get (within traffic_min_separation_m of a standing TB-5) is not picked:
    another one is, and FRONTIER none names it TOO_CLOSE (시뮬레이션 2026-10-16 3회차 C10: the coordinator picked a
    frontier 0.39 m beside the parked TB-5 and held TRAFFIC_ROBOTS_TOO_CLOSE until the exploration timed out; TB-5's
    body leaves an unknown shadow in TB-6's map, so such a frontier can appear on the field too [추정])."""

    BESIDE_TB5 = staticmethod(lambda x, y: x < -0.5 and 3.4 < y < 4.6)  # TB-5 at its start (0, 4)

    def core_with(self, unknown):
        core = core_(**TrafficBase.CORE)
        core.on_map_state(map_state(SCOUT, revision=3))
        core.on_start_pose((0., 0., 0.))
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_status(status_msg(SUPPORT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(scout_grid(unknown=unknown))
        core.on_pose(scout_pose((1.2, 4.2)))
        return core

    @staticmethod
    def tick(core, now):
        core.on_status(status_msg(SCOUT, observed_at_ns=now))
        core.on_status(status_msg(SUPPORT, observed_at_ns=now))
        return [f for f in core.tick(now) if f['kind'] == 'SEARCH_AREA']

    def test_the_other_frontier_goes_instead_of_waiting(self):
        core = self.core_with(lambda x, y: x > 8. or self.BESIDE_TB5(x, y))
        sent = self.tick(core, NOW) or self.tick(core, NOW + 1)
        self.assertEqual(1, len(sent))
        x, y, _ = live_task_inputs.destinations(core)[sent[0]['search_area_ref']]['xy_yaw']
        self.assertGreaterEqual(math.dist((x, y), TB5_START), MIN_SEP)
        self.assertNotIn('TRAFFIC_ROBOTS_TOO_CLOSE', ' '.join(core.drain_notes()))

    def test_the_only_frontier_beside_tb5_is_named_too_close(self):
        core = self.core_with(self.BESIDE_TB5)
        core.frontier_reach_m = 0.6  # every goal cell near this frontier is then within the separation of TB-5
        self.assertEqual([], self.tick(core, NOW))
        line = [n for n in core.drain_notes() if n.startswith('FRONTIER none')][-1]
        self.assertIn('too_close=1', line)
        self.assertIn('TOO_CLOSE cells=', line)


class PathPastTb5Tests(unittest.TestCase):
    """The circle of traffic_min_separation_m around a standing TB-5 holds only the goal, not the way to it (11회차
    2026-10-16 C16: TB-6's way from its start to the gap passes within 0.6 m of TB-5, so every frontier came out
    TOO_CLOSE three times, EXPLORATION_COMPLETE frontiers_sent=0, and TB-6 never moved; the field layout has TB-5
    beside TB-6 at the start too [추정]). Passing TB-5 is traffic control's part (_traffic_scout), and a frontier
    left out only for TB-5 waits for the next map update instead of counting toward EXPLORATION_COMPLETE."""

    # a corridor 1.6 m wide, TB-5's body in it at its start (0, 4): the way past it is 0.5 m from its centre
    WALLS = [(-1., -1., 9., 3.2), (-1., 4.8, 9., 9.), (-.2, 3.8, .2, 4.2)]

    def core_with(self, unknown, tb6=(-0.8, 4.), walls=WALLS, **changes):
        core = core_(**TrafficBase.CORE)
        for key, value in changes.items():
            setattr(core, key, value)
        core.on_map_state(map_state(SCOUT, revision=3))
        core.on_start_pose((0., 0., 0.))
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_status(status_msg(SUPPORT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        self.grid = scout_grid(unknown=unknown, walls=walls)
        core.on_scout_map(self.grid)
        core.on_pose(scout_pose(tb6))
        return core

    def first_goal(self, core):
        sent = FrontierBesideTb5Tests.tick(core, NOW) or FrontierBesideTb5Tests.tick(core, NOW + 1)
        self.assertEqual(1, len(sent), core.drain_notes())
        return live_task_inputs.destinations(core)[sent[0]['search_area_ref']]['xy_yaw']

    def test_the_way_past_tb5_is_taken(self):
        core = self.core_with(lambda x, y: x > 8., frontier_goal_max_m=20.)
        x, y, _ = self.first_goal(core)
        self.assertGreater(x, 6.)  # the frontier's own goal, not cut
        self.assertFalse(core.exploration_complete)

    def test_a_cut_point_beside_tb5_goes_on_past_the_circle(self):
        core = self.core_with(lambda x, y: x > 8.)  # frontier_goal_max_m 1.0: the cut point is beside TB-5
        x, y, _ = self.first_goal(core)
        self.assertGreaterEqual(math.dist((x, y), TB5_START), MIN_SEP)
        self.assertGreater(x, 0.)  # past TB-5 on the way, not back

    def test_a_frontier_left_out_only_for_tb5_does_not_end_the_exploration(self):
        core = self.core_with(FrontierBesideTb5Tests.BESIDE_TB5, tb6=(1.2, 4.2), walls=(),
                              frontier_reach_m=0.6)
        for k in range(3 * core.exploration_empty_checks):
            core.on_scout_map(self.grid)
            self.assertEqual([], FrontierBesideTb5Tests.tick(core, NOW + k))
        self.assertFalse(core.exploration_complete)
        lines = [n for n in core.drain_notes() if n.startswith('FRONTIER none')]
        self.assertTrue(lines)
        self.assertTrue(all('too_close=1' in n and 'not counted' in n for n in lines), lines)


RUN11 = Path(__file__).resolve().parents[1] / 'docs/evidence/sim-2026-10-16-run050248/maps'


@unittest.skipUnless((RUN11 / '050550-before-declare.yaml').exists(),
                     'installed-wheel checkout has no docs/evidence (or the 11회차 evidence is not merged yet)')
class Run11DeclareTests(unittest.TestCase):
    """11회차 (2026-10-16, build b8a758d), the map just before the declaration: TB-6 at its start (-0.35, 0.00), TB-5
    at its start spot (-0.43, 1.80). Before: FRONTIER none three times (frontiers=5 small=2 too_close=3, the largest
    TOO_CLOSE cells=90 centroid (-1.32, 1.67)), then EXPLORATION_COMPLETE frontiers_sent=0. Field values."""

    TB6, TB5 = (-0.353, 0.0), (-0.431, 1.795)

    def test_the_90_cell_frontier_goes(self):
        core = core_(**dict(TrafficBase.CORE, support_start_pose=(*self.TB5, 0.)), frontier_min_cells=10,
                     frontier_reach_m=1.0, frontier_goal_max_m=1.5, frontier_goal_clearance_m=0.35,
                     exploration_empty_checks=3)
        core.on_map_state(map_state(SCOUT, revision=3))
        core.on_start_pose((0., 0., 0.))
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_status(status_msg(SUPPORT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(saved_map('050550-before-declare', RUN11))
        core.on_pose(scout_pose(self.TB6))
        sent = FrontierBesideTb5Tests.tick(core, NOW)
        notes = core.drain_notes()
        self.assertEqual(1, len(sent), notes)
        (line,) = [n for n in notes if n.startswith('FRONTIER ') and 'cells=' in n]
        self.assertIn('cells=90 ', line)
        x, y, _ = live_task_inputs.destinations(core)[sent[0]['search_area_ref']]['xy_yaw']
        self.assertGreaterEqual(math.dist((x, y), self.TB5), MIN_SEP)
