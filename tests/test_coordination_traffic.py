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
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_contracts.occupancy import occupancy_sha256
from rescue_coordination import live_task_inputs, ros_node
from test_coordination_approach import pose_msg
from test_coordination_frontier import scout_pose
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
        # The field file has no robots.<TB-5>.initial_pose_xy_yaw until it is measured; holding TB-6 then would
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

    def test_a_new_goal_that_close_to_tb5_is_held(self):
        self.arrived()
        self.scout_at((3., 5.3), NOW + 1, grid=scout_grid(unknown=lambda x, y: y < 1.))  # next goal: down to TB-5
        self.assertEqual({}, self.sent(NOW + 1))
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
