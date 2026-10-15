"""TB-6 explores the whole map by frontiers (owner decision 2026-10-13 night; 작전 OPS-154·156; E2).

``search_mode=FRONTIER``: from TB-6's live SLAM grid and its pose the coordinator finds frontier clusters
(rescue_contracts.grid_search, 통합-인식 #241), keeps those of at least ``frontier_min_cells`` whose goal is not
near a frontier already tried, and picks the one with the lowest explore_lite cost (grid path length from TB-6
against frontier size, FrontierGainTests). The goal is cut to ``frontier_goal_max_m`` along that path so TB-6's
tasks stay short. It becomes one SEARCH_AREA site (UUID ref, search basis) in ~/task_inputs first and the task
follows ``frontier_issue_delay_ns`` later, after the robot has read the destination. A frontier whose own goal cell
TB-6 reached is not picked again within ``frontier_blacklist_m`` (a cut goal is a step, CutGoalTests); a failed one
is picked again once (FrontierRetryTests); a goal the robot never started (NOT_STARTED_REASONS) is not a try.
``exploration_empty_checks`` map updates in a row with nothing to pick, or
``exploration_timeout_ns`` since the first frontier, is EXPLORATION_COMPLETE (EXPLORATION_TIMEOUT shown).
"""
import math
from pathlib import Path
import re
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import live_task_inputs, ros_node
from test_coordination_ros_core import (SCOUT, admission_msg, frame, location, make_core, request, site_msg,
                                        status_msg)
from test_coordination_search import MAP, NOW, uid

S = 1_000_000_000
DELAY = 2 * S
RES, W, H = 0.2, 40, 20  # 8 m x 4 m
NEAR = (3.2, 1.8)  # the frontier's goal cell within frontier_goal_max_m: a goal there is not cut (C11)


def grid(known_cols=20):
    """Free on the left ``known_cols`` columns, unknown to the right: one frontier along that edge."""
    data = [0 if ix < known_cols else -1 for iy in range(H) for ix in range(W)]
    return NS(fields=dict(frame='map', width=W, height=H, resolution=RES, origin=(0., 0., 0.), data=data))


def scout_pose(xy=(1., 1.8), *, at=NOW, age_ns=10**8, status='VALID', **frame_changes):
    return NS(robot_id=SCOUT, mission_pose_status=status, frame_ref=frame(robot_id=SCOUT, **frame_changes),
              time=NS(observed_at_ns=at - age_ns),
              mission_pose=NS(pose=NS(pose=NS(position=NS(x=xy[0], y=xy[1], z=0.)))))


def frontier_core(**changes):
    values = dict(search_mode='FRONTIER', frontier_min_cells=3, frontier_reach_m=1.0, frontier_goal_max_m=1.0,
                  frontier_blacklist_m=0.5, frontier_issue_delay_ns=DELAY, exploration_empty_checks=2,
                  exploration_timeout_ns=600 * S,
                  policy_changes=dict(executable_bases=frozenset({'FRONTIER'})))
    return make_core(**(values | changes))


class Base(unittest.TestCase):
    def setUp(self):
        self.core = frontier_core()
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SCOUT))
        self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)

    def searches(self, now=NOW):
        self.core.on_status(status_msg(SCOUT, observed_at_ns=now - 10**8))
        return [f for f in self.core.tick(now) if f['kind'] == 'SEARCH_AREA']

    def reason(self, now=NOW):
        self.core.system_state_change(now)
        return self.core.last_state['reason']

    def finish(self, fields, outcome='COMPLETED'):
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True, terminal_reason=''))


class FrontierPickTests(Base):
    def test_no_map_or_no_pose_no_goal(self):
        self.assertEqual(([], 'EXPLORATION_MAP_UNKNOWN'), (self.searches(), self.reason()))
        self.core.on_scout_map(grid())
        self.assertEqual(([], 'SCOUT_POSE_UNKNOWN'), (self.searches(), self.reason()))

    def test_the_destination_goes_first_then_the_task_short_of_the_frontier(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        self.assertEqual([], self.searches())  # the robot reads ~/task_inputs first
        (site_id,) = [k for k in live_task_inputs.destinations(self.core)]
        point = live_task_inputs.destinations(self.core)[site_id]
        self.assertEqual((SCOUT, MAP, 3), (point['robot_id'], point['map_session_id'], point['map_revision']))
        x, y, yaw = point['xy_yaw']
        self.assertLessEqual(math.dist((1., 1.8), (x, y)), 1.0 + RES)  # cut to frontier_goal_max_m
        self.assertGreater(x, 1.5)                                     # towards the frontier at x = 4 m
        self.assertAlmostEqual(0.0, yaw, places=6)
        self.assertEqual([], self.searches(NOW + DELAY - 1))
        (fields,) = self.searches(NOW + DELAY)
        self.assertEqual(site_id, fields['search_area_ref'])

    def test_a_tried_frontier_is_not_picked_again_and_exploration_completes(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR))
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)  # reached: not picked again (a failed one is picked again once, FrontierRetryTests)
        for k in range(2):  # two map updates in a row with nothing left to pick
            self.core.on_scout_map(grid())
            self.core.on_pose(scout_pose(NEAR, at=NOW + DELAY + k + 1))
            self.assertEqual([], self.searches(NOW + DELAY + k + 1))
            self.assertEqual(k + 1, self.core.frontier_empty)
        self.assertTrue(self.core.exploration_complete)
        self.assertEqual('EXPLORATION_COMPLETE', self.reason(NOW + DELAY + 3))
        self.assertIn('EXPLORATION_COMPLETE', ' '.join(self.core.drain_notes()))

    def test_the_same_map_is_counted_once(self):
        self.core.on_scout_map(grid(known_cols=W))  # all known: no frontier
        self.core.on_pose(scout_pose())
        for k in range(5):
            self.searches(NOW + k)
        self.assertFalse(self.core.exploration_complete)  # one map, one check
        self.core.on_scout_map(grid(known_cols=W))
        self.searches(NOW + 6)
        self.assertTrue(self.core.exploration_complete)

    def test_the_timeout_ends_the_exploration_with_its_reason(self):
        self.core = frontier_core(exploration_timeout_ns=10 * S)
        self.setUp_after()
        self.searches()
        self.assertEqual([], self.searches(NOW + 10 * S + 1))
        self.assertTrue(self.core.exploration_complete)
        self.assertEqual('EXPLORATION_TIMEOUT', self.reason(NOW + 10 * S + 1))

    def setUp_after(self):
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SCOUT))
        self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())

    def test_a_pose_on_another_basis_is_not_used(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(map_revision=9))
        self.assertEqual(([], 'SCOUT_POSE_UNKNOWN'), (self.searches(), self.reason()))


class RobotNotStartedTests(Base):
    """A frontier goal the robot never started (Nav2 refused it, no acceptance in time, stale stop evidence) is
    not a tried frontier: TB-6 did not go there. It is picked again and the reason shows TB-6 did not start
    (현장 2026-10-15 17:54: bt_navigator inactive, then EXPLORATION_COMPLETE and NO_SURVIVOR_FOUND)."""

    def reject(self, fields, cause='GOAL_REJECTED'):
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='FAILED', stop_confirmed=True, terminal_reason=cause))

    def first_goal(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        return fields

    def test_a_rejected_first_goal_does_not_end_the_exploration(self):
        self.reject(self.first_goal())
        for k in range(3):  # TB-6's SLAM updates its map while it stands
            self.core.on_scout_map(grid())
            self.core.on_pose(scout_pose(at=NOW + DELAY + k + 1))
            self.searches(NOW + DELAY + k + 1)
        self.assertFalse(self.core.exploration_complete)
        self.assertEqual(0, self.core.frontier_empty)
        self.assertEqual([], self.core.frontier_tried)
        self.assertEqual('SCOUT_GOAL_NOT_STARTED', self.reason(NOW + DELAY + 4))
        self.assertIn('FRONTIER_NOT_STARTED', ' '.join(self.core.drain_notes()))

    def test_the_same_frontier_goes_again_after_the_delay(self):
        first = self.first_goal()
        self.reject(first)
        later = NOW + DELAY + 1
        self.core.on_pose(scout_pose(at=later))
        self.assertEqual([], self.searches(later))  # picked again: its destination goes first
        self.core.on_pose(scout_pose(at=later + DELAY))
        (again,) = self.searches(later + DELAY)
        self.assertNotEqual(first['task_id'], again['task_id'])
        self.core.on_admission(admission_msg(again))
        self.assertEqual('', self.reason(later + DELAY))  # the robot took the new goal

    def test_every_not_started_cause(self):
        for cause in ('GOAL_REJECTED', 'ACCEPT_TIMEOUT', 'STOP_EVIDENCE_STALE'):
            with self.subTest(cause=cause):
                self.setUp()
                self.reject(self.first_goal(), cause)
                self.assertEqual([], self.core.frontier_tried)

    def test_a_goal_nav2_took_and_aborted_counts_as_a_failed_attempt(self):
        self.reject(self.first_goal(), 'ACTION_ABORTED')
        self.assertEqual(([], 1), (self.core.frontier_tried, len(self.core.frontier_failures)))  # retried once (C9)
        # the reason is in the log, not only the outcome (RPP "collision ahead" ends as ACTION_ABORTED, 2026-10-15)
        self.assertIn('outcome=FAILED reason=ACTION_ABORTED', ' '.join(self.core.drain_notes()))
        self.assertNotEqual('SCOUT_GOAL_NOT_STARTED', self.reason(NOW + DELAY))


class FrontierRetryTests(Base):
    """A frontier Nav2 took and that then failed is picked again once: it is tried only after its second failure
    (재범 decision 2026-10-16, C9: the one way into the main zone failed once and the exploration ended). Untried
    frontiers go first. An exploration that ends with such frontiers left says so (EXPLORATION_FRONTIERS_FAILED)
    instead of a plain EXPLORATION_COMPLETE."""

    def attempt(self, now):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(at=now))
        self.searches(now)
        self.core.on_pose(scout_pose(at=now + DELAY))
        (fields,) = self.searches(now + DELAY)
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='FAILED', stop_confirmed=True,
                                                  terminal_reason='ACTION_ABORTED'))
        return fields

    def test_the_only_frontier_is_picked_again_after_one_failure(self):
        first = self.attempt(NOW)
        self.assertEqual([], self.core.frontier_tried)
        self.assertIn('FRONTIER_RETRY', ' '.join(self.core.drain_notes()))
        second = self.attempt(NOW + 3 * DELAY)
        self.assertNotEqual(first['search_area_ref'], second['search_area_ref'])  # a new site, same frontier

    def test_after_the_second_failure_it_is_tried_and_the_end_says_so(self):
        self.attempt(NOW)
        self.attempt(NOW + 3 * DELAY)
        self.assertEqual(2, len(self.core.frontier_tried))
        for k in range(2):  # exploration_empty_checks = 2
            self.core.on_scout_map(grid())
            self.core.on_pose(scout_pose(at=NOW + 6 * DELAY + k))
            self.assertEqual([], self.searches(NOW + 6 * DELAY + k))
        self.assertTrue(self.core.exploration_complete)
        self.assertEqual('EXPLORATION_FRONTIERS_FAILED', self.reason(NOW + 6 * DELAY + 2))
        notes = ' '.join(self.core.drain_notes())
        self.assertIn('EXPLORATION_COMPLETE reason=EXPLORATION_FRONTIERS_FAILED', notes)
        self.assertIn('failed_frontiers=1', notes)

    def test_no_doll_after_given_up_frontiers_is_not_shown_as_no_survivor(self):
        # SLAM_FREEZE with no doll would show NO_SURVIVOR_FOUND ("생존자 없음 — 탐색 완료"): not a whole map here
        self.core.map_mode, self.core.exploration_complete = 'SLAM_FREEZE', True
        self.core.exploration_reason = 'EXPLORATION_FRONTIERS_FAILED'
        self.assertEqual('EXPLORATION_FRONTIERS_FAILED', self.core.mission_reason(NOW))
        self.core.exploration_reason = 'EXPLORATION_COMPLETE'
        self.assertEqual('NO_SURVIVOR_FOUND', self.core.mission_reason(NOW))


FINE = 0.05  # TB-6 SLAM map cell (live map yaml resolution 0.05, docs/evidence/sim-2026-10-15-run232037/maps)
# The goal must leave TB-6 room to stop and turn in place without RPP's or the shim's collision check (footprint cost
# LETHAL) firing: robot 0.189 + local costmap cell half diagonal 0.042 + one noise cell 0.06 + pose error 0.05
# = 0.341 m from the obstacle surface (PR body table). frontier_goal_clearance_m 0.35 gives that: a clear cell's
# nearest occupied 0.05 m cell centre is over 0.35 + 0.035 away, its face over 0.36.
ROOM, GOAL_CLEARANCE = 0.341, 0.35


def fine_grid(width_m, height_m, cell):
    """``cell(x, y)`` -> 100 / -1 / 0 at each cell centre, FINE resolution."""
    w, h = round(width_m / FINE), round(height_m / FINE)
    data = [cell((ix + .5) * FINE, (iy + .5) * FINE) for iy in range(h) for ix in range(w)]
    return NS(fields=dict(frame='map', width=w, height=h, resolution=FINE, origin=(0., 0., 0.), data=data))


def wall_distance(grid, xy):
    f = grid.fields
    return min(math.dist(xy, ((i % f['width'] + .5) * FINE, (i // f['width'] + .5) * FINE))
               for i, v in enumerate(f['data']) if v == 100)


class WaitingForArmTests(Base):
    """After a FAILED task the executor does not arm again by itself (OPS-231: only after COMPLETED), so RobotStatus
    is NOT_READY until someone runs task_arm. The next frontier then waits; the reason and the log say why
    (시뮬레이션 2026-10-15 23:03-23:35: it waited with an empty reason until EXPLORATION_TIMEOUT)."""

    def test_the_next_frontier_waits_with_a_reason(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='FAILED', stop_confirmed=True,
                                                  terminal_reason='ACTION_ABORTED'))
        self.core.on_scout_map(grid(known_cols=10))  # a new map with another frontier
        later = NOW + 3 * DELAY
        self.core.on_pose(scout_pose((1., 1.8), at=later))
        not_ready = dict(readiness='NOT_READY', stop_reason='ACTION_ABORTED', observed_at_ns=later - 10**8)
        for now in (later, later + DELAY + 1):
            self.core.on_status(status_msg(SCOUT, **not_ready))
            self.assertEqual([], [f for f in self.core.tick(now) if f['kind'] == 'SEARCH_AREA'])
        self.assertEqual('SCOUT_NOT_READY', self.reason(later + DELAY + 1))
        notes = ' '.join(self.core.drain_notes())
        self.assertIn('SCOUT_NOT_READY', notes)
        self.assertIn('stop_reason=ACTION_ABORTED', notes)
        self.assertEqual(1, notes.count('SCOUT_NOT_READY'))  # logged once, not every tick
        self.core.on_status(status_msg(SCOUT, observed_at_ns=later + DELAY + 2))  # armed again
        self.assertEqual(1, len([f for f in self.core.tick(later + DELAY + 2) if f['kind'] == 'SEARCH_AREA']))


class WallClearanceTests(unittest.TestCase):
    """The frontier goal TB-6 gets leaves it room to stop and turn in place (재범 decision 2026-10-16, C3; 1·2회차:
    a goal 0.20 m from the wall cell ended FAILED after RPP "detected collision ahead!"). The path itself keeps
    ``freeze_clearance_m``, so a narrow way (the 0.68 m gap beside tunnel 1) still counts as reachable."""

    def core_with(self, grid, start, **changes):
        core = frontier_core(**changes)
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(grid)
        core.on_pose(scout_pose(start))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW - 10**8))
        core.tick(NOW)
        (point,) = live_task_inputs.destinations(core).values()
        return core, tuple(point['xy_yaw'][:2])

    @staticmethod
    def wall_room():
        """South wall (two cell rows), free room, an unknown pocket in the south-east corner."""
        return fine_grid(4.0, 2.0, lambda x, y: 100 if y < 0.1 else (-1 if x > 2.0 and y < 0.5 else 0))

    def test_the_goal_beside_a_wall_leaves_room_to_turn(self):
        grid = self.wall_room()
        _, old = self.core_with(grid, (0.4, 0.3), frontier_goal_clearance_m=0.2)  # = freeze_clearance_m
        self.assertLess(wall_distance(grid, old) - FINE / 2, ROOM)              # 0.225 m to the wall face
        core, xy = self.core_with(grid, (0.4, 0.3), frontier_goal_clearance_m=GOAL_CLEARANCE)
        self.assertGreaterEqual(wall_distance(grid, xy) - FINE / 2, ROOM)

    def test_the_same_clearance_changes_nothing(self):
        grid = self.wall_room()
        _, old = self.core_with(grid, (0.4, 0.3), frontier_goal_clearance_m=0.2)
        self.assertAlmostEqual(0.25, wall_distance(grid, old), delta=0.01)  # the goal of main before this change

    def test_a_roomy_goal_that_failed_is_picked_again_at_the_same_place_then_given_up(self):
        # C3 with C9: the failures count by the frontier (its goal cell and centroid) and by the place (C12)
        grid = self.wall_room()
        core, first = self.core_with(grid, (0.4, 0.3), frontier_goal_clearance_m=GOAL_CLEARANCE)
        for attempt in (1, 2):
            core.on_status(status_msg(SCOUT, observed_at_ns=NOW + attempt * 10 * DELAY - 10**8))
            core.on_pose(scout_pose((0.4, 0.3), at=NOW + attempt * 10 * DELAY))
            (fields,) = [f for f in core.tick(NOW + attempt * 10 * DELAY) if f['kind'] == 'SEARCH_AREA']
            xy = tuple(live_task_inputs.destinations(core)[fields['search_area_ref']]['xy_yaw'][:2])
            self.assertEqual(first, xy)                                   # the same moved goal both times
            self.assertGreaterEqual(wall_distance(grid, xy) - FINE / 2, ROOM)
            core.on_admission(admission_msg(fields))
            core.on_result(fields['task_id'], NS(outcome='FAILED', stop_confirmed=True,
                                                 terminal_reason='ACTION_ABORTED'))
            core.on_scout_map(grid)
            core.tick(NOW + attempt * 10 * DELAY + 1)                     # picks it again after the 1st failure
        notes = ' '.join(core.drain_notes())
        self.assertIn('FRONTIER_RETRY', notes)
        self.assertIn('FRONTIER_GAVE_UP', notes)

    def test_the_clearance_is_a_parameter(self):
        self.assertEqual('DOUBLE', ros_node.PARAMETERS['frontier_goal_clearance_m'])
        with self.assertRaises(RescueError):
            frontier_core(frontier_goal_clearance_m=-0.1)


class TightGapTests(unittest.TestCase):
    """No goal in a place narrower than ``frontier_goal_clearance_m`` (7회차 2026-10-16, C12: four frontiers past the
    0.68 m gap beside tunnel 1 were all cut to the same point in the gap, sent at 0.2 m clear as FRONTIER_GOAL_TIGHT,
    and all EXPIRED with RPP "detected collision ahead"). The frontier's goal is the cell that clear within
    ``frontier_reach_m`` of it with the shortest path, past the gap too; a cut point in a tight part of the path goes
    on to the first point past it that clear; a frontier with no such cell is left out (TIGHT) for another one; a
    place that failed FRONTIER_ATTEMPTS times is given up whichever frontier sends TB-6 there."""

    @staticmethod
    def gap_room(zones='E'):
        """6.0 m x 3.0 m with walls round it; a 0.3 m partition at x 2.0-2.3 with a 0.65 m gap at y 1.2-1.85
        (tunnel 1 gap 0.68 m; 0.05 m cells); the unknown zone beyond x = 4.0 (``E``), or two of them south and
        north of a known strip at y 1.0-2.0 (``SN``)."""
        def cell(x, y):
            if y < 0.1 or y > 2.9 or x < 0.1 or x > 5.9 or (2.0 < x < 2.3 and not 1.2 < y < 1.85):
                return 100
            if x > 4.0 and (zones == 'E' or not 1.0 < y < 2.0):
                return -1
            return 0
        return fine_grid(6.0, 3.0, cell)

    def core_with(self, grid, start):
        core = frontier_core(frontier_goal_clearance_m=GOAL_CLEARANCE)
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(grid)
        core.on_pose(scout_pose(start))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW - 10**8))
        core.tick(NOW)
        return core

    @staticmethod
    def site(core):
        (point,) = live_task_inputs.destinations(core).values()
        return tuple(point['xy_yaw'][:2])

    def test_a_cut_point_in_the_gap_goes_on_past_it(self):
        grid = self.gap_room()
        core = self.core_with(grid, (1.2, 1.5))  # 1.0 m along the path is in the gap
        xy = self.site(core)
        self.assertGreater(xy[0], 2.3)  # through the gap, not short of it
        self.assertGreaterEqual(wall_distance(grid, xy) - FINE / 2, ROOM)
        self.assertNotIn('FRONTIER_GOAL_TIGHT', ' '.join(core.drain_notes()))

    def test_a_frontier_with_no_roomy_cell_is_left_out_for_another(self):
        # a 0.70 m corridor north of the room, unknown 1.1 m in (the lower cost); a small unknown corner in the
        # south-east
        def cell(x, y):
            if y < 0.1 or x < 0.1 or x > 3.9 or (y > 1.9 and not 0.65 < x < 1.35) or y > 3.9:
                return 100
            if y > 3.0 or (x > 3.6 and y < 0.3):
                return -1
            return 0
        grid = fine_grid(4.0, 4.0, cell)
        core = self.core_with(grid, (1.0, 1.0))
        xy = self.site(core)
        self.assertGreater(xy[0], 1.5)  # the south-east corner, not the corridor
        self.assertGreaterEqual(wall_distance(grid, xy) - FINE / 2, ROOM)
        self.assertIn('FRONTIER_GOAL_TIGHT', ' '.join(core.drain_notes()))

    def test_only_tight_frontiers_send_nothing_and_say_tight(self):
        grid = fine_grid(3.0, 0.8, lambda x, y: 100 if y < 0.05 or y > 0.75 else (-1 if x > 2.5 else 0))
        core = self.core_with(grid, (0.4, 0.4))
        self.assertEqual({}, live_task_inputs.destinations(core))
        line = [n for n in core.drain_notes() if n.startswith('FRONTIER none')][-1]
        self.assertIn('tight=1', line)
        self.assertIn('TIGHT cells=', line)

    def test_a_place_that_failed_twice_is_given_up_for_every_frontier(self):
        grid = self.gap_room('SN')
        core = self.core_with(grid, (1.2, 1.5))
        sent = []
        for k in range(3):
            now = NOW + (k + 1) * 10 * DELAY
            core.on_status(status_msg(SCOUT, observed_at_ns=now - 10**8))
            core.on_pose(scout_pose((1.2, 1.5), at=now))
            searches = [f for f in core.tick(now) if f['kind'] == 'SEARCH_AREA']
            if not searches:
                break
            (fields,) = searches
            sent.append(tuple(live_task_inputs.destinations(core)[fields['search_area_ref']]['xy_yaw'][:2]))
            core.on_admission(admission_msg(fields))
            core.on_result(fields['task_id'], NS(outcome='EXPIRED', stop_confirmed=True,
                                                 terminal_reason='DEADLINE_EXPIRED'))
            core.on_scout_map(grid)
            core.tick(now + 1)
        self.assertEqual(2, len(sent))  # two frontiers, one place: not a third time
        self.assertLessEqual(math.dist(*sent), 0.5)


RUN9 = Path(__file__).resolve().parents[1] / 'docs/evidence/sim-2026-10-16-run035105/maps'


def saved_map(name):
    """A map_saver trinary map (pgm 0 occupied, 254 free, 205 unknown) as a live grid."""
    meta = (RUN9 / f'{name}.yaml').read_text()
    resolution = float(re.search(r'resolution: ([0-9.e-]+)', meta).group(1))
    ox, oy = map(float, re.search(r'origin: \[([^,]+), ([^,]+),', meta).groups())
    _, size, _, pixels = (RUN9 / f'{name}.pgm').read_bytes().split(b'\n', 3)
    w, h = map(int, size.split())
    data = [(-1 if p == 205 else 100 if p < 100 else 0)
            for p in (pixels[(h - 1 - iy) * w + ix] for iy in range(h) for ix in range(w))]
    return NS(fields=dict(frame='map', width=w, height=h, resolution=resolution, origin=(ox, oy, 0.), data=data))


def sees_unknown(grid, xy, degrees):
    """Straight lines on the map from ``xy`` at these headings: the ones that reach an unknown cell before an
    occupied one (one leaving the map counts as nothing)."""
    f = grid.fields
    hits = []
    for a in degrees:
        for k in range(1, 1200):
            x, y = xy[0] + k * 0.01 * math.cos(math.radians(a)), xy[1] + k * 0.01 * math.sin(math.radians(a))
            ix = math.floor((x - f['origin'][0]) / f['resolution'])
            iy = math.floor((y - f['origin'][1]) / f['resolution'])
            if not (0 <= ix < f['width'] and 0 <= iy < f['height']):
                break
            if f['data'][iy * f['width'] + ix] == -1:
                hits.append(a)
                break
            if f['data'][iy * f['width'] + ix] == 100:
                break
    return hits


@unittest.skipUnless((RUN9 / '035525-live-r0-6cf8199b4ae7.yaml').exists(),
                     'installed-wheel checkout has no docs/evidence')
class GapMouthTests(unittest.TestCase):
    """The frontier goal is the roomy cell nearest the frontier, not the one with the shortest path (9회차
    2026-10-16, C14). Its map (035525) is the start zone only; the one frontier (13 cells, centroid (-1.84, 2.20))
    is the 0.68 m gap between the partition end and the north wall, opening on a passage that turns west. #420
    sent TB-6 to (-1.19, 1.45), 0.30 m of path away and slanting to the gap: nothing new was in sight, the goal was
    not cut, so the frontier was tried and the exploration ended at two frontiers outside the main zone (8회차, before
    #420, went to the gap mouth and on). Field values: frontier_min_cells 10, goal 1.5 m, reach 1.0 m, 0.35 m clear."""

    START = (-1.17, 1.17)  # TB-6 at its first goal, as at the second pick (coordinator.log 44)
    WEST = [150 + a / 2 for a in range(121)]  # 150-210 degrees, through the gap

    def core(self):
        core = frontier_core(frontier_goal_clearance_m=GOAL_CLEARANCE, frontier_min_cells=10, frontier_goal_max_m=1.5)
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(saved_map('035525-live-r0-6cf8199b4ae7'))
        core.on_pose(scout_pose(self.START))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW - 10**8))
        core.tick(NOW)
        return core

    def test_the_goal_faces_the_gap_and_sees_past_it(self):
        core = self.core()
        (point,) = live_task_inputs.destinations(core).values()
        xy = tuple(point['xy_yaw'][:2])
        self.assertLess(math.dist(xy, (-1.29, 2.15)), 0.05)  # nearest roomy cell to the centroid, 0.56 m from it
        grid = saved_map('035525-live-r0-6cf8199b4ae7')
        self.assertGreater(len(sees_unknown(grid, xy, self.WEST)), 60)  # west through the gap: unknown in sight

    def test_from_the_old_goal_nothing_unknown_was_in_sight(self):
        # what TB-6 saw from the #420 goal: why the frontier stayed (calculated on the same map)
        grid = saved_map('035525-live-r0-6cf8199b4ae7')
        self.assertEqual([], sees_unknown(grid, (-1.19, 1.45), range(360)))

    def test_a_frontier_left_after_its_goal_is_logged(self):
        core = self.core()
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW + DELAY - 10**8))
        core.on_pose(scout_pose(self.START, at=NOW + DELAY))
        (fields,) = [f for f in core.tick(NOW + DELAY) if f['kind'] == 'SEARCH_AREA']
        core.on_admission(admission_msg(fields))
        core.on_result(fields['task_id'], NS(outcome='COMPLETED', stop_confirmed=True,
                                             terminal_reason='NAVIGATION_SUCCEEDED'))
        (done,) = [n for n in core.drain_notes() if n.startswith('FRONTIER_DONE')]
        self.assertIn('goal_at=(-1.29, 2.15) frontier_remains=True', done)


class FrontierGainTests(unittest.TestCase):
    """A large unexplored zone goes before a small nook nearby (재범 2026-10-16, 5회차 02:23: TB-6 turned to the
    side its LiDAR already saw). The pick ranks by the explore_lite cost (potential_scale x path length (m) -
    gain_scale x frontier size (cells)) x resolution, the repository explore.yaml 3.0 / 1.0, as frontierCost in
    src/vendor/m-explore-ros2 writes it; untried before retried first."""

    def test_the_5th_run_candidates(self):
        # 5회차 2026-10-16 02:23, the map before the declaration (#403 README 「선포 순간 후보」): cells and distance
        # from TB-6. The coordinator took the 14-cell corner; the cost puts the large ones first.
        core = frontier_core()
        candidates = {'east of TB-5': (32, 2.24), 'tunnel 1 entrance': (31, 2.63), 'south-west corner': (14, 0.78),
                      'north': (10, 2.51), 'west': (10, 1.29)}
        cost = {name: core._cost(ros_node.Frontier(cells, (0., 0.), None), distance, dict(resolution=0.05))
                for name, (cells, distance) in candidates.items()}
        self.assertAlmostEqual(-1.16, cost['tunnel 1 entrance'], places=2)   # 3 x 2.63 x 0.05 - 31 x 0.05
        self.assertAlmostEqual(-0.58, cost['south-west corner'], places=2)   # 3 x 0.78 x 0.05 - 14 x 0.05
        self.assertEqual(['east of TB-5', 'tunnel 1 entrance', 'south-west corner'], sorted(cost, key=cost.get)[:3])

    @staticmethod
    def room():
        """3.6 m x 3.0 m: walls south, north and west, a 0.2 m x 0.2 m nook in the west wall (12 frontier cells),
        the unexplored zone beyond x = 3.0 (56 frontier cells)."""
        def cell(x, y):
            if y < 0.1 or y > 2.9 or x < 0.1:
                return 100
            if x > 3.0 or (x < 0.3 and 1.15 < y < 1.35):
                return -1
            return 0
        return fine_grid(3.6, 3.0, cell)

    def picked(self, start):
        core = frontier_core()
        core.on_site_event(site_msg())
        core.on_status(status_msg(SCOUT))
        core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        core.on_scout_map(self.room())
        core.on_pose(scout_pose(start))
        core.on_status(status_msg(SCOUT, observed_at_ns=NOW - 10**8))
        core.tick(NOW)
        return core.frontier['centroid']

    def test_the_large_zone_a_little_farther_goes_before_the_near_nook(self):
        # from (1.4, 1.5): the nook goal 0.97 m of path (12 cells), the zone goal 1.42 m (56 cells)
        x, _ = self.picked((1.4, 1.5))
        self.assertGreater(x, 2.9)  # the zone beyond x = 3.0, not the nook at x 0.24

    def test_the_size_decides_even_when_the_zone_is_farther(self):
        # from (1.2, 1.5): nook 0.77 m, zone 1.62 m; 3 x 0.85 = 2.55 path cells' worth against 44 cells more
        x, _ = self.picked((1.2, 1.5))
        self.assertGreater(x, 2.9)

    def test_the_distance_splits_frontiers_of_one_size(self):
        core = frontier_core()
        near, far = (core._cost(ros_node.Frontier(20, (0., 0.), None), m, dict(resolution=0.05)) for m in (1.0, 2.0))
        self.assertAlmostEqual(0.15, far - near)  # 3 x 1 m x 0.05


class CutGoalTests(Base):
    """A goal cut to ``frontier_goal_max_m`` on the way to a frontier is a step, not a visit (5회차 2026-10-16
    02:25, C11: TB-6 COMPLETED a goal 1.4 m short of the frontier a116, a116 was marked tried while it stayed on the
    map, and three empty map updates ended the exploration outside the main zone). Only a goal at the frontier's own
    goal cell (within ``frontier_reach_m`` of it) makes it tried."""

    def test_after_a_cut_goal_the_same_frontier_goes_on(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        site = live_task_inputs.destinations(self.core)[fields['search_area_ref']]['xy_yaw'][:2]
        self.assertLess(site[0], 3.0)  # cut: 1 m along a 2.7 m path to the frontier at x = 4 m
        self.finish(fields)
        self.core.on_scout_map(grid())  # the frontier is still there
        self.core.on_pose(scout_pose(tuple(site), at=NOW + DELAY + 1))
        self.searches(NOW + DELAY + 1)
        self.core.on_pose(scout_pose(tuple(site), at=NOW + 3 * DELAY))
        (again,) = self.searches(NOW + 3 * DELAY)
        nxt = live_task_inputs.destinations(self.core)[again['search_area_ref']]['xy_yaw'][:2]
        self.assertGreater(nxt[0], site[0])  # on towards the same frontier
        self.assertFalse(self.core.exploration_complete)
        self.assertIn('FRONTIER_GOAL_CUT', ' '.join(self.core.drain_notes()))

    def test_a_goal_at_the_frontier_still_makes_it_tried(self):
        # A frontier still there after TB-6 reached it cannot be cleared from there; it is not chased forever.
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR))
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR, at=NOW + DELAY + 1))
        self.assertEqual([], self.searches(NOW + DELAY + 1))
        self.assertEqual((None, 1), (self.core.frontier, self.core.frontier_empty))


class LookTurnTests(Base):
    """At a frontier's own goal TB-6 turns in place once round, ``frontier_look_turns`` goals of 90 degrees each at the
    same spot, so the camera sweeps for the doll (재범 decision 2026-10-16 05:0x, camera sweep option 1: 「목표마다
    제자리 한 바퀴(90°×4)」). Not at a cut goal (a step on the way). A failed turn ends the turn and counts nothing;
    a doll report lets the turn in progress finish and sends no more."""

    def setUp(self):
        self.core = frontier_core(frontier_look_turns=4)
        self.setUp_after()

    setUp_after = FrontierPickTests.setUp_after

    def at_the_frontier(self):
        self.core.on_pose(scout_pose(NEAR))
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        point = live_task_inputs.destinations(self.core)[fields['search_area_ref']]['xy_yaw']
        self.finish(fields)
        return point

    def turn(self, k):
        """The k-th turn goal (1-based): its fields and (x, y, yaw); each one after the issue delay."""
        now = NOW + (2 * k + 1) * DELAY
        self.core.on_pose(scout_pose(NEAR, at=now))
        self.searches(now)
        (fields,) = self.searches(now + DELAY)
        return fields, live_task_inputs.destinations(self.core)[fields['search_area_ref']]['xy_yaw']

    def test_four_turns_of_90_degrees_at_the_same_spot_then_the_frontier_is_tried(self):
        x, y, yaw = self.at_the_frontier()
        for k in range(1, 5):
            fields, (tx, ty, tyaw) = self.turn(k)
            self.assertEqual((x, y), (tx, ty))
            self.assertAlmostEqual(0., math.remainder(tyaw - yaw - k * math.pi / 2, 2 * math.pi), places=6)
            self.finish(fields)
        self.assertNotIn(fields['search_area_ref'], {self.core.frontier and self.core.frontier['site_id']})
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR, at=NOW + 11 * DELAY))
        self.assertEqual([], self.searches(NOW + 11 * DELAY))  # no fifth turn; the frontier is tried
        self.assertEqual(1, self.core.frontier_empty)
        notes = ' '.join(self.core.drain_notes())
        self.assertIn('FRONTIER_LOOK 4/4', notes)
        self.assertIn('FRONTIER_LOOK_DONE', notes)

    def test_no_turn_at_a_cut_goal(self):
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        site = live_task_inputs.destinations(self.core)[fields['search_area_ref']]['xy_yaw']
        self.finish(fields)
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(tuple(site[:2]), at=NOW + 3 * DELAY))
        self.searches(NOW + 3 * DELAY)
        (again,) = self.searches(NOW + 4 * DELAY)
        nxt = live_task_inputs.destinations(self.core)[again['search_area_ref']]['xy_yaw']
        self.assertGreater(nxt[0], site[0])  # on towards the frontier, no turn in place
        self.assertNotIn('FRONTIER_LOOK', ' '.join(self.core.drain_notes()))

    def test_a_failed_turn_ends_the_turning_and_counts_nothing(self):
        self.at_the_frontier()
        fields, _ = self.turn(1)
        self.finish(fields, outcome='FAILED')
        self.assertEqual([], self.core.frontier_failures)  # not a failed attempt at the frontier
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR, at=NOW + 5 * DELAY))
        self.assertEqual([], self.searches(NOW + 5 * DELAY))  # no second turn; the frontier is tried as reached
        self.assertIn('FRONTIER_LOOK_DONE', ' '.join(self.core.drain_notes()))

    def test_a_doll_report_lets_the_turn_finish_and_sends_no_more(self):
        self.at_the_frontier()
        fields, _ = self.turn(1)
        self.core.on_target_location(location(1), now_ns=NOW + 4 * DELAY)
        self.assertEqual([], self.core.cancels)  # the turn in progress is not stopped
        self.finish(fields)
        self.core.on_pose(scout_pose(NEAR, at=NOW + 5 * DELAY))
        self.searches(NOW + 5 * DELAY)
        self.assertEqual([], self.searches(NOW + 6 * DELAY))
        self.assertNotIn('FRONTIER_LOOK 2/4', ' '.join(self.core.drain_notes()))

    def test_zero_turns_is_the_old_way(self):
        self.core = frontier_core(frontier_look_turns=0)
        self.setUp_after()
        self.at_the_frontier()
        self.assertIsNone(self.core.frontier)  # tried at once


class FrontierNoneWhyTests(Base):
    """``FRONTIER none`` says why each frontier was left out, largest first (시뮬레이션 2026-10-15 C9: two frontiers
    FAILED, then three empty map updates ended the exploration and nobody could tell why the main zone was not
    picked)."""

    def none_line(self):
        lines = [n for n in self.core.drain_notes() if n.startswith('FRONTIER none')]
        self.assertTrue(lines)
        return lines[-1]

    def test_a_tried_frontier_is_named_as_tried(self):
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR))
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)  # reached once: tried
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(NEAR, at=NOW + DELAY + 1))
        self.searches(NOW + DELAY + 1)
        line = self.none_line()
        self.assertIn('frontiers=1 small=0 no_goal=0 tried=1 tight=0 no_path=0', line)
        self.assertIn('TRIED cells=20 centroid=(3.90, 2.00)', line)

    def test_an_unreachable_and_a_small_frontier_are_named(self):
        data = []
        for iy in range(H):
            for ix in range(W):
                if ix == 20:
                    data.append(100)          # a wall across the map: the right part cannot be reached
                elif ix > 30 or (ix == 0 and iy == 0):
                    data.append(-1)           # unknown beyond x = 6 m, and a 1-cell nook (2 frontier cells) at the corner
                else:
                    data.append(0)
        self.core.on_scout_map(NS(fields=dict(frame='map', width=W, height=H, resolution=RES, origin=(0., 0., 0.),
                                              data=data)))
        self.core.on_pose(scout_pose())
        self.searches()
        line = self.none_line()
        self.assertIn('small=1', line)
        self.assertIn('no_path=1', line)
        self.assertIn('NO_PATH cells=20', line)


class GoalPickTests(unittest.TestCase):
    """The RViz search goal pick (#132) replaces the areas: only AREAS uses it. FRONTIER/PATROL keep it off even
    when the field gives the topic (a FRONTIER core has no area, the pick would refuse to start; OPS-174)."""

    def test_only_areas_uses_the_rviz_goal_pick(self):
        self.assertIsNone(live_task_inputs.goal_pick_topic(frontier_core(), '/rescue/search_goal'))
        self.assertEqual('/rescue/search_goal', live_task_inputs.goal_pick_topic(make_core(), '/rescue/search_goal'))
        self.assertIsNone(live_task_inputs.goal_pick_topic(make_core(), ''))


class ParameterTests(unittest.TestCase):
    def test_typed_parameters(self):
        self.assertEqual(('INTEGER', 'DOUBLE', 'DOUBLE', 'DOUBLE', 'DOUBLE', 'INTEGER', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[k] for k in (
                             'frontier_min_cells', 'frontier_reach_m', 'frontier_goal_max_m', 'frontier_blacklist_m',
                             'frontier_issue_delay_s', 'exploration_empty_checks', 'exploration_timeout_s')))
        self.assertIn('FRONTIER', ros_node.SEARCH_MODES)
        self.assertEqual('INTEGER', ros_node.PARAMETERS['frontier_look_turns'])

    def test_bad_values_are_refused(self):
        for changes in (dict(policy_changes=dict(executable_bases=frozenset({'PRIOR_MAP'}))),  # FRONTIER not executable
                        dict(frontier_min_cells=0), dict(frontier_goal_max_m=0.), dict(exploration_empty_checks=0),
                        dict(exploration_timeout_ns=0), dict(frontier_issue_delay_ns=-1),
                        dict(frontier_look_turns=-1), dict(frontier_look_turns=1.5)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                frontier_core(**changes)


if __name__ == '__main__':
    unittest.main()
