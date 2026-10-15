"""TB-6 explores the whole map by frontiers (owner decision 2026-10-13 night; 작전 OPS-154·156; E2).

``search_mode=FRONTIER``: from TB-6's live SLAM grid and its pose the coordinator finds frontier clusters
(rescue_contracts.grid_search, 통합-인식 #241), keeps those of at least ``frontier_min_cells`` whose goal is not
near a frontier already tried, and picks the one with the shortest clear grid path from TB-6. The goal is cut to
``frontier_goal_max_m`` along that path so TB-6's tasks stay short. It becomes one SEARCH_AREA site (UUID ref,
search basis) in ~/task_inputs first and the task follows ``frontier_issue_delay_ns`` later, after the robot has
read the destination. A frontier tried once (completed or not) is not picked again within
``frontier_blacklist_m``; a goal the robot never started (NOT_STARTED_REASONS) is not a try.
``exploration_empty_checks`` map updates in a row with nothing to pick, or
``exploration_timeout_ns`` since the first frontier, is EXPLORATION_COMPLETE (EXPLORATION_TIMEOUT shown).
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import live_task_inputs, ros_node
from test_coordination_ros_core import SCOUT, admission_msg, frame, make_core, request, site_msg, status_msg
from test_coordination_search import MAP, NOW, uid

S = 1_000_000_000
DELAY = 2 * S
RES, W, H = 0.2, 40, 20  # 8 m x 4 m


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
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)  # reached: not picked again (a failed one is picked again once, FrontierRetryTests)
        for k in range(2):  # two map updates in a row with nothing left to pick
            self.core.on_scout_map(grid())
            self.core.on_pose(scout_pose(at=NOW + DELAY + k + 1))
            self.assertEqual([], self.searches(NOW + DELAY + k + 1))
            self.assertEqual(k + 1, self.core.frontier_empty)
        self.assertTrue(self.core.exploration_complete)
        self.assertEqual('EXPLORATION_COMPLETE', self.reason(NOW + DELAY + 3))
        self.assertIn('EXPLORATION_COMPLETE', ' '.join(self.core.drain_notes()))

    def test_a_completed_frontier_counts_as_tried_too(self):
        # A frontier still there after TB-6 reached it cannot be cleared from there; it is not chased forever.
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(at=NOW + DELAY + 1))
        self.assertEqual([], self.searches(NOW + DELAY + 1))
        self.assertEqual((None, 1), (self.core.frontier, self.core.frontier_empty))

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
        self.assertLessEqual(math.dist(old, xy), GOAL_CLEARANCE - 0.2 + FINE * math.sqrt(2))  # pushed, not re-planned
        self.assertIn('FRONTIER_GOAL_MOVED', ' '.join(core.drain_notes()))

    def test_the_same_clearance_changes_nothing(self):
        grid = self.wall_room()
        _, old = self.core_with(grid, (0.4, 0.3), frontier_goal_clearance_m=0.2)
        self.assertAlmostEqual(0.25, wall_distance(grid, old), delta=0.01)  # the goal of main before this change

    def test_a_narrow_way_still_gets_a_goal_and_says_so(self):
        # 0.70 m between two walls (tunnel 1 gap 0.68 m), unknown beyond x = 2.5
        grid = fine_grid(3.0, 0.8, lambda x, y: 100 if y < 0.05 or y > 0.75 else (-1 if x > 2.5 else 0))
        core, xy = self.core_with(grid, (0.4, 0.4), frontier_goal_clearance_m=GOAL_CLEARANCE)
        self.assertGreater(xy[0], 1.0)  # a goal along the way, not refused
        self.assertIn('FRONTIER_GOAL_TIGHT', ' '.join(core.drain_notes()))

    def test_a_moved_goal_that_failed_is_picked_and_moved_again_then_given_up(self):
        # C3 with C9: the failures count by the frontier (its goal cell and centroid), not by the moved site
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
        self.assertEqual(2, notes.count('FRONTIER_GOAL_MOVED'))

    def test_the_clearance_is_a_parameter(self):
        self.assertEqual('DOUBLE', ros_node.PARAMETERS['frontier_goal_clearance_m'])
        with self.assertRaises(RescueError):
            frontier_core(frontier_goal_clearance_m=-0.1)


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
        self.core.on_pose(scout_pose())
        self.searches()
        (fields,) = self.searches(NOW + DELAY)
        self.finish(fields)  # reached once: tried
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose(at=NOW + DELAY + 1))
        self.searches(NOW + DELAY + 1)
        line = self.none_line()
        self.assertIn('frontiers=1 small=0 no_goal=0 tried=1 no_path=0', line)
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

    def test_bad_values_are_refused(self):
        for changes in (dict(policy_changes=dict(executable_bases=frozenset({'PRIOR_MAP'}))),  # FRONTIER not executable
                        dict(frontier_min_cells=0), dict(frontier_goal_max_m=0.), dict(exploration_empty_checks=0),
                        dict(exploration_timeout_ns=0), dict(frontier_issue_delay_ns=-1)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                frontier_core(**changes)


if __name__ == '__main__':
    unittest.main()
