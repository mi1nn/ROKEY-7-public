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
        self.finish(fields, 'UNREACHABLE')
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

    def test_a_goal_nav2_took_and_aborted_is_still_tried(self):
        self.reject(self.first_goal(), 'ACTION_ABORTED')
        self.assertEqual(2, len(self.core.frontier_tried))
        self.assertNotEqual('SCOUT_GOAL_NOT_STARTED', self.reason(NOW + DELAY))


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
