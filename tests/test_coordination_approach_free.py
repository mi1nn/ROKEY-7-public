"""TB-5's approach point in a free direction around the doll (15회차 2026-10-16, C23).

15회차 found the doll, froze the map and armed TB-5, then held ``SUPPORT_APPROACH_NOT_FREE`` to the end: the one
point approach_standoff_m from the doll straight toward TB-5 (-1.842, 0.774) lay in a box on the frozen map, while
other directions on the same circle were free (작전 analysis, docs/evidence/sim-2026-10-16-run073537). A doll next
to a wall gives the same in the field. Now the point is chosen on the approach_standoff_m circle: free within
freeze_clearance_m, the shortest clear grid path from TB-5 (straight-line distance where the grid has no cells).
None free: SUPPORT_APPROACH_NOT_FREE as before, and the log says how many were checked.
"""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from rescue_contracts.grid_search import path_lengths
from rescue_contracts.occupancy import footprint_clear
from rescue_coordination import ros_node
from test_coordination_approach import pose_msg
from test_coordination_frontier import saved_map, scout_pose
from test_coordination_map_freeze import CLEARANCE, map_state
from test_coordination_mission_end import RealGrid
from test_coordination_ros_core import SUPPORT
from test_coordination_traffic import scout_grid
from test_coordination_two_freezes import NOW, R1, Base, core_

DOLL, TB5 = (4., 4.), (0., 4.)  # Base: TB-5 at (0, 4), standoff 1.0 -> the straight point is (3, 4)
STRAIGHT = (3., 4.)
SIDE_BOX = [(2.4, 2.4, 3.6, 4.6)]  # over the straight point, wider below: the upper side opens nearer
# Over the straight point plus a wall that closes the lower side from TB-5: the lower side is nearer in a straight
# line (225°, 3.37 m) but its path goes round the doll (6.9 m); the upper side's path is shorter (90°, 4.58 m).
WALLED_BELOW = [(2.5, 3.6, 3.5, 5.0), (2.0, -1., 2.2, 3.8)]
AROUND = [(2.6, 2.6, 5.4, 5.4)]  # every direction occupied


class FreeDirectionBase(Base):
    WALLS = ()

    def freeze(self, revision, sha=None, grid=None):
        super().freeze(revision, grid=grid or RealGrid(self.WALLS))

    def picks(self):
        self.core.drain_notes()
        approach = self.dispatch()
        goal = approach['route']['approach_goal']
        return goal, ' '.join(self.core.drain_notes())

    def assert_free_on_circle_facing_the_doll(self, goal):
        xy = tuple(goal.mission_xy)
        self.assertAlmostEqual(1.0, math.dist(xy, DOLL), places=6)
        self.assertTrue(self.core.frozen_grid.clear(*xy, CLEARANCE))
        self.assertAlmostEqual(math.atan2(DOLL[1] - xy[1], DOLL[0] - xy[0]), goal.yaw, places=6)


class StraightBlockedTests(FreeDirectionBase):
    WALLS = SIDE_BOX

    def test_a_blocked_straight_point_goes_to_a_free_direction(self):
        self.assertFalse(self.core.frozen_grid.clear(*STRAIGHT, CLEARANCE))
        goal, notes = self.picks()
        self.assert_free_on_circle_facing_the_doll(goal)
        self.assertGreater(goal.mission_xy[1], DOLL[1])  # the side that opens nearer
        self.assertIn('APPROACH_POINT_PICKED', notes)


class PathNotLineTests(FreeDirectionBase):
    WALLS = WALLED_BELOW

    def test_the_shortest_path_wins_over_the_nearest_in_a_straight_line(self):
        goal, notes = self.picks()
        self.assert_free_on_circle_facing_the_doll(goal)
        self.assertGreater(goal.mission_xy[1], DOLL[1])
        fields = self.core.frozen_grid.fields
        picked, lower = path_lengths(**fields, radius=CLEARANCE, start=TB5,
                                     goals=[tuple(goal.mission_xy), (3.29, 3.29)])  # (3.29, 3.29): 225°
        self.assertLess(picked, lower)
        self.assertIn('path_m=', notes)


class NoneFreeTests(FreeDirectionBase):
    WALLS = AROUND

    def test_nothing_free_holds_and_says_how_many_were_checked_once(self):
        self.core.drain_notes()
        self.dispatch_none()
        self.assertEqual('SUPPORT_APPROACH_NOT_FREE', self.reason())
        notes = [n for n in self.core.drain_notes() if n.startswith('APPROACH_POINT_NONE')]
        self.assertEqual(1, len(notes))
        self.assertIn('checked=', notes[0])
        self.assertIn('free=0', notes[0])
        self.assertEqual([], self.tick(NOW + 1, 'APPROACH_TARGET'))
        self.assertEqual([], [n for n in self.core.drain_notes() if n.startswith('APPROACH_POINT_NONE')])

    def dispatch_none(self):
        self.core.on_scout_map(scout_grid())  # TB-6 exploring, so the reason shown is TB-5's
        self.core.on_pose(scout_pose((0., 0.)))
        self.core.on_map_state(map_state(SUPPORT, revision=R1))
        self.core.on_pose(pose_msg(map_revision=R1))
        self.assertEqual([], self.tick(NOW, 'APPROACH_TARGET'))


RUN15 = Path(__file__).resolve().parents[1] / 'docs/evidence/sim-2026-10-16-run073537/maps'


class Run15Tests(unittest.TestCase):
    """The 15회차 frozen map R2 with its launch values (standoff 0.8 m, clearance 0.2 m) and logged positions."""
    DOLL, TB5 = (-2.5007, 0.3200), (-0.402, 1.765)  # TARGET_REBASED_ON_FREEZE; TB-5 RobotPose 07:39:59.72

    def setUp(self):
        fields = saved_map('073955-frozen-r2', RUN15).fields
        self.core = core_(approach_standoff_m=0.8)
        self.core.frozen_grid = NS(fields=fields, clear=lambda x, y, r: footprint_clear(**fields, x=x, y=y, radius=r))
        self.core.target_geo = {'doll': (None, self.DOLL)}

    def test_the_straight_point_was_in_the_box(self):
        (xy, _), = [ros_node.approach_point(self.DOLL, self.TB5, 0.8)]
        self.assertEqual((-1.842, 0.774), tuple(round(v, 3) for v in xy))
        self.assertFalse(self.core.frozen_grid.clear(*xy, CLEARANCE))

    def test_a_free_point_on_the_circle_is_picked(self):
        xy, yaw = self.core._approach_goal('doll', self.TB5)
        self.assertAlmostEqual(0.8, math.dist(xy, self.DOLL), places=6)
        self.assertTrue(self.core.frozen_grid.clear(*xy, CLEARANCE))
        self.assertAlmostEqual(math.atan2(self.DOLL[1] - xy[1], self.DOLL[0] - xy[0]), yaw, places=6)
        (note,) = self.core.drain_notes()
        self.assertTrue(note.startswith('APPROACH_POINT_PICKED target=doll'), note)


if __name__ == '__main__':
    unittest.main()
