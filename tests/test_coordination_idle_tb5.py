"""TB-6 beside a TB-5 that has never had a task (18회차 2026-10-16, C30).

18회차 07:38:41 and 08:47:58: TB-5 stood at its start spot with no task, TB-6 came within traffic_min_separation_m
0.6 m of it and got a 0.3 m step aside (TRAFFIC_YIELD why=TRAFFIC_ROBOTS_TOO_CLOSE path_m=0.30), on which Nav2 spent
19 s. A TB-5 that never had a task and is not waiting to go has no path and does not move, so TB-6 is not sent aside
and not held for it; only a TB-6 goal on TB-5's body (within TB5_BODY_M) is held. The separation and the path
clearance hold again once TB-5 waits to go or has gone (작전 2026-10-16).
"""
import math
import unittest

from rescue_coordination import ros_node
from test_coordination_ros_core import SCOUT
from test_coordination_search import NOW
from test_coordination_traffic import MIN_SEP, TB5_START, TrafficBase, scout_grid


class IdleTb5Tests(TrafficBase):
    """TB-5 never sent and not localized on the frozen map yet: it stands at its start spot (0, 4)."""

    def test_tb6_beside_an_idle_tb5_explores_on_without_stepping_aside(self):
        self.scout_at((0.5, 4.))  # 0.5 m from TB-5, under traffic_min_separation_m
        sent = self.sent(NOW)
        self.assertIn((SCOUT, 'SEARCH_AREA'), sent)
        self.assertNotIn('TRAFFIC_YIELD', ' '.join(self.core.drain_notes()))
        self.assertIsNone(self.core.traffic_hold.get('SCOUT'))
        self.assertIsNone((self.core.frontier or {}).get('step_aside'))

    def test_a_goal_on_its_body_is_still_held(self):
        self.scout_at((2., 6.))
        self.status(NOW)
        self.assertFalse(self.core._traffic_scout((TB5_START[0] + ros_node.TB5_BODY_M - 0.05, TB5_START[1]), NOW))
        self.assertEqual('TRAFFIC_ROBOTS_TOO_CLOSE', self.core.traffic_hold.get('SCOUT'))

    def test_a_goal_off_its_body_goes_though_under_the_separation(self):
        self.scout_at((2., 6.))
        self.status(NOW)
        goal = (TB5_START[0] + ros_node.TB5_BODY_M + 0.05, TB5_START[1])
        self.assertLess(math.dist(goal, TB5_START), MIN_SEP)
        self.assertTrue(self.core._traffic_scout(goal, NOW))

    def test_a_frontier_beside_an_idle_tb5_is_not_left_out(self):
        # The pick keeps only TB5_BODY_M off an idle TB-5 (the goal gate above), not traffic_min_separation_m.
        self.scout_at((1.2, 4.2), grid=scout_grid(unknown=lambda x, y: x < -0.5 and 3.4 < y < 4.6))
        self.core.frontier_reach_m = 0.6
        sent = self.sent(NOW) or self.sent(NOW + 1)
        self.assertIn((SCOUT, 'SEARCH_AREA'), sent)


if __name__ == '__main__':
    unittest.main()
