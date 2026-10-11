"""Phase 1 path-evidence rule (작전 2026-10-11, #88): target tasks without a PathCandidate (E2).

MOVE_SEGMENT may be reserved without a candidate only with a ONE_SEGMENT operator
approval and its segment-end goal; REOBSERVE_TARGET only with a fixed observation
goal. APPROACH_TARGET still needs planner evidence. Nothing here asserts a path.
"""
import math
import unittest

from rescue_coordination.planning import CoordinationError, GoalPoint, Reservation
from test_coordination_search import EPOCH, MAP, BOOT, candidate, uid

SUPPORT, SCOUT = 'unit_2', 'unit_1'


def goal(**changes):
    values = dict(mission_xy=(1., 2.), yaw=.5, map_session_id=MAP, map_revision=3, transform_revision=1)
    return GoalPoint(**(values | changes))


def reservation(kind, robot=SUPPORT, **changes):
    values = dict(task_id=uid(1), target_id=uid(2), target_revision=0, robot_id=robot, generation=0,
                  authority_epoch=EPOCH, candidate=None, command_id=uid(3), robot_boot_id=BOOT[robot],
                  task_kind=kind)
    return Reservation(**(values | changes))


class TargetGoalTests(unittest.TestCase):
    def test_approved_segment_needs_no_candidate(self):
        item = reservation('MOVE_SEGMENT', goal=goal(), approval_id=uid(4))
        self.assertEqual((None, uid(4)), (item.candidate, item.approval_id))

    def test_segment_without_approval_or_goal_is_still_rejected(self):
        for changes in (dict(goal=goal()), dict(approval_id=uid(4)), {}):
            with self.subTest(changes=changes), self.assertRaises(CoordinationError):
                reservation('MOVE_SEGMENT', **changes)

    def test_reobserve_with_an_observation_goal_needs_no_candidate(self):
        self.assertEqual(goal(), reservation('REOBSERVE_TARGET', robot=SCOUT, goal=goal()).goal)

    def test_reobserve_without_an_observation_goal_is_still_rejected(self):
        with self.assertRaises(CoordinationError):
            reservation('REOBSERVE_TARGET', robot=SCOUT)
        with self.assertRaises(CoordinationError):  # an approval is not an observation goal
            reservation('REOBSERVE_TARGET', robot=SCOUT, approval_id=uid(4))

    def test_approach_still_needs_planner_evidence(self):
        with self.assertRaises(CoordinationError):
            reservation('APPROACH_TARGET', goal=goal(), approval_id=uid(4))

    def test_goal_and_approval_only_where_the_rule_allows(self):
        for kind, changes in (('SEARCH_AREA', dict(goal=goal())), ('HOLD', dict(approval_id=uid(4))),
                              ('APPROACH_TARGET', dict(candidate=candidate(), goal=goal())),
                              ('MOVE_SEGMENT', dict(candidate=candidate(), goal=goal(), approval_id=uid(4)))):
            with self.subTest(kind=kind), self.assertRaises(CoordinationError):
                reservation(kind, **changes)

    def test_goal_point_validation(self):
        for changes in (dict(mission_xy=(1., math.nan)), dict(yaw=math.inf), dict(map_session_id='x'),
                        dict(map_revision=-1), dict(transform_revision=1.0)):
            with self.subTest(changes=changes), self.assertRaises(CoordinationError):
                goal(**changes)
        with self.assertRaises(CoordinationError):
            reservation('MOVE_SEGMENT', goal=(1., 2.), approval_id=uid(4))


if __name__ == '__main__':
    unittest.main()
