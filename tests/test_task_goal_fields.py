"""Task carries C-06 Goal's search_area_ref and a goal pose (shape checks only)."""
import unittest
from dataclasses import replace
from uuid import UUID

from rescue_robot.executor import ContractError, Executor, GoalPose, Policy, Readiness, Task


def uid(n):
    return str(UUID(int=n))


POSE = GoalPose('map', 1., 2., 0., uid(6), 0, 0)


def task(kind='SEARCH_AREA', **changes):
    fields = dict(search_area_ref=uid(5) if kind == 'SEARCH_AREA' else None,
                  goal_pose=None if kind == 'HOLD' else POSE)
    if kind in ('REOBSERVE_TARGET', 'APPROACH_TARGET'):
        fields.update(target_id=uid(7), target_revision=0)
    fields.update(changes)
    return Task(uid(3), uid(4), 'robot_a', uid(1), uid(2), 1, kind, 100, 200, 2, 'test-only', **fields)


class TaskGoalFieldTests(unittest.TestCase):
    def test_kind_field_rules(self):
        for kind in ('SEARCH_AREA', 'REOBSERVE_TARGET', 'APPROACH_TARGET', 'HOLD'):
            task(kind)
        for kind, changes in (('SEARCH_AREA', dict(search_area_ref=None)),
                              ('SEARCH_AREA', dict(search_area_ref='area-1')),
                              ('APPROACH_TARGET', dict(search_area_ref=uid(5))),
                              ('HOLD', dict(search_area_ref=uid(5))),
                              ('SEARCH_AREA', dict(goal_pose=None)),
                              ('APPROACH_TARGET', dict(goal_pose=None)),
                              ('REOBSERVE_TARGET', dict(goal_pose=(1., 2., 0.))),
                              ('HOLD', dict(goal_pose=POSE))):
            with self.assertRaises(ContractError, msg=(kind, changes)):
                task(kind, **changes)

    def test_goal_pose_shape(self):
        for changes in (dict(frame_id=''), dict(x=float('nan')), dict(yaw=float('inf')), dict(y=True),
                        dict(map_session_id='session'), dict(map_session_id=uid(0)),
                        dict(map_revision=-1), dict(map_revision=True)):
            with self.assertRaises(ContractError, msg=changes):
                replace(POSE, **changes)

    def test_goal_fields_take_part_in_content_conflict(self):
        policy = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)
        executor = Executor('robot_a', uid(1), uid(2), policy, allowed_robot_ids=('robot_a', 'robot_b'))
        ready = Readiness(.2, True, True, True, True, True, True)
        for t in (0, .1, .2):
            executor.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertTrue(executor.arm(uid(2), now=.2, readiness=ready))
        original = task()
        self.assertTrue(executor.admit(original, now_wall_ns=100, now_steady=.2, readiness=ready).accepted)
        for changed in (replace(original, search_area_ref=uid(8)),
                        replace(original, goal_pose=replace(POSE, x=1.5)),
                        replace(original, goal_pose=replace(POSE, map_revision=1))):
            receipt = executor.admit(changed, now_wall_ns=100, now_steady=.2, readiness=ready)
            self.assertEqual(receipt.reason, 'ID_CONTENT_CONFLICT')
        self.assertTrue(executor.admit(original, now_wall_ns=100, now_steady=.2, readiness=ready).accepted)


if __name__ == '__main__':
    unittest.main()
