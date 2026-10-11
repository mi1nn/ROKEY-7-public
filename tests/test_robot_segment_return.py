"""E2: new task kinds obey the same generation, frame and stop boundaries."""
import unittest
from dataclasses import replace

from rescue_robot.executor import ContractError
import test_mapping_guard as fixtures

uid = fixtures.uid


class SegmentReturnTests(unittest.TestCase):
    setUp = fixtures.MappingGuardTests.setUp
    align = fixtures.MappingGuardTests.align
    tick = fixtures.MappingGuardTests.tick
    submit_and_step = fixtures.MappingGuardTests.submit_and_step
    def task(self, n=3, kind='RETURN_HOME', pose=None, **extra):
        if kind == 'MOVE_SEGMENT':
            extra.update(target_id=uid(600), target_revision=0)
        return fixtures.MappingGuardTests.task(self, n, kind, pose, **extra)

    def test_new_kinds_need_a_pose(self):
        for kind in ('MOVE_SEGMENT', 'RETURN_HOME'):
            with self.assertRaises(ContractError):
                replace(self.task(kind=kind), goal_pose=None)

    def test_segment_requires_target_and_revision(self):
        for changes in (dict(target_id=None), dict(target_revision=None), dict(target_revision=True)):
            with self.assertRaises(ContractError):
                replace(self.task(kind='MOVE_SEGMENT'), **changes)

    def test_return_has_no_target_or_search_area(self):
        for changes in (dict(target_id=uid(600)), dict(target_revision=0), dict(search_area_ref=uid(601))):
            with self.assertRaises(ContractError):
                replace(self.task(), **changes)

    def test_segment_needs_live_target_gate_before_send(self):
        self.submit_and_step(self.task(kind='MOVE_SEGMENT'))
        self.assertEqual(self.port.goals, [])
        self.assertEqual(self.adapter.executor.stop_cause, 'GUARD_INVALID')

    def test_segment_permission_loss_cancels_only_active_goal(self):
        self.gate_ok = True
        self.submit_and_step(self.task(kind='MOVE_SEGMENT'))
        self.adapter.on_goal_response(uid(3), True)
        self.gate_ok = False
        self.tick(linear=.1)
        self.adapter.step()
        self.assertEqual(self.port.cancels, [uid(3)])

    def test_return_is_bound_into_robot_map_and_waits_for_stop(self):
        self.submit_and_step(self.task())
        self.assertEqual(self.port.goals[0][1].frame_id, 'robot_a/map')
        self.assertAlmostEqual(self.port.goals[0][1].x, 1.)
        self.adapter.on_goal_response(uid(3), True)
        self.adapter.on_terminal(uid(3), 4)
        self.assertIsNone(self.adapter.result(uid(3)))
        for _ in range(4):
            self.tick()
        result = self.adapter.result(uid(3))
        self.assertEqual((result.outcome, result.stop_confirmed), ('COMPLETED', True))
        self.assertEqual(self.adapter.executor.latch, 'STOP_LATCHED')

    def test_new_kinds_reject_stale_alignment(self):
        for kind in ('MOVE_SEGMENT', 'RETURN_HOME'):
            self.setUp()
            self.gate_ok = True
            task = self.task(kind=kind)
            task = replace(task, goal_pose=replace(task.goal_pose, map_session_id=uid(801)))
            self.submit_and_step(task)
            self.assertEqual(self.port.goals, [])

    def test_duplicate_return_does_not_dispatch_twice(self):
        task = self.task()
        self.submit_and_step(task)
        self.assertTrue(self.adapter.submit(task).accepted)
        self.adapter.step()
        self.assertEqual(len(self.port.goals), 1)


if __name__ == '__main__':
    unittest.main()
