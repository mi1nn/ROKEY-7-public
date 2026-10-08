"""E2: mission->map goal binding and map/alignment revision guard on the Nav2 adapter."""
import math
import unittest
from dataclasses import replace
from uuid import UUID

from rescue_mapping import FrameAlignment, MappingError, MappingRegistry, RigidTransform
from rescue_robot.executor import Executor, GoalPose, Policy, Task
from rescue_robot.goal_binding import MissionGoalBinder
from rescue_robot.nav2_adapter import CommandAdapter

WALL0 = 1700000000_000000000
POLICY = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)  # test-only thresholds


def uid(n):
    return str(UUID(int=n))


SESSION = uid(800)


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def wall(self):
        return WALL0 + round(self.now * 1e9)


class Port:
    def __init__(self):
        self.goals, self.cancels = [], []

    def send_goal(self, command_id, pose):
        self.goals.append((command_id, pose))

    def cancel_goal(self, command_id):
        self.cancels.append(command_id)


class MappingGuardTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.registry = MappingRegistry(clock_domain_id='test-clock', alignment_authority='test-mapping',
                                        max_position_error_m=.2, max_rotation_error_rad=.1)
        self.registry.start_session('robot_a', 'robot_a/map', SESSION)
        self.align(1, RigidTransform.planar(2., 0., 0.))
        self.gate_ok = False
        self.binder = MissionGoalBinder(self.registry, 'robot_a', target_gate=lambda task: self.gate_ok,
                                        wall_clock_ns=self.clock.wall)
        executor = Executor('robot_a', uid(1), uid(2), POLICY, allowed_robot_ids=('robot_a', 'robot_b'))
        self.port = Port()
        self.adapter = CommandAdapter(executor, self.port, guards=self.binder.check, bind_goal=self.binder.bind,
                                      clock_error_bound_s=2.0, clock=self.clock, wall_clock_ns=self.clock.wall)
        for _ in range(4):
            self.tick()
        self.assertTrue(executor.arm(uid(2), now=self.clock(), readiness=self.adapter.readiness(self.clock())))

    def align(self, transform_revision, transform, revision=0):
        self.registry.set_alignment(FrameAlignment(
            'robot_a', 'robot_a/map', 'mission', SESSION, revision, transform_revision, 'test-mapping',
            'test-clock', transform, WALL0, WALL0 + 10**12, .01, .01))

    def tick(self, linear=0.0):
        self.clock.now = round(self.clock.now + .1, 6)
        self.adapter.observe_status(driver=True, sensors=True, slam=True, nav2=True, watchdog=True)
        self.adapter.on_dock_status(False)
        sec, nanosec = divmod(self.clock.wall(), 10**9)
        self.adapter.on_odom(stamp_sec=sec, stamp_nanosec=nanosec, linear=(linear, 0, 0), angular=(0, 0, 0))

    def task(self, n=3, kind='SEARCH_AREA', pose=None, **extra):
        if kind != 'HOLD':
            extra['goal_pose'] = pose or GoalPose('mission', 3., 0., 0., SESSION, 0, 1)
        if kind == 'SEARCH_AREA':
            extra['search_area_ref'] = uid(700)
        if kind in ('REOBSERVE_TARGET', 'APPROACH_TARGET'):
            extra.update(target_id=uid(600), target_revision=0)
        wall = self.clock.wall()
        return Task(uid(n), uid(n + 1000), 'robot_a', uid(1), uid(2), 1, kind, wall,
                    wall + 60 * 10**9, 5, 'test-only', **extra)

    def submit_and_step(self, task):
        receipt = self.adapter.submit(task)
        self.assertTrue(receipt.accepted, receipt.reason)
        self.adapter.step()
        return receipt

    def test_mission_goal_is_sent_in_robot_map_frame(self):
        task = self.task()
        self.submit_and_step(task)
        (command_id, sent), = self.port.goals
        self.assertEqual((command_id, sent.frame_id), (uid(3), 'robot_a/map'))
        self.assertAlmostEqual(sent.x, 1.0)
        self.assertAlmostEqual(sent.y, 0.0)
        self.assertEqual(self.adapter.executor.active, task)  # Task itself unchanged
        self.assertEqual(self.adapter.submit(task).accepted, True)  # same receipt, no conflict

    def test_heading_is_rotated_into_map(self):
        self.align(2, RigidTransform.planar(0., 0., math.pi / 2))
        self.submit_and_step(self.task(pose=GoalPose('mission', 0., 1., math.pi / 2, SESSION, 0, 2)))
        (_, sent), = self.port.goals
        self.assertAlmostEqual(sent.x, 1.0)
        self.assertAlmostEqual(sent.y, 0.0, places=9)
        self.assertAlmostEqual(sent.yaw, 0.0, places=9)

    def test_stale_goal_version_or_frame_sends_nothing(self):
        for n, pose in ((3, GoalPose('mission', 3., 0., 0., SESSION, 0, 0)),     # old alignment
                        (4, GoalPose('mission', 3., 0., 0., uid(801), 0, 1)),   # other session
                        (5, GoalPose('robot_a/map', 3., 0., 0., SESSION, 0, 1))):  # not mission
            self.setUp()
            self.adapter.submit(self.task(n, pose=pose))
            self.adapter.step()
            self.assertEqual(self.port.goals, [], pose)
            self.assertEqual(self.adapter.executor.stop_cause, 'GUARD_INVALID')
            self.assertTrue(self.binder.last_reason.endswith('FRAME_REVISION_MISMATCH')
                            or self.binder.last_reason.endswith('INVALID_INPUT'), self.binder.last_reason)

    def test_alignment_change_while_driving_cancels_and_needs_stop(self):
        self.submit_and_step(self.task())
        self.adapter.on_goal_response(uid(3), True)
        self.align(2, RigidTransform.planar(2.5, 0., 0.))   # new transform_revision
        self.tick(linear=.3)
        self.adapter.step()
        self.assertEqual(self.port.cancels, [uid(3)])
        self.assertEqual(self.adapter.executor.stop_cause, 'GUARD_INVALID')
        self.adapter.on_terminal(uid(3), 5)
        for _ in range(4):
            self.tick()
        result = self.adapter.result(uid(3))
        self.assertEqual((result.outcome, result.stop_confirmed), ('FAILED', True))

    def test_map_revision_advance_blocks_dispatch(self):
        self.registry.advance_map_revision('robot_a', SESSION, 1)  # explicit correction input
        self.submit_and_step(self.task())
        self.assertEqual(self.port.goals, [])

    def test_target_kinds_need_the_gate(self):
        for kind in ('REOBSERVE_TARGET', 'APPROACH_TARGET'):
            self.setUp()
            self.submit_and_step(self.task(kind=kind))
            self.assertEqual(self.port.goals, [], kind)
            self.assertTrue(self.binder.last_reason.endswith('NOT_AUTHORIZED'))
        self.setUp()
        self.gate_ok = True
        self.submit_and_step(self.task(kind='APPROACH_TARGET'))
        self.assertEqual(len(self.port.goals), 1)

    def test_hold_needs_no_alignment(self):
        self.registry.invalidate_alignment('robot_a')
        self.submit_and_step(self.task(kind='HOLD'))
        for _ in range(4):
            self.tick()
        self.assertEqual(self.adapter.result(uid(3)).outcome, 'COMPLETED')
        self.assertEqual(self.port.goals, [])

    def test_current_alignment_is_read_only(self):
        before = (self.registry.map_state('robot_a'), dict(self.registry._alignments),
                  dict(self.registry._projections), set(self.registry._invalid_goals))
        alignment = self.registry.current_alignment('robot_a', now_ns=self.clock.wall())
        self.assertEqual(alignment.transform_revision, 1)
        after = (self.registry.map_state('robot_a'), dict(self.registry._alignments),
                 dict(self.registry._projections), set(self.registry._invalid_goals))
        self.assertEqual(before, after)
        self.registry.invalidate_alignment('robot_a')
        with self.assertRaises(MappingError):
            self.registry.current_alignment('robot_a', now_ns=self.clock.wall())

    def test_binder_rejects_other_robot_task(self):
        other = replace(self.task(), robot_id='robot_b')
        self.assertFalse(self.binder.check(other))
        self.assertTrue(self.binder.last_reason.endswith('WRONG_ROBOT'))


if __name__ == '__main__':
    unittest.main()
