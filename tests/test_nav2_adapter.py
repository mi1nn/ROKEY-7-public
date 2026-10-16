"""E2: Executor <-> Nav2 adapter with a fake action server and fake odometry (no ROS)."""
from pathlib import Path
import unittest
from uuid import UUID

from rescue_bringup.profile import load_deployment
from rescue_contracts import TaskAdmission
from rescue_robot.executor import ContractError, Executor, GoalPose, Policy, Task
from rescue_robot.nav2_adapter import CommandAdapter

ROOT = Path(__file__).resolve().parents[1]
WALL0 = 1700000000_000000000
# Test-only thresholds, not field values (T-01 is still open).
POLICY = Policy('test-only', 1, 3, .2, .11, .5, .01, .01)
CLOCK_ERROR_BOUND = 2.0  # test-only; wider than evidence_max_age to separate the two checks


def uid(n):
    return str(UUID(int=n))


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def wall(self):
        return WALL0 + round(self.now * 1e9)


class FakeNav2:
    """Records what the adapter asked of one robot's NavigateToPose server."""
    def __init__(self):
        self.goals, self.cancels = [], []

    def send_goal(self, command_id, pose):
        self.goals.append((command_id, pose))

    def cancel_goal(self, command_id):
        self.cancels.append(command_id)


class AdapterHelpers:
    """Two robots' adapters with fake Nav2 and odometry (helpers only; the tests are in AdapterTests)."""

    def setUp(self):
        self.deployment = load_deployment(ROOT / 'config' / 'deployment.example.json')
        self.ids = tuple(robot.id for robot in self.deployment.robots)
        self.clock = FakeClock()
        self.guard_ok = True
        self.a, self.nav_a = self.make(self.deployment.robots[0].id, boot=1)
        self.b, self.nav_b = self.make(self.deployment.robots[1].id, boot=11)
        self.pose = GoalPose('map', 1.0, 2.0, 0.0, uid(900), 0, 0)

    def make(self, robot_id, boot):
        executor = Executor(robot_id, uid(boot), uid(boot + 1), POLICY, allowed_robot_ids=self.ids)
        nav = FakeNav2()
        adapter = CommandAdapter(executor, nav, guards=lambda task: self.guard_ok,
                                 clock_error_bound_s=CLOCK_ERROR_BOUND,
                                 clock=self.clock, wall_clock_ns=self.clock.wall)
        return adapter, nav

    # ---- helpers --------------------------------------------------------
    def advance(self, dt=.1):
        self.clock.now = round(self.clock.now + dt, 6)

    def ready(self, *adapters):
        for adapter in adapters:
            adapter.observe_status(driver=True, sensors=True, slam=True, nav2=True, watchdog=True)
            adapter.on_dock_status(False)

    def odom(self, adapter, linear=0.0, age=0.0):
        stamp = self.clock.wall() - round(age * 1e9)
        sec, nanosec = divmod(stamp, 10**9)
        return adapter.on_odom(stamp_sec=sec, stamp_nanosec=nanosec,
                               linear=(linear, 0.0, 0.0), angular=(0.0, 0.0, 0.0))

    def still(self, adapter, count=4):
        for _ in range(count):
            self.advance()
            self.ready(adapter)
            self.odom(adapter)

    def arm(self, adapter, boot):
        self.still(adapter)
        self.assertTrue(adapter.executor.arm(uid(boot + 1), now=self.clock(),
                                             readiness=adapter.readiness(self.clock())))

    def task(self, adapter, boot, n, kind='SEARCH_AREA', generation=1, **extra):
        if kind in ('REOBSERVE_TARGET', 'APPROACH_TARGET'):
            extra.setdefault('target_id', uid(500 + n))
            extra.setdefault('target_revision', 0)
        if kind == 'SEARCH_AREA':
            extra.setdefault('search_area_ref', uid(700 + n))
        if kind not in ('HOLD', 'SPIN'):   # SPIN turns in place (C29)
            extra.setdefault('goal_pose', self.pose)
        wall = self.clock.wall()
        return Task(uid(n), uid(n + 1000), adapter.robot_id, uid(boot), uid(boot + 1), generation,
                    kind, wall, wall + 60 * 10**9, 5, 'test-only', **extra)


class AdapterTests(AdapterHelpers, unittest.TestCase):
    def send(self, adapter, nav, boot, n, kind='SEARCH_AREA', generation=1):
        self.ready(adapter)
        receipt = adapter.submit(self.task(adapter, boot, n, kind, generation))
        self.assertTrue(receipt.accepted, receipt.reason)
        adapter.step()
        return receipt

    # ---- completion conditions -------------------------------------------
    def test_full_cycle_send_accept_terminal_stop_completed(self):
        self.arm(self.a, 1)
        receipt = self.send(self.a, self.nav_a, 1, 3)
        self.assertIsInstance(receipt, TaskAdmission)
        self.assertEqual(self.nav_a.goals, [(uid(3), self.pose)])
        self.a.on_goal_response(uid(3), True)
        self.assertEqual(self.a.executor.state, 'NAVIGATING')
        self.advance()
        self.a.on_terminal(uid(3), 4)
        self.assertIsNone(self.a.result(uid(3)))  # terminal alone is not a stop
        self.still(self.a)
        result = self.a.result(uid(3))
        self.assertEqual((result.outcome, result.stop_confirmed, result.action_status),
                         ('COMPLETED', True, 'SUCCEEDED'))
        kinds = [event[0] for event in self.a.events]
        for earlier, later in (('ADMISSION', 'SEND'), ('SEND', 'GOAL_RESPONSE'),
                               ('GOAL_RESPONSE', 'TERMINAL'), ('TERMINAL', 'RESULT')):
            self.assertLess(kinds.index(earlier), kinds.index(later))

    def test_nav2_acceptance_is_not_an_admission(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        admissions = [e for e in self.a.events if e[0] == 'ADMISSION']
        self.assertIsNone(self.a.on_goal_response(uid(3), True))
        self.assertEqual([e for e in self.a.events if e[0] == 'ADMISSION'], admissions)

    def test_cancel_request_reply_terminal_and_stop_are_separate(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.advance()
        self.a.request_stop('CANCEL_USER')
        self.assertEqual(self.nav_a.cancels, [uid(3)])
        self.a.on_cancel_response(uid(3), True)
        self.still(self.a)
        self.assertIsNone(self.a.result(uid(3)))  # cancel ack + zero odom without terminal
        self.a.on_terminal(uid(3), 5)
        self.assertIsNone(self.a.result(uid(3)))
        self.still(self.a)
        result = self.a.result(uid(3))
        self.assertEqual((result.outcome, result.stop_confirmed, result.action_status),
                         ('CANCELED', True, 'CANCELED'))

    def test_no_odometry_after_cancel_is_stop_unconfirmed(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.a.request_stop('CANCEL_USER')
        self.a.on_terminal(uid(3), 5)
        self.advance(3.5)
        self.ready(self.a)
        self.a.step()
        result = self.a.result(uid(3))
        self.assertEqual((result.outcome, result.reason, result.stop_confirmed),
                         ('FAILED', 'STOP_UNCONFIRMED', False))
        self.assertEqual(self.a.executor.latch, 'FAULT_LATCHED')

    def test_hold_completes_only_after_stop_and_sends_no_goal(self):
        self.arm(self.b, 11)
        self.send(self.b, self.nav_b, 11, 3, kind='HOLD')
        self.assertEqual((self.nav_b.goals, self.nav_b.cancels), ([], []))
        self.assertIsNone(self.b.result(uid(3)))
        self.still(self.b)
        result = self.b.result(uid(3))
        self.assertEqual((result.outcome, result.reason, result.stop_confirmed),
                         ('COMPLETED', 'HOLD', True))

    def test_role_task_kinds_use_the_same_adapter(self):
        # TB1 recon kinds on robot A, TB2 rescue-support kinds on robot B; roles are data.
        for adapter, nav, boot, kinds in ((self.a, self.nav_a, 1, ('SEARCH_AREA', 'REOBSERVE_TARGET')),
                                          (self.b, self.nav_b, 11, ('HOLD', 'APPROACH_TARGET'))):
            for generation, kind in enumerate(kinds, start=1):
                self.arm(adapter, boot)
                n = 100 * boot + generation
                self.send(adapter, nav, boot, n, kind=kind, generation=generation)
                if kind != 'HOLD':
                    adapter.on_goal_response(uid(n), True)
                    adapter.on_terminal(uid(n), 4)
                self.still(adapter)
                self.assertEqual(adapter.result(uid(n)).outcome, 'COMPLETED')

    # ---- late / stale events ---------------------------------------------
    def test_late_accept_after_timeout_cancels_only_that_goal(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.advance(1.2)
        self.ready(self.a)
        self.a.step()  # acceptance timeout: cancel before any goal handle exists
        self.assertEqual(self.a.executor.state, 'STOPPING')
        self.assertEqual(self.nav_a.cancels, [uid(3)])
        self.a.on_goal_response(uid(3), True)
        # The late acceptance re-issues cancel for that goal UUID only.
        self.assertEqual(self.nav_a.cancels, [uid(3), uid(3)])

    def test_late_terminal_of_previous_generation_cannot_touch_current(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.a.on_terminal(uid(3), 4)
        self.still(self.a)
        self.assertEqual(self.a.result(uid(3)).outcome, 'COMPLETED')
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 7, generation=2)
        self.a.on_goal_response(uid(7), True)
        before = self.a.result(uid(3))
        self.assertFalse(self.a.on_terminal(uid(3), 6))        # late ABORTED for old goal
        self.a.on_cancel_response(uid(3), True)                # late cancel reply
        self.a.on_goal_response(uid(3), True)                  # late duplicate accept
        self.assertEqual(self.a.executor.state, 'NAVIGATING')
        self.assertEqual(self.a.executor.active.command_id, uid(7))
        self.assertEqual(self.a.result(uid(3)), before)
        self.assertNotIn(uid(7), self.nav_a.cancels)

    def test_stale_odometry_cannot_confirm_stop(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.a.on_terminal(uid(3), 4)
        for _ in range(6):
            self.advance()
            self.odom(self.a, age=1.0)   # older than evidence_max_age
        self.assertIsNone(self.a.result(uid(3)))
        self.assertEqual(self.a.executor.motion, 'STOP_UNCONFIRMED')

    def test_unusable_odom_stamp_resets_stop_window(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.a.on_terminal(uid(3), 4)
        self.advance()
        self.odom(self.a)
        self.advance()
        self.a.on_odom(stamp_sec=0, stamp_nanosec=0, linear=(0, 0, 0), angular=(0, 0, 0))
        self.assertIsNone(self.a.executor.stop_start)
        self.assertIn(('ODOM_UNUSABLE', None), self.a.events)

    def test_non_terminal_status_does_not_fence(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.assertFalse(self.a.on_terminal(uid(3), 2))  # EXECUTING
        self.still(self.a)
        self.assertIsNone(self.a.result(uid(3)))
        self.assertEqual(self.a.executor.motion, 'MOVING')

    # ---- two robots ---------------------------------------------------------
    def test_two_namespaces_do_not_mix_callbacks(self):
        self.arm(self.a, 1)
        self.arm(self.b, 11)
        self.send(self.a, self.nav_a, 1, 3)
        self.send(self.b, self.nav_b, 11, 5)
        self.assertEqual(([c for c, _ in self.nav_a.goals], [c for c, _ in self.nav_b.goals]),
                         ([uid(3)], [uid(5)]))
        # A's callbacks delivered to B's adapter are ignored and emit no cancel.
        self.b.on_goal_response(uid(3), True)
        self.assertFalse(self.b.on_terminal(uid(3), 4))
        self.assertEqual(self.nav_b.cancels, [])
        self.assertEqual(self.b.executor.state, 'AWAITING_ACCEPTANCE')
        self.assertEqual(self.a.executor.state, 'AWAITING_ACCEPTANCE')
        self.assertIn(('FOREIGN_GOAL_RESPONSE', uid(3)), self.b.events)
        # Odometry of a moving A does not touch B's stop evidence.
        self.a.on_goal_response(uid(3), True)
        self.b.on_goal_response(uid(5), True)
        self.b.on_terminal(uid(5), 4)
        for _ in range(4):
            self.advance()
            self.odom(self.a, linear=.4)
            self.odom(self.b)
        self.assertEqual(self.b.result(uid(5)).outcome, 'COMPLETED')
        self.assertIsNone(self.a.result(uid(3)))
        self.assertEqual(self.a.executor.motion, 'MOVING')

    def test_task_for_other_robot_is_rejected_by_executor(self):
        self.arm(self.a, 1)
        self.ready(self.a)
        wrong = Task(uid(3), uid(4), self.b.robot_id, uid(1), uid(2), 1, 'SEARCH_AREA',
                     self.clock.wall(), self.clock.wall() + 10**10, 5, 'test-only',
                     search_area_ref=uid(5), goal_pose=self.pose)
        self.assertEqual(self.a.submit(wrong).reason, 'WRONG_ROBOT')
        self.a.step()
        self.assertEqual(self.nav_a.goals, [])

    # ---- guards, readiness, inputs -----------------------------------------
    def test_guard_failure_blocks_send(self):
        self.arm(self.a, 1)
        self.ready(self.a)
        self.assertTrue(self.a.submit(self.task(self.a, 1, 3)).accepted)
        self.guard_ok = False
        self.a.step()
        self.assertEqual(self.nav_a.goals, [])
        self.assertEqual(self.a.executor.state, 'STOPPING')

    def test_missing_or_docked_readiness_is_rejected(self):
        self.arm(self.a, 1)
        self.a.on_dock_status(True)
        self.assertEqual(self.a.submit(self.task(self.a, 1, 3)).reason, 'NAV_NOT_READY')
        fresh = self.make(self.a.robot_id, boot=1)[0]
        self.assertFalse(fresh.readiness(self.clock()).valid(self.clock(), POLICY))

    def test_same_command_with_other_goal_is_conflict_and_sends_first_goal(self):
        self.arm(self.a, 1)
        self.ready(self.a)
        task = self.task(self.a, 1, 3)
        self.assertTrue(self.a.submit(task).accepted)
        other = self.task(self.a, 1, 3, goal_pose=GoalPose('map', 9.0, 9.0, 0.0, uid(900), 0, 0))
        self.assertEqual(self.a.submit(other).reason, 'ID_CONTENT_CONFLICT')
        self.a.step()
        self.assertEqual(self.nav_a.goals, [(uid(3), self.pose)])

    def test_clock_offset_beyond_bound_is_not_a_sample(self):
        self.arm(self.a, 1)
        self.send(self.a, self.nav_a, 1, 3)
        self.a.on_goal_response(uid(3), True)
        self.a.on_terminal(uid(3), 4)
        for offset in (CLOCK_ERROR_BOUND + .5, -(CLOCK_ERROR_BOUND + .5)):
            for _ in range(4):
                self.advance()
                self.odom(self.a, age=offset)
            self.assertIsNone(self.a.result(uid(3)))
        self.assertEqual(sum(e[0] == 'ODOM_UNUSABLE' for e in self.a.events), 8)
        with self.assertRaises(ContractError):
            CommandAdapter(self.a.executor, self.nav_a, guards=lambda task: True, clock_error_bound_s=0)

    def test_stale_stop_evidence_blocks_dispatch(self):
        self.arm(self.a, 1)
        self.ready(self.a)
        self.assertTrue(self.a.submit(self.task(self.a, 1, 3)).accepted)
        self.advance(1.0)   # no odometry for longer than evidence_max_age
        self.ready(self.a)
        self.a.step()
        self.assertEqual(self.nav_a.goals, [])
        self.assertEqual(self.a.executor.stop_cause, 'STOP_EVIDENCE_STALE')


class RosBoundaryImportTests(unittest.TestCase):
    def test_ros_boundary_imports_without_ros(self):
        import math
        from rescue_robot import ros_node
        x, y, z, w = ros_node.yaw_to_quaternion(math.pi / 2)
        self.assertAlmostEqual(z, math.sqrt(.5))
        self.assertAlmostEqual(w, math.sqrt(.5))
        self.assertEqual((x, y), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
