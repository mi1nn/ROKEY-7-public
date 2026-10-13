"""TB-5 passage stop is reported, never silent (OPS-115/119/126/128), no ROS.

Decision 2026-10-13 (scope Q-25): TB-5 passes the passage only with 46 cm headroom or more.
Nav2's collision_monitor stops TB-5 when depth points fall in the front stop polygon (height
band up to 0.46 m); it publishes <ns>/collision_monitor_state on each change. While that
polygon holds TB-5 the executor says why: ExecuteTask feedback reason_code PASSAGE_BLOCKED,
not latched (cleared when the stop lifts). RobotStatus.stop_reason stays empty: the
coordinator ends the lease on a stop_reason, and a person walking by must not end the task.
If Nav2 then gives up (ACTION_ABORTED) within the window after such a stop, the result
terminal_reason is PASSAGE_BLOCKED (original_cause keeps the Nav2 cause).
"""
import asyncio
import unittest

from rescue_contracts import CANONICAL_CODES
from rescue_robot.executor import ContractError
from rescue_robot.live_passage import PassageMonitor, passage_settings

import test_robot_live_auto_approach as approach

PASSAGE = dict(polygon_name='PassageStop', result_window_ns=5_000_000_000)
STOP, DO_NOTHING, SLOWDOWN = 1, 0, 2


class Monitor(unittest.TestCase):
    def test_only_a_stop_by_the_passage_polygon_blocks(self):
        m = PassageMonitor(passage_settings(PASSAGE))
        self.assertFalse(m.blocked())
        m.on_state(STOP, 'PassageStop', 10)
        self.assertTrue(m.blocked())
        m.on_state(DO_NOTHING, '', 20)
        self.assertFalse(m.blocked())
        for action, polygon in ((STOP, 'FootprintApproach'), (SLOWDOWN, 'PassageStop')):
            m.on_state(action, polygon, 30)
            self.assertFalse(m.blocked())

    def test_recent_block_counts_for_the_result(self):
        m = PassageMonitor(passage_settings(PASSAGE))
        m.on_state(STOP, 'PassageStop', 1_000_000_000)
        m.on_state(DO_NOTHING, '', 2_000_000_000)          # released at 2 s
        self.assertTrue(m.blocked_within(now_ns=6_000_000_000))
        self.assertFalse(m.blocked_within(now_ns=7_500_000_000))
        m.on_state(STOP, 'PassageStop', 8_000_000_000)     # still blocked: counts however long ago it began
        self.assertTrue(m.blocked_within(now_ns=60_000_000_000))

    def test_settings_are_checked(self):
        for bad in (dict(PASSAGE, polygon_name=''), dict(PASSAGE, result_window_ns=0),
                    dict(PASSAGE, result_window_ns=1.0), {'polygon_name': 'PassageStop'}):
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                passage_settings(bad)

    def test_passage_blocked_is_a_common_code(self):
        self.assertIn('PASSAGE_BLOCKED', CANONICAL_CODES)


class Runtime(unittest.TestCase):
    def setUp(self):
        a = self.a = approach.LiveAutoApproach('test_policy_on_accepts_and_drives_to_the_approach_goal_in_the_native_frame')
        a.setUp()
        a.build(approach.POLICY, passage=dict(PASSAGE))
        self.adapter, self.g, self.f = a.adapter, a.g, a.f

    @property
    def runtime(self):
        return self.a.runtime

    def drive(self):
        self.assertEqual('accept', self.a.submit(), self.a.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)

    def finish(self, code):
        self.adapter.on_terminal(self.g.command_id, code)
        for _ in range(4):
            self.f.r.f.tick()
        report = self.runtime.server.ingress.report(self.g.command_id)
        asyncio.run(self.runtime.before_result(report, self.g))
        return self.runtime.encode_result(report, self.g).result

    def test_feedback_says_passage_blocked_only_while_held(self):
        self.assertEqual('', self.runtime.feedback_reason())
        self.drive()
        self.assertEqual('', self.runtime.feedback_reason())
        self.runtime.passage.on_state(STOP, 'PassageStop', self.adapter.wall_clock_ns())
        self.assertEqual('PASSAGE_BLOCKED', self.runtime.feedback_reason())
        self.runtime.passage.on_state(DO_NOTHING, '', self.adapter.wall_clock_ns())
        self.assertEqual('', self.runtime.feedback_reason())

    def test_no_feedback_reason_without_an_active_task(self):
        self.runtime.passage.on_state(STOP, 'PassageStop', self.adapter.wall_clock_ns())
        self.assertEqual('', self.runtime.feedback_reason())

    def test_nav2_giving_up_after_a_passage_stop_ends_passage_blocked(self):
        self.drive()
        self.runtime.passage.on_state(STOP, 'PassageStop', self.adapter.wall_clock_ns())
        result = self.finish(6)   # ABORTED
        self.assertEqual(('FAILED', 'PASSAGE_BLOCKED'), (result.outcome, result.terminal_reason))
        self.assertTrue(result.original_cause)

    def test_other_ends_keep_their_reason(self):
        self.drive()
        result = self.finish(6)
        self.assertEqual('ACTION_ABORTED', result.terminal_reason)

    def test_success_after_a_passage_stop_is_still_success(self):
        self.drive()
        self.runtime.passage.on_state(STOP, 'PassageStop', self.adapter.wall_clock_ns())
        self.runtime.passage.on_state(DO_NOTHING, '', self.adapter.wall_clock_ns())
        result = self.finish(4)
        self.assertEqual('COMPLETED', result.outcome)
        self.assertNotEqual('PASSAGE_BLOCKED', result.terminal_reason)

    def test_robot_status_stop_reason_is_not_set_by_a_passage_stop(self):
        self.drive()
        self.runtime.passage.on_state(STOP, 'PassageStop', self.adapter.wall_clock_ns())
        self.assertNotEqual('STOPPING', self.adapter.executor.state)

    def test_no_passage_settings_no_passage_monitor(self):
        self.a.build(approach.POLICY)
        self.assertIsNone(self.runtime.passage)
        self.assertEqual('', self.runtime.feedback_reason())


if __name__ == '__main__':
    unittest.main()
