"""A latched watchdog says so (작전 OPS, 2026-10-16 run 7; no ROS). The safety behaviour does not change.

Run 7 (#416): TB-5's watchdog latched at 02:56:43 (EXECUTOR_HEARTBEAT_STALE: one executor heartbeat gap over 0.5 s
while the simulation PC was loaded), then sent no heartbeat of its own (it does so only while ARMED and fed), so
every task_arm for 12 minutes said `NAV_NOT_READY missing=['watchdog'] ... watchdog_heartbeat(719.5 s)`: the
watchdog looked absent, not latched, and the way out (its ~/reset) was nowhere in the answer.

- The executor logs EXECUTOR_STEP_GAP with the gap and the time whenever its step (which ends with its heartbeat)
  came later than the watchdog heartbeat age limit after the previous one.
- The watchdog publishes its state (WAITING / ARMED / STOP_LATCHED, reason, reset service) latched on
  <ns>/rescue/watchdog_state on every change. Its heartbeat and latch rules are unchanged.
- task_arm refusals start with `watchdog STOP_LATCHED <reason> - reset needed: ros2 service call ...` while the
  last state the executor heard is STOP_LATCHED.

The tests marked expectedFailure fail on main.
"""
import json
import unittest


class StateTopic(unittest.TestCase):
    @unittest.expectedFailure
    def test_next_to_the_heartbeat(self):
        from rescue_robot.watchdog import watchdog_state_topic
        self.assertEqual('/robot5/rescue/watchdog_state', watchdog_state_topic('/robot5/rescue/watchdog_heartbeat'))

    @unittest.expectedFailure
    def test_state_text_and_the_refusal_hint(self):
        from rescue_robot.watchdog import latched_hint, state_text
        text = state_text('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE', '/robot5/rescue_command_watchdog/reset')
        self.assertEqual({'state': 'STOP_LATCHED', 'reason': 'EXECUTOR_HEARTBEAT_STALE',
                          'reset_service': '/robot5/rescue_command_watchdog/reset'}, json.loads(text))
        self.assertEqual('watchdog STOP_LATCHED EXECUTOR_HEARTBEAT_STALE - reset needed: ros2 service call '
                         '/robot5/rescue_command_watchdog/reset std_srvs/srv/Trigger {}', latched_hint(text))
        self.assertEqual('', latched_hint(state_text('ARMED', '', '/robot5/rescue_command_watchdog/reset')))
        for bad in (None, '', 'not json', '{"state": 1}'):
            with self.subTest(bad=bad):
                self.assertEqual('', latched_hint(bad))


class StepGap(unittest.TestCase):
    @unittest.expectedFailure
    def test_a_late_step_is_reported_once_per_gap(self):
        from rescue_robot.ros_node import StepGap
        gap = StepGap(0.5)
        self.assertIsNone(gap.tick(10.0))
        self.assertIsNone(gap.tick(10.05))
        self.assertAlmostEqual(0.62, gap.tick(10.67))
        self.assertIsNone(gap.tick(10.72))


class Refusal(unittest.TestCase):
    @unittest.expectedFailure
    def test_arm_detail_puts_the_latch_first(self):
        from rescue_robot.task_site_runtime import arm_refusal_detail
        from rescue_robot.watchdog import state_text
        latched = state_text('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE', '/robot5/rescue_command_watchdog/reset')
        detail = arm_refusal_detail('NAV_NOT_READY', ['watchdog'], 'STOPPED', ['watchdog_heartbeat(719.5 s)'], latched)
        self.assertTrue(detail.startswith('watchdog STOP_LATCHED EXECUTOR_HEARTBEAT_STALE - reset needed: '), detail)
        self.assertIn("NAV_NOT_READY missing=['watchdog'] motion=STOPPED sources=['watchdog_heartbeat(719.5 s)']",
                      detail)
        self.assertEqual("NAV_NOT_READY missing=['scan'] motion=STOPPED sources=[]",
                         arm_refusal_detail('NAV_NOT_READY', ['scan'], 'STOPPED', [], None))


if __name__ == '__main__':
    unittest.main()
