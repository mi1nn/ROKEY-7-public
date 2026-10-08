"""E2: robot-side final command watchdog core (C-01, TR-11). No ROS import.

Times are local monotonic seconds; all thresholds below are test-only values.
"""
import math
import unittest

from rescue_robot.executor import ContractError
from rescue_robot.watchdog import CommandWatchdog, WatchdogPolicy

POLICY = WatchdogPolicy('test-only', heartbeat_max_age_s=.5, command_max_age_s=.3,
                        max_linear_mps=.3, max_angular_rps=1.)
GO = ((.2, 0., 0.), (0., 0., .5))
ZERO = ('ZERO',)


def armed(at=0.):
    dog = CommandWatchdog(POLICY)
    dog.heartbeat(at)
    return dog


class WatchdogPolicyTests(unittest.TestCase):
    def test_every_threshold_must_be_positive_and_finite(self):
        for field in ('heartbeat_max_age_s', 'command_max_age_s', 'max_linear_mps', 'max_angular_rps'):
            for bad in (0, -1., math.inf, math.nan, '1', True):
                with self.subTest(field=field, bad=bad), self.assertRaises(ContractError):
                    WatchdogPolicy(**{**POLICY.__dict__, field: bad})
        with self.assertRaises(ContractError):
            WatchdogPolicy(**{**POLICY.__dict__, 'version': ''})


class CommandWatchdogTests(unittest.TestCase):
    def test_nothing_is_forwarded_before_the_first_executor_heartbeat(self):
        dog = CommandWatchdog(POLICY)
        self.assertEqual('WAITING', dog.state)
        self.assertEqual(ZERO, dog.command(*GO, 0.))
        self.assertFalse(dog.alive(0.))

    def test_fresh_heartbeat_arms_and_forwards_commands_within_limits(self):
        dog = armed()
        self.assertEqual('ARMED', dog.state)
        self.assertTrue(dog.alive(.1))
        self.assertEqual(('FORWARD',) + GO, dog.command(*GO, .1))

    def test_stale_executor_heartbeat_latches_stop_and_silences_own_heartbeat(self):
        dog = armed()
        dog.command(*GO, .4)
        self.assertEqual([], dog.step(.5))
        self.assertEqual([ZERO], dog.step(.51))
        self.assertEqual(('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE'), (dog.state, dog.reason))
        self.assertFalse(dog.alive(.51))
        self.assertEqual(ZERO, dog.command(*GO, .52))
        self.assertEqual([], dog.step(.6))   # zero once on the transition

    def test_command_arriving_after_heartbeat_expiry_latches_even_before_step(self):
        dog = armed()
        self.assertEqual(ZERO, dog.command(*GO, .6))
        self.assertEqual('STOP_LATCHED', dog.state)

    def test_heartbeat_recovery_never_releases_the_latch(self):
        dog = armed()
        dog.step(1.)
        dog.heartbeat(1.1)
        self.assertEqual('STOP_LATCHED', dog.state)
        self.assertEqual(ZERO, dog.command(*GO, 1.1))
        self.assertFalse(dog.alive(1.1))

    def test_explicit_reset_needs_a_fresh_heartbeat(self):
        dog = armed()
        dog.step(1.)
        self.assertFalse(dog.reset(1.2))          # heartbeat still old
        dog.heartbeat(1.3)
        self.assertTrue(dog.reset(1.4))
        self.assertEqual(('ARMED', ''), (dog.state, dog.reason))
        self.assertFalse(armed().reset(0.))       # nothing latched, nothing to reset

    def test_command_over_limit_or_not_finite_latches_instead_of_clamping(self):
        cases = (((.31, 0., 0.), (0., 0., 0.), 'COMMAND_OUT_OF_LIMIT'),
                 ((.2, .25, 0.), (0., 0., 0.), 'COMMAND_OUT_OF_LIMIT'),   # vector norm
                 ((0., 0., 0.), (0., 0., -1.01), 'COMMAND_OUT_OF_LIMIT'),
                 ((math.nan, 0., 0.), (0., 0., 0.), 'COMMAND_NOT_FINITE'))
        for linear, angular, reason in cases:
            with self.subTest(linear=linear, angular=angular):
                dog = armed()
                self.assertEqual(ZERO, dog.command(linear, angular, .1))
                self.assertEqual(('STOP_LATCHED', reason), (dog.state, dog.reason))

    def test_input_silence_after_motion_sends_one_zero_without_latching(self):
        dog = armed()
        dog.command(*GO, .1)
        dog.heartbeat(.3)
        self.assertEqual([], dog.step(.4))
        self.assertEqual([ZERO], dog.step(.41))
        self.assertEqual([], dog.step(.45))
        self.assertEqual('ARMED', dog.state)

    def test_zero_command_does_not_need_a_follow_up_zero(self):
        dog = armed()
        dog.command((0., 0., 0.), (0., 0., 0.), .1)
        self.assertEqual([], dog.step(.45))

    def test_time_going_backwards_latches(self):
        dog = armed(1.)
        self.assertEqual([ZERO], dog.step(.9))
        self.assertEqual(('STOP_LATCHED', 'CLOCK_REGRESSION'), (dog.state, dog.reason))

    def test_events_record_each_transition_with_its_reason(self):
        dog = armed()
        dog.step(1.)
        dog.heartbeat(1.1)
        dog.reset(1.1)
        self.assertEqual([('ARMED', '', 0.), ('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE', 1.),
                          ('ARMED', 'RESET', 1.1)], dog.events)


class AdapterWatchdogSourceTests(unittest.TestCase):
    def test_watchdog_flag_comes_only_from_a_configured_heartbeat_topic(self):
        from rescue_robot.ros_node import readiness_policy
        ages = {'odom': .5, 'scan': .5, 'camera': 1., 'wheel_status': .5, 'map_to_odom_tf': 1.,
                'watchdog_heartbeat': .5}
        self.assertNotIn('watchdog', readiness_policy('t', ages, (), '', '').flag_sources)
        policy = readiness_policy('t', ages, (), '', 'command_watchdog/heartbeat')
        self.assertEqual(('watchdog_heartbeat',), policy.flag_sources['watchdog'])
        self.assertEqual(.5, policy.max_age_s['watchdog_heartbeat'])


if __name__ == '__main__':
    unittest.main()
