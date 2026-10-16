"""The watchdog's start-up latch releases itself when the executor heartbeat comes back (option B; no ROS).

재범 decision 2026-10-16 11:0x 「B로 가자」 (grade 4, via 작전): run 18 (#473) needed a person to call
<ns>/rescue_command_watchdog/reset at 08:46:57 (runbook line 48) before TB-6 could be armed. The executor heartbeat had
gone stale for longer than heartbeat_max_age_s 0.5 s while the PC was starting everything, so the watchdog latched
EXECUTOR_HEARTBEAT_STALE before the robot had ever been armed.

- Start-up phase: the executor has never been ARMED and has no task. It says so on every heartbeat
  (Header.frame_id = HEARTBEAT_STARTUP; otherwise HEARTBEAT_RUN).
- In that phase only, a latch for EXECUTOR_HEARTBEAT_STALE releases itself on the next fresh heartbeat, at most
  AUTO_RESET_MAX times [제안]; event WATCHDOG_AUTO_RESET with how long the heartbeat was away.
- Once any heartbeat has said HEARTBEAT_RUN, a latch needs a person's reset as before (C-06). A heartbeat saying
  nothing (an older executor) counts as not start-up. COMMAND_OUT_OF_LIMIT, COMMAND_NOT_FINITE and
  CLOCK_REGRESSION never release themselves. heartbeat_max_age_s is unchanged.
"""
import unittest
from uuid import UUID

from rescue_robot.executor import Executor, Policy, Readiness

from test_command_watchdog import GO, POLICY, ZERO


def dog_at_start(at=0.):
    from rescue_robot.watchdog import HEARTBEAT_STARTUP, CommandWatchdog
    dog = CommandWatchdog(POLICY)
    dog.heartbeat(at, phase=HEARTBEAT_STARTUP)
    return dog


def stale_then_back(dog, phase, *, last=0., back=2.0):
    """The heartbeat stops after ``last`` (latched at the next step), then comes back at ``back``."""
    dog.step(last + POLICY.heartbeat_max_age_s + .01)
    assert (dog.state, dog.reason) == ('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE')
    dog.heartbeat(back, phase=phase)
    return back


class StartupAutoReset(unittest.TestCase):
    @unittest.expectedFailure
    def test_start_up_stale_then_back_releases_and_the_robot_can_be_armed(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        now = stale_then_back(dog, HEARTBEAT_STARTUP, back=2.0)
        self.assertEqual('ARMED', dog.state)
        self.assertTrue(dog.alive(now))   # its own heartbeat again: the executor's watchdog readiness comes back
        self.assertEqual(('FORWARD',) + GO, dog.command(*GO, now + .1))
        self.assertIn('WATCHDOG_AUTO_RESET startup reason=EXECUTOR_HEARTBEAT_STALE stale_for=2.000s',
                      dog.events[-1][1])

    @unittest.expectedFailure
    def test_after_the_robot_was_armed_a_latch_stays_until_a_person_resets(self):
        from rescue_robot.watchdog import HEARTBEAT_RUN, HEARTBEAT_STARTUP
        dog = dog_at_start()
        dog.heartbeat(.1, phase=HEARTBEAT_RUN)            # the executor was armed
        dog.heartbeat(.2, phase=HEARTBEAT_STARTUP)        # whatever it says later
        stale_then_back(dog, HEARTBEAT_STARTUP, last=.2, back=2.0)
        self.assertEqual('STOP_LATCHED', dog.state)
        self.assertTrue(dog.reset(2.1))                   # the person's reset still works

    @unittest.expectedFailure
    def test_at_most_auto_reset_max_times_then_it_stays_latched(self):
        from rescue_robot.watchdog import AUTO_RESET_MAX, HEARTBEAT_STARTUP
        dog = dog_at_start()
        last = 0.
        for _ in range(AUTO_RESET_MAX):
            last = stale_then_back(dog, HEARTBEAT_STARTUP, last=last, back=last + 2.0)
            self.assertEqual('ARMED', dog.state)
        stale_then_back(dog, HEARTBEAT_STARTUP, last=last, back=last + 2.0)
        self.assertEqual(('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE'), (dog.state, dog.reason))
        self.assertIn('WATCHDOG_AUTO_RESET_LIMIT', dog.events[-1][1])
        self.assertEqual(3, AUTO_RESET_MAX)

    @unittest.expectedFailure
    def test_other_latches_and_unmarked_heartbeats_never_release_themselves(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        self.assertEqual(ZERO, dog.command((1.0, 0., 0.), (0., 0., 0.), .1))   # over the limit
        dog.heartbeat(.2, phase=HEARTBEAT_STARTUP)
        self.assertEqual(('STOP_LATCHED', 'COMMAND_OUT_OF_LIMIT'), (dog.state, dog.reason))
        older = dog_at_start()
        stale_then_back(older, None, back=2.0)                                  # an executor that says nothing
        self.assertEqual('STOP_LATCHED', older.state)


class ExecutorSaysItsPhase(unittest.TestCase):
    @unittest.expectedFailure
    def test_start_up_until_the_first_arm_then_run(self):
        from rescue_robot.watchdog import HEARTBEAT_RUN, HEARTBEAT_STARTUP, heartbeat_phase
        e = Executor('robot_a', str(UUID(int=1)), str(UUID(int=2)), Policy('test-only', 1, 3, .2, .11, .5, .01, .01),
                     allowed_robot_ids=('robot_a',))
        self.assertEqual(HEARTBEAT_STARTUP, heartbeat_phase(e))
        for t in (0, .1, .2):
            e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertTrue(e.arm(str(UUID(int=2)), now=.2, readiness=Readiness(.2, True, True, True, True, True, True)))
        self.assertEqual(HEARTBEAT_RUN, heartbeat_phase(e))
        e.latch = 'STOP_LATCHED'                       # stopped later: still not start-up
        self.assertEqual(HEARTBEAT_RUN, heartbeat_phase(e))


if __name__ == '__main__':
    unittest.main()
