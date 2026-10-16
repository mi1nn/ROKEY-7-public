"""The watchdog's start-up latch releases itself when the executor heartbeat comes back (option B; no ROS).

재범 decision 2026-10-16 11:0x 「B로 가자」 (grade 4, via 작전): run 18 (#473) needed a person to call
<ns>/rescue_command_watchdog/reset at 08:46:57 (runbook line 48) before TB-6 could be armed. The executor heartbeat had
gone stale for longer than heartbeat_max_age_s 0.5 s while the PC was starting everything, so the watchdog latched
EXECUTOR_HEARTBEAT_STALE before the robot had ever been armed.

- Start-up phase: the executor has never been ARMED and has no task. It says so on every heartbeat
  (Header.frame_id = HEARTBEAT_STARTUP; otherwise HEARTBEAT_RUN).
- In that phase only, a latch for EXECUTOR_HEARTBEAT_STALE releases itself once start-up heartbeats have kept coming
  for heartbeat_max_age_s (REV-22: not on the first one, see QueuedHeartbeatsAcrossTheArm), at most AUTO_RESET_MAX
  times [제안]; event WATCHDOG_AUTO_RESET with how long the heartbeat was away.
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
    """The heartbeat stops after ``last`` (latched at the next step), then comes back at ``back`` and keeps coming at
    20 Hz for heartbeat_max_age_s (0.5 s; a start-up release waits that long, REV-22). Returns its last time."""
    dog.step(last + POLICY.heartbeat_max_age_s + .01)
    assert (dog.state, dog.reason) == ('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE')
    for k in range(11):
        dog.heartbeat(back + k / 20, phase=phase)
    return back + 10 / 20


class StartupAutoReset(unittest.TestCase):
    def test_start_up_stale_then_back_releases_and_the_robot_can_be_armed(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        now = stale_then_back(dog, HEARTBEAT_STARTUP, back=2.0)
        self.assertEqual('ARMED', dog.state)
        self.assertTrue(dog.alive(now))   # its own heartbeat again: the executor's watchdog readiness comes back
        self.assertEqual(('FORWARD',) + GO, dog.command(*GO, now + .1))
        self.assertIn('WATCHDOG_AUTO_RESET startup reason=EXECUTOR_HEARTBEAT_STALE stale_for=2.000s',
                      dog.events[-1][1])

    def test_after_the_robot_was_armed_a_latch_stays_until_a_person_resets(self):
        from rescue_robot.watchdog import HEARTBEAT_RUN, HEARTBEAT_STARTUP
        dog = dog_at_start()
        dog.heartbeat(.1, phase=HEARTBEAT_RUN)            # the executor was armed
        dog.heartbeat(.2, phase=HEARTBEAT_STARTUP)        # whatever it says later
        now = stale_then_back(dog, HEARTBEAT_STARTUP, last=.2, back=2.0)
        self.assertEqual('STOP_LATCHED', dog.state)
        self.assertTrue(dog.reset(now + .1))              # the person's reset still works

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

    def test_other_latches_and_unmarked_heartbeats_never_release_themselves(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        self.assertEqual(ZERO, dog.command((1.0, 0., 0.), (0., 0., 0.), .1))   # over the limit
        dog.heartbeat(.2, phase=HEARTBEAT_STARTUP)
        self.assertEqual(('STOP_LATCHED', 'COMMAND_OUT_OF_LIMIT'), (dog.state, dog.reason))
        older = dog_at_start()
        stale_then_back(older, None, back=2.0)                                  # an executor that says nothing
        self.assertEqual('STOP_LATCHED', older.state)



class QueuedHeartbeatsAcrossTheArm(unittest.TestCase):
    """REV-22 (#476): the watchdog process got no CPU for over heartbeat_max_age_s while the executor armed.

    Jazzy rclpy (ros:jazzy, watchdog_node as launched) then runs the overdue step first (latch
    EXECUTOR_HEARTBEAT_STALE) and takes the queued heartbeats one by one afterwards: a 'startup' sent before the arm,
    then the 'run' ones. Released on that first 'startup', 44-48 commands of 0.2 m/s went through in 8 of 9 runs
    with the arm in the last 0.5 s of the stall; main kept the latch.
    """

    def test_a_start_up_heartbeat_queued_before_the_arm_does_not_release_the_latch(self):
        from rescue_robot.watchdog import HEARTBEAT_RUN, HEARTBEAT_STARTUP
        dog = dog_at_start()
        dog.heartbeat(.05, phase=HEARTBEAT_STARTUP)   # then no CPU for this process until 1.0
        dog.step(1.0)
        self.assertEqual(('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE'), (dog.state, dog.reason))
        dog.heartbeat(1.001, phase=HEARTBEAT_STARTUP)   # sent before the arm
        self.assertEqual(ZERO, dog.command(*GO, 1.002))   # Nav2, after the arm
        dog.heartbeat(1.003, phase=HEARTBEAT_RUN)
        for k in range(1, 21):                             # fresh 'run' heartbeats at 20 Hz
            dog.heartbeat(1.003 + k / 20, phase=HEARTBEAT_RUN)
            self.assertEqual(ZERO, dog.command(*GO, 1.004 + k / 20))
        self.assertEqual(('STOP_LATCHED', 'EXECUTOR_HEARTBEAT_STALE'), (dog.state, dog.reason))
        self.assertNotIn('WATCHDOG_AUTO_RESET', ' '.join(reason for _, reason, _ in dog.events))

    def test_release_waits_for_heartbeat_max_age_of_start_up_heartbeats(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        dog.step(POLICY.heartbeat_max_age_s + .01)
        for k in range(10):                                # 2.0 .. 2.45: not yet
            dog.heartbeat(2.0 + k / 20, phase=HEARTBEAT_STARTUP)
            self.assertEqual('STOP_LATCHED', dog.state)
        dog.heartbeat(2.0 + 10 / 20, phase=HEARTBEAT_STARTUP)
        self.assertEqual('ARMED', dog.state)
        self.assertIn('stale_for=2.000s', dog.events[-1][1])   # how long the heartbeat was away

    def test_a_new_gap_while_waiting_starts_the_wait_again(self):
        from rescue_robot.watchdog import HEARTBEAT_STARTUP
        dog = dog_at_start()
        dog.step(POLICY.heartbeat_max_age_s + .01)
        dog.heartbeat(2.0, phase=HEARTBEAT_STARTUP)
        dog.heartbeat(2.0 + POLICY.heartbeat_max_age_s + .1, phase=HEARTBEAT_STARTUP)   # away again
        self.assertEqual('STOP_LATCHED', dog.state)
        dog.heartbeat(2.0 + 2 * POLICY.heartbeat_max_age_s + .05, phase=HEARTBEAT_STARTUP)
        self.assertEqual('STOP_LATCHED', dog.state)


class ExecutorSaysItsPhase(unittest.TestCase):
    def test_start_up_until_the_first_arm_then_run(self):
        from rescue_robot.watchdog import HEARTBEAT_RUN, HEARTBEAT_STARTUP, heartbeat_phase
        e = Executor('robot_a', str(UUID(int=1)), str(UUID(int=2)), Policy('test-only', 1, 3, .2, .11, .5, .01, .01),
                     allowed_robot_ids=('robot_a', 'robot_b'))
        self.assertEqual(HEARTBEAT_STARTUP, heartbeat_phase(e))
        for t in (0, .1, .2):
            e.observe_motion(sample_steady=t, received_steady=t, linear=0, angular=0)
        self.assertTrue(e.arm(str(UUID(int=2)), now=.2, readiness=Readiness(.2, True, True, True, True, True, True)))
        self.assertEqual(HEARTBEAT_RUN, heartbeat_phase(e))
        e.latch = 'STOP_LATCHED'                       # stopped later: still not start-up
        self.assertEqual(HEARTBEAT_RUN, heartbeat_phase(e))


if __name__ == '__main__':
    unittest.main()
