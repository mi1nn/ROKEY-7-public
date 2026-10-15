"""A failed return goes again (재범 decision 2026-10-16 05:3x, 「2번까지 다시 보내기」; 11회차 C17).

TB-6's RETURN_HOME and TB-5's exit RETURN_HOME that end FAILED go again after the refusal hold
(``admission_timeout_ns``), until ``return_home_attempts`` attempts failed; then, as before (#194, #441), it is
shown (SCOUT_RETURN_FAILED / SUPPORT_EXIT_FAILED) for a person to handle. While it waits, SystemState says the robot
is still returning, and the log says which attempt failed. Only FAILED: a CANCELED one (a stop) never goes again.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_coordination import ros_node
import test_coordination_mission_end as mission_end  # its test classes are not collected here
from test_coordination_ros_core import SCOUT, SUPPORT, admission_msg, status_msg
from test_coordination_search import NOW
from test_coordination_approach import pose_msg
from test_coordination_two_freezes import R2, Base

S = 1_000_000_000
HOLD = 3 * S  # admission_timeout_ns of the test core (test_coordination_ros_core ACCEPT)


def fail(core, fields, outcome='FAILED'):
    core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True, terminal_reason='ACTION_ABORTED'))


class ScoutReturnAgainTests(unittest.TestCase):
    returning = mission_end.NoSurvivorTests.returning

    def returns(self, core, now):
        core.on_status(status_msg(SCOUT, observed_at_ns=now))
        core.on_status(status_msg(SUPPORT, observed_at_ns=now))
        return [f for f in core.tick(now) if f['kind'] == 'RETURN_HOME']

    def test_a_failed_return_goes_again_after_the_hold_then_twice_failed_is_shown(self):
        core, ret = self.returning()
        core.return_home_attempts = 2
        core.drain_notes()
        fail(core, ret)
        self.assertIn(f'SCOUT_RETURN_AGAIN task={ret["task_id"]} outcome=FAILED reason=ACTION_ABORTED failed 1/2: '
                      'sent again after admission_timeout_s', core.drain_notes())
        t = NOW + 10
        self.assertEqual([], self.returns(core, t))  # the hold starts here
        core.system_state_change(t)
        # as while it moves: no doll, so NO_SURVIVOR_FOUND, not a failure yet
        self.assertEqual(('RETURNING', 'NO_SURVIVOR_FOUND'), (core.last_state['phase'], core.last_state['reason']))
        self.assertEqual([], self.returns(core, t + HOLD))
        (again,) = self.returns(core, t + HOLD + 1)
        self.assertEqual(SCOUT, again['robot_id'])
        core.on_admission(admission_msg(again))
        fail(core, again)
        self.assertIn(f'SCOUT_RETURN_FAILED task={again["task_id"]} outcome=FAILED reason=ACTION_ABORTED',
                      core.drain_notes())
        self.assertEqual([], self.returns(core, t + 10 * HOLD))
        core.system_state_change(t + 10 * HOLD)
        self.assertEqual('SCOUT_RETURN_FAILED', core.last_state['reason'])

    def test_a_canceled_return_is_not_sent_again(self):
        core, ret = self.returning()
        core.return_home_attempts = 2
        fail(core, ret, outcome='CANCELED')
        self.assertEqual([], self.returns(core, NOW + 10 * HOLD))
        self.assertEqual('FAILED', core.scout_return)


class SupportExitAgainTests(Base):
    arrive_and_finish_exploring = mission_end.ExitChoiceTests.arrive_and_finish_exploring
    support_home = mission_end.ExitChoiceTests.support_home

    def test_a_failed_exit_goes_again_after_the_hold_then_twice_failed_is_shown(self):
        self.core.exits = []
        self.core.support_start_pose = (2., .5, math.pi)  # no exit seen: TB-5 goes to its start spot
        self.core.return_home_attempts = 2
        self.arrive_and_finish_exploring()
        (exit_,) = self.support_home()[0]
        self.core.on_admission(admission_msg(exit_))
        self.core.drain_notes()
        fail(self.core, exit_)
        self.assertIn(f'SUPPORT_EXIT_AGAIN task={exit_["task_id"]} outcome=FAILED reason=ACTION_ABORTED failed 1/2: '
                      'sent again after admission_timeout_s', self.core.drain_notes())
        t = NOW + 3 * S + 10
        self.assertEqual([], self.support_home(t)[0])
        self.assertEqual('SUPPORT_EXITING', self.reason(t))
        self.assertEqual([], self.support_home(t + HOLD)[0])
        self.core.on_pose(pose_msg((3., 4.), map_revision=R2, age_ns=-(t + HOLD + 1 - NOW) + 10**8))  # still there
        (again,) = self.support_home(t + HOLD + 1)[0]
        self.core.on_admission(admission_msg(again))
        fail(self.core, again)
        self.assertIn(f'SUPPORT_EXIT_FAILED task={again["task_id"]} outcome=FAILED reason=ACTION_ABORTED',
                      self.core.drain_notes())
        self.assertEqual([], self.support_home(t + 10 * HOLD)[0])
        self.assertEqual('FAILED', self.core.support_exit)


class ParameterTests(unittest.TestCase):
    def test_typed_and_checked(self):
        self.assertEqual('INTEGER', ros_node.PARAMETERS['return_home_attempts'])
        from test_coordination_ros_core import make_core
        from rescue_contracts import RescueError
        for bad in (0, 1.5):
            with self.subTest(bad=bad), self.assertRaises(RescueError):
                make_core(return_home_attempts=bad)


if __name__ == '__main__':
    unittest.main()
