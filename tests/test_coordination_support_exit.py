"""TB-5 leaves for the exit N seconds after it reached the target (owner decision 2026-10-13, Q1-b; E2).

After the SUPPORT's APPROACH_TARGET ends COMPLETED with a confirmed stop, the coordinator waits
``support_exit_wait_s`` and then sends the SUPPORT one RETURN_HOME (home_ref ``home/<robot_id>``: the robot
holds its exit pose, 5A). SystemState.reason counts down meanwhile. A judged target or the end of disaster
mode cancels it; a failed approach never sends the SUPPORT to the exit.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_coordination import ros_node
from test_coordination_approach import auto_core, pose_msg
from test_coordination_ros_core import SUPPORT, admission_msg, goal_template, location, receipt, site_msg, status_msg
from test_coordination_search import NOW

S = 1_000_000_000
WAIT = 5 * S


class SupportExitTests(unittest.TestCase):
    def setUp(self):
        self.core = auto_core(support_exit_wait_ns=WAIT)
        self.core.on_site_event(site_msg())
        self.status(NOW)
        self.core.on_pose(pose_msg())
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        (self.approach,) = self.tasks('APPROACH_TARGET', NOW)
        self.core.on_admission(admission_msg(self.approach))

    def status(self, now):
        self.core.on_status(status_msg(SUPPORT, observed_at_ns=now - 10**8))

    def tasks(self, kind, now):
        self.status(now)
        return [f for f in self.core.tick(now) if f['kind'] == kind]

    def arrive(self, outcome='COMPLETED'):
        self.core.on_result(self.approach['task_id'], NS(outcome=outcome, stop_confirmed=True))

    def test_exit_after_the_wait_with_the_home_ref(self):
        self.arrive()
        self.assertEqual([], self.tasks('RETURN_HOME', NOW + S))  # the wait starts at the first tick after arrival
        self.assertEqual('SUPPORT_EXIT_IN_5S', self.core.system_state_change(NOW + S)['reason'])
        self.assertEqual([], self.tasks('RETURN_HOME', NOW + S + WAIT - 1))
        (fields,) = self.tasks('RETURN_HOME', NOW + S + WAIT)
        self.assertEqual((SUPPORT, f'home/{SUPPORT}', None), (fields['robot_id'], fields['home_ref'], fields['goal_pose']))
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        self.assertEqual(('RETURN_HOME', f'home/{SUPPORT}', False, False),
                         (goal.kind, goal.home_ref, goal.has_route, goal.has_approval))
        self.assertEqual([], self.tasks('RETURN_HOME', NOW + 2 * WAIT))  # once
        self.assertEqual('SUPPORT_EXITING', self.core.system_state_change(NOW + 2 * WAIT)['reason'])

    def test_the_countdown_rounds_up_and_changes_each_second(self):
        self.arrive()
        self.tasks('RETURN_HOME', NOW)
        self.assertEqual('SUPPORT_EXIT_IN_5S', self.core.system_state_change(NOW)['reason'])
        self.tasks('RETURN_HOME', NOW + S + 1)
        self.assertEqual('SUPPORT_EXIT_IN_4S', self.core.system_state_change(NOW + S + 1)['reason'])

    def test_arrival_at_the_exit_is_shown(self):
        self.arrive()
        self.tasks('RETURN_HOME', NOW)
        (fields,) = self.tasks('RETURN_HOME', NOW + WAIT)
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual('SUPPORT_AT_EXIT', self.core.system_state_change(NOW + WAIT)['reason'])

    def test_a_failed_approach_never_goes_to_the_exit(self):
        self.arrive('UNREACHABLE')
        self.assertEqual([], self.tasks('RETURN_HOME', NOW + 10 * WAIT))
        self.assertEqual('APPROACH_FAILED', self.core.system_state_change(NOW + 10 * WAIT)['reason'])

    def test_a_judged_target_or_the_end_of_disaster_cancels_the_exit(self):
        for change in ('judged', 'normal'):
            with self.subTest(change):
                self.setUp()
                self.arrive()
                self.tasks('RETURN_HOME', NOW)
                if change == 'judged':
                    self.core.mission.open_rechecks[self.target] = None
                    self.core.mission.record_rescue_verdict(self.target, 'RESCUE_CONFIRMED')
                else:
                    self.core.mission.mode = 'NORMAL'
                self.assertEqual([], self.tasks('RETURN_HOME', NOW + WAIT))
                self.assertEqual('SUPPORT_EXIT_CANCELED', self.core.issue_block or self.core.support_exit)

    def test_wait_is_a_typed_required_parameter(self):
        self.assertEqual('DOUBLE', ros_node.PARAMETERS['support_exit_wait_s'])


if __name__ == '__main__':
    unittest.main()
