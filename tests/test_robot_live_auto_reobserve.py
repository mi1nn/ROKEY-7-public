"""LIVE_ROS REOBSERVE_TARGET for TB-6 and the ObserveTarget recheck after the stop (C1-b), no ROS.

User decision 4A (2026-10-13): rescue is confirmed by the webcam exit candidate plus a TB-6
on-site recheck. The coordinator sends REOBSERVE_TARGET (no operator approval, Phase 1 shape);
TB-6 takes it under its own automatic-move policy (kinds include REOBSERVE_TARGET), drives to
the observation pose, and only after a confirmed stop asks the recheck server (#165,
/rescue/robots/<id>/observe_target) to observe the target's current position. Whatever the
server answers (OBSERVED, UNKNOWN, FAILED) goes into TaskResult.recheck unchanged; when no
observation could be asked (no target position, no server, deadline) has_recheck stays false.
"""
import asyncio
from types import SimpleNamespace as NS
import unittest
import unittest.mock

import test_robot_live_auto_approach as approach

POLICY = dict(approach.POLICY, kinds=['REOBSERVE_TARGET'])
RECHECK = dict(window_ns=3_000_000_000, deadline_ns=10_000_000_000)


class LiveAutoReobserve(unittest.TestCase):
    def setUp(self):
        a = self.a = approach.LiveAutoApproach('test_policy_on_accepts_and_drives_to_the_approach_goal_in_the_native_frame')
        a.setUp()
        self.f, self.adapter, self.node, self.g = a.f, a.adapter, a.node, a.g
        self.g.kind = 'REOBSERVE_TARGET'
        self.observed = []
        self.answer = NS(outcome='OBSERVED', reason_code='', observation=NS(observation_id='obs-1',
                         camera_state='ABSENT', lidar_state='CLEAR', coverage_state='VALID'))
        self.build(POLICY)

    @property
    def runtime(self):
        return self.a.runtime

    def submit(self):
        return self.a.submit()

    def build(self, policy, **changes):
        self.a.build(policy, **dict(dict(recheck=dict(RECHECK)), **changes))
        now = self.adapter.wall_clock_ns()
        # The coordinator's TargetSnapshot (IF-070): revision and mission position of the target.
        self.runtime._targets = {self.g.target_id: dict(revision=self.g.target_estimate_revision, observed_at_ns=now,
                                                         valid_until_ns=now + 60_000_000_000,
                                                         mission_position=(5.0, 0.0, 0.0))}

        async def observe(goal):
            self.observed.append(goal)
            return self.answer
        self.runtime.observe = observe

    def finish(self, outcome_code=4):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)
        self.adapter.on_terminal(self.g.command_id, outcome_code)
        for _ in range(4):
            self.f.r.f.tick()
        report = self.runtime.server.ingress.report(self.g.command_id)
        asyncio.run(self.runtime.before_result(report, self.g))
        return self.runtime.encode_result(report, self.g).result

    # --- admission --------------------------------------------------------------------------

    def test_policy_on_accepts_without_a_protected_route_record(self):
        with unittest.mock.patch.object(self.runtime.gate, 'validate', side_effect=AssertionError('phase1 used')):
            self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.assertEqual(1., self.f.r.f.port.goals[0][1].x)

    def test_kinds_without_reobserve_reject_it(self):
        self.build(dict(POLICY, kinds=['APPROACH_TARGET']))
        self.assertEqual('reject', self.submit())

    def test_target_revision_must_be_the_coordinators_current_one(self):
        self.runtime._targets[self.g.target_id]['revision'] += 1
        self.assertEqual('reject', self.submit())
        self.runtime._targets.clear()
        self.assertEqual('reject', self.submit())

    # --- recheck after the stop --------------------------------------------------------------

    def test_completed_and_stopped_asks_the_recheck_server_and_carries_its_answer(self):
        result = self.finish()
        self.assertTrue(result.stop_confirmed)
        self.assertEqual('RECHECK', result.completion_scope)
        self.assertEqual(1, len(self.observed))
        goal = self.observed[0]
        self.assertEqual((self.g.task_id, self.g.target_id, self.g.robot_id), (goal['task_id'], goal['target_id'],
                                                                               goal['robot_id']))
        # mission (5, 0) with the declared alignment x+2 -> native (3, 0)
        self.assertEqual((3.0, 0.0, 0.0), goal['watched_native_point'])
        self.assertEqual(RECHECK['window_ns'], goal['requested_window_ns'])
        self.assertEqual(('VALID', self.g.frame_ref.map_session_id), (goal['frame_ref']['alignment_status'],
                                                                       goal['frame_ref']['map_session_id']))
        self.assertTrue(result.has_recheck)
        self.assertIs(self.answer.observation, result.recheck)

    def test_unknown_or_failed_answers_are_carried_unchanged(self):
        for outcome in ('UNKNOWN', 'FAILED'):
            with self.subTest(outcome=outcome):
                self.build(POLICY)
                self.answer = NS(outcome=outcome, reason_code='DEADLINE', observation=NS(camera_state='UNKNOWN'))
                result = self.finish()
                self.assertTrue(result.has_recheck)
                self.assertIs(self.answer.observation, result.recheck)

    def test_no_recheck_without_a_confirmed_completed_stop(self):
        result = self.finish(outcome_code=6)  # Nav2 aborted
        self.assertEqual([], self.observed)
        self.assertFalse(result.has_recheck)

    def test_no_answer_means_no_recheck(self):
        async def unavailable(goal):
            return None
        self.runtime.observe = unavailable
        self.assertFalse(self.finish().has_recheck)

    def test_target_position_gone_after_the_move_means_no_recheck(self):
        self.assertEqual('accept', self.submit(), self.node.errors)
        self.adapter.step()
        self.adapter.on_goal_response(self.g.command_id, True)
        self.adapter.on_terminal(self.g.command_id, 4)
        for _ in range(4):
            self.f.r.f.tick()
        self.runtime._targets.clear()
        report = self.runtime.server.ingress.report(self.g.command_id)
        asyncio.run(self.runtime.before_result(report, self.g))
        self.assertEqual([], self.observed)
        self.assertFalse(self.runtime.encode_result(report, self.g).result.has_recheck)

    def test_approach_results_never_carry_a_recheck(self):
        self.g.kind = 'APPROACH_TARGET'
        self.build(dict(POLICY, kinds=['APPROACH_TARGET']))
        result = self.finish()
        self.assertEqual([], self.observed)
        self.assertFalse(result.has_recheck)

    def test_recheck_settings_are_checked(self):
        from rescue_robot.executor import ContractError
        for bad in (dict(window_ns=0, deadline_ns=1), dict(window_ns=5, deadline_ns=4), dict(window_ns=1),
                    dict(window_ns=1.0, deadline_ns=2)):
            with self.subTest(bad=bad), self.assertRaises(ContractError):
                self.a.build(POLICY, recheck=bad)

    def test_reobserve_policy_needs_recheck_settings(self):
        from rescue_robot.executor import ContractError
        with self.assertRaises(ContractError):
            self.a.build(POLICY)


if __name__ == '__main__':
    unittest.main()
