"""TB-5 (SUPPORT) automatic dispatch: APPROACH_TARGET to a standoff point once the target is saved (E2).

Owner decision 2026-10-13 (1B·2B): with ``support_dispatch=AUTO`` the coordinator sends the SUPPORT one
APPROACH_TARGET without operator approval, once the target is CONFIRMED and the ReportReceipt of its
current revision is PERSISTED. The goal is ``approach_standoff_m`` from the target toward the SUPPORT's
current mission pose, facing the target. Nothing is made up: no fresh VALID pose on the coordinator's
map basis, or a target frame that is not VALID on that basis, holds the task with a reason.
``support_dispatch=OPERATOR`` keeps DISPATCH_SUPPORT / MOVE_SEGMENT.
"""
import math
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from rescue_coordination.planning import GoalPoint, Reservation
from test_coordination_ros_core import (ACCEPT, SUPPORT, admission_msg, frame, goal_template, location, make_core, receipt,
                                        request, site_msg, status_msg)
from test_coordination_search import MAP, NOW, uid

POSE_AGE = 1_000_000_000


def pose_msg(xy=(0., 4.), *, age_ns=10**8, status='VALID', **frame_changes):
    return NS(robot_id=SUPPORT, mission_pose_status=status, frame_ref=frame(robot_id=SUPPORT, **frame_changes),
              time=NS(observed_at_ns=NOW - age_ns),
              mission_pose=NS(pose=NS(pose=NS(position=NS(x=xy[0], y=xy[1], z=0.)))))


def auto_core(**changes):
    return make_core(**(dict(support_dispatch='AUTO', approach_standoff_m=1.0, support_pose_max_age_ns=POSE_AGE)
                        | changes))


class ApproachPointTests(unittest.TestCase):
    def test_standoff_toward_the_support_facing_the_target(self):
        xy, yaw = ros_node.approach_point((4., 4.), (0., 4.), 1.0)
        self.assertEqual((3.0, 4.0), tuple(round(v, 9) for v in xy))
        self.assertAlmostEqual(0.0, yaw)
        xy, yaw = ros_node.approach_point((0., 0.), (0., -5.), 2.0)
        self.assertEqual((0.0, -2.0), tuple(round(v, 9) for v in xy))
        self.assertAlmostEqual(math.pi / 2, yaw)

    def test_already_within_the_standoff_stays_and_turns_to_the_target(self):
        xy, yaw = ros_node.approach_point((4., 4.), (3.5, 4.), 1.0)
        self.assertEqual((3.5, 4.0), xy)
        self.assertAlmostEqual(0.0, yaw)

    def test_no_direction_when_on_the_target(self):
        self.assertIsNone(ros_node.approach_point((4., 4.), (4., 4.), 1.0))


class ReservationRuleTests(unittest.TestCase):
    def reservation(self, **changes):
        values = dict(task_id=uid(1), target_id=uid(2), target_revision=0, robot_id=SUPPORT, generation=0,
                      authority_epoch=uid(3), candidate=None, command_id=uid(4), robot_boot_id=uid(5),
                      task_kind='APPROACH_TARGET', goal=GoalPoint((3., 4.), 0., MAP, 3, 1))
        return Reservation(**(values | changes))

    def test_approach_with_a_goal_and_no_approval_needs_no_candidate(self):
        self.assertEqual('APPROACH_TARGET', self.reservation().task_kind)

    def test_approach_without_goal_or_with_an_approval_is_refused(self):
        for changes in (dict(goal=None), dict(approval_id=uid(6))):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                self.reservation(**changes)


class AutoDispatchTests(unittest.TestCase):
    def setUp(self):
        self.core = auto_core()
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SUPPORT))
        self.target = self.core.on_target_location(location(1), now_ns=NOW)

    def save(self):
        (report,) = self.core.reports_due(NOW)
        self.assertTrue(self.core.on_receipt(receipt(report)))

    def approaches(self):
        return [f for f in self.core.tick(NOW) if f['kind'] == 'APPROACH_TARGET']

    def test_nothing_before_the_report_is_saved(self):
        self.core.on_pose(pose_msg())
        self.assertEqual([], self.approaches())

    def test_one_approach_after_persisted_with_the_standoff_goal(self):
        self.core.on_pose(pose_msg())
        self.save()
        (fields,) = self.approaches()
        self.assertEqual((SUPPORT, self.target, 0), (fields['robot_id'], fields['target_id'], fields['target_revision']))
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        r = goal.route
        self.assertEqual((True, False, 3.0, 4.0, 'VALID', MAP, 'mission', 'UNKNOWN', ''),
                         (goal.has_route, goal.has_approval, round(r.approach_goal.pose.position.x, 9),
                          round(r.approach_goal.pose.position.y, 9), r.frame_ref.alignment_status,
                          r.frame_ref.map_session_id, r.approach_goal.header.frame_id, r.assessment.status,
                          r.path_sha256))
        self.assertEqual((SUPPORT, self.target, fields['policy_version']), (r.robot_id, r.target_id, r.policy_version))
        self.assertEqual([], self.approaches())  # once per mission (idempotent)
        state = self.core.system_state_change(NOW)
        self.assertEqual((True, self.target, 'DISPATCH_APPROVED', 'EXECUTING'),
                         (state['has_dispatch'], state['target_id'], state['phase'], state['segment_phase']))

    def test_no_pose_stale_pose_other_basis_or_not_valid_holds_with_a_reason(self):
        self.save()
        self.assertEqual(([], 'SUPPORT_POSE_UNKNOWN'), (self.approaches(), self.core.issue_block))
        self.assertEqual('SUPPORT_POSE_UNKNOWN', self.core.system_state_change(NOW)['reason'])
        for msg, reason in ((pose_msg(age_ns=POSE_AGE + 1), 'SUPPORT_POSE_UNKNOWN'),
                            (pose_msg(status='UNKNOWN'), 'SUPPORT_POSE_UNKNOWN'),
                            (pose_msg(map_revision=4), 'SUPPORT_POSE_BASIS_MISMATCH'),
                            (pose_msg(mission_frame='other'), 'SUPPORT_POSE_BASIS_MISMATCH'),
                            (pose_msg((4., 4.)), 'SUPPORT_AT_TARGET')):
            with self.subTest(reason=reason):
                self.core.on_pose(msg)
                self.assertEqual(([], reason), (self.approaches(), self.core.issue_block))

    def test_tb5_not_ready_holds_with_a_reason(self):
        # Field check 2026-10-13: TB-5 reports READY only when armed (rescue_robot task_status.py:17). Before arm
        # the dispatch waited with no SystemState reason, so the operator could not see why TB-5 did not go.
        self.core.on_pose(pose_msg())
        self.core.on_status(status_msg(SUPPORT, readiness='NOT_READY'))
        self.save()
        self.assertEqual(([], 'SUPPORT_NOT_READY'), (self.approaches(), self.core.issue_block))
        self.assertEqual('SUPPORT_NOT_READY', self.core.system_state_change(NOW)['reason'])
        self.core.on_status(status_msg(SUPPORT))  # armed
        (fields,) = self.approaches()
        self.assertEqual(SUPPORT, fields['robot_id'])

    def test_target_frame_not_valid_holds(self):
        core = auto_core()
        core.on_site_event(site_msg())
        core.on_status(status_msg(SUPPORT))
        core.on_pose(pose_msg())
        core.on_target_location(location(1, frame_ref=frame(alignment_status='DEGRADED')), now_ns=NOW)
        (report,) = core.reports_due(NOW)
        core.on_receipt(receipt(report))
        self.assertEqual([], [f for f in core.tick(NOW) if f['kind'] == 'APPROACH_TARGET'])
        self.assertEqual('TARGET_ALIGNMENT_NOT_VALID', core.issue_block)

    def test_operator_dispatch_is_refused_while_auto(self):
        self.save()
        reply = self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=self.target), now_ns=NOW)
        self.assertEqual((False, 'NOT_AUTHORIZED', 'AUTO_DISPATCH_ENABLED'),
                         (reply['accepted'], reply['reason_code'], self.core.last_reason))

    def test_arrival_and_failure_are_shown(self):
        self.core.on_pose(pose_msg())
        self.save()
        (fields,) = self.approaches()
        self.assertTrue(self.core.on_admission(admission_msg(fields)))
        self.assertEqual('SEGMENT_MOVING', self.core.mission.phase())
        self.assertTrue(self.core.on_result(fields['task_id'], NS(outcome='COMPLETED', stop_confirmed=True)))
        self.assertEqual('ARRIVED', self.core.mission.phase())
        (estimate,) = self.core.target_snapshot_change()['targets']
        self.assertEqual(('APPROACH_COMPLETE', 'STOPPED'), (estimate['approach_state'], estimate['segment_phase']))

    def test_failed_approach_is_not_retried_and_says_why(self):
        self.core.on_pose(pose_msg())
        self.save()
        (fields,) = self.approaches()
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='UNREACHABLE', stop_confirmed=True,
                                                  terminal_reason='PASSAGE_BLOCKED'))
        self.assertEqual([], self.approaches())
        # 작전 OPS-119: the executor's terminal reason is shown, so a blocked passage differs from other failures.
        self.assertEqual('APPROACH_FAILED_PASSAGE_BLOCKED', self.core.system_state_change(NOW)['reason'])
        (estimate,) = self.core.target_snapshot_change()['targets']
        self.assertEqual('FAILED', estimate['approach_state'])

    def test_failed_approach_without_a_terminal_reason_shows_the_outcome(self):
        self.core.on_pose(pose_msg())
        self.save()
        (fields,) = self.approaches()
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='FAILED', stop_confirmed=True, terminal_reason=''))
        self.assertEqual('APPROACH_FAILED_FAILED', self.core.system_state_change(NOW)['reason'])

    def test_refused_admission_waits_then_goes_again_as_a_new_task(self):
        # Same hold as a refused MOVE_SEGMENT/REOBSERVE (#167): admission_timeout_ns, TASK_REFUSED meanwhile.
        self.core.on_pose(pose_msg())
        self.save()
        (first,) = self.approaches()
        self.core.on_admission(admission_msg(first, accepted=False))
        self.assertEqual(([], 'TASK_REFUSED'), (self.approaches(), self.core.issue_block))
        later = NOW + ACCEPT + 1
        self.core.on_status(status_msg(SUPPORT, observed_at_ns=later - 10**8))
        self.core.on_pose(pose_msg(age_ns=10**8 - (later - NOW)))
        (second,) = [f for f in self.core.tick(later) if f['kind'] == 'APPROACH_TARGET']
        self.assertNotEqual(first['task_id'], second['task_id'])


class OperatorModeTests(unittest.TestCase):
    def test_operator_mode_never_dispatches_by_itself(self):
        core = make_core(support_dispatch='OPERATOR', approach_standoff_m=1.0, support_pose_max_age_ns=POSE_AGE)
        core.on_site_event(site_msg())
        core.on_status(status_msg(SUPPORT))
        core.on_pose(pose_msg())
        core.on_target_location(location(1), now_ns=NOW)
        (report,) = core.reports_due(NOW)
        core.on_receipt(receipt(report))
        self.assertEqual([], core.tick(NOW))

    def test_policy_values_are_checked(self):
        for changes in (dict(support_dispatch='SOMETIMES'), dict(approach_standoff_m=0.0),
                        dict(support_pose_max_age_ns=0)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                auto_core(**changes)

    def test_parameters_are_typed_and_required(self):
        self.assertEqual(('STRING', 'DOUBLE', 'DOUBLE'),
                         tuple(ros_node.PARAMETERS[n] for n in ('support_dispatch', 'approach_standoff_m',
                                                                'support_pose_max_age_s')))


if __name__ == '__main__':
    unittest.main()
