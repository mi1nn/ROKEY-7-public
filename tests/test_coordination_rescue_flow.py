"""Candidate -> TB-6 recheck -> rescue verdict -> TB-6 return, in the node core (C-11 rows 5, 7-11; E2).

Owner decision 2026-10-13 (4A, 5A): rescue complete = an accepted webcam candidate (exit side) + a TB-6
recheck that judges ABSENT/CLEAR/VALID. An accepted candidate opens a CAMERA recheck of its target (row 5);
the REOBSERVE_TARGET result's RecheckObservation is judged by ``verdict.judge``. After RESCUE_CONFIRMED,
TB-6 gets RETURN_HOME (home_ref ``home/<robot_id>``, the docking approach point; docking is done by hand),
and the mission is FINISHED once TB-6 is back and TB-5 is at the exit.
"""
from types import SimpleNamespace as NS
import unittest

from test_coordination_approach import pose_msg
from test_coordination_ros_core import (SCOUT, SUPPORT, admission_msg, location, make_core, receipt, request,
                                        site_msg, status_msg)
from test_coordination_search import MAP, NOW, uid
from test_coordination_watch import book, candidate, reply

MS = 1_000_000


def recheck(fields, **changes):
    frame = NS(map_session_id=MAP, map_revision=3, transform_revision=1, alignment_status='VALID',
               mission_frame='mission')
    values = dict(task_id=fields['task_id'], target_id=fields['target_id'], robot_id=SCOUT, frame_ref=frame,
                  time=NS(observed_at_ns=NOW, stamp_semantics='CAPTURE', clock_domain_id='site-clock',
                          clock_state='SYNCED', has_clock_error_bound=True, clock_error_bound_ns=5 * MS),
                  camera_state='ABSENT', lidar_state='CLEAR', coverage_state='VALID', sensor_window_start_ns=NOW)
    return NS(**(values | changes))


def result(fields, outcome='COMPLETED', obs=None):
    return NS(outcome=outcome, stop_confirmed=True, has_recheck=obs is not None, recheck=obs)


class RescueFlowTests(unittest.TestCase):
    def setUp(self):
        self.core = make_core(camera_watch=book(), recheck_max_clock_error_ns=20 * MS)
        self.core.on_site_event(site_msg())
        self.fresh(NOW)
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))

    def fresh(self, now):
        for robot in (SCOUT, SUPPORT):
            self.core.on_status(status_msg(robot, observed_at_ns=now - 10**8))

    def tasks(self, kind, now=NOW):
        self.fresh(now)
        return [f for f in self.core.tick(now) if f['kind'] == kind]

    def accepted_candidate(self, n=1):
        self.core.on_command(request('SET_CAMERA_WATCH', command_id=uid(720 + n), target_id=self.target), now_ns=NOW)
        (arm,) = self.core.watch_requests(NOW)
        self.core.on_watch_reply(**reply(arm))
        return self.core.on_site_event(candidate(n, arm['watch_id'], self.target))

    def reobserve(self, now=NOW):
        (fields,) = self.tasks('REOBSERVE_TARGET', now)
        self.core.on_admission(admission_msg(fields))
        return fields

    def test_accepted_candidate_opens_a_camera_recheck_and_issues_reobserve(self):
        self.assertEqual('CANDIDATE_ACCEPTED', self.accepted_candidate())
        self.assertEqual('CAMERA', self.core.mission.open_rechecks[self.target].source)
        fields = self.reobserve()
        self.assertEqual((SCOUT, self.target), (fields['robot_id'], fields['target_id']))

    def test_a_good_recheck_is_rescue_confirmed_then_tb6_returns_to_the_docking_approach(self):
        self.accepted_candidate()
        fields = self.reobserve()
        self.assertTrue(self.core.on_result(fields['task_id'], result(fields, obs=recheck(fields))))
        self.assertEqual('RESCUE_CONFIRMED', self.core.mission.verdicts[self.target])
        (estimate,) = self.core.target_snapshot_change()['targets']
        self.assertEqual('RESCUE_CONFIRMED', estimate['rescue_state'])
        (home,) = self.tasks('RETURN_HOME')
        self.assertEqual((SCOUT, f'home/{SCOUT}'), (home['robot_id'], home['home_ref']))
        self.assertEqual('RETURNING', self.core.mission.phase())
        self.assertEqual('SCOUT_RETURNING', self.core.system_state_change(NOW)['reason'])
        self.core.on_admission(admission_msg(home))
        self.core.on_result(home['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual('SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND', self.core.system_state_change(NOW)['reason'])
        self.assertEqual([], self.tasks('RETURN_HOME'))  # once

    def test_finished_when_tb6_is_back_and_tb5_is_at_the_exit(self):
        core = self.core
        self.accepted_candidate()
        fields = self.reobserve()
        core.on_result(fields['task_id'], result(fields, obs=recheck(fields)))
        (home,) = self.tasks('RETURN_HOME')
        core.on_admission(admission_msg(home))
        core.on_result(home['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual('RETURNING', core.mission.phase())  # TB-5 is not at the exit yet
        core.support_exit = 'AT_EXIT'
        core.note_support_at_exit()
        self.assertEqual(('FINISHED', 'MISSION_FINISHED'),
                         (core.mission.phase(), core.system_state_change(NOW)['mode']))

    def test_each_verdict_leaves_one_log_note_for_the_operator(self):
        # Field run (2026-10-13): RESULT lines alone do not show the verdict; the node logs these notes.
        self.accepted_candidate()
        fields = self.reobserve()
        self.core.on_result(fields['task_id'], result(fields, obs=recheck(fields, lidar_state='BLOCKED')))
        (note,) = self.core.drain_notes()
        self.assertTrue(note.startswith(f'VERDICT target={self.target} state=RECHECK_NEEDED reason=LIDAR_NOT_CLEAR '
                                        'source=CAMERA'), note)
        self.assertEqual([], self.core.drain_notes())

    def test_a_failing_condition_is_recheck_needed_with_its_reason(self):
        self.accepted_candidate()
        fields = self.reobserve()
        self.core.on_result(fields['task_id'], result(fields, obs=recheck(fields, lidar_state='BLOCKED')))
        self.assertEqual('RECHECK_NEEDED', self.core.mission.verdicts[self.target])
        self.assertEqual('RECHECK_NEEDED_LIDAR_NOT_CLEAR', self.core.system_state_change(NOW)['reason'])
        self.assertEqual([], self.tasks('RETURN_HOME'))  # TB-6 does not go home without a confirmed rescue

    def test_a_recheck_that_started_before_the_candidate_is_an_order_violation(self):
        self.accepted_candidate()
        fields = self.reobserve()
        early = NOW - 10**9  # site_msg candidates are observed at NOW - 1 s
        self.core.on_result(fields['task_id'], result(fields, obs=recheck(fields, sensor_window_start_ns=early)))
        self.assertEqual('RECHECK_NEEDED_ORDER_VIOLATION', self.core.system_state_change(NOW)['reason'])

    def test_completed_without_a_recheck_observation_is_recheck_needed_not_reissued(self):
        # Row 5: no new task after COMPLETED; nothing to judge closes the recheck for an operator REQUEST_RECHECK.
        self.accepted_candidate()
        first = self.reobserve()
        self.core.on_result(first['task_id'], result(first))
        self.assertEqual('RECHECK_NEEDED', self.core.mission.verdicts[self.target])
        self.assertEqual('RECHECK_NEEDED_RECHECK_OBSERVATION_MISSING', self.core.system_state_change(NOW)['reason'])
        self.assertEqual([], self.tasks('REOBSERVE_TARGET'))

    def test_a_recheck_that_did_not_complete_is_issued_again(self):
        self.accepted_candidate()
        first = self.reobserve()
        self.core.on_result(first['task_id'], result(first, outcome='FAILED'))
        (again,) = self.tasks('REOBSERVE_TARGET')
        self.assertNotEqual(first['task_id'], again['task_id'])

    def test_operator_recheck_cannot_confirm_without_a_measured_coordinator_clock(self):
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(702), target_id=self.target), now_ns=NOW)
        fields = self.reobserve()
        self.core.on_result(fields['task_id'], result(fields, obs=recheck(fields)))
        self.assertEqual('RECHECK_NEEDED_CANDIDATE_TIME_BASIS_UNUSABLE', self.core.system_state_change(NOW)['reason'])

    def test_a_candidate_while_an_operator_recheck_is_open_is_recorded_only(self):
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(702), target_id=self.target), now_ns=NOW)
        self.assertEqual('CANDIDATE_ACCEPTED', self.accepted_candidate())
        self.assertEqual('OPERATOR', self.core.mission.open_rechecks[self.target].source)  # row 10: link unchanged


if __name__ == '__main__':
    unittest.main()
