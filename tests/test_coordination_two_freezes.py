"""Two map freezes per run and a map basis per robot (owner decision 2026-10-13 night; 작전 OPS-154·156·159; E2).

1st freeze (R0+1) when the doll is saved and TB-6 stands: TB-5 goes on R1. TB-6 keeps mapping with SLAM on the
search revision R0 and keeps exploring (frontiers on R0); its observations stay on R0. A doll observation on R0
then never moves the target off R1 (same SLAM lineage, OPS-156 ④).
2nd freeze (R0+2) when the exploration is complete and TB-6 stands: both robots move to R2; the target and every
exit of R0/R1 are re-based to R2 without a new report (OPS-156 ②). Each freeze is known by the
/rescue/map/frozen_revision text naming the frozen grid's hash (통합-로봇 #242).
"""
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import live_task_inputs
from test_coordination_approach import POSE_AGE, pose_msg
from test_coordination_frontier import grid, scout_pose
from test_coordination_map_freeze import (CLEARANCE, EXIT_STANDOFF, MAP, FakeGrid, exit_msg, map_state, revision_info,
                                          TIMEOUT)
from test_coordination_ros_core import SCOUT, SUPPORT, admission_msg, frame, location, make_core, receipt, request, \
    site_msg, status_msg
from test_coordination_search import NOW, uid

S = 1_000_000_000
R0, R1, R2, TRANSFORM = 3, 4, 5, 1
B0, B1, B2 = (MAP, R0, TRANSFORM), (MAP, R1, TRANSFORM), (MAP, R2, TRANSFORM)


def core_():
    return make_core(support_dispatch='AUTO', approach_standoff_m=1.0, support_pose_max_age_ns=POSE_AGE,
                     map_mode='SLAM_FREEZE', freeze_clearance_m=CLEARANCE, map_freeze_timeout_ns=TIMEOUT,
                     exit_standoff_m=EXIT_STANDOFF, search_mode='FRONTIER', frontier_issue_delay_ns=0,
                     policy_changes=dict(executable_bases=frozenset({'FRONTIER'})))


class TwoFreezesTests(unittest.TestCase):
    def setUp(self):
        self.core = core_()
        self.core.on_site_event(site_msg())
        self.status(NOW)
        self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        self.core.on_exit_location(exit_msg(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.tick(NOW)
        self.freeze(R1)                       # the doll's freeze
        (report,) = self.core.reports_due(NOW)  # the re-based revision saved again (first freeze only)
        self.core.on_receipt(receipt(report))

    def status(self, now, **scout):
        self.core.on_status(status_msg(SCOUT, observed_at_ns=now - 10**8, **scout))
        self.core.on_status(status_msg(SUPPORT, observed_at_ns=now - 10**8))

    def tick(self, now, kind=None):
        self.status(now)
        return [f for f in self.core.tick(now) if kind is None or f['kind'] == kind]

    def reason(self, now=NOW):
        self.core.system_state_change(now)
        return self.core.last_state['reason']

    def freeze(self, revision, sha=None):
        self.assertTrue(self.core.take_freeze_request())
        self.core.on_freeze_reply(True, 'frozen')
        sha = sha or ('c' * 63 + str(revision))
        self.core.on_frozen_map(FakeGrid(sha256=sha))
        self.core.on_frozen_revision(revision_info(revision, sha))

    def dispatch(self, now=NOW):
        self.core.on_map_state(map_state(SUPPORT, revision=R1))
        self.core.on_pose(pose_msg(map_revision=R1, age_ns=-(now - NOW) + 10**8))
        (approach,) = self.tick(now, 'APPROACH_TARGET')
        return approach

    def explore_to_the_end(self, now):
        """TB-6 on R0 with a map that has no frontier left: two map updates in a row end the exploration."""
        for k in range(2):
            self.core.on_scout_map(grid(known_cols=40))
            self.core.on_pose(scout_pose(at=now + k))
            self.tick(now + k)
        self.assertTrue(self.core.exploration_complete)

    def test_after_the_first_freeze_tb5_is_on_r1_and_tb6_explores_on_r0(self):
        approach = self.dispatch()
        goal = approach['route']['approach_goal']
        self.assertEqual(B1, (goal.map_session_id, goal.map_revision, goal.transform_revision))
        self.core.on_admission(admission_msg(approach))
        self.core.on_scout_map(grid())
        self.core.on_pose(scout_pose())
        (search,) = self.tick(NOW, 'SEARCH_AREA')  # frontier_issue_delay 0 here: picked and sent in one tick
        self.assertEqual(SCOUT, search['robot_id'])
        (site,) = [p for k, p in live_task_inputs.destinations(self.core).items() if p['robot_id'] == SCOUT]
        self.assertEqual(R0, site['map_revision'])

    def test_a_doll_observation_on_r0_between_the_freezes_keeps_the_target_on_r1(self):
        self.core.on_target_location(location(2, (4.2, 4.)), now_ns=NOW)
        self.assertEqual(B1, self.core.target_geo[self.target][0])

    def test_tb6_observations_on_r1_are_refused(self):
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(location(3, (4.1, 4.), frame_ref=frame(map_revision=R1)), now_ns=NOW)
        self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)

    def test_the_second_freeze_follows_the_complete_exploration(self):
        self.dispatch()
        self.assertFalse(self.core.take_freeze_request())
        self.explore_to_the_end(NOW)
        self.tick(NOW + 2)
        self.freeze(R2)
        self.assertEqual(B2, self.core.frozen)
        self.assertEqual(B2, self.core.target_geo[self.target][0])
        self.assertEqual([B2], [e['basis'] for e in self.core.exits])
        self.assertEqual([], self.core.reports_due(NOW + 3))  # no new save at the second freeze (OPS-156 ②)
        notes = ' '.join(self.core.drain_notes())
        self.assertIn('TARGET_REBASED_ON_FREEZE', notes)
        self.assertIn('EXIT_REBASED_ON_FREEZE', notes)

    def test_the_second_freeze_waits_for_tb6_to_stand(self):
        self.dispatch()
        self.explore_to_the_end(NOW)
        self.status(NOW + 2, motion_state='MOVING')
        self.core.tick(NOW + 2)
        self.assertFalse(self.core.take_freeze_request())
        self.assertEqual('MAP_FREEZE_WAITING_SCOUT_STOP', self.reason(NOW + 2))

    def test_the_first_frozen_revision_again_is_not_the_second_freeze(self):
        self.dispatch()
        self.explore_to_the_end(NOW)
        self.tick(NOW + 2)
        self.assertTrue(self.core.take_freeze_request())
        self.core.on_freeze_reply(True, 'frozen')
        self.core.on_frozen_map(FakeGrid(sha256='c' * 63 + str(R1)))
        self.core.on_frozen_revision(revision_info(R1, 'c' * 63 + str(R1)))  # the latched first one
        self.assertEqual(B1, self.core.frozen)

    def test_after_the_second_freeze_tb6_observes_on_r2_only(self):
        self.dispatch()
        self.explore_to_the_end(NOW)
        self.tick(NOW + 2)
        self.freeze(R2)
        with self.assertRaises(RescueError):
            self.core.on_exit_location(exit_msg(4, (5., 5.), track='EXIT-2'), now_ns=NOW)  # R0
        self.core.on_exit_location(exit_msg(5, (5., 5.), revision=R2, track='EXIT-2'), now_ns=NOW)
        self.assertEqual(B2, self.core.exits[-1]['basis'])


if __name__ == '__main__':
    unittest.main()
