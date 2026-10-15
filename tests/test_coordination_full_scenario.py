"""Coordinator end of the one-PC full scenario (방식 B, completion_mode EXIT_WAIT_SCOUT_DOCK, 2026-10-15). No ROS.

TB-5 waits at its exit; TB-6 returns to its own start pose on its exploration end (never on TB-5's arrival) and the
run is FINISHED only once its dock is confirmed. Also the RETURN_HOME frame_ref, the exit without a QR and the two
traffic fixes found in the rehearsal.
"""
import json
from types import SimpleNamespace as NS
import unittest

from rescue_coordination import ros_node
from rescue_coordination.mission import HomePosition
from test_coordination_approach import pose_msg
from test_coordination_frontier import scout_pose
from test_coordination_map_freeze import map_state
from test_coordination_mission_end import RealGrid
from test_coordination_ros_core import (SCOUT, SUPPORT, admission_msg, location, make_core, receipt, request,
                                        site_msg, status_msg)
from test_coordination_search import MAP, NOW, uid
from test_coordination_two_freezes import core_

S = 1_000_000_000
DONE = NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason='')


def support_pose(xy=(0., 4.)):
    msg = pose_msg(xy, map_revision=3)
    msg.mission_pose.pose.pose.orientation = NS(x=0., y=0., z=0., w=1.)
    return msg


def dock_core(**changes):
    return core_(**(dict(map_mode='SLAM_LIVE', live_map_max_age_ns=10 * S, completion_mode='EXIT_WAIT_SCOUT_DOCK',
                         scout_dock_timeout_ns=60 * S) | changes))


class NoSurvivorDockTests(unittest.TestCase):
    """The whole map, no doll: TB-6 goes home and docks; TB-5 never leaves."""

    def setUp(self):
        c = self.c = dock_core()
        c.mission.homes = {}            # no profile docking point: TB-6's own start pose is the goal
        c.on_map_state(map_state(SCOUT, revision=3))
        c.on_start_pose((0.5, 0.25, 0.1))
        c.on_site_event(site_msg())
        c.on_status(status_msg(SCOUT))
        c.on_status(status_msg(SUPPORT))
        c.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        sent = []
        for k in range(3):
            c.on_scout_map(RealGrid(), now_ns=NOW + k)
            c.on_pose(scout_pose(at=NOW + k))
            c.on_status(status_msg(SCOUT, observed_at_ns=NOW + k))
            sent += c.tick(NOW + k)
        self.assertTrue(c.exploration_complete)
        (self.ret,) = [t for t in sent if t['kind'] == 'RETURN_HOME']

    def arrive(self):
        c = self.c
        c.on_admission(admission_msg(self.ret))
        c.on_result(self.ret['task_id'], DONE)
        c.on_status(status_msg(SCOUT, observed_at_ns=NOW + 3))
        c.tick(NOW + 3)

    def test_return_goes_to_the_recorded_start_pose_with_a_full_frame(self):
        c = self.c
        self.assertEqual((SCOUT, f'home/{SCOUT}'), (self.ret['robot_id'], self.ret['home_ref']))
        home = c.live_destinations()[f'home/{SCOUT}']
        self.assertEqual([0.5, 0.25, 0.1], home['xy_yaw'])
        goal = ros_node.fill_execute_task_goal(NS(frame_ref=NS(), route=NS(), approval=NS()), self.ret,
                                               coordinator_id='c')
        f = goal.frame_ref
        self.assertEqual((SCOUT, 'mission', 'mission', MAP, 3, 'VALID'),
                         (f.robot_id, f.native_frame, f.mission_frame, f.map_session_id, f.map_revision,
                          f.alignment_status))
        from rescue_robot.task_wire import frame_key
        frame_key(f)   # the robot's wire check passes (it refused 'native frame' before)

    def test_arrival_alone_is_not_finished_dock_confirmation_is(self):
        c = self.c
        self.arrive()
        self.assertEqual('AT_DOCK_APPROACH', c.scout_return)
        self.assertTrue(c.take_dock_request())
        self.assertFalse(c.take_dock_request())       # once
        c.system_state_change(NOW + 3)
        self.assertNotEqual('FINISHED', c.last_state['phase'])
        self.assertEqual('SCOUT_DOCKING', c.last_state['reason'])
        self.assertTrue(c.on_dock_reply(True, json.dumps({'request': 4})))
        self.assertFalse(c.on_dock_state({'state': 'DOCKED', 'request': 3, 'is_docked': True}))   # older request
        self.assertFalse(c.on_dock_state({'state': 'DOCKED', 'request': 4, 'is_docked': False}))  # not confirmed
        self.assertTrue(c.on_dock_state({'state': 'DOCKED', 'request': 4, 'is_docked': True}))
        c.system_state_change(NOW + 4)
        self.assertEqual(('FINISHED', 'MISSION_FINISHED'), (c.last_state['phase'], c.last_state['mode']))
        out = c.outcome()
        self.assertEqual(('DOCKED', True, 'EXPLORATION_COMPLETE'),
                         (out['scout_dock'], out['no_survivor'], out['exploration_reason']))

    def test_dock_refused_or_timed_out_never_finishes(self):
        for case in ('refused', 'timeout', 'failed'):
            with self.subTest(case=case):
                self.setUp()
                c = self.c
                self.arrive()
                c.take_dock_request()
                if case == 'refused':
                    c.on_dock_reply(False, 'ROBOT_NOT_STOPPED_MOVING')
                elif case == 'failed':
                    c.on_dock_reply(True, json.dumps({'request': 1}))
                    c.on_dock_state({'state': 'FAILED', 'request': 1, 'reason': 'DOCK_RESULT_NOT_DOCKED'})
                else:
                    c.on_dock_reply(True, json.dumps({'request': 1}))
                    c.on_status(status_msg(SCOUT, observed_at_ns=NOW + 70 * S))
                    c.on_scout_map(RealGrid(), now_ns=NOW + 70 * S)
                    c.tick(NOW + 70 * S)
                self.assertEqual('FAILED', c.scout_dock)
                c.system_state_change(NOW + 70 * S)
                self.assertNotEqual('FINISHED', c.last_state['phase'])
                self.assertEqual('SCOUT_DOCK_FAILED', c.last_state['reason'])
                self.assertFalse(c.take_dock_request())   # never asked again


class ExitWaitTests(unittest.TestCase):
    """With a doll: TB-5 approaches, waits, goes to its exit and stays; TB-6 keeps exploring."""

    def core(self, *, homes=True):
        c = make_core(map_mode='SLAM_LIVE', live_map_max_age_ns=10 * S, support_dispatch='AUTO',
                      search_mode='FRONTIER', completion_mode='EXIT_WAIT_SCOUT_DOCK', scout_dock_timeout_ns=60 * S,
                      policy_changes=dict(executable_bases=frozenset({'FRONTIER'})))
        if homes:
            c.mission.homes = {SUPPORT: HomePosition(SUPPORT, (0.0, 1.5), 0.0, MAP, 3, 1)}
        c.on_map_state(map_state(SCOUT, revision=3))
        c.on_start_pose((0.0, 0.0, 0.0))
        c.on_site_event(site_msg())
        c.on_status(status_msg(SCOUT, motion_state='MOVING', active_task_id='scouting'))
        c.on_status(status_msg(SUPPORT))
        c.on_map_state(map_state(SUPPORT, revision=3))
        c.on_pose(support_pose())
        c.on_target_location(location(1), now_ns=NOW)
        for report in c.reports_due(NOW):
            c.on_receipt(receipt(report))
        c.on_scout_map(RealGrid(), now_ns=NOW)
        return c

    def to_exit(self, c):
        (approach,) = [t for t in c.tick(NOW) if t['kind'] == 'APPROACH_TARGET']
        c.on_admission(admission_msg(approach))
        c.on_result(approach['task_id'], DONE)
        c.on_pose(support_pose())
        c.on_scout_map(RealGrid(), now_ns=NOW + 1)
        return [t for t in c.tick(NOW + 1) if t['kind'] == 'RETURN_HOME']

    def test_support_leaves_for_the_profile_exit_before_the_map_is_complete(self):
        c = self.core()
        exits = self.to_exit(c)
        self.assertFalse(c.exploration_complete)     # 방식 B: no MAP_NOT_FINAL wait for TB-6's whole map
        self.assertEqual([SUPPORT], [t['robot_id'] for t in exits])
        self.assertEqual([0.0, 1.5, 0.0], c.live_destinations()[f'home/{SUPPORT}']['xy_yaw'])
        self.assertTrue(any('EXIT_CHOSEN PROFILE_EXIT' in n for n in c.drain_notes()))

    def test_without_a_profile_exit_the_support_start_spot_is_the_exit(self):
        c = self.core(homes=False)
        c.last_tick_ns = NOW
        c.support_start_map_pose = None
        c.on_pose(support_pose((0.2, 3.9)))
        self.to_exit(c)
        self.assertEqual([0.2, 3.9, 0.0], c.live_destinations()[f'home/{SUPPORT}']['xy_yaw'])

    def test_exit_arrival_starts_no_scout_return_and_nothing_more_for_tb5(self):
        c = self.core()
        (exit_task,) = self.to_exit(c)
        c.on_admission(admission_msg(exit_task))
        c.on_result(exit_task['task_id'], DONE)
        self.assertEqual('AT_EXIT', c.support_exit)
        c.on_status(status_msg(SCOUT))
        c.on_status(status_msg(SUPPORT, observed_at_ns=NOW + 2))
        sent = c.tick(NOW + 2)
        self.assertEqual([], [t for t in sent if t['robot_id'] == SUPPORT])            # TB-5 stays
        self.assertEqual([], [t for t in sent if t['kind'] == 'RETURN_HOME'])          # TB-6 not sent home
        self.assertIsNone(c.scout_return)
        c.system_state_change(NOW + 2)
        self.assertNotEqual('FINISHED', c.last_state['phase'])

    def test_frozen_run_keeps_waiting_for_the_final_map(self):
        c = make_core(map_mode='SAVED_MAP')
        self.assertEqual('RECHECK', c.completion_mode)
        with self.assertRaises(ros_node.CoordinationError):
            make_core(completion_mode='EXIT_WAIT_SCOUT_DOCK', scout_dock_timeout_ns=S)   # needs SLAM_LIVE + FRONTIER


class TrafficFixTests(unittest.TestCase):
    def test_step_aside_spot_clears_the_goal_tolerance(self):
        self.assertGreaterEqual(ros_node.STEP_ASIDE_MARGIN_M, 0.25)   # TurtleBot4 xy_goal_tolerance

    def test_frontier_by_an_idle_support_is_dropped_not_waited_for(self):
        c = core_(map_mode='SLAM_LIVE', live_map_max_age_ns=10 * S, traffic_control='TB5_PRIORITY',
                  support_start_pose=(0.0, 1.0, 0.0))
        c.on_map_state(map_state(SCOUT, revision=3))
        c.on_start_pose((0.0, 0.0, 0.0))
        c.on_site_event(site_msg())
        c.on_status(status_msg(SCOUT))
        c.on_status(status_msg(SUPPORT))
        c.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        c.frontier = dict(site_id=uid(5), goal=(0.2, 1.1), centroid=(0.3, 1.5), site_xy=(0.2, 1.1))
        c.frontier_hold_until = NOW - 1
        c.on_scout_map(RealGrid(), now_ns=NOW)
        c.on_pose(scout_pose((-1.0, -1.0), at=NOW))
        c._issue_search(NOW)
        self.assertIsNone(c.frontier)
        self.assertIn((0.2, 1.1), c.frontier_tried)
        self.assertTrue(any('FRONTIER_SKIPPED_NEAR_IDLE_TB5' in n for n in c.drain_notes()))


if __name__ == '__main__':
    unittest.main()
