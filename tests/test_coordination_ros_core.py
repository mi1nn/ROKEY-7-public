"""Mission coordinator node core without ROS: message conversion, SCOUT auto-dispatch, operator service (E2).

The ROS wiring (``ros_node.main``) is exercised by tests/ros/e3_coordinator_probe.py.
"""
import itertools
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination.planning import Allocator
from rescue_coordination.search import SearchCoordinator
from rescue_coordination import mission, ros_node
from test_coordination_search import BOOT, EPOCH, MAP, NOW, policy, uid

SCOUT, SUPPORT = 'unit_1', 'unit_2'
TARGET = uid(500)
MAX_AGE = 2_000_000_000


def time_msg(**changes):
    values = dict(observed_at_ns=NOW - 10**9, valid_until_ns=0, stamp_semantics='CAPTURE',
                  clock_domain_id='site-clock', clock_state='SYNCED', has_clock_error_bound=True,
                  clock_error_bound_ns=5_000_000)
    return NS(**(values | changes))


def site_msg(n=1, **changes):
    values = dict(event_id=uid(300 + n), camera_id='cam_fixed', source_boot_id=uid(9), kind='DISASTER_SIGN',
                  time=time_msg(), target_id='', watch_id='', confidence=.9, visibility_state='VALID',
                  evidence=NS(evidence_id=uid(400 + n), sha256='a' * 64))
    return NS(**(values | changes))


def status_msg(robot=SCOUT, **changes):
    values = dict(robot_id=robot, robot_boot_id=BOOT[robot], observed_at_ns=NOW - 10**8, readiness='READY',
                  lease_valid=True)
    return NS(**(values | changes))


def admission_msg(fields, accepted=True):
    return NS(command_id=fields['command_id'], task_id=fields['task_id'], robot_id=fields['robot_id'],
              robot_boot_id=fields['robot_boot_id'], authority_epoch=fields['authority_epoch'],
              generation=fields['generation'], accepted=accepted, reason_code='' if accepted else 'NAV_NOT_READY')


def request(operation, **changes):
    values = dict(command_id=uid(700), operation=operation, operator_id='admin', robot_id='', target_id='',
                  expected_target_revision=0, route_revision=0, segment_index=0, issued_at_ns=NOW)
    return NS(**(values | changes))


def goal_template():
    frame = NS(robot_id='', native_frame='', mission_frame='', map_session_id='', map_revision=0,
               transform_revision=0, alignment_status='')
    return NS(command_id='', task_id='', robot_id='', expected_robot_boot_id='', coordinator_id='',
              authority_epoch='', generation=0, kind='', issued_at_ns=0, deadline_ns=0, lease_id='', target_id='',
              target_estimate_revision=0, search_area_ref='', home_ref='', frame_ref=frame, has_route=False,
              has_approval=False, policy_version='')


class CoreTestCase(unittest.TestCase):
    def setUp(self):
        self.ids = itertools.count(1000)
        search = SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=(SCOUT, SUPPORT)), policy(),
                                   new_id=lambda: uid(next(self.ids)))
        self.core = ros_node.CoordinatorCore(mission.MissionCoordinator(search, home_positions=()),
                                             status_max_age_ns=MAX_AGE)

    def disaster(self):
        self.assertEqual('ENTERED_DISASTER', self.core.on_site_event(site_msg()).reason)


class ConversionTests(CoreTestCase):
    def test_disaster_sign_message_becomes_a_site_event(self):
        event = ros_node.site_event_from_msg(site_msg(time=time_msg(has_clock_error_bound=False)))
        self.assertEqual(('DISASTER_SIGN', 'cam_fixed', None, None, None, uid(401), 'a' * 64),
                         (event.kind, event.camera_id, event.target_id, event.watch_id, event.clock_error_bound_ns,
                          event.evidence_id, event.evidence_sha256))

    def test_invalid_site_event_messages_are_rejected_not_repaired(self):
        for changes in (dict(visibility_state='OCCLUDED'), dict(visibility_state=''),
                        dict(kind='TARGET_DISAPPEARED_CANDIDATE'), dict(target_id=uid(1)), dict(confidence=2.)):
            with self.subTest(changes=changes), self.assertRaises(RescueError) as caught:
                ros_node.site_event_from_msg(site_msg(**changes))
            self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_execute_task_goal_carries_the_reservation(self):
        self.disaster()
        self.core.on_status(status_msg())
        (fields,) = self.core.tick(NOW)
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        self.assertEqual((fields['task_id'], SCOUT, BOOT[SCOUT], 'coord', 'SEARCH_AREA', fields['search_area_ref'],
                          False, False),
                         (goal.task_id, goal.robot_id, goal.expected_robot_boot_id, goal.coordinator_id, goal.kind,
                          goal.search_area_ref, goal.has_route, goal.has_approval))
        self.assertEqual((SCOUT, 'mission', MAP, 3, 1, ''),
                         (goal.frame_ref.robot_id, goal.frame_ref.mission_frame, goal.frame_ref.map_session_id,
                          goal.frame_ref.map_revision, goal.frame_ref.transform_revision,
                          goal.frame_ref.alignment_status))  # alignment is the mapping's call, never assumed
        self.assertEqual(('', ''), (goal.target_id, goal.lease_id))


class ScoutDispatchTests(CoreTestCase):
    def test_disaster_and_a_ready_scout_give_one_search_goal(self):
        self.assertEqual([], self.core.tick(NOW))  # no status yet
        self.core.on_status(status_msg())
        self.assertEqual([], self.core.tick(NOW))  # NORMAL
        self.disaster()
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('SEARCH_AREA', SCOUT), (fields['kind'], fields['robot_id']))
        self.assertEqual([], self.core.tick(NOW))  # PENDING is kept, not re-sent

    def test_scout_that_is_not_ready_gets_nothing(self):
        self.disaster()
        for changes in (dict(readiness='DEGRADED'), dict(readiness='UNKNOWN'), dict(lease_valid=False),
                        dict(observed_at_ns=NOW - MAX_AGE - 1), dict(observed_at_ns=NOW + 1)):
            with self.subTest(changes=changes):
                self.core.on_status(status_msg(**changes))
                self.assertEqual([], self.core.tick(NOW))

    def test_status_of_an_unknown_robot_is_rejected(self):
        with self.assertRaises(RescueError):
            self.core.on_status(status_msg(robot_id='unit_9'))

    def test_admission_activates_and_completed_result_moves_to_the_next_area(self):
        self.disaster()
        self.core.on_status(status_msg())
        (first,) = self.core.tick(NOW)
        self.assertTrue(self.core.on_admission(admission_msg(first)))
        self.assertIn(first['task_id'], self.core.mission.search.allocator.active)
        self.assertTrue(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=True)))
        (second,) = self.core.tick(NOW)
        self.assertNotEqual(first['search_area_ref'], second['search_area_ref'])

    def test_result_without_confirmed_stop_keeps_the_task(self):
        self.disaster()
        self.core.on_status(status_msg())
        (first,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(first))
        self.assertFalse(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=False)))
        self.assertFalse(self.core.on_result(uid(999), NS(outcome='COMPLETED', stop_confirmed=True)))
        self.assertEqual([], self.core.tick(NOW))

    def test_rejected_admission_frees_the_scout(self):
        self.disaster()
        self.core.on_status(status_msg())
        (first,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(first, accepted=False))
        (again,) = self.core.tick(NOW)
        self.assertNotEqual(first['task_id'], again['task_id'])


class OperatorServiceTests(CoreTestCase):
    def response(self, operation, **changes):
        return self.core.on_command(request(operation, **changes), now_ns=NOW + 1)

    def test_normal_mode_rejects_with_a_closed_reason(self):
        reply = self.response('RETURN_HOME')
        self.assertEqual((False, 'REJECTED', 'MODE_NOT_DISASTER', ''),
                         (reply['accepted'], reply['request_state'], reply['reason_code'], reply['created_task_id']))

    def test_recheck_rejections_use_c11_row_10_codes(self):
        self.disaster()
        self.assertEqual('NOT_FOUND', self.response('REQUEST_RECHECK', target_id=TARGET)['reason_code'])
        self.core.mission.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True)
        self.assertEqual('ACCEPTED', self.response('REQUEST_RECHECK', command_id=uid(701),
                                                   target_id=TARGET)['request_state'])
        self.assertEqual('TASK_BUSY', self.response('REQUEST_RECHECK', command_id=uid(702),
                                                    target_id=TARGET)['reason_code'])

    def test_replay_is_duplicate_with_the_first_answer_and_conflict_is_rejected(self):
        self.disaster()
        first = self.response('RETURN_HOME')
        self.assertEqual((False, 'NO_VERDICT'), (first['accepted'], first['reason_code']))
        again = self.response('RETURN_HOME')
        self.assertEqual(('DUPLICATE', False, ''), (again['request_state'], again['accepted'], again['created_task_id']))
        other = self.response('RETURN_HOME', operator_id='someone-else')
        self.assertEqual(('REJECTED', 'ID_CONTENT_CONFLICT'), (other['request_state'], other['reason_code']))

    def test_dispatch_is_accepted_but_creates_no_task_yet(self):
        self.disaster()
        self.core.mission.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True)
        self.core.mission.record_route(TARGET, route_revision=7, segment_count=2)
        reply = self.response('DISPATCH_SUPPORT', target_id=TARGET, expected_target_revision=0)
        # 작전 Q7 open: no path evidence source for MOVE_SEGMENT, so no Task and no created_task_id.
        self.assertEqual((True, 'ACCEPTED', '', ''),
                         (reply['accepted'], reply['request_state'], reply['reason_code'], reply['created_task_id']))
        self.assertEqual('APPROVED', self.core.mission.segment_phase)

    def test_continue_maps_route_fields(self):
        self.disaster()
        self.core.mission.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True)
        self.core.mission.record_route(TARGET, route_revision=7, segment_count=2)
        self.response('DISPATCH_SUPPORT', target_id=TARGET)
        reply = self.response('CONTINUE_SEGMENT', command_id=uid(701), target_id=TARGET, route_revision=7,
                              segment_index=1)
        self.assertEqual('NOT_AWAITING_APPROVAL', reply['reason_code'])

    def test_malformed_or_unsupported_requests_are_rejected_as_invalid_input(self):
        self.disaster()
        for operation, changes in (('SET_CAMERA_WATCH', {}), ('FINISH_MISSION', {}), ('RESET_STOP', {}),
                                   ('REQUEST_RECHECK', {}), ('REQUEST_RECHECK', dict(target_id='nope')),
                                   ('RETURN_HOME', dict(command_id='nope')), ('RETURN_HOME', dict(issued_at_ns=0)),
                                   ('REQUEST_RECHECK', dict(target_id=TARGET, robot_id=SUPPORT)),
                                   ('DISPATCH_SUPPORT', dict(target_id=TARGET, robot_id=SCOUT))):
            with self.subTest(operation=operation, changes=changes):
                reply = self.response(operation, **changes)
                self.assertEqual((False, 'REJECTED', 'INVALID_INPUT'),
                                 (reply['accepted'], reply['request_state'], reply['reason_code']))

    def test_role_robot_may_be_named(self):
        self.disaster()
        self.core.mission.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True)
        reply = self.response('REQUEST_RECHECK', target_id=TARGET, robot_id=SCOUT)
        self.assertEqual('ACCEPTED', reply['request_state'])


if __name__ == '__main__':
    unittest.main()
