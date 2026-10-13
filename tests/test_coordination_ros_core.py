"""Mission coordinator node core without ROS: message conversion, SCOUT auto-dispatch, operator service (E2).

The ROS wiring (``ros_node.main``) is exercised by tests/ros/e3_coordinator_probe.py.
"""
import itertools
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import CANONICAL_CODES, RescueError
from rescue_coordination.planning import Allocator, GoalPoint
from rescue_coordination.search import SearchCoordinator
from rescue_coordination import mission, ros_node
from test_coordination_search import BOOT, EPOCH, MAP, NOW, policy, uid

SCOUT, SUPPORT = 'unit_1', 'unit_2'
TARGET = uid(500)
MAX_AGE = 2_000_000_000
PERIOD = 2_000_000_000   # lease renewal period; policy() lease_duration is 10 s
LEASE = 10_000_000_000
ACCEPT = 3_000_000_000   # admission timeout
TTL = 1_500_000_000      # CommandLease message time-to-live (AST88-02)
SEGMENTS = tuple(GoalPoint((float(i), 1.), .25 * i, MAP, 3, 1) for i in (1, 2))
OBSERVE = GoalPoint((5., 5.), 1., MAP, 3, 1)
CLOCK_DOMAIN = 'site-clock'
TARGET_AGE = 2_000_000_000     # target_max_age_s 2.0
TARGET_FUTURE = 50_000_000     # target_max_future_s 0.05


def make_core(policy_changes=None, **changes):
    ids = itertools.count(1000)
    search = SearchCoordinator(Allocator(EPOCH, allowed_robot_ids=(SCOUT, SUPPORT)), policy(**(policy_changes or {})),
                               new_id=lambda: uid(next(ids)))
    values = dict(status_max_age_ns=MAX_AGE, segment_goals=SEGMENTS, route_revision=7, observe_goal=OBSERVE,
                  merge_distance_m=1.0, report_retry_ns=1_000_000_000, lease_renew_period_ns=PERIOD,
                  admission_timeout_ns=ACCEPT, segment_passage_free=(True,) * len(SEGMENTS),
                  lease_message_ttl_ns=TTL, camera_watch=None, target_clock_domain_id=CLOCK_DOMAIN,
                  target_max_age_ns=TARGET_AGE, target_max_future_ns=TARGET_FUTURE, support_dispatch='OPERATOR',
                  approach_standoff_m=1.0, support_pose_max_age_ns=MAX_AGE, support_exit_wait_ns=0,
                  recheck_max_clock_error_ns=20_000_000, map_mode='SAVED_MAP', freeze_clearance_m=0.2,
                  map_freeze_timeout_ns=5_000_000_000,
                  search_mode='AREAS', patrol_points=(), patrol_rounds=1, exit_standoff_m=0.5,
                  frontier_min_cells=3, frontier_reach_m=1.0, frontier_goal_max_m=1.0, frontier_blacklist_m=0.5,
                  frontier_issue_delay_ns=2_000_000_000, exploration_empty_checks=2,
                  exploration_timeout_ns=600_000_000_000,
                  search_start='OPERATOR',
                  new_id=lambda: uid(next(ids)))
    return ros_node.CoordinatorCore(mission.MissionCoordinator(search, home_positions=()), **(values | changes))


def frame(**changes):
    values = dict(robot_id=SCOUT, native_frame='map', mission_frame='mission', map_session_id=MAP, map_revision=3,
                  transform_revision=1, alignment_status='VALID')
    return NS(**(values | changes))


def obs_time(**changes):
    """A TargetObservation time with a declared basis on the coordinator's domain (OPS-103 gate)."""
    values = dict(observed_at_ns=NOW - 10**8, valid_until_ns=NOW + 10**9, stamp_semantics='CAPTURE',
                  clock_domain_id=CLOCK_DOMAIN, clock_state='SYNCED', has_clock_error_bound=True,
                  clock_error_bound_ns=5_000_000)
    return NS(**(values | changes))


def location(n, xy=(4., 4.), **changes):
    values = dict(location_id=uid(800 + n), frame_ref=frame(), projection_status='VALID',
                  mission_position=NS(x=xy[0], y=xy[1], z=0.), covariance=[.01 * n] * 9,
                  observation=NS(time=obs_time(observed_at_ns=NOW - 10**8 + n, valid_until_ns=NOW + 10**9 + n),
                                 class_label='survivor'))
    return NS(**(values | changes))


def receipt(report, state='PERSISTED', **changes):
    values = dict(report_id=report['report_id'], included_record_ids=list(report['record_ids']), state=state)
    return NS(**(values | changes))


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
                  lease_valid=True, stop_reason='', motion_state='STOPPED', active_task_id='')
    return NS(**(values | changes))


def admission_msg(fields, accepted=True):
    return NS(command_id=fields['command_id'], task_id=fields['task_id'], robot_id=fields['robot_id'],
              robot_boot_id=fields['robot_boot_id'], authority_epoch=fields['authority_epoch'],
              generation=fields['generation'], accepted=accepted, reason_code='' if accepted else 'NAV_NOT_READY')


def request(operation, **changes):
    values = dict(command_id=uid(700), operation=operation, operator_id='admin', robot_id='', target_id='',
                  expected_target_revision=0, route_revision=0, segment_index=0, issued_at_ns=NOW,
                  roi_xyxy_normalized=[0.] * 4)
    return NS(**(values | changes))


def frame_template():
    return NS(robot_id='', native_frame='', mission_frame='', map_session_id='', map_revision=0, transform_revision=0,
              alignment_status='')


def goal_template():
    frame = frame_template()
    pose = NS(header=NS(frame_id=''), pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
    route = NS(route_id='', route_revision=0, robot_id='', target_id='', segment_index=0, frame_ref=frame_template(),
               approach_goal=pose, generated_at_ns=0, valid_until_ns=0, path_sha256='', approval_corridor_sha256='',
               planner_evidence_id='', policy_version='',
               assessment=NS(status='', width_known=False, height_known=False, floor_geometry_known=False))
    approval = NS(approval_id='', command_id='', operator_id='', robot_id='', robot_boot_id='', authority_epoch='',
                  target_id='', target_estimate_revision=0, route_id='', route_revision=0, segment_index=0,
                  approved_at_ns=0, valid_until_ns=0, scope='')
    return NS(command_id='', task_id='', robot_id='', expected_robot_boot_id='', coordinator_id='',
              authority_epoch='', generation=0, kind='', issued_at_ns=0, deadline_ns=0, lease_id='', target_id='',
              target_estimate_revision=0, search_area_ref='', home_ref='', frame_ref=frame, route=route,
              has_route=False, approval=approval, has_approval=False, policy_version='')


class CoreTestCase(unittest.TestCase):
    def setUp(self):
        self.core = make_core()

    def disaster(self):
        """DISASTER_SIGN only; AMR1 waits for START_SEARCH (owner decision 2026-10-11, #85)."""
        self.assertEqual('ENTERED_DISASTER', self.core.on_site_event(site_msg()).reason)

    def start_search(self, **status):
        """A ready SCOUT, then the operator's START_SEARCH (accepted only when the SCOUT is ready)."""
        self.core.on_status(status_msg(**status))
        reply = self.core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        self.assertEqual('ACCEPTED', reply['request_state'])


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
        self.start_search()
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
        self.assertEqual(('', fields['lease_id']), (goal.target_id, goal.lease_id))


class ScoutDispatchTests(CoreTestCase):
    def test_disaster_alone_starts_no_search(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        self.assertEqual([], self.core.tick(NOW))
        self.assertEqual('DISASTER', self.core.mission.phase())

    def test_start_search_issues_the_first_area_at_once(self):
        self.core.on_status(status_msg())
        self.core.on_site_event(site_msg())
        reply = self.core.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        self.assertEqual((True, fields['task_id'], 'SEARCH_AREA'),
                         (reply['accepted'], reply['created_task_id'], fields['kind']))
        again = self.core.on_command(request('START_SEARCH', command_id=uid(691)), now_ns=NOW)
        self.assertEqual(('REJECTED', 'TASK_BUSY'), (again['request_state'], again['reason_code']))
        wrong = self.core.on_command(request('START_SEARCH', command_id=uid(692), robot_id=SUPPORT), now_ns=NOW)
        self.assertEqual('INVALID_INPUT', wrong['reason_code'])

    def test_start_search_before_the_scout_is_ready_is_refused_and_not_counted(self):
        # 작전 2026-10-11 (#98): never a robot leaving at an unknown moment after the button.
        self.core.on_site_event(site_msg())
        for status in (None, dict(readiness='DEGRADED'), dict(observed_at_ns=NOW - MAX_AGE - 1)):
            with self.subTest(status=status):
                if status is not None:
                    self.core.on_status(status_msg(**status))
                reply = self.core.on_command(request('START_SEARCH', command_id=uid(680 + len(str(status)))),
                                             now_ns=NOW)
                self.assertEqual((False, 'REJECTED', 'NAV_NOT_READY', ''),
                                 (reply['accepted'], reply['request_state'], reply['reason_code'],
                                  reply['created_task_id']))
                self.assertEqual([], self.core.tick(NOW))
        self.core.on_status(status_msg())
        reply = self.core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('ACCEPTED', fields['task_id'], 'SEARCH_AREA'),
                         (reply['request_state'], reply['created_task_id'], fields['kind']))

    def queued(self):
        """START_SEARCH accepted while the ready SCOUT is busy with a recheck: the search waits (#99)."""
        self.disaster()
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.core.on_status(status_msg())
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(701), target_id=target), now_ns=NOW)
        (recheck,) = self.core.tick(NOW)
        reply = self.core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        self.assertEqual((True, ''), (reply['accepted'], reply['created_task_id']))
        return recheck

    def finish(self, fields, outcome='COMPLETED'):
        self.core.on_admission(admission_msg(fields))
        self.assertTrue(self.core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True)))

    def test_waiting_search_is_shown_then_issued_after_the_current_task(self):
        recheck = self.queued()
        self.assertEqual('SEARCH_QUEUED', self.core.system_state_change(NOW)['reason'])
        self.finish(recheck)
        (search,) = self.core.tick(NOW)
        self.assertEqual('SEARCH_AREA', search['kind'])
        # The recheck completed without an observation: judged RECHECK_NEEDED (C-11 row 5), shown in place of the queue.
        self.assertEqual('RECHECK_NEEDED_RECHECK_OBSERVATION_MISSING', self.core.system_state_change(NOW)['reason'])

    def test_stop_cancels_the_waiting_search(self):
        for stop in ('status', 'canceled', 'mode'):
            with self.subTest(stop=stop):
                self.setUp()
                recheck = self.queued()
                if stop == 'status':
                    self.core.on_status(status_msg(motion_state='STOPPING', active_task_id=recheck['task_id']))
                    self.core.tick(NOW)
                    self.assertFalse(self.core.search_queued())  # withdrawn by the stopping status alone
                    self.finish(recheck)
                elif stop == 'canceled':
                    self.finish(recheck, 'CANCELED')  # the executor ends an operator stop as CANCELED
                else:
                    self.core.mission.mode = 'NORMAL'  # disaster mode cleared (CLEARING is not built yet)
                    self.core.tick(NOW)
                    self.core.mission.mode = 'DISASTER'
                    self.finish(recheck)
                self.assertEqual([], [f for f in self.core.tick(NOW) if f['kind'] == 'SEARCH_AREA'])
                self.assertNotEqual('SEARCH_QUEUED', self.core.system_state_change(NOW)['reason'])
                again = self.core.on_command(request('START_SEARCH', command_id=uid(691)), now_ns=NOW)
                self.assertEqual('ACCEPTED', again['request_state'])  # the operator presses again

    def test_disaster_and_a_ready_scout_give_one_search_goal(self):
        self.assertEqual([], self.core.tick(NOW))  # no status yet
        self.core.on_status(status_msg())
        self.assertEqual([], self.core.tick(NOW))  # NORMAL
        self.disaster()
        self.assertEqual([], self.core.tick(NOW))  # no START_SEARCH yet
        self.core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('SEARCH_AREA', SCOUT), (fields['kind'], fields['robot_id']))
        self.assertEqual([], self.core.tick(NOW))  # PENDING is kept, not re-sent

    def test_scout_that_is_not_ready_gets_nothing(self):
        self.disaster()
        for changes in (dict(readiness='DEGRADED'), dict(readiness='UNKNOWN'),
                        dict(observed_at_ns=NOW - MAX_AGE - 1), dict(observed_at_ns=NOW + 1)):
            with self.subTest(changes=changes):
                self.core.on_status(status_msg(**changes))
                self.assertEqual([], self.core.tick(NOW))

    def test_idle_scout_without_a_lease_gets_the_first_search_goal(self):
        # An idle robot holds no CommandLease (leases are per task), so lease_valid is false
        # until its first task; requiring it would never issue that task (#100).
        self.disaster()
        self.start_search(lease_valid=False)  # START_SEARCH is accepted, not NAV_NOT_READY
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('SEARCH_AREA', SCOUT), (fields['kind'], fields['robot_id']))

    def test_status_of_an_unknown_robot_is_rejected(self):
        with self.assertRaises(RescueError):
            self.core.on_status(status_msg(robot_id='unit_9'))

    def test_admission_activates_and_completed_result_moves_to_the_next_area(self):
        self.disaster()
        self.start_search()
        (first,) = self.core.tick(NOW)
        self.assertTrue(self.core.on_admission(admission_msg(first)))
        self.assertIn(first['task_id'], self.core.mission.search.allocator.active)
        self.assertTrue(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=True)))
        (second,) = self.core.tick(NOW)
        self.assertNotEqual(first['search_area_ref'], second['search_area_ref'])

    def test_result_without_confirmed_stop_keeps_the_task(self):
        self.disaster()
        self.start_search()
        (first,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(first))
        self.assertFalse(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=False)))
        self.assertFalse(self.core.on_result(uid(999), NS(outcome='COMPLETED', stop_confirmed=True)))
        self.assertEqual([], self.core.tick(NOW))

    def test_rejected_admission_frees_the_scout(self):
        self.disaster()
        self.start_search()
        (first,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(first, accepted=False))
        (again,) = self.core.tick(NOW)
        self.assertNotEqual(first['task_id'], again['task_id'])


class OperatorServiceTests(CoreTestCase):
    def response(self, operation, **changes):
        return self.core.on_command(request(operation, **changes), now_ns=NOW + 1)

    def test_normal_mode_rejects_with_a_closed_reason(self):
        reply = self.response('RETURN_HOME')
        self.assertEqual((False, 'REJECTED', 'NOT_AUTHORIZED', ''),
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
        self.assertEqual((False, 'PENDING_EVIDENCE'), (first['accepted'], first['reason_code']))
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
        self.assertEqual('TASK_BUSY', reply['reason_code'])

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

    def test_every_refusal_maps_to_a_c09_code(self):
        # OPS-58 (2026-10-11): reason_code carries the common code at the node boundary.
        self.assertEqual(set(mission.COMMAND_REJECT_REASONS), set(ros_node.REASON_CODES))
        self.assertLessEqual(set(ros_node.REASON_CODES.values()), CANONICAL_CODES)
        for reason, code in (('UNKNOWN_TARGET', 'NOT_FOUND'), ('TARGET_ALREADY_JUDGED', 'INVALID_INPUT'),
                             ('RECHECK_ALREADY_REQUESTED', 'TASK_BUSY')):  # C-11 row 10
            self.assertEqual(code, ros_node.REASON_CODES[reason])

    def test_role_robot_may_be_named(self):
        self.disaster()
        self.core.mission.record_target(TARGET, 0, detection_confirmed=True, report_persisted=True)
        reply = self.response('REQUEST_RECHECK', target_id=TARGET, robot_id=SCOUT)
        self.assertEqual('ACCEPTED', reply['request_state'])



class TaskIssueTests(CoreTestCase):
    """작전 2026-10-11 (#88 Phase 1 rule): approvals and rechecks become ExecuteTask goals."""

    def setUp(self):
        super().setUp()
        self.disaster()
        self.target = self.core.on_target_location(location(1), now_ns=NOW)  # route recorded on creation
        (report,) = self.core.reports_due(NOW)
        self.assertTrue(self.core.on_receipt(receipt(report)))

    def command(self, operation, n, **fields):
        return self.core.on_command(request(operation, command_id=uid(700 + n), target_id=self.target, **fields),
                                    now_ns=NOW + n)

    def dispatch(self):
        self.core.on_status(status_msg(SUPPORT))
        reply = self.command('DISPATCH_SUPPORT', 1)
        (fields,) = self.core.tick(NOW)
        self.assertEqual((True, fields['task_id']), (reply['accepted'], reply['created_task_id']))
        return fields

    def finish(self, fields, outcome='COMPLETED'):
        self.assertTrue(self.core.on_admission(admission_msg(fields)))
        self.assertTrue(self.core.on_result(fields['task_id'], NS(outcome=outcome, stop_confirmed=True)))

    def test_dispatch_issues_one_move_segment_with_route_and_approval(self):
        fields = self.dispatch()
        self.assertEqual(('MOVE_SEGMENT', SUPPORT, self.target, 0),
                         (fields['kind'], fields['robot_id'], fields['target_id'], fields['target_revision']))
        route, approval = fields['route'], fields['approval']
        self.assertEqual((7, 0, SEGMENTS[0], ros_node.route_id(SEGMENTS)),
                         (route['route_revision'], route['segment_index'], route['approach_goal'], route['route_id']))
        self.assertEqual((uid(701), uid(701), 'admin', 'ONE_SEGMENT', 0, NOW + 1),
                         (approval['approval_id'], approval['command_id'], approval['operator_id'], approval['scope'],
                          approval['segment_index'], approval['approved_at_ns']))
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        self.assertEqual((True, True, 'ONE_SEGMENT', 1.0, '', ''),
                         (goal.has_route, goal.has_approval, goal.approval.scope, goal.route.approach_goal.pose.position.x,
                          goal.route.planner_evidence_id, goal.route.path_sha256))  # no planner-looking values
        self.assertEqual([], self.core.tick(NOW))  # one approval, one task

    def test_idle_support_without_a_lease_gets_the_segment(self):
        self.core.on_status(status_msg(SUPPORT, lease_valid=False))
        reply = self.command('DISPATCH_SUPPORT', 1)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('MOVE_SEGMENT', reply['created_task_id']), (fields['kind'], fields['task_id']))

    def test_segment_route_keeps_the_time_the_route_was_made(self):
        # The route was made when the target was created (setUp, NOW); a later issue keeps that time.
        self.core.on_status(status_msg(SUPPORT))
        self.command('DISPATCH_SUPPORT', 1)
        (fields,) = self.core.tick(NOW + 10**9)
        self.assertEqual(NOW, fields['route']['generated_at_ns'])
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        self.assertEqual(NOW, goal.route.generated_at_ns)
        self.assertLess(goal.route.generated_at_ns, goal.route.valid_until_ns)

    def test_segment_waits_for_a_known_support_and_is_issued_later(self):
        reply = self.command('DISPATCH_SUPPORT', 1)
        self.assertEqual((True, ''), (reply['accepted'], reply['created_task_id']))
        self.assertEqual([], self.core.tick(NOW))
        self.core.on_status(status_msg(SUPPORT))
        (fields,) = self.core.tick(NOW)
        self.assertEqual('MOVE_SEGMENT', fields['kind'])

    def test_admission_and_stopped_result_drive_the_segment_state(self):
        first = self.dispatch()
        self.assertTrue(self.core.on_admission(admission_msg(first)))
        self.assertEqual('MOVING', self.core.mission.segment_phase)
        self.assertTrue(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=True)))
        self.assertEqual(('AWAITING_APPROVAL', 1), (self.core.mission.segment_phase, self.core.mission.segment_index))
        self.assertEqual([], self.core.tick(NOW))  # never the next segment by itself
        reply = self.command('CONTINUE_SEGMENT', 2, route_revision=7, segment_index=1)
        (second,) = self.core.tick(NOW)
        self.assertEqual((reply['created_task_id'], SEGMENTS[1], 1), (second['task_id'], second['route']['approach_goal'],
                                                                     second['route']['segment_index']))

    def test_failed_segment_waits_for_approval_again(self):
        first = self.dispatch()
        self.finish(first, 'UNREACHABLE')
        self.assertEqual(('AWAITING_APPROVAL', 0), (self.core.mission.segment_phase, self.core.mission.segment_index))
        self.assertEqual([], self.core.tick(NOW))

    def test_result_without_a_confirmed_stop_changes_nothing(self):
        first = self.dispatch()
        self.core.on_admission(admission_msg(first))
        self.assertFalse(self.core.on_result(first['task_id'], NS(outcome='COMPLETED', stop_confirmed=False)))
        self.assertEqual('MOVING', self.core.mission.segment_phase)

    def test_recheck_issues_reobserve_at_the_observation_goal(self):
        self.core.on_status(status_msg(SCOUT))
        reply = self.command('REQUEST_RECHECK', 3)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('REOBSERVE_TARGET', SCOUT, self.target, fields['task_id'], None),
                         (fields['kind'], fields['robot_id'], fields['target_id'], reply['created_task_id'],
                          fields['approval']))
        self.assertEqual((OBSERVE.mission_xy[0], OBSERVE.yaw), (fields['goal_pose']['x'], fields['goal_pose']['yaw']))

    def test_recheck_waits_for_the_current_search_area_then_goes_first(self):
        self.start_search()
        (search,) = self.core.tick(NOW)
        self.assertEqual('', self.command('REQUEST_RECHECK', 3)['created_task_id'])  # SCOUT is searching
        self.finish(search)
        (fields,) = self.core.tick(NOW)
        self.assertEqual('REOBSERVE_TARGET', fields['kind'])  # before the next search area

    def test_recheck_that_did_not_complete_is_reissued_once(self):
        self.core.on_status(status_msg(SCOUT))
        self.command('REQUEST_RECHECK', 3)
        (first,) = self.core.tick(NOW)
        # The SCOUT is ready but busy rechecking: START_SEARCH is accepted and the search follows the recheck.
        self.assertEqual('', self.core.on_command(request('START_SEARCH', command_id=uid(690)),
                                                  now_ns=NOW)['created_task_id'])
        self.finish(first, 'FAILED')
        (again,) = self.core.tick(NOW)
        self.assertEqual('REOBSERVE_TARGET', again['kind'])
        self.assertNotEqual(first['task_id'], again['task_id'])
        self.finish(again)
        (search,) = self.core.tick(NOW)  # completed: the recheck waits for its verdict, the SCOUT searches on
        self.assertEqual('SEARCH_AREA', search['kind'])

    def test_core_rejects_a_route_without_segments_or_with_another_basis(self):
        for changes in (dict(segment_goals=()), dict(segment_goals=(OBSERVE, 'x')), dict(merge_distance_m=0.),
                        dict(merge_distance_m=float('nan')), dict(report_retry_ns=0), dict(admission_timeout_ns=0)):
            with self.subTest(changes=changes), self.assertRaises(RescueError):
                make_core(**changes)



class CommandLeaseTests(CoreTestCase):
    """작전 2026-10-11: renew every task's CommandLease; stop renewing and let the executor stop itself."""

    def issued(self):
        self.disaster()
        self.start_search()
        (fields,) = self.core.tick(NOW)
        return fields

    def test_each_task_gets_a_lease_renewed_with_rising_sequence(self):
        fields = self.issued()
        (lease,) = self.core.leases_due(NOW)
        self.assertEqual((fields['lease_id'], SCOUT, BOOT[SCOUT], fields['authority_epoch'], fields['generation'], 1,
                          NOW, NOW + TTL),
                         (lease['lease_id'], lease['robot_id'], lease['robot_boot_id'], lease['authority_epoch'],
                          lease['generation'], lease['renewal_sequence'], lease['issued_at_ns'],
                          lease['valid_until_ns']))
        self.core.on_status(status_msg(observed_at_ns=NOW + PERIOD - 10))  # keep the robot's status fresh
        self.assertEqual([], self.core.leases_due(NOW + PERIOD - 1))
        (again,) = self.core.leases_due(NOW + PERIOD)
        self.assertEqual((fields['lease_id'], 2), (again['lease_id'], again['renewal_sequence']))

    def test_finished_or_refused_task_is_not_renewed(self):
        fields = self.issued()
        self.core.on_admission(admission_msg(fields))
        self.core.on_result(fields['task_id'], NS(outcome='COMPLETED', stop_confirmed=True))
        self.assertEqual([], [l for l in self.core.leases_due(NOW) if l['lease_id'] == fields['lease_id']])
        (second,) = self.core.tick(NOW)
        self.core.on_admission(admission_msg(second, accepted=False))
        self.assertEqual([], [l for l in self.core.leases_due(NOW) if l['lease_id'] == second['lease_id']])

    def test_stale_status_stops_renewal_for_good(self):
        fields = self.issued()
        self.core.leases_due(NOW)
        self.assertEqual([], self.core.leases_due(NOW + MAX_AGE + PERIOD))  # status from NOW - 0.1 s is stale
        self.core.on_status(status_msg(observed_at_ns=NOW + MAX_AGE + PERIOD))
        self.assertEqual([], self.core.leases_due(NOW + MAX_AGE + 2 * PERIOD))  # never silently extended again
        self.assertNotIn(fields['lease_id'], {l['lease_id'] for l in self.core.leases_due(NOW + 10 * PERIOD)})

    def test_new_robot_boot_stops_renewal(self):
        self.issued()
        self.core.on_status(status_msg(robot_boot_id=uid(77)))
        self.assertEqual([], self.core.leases_due(NOW))

    def test_segment_lease_is_bound_to_the_support_task(self):
        self.disaster()
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.core.on_status(status_msg(SUPPORT))
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=target), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        (lease,) = self.core.leases_due(NOW)
        self.assertEqual((SUPPORT, fields['generation'], fields['lease_id']),
                         (lease['robot_id'], lease['generation'], lease['lease_id']))

    def admitted(self):
        fields = self.issued()
        self.assertTrue(self.core.on_admission(admission_msg(fields)))
        return fields

    def moving(self, fields, **changes):
        values = dict(active_task_id=fields['task_id'], motion_state='MOVING')
        return status_msg(**(values | changes))

    def test_stopped_robot_gets_no_more_renewals(self):
        # 작전 2026-10-11: a supporting rule; the stop itself is the executor latch.
        for changes in (dict(stop_reason='OPERATOR_STOP'), dict(motion_state='STOPPING'),
                        dict(motion_state='STOP_UNCONFIRMED'), dict(lease_valid=False)):
            with self.subTest(changes=changes):
                self.setUp()
                fields = self.admitted()
                self.core.on_status(self.moving(fields))
                self.assertEqual(1, len(self.core.leases_due(NOW)))
                self.core.on_status(self.moving(fields, observed_at_ns=NOW + PERIOD - 10, **changes))  # fresh
                self.assertEqual([], self.core.leases_due(NOW + PERIOD))
                self.core.on_status(self.moving(fields, observed_at_ns=NOW + 2 * PERIOD - 10))
                self.assertEqual([], self.core.leases_due(NOW + 2 * PERIOD))  # not resumed

    def test_status_naming_another_task_after_admission_stops_renewal(self):
        fields = self.admitted()
        self.core.on_status(self.moving(fields, active_task_id=uid(42)))
        self.assertEqual([], self.core.leases_due(NOW))

    def test_status_before_admission_does_not_cut_a_new_lease(self):
        fields = self.issued()  # the executor has not admitted it: no active task, no lease yet
        self.core.on_status(status_msg(lease_valid=False, stop_reason='COMPLETED', active_task_id=uid(42)))
        self.assertEqual(fields['lease_id'], self.core.leases_due(NOW)[0]['lease_id'])
        self.core.on_admission(admission_msg(fields))
        self.core.on_status(self.moving(fields, observed_at_ns=NOW + 1))
        self.assertEqual(1, len(self.core.leases_due(NOW + PERIOD)))

    def test_task_never_admitted_stops_renewing_and_is_released_on_stop_evidence(self):
        # 작전 2026-10-11: no lease renewed forever for a task the executor never admitted.
        fields = self.issued()
        late = NOW + ACCEPT + 1
        self.core.on_status(status_msg(observed_at_ns=late, motion_state='MOVING', active_task_id=fields['task_id']))
        self.assertEqual([], self.core.leases_due(late))
        self.assertEqual([], self.core.tick(late))  # still reserved: the robot may be running it
        self.assertIn(fields['task_id'], self.core.search.allocator.pending)
        self.core.on_status(status_msg(observed_at_ns=late + 1))  # stopped, no active task
        (again,) = self.core.tick(late + 1)
        self.assertNotIn(fields['task_id'], self.core.search.allocator.pending)
        self.assertEqual((fields['search_area_ref'], 'SEARCH_AREA'), (again['search_area_ref'], again['kind']))
        self.assertNotEqual(fields['task_id'], again['task_id'])

    def test_admission_in_time_keeps_the_lease(self):
        fields = self.admitted()
        self.core.on_status(self.moving(fields, observed_at_ns=NOW + ACCEPT + 1))
        self.assertEqual(1, len(self.core.leases_due(NOW + ACCEPT + 1)))

    def test_period_must_be_at_most_half_the_lease(self):
        with self.assertRaises(RescueError):
            make_core(lease_renew_period_ns=LEASE // 2 + 1)
        make_core(lease_renew_period_ns=LEASE // 2)


if __name__ == '__main__':
    unittest.main()
