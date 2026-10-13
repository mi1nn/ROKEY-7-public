"""Coordinator core: target intake and merge, report until PERSISTED, route frame, SystemState (E2).

작전 2026-10-11 (#88): target_id is the coordinator's discovery UUID and the revision its own
counter; a new VALID TargetLocation within ``merge_distance_m`` on the same map basis is the
same target. Saved = ReportReceipt PERSISTED for the coordinator's RescueReport (IF-072 -> 073).
"""
from operator import itemgetter
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from test_coordination_ros_core import (OBSERVE, SEGMENTS, SCOUT, SUPPORT, frame, goal_template, location,
                                        make_core, receipt, request, site_msg, status_msg)
from test_coordination_search import MAP, NOW, uid

MERGE = 1.0
RETRY = 1_000_000_000


class TargetTestCase(unittest.TestCase):
    def setUp(self):
        self.core = make_core()

    def saved(self, n=1, xy=(4., 4.)):
        target = self.core.on_target_location(location(n, xy), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.assertTrue(self.core.on_receipt(receipt(report)))
        return target


class IntakeTests(TargetTestCase):
    def test_first_valid_location_makes_a_target_and_a_report(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        self.assertEqual((0, True, False), self.core.mission.targets[target])
        (report,) = self.core.reports_due(NOW)
        self.assertEqual((target, 0, [uid(801)]), (report['target_id'], report['target_revision'],
                                                   report['record_ids']))
        self.assertEqual([uid(801)], [loc.location_id for loc in report['locations']])
        self.assertEqual([], self.core.reports_due(NOW + RETRY - 1))
        (again,) = self.core.reports_due(NOW + RETRY)  # resent until PERSISTED
        self.assertEqual(report['report_id'], again['report_id'])

    def test_persisted_receipt_saves_the_revision_and_stops_resending(self):
        target = self.saved()
        self.assertEqual((0, True, True), self.core.mission.targets[target])
        self.assertEqual([], self.core.reports_due(NOW + 10 * RETRY))
        self.core.on_site_event(site_msg())
        self.assertEqual('TARGET_FOUND', self.core.mission.phase())

    def test_receipts_that_do_not_cover_the_report_save_nothing(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        for changes in (dict(report_id=uid(999)), dict(included_record_ids=[]), dict(state='RECEIVED'),
                        dict(state='PENDING_EVIDENCE')):
            with self.subTest(changes=changes):
                self.assertFalse(self.core.on_receipt(receipt(report, **changes)))
        self.assertEqual((0, True, False), self.core.mission.targets[target])

    def test_rejected_report_is_not_resent_and_not_saved(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.assertFalse(self.core.on_receipt(receipt(report, state='REJECTED')))
        self.assertEqual([], self.core.reports_due(NOW + 10 * RETRY))
        self.assertEqual((0, True, False), self.core.mission.targets[target])

    def test_nearby_location_on_the_same_basis_is_the_same_target(self):
        target = self.saved()
        self.assertEqual(target, self.core.on_target_location(location(2, (4.5, 4.)), now_ns=NOW))
        self.assertEqual((1, True, False), self.core.mission.targets[target])  # the new revision needs its own save
        (report,) = self.core.reports_due(NOW)
        # 801 is already in a PERSISTED receipt: the new report carries only the unsaved location.
        self.assertEqual((1, [uid(802)], [uid(802)]), (report['target_revision'], report['record_ids'],
                                                       [loc.location_id for loc in report['locations']]))

    def test_unsaved_locations_ride_the_newest_report_and_the_older_one_stops(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (first,) = self.core.reports_due(NOW)
        self.core.on_target_location(location(2, (4.5, 4.)), now_ns=NOW)
        (second,) = self.core.reports_due(NOW)  # the revision-0 report is covered and no longer resent
        self.assertEqual((1, [uid(801), uid(802)]), (second['target_revision'], second['record_ids']))
        self.assertFalse(self.core.on_receipt(receipt(first)))  # a late receipt of the covered report saves nothing
        self.assertEqual((1, True, False), self.core.mission.targets[target])
        self.assertTrue(self.core.on_receipt(receipt(second)))
        self.assertEqual((1, True, True), self.core.mission.targets[target])
        self.assertEqual([], self.core.reports_due(NOW + 10 * RETRY))

    def test_report_size_does_not_grow_with_the_number_of_saved_locations(self):
        for n in range(1, 41):  # #137 measured: all-locations reports failed from the 34th location at 64 KiB
            self.core.on_target_location(location(n, (4. + n * 1e-3, 4.)), now_ns=NOW + n)
            (report,) = self.core.reports_due(NOW + n)
            self.assertEqual([uid(800 + n)], report['record_ids'])
            self.assertTrue(self.core.on_receipt(receipt(report)))

    def test_far_location_is_a_new_target(self):
        target = self.saved()
        other = self.core.on_target_location(location(2, (4. + MERGE + .01, 4.)), now_ns=NOW)
        self.assertNotEqual(target, other)
        self.assertEqual((0, True, False), self.core.mission.targets[other])

    def test_location_on_another_map_basis_is_refused_not_split(self):
        self.saved()
        for changes in (dict(map_revision=4), dict(map_session_id=uid(6)), dict(transform_revision=2)):
            with self.subTest(changes=changes), self.assertRaises(RescueError) as caught:
                self.core.on_target_location(location(2, frame_ref=frame(**changes)), now_ns=NOW)
            self.assertEqual('FRAME_REVISION_MISMATCH', caught.exception.code)
        self.assertEqual(1, len(self.core.mission.targets))

    def test_only_valid_projections_enter(self):
        for status in ('STALE_DERIVATION', 'NEEDS_REOBSERVATION', 'EXPIRED', ''):
            with self.subTest(status=status), self.assertRaises(RescueError):
                self.core.on_target_location(location(1, projection_status=status), now_ns=NOW)
        self.assertEqual({}, self.core.mission.targets)

    def test_location_replay_is_idempotent_and_conflict_is_rejected(self):
        target = self.saved()
        self.assertEqual(target, self.core.on_target_location(location(1), now_ns=NOW))
        self.assertEqual((0, True, True), self.core.mission.targets[target])
        with self.assertRaises(RescueError) as caught:
            self.core.on_target_location(location(1, (4.2, 4.)), now_ns=NOW)
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)


class RouteFrameTests(TargetTestCase):
    def setUp(self):
        super().setUp()
        self.core.on_site_event(site_msg())
        self.target = self.saved()

    def send(self, operation, n, **fields):
        return self.core.on_command(request(operation, command_id=uid(700 + n), target_id=self.target, **fields),
                                    now_ns=NOW + n)

    def test_segment_route_uses_the_target_frame_and_unknown_assessment(self):
        self.core.on_status(status_msg(SUPPORT))
        reply = self.send('DISPATCH_SUPPORT', 1)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(reply['created_task_id'], fields['task_id'])
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        r = goal.route
        self.assertEqual((MAP, 3, 1, 'VALID', 'mission', 'mission'),
                         (r.frame_ref.map_session_id, r.frame_ref.map_revision, r.frame_ref.transform_revision,
                          r.frame_ref.alignment_status, r.frame_ref.mission_frame, r.approach_goal.header.frame_id))
        self.assertEqual(('UNKNOWN', False, False, False),
                         (r.assessment.status, r.assessment.width_known, r.assessment.height_known,
                          r.assessment.floor_geometry_known))
        self.assertEqual((fields['deadline_ns'], fields['deadline_ns']), (r.valid_until_ns, goal.approval.valid_until_ns))
        self.assertEqual((goal.robot_id, goal.expected_robot_boot_id, goal.authority_epoch, r.route_id),
                         (goal.approval.robot_id, goal.approval.robot_boot_id, goal.approval.authority_epoch,
                          goal.approval.route_id))

    def test_no_segment_when_the_goal_basis_differs_from_the_target_frame(self):
        self.core = make_core(segment_goals=tuple(ros_node.GoalPoint(g.mission_xy, g.yaw, MAP, 4, 1) for g in SEGMENTS))
        self.core.on_site_event(site_msg())
        self.target = self.saved()
        self.core.on_status(status_msg(SUPPORT))
        reply = self.send('DISPATCH_SUPPORT', 1)
        self.assertEqual((True, ''), (reply['accepted'], reply['created_task_id']))
        self.assertEqual([], self.core.tick(NOW))
        self.assertEqual('ROUTE_BASIS_MISMATCH', self.core.issue_block)

    def test_no_segment_while_the_target_alignment_is_not_valid(self):
        self.core = make_core()
        self.core.on_site_event(site_msg())
        target = self.core.on_target_location(location(1, frame_ref=frame(alignment_status='DEGRADED')), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))
        self.core.on_status(status_msg(SUPPORT))
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=target), now_ns=NOW)
        self.assertEqual([], self.core.tick(NOW))
        self.assertEqual('TARGET_ALIGNMENT_NOT_VALID', self.core.issue_block)

    def test_blocked_recheck_does_not_stop_the_search(self):
        self.core = make_core()
        self.core.on_site_event(site_msg())
        target = self.core.on_target_location(location(1, frame_ref=frame(alignment_status='DEGRADED')), now_ns=NOW)
        self.core.on_status(status_msg(SCOUT))
        self.core.on_command(request('REQUEST_RECHECK', command_id=uid(701), target_id=target), now_ns=NOW)
        self.core.on_command(request('START_SEARCH', command_id=uid(690)), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('SEARCH_AREA', 'TARGET_ALIGNMENT_NOT_VALID'), (fields['kind'], self.core.issue_block))

    def test_reobserve_carries_the_observation_point_as_route_without_approval(self):
        self.core.on_status(status_msg(SCOUT))
        self.send('REQUEST_RECHECK', 3)
        (fields,) = self.core.tick(NOW)
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        self.assertEqual((True, False, OBSERVE.mission_xy[0], 'UNKNOWN'),
                         (goal.has_route, goal.has_approval, goal.route.approach_goal.pose.position.x,
                          goal.route.assessment.status))
        self.assertEqual(NOW, goal.route.generated_at_ns)  # made with the target, not at sending


class DispatchGateTests(TargetTestCase):
    """B-stage gate: CONFIRMED + ReportReceipt PERSISTED of the current revision opens DISPATCH_SUPPORT (S-00)."""

    def setUp(self):
        super().setUp()
        self.core.on_site_event(site_msg())
        self.core.on_status(status_msg(SUPPORT))

    def dispatch(self, n, target, target_revision):
        return self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(700 + n), target_id=target,
                                            expected_target_revision=target_revision), now_ns=NOW + n)

    def test_receipt_persisted_names_the_report_and_opens_dispatch(self):
        target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.assertEqual((False, 'PENDING_EVIDENCE'), itemgetter('accepted', 'reason_code')(self.dispatch(1, target, 0)))
        self.assertEqual('TARGET_NOT_CONFIRMED', self.core.last_reason)
        self.assertTrue(self.core.on_receipt(receipt(report)))
        (estimate,) = self.core.target_snapshot_change()['targets']
        self.assertEqual((target, 0, report['report_id']),
                         (estimate['target_id'], estimate['estimate_revision'], estimate['persisted_report_id']))
        reply = self.dispatch(2, target, 0)
        self.assertEqual((True, 'ACCEPTED'), (reply['accepted'], reply['request_state']))
        (fields,) = self.core.tick(NOW)
        self.assertEqual(('MOVE_SEGMENT', SUPPORT, reply['created_task_id']),
                         (fields['kind'], fields['robot_id'], fields['task_id']))

    def test_a_newer_unsaved_revision_closes_the_gate_until_its_own_receipt(self):
        target = self.saved()
        self.core.on_target_location(location(2, (4.5, 4.)), now_ns=NOW)
        self.assertEqual('PENDING_EVIDENCE', self.dispatch(1, target, 1)['reason_code'])
        (report,) = self.core.reports_due(NOW)
        self.assertTrue(self.core.on_receipt(receipt(report)))
        self.assertEqual('INVALID_INPUT', self.dispatch(2, target, 0)['reason_code'])  # TARGET_REVISION_STALE
        self.assertEqual(True, self.dispatch(3, target, 1)['accepted'])


class SystemStateTests(TargetTestCase):
    def test_state_is_published_on_change_with_the_c10_vocabulary(self):
        first = self.core.system_state_change(NOW)
        self.assertEqual((1, 'NORMAL', 'IDLE', False, False), (first['state_sequence'], first['mode'], first['phase'],
                                                              first['has_trigger'], first['has_dispatch']))
        self.assertIsNone(self.core.system_state_change(NOW + 1))
        self.core.on_site_event(site_msg())
        state = self.core.system_state_change(NOW + 2)
        self.assertEqual((2, 'DISASTER', True, [uid(301)], NOW + 2),
                         (state['state_sequence'], state['mode'], state['has_trigger'], state['cause_event_ids'],
                          state['changed_at_ns']))
        target = self.saved()
        self.core.on_status(status_msg(SUPPORT))
        self.core.on_command(request('DISPATCH_SUPPORT', command_id=uid(701), target_id=target), now_ns=NOW)
        (fields,) = self.core.tick(NOW)
        state = self.core.system_state_change(NOW + 3)
        self.assertEqual((True, target, 0, 7, 0, len(SEGMENTS), 'EXECUTING', [fields['task_id']]),
                         (state['has_dispatch'], state['target_id'], state['target_revision'], state['route_revision'],
                          state['segment_index'], state['segment_count'], state['segment_phase'],
                          state['active_task_ids']))

    def test_segment_phase_and_mode_mapping(self):
        self.assertEqual({'APPROVED': 'EXECUTING', 'MOVING': 'EXECUTING', 'AWAITING_APPROVAL': 'AWAITING_APPROVAL',
                          'ARRIVED': 'STOPPED'}, ros_node.SEGMENT_PHASE_STATES)
        self.assertEqual('MISSION_FINISHED', ros_node.system_mode('DISASTER', 'FINISHED'))
        self.assertEqual('DISASTER', ros_node.system_mode('DISASTER', 'RETURNING'))


if __name__ == '__main__':
    unittest.main()
