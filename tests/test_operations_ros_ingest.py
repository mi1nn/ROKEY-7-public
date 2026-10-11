"""DB writer adapter core: ROS message fields -> site-state records, without ROS (E2).

Messages are plain attribute objects with the 1차 IDL field names (src/rescue_interfaces/msg),
so this runs where rclpy is not installed. The rclpy node only subscribes and calls this core.
"""
from pathlib import Path
from types import SimpleNamespace as Msg
import tempfile
import unittest
from uuid import UUID

from rescue_contracts import SiteEvent
from rescue_operations import site_state
from rescue_operations.schema import OperationsError

ROBOTS = ('robot_a', 'robot_b')
T0 = 1_700_000_000_000_000_000


def uid(n):
    return str(UUID(int=n))


def envelope(sequence=1, at=T0 + 5):
    return Msg(schema_version='1.0', publisher_id='cam_quake', publisher_boot_id=uid(90), sequence=sequence,
               published_at_ns=at)


def obs_time(at=T0, bound=2_000_000):
    return Msg(observed_at_ns=at, valid_until_ns=at + 1_000_000_000, stamp_semantics='CAPTURE',
               clock_domain_id='site-clock', clock_state='SYNCED', has_clock_error_bound=bound is not None,
               clock_error_bound_ns=0 if bound is None else bound)


def site_event_msg(n=10, kind='DISASTER_SIGN', **changes):
    values = dict(envelope=envelope(), event_id=uid(n), camera_id='cam_fixed', source_boot_id=uid(91), kind=kind,
                  time=obs_time(), target_id='', watch_id='', confidence=0.9, visibility_state='VALID',
                  metrics_name=['reversals'], metrics_value=[4.0], policy_version='quake-v1',
                  evidence=Msg(evidence_id=uid(n + 1), sha256='a' * 64, mime_type='image/jpeg',
                               source_id='cam_fixed', source_boot_id=uid(91), observed_at_ns=T0))
    return Msg(**(values | changes))


ROLES = {'robot_a': 'SCOUT', 'robot_b': 'SUPPORT'}


def robot_status_msg(robot='robot_a', sequence=5, **changes):
    values = dict(envelope=envelope(sequence), robot_id=robot, robot_boot_id=uid(1), state_sequence=sequence,
                  observed_at_ns=T0 + sequence, authority_epoch='epoch-1', active_task_id=uid(2), generation=3,
                  readiness='READY', task_state='EXECUTING', motion_state='MOVING', segment_index=1,
                  stop_reason='', stop_evidence=[], dock_state='UNDOCKED', dock_observed_at_ns=T0,
                  lease_valid=True, sensor_quality='OK', nav2_state='ACTIVE', battery_known=True,
                  battery_fraction=0.8)
    return Msg(**(values | changes))


def pose_msg(robot='robot_a', status='VALID', x=1.0, y=2.0, qz=0.0, qw=1.0, frame='mission'):
    stamped = Msg(header=Msg(frame_id=frame),
                  pose=Msg(pose=Msg(position=Msg(x=x, y=y, z=0.0), orientation=Msg(x=0.0, y=0.0, z=qz, w=qw))))
    return Msg(envelope=envelope(), robot_id=robot, source_boot_id=uid(1), time=obs_time(), native_pose=stamped,
               mission_pose=stamped, frame_ref=None, mission_pose_status=status)


def system_state_msg(phase='SEGMENT_MOVING', changed=T0 + 100, dispatch=True, trigger=True, **changes):
    """Field names and empty-value convention as #90 publishes them (0 / '' when has_* is false)."""
    values = dict(envelope=envelope(), state_sequence=7, mode='DISASTER', changed_at_ns=changed,
                  cause_event_ids=[uid(10)] if trigger else [], active_task_ids=[uid(2)], policy_version='p1',
                  reason='', phase=phase, has_trigger=trigger, trigger_event_id=uid(10) if trigger else '',
                  trigger_observed_at_ns=T0 if trigger else 0, has_dispatch=dispatch,
                  target_id=uid(500) if dispatch else '', target_revision=2 if dispatch else 0,
                  route_revision=1 if dispatch else 0, segment_index=1 if dispatch else 0,
                  segment_count=3 if dispatch else 0, segment_phase='EXECUTING' if dispatch else 'IDLE')
    return Msg(**(values | changes))


class SiteEventConversion(unittest.TestCase):
    def test_disaster_sign_message_becomes_the_contract_event(self):
        from rescue_operations.ros_ingest import site_event_from_msg
        event = site_event_from_msg(site_event_msg())
        self.assertEqual(SiteEvent(uid(10), 'cam_fixed', 'DISASTER_SIGN', T0, uid(11), 'a' * 64, 0.9, 'site-clock',
                                   'CAPTURE', 'SYNCED', 2_000_000), event)

    def test_candidate_keeps_target_and_watch_and_missing_bound_is_none(self):
        from rescue_operations.ros_ingest import site_event_from_msg
        event = site_event_from_msg(site_event_msg(20, kind='TARGET_DISAPPEARED_CANDIDATE', target_id=uid(500),
                                                   watch_id=uid(510), time=obs_time(bound=None)))
        self.assertEqual((uid(500), uid(510), None), (event.target_id, event.watch_id, event.clock_error_bound_ns))

    def test_not_valid_visibility_is_refused(self):
        """C-10: ROS -> Python refuses every visibility other than VALID, the empty string included."""
        from rescue_operations.ros_ingest import site_event_from_msg
        for state in ('OCCLUDED', 'UNKNOWN', ''):
            with self.subTest(state=state), self.assertRaises(OperationsError) as caught:
                site_event_from_msg(site_event_msg(visibility_state=state))
            self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_contract_violations_surface_as_invalid_input(self):
        from rescue_operations.ros_ingest import site_event_from_msg
        bad = [dict(event_id='not-a-uuid'), dict(kind='EARTHQUAKE'), dict(camera_id='robot_a'),
               dict(target_id=uid(500)),  # target only on a candidate
               dict(kind='TARGET_DISAPPEARED_CANDIDATE'),  # candidate without target/watch
               dict(confidence=1.5)]
        for change in bad:
            with self.subTest(change=change), self.assertRaises(OperationsError) as caught:
                site_event_from_msg(site_event_msg(**change))
            self.assertEqual('INVALID_INPUT', caught.exception.code)


class SiteEventIngest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)

    def ingest(self):
        from rescue_operations.ros_ingest import SiteStateIngest
        return SiteStateIngest(self.ledger)

    def test_valid_event_is_stored_and_a_resend_is_not_an_error(self):
        ingest = self.ingest()
        self.assertTrue(ingest.on_site_event(site_event_msg(), received_at_ns=T0 + 10))
        self.assertFalse(ingest.on_site_event(site_event_msg(), received_at_ns=T0 + 11))
        self.assertEqual([uid(10)], [e['event_id'] for e in site_state.read_site_events(self.path)['site_events']])
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])

    def test_refused_message_is_not_stored_and_lands_in_the_error_log(self):
        ingest = self.ingest()
        self.assertIsNone(ingest.on_site_event(site_event_msg(30, visibility_state='OCCLUDED'),
                                               received_at_ns=T0 + 20))
        self.assertEqual([], site_state.read_site_events(self.path)['site_events'])
        (error,) = site_state.read_error_log(self.path)['errors']
        self.assertEqual(('INVALID_INPUT', None, None, T0 + 20),
                         (error['code'], error['robot_id'], error['task_id'], error['at_ns']))
        self.assertIn('site_event', error['detail'])
        self.assertIn(uid(30), error['detail'])  # which message, so the log can be traced

    def test_same_id_with_other_content_is_logged_as_a_conflict(self):
        ingest = self.ingest()
        ingest.on_site_event(site_event_msg(), received_at_ns=T0 + 10)
        self.assertIsNone(ingest.on_site_event(site_event_msg(confidence=0.5), received_at_ns=T0 + 12))
        self.assertEqual([('ID_CONTENT_CONFLICT', T0 + 12)], [(e['code'], e['at_ns']) for e in
                                                              site_state.read_error_log(self.path)['errors']])


class RobotStatusConversion(unittest.TestCase):
    def convert(self, msg, pose=None, role='SCOUT'):
        from rescue_operations.ros_ingest import robot_status_from_msg
        return robot_status_from_msg(msg, role=role, pose=pose)

    def test_status_message_becomes_the_db_snapshot(self):
        self.assertEqual(dict(robot_id='robot_a', robot_boot_id=uid(1), state_sequence=5, role='SCOUT',
                              stamp_ns=T0 + 5, pose=None, motion_state='MOVING', readiness='READY',
                              battery_percent=80.0, battery_stamp_ns=T0 + 5, active_task_id=uid(2), generation=3,
                              dock_state='UNDOCKED', segment_index=1),
                         self.convert(robot_status_msg()))

    def test_absent_values_become_none_not_invented_numbers(self):
        status = self.convert(robot_status_msg(battery_known=False, battery_fraction=0.0, active_task_id='',
                                               generation=0, segment_index=0, dock_state=''))
        self.assertEqual((None, None, None, None, None, None),
                         (status['battery_percent'], status['battery_stamp_ns'], status['active_task_id'],
                          status['generation'], status['segment_index'], status['dock_state']))

    def test_valid_mission_pose_is_attached_with_yaw(self):
        from rescue_operations.ros_ingest import pose_from_msg
        pose = pose_from_msg(pose_msg(qz=0.7071067811865476, qw=0.7071067811865476))
        self.assertEqual(('mission', 1.0, 2.0), (pose['frame_id'], pose['x'], pose['y']))
        self.assertAlmostEqual(1.5707963267948966, pose['yaw'])
        self.assertEqual(pose, self.convert(robot_status_msg(), pose=pose)['pose'])
        for status in ('UNKNOWN', 'INVALID', ''):
            with self.subTest(status=status):
                self.assertIsNone(pose_from_msg(pose_msg(status=status)))

    def test_malformed_status_is_invalid_input(self):
        for change in (dict(robot_boot_id='x'), dict(battery_fraction=float('nan')), dict(battery_fraction=1.5),
                       dict(motion_state=''), dict(dock_state='FLOATING')):
            with self.subTest(change=change), self.assertRaises(OperationsError) as caught:
                from rescue_operations import site_state as db
                db._robot_status(self.convert(robot_status_msg(**change)), ROBOTS)
            self.assertEqual('INVALID_INPUT', caught.exception.code)


class RobotStatusIngest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)
        from rescue_operations.ros_ingest import SiteStateIngest
        self.ingest = SiteStateIngest(self.ledger, roles=ROLES)

    def robots(self):
        return {r['robot_id']: r for r in site_state.read_robot_status(self.path)['robots']}

    def test_status_uses_the_configured_role_and_the_latest_pose(self):
        self.assertTrue(self.ingest.on_robot_pose(pose_msg(x=4.0), received_at_ns=T0 + 1))
        self.assertTrue(self.ingest.on_robot_status(robot_status_msg(), received_at_ns=T0 + 3))
        self.assertTrue(self.ingest.on_robot_status(robot_status_msg('robot_b'), received_at_ns=T0 + 4))
        robots = self.robots()
        self.assertEqual(('SCOUT', 4.0), (robots['robot_a']['role'], robots['robot_a']['pose']['x']))
        self.assertEqual(('SUPPORT', None), (robots['robot_b']['role'], robots['robot_b']['pose']))

    def test_a_pose_that_is_no_longer_valid_hides_the_old_one(self):
        """AGENTS 7절: an old position is not shown as if it were current."""
        self.ingest.on_robot_pose(pose_msg(x=4.0), received_at_ns=T0 + 1)
        self.assertFalse(self.ingest.on_robot_pose(pose_msg(x=9.0, status='UNKNOWN'), received_at_ns=T0 + 2))
        self.ingest.on_robot_status(robot_status_msg(), received_at_ns=T0 + 3)
        self.assertIsNone(self.robots()['robot_a']['pose'])

    def test_pose_after_status_updates_the_map_and_a_status_resend_is_not_a_conflict(self):
        """Found in the local ROS run (2026-10-11): pose comes on its own topic, faster than status."""
        self.assertTrue(self.ingest.on_robot_status(robot_status_msg(), received_at_ns=T0 + 1))
        self.assertTrue(self.ingest.on_robot_pose(pose_msg(x=4.0), received_at_ns=T0 + 2))
        self.assertEqual(4.0, self.robots()['robot_a']['pose']['x'])  # shown without waiting for a status
        self.assertFalse(self.ingest.on_robot_status(robot_status_msg(), received_at_ns=T0 + 3))
        self.assertTrue(self.ingest.on_robot_pose(pose_msg(x=5.0), received_at_ns=T0 + 4))
        self.assertEqual(5.0, self.robots()['robot_a']['pose']['x'])
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])

    def test_stale_status_is_ignored_without_an_error(self):
        self.ingest.on_robot_status(robot_status_msg(sequence=6), received_at_ns=T0 + 3)
        self.assertFalse(self.ingest.on_robot_status(robot_status_msg(sequence=5), received_at_ns=T0 + 4))
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])

    def test_refused_status_lands_in_the_error_log_with_its_robot(self):
        self.assertIsNone(self.ingest.on_robot_status(robot_status_msg(battery_fraction=1.5), received_at_ns=T0 + 7))
        self.assertIsNone(self.ingest.on_robot_status(robot_status_msg('robot_x'), received_at_ns=T0 + 8))
        self.assertEqual([], site_state.read_robot_status(self.path)['robots'])
        errors = site_state.read_error_log(self.path)['errors']
        self.assertEqual([('WRONG_ROBOT', None), ('INVALID_INPUT', 'robot_a')],
                         [(e['code'], e['robot_id']) for e in errors])
        self.assertTrue(all('robot_status' in e['detail'] for e in errors))


class SystemStateConversion(unittest.TestCase):
    def convert(self, msg):
        from rescue_operations.ros_ingest import mission_from_msg
        return mission_from_msg(msg)

    def test_dispatching_state_becomes_the_mission_row(self):
        self.assertEqual(dict(mode='DISASTER', phase='SEGMENT_MOVING', trigger_event_id=uid(10), started_at_ns=T0,
                              generation=None, updated_at_ns=T0 + 100, target_id=uid(500), target_revision=2,
                              route_revision=1, segment_index=1, segment_count=3, segment_phase='EXECUTING',
                              reason=None),
                         self.convert(system_state_msg()))

    def test_reason_code_is_kept_and_empty_is_none(self):
        self.assertEqual('SEARCH_QUEUED', self.convert(system_state_msg(reason='SEARCH_QUEUED'))['reason'])
        self.assertIsNone(self.convert(system_state_msg(reason=''))['reason'])

    def test_has_flags_false_give_none_not_zeroes(self):
        mission = self.convert(system_state_msg(phase='IDLE', dispatch=False, trigger=False, mode='NORMAL'))
        self.assertEqual((None, None), (mission['trigger_event_id'], mission['started_at_ns']))
        self.assertEqual([None] * 6, [mission[k] for k in ('target_id', 'target_revision', 'route_revision',
                                                            'segment_index', 'segment_count', 'segment_phase')])


class SystemStateIngest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)
        from rescue_operations.ros_ingest import SiteStateIngest
        self.ingest = SiteStateIngest(self.ledger, roles=ROLES)

    def mission(self):
        return site_state.read_mission_state(self.path)['mission']

    def test_states_are_stored_in_change_order_and_a_coordinator_restart_is_not_mistaken(self):
        self.assertTrue(self.ingest.on_system_state(system_state_msg(), received_at_ns=T0 + 101))
        self.assertFalse(self.ingest.on_system_state(system_state_msg(), received_at_ns=T0 + 102))  # resend
        # A restarted coordinator counts state_sequence from 1 again; change time decides, not the counter.
        restarted = system_state_msg('AWAITING_CONTINUE', changed=T0 + 200, state_sequence=1,
                                     segment_phase='AWAITING_APPROVAL')
        self.assertTrue(self.ingest.on_system_state(restarted, received_at_ns=T0 + 201))
        self.assertFalse(self.ingest.on_system_state(system_state_msg(changed=T0 + 150), received_at_ns=T0 + 202))
        self.assertEqual(('AWAITING_CONTINUE', 1, 3), (self.mission()['phase'], self.mission()['segment_index'],
                                                       self.mission()['segment_count']))
        done = system_state_msg('FINISHED', changed=T0 + 300, mode='MISSION_FINISHED', segment_index=3,
                                segment_phase='STOPPED')
        self.assertTrue(self.ingest.on_system_state(done, received_at_ns=T0 + 301))
        self.assertEqual(('MISSION_FINISHED', 'FINISHED'), (self.mission()['mode'], self.mission()['phase']))
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])

    def test_refused_state_lands_in_the_error_log(self):
        self.assertIsNone(self.ingest.on_system_state(system_state_msg(segment_phase='MOVING'),
                                                      received_at_ns=T0 + 5))
        self.assertIsNone(self.mission())
        (error,) = site_state.read_error_log(self.path)['errors']
        self.assertEqual(('INVALID_INPUT', None), (error['code'], error['robot_id']))
        self.assertIn('system_state', error['detail'])


if __name__ == '__main__':
    unittest.main()
