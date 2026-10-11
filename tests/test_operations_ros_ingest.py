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


if __name__ == '__main__':
    unittest.main()
