"""QR exit observations in the site-state DB for the console map (작전 OPS-144·150; E2).

db_writer stores each VALID /rescue/exit_location TargetLocation (class_label exit_qr, 통합-인식 안 「나」): map
position, map basis and observation time; anything else is refused into error_log (the console maps VALID only). The console reads them newest first and shows the latest one.
This is the observation record, not the coordinator's registered exit. Schema 1.3 adds the table; a 1.2 file
gets it on open and keeps its rows.
"""
from pathlib import Path
from types import SimpleNamespace as Msg
import sqlite3
import tempfile
import unittest

from rescue_operations import site_state
from rescue_operations.schema import OperationsError
from test_operations_ros_ingest import ROBOTS, T0, envelope, obs_time, uid


def exit_record(n=1, **changes):
    values = dict(location_id=uid(700 + n), observed_at_ns=T0 + n, received_at_ns=T0 + 100 + n,
                  map_session_id=uid(5), map_revision=0, transform_revision=1, mission_frame='map',
                  x=-5.0 + n, y=0.5, class_label='exit_qr')
    return values | changes


def exit_msg(n=1, label='exit_qr', status='VALID', x=-4.0):
    frame_ref = Msg(robot_id='robot_a', native_frame='map', mission_frame='map', map_session_id=uid(5),
                    map_revision=0, transform_revision=1, alignment_status='VALID')
    return Msg(envelope=envelope(), location_id=uid(700 + n), frame_ref=frame_ref, projection_status=status,
               observation=Msg(time=obs_time(at=T0 + n), class_label=label),
               mission_position=Msg(x=x, y=0.5, z=0.0), covariance=[0.0] * 9)


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'

    def ledger(self):
        return site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)


class ExitLocationRecordTests(Case):
    def test_newest_first_and_a_resend_is_not_new(self):
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_exit_location(exit_record(1)))
            self.assertTrue(ledger.record_exit_location(exit_record(2)))
            self.assertFalse(ledger.record_exit_location(exit_record(1)))
        result = site_state.read_exit_locations(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        self.assertEqual([exit_record(2), exit_record(1)], result['exits'])
        self.assertEqual([exit_record(2)], site_state.read_exit_locations(self.path, limit=1)['exits'])

    def test_same_observation_time_is_ordered_by_location_id(self):
        # Console (hv-06, OPS-150): newest observed_at_ns first, ties by location_id; received_at_ns is display only.
        with self.ledger() as ledger:
            ledger.record_exit_location(exit_record(2, observed_at_ns=T0 + 9, received_at_ns=T0 + 1))
            ledger.record_exit_location(exit_record(1, observed_at_ns=T0 + 9, received_at_ns=T0 + 2))
        self.assertEqual([uid(701), uid(702)],
                         [e['location_id'] for e in site_state.read_exit_locations(self.path)['exits']])

    def test_same_id_other_content_or_bad_fields_are_refused(self):
        with self.ledger() as ledger:
            ledger.record_exit_location(exit_record(1))
            for record, code in ((exit_record(1, x=9.0), 'ID_CONTENT_CONFLICT'),
                                 (exit_record(3, class_label='survivor'), 'INVALID_INPUT'),
                                 (exit_record(4, x=float('nan')), 'INVALID_INPUT'),
                                 (exit_record(5, map_session_id='x'), 'INVALID_INPUT'),
                                 (exit_record(6) | dict(extra=1), 'INVALID_INPUT')):
                with self.subTest(record=record), self.assertRaises(OperationsError) as caught:
                    ledger.record_exit_location(record)
                self.assertEqual(code, caught.exception.detail_code.split('.')[-1])

    def test_a_schema_1_2_file_gets_the_table_and_keeps_its_rows(self):
        with self.ledger() as ledger:
            ledger.record_error(dict(at_ns=T0, robot_id=None, task_id=None, code='INVALID_INPUT', detail='kept'))
        db = sqlite3.connect(self.path)
        with db:
            db.execute('DROP TABLE exit_locations')
            db.execute("UPDATE metadata SET value = '1.2' WHERE key = 'schema'")
        db.close()
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_exit_location(exit_record(1)))
        self.assertEqual(['kept'], [e['detail'] for e in site_state.read_error_log(self.path)['errors']])
        self.assertEqual('1.3', site_state.SCHEMA_VERSION)


class ExitLocationIngestTests(Case):
    def setUp(self):
        super().setUp()
        from rescue_operations.ros_ingest import SiteStateIngest
        self.db = self.ledger()
        self.addCleanup(self.db.close)
        self.ingest = SiteStateIngest(self.db)

    def test_exit_message_is_stored_with_its_map_basis(self):
        self.assertTrue(self.ingest.on_exit_location(exit_msg(1), received_at_ns=T0 + 50))
        self.assertFalse(self.ingest.on_exit_location(exit_msg(1), received_at_ns=T0 + 51))
        (row,) = site_state.read_exit_locations(self.path)['exits']
        self.assertEqual(exit_record(1, x=-4.0, received_at_ns=T0 + 50), row)

    def test_another_class_or_not_valid_is_refused_and_logged(self):
        for msg in (exit_msg(2, label='survivor'), exit_msg(3, status='STALE_DERIVATION')):
            with self.subTest(msg=msg):
                self.assertIsNone(self.ingest.on_exit_location(msg, received_at_ns=T0 + 60))
        self.assertEqual([], site_state.read_exit_locations(self.path)['exits'])
        self.assertEqual(2, len(site_state.read_error_log(self.path)['errors']))


if __name__ == '__main__':
    unittest.main()
