"""Overhead passages (tunnels) in the site-state DB for the console map (작전 OPS-194; E2).

db_writer reads each robot's /rescue/robots/<id>/overhead_passages text (C-04b, 통합-인식 #259) only through
rescue_contracts.overhead.parse_overhead_passages and keeps that robot's current list: a newer list
(generated_at_ns) replaces the robot's whole list, so an item missing from it is gone from the DB too (the robot
keeps items of earlier map revisions in its list, the web dims those). The same list again is not new, an older
one is dropped. A bad text, or a list naming another robot than its topic, is refused into error_log.
Items of both robots are measured on the SLAM robot's (TB-6) map, so the read also gives that robot's latest
MapState (``current_map``, None until heard): the web draws items of that revision bold, others faint (hv-06).
Schema 1.4 adds the three tables; a 1.3 file gets them on open and keeps its rows.
"""
from pathlib import Path
from types import SimpleNamespace as Msg
import sqlite3
import tempfile
import unittest

from rescue_contracts.overhead import overhead_passages_text, parse_overhead_passages
from rescue_operations import site_state
from rescue_operations.schema import OperationsError
from test_operations_ros_ingest import ROBOTS, T0, uid


def item(n=1, state='PASSABLE', lowest=0.52, revision=0, **changes):
    values = dict(id=uid(800 + n), state=state, lowest_m=lowest, x=1.0 + n, y=-0.5, map_session_id=uid(5),
                  map_revision=revision, frames=4, first_seen_ns=T0 + n, last_seen_ns=T0 + 10 + n, reason='')
    return values | changes


def text(robot='robot_a', at=T0 + 100, items=(item(),)):
    return overhead_passages_text(robot_id=robot, map_frame='map', generated_at_ns=at, clearance_m=0.46,
                                  floor_noise_m=0.03, lidar_plane_m=0.18, items=list(items))


def record(robot='robot_a', at=T0 + 100, items=(item(),), received=T0 + 200):
    return parse_overhead_passages(text(robot, at, items)) | dict(received_at_ns=received)


def listing(path, robot='robot_a'):
    return site_state.read_overhead_passages(path)['robots'].get(robot)


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site_state.sqlite3'

    def ledger(self):
        return site_state.SiteStateLedger(self.path, allowed_robot_ids=ROBOTS)


class OverheadRecordTests(Case):
    def test_each_robot_s_list_is_kept_with_its_header(self):
        items = (item(1), item(2, 'BLOCKED', 0.31), item(3, 'UNKNOWN', None, reason='NO_FLOOR_SEEN'))
        with self.ledger() as ledger:
            self.assertIs(True, ledger.record_overhead_passages(record(items=items)))
            self.assertIs(True, ledger.record_overhead_passages(record('robot_b', items=())))
        result = site_state.read_overhead_passages(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        a = result['robots']['robot_a']
        self.assertEqual(dict(robot_id='robot_a', generated_at_ns=T0 + 100, received_at_ns=T0 + 200, map_frame='map',
                              clearance_m=0.46, floor_noise_m=0.03, lidar_plane_m=0.18),
                         {k: v for k, v in a.items() if k != 'items'})
        # newest last_seen_ns first, the item fields as C-04b has them
        self.assertEqual([items[2], items[1], items[0]], a['items'])
        self.assertEqual([], result['robots']['robot_b']['items'])

    def test_a_robot_never_heard_is_absent(self):
        self.ledger().close()
        self.assertEqual({}, site_state.read_overhead_passages(self.path)['robots'])

    def test_a_newer_list_replaces_the_robot_s_list_and_drops_missing_items(self):
        with self.ledger() as ledger:
            ledger.record_overhead_passages(record(items=(item(1), item(2))))
            ledger.record_overhead_passages(record('robot_b', items=(item(5),)))
            self.assertIs(True, ledger.record_overhead_passages(
                record(at=T0 + 101, items=(item(2, revision=1, last_seen_ns=T0 + 50),))))
        self.assertEqual([uid(802)], [i['id'] for i in listing(self.path)['items']])
        self.assertEqual(1, listing(self.path)['items'][0]['map_revision'])
        self.assertEqual([uid(805)], [i['id'] for i in listing(self.path, 'robot_b')['items']])  # the other robot kept

    def test_the_same_list_again_is_not_new_and_an_older_one_is_dropped(self):
        with self.ledger() as ledger:
            ledger.record_overhead_passages(record(items=(item(1),)))
            self.assertIs(False, ledger.record_overhead_passages(record(items=(item(1),), received=T0 + 300)))
            self.assertIs(False, ledger.record_overhead_passages(record(at=T0 + 99, items=(item(2),))))
        a = listing(self.path)
        self.assertEqual((T0 + 200, [uid(801)]), (a['received_at_ns'], [i['id'] for i in a['items']]))

    def test_same_time_other_content_or_bad_values_are_refused(self):
        with self.ledger() as ledger:
            ledger.record_overhead_passages(record(items=(item(1),)))
            for value, code in ((record(items=(item(2),)), 'ID_CONTENT_CONFLICT'),
                                (record('robot_z', at=T0 + 101), 'WRONG_ROBOT'),
                                (record(at=T0 + 102) | dict(clearance_m=0.1), 'INVALID_INPUT'),
                                (record(at=T0 + 103) | dict(extra=1), 'INVALID_INPUT'),
                                ({k: v for k, v in record(at=T0 + 104).items() if k != 'received_at_ns'},
                                 'INVALID_INPUT')):
                with self.subTest(value=value), self.assertRaises(OperationsError) as caught:
                    ledger.record_overhead_passages(value)
                self.assertEqual(code, caught.exception.detail_code.split('.')[-1])
        self.assertEqual([uid(801)], [i['id'] for i in listing(self.path)['items']])

    def test_a_schema_1_3_file_gets_the_tables_and_keeps_its_rows(self):
        with self.ledger() as ledger:
            ledger.record_error(dict(at_ns=T0, robot_id=None, task_id=None, code='INVALID_INPUT', detail='kept'))
        db = sqlite3.connect(self.path)
        with db:
            db.execute('DROP TABLE overhead_passages')
            db.execute('DROP TABLE overhead_passage_lists')
            db.execute('DROP TABLE current_map')
            db.execute("UPDATE metadata SET value = '1.3' WHERE key = 'schema'")
        db.close()
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_overhead_passages(record()))
        self.assertEqual(['kept'], [e['detail'] for e in site_state.read_error_log(self.path)['errors']])
        self.assertEqual('1.4', site_state.SCHEMA_VERSION)


def current(revision=3, at=T0 + 50, robot='robot_a', received=T0 + 60, session=None):
    return dict(robot_id=robot, map_session_id=session or uid(5), map_revision=revision, generated_at_ns=at,
                received_at_ns=received)


class CurrentMapTests(Case):
    def test_none_until_the_map_state_is_heard_then_the_latest(self):
        self.ledger().close()
        self.assertIsNone(site_state.read_overhead_passages(self.path)['current_map'])
        with self.ledger() as ledger:
            self.assertIs(True, ledger.record_current_map(current(3)))
            self.assertIs(True, ledger.record_current_map(current(4, at=T0 + 51)))
            self.assertIs(False, ledger.record_current_map(current(4, at=T0 + 51, received=T0 + 99)))  # resend
            self.assertIs(False, ledger.record_current_map(current(2, at=T0 + 40)))  # older
        self.assertEqual(current(4, at=T0 + 51), site_state.read_overhead_passages(self.path)['current_map'])

    def test_same_time_other_content_or_bad_values_are_refused(self):
        with self.ledger() as ledger:
            ledger.record_current_map(current(3))
            for value, code in ((current(5), 'ID_CONTENT_CONFLICT'), (current(robot='robot_z'), 'WRONG_ROBOT'),
                                (current(session='x', at=T0 + 70), 'INVALID_INPUT'),
                                (current(-1, at=T0 + 71), 'INVALID_INPUT'),
                                (current(at=T0 + 72) | dict(extra=1), 'INVALID_INPUT')):
                with self.subTest(value=value), self.assertRaises(OperationsError) as caught:
                    ledger.record_current_map(value)
                self.assertEqual(code, caught.exception.detail_code.split('.')[-1])


class OverheadIngestTests(Case):
    def setUp(self):
        super().setUp()
        from rescue_operations.ros_ingest import SiteStateIngest
        self.db = self.ledger()
        self.addCleanup(self.db.close)
        self.ingest = SiteStateIngest(self.db)

    def test_the_text_is_stored_for_the_topic_s_robot(self):
        msg = Msg(data=text(items=(item(1),)))
        self.assertIs(True, self.ingest.on_overhead_passages(msg, robot_id='robot_a', received_at_ns=T0 + 200))
        self.assertIs(False, self.ingest.on_overhead_passages(msg, robot_id='robot_a', received_at_ns=T0 + 201))
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])
        self.assertEqual([item(1)], listing(self.path)['items'])

    def test_a_bad_text_or_another_robot_s_list_is_refused_and_logged(self):
        for data in ('{}', 'not json', text().replace('rescue-overhead-passages/1', 'rescue-overhead-passages/2'),
                     text(robot='robot_b')):
            with self.subTest(data=data[:40]):
                self.assertIsNone(self.ingest.on_overhead_passages(Msg(data=data), robot_id='robot_a',
                                                                   received_at_ns=T0 + 60))
        self.assertIsNone(listing(self.path))
        self.assertIsNone(listing(self.path, 'robot_b'))
        errors = site_state.read_error_log(self.path)['errors']
        self.assertEqual((4, {'robot_a'}), (len(errors), {e['robot_id'] for e in errors}))

    def test_the_slam_robot_s_map_state_is_the_current_map(self):
        msg = Msg(robot_id='robot_a', snapshot_id=uid(6), map_session_id=uid(5), map_revision=3,
                  map_update_sequence=9, generated_at_ns=T0 + 50, localization_quality='GOOD')
        self.assertIs(True, self.ingest.on_map_state(msg, received_at_ns=T0 + 60))
        self.assertEqual(current(3), site_state.read_overhead_passages(self.path)['current_map'])
        self.assertIsNone(self.ingest.on_map_state(Msg(robot_id='robot_a', map_session_id='x', map_revision=4,
                                                       generated_at_ns=T0 + 70), received_at_ns=T0 + 80))
        self.assertEqual(1, len(site_state.read_error_log(self.path)['errors']))


if __name__ == '__main__':
    unittest.main()
