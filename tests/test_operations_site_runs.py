"""Runs (회차) and replay history in the site-state DB, schema 1.5 (작전·웹개발 2026-10-15; E2).

The scenario reset (초기화) keeps every earlier record and starts a new run instead of a new file: one DB,
a ``runs`` table, and a ``run_id`` on every appended row (site events, operator commands, errors, map
snapshots, exit observations). The writer opens a run when none is open. The screen's readers give only the
open run's appended rows and carry its ``run_id``; with no open run (just after a reset) they give none.
Replay reads one run by time window (``read_*_range``) and two history tables: robot snapshots thinned to one
row per robot per second, and every accepted target state. A 1.4 file is upgraded on open: its rows go into
one closed 「이전 기록」 run.
"""
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest import mock

from rescue_operations import site_state
from rescue_operations.schema import OperationsError
from test_operations_site_state import (ALLOWED, command, error_entry, map_snapshot, robot_status, site_event,
                                        target_state, uid)


def exit_location(n=1, at=6_000):
    return dict(location_id=uid(900 + n), observed_at_ns=at, received_at_ns=at + 10, map_session_id=uid(5),
                map_revision=1, transform_revision=0, mission_frame='mission', x=1.0, y=2.0, class_label='exit_qr')


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site.db'

    def ledger(self):
        return site_state.SiteStateLedger(self.path, allowed_robot_ids=ALLOWED)

    def code(self, caught):
        return caught.exception.detail_code.split('.')[-1]

    def fill(self, ledger, n=0):
        """One appended row in each run table, ids offset by ``n``."""
        ledger.record_site_event(site_event(10 + n))
        ledger.record_operator_command(command(700 + n))
        ledger.record_error(error_entry(detail=f'error {n}'))
        ledger.record_map_snapshot(map_snapshot(800 + n))
        ledger.record_exit_location(exit_location(n))


class RunTests(Case):
    def test_writer_opens_a_run_and_keeps_it_on_reopen(self):
        with self.ledger() as ledger:
            first = ledger.run_id
        self.assertIsNotNone(first)
        with self.ledger() as ledger:
            self.assertEqual(first, ledger.run_id)
        runs = site_state.list_runs(self.path)
        self.assertEqual(first, runs['open_run_id'])
        self.assertEqual([first], [run['run_id'] for run in runs['runs']])
        self.assertIsNone(runs['runs'][0]['ended_at_ns'])

    def test_appended_rows_carry_the_open_run(self):
        with self.ledger() as ledger:
            self.fill(ledger)
            run = ledger.run_id
        db = sqlite3.connect(self.path)
        try:
            for table in site_state.RUN_TABLES:
                self.assertEqual([(run,)], db.execute(f'SELECT run_id FROM {table}').fetchall(), table)
        finally:
            db.close()

    def test_close_then_open_starts_a_new_run_and_keeps_old_rows(self):
        with self.ledger() as ledger:
            self.fill(ledger)
            old = ledger.run_id
            self.assertEqual(old, ledger.close_run(at_ns=50_000))
            self.assertIsNone(ledger.run_id)
            self.assertIsNone(ledger.close_run(at_ns=50_001))  # nothing open: nothing to close
            new = ledger.open_run(at_ns=60_000, note='두 번째 회차', previous_coordinator_boot_id=uid(77))
            self.fill(ledger, 1)
        self.assertNotEqual(old, new)
        runs = site_state.list_runs(self.path)['runs']
        self.assertEqual([old, new], [run['run_id'] for run in runs])
        self.assertEqual(50_000, runs[0]['ended_at_ns'])
        self.assertEqual((60_000, '두 번째 회차', uid(77)),
                         (runs[1]['started_at_ns'], runs[1]['note'], runs[1]['previous_coordinator_boot_id']))
        events = site_state.read_site_events_range(self.path, run_id=old)['site_events']
        self.assertEqual([uid(10)], [event['event_id'] for event in events])

    def test_only_one_run_is_open(self):
        with self.ledger() as ledger:
            with self.assertRaises(OperationsError) as caught:
                ledger.open_run(at_ns=60_000)
            self.assertEqual('TASK_BUSY', self.code(caught))

    def test_first_map_state_of_a_run_names_its_map_session(self):
        current = dict(robot_id='robot_a', map_session_id=uid(5), map_revision=0, generated_at_ns=1_000,
                       received_at_ns=1_010)
        with self.ledger() as ledger:
            ledger.record_current_map(current)
            ledger.record_current_map(current | dict(map_session_id=uid(6), generated_at_ns=2_000))
        self.assertEqual(uid(5), site_state.list_runs(self.path)['runs'][0]['map_session_id'])


class ScreenReaderTests(Case):
    """The live screen's readers keep their names, arguments and order, and see only the open run."""

    def test_readers_show_only_the_open_run_and_name_it(self):
        with self.ledger() as ledger:
            self.fill(ledger)
            ledger.close_run(at_ns=50_000)
            new = ledger.open_run(at_ns=60_000)
            ledger.record_site_event(site_event(40, kind='DISASTER_SIGN'))
        events = site_state.read_site_events(self.path)
        self.assertEqual(new, events['run_id'])
        self.assertEqual([uid(40)], [event['event_id'] for event in events['site_events']])
        self.assertEqual(0, events['exit_pass_count'])
        self.assertEqual([], site_state.read_operator_commands(self.path)['commands'])
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])
        self.assertEqual([], site_state.read_map_snapshots(self.path)['maps'])
        self.assertEqual([], site_state.read_exit_locations(self.path)['exits'])
        self.assertEqual(new, site_state.read_robot_status(self.path)['run_id'])

    def test_no_open_run_reads_empty_not_the_last_run(self):
        with self.ledger() as ledger:
            self.fill(ledger)
            ledger.close_run(at_ns=50_000)
        events = site_state.read_site_events(self.path)
        self.assertEqual((None, [], 0), (events['run_id'], events['site_events'], events['exit_pass_count']))
        self.assertEqual([], site_state.read_operator_commands(self.path)['commands'])
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])
        self.assertEqual([], site_state.read_map_snapshots(self.path)['maps'])
        self.assertEqual([], site_state.read_exit_locations(self.path)['exits'])

    def test_rows_written_with_no_open_run_are_kept_but_belong_to_no_run(self):
        with self.ledger() as ledger:
            ledger.close_run(at_ns=50_000)
            ledger.record_error(error_entry(detail='between runs'))
            ledger.open_run(at_ns=60_000)
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])
        db = sqlite3.connect(self.path)
        try:
            self.assertEqual([(None, 'between runs')], db.execute('SELECT run_id, detail FROM error_log').fetchall())
        finally:
            db.close()

    def test_resends_are_still_idempotent_with_run_id(self):
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_map_snapshot(map_snapshot()))
            self.assertFalse(ledger.record_map_snapshot(map_snapshot()))
            self.assertTrue(ledger.record_exit_location(exit_location()))
            self.assertFalse(ledger.record_exit_location(exit_location() | dict(received_at_ns=99_999)))
        self.assertEqual([map_snapshot()], site_state.read_map_snapshots(self.path)['maps'])
        self.assertEqual([exit_location()], site_state.read_exit_locations(self.path)['exits'])


class RangeReaderTests(Case):
    def test_range_is_oldest_first_inside_the_window(self):
        with self.ledger() as ledger:
            for n, at in enumerate((3_000, 1_000, 2_000, 4_000)):
                ledger.record_site_event(site_event(10 + 2 * n, at=at))
            run = ledger.run_id
        result = site_state.read_site_events_range(self.path, run_id=run, start_ns=1_500, end_ns=4_000)
        self.assertEqual((run, False), (result['run_id'], result['truncated']))
        self.assertEqual([2_000, 3_000], [event['observed_at_ns'] for event in result['site_events']])

    def test_limit_cuts_the_newest_and_says_so(self):
        with self.ledger() as ledger:
            for n in range(3):
                ledger.record_error(error_entry(at_ns=1_000 + n, detail=f'e{n}'))
            run = ledger.run_id
        result = site_state.read_error_log_range(self.path, run_id=run, limit=2)
        self.assertTrue(result['truncated'])
        self.assertEqual(['e0', 'e1'], [error['detail'] for error in result['errors']])
        self.assertFalse(site_state.read_error_log_range(self.path, run_id=run, limit=3)['truncated'])

    def test_every_run_table_has_a_range_reader(self):
        with self.ledger() as ledger:
            self.fill(ledger)
            run = ledger.run_id
        readers = {'read_site_events_range': 'site_events', 'read_operator_commands_range': 'commands',
                   'read_error_log_range': 'errors', 'read_map_snapshots_range': 'maps',
                   'read_exit_locations_range': 'exits'}
        for name, key in readers.items():
            self.assertEqual(1, len(getattr(site_state, name)(self.path, run_id=run)[key]), name)
        self.assertEqual([command(700)], site_state.read_operator_commands_range(self.path, run_id=run)['commands'])

    def test_bad_window_or_limit_is_refused(self):
        with self.ledger() as ledger:
            run = ledger.run_id
        for kwargs in (dict(run_id='not-a-uuid'), dict(run_id=run, limit=0), dict(run_id=run, limit=1001),
                       dict(run_id=run, start_ns=5, end_ns=5), dict(run_id=run, start_ns=-1)):
            with self.assertRaises(OperationsError, msg=kwargs) as caught:
                site_state.read_site_events_range(self.path, **kwargs)
            self.assertEqual('INVALID_INPUT', self.code(caught))

    def test_an_unknown_run_reads_empty(self):
        self.ledger().close()
        self.assertEqual([], site_state.read_site_events_range(self.path, run_id=uid(4242))['site_events'])


class HistoryTests(Case):
    def test_robot_history_keeps_one_row_per_robot_per_second(self):
        clock = iter([10_000_000_000, 10_400_000_000, 11_000_000_000, 11_100_000_000])
        with self.ledger() as ledger, mock.patch.object(site_state.time, 'time_ns', lambda: next(clock)):
            for sequence in (5, 6, 7):
                ledger.record_robot_status(robot_status(state_sequence=sequence))
            ledger.record_robot_status(robot_status(robot_id='robot_b', role='SUPPORT'))
            run = ledger.run_id
        history = site_state.read_robot_status_history(self.path, run_id=run)['history']
        self.assertEqual([('robot_a', 5, 10_000_000_000), ('robot_a', 7, 11_000_000_000),
                          ('robot_b', 5, 11_100_000_000)],
                         [(row['robot_id'], row['status']['state_sequence'], row['at_ns']) for row in history])

    def test_robot_history_takes_the_pose_topic_too(self):
        clock = iter([10_000_000_000, 12_000_000_000])
        moved = dict(x=5.0, y=6.0, yaw=0.0, frame_id='robot_a/map')
        with self.ledger() as ledger, mock.patch.object(site_state.time, 'time_ns', lambda: next(clock)):
            ledger.record_robot_status(robot_status())
            ledger.record_robot_pose('robot_a', moved)
            run = ledger.run_id
        history = site_state.read_robot_status_history(self.path, run_id=run)['history']
        self.assertEqual([robot_status()['pose'], moved], [row['status']['pose'] for row in history])

    def test_target_history_keeps_every_accepted_state(self):
        with self.ledger() as ledger:
            ledger.record_target_state(target_state())
            ledger.record_target_state(target_state())  # resend: not a change
            ledger.record_target_state(target_state(updated_at_ns=2_000, rescue_state='NOT_CHECKED', rescue_reasons=[]))  # older
            ledger.record_target_state(target_state(updated_at_ns=4_000, rescue_state='RESCUE_CONFIRMED', rescue_reasons=[]))
            run = ledger.run_id
        history = site_state.read_target_state_history(self.path, run_id=run)['history']
        self.assertEqual([(3_000, 'RECHECK_NEEDED'), (4_000, 'RESCUE_CONFIRMED')],
                         [(row['updated_at_ns'], row['state']['rescue_state']) for row in history])


class UpgradeTests(Case):
    def write_schema_1_4(self):
        """A 1.4 file with one row in each appended table, as the 1.4 writer leaves it."""
        db = sqlite3.connect(self.path)
        with db:
            db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            db.executemany('INSERT INTO metadata VALUES (?, ?)',
                           [('schema', '1.4'), ('role', 'operations-site-state'),
                            ('allowed_robot_ids', site_state.canonical(site_state.allowed_robots(ALLOWED)))])
            for ddl in TABLES_1_4:
                db.execute(ddl)
            event = site_event(10)
            db.execute('INSERT INTO site_events VALUES (?, ?, ?, ?, ?)',
                       (event.event_id, event.kind, event.observed_at_ns,
                        site_state.canonical(site_state.asdict(event)), 2_100))
            db.execute("INSERT INTO error_log (at_ns, robot_id, task_id, code, detail) VALUES (500, NULL, NULL, "
                       "'X', 'old')")
        db.close()

    def test_a_1_4_file_keeps_its_rows_in_one_closed_earlier_run(self):
        self.write_schema_1_4()
        with self.ledger() as ledger:
            current = ledger.run_id
        runs = site_state.list_runs(self.path)['runs']
        self.assertEqual(2, len(runs))
        earlier = runs[0]
        self.assertEqual(('이전 기록', 500), (earlier['note'], earlier['started_at_ns']))
        self.assertIsNotNone(earlier['ended_at_ns'])
        self.assertEqual(current, runs[1]['run_id'])
        self.assertEqual([uid(10)], [e['event_id'] for e in
                                     site_state.read_site_events_range(self.path, run_id=earlier['run_id'])['site_events']])
        self.assertEqual([], site_state.read_site_events(self.path)['site_events'])
        self.assertEqual('1.5', site_state.SCHEMA_VERSION)

    def test_an_empty_1_4_file_gets_no_earlier_run(self):
        db = sqlite3.connect(self.path)
        with db:
            db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            db.executemany('INSERT INTO metadata VALUES (?, ?)',
                           [('schema', '1.4'), ('role', 'operations-site-state'),
                            ('allowed_robot_ids', site_state.canonical(site_state.allowed_robots(ALLOWED)))])
            for ddl in TABLES_1_4:
                db.execute(ddl)
        db.close()
        self.ledger().close()
        self.assertEqual(1, len(site_state.list_runs(self.path)['runs']))


# The 1.4 tables as main 2fa0ebc creates them (site_state.TABLES before 1.5), copied so the upgrade test
# does not follow later DDL changes.
TABLES_1_4 = (
    """CREATE TABLE robot_status (robot_id TEXT PRIMARY KEY, robot_boot_id TEXT NOT NULL,
        state_sequence INTEGER NOT NULL, stamp_ns INTEGER NOT NULL, status_json TEXT NOT NULL,
        received_at_ns INTEGER NOT NULL)""",
    """CREATE TABLE site_events (event_id TEXT PRIMARY KEY, kind TEXT NOT NULL, observed_at_ns INTEGER NOT NULL,
        event_json TEXT NOT NULL, received_at_ns INTEGER NOT NULL)""",
    """CREATE TABLE target_states (target_id TEXT PRIMARY KEY, updated_at_ns INTEGER NOT NULL,
        state_json TEXT NOT NULL)""",
    """CREATE TABLE error_log (seq INTEGER PRIMARY KEY, at_ns INTEGER NOT NULL, robot_id TEXT, task_id TEXT,
        code TEXT NOT NULL, detail TEXT NOT NULL)""",
    """CREATE TABLE mission_state (singleton INTEGER PRIMARY KEY CHECK (singleton = 1), mode TEXT NOT NULL,
        phase TEXT NOT NULL, trigger_event_id TEXT, started_at_ns INTEGER, generation INTEGER,
        updated_at_ns INTEGER NOT NULL, target_id TEXT, target_revision INTEGER, route_revision INTEGER,
        segment_index INTEGER, segment_count INTEGER, segment_phase TEXT, reason TEXT)""",
    """CREATE TABLE operator_commands (command_id TEXT PRIMARY KEY, kind TEXT NOT NULL, operator_id TEXT NOT NULL,
        robot_id TEXT, target_id TEXT, segment_index INTEGER, route_revision INTEGER,
        requested_at_ns INTEGER NOT NULL, accepted INTEGER NOT NULL, result TEXT, reason TEXT)""",
    """CREATE TABLE map_snapshots (snapshot_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, revision INTEGER NOT NULL,
        path TEXT NOT NULL, sha256 TEXT NOT NULL, at_ns INTEGER NOT NULL)""",
    """CREATE TABLE exit_locations (location_id TEXT PRIMARY KEY, observed_at_ns INTEGER NOT NULL,
        received_at_ns INTEGER NOT NULL, map_session_id TEXT NOT NULL, map_revision INTEGER NOT NULL,
        transform_revision INTEGER NOT NULL, mission_frame TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
        class_label TEXT NOT NULL)""",
    """CREATE TABLE overhead_passage_lists (robot_id TEXT PRIMARY KEY, generated_at_ns INTEGER NOT NULL,
        received_at_ns INTEGER NOT NULL, map_frame TEXT NOT NULL, clearance_m REAL NOT NULL,
        floor_noise_m REAL NOT NULL, lidar_plane_m REAL NOT NULL)""",
    """CREATE TABLE overhead_passages (robot_id TEXT NOT NULL, id TEXT NOT NULL, state TEXT NOT NULL,
        lowest_m REAL, x REAL NOT NULL, y REAL NOT NULL, map_session_id TEXT NOT NULL,
        map_revision INTEGER NOT NULL, frames INTEGER NOT NULL, first_seen_ns INTEGER NOT NULL,
        last_seen_ns INTEGER NOT NULL, reason TEXT NOT NULL, PRIMARY KEY (robot_id, id))""",
    """CREATE TABLE current_map (singleton INTEGER PRIMARY KEY CHECK (singleton = 1), robot_id TEXT NOT NULL,
        map_session_id TEXT NOT NULL, map_revision INTEGER NOT NULL, generated_at_ns INTEGER NOT NULL,
        received_at_ns INTEGER NOT NULL)""",
)


if __name__ == '__main__':
    unittest.main()
