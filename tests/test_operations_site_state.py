"""Operations read API for the control screen: robot snapshot, site events + exit count, target states (E2)."""
from dataclasses import asdict
from pathlib import Path
import re
import sqlite3
import tempfile
import unittest
from uuid import UUID

from rescue_contracts import SiteEvent
from rescue_operations.schema import OperationsError
from rescue_operations import site_state

ALLOWED = ('robot_a', 'robot_b')
# 1차 IDL of the operator command service; every operation it names must be recordable.
OPERATOR_SRV = Path(__file__).resolve().parents[1] / 'src' / 'rescue_interfaces' / 'srv' / 'SubmitOperatorCommand.srv'
READERS = ('read_robot_status', 'read_site_events', 'read_target_states', 'read_error_log',
           'read_mission_state', 'read_operator_commands', 'read_map_snapshots')


def uid(n):
    return str(UUID(int=n))


def robot_status(**changes):
    values = dict(robot_id='robot_a', robot_boot_id=uid(1), state_sequence=5, role='SCOUT', stamp_ns=1_000,
                  pose=dict(x=1.0, y=2.0, yaw=0.5, frame_id='robot_a/map'), motion_state='MOVING',
                  readiness='READY', battery_percent=80.0, battery_stamp_ns=900, active_task_id=uid(2),
                  generation=3)
    return values | changes


def site_event(n=10, kind='EXIT_PASS', at=2_000, **changes):
    values = dict(clock_domain_id='site-clock', stamp_semantics='CAPTURE', clock_state='SYNCED',
                  clock_error_bound_ns=1_000)
    return SiteEvent(uid(n), 'cam_exit', kind, at, uid(n + 1), 'a' * 64, 0.9, **(values | changes))


def target_state(**changes):
    values = dict(target_id=uid(500),
                  position=dict(x=3.0, y=4.0, frame_id='mission', map_session_id=uid(5), map_revision=2),
                  detection_confirmed=True, rescue_state='RECHECK_NEEDED',
                  rescue_reasons=['EXIT_NOT_LINKED'], verdict_id=uid(600), support_state='ASSIGNED',
                  support_robot_id='robot_b', updated_at_ns=3_000)
    return values | changes


def error_entry(**changes):
    values = dict(at_ns=5_000, robot_id='robot_a', task_id=uid(2), code='NAV_NOT_READY',
                  detail='rescue_robot.NAV2_INACTIVE: bt_navigator not active')
    return values | changes


def mission(**changes):
    values = dict(mode='DISASTER', phase='SEARCHING', trigger_event_id=uid(30), started_at_ns=1_500, generation=1,
                  updated_at_ns=4_000)
    return values | changes


def command(n=700, **changes):
    values = dict(command_id=uid(n), kind='CONTINUE_SEGMENT', operator_id='admin', robot_id='robot_b',
                  target_id=uid(500), segment_index=2, route_revision=1, requested_at_ns=7_000, accepted=True,
                  result='ACCEPTED', reason=None)
    return values | changes


def map_snapshot(n=800, **changes):
    values = dict(snapshot_id=uid(n), robot_id='robot_a', revision=3, path='maps/robot_a/rev3.png', sha256='b' * 64,
                  at_ns=9_000)
    return values | changes


class SiteStateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'site.db'

    def ledger(self):
        return site_state.SiteStateLedger(self.path, allowed_robot_ids=ALLOWED)

    def code(self, caught):
        return caught.exception.detail_code.split('.')[-1]

    # --- robot status -------------------------------------------------------
    def test_robot_snapshot_round_trips_latest_per_robot(self):
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_robot_status(robot_status()))
            self.assertTrue(ledger.record_robot_status(robot_status(robot_id='robot_b', role='SUPPORT', pose=None,
                                                                    battery_percent=None, battery_stamp_ns=None,
                                                                    active_task_id=None, generation=None,
                                                                    motion_state='UNKNOWN', readiness='UNKNOWN')))
            self.assertTrue(ledger.record_robot_status(robot_status(state_sequence=6, stamp_ns=1_100)))
        snapshot = site_state.read_robot_status(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, snapshot['schema_version'])
        robots = snapshot['robots']
        self.assertEqual(['robot_a', 'robot_b'], [r['robot_id'] for r in robots])
        self.assertEqual(6, robots[0]['state_sequence'])
        self.assertIsNone(robots[1]['pose'])
        expected = set(robot_status()) | {'received_at_ns'}
        self.assertEqual([expected, expected], [set(r) for r in robots])
        self.assertIsInstance(robots[0]['received_at_ns'], int)

    def test_stale_or_old_boot_status_never_overwrites(self):
        with self.ledger() as ledger:
            ledger.record_robot_status(robot_status(state_sequence=6, stamp_ns=1_100))
            self.assertFalse(ledger.record_robot_status(robot_status(state_sequence=5)))
            self.assertFalse(ledger.record_robot_status(robot_status(state_sequence=6, stamp_ns=1_100)))
            # A delayed message from another boot that is not newer must not replace the snapshot.
            self.assertFalse(ledger.record_robot_status(robot_status(robot_boot_id=uid(9), state_sequence=99,
                                                                     stamp_ns=1_000)))
            self.assertTrue(ledger.record_robot_status(robot_status(robot_boot_id=uid(9), state_sequence=0,
                                                                    stamp_ns=1_200)))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_robot_status(robot_status(robot_boot_id=uid(9), state_sequence=0,
                                                        stamp_ns=1_200, readiness='NOT_READY'))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
        robot = site_state.read_robot_status(self.path)['robots'][0]
        self.assertEqual((uid(9), 0), (robot['robot_boot_id'], robot['state_sequence']))

    def test_robot_status_rejects_bad_input(self):
        bad = [dict(robot_id='robot_x'), dict(role='LEADER'), dict(state_sequence=-1), dict(stamp_ns=0),
               dict(pose=dict(x=float('nan'), y=0.0, yaw=0.0, frame_id='m')), dict(pose=dict(x=0.0, y=0.0)),
               dict(battery_percent=101.0), dict(battery_percent=None), dict(battery_stamp_ns=None),
               dict(generation=None), dict(active_task_id=None), dict(motion_state=''), dict(extra=1)]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError) as caught:
                    ledger.record_robot_status(robot_status(**change))
                self.assertIn(self.code(caught), ('INVALID_INPUT', 'WRONG_ROBOT'))
            missing = robot_status()
            del missing['readiness']
            with self.assertRaises(OperationsError):
                ledger.record_robot_status(missing)

    # --- site events --------------------------------------------------------
    def test_site_events_keep_fields_and_count_distinct_exit_passes(self):
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_site_event(site_event(10)))
            self.assertFalse(ledger.record_site_event(site_event(10)))  # identical resend
            self.assertTrue(ledger.record_site_event(site_event(20, at=2_500)))
            self.assertTrue(ledger.record_site_event(site_event(30, kind='DISASTER_SIGN', at=1_500)))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_site_event(site_event(10, at=9_999))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
            with self.assertRaises(OperationsError):
                ledger.record_site_event({'event_id': uid(40)})
        result = site_state.read_site_events(self.path)
        self.assertEqual(2, result['exit_pass_count'])
        self.assertEqual([uid(30), uid(10), uid(20)], [e['event_id'] for e in result['site_events']])
        first = result['site_events'][1]
        self.assertEqual(set(SiteEvent.__dataclass_fields__), set(first))
        self.assertEqual(site_event(10), SiteEvent(**first))

    def test_disappeared_candidate_keeps_target_and_watch(self):
        candidate = site_event(50, kind='TARGET_DISAPPEARED_CANDIDATE', at=3_000, target_id=uid(500),
                               watch_id=uid(510))
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_site_event(candidate))
            self.assertFalse(ledger.record_site_event(candidate))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_site_event(site_event(50, kind='TARGET_DISAPPEARED_CANDIDATE', at=3_000,
                                                    target_id=uid(500), watch_id=uid(511)))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
        result = site_state.read_site_events(self.path)
        self.assertEqual(0, result['exit_pass_count'])
        self.assertEqual([candidate], [SiteEvent(**e) for e in result['site_events']])

    def write_event_before_target_and_watch(self, event):
        """A row written before #48: event_json has no target_id/watch_id keys."""
        self.ledger().close()
        legacy = {k: v for k, v in asdict(event).items() if k not in ('target_id', 'watch_id')}
        db = sqlite3.connect(self.path)
        with db:
            db.execute('INSERT INTO site_events VALUES (?, ?, ?, ?, ?)',
                       (event.event_id, event.kind, event.observed_at_ns, site_state.canonical(legacy), 1_100))
        db.close()

    def test_events_stored_before_target_and_watch_read_back_with_none(self):
        old = site_event(60, kind='DISASTER_SIGN', at=1_000)
        self.write_event_before_target_and_watch(old)
        stored = site_state.read_site_events(self.path)['site_events']
        self.assertEqual([set(SiteEvent.__dataclass_fields__)], [set(e) for e in stored])  # db-schema 5절
        self.assertEqual([old], [SiteEvent(**e) for e in stored])

    def test_resending_an_event_stored_before_target_and_watch_is_not_a_conflict(self):
        old = site_event(60, kind='DISASTER_SIGN', at=1_000)
        self.write_event_before_target_and_watch(old)
        with self.ledger() as ledger:
            self.assertFalse(ledger.record_site_event(old))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_site_event(site_event(60, kind='DISASTER_SIGN', at=1_001))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))

    def test_exit_count_is_over_all_events_not_the_page(self):
        with self.ledger() as ledger:
            for n in range(3):
                ledger.record_site_event(site_event(100 + 2 * n, at=2_000 + n))
        page = site_state.read_site_events(self.path, limit=1)
        self.assertEqual((3, 1), (page['exit_pass_count'], len(page['site_events'])))
        for bad in (0, 1001, True, 1.0):
            with self.subTest(limit=bad), self.assertRaises(OperationsError):
                site_state.read_site_events(self.path, limit=bad)

    # --- target states ------------------------------------------------------
    def test_target_state_round_trips_and_only_newer_replaces(self):
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_target_state(target_state()))
            self.assertFalse(ledger.record_target_state(target_state()))
            self.assertFalse(ledger.record_target_state(target_state(updated_at_ns=2_000, support_state='FAILED')))
            confirmed = target_state(rescue_state='RESCUE_CONFIRMED', rescue_reasons=[], support_state='ARRIVED',
                                     updated_at_ns=4_000)
            self.assertTrue(ledger.record_target_state(confirmed))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_target_state(confirmed | {'support_state': 'FAILED'})
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
            self.assertTrue(ledger.record_target_state(target_state(target_id=uid(501), position=None,
                                                                    detection_confirmed=False,
                                                                    rescue_state='NOT_CHECKED', rescue_reasons=[],
                                                                    verdict_id=None, support_state='NONE',
                                                                    support_robot_id=None)))
        result = site_state.read_target_states(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        self.assertEqual([confirmed, target_state(target_id=uid(501), position=None, detection_confirmed=False,
                                                  rescue_state='NOT_CHECKED', rescue_reasons=[], verdict_id=None,
                                                  support_state='NONE', support_robot_id=None)],
                         result['targets'])

    def test_target_state_rejects_inconsistent_input(self):
        bad = [dict(rescue_state='DONE'), dict(rescue_reasons=[]), dict(rescue_state='RESCUE_CONFIRMED'),
               dict(rescue_reasons=['A', 'A']), dict(rescue_reasons=['not an id']), dict(support_state='MOVING'),
               dict(support_state='NONE'), dict(support_robot_id=None), dict(support_robot_id='robot_x'),
               dict(detection_confirmed=1), dict(position=dict(x=1.0, y=2.0)), dict(updated_at_ns=0),
               dict(verdict_id='x')]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError):
                    ledger.record_target_state(target_state(**change))

    # --- error log (BRD 3.7.4) ----------------------------------------------
    def test_error_log_appends_every_entry_and_reads_newest_first(self):
        with self.ledger() as ledger:
            first = ledger.record_error(error_entry())
            again = ledger.record_error(error_entry())  # a repeated failure is a new entry, not a resend
            system = ledger.record_error(error_entry(at_ns=6_000, robot_id=None, task_id=None,
                                                     code='SESSION_FAULT',
                                                     detail='rescue_mapping.SESSION_LOST'))
            self.assertEqual(3, len({first, again, system}))
        result = site_state.read_error_log(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        errors = result['errors']
        self.assertEqual([system, again, first], [e['seq'] for e in errors])
        self.assertEqual(error_entry() | {'seq': first}, errors[2])
        self.assertEqual((None, None), (errors[0]['robot_id'], errors[0]['task_id']))
        self.assertEqual([system], [e['seq'] for e in site_state.read_error_log(self.path, limit=1)['errors']])

    def test_error_log_rejects_bad_input(self):
        bad = [dict(code='SOMETHING_WENT_WRONG'), dict(code='rescue_robot.NAV2_INACTIVE'), dict(robot_id='robot_x'),
               dict(task_id='x'), dict(at_ns=0), dict(detail=''), dict(detail='x' * 1025), dict(extra=1)]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError):
                    ledger.record_error(error_entry(**change))
        self.assertEqual([], site_state.read_error_log(self.path)['errors'])

    # --- mission state ------------------------------------------------------
    def test_mission_state_is_one_row_and_only_newer_replaces(self):
        self.ledger().close()
        self.assertIsNone(site_state.read_mission_state(self.path)['mission'])
        idle = mission(mode='NORMAL', phase='IDLE', trigger_event_id=None, started_at_ns=None, generation=0,
                       updated_at_ns=1_000)
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_mission_state(idle))
            self.assertTrue(ledger.record_mission_state(mission()))
            self.assertFalse(ledger.record_mission_state(mission()))  # identical resend
            self.assertFalse(ledger.record_mission_state(mission(phase='DISASTER', updated_at_ns=3_000)))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_mission_state(mission(phase='TARGET_FOUND'))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
        result = site_state.read_mission_state(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        self.assertEqual(mission(), result['mission'])
        with sqlite3.connect(self.path) as db:
            self.assertEqual(1, db.execute('SELECT COUNT(*) FROM mission_state').fetchone()[0])
        db.close()

    def test_mission_state_rejects_bad_input(self):
        bad = [dict(mode='PANIC'), dict(phase='DONE'), dict(trigger_event_id='x'), dict(trigger_event_id=None),
               dict(started_at_ns=None), dict(started_at_ns=0), dict(generation=-1), dict(generation=None),
               dict(updated_at_ns=0), dict(extra=1)]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError):
                    ledger.record_mission_state(mission(**change))
        self.assertIsNone(site_state.read_mission_state(self.path)['mission'])

    # --- operator commands --------------------------------------------------
    def test_operator_commands_keep_rejections_and_resends_are_idempotent(self):
        rejected = command(701, kind='DISPATCH_SUPPORT', segment_index=None, route_revision=None,
                           requested_at_ns=6_000, accepted=False, result=None,
                           reason='GUARD_INVALID: no confirmed target')
        stop = command(702, kind='STOP_ROBOT', target_id=None, segment_index=None, route_revision=None,
                       requested_at_ns=8_000)
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_operator_command(command()))
            self.assertFalse(ledger.record_operator_command(command()))  # identical resend
            self.assertTrue(ledger.record_operator_command(rejected))
            self.assertTrue(ledger.record_operator_command(stop))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_operator_command(command(accepted=False, reason='late'))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
        result = site_state.read_operator_commands(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        self.assertEqual([stop, command(), rejected], result['commands'])
        self.assertIs(False, result['commands'][2]['accepted'])
        self.assertEqual([stop], site_state.read_operator_commands(self.path, limit=1)['commands'])

    def test_every_operation_of_the_command_service_can_be_recorded(self):
        """The console forwards SubmitOperatorCommand (1차 IDL, #77·#98); a refused record would lose it."""
        operations = re.findall(r'^string OPERATION_\w+="(\w+)"', OPERATOR_SRV.read_text(), re.M)
        self.assertGreaterEqual(len(operations), 8, operations)  # parsed, not an empty pass
        self.assertLessEqual(set(operations), site_state.OPERATOR_COMMAND_KINDS,
                             sorted(set(operations) - site_state.OPERATOR_COMMAND_KINDS))
        with self.ledger() as ledger:
            for n, kind in enumerate(operations):
                with self.subTest(kind=kind):
                    self.assertTrue(ledger.record_operator_command(command(900 + n, kind=kind)))

    def test_operator_command_rejects_bad_input(self):
        bad = [dict(command_id='x'), dict(kind='SELF_DESTRUCT'), dict(operator_id=''), dict(robot_id='robot_x'),
               dict(target_id='x'), dict(segment_index=-1), dict(route_revision=1.0), dict(requested_at_ns=0),
               dict(accepted=1), dict(result=''), dict(accepted=False, reason=None), dict(reason=''),
               dict(extra=1)]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError):
                    ledger.record_operator_command(command(**change))
        self.assertEqual([], site_state.read_operator_commands(self.path)['commands'])

    # --- map snapshots ------------------------------------------------------
    def test_map_snapshots_append_and_resends_are_idempotent(self):
        older = map_snapshot(801, revision=2, path='maps/robot_a/rev2.png', sha256='c' * 64, at_ns=8_000)
        other = map_snapshot(802, robot_id='robot_b', at_ns=8_500)
        with self.ledger() as ledger:
            self.assertTrue(ledger.record_map_snapshot(map_snapshot()))
            self.assertFalse(ledger.record_map_snapshot(map_snapshot()))
            self.assertTrue(ledger.record_map_snapshot(older))
            self.assertTrue(ledger.record_map_snapshot(other))
            with self.assertRaises(OperationsError) as caught:
                ledger.record_map_snapshot(map_snapshot(sha256='d' * 64))
            self.assertEqual('ID_CONTENT_CONFLICT', self.code(caught))
        result = site_state.read_map_snapshots(self.path)
        self.assertEqual(site_state.SCHEMA_VERSION, result['schema_version'])
        self.assertEqual([map_snapshot(), other, older], result['maps'])
        self.assertEqual([map_snapshot()], site_state.read_map_snapshots(self.path, limit=1)['maps'])

    def test_map_snapshot_rejects_bad_input(self):
        bad = [dict(snapshot_id='x'), dict(robot_id='robot_x'), dict(revision=-1), dict(path=''),
               dict(path='a\x00b'), dict(sha256='B' * 64), dict(sha256='b' * 63), dict(at_ns=0), dict(extra=1)]
        with self.ledger() as ledger:
            for change in bad:
                with self.subTest(change=change), self.assertRaises(OperationsError):
                    ledger.record_map_snapshot(map_snapshot(**change))
        self.assertEqual([], site_state.read_map_snapshots(self.path)['maps'])

    # --- read boundary ------------------------------------------------------
    def test_readers_do_not_create_or_accept_other_databases(self):
        missing = Path(self.temp.name) / 'missing.db'
        for read in (getattr(site_state, name) for name in READERS):
            with self.subTest(read=read.__name__):
                with self.assertRaises(OperationsError) as caught:
                    read(missing)
                self.assertEqual('STORAGE_ERROR', self.code(caught))
                self.assertFalse(missing.exists())
        other = Path(self.temp.name) / 'reports.db'
        from rescue_operations.ledger import ReportLedger
        ReportLedger(other, evidence_policy='OPTIONAL', allowed_robot_ids=ALLOWED).close()
        for read in (getattr(site_state, name) for name in READERS):
            with self.subTest(other=read.__name__), self.assertRaises(OperationsError) as caught:
                read(other)
            self.assertEqual('POLICY_MISMATCH', self.code(caught))

    def test_readers_are_read_only(self):
        self.ledger().close()
        before = self.path.read_bytes()
        site_state.read_robot_status(self.path)
        site_state.read_site_events(self.path)
        site_state.read_target_states(self.path)
        self.assertEqual(before, self.path.read_bytes())

    def test_policy_cannot_change_on_reopen(self):
        self.ledger().close()
        with self.assertRaises(OperationsError) as caught:
            site_state.SiteStateLedger(self.path, allowed_robot_ids=('robot_a', 'robot_c'))
        self.assertEqual('POLICY_MISMATCH', self.code(caught))
        with sqlite3.connect(self.path) as db:
            self.assertEqual('operations-site-state',
                             dict(db.execute('SELECT key, value FROM metadata'))['role'])

    # --- schema 1.0 -> 1.1 ----------------------------------------------------
    def write_schema_1_0(self, allowed=ALLOWED):
        """A site-state file exactly as the schema 1.0 writer left it (main at PR #66)."""
        status = robot_status()
        db = sqlite3.connect(self.path)
        with db:
            db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            db.executemany('INSERT INTO metadata VALUES (?, ?)',
                           [('schema', '1.0'), ('role', 'operations-site-state'),
                            ('allowed_robot_ids', site_state.canonical(site_state.allowed_robots(allowed)))])
            db.execute("""CREATE TABLE robot_status (
                robot_id TEXT PRIMARY KEY, robot_boot_id TEXT NOT NULL, state_sequence INTEGER NOT NULL,
                stamp_ns INTEGER NOT NULL, status_json TEXT NOT NULL, received_at_ns INTEGER NOT NULL)""")
            db.execute("""CREATE TABLE site_events (
                event_id TEXT PRIMARY KEY, kind TEXT NOT NULL, observed_at_ns INTEGER NOT NULL,
                event_json TEXT NOT NULL, received_at_ns INTEGER NOT NULL)""")
            db.execute("""CREATE TABLE target_states (
                target_id TEXT PRIMARY KEY, updated_at_ns INTEGER NOT NULL, state_json TEXT NOT NULL)""")
            db.execute('INSERT INTO robot_status VALUES (?, ?, ?, ?, ?, ?)',
                       (status['robot_id'], status['robot_boot_id'], status['state_sequence'], status['stamp_ns'],
                        site_state.canonical(status), 1_500))
            db.execute('INSERT INTO target_states VALUES (?, ?, ?)',
                       (uid(500), 3_000, site_state.canonical(target_state())))
        db.close()

    def metadata_and_tables(self):
        db = sqlite3.connect(self.path)
        try:
            metadata = dict(db.execute('SELECT key, value FROM metadata'))
            tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type = 'table'")}
        finally:
            db.close()
        return metadata, tables

    def test_opening_schema_1_0_db_upgrades_tables_without_dropping_data(self):
        self.write_schema_1_0()
        self.ledger().close()
        metadata, tables = self.metadata_and_tables()
        self.assertEqual('1.1', metadata['schema'])
        self.assertLessEqual({'error_log', 'mission_state', 'operator_commands', 'map_snapshots',
                              'robot_status', 'site_events', 'target_states'}, tables)
        robots = site_state.read_robot_status(self.path)['robots']
        self.assertEqual([robot_status() | {'received_at_ns': 1_500}], robots)
        self.assertEqual([target_state()], site_state.read_target_states(self.path)['targets'])
        self.ledger().close()  # an upgraded file reopens as a plain 1.1 file

    def test_schema_1_0_db_with_other_policy_is_rejected_and_left_unchanged(self):
        self.write_schema_1_0(allowed=('robot_a', 'robot_c'))
        with self.assertRaises(OperationsError) as caught:
            self.ledger()
        self.assertEqual('POLICY_MISMATCH', self.code(caught))
        metadata, tables = self.metadata_and_tables()
        self.assertEqual('1.0', metadata['schema'])
        self.assertNotIn('error_log', tables)

    def test_readers_reject_a_schema_1_0_file_until_the_writer_upgrades_it(self):
        self.write_schema_1_0()
        with self.assertRaises(OperationsError) as caught:
            site_state.read_robot_status(self.path)
        self.assertEqual('POLICY_MISMATCH', self.code(caught))


if __name__ == '__main__':
    unittest.main()
