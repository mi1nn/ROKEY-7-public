"""Operations read API for the control screen: robot snapshot, site events + exit count, target states (E2)."""
from pathlib import Path
import sqlite3
import tempfile
import unittest
from uuid import UUID

from rescue_contracts import SiteEvent
from rescue_operations.schema import OperationsError
from rescue_operations import site_state

ALLOWED = ('robot_a', 'robot_b')


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

    # --- read boundary ------------------------------------------------------
    def test_readers_do_not_create_or_accept_other_databases(self):
        missing = Path(self.temp.name) / 'missing.db'
        for read in (site_state.read_robot_status, site_state.read_site_events, site_state.read_target_states):
            with self.subTest(read=read.__name__):
                with self.assertRaises(OperationsError) as caught:
                    read(missing)
                self.assertEqual('STORAGE_ERROR', self.code(caught))
                self.assertFalse(missing.exists())
        other = Path(self.temp.name) / 'reports.db'
        from rescue_operations.ledger import ReportLedger
        ReportLedger(other, evidence_policy='OPTIONAL', allowed_robot_ids=ALLOWED).close()
        for read in (site_state.read_robot_status, site_state.read_site_events, site_state.read_target_states):
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


if __name__ == '__main__':
    unittest.main()
