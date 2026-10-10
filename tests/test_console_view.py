"""Console aggregation: ledger -> screen model is 1:1, read-only, and the Q-20 rule table holds."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RESCUE_STATES, SiteEvent
from rescue_operations.ledger import ReportLedger
from rescue_operations.site_state import SiteStateLedger
from rescue_console.app import current_view
from rescue_console.render import render_page
from rescue_console.sources import load_ledger, load_site_state, load_status
from rescue_console.view import (STAGE_RULES, STAGES, STATUS_STALE_S, ConsoleError, build_view,
                                 compose_stage)

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ('robot_a', 'robot_b')
T0 = 1_760_000_000_000_000_000
SECOND = 1_000_000_000


def uid(label):
    return str(uuid5(NAMESPACE_URL, 'rokey7/console-test/' + label))


def report(label, *, confirmed, approach='NOT_ATTEMPTED', x=1.0, y=2.0, offset=0, robot='robot_a'):
    observed = T0 + offset * SECOND
    return {
        'schema_version': '1.0', 'report_id': uid(f'{label}/report/{offset}'), 'target_id': uid(label),
        'observation_id': uid(f'{label}/obs/{offset}'), 'source_robot': robot, 'source_camera': 'oakd',
        'source_boot_id': uid('boot'), 'clock_domain_id': 'test-clock', 'robot_id': robot,
        'task_id': uid(f'{label}/task'), 'generation': 1, 'observation_result': 'DETECTED',
        'confirmation_result': 'CONFIRMED' if confirmed else 'UNCONFIRMED',
        'confirmation_observation_id': uid(f'{label}/confirm/{offset}') if confirmed else None,
        'approach_result': approach,
        'position': {'x': x, 'y': y, 'z': 0.0, 'frame_id': 'map', 'map_session_id': uid('map'),
                     'map_revision': 3, 'transform_revision': 1, 'authority_id': 'test'},
        'observed_at_ns': observed, 'confirmed_at_ns': observed + SECOND if confirmed else None,
        'reported_at_ns': observed + 2 * SECOND, 'reason': 'test', 'evidence_refs': [],
        'model_id': 'test-model', 'model_sha256': '0' * 64, 'deployment_version': 'test',
    }


def envelope(sequence):
    return {'schema_version': '1.0', 'publisher_id': 'test', 'publisher_boot_id': uid('publisher'),
            'sequence': sequence, 'published_at_ns': T0 + 100 * SECOND}


def site_event(label, kind='EXIT_PASS', offset=10, detail=''):
    return {'event_id': uid(label), 'camera_id': 'cam_b' if kind == 'EXIT_PASS' else 'cam_a', 'kind': kind,
            'observed_at_ns': T0 + offset * SECOND, 'evidence_id': uid(label + '/evidence'),
            'evidence_sha256': 'a' * 64, 'confidence': 0.9, 'clock_domain_id': 'test-clock',
            'stamp_semantics': 'CAPTURE', 'clock_state': 'SYNCED', 'clock_error_bound_ns': 1000, 'detail': detail}


def target_state(label, *, confirmed=True, rescue_state='NOT_CHECKED', support='NONE', robot=None, reasons=()):
    return {'target_id': uid(label), 'position': {'x': 1.0, 'y': 2.0, 'frame_id': 'map',
                                                  'map_session_id': uid('map'), 'map_revision': 3},
            'detection_confirmed': confirmed, 'rescue_state': rescue_state, 'rescue_reasons': list(reasons),
            'verdict_id': None, 'support_state': support, 'support_robot_id': robot, 'updated_at_ns': T0}


def robot_status(robot, *, battery=81.0, stamp_offset=20):
    return {'robot_id': robot, 'robot_boot_id': uid(robot + '/boot'), 'state_sequence': 7,
            'role': 'SCOUT' if robot == 'robot_a' else 'SUPPORT', 'stamp_ns': T0 + stamp_offset * SECOND,
            'pose': {'x': 0.5, 'y': -1.0, 'yaw': 1.57, 'frame_id': 'map'}, 'motion_state': 'MOVING',
            'readiness': 'READY', 'battery_percent': battery,
            'battery_stamp_ns': None if battery is None else T0 + stamp_offset * SECOND,
            'active_task_id': uid(robot + '/task'), 'generation': 2, 'received_at_ns': T0 + stamp_offset * SECOND}


def digest_dir(path):
    """Bytes of every file except SQLite WAL sidecars. A read-only open of a WAL database
    may create empty -wal/-shm files; they must stay empty (no frames written)."""
    result = {}
    for p in sorted(path.iterdir()):
        if p.name.endswith('-wal'):
            assert p.stat().st_size == 0, 'read-only open left WAL frames'
        if not p.name.endswith(('-wal', '-shm')):
            result[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    return result


def write_site_state(path, status):
    """Writes the fixture shapes into a site-state DB with the operations API (test setup only)."""
    with SiteStateLedger(path, allowed_robot_ids=ROBOTS) as ledger:
        for robot in status['robot_status']['robots']:
            ledger.record_robot_status({k: v for k, v in robot.items() if k != 'received_at_ns'})
        for raw in status['site_events']['site_events']:
            ledger.record_site_event(SiteEvent(**raw))
        for state in status['target_states']['targets']:
            ledger.record_target_state(state)


class LedgerFixture(unittest.TestCase):
    """Writes a fixture ledger with the operations API; the console under test only reads it."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.db = self.dir / 'operations.sqlite3'
        self.reports = [
            report('unconfirmed', confirmed=False),
            report('waiting', confirmed=True, x=3.5, y=-4.25, offset=1),
            report('waiting', confirmed=True, x=3.75, y=-4.0, offset=5),   # newer report wins
            report('supporting', confirmed=True, offset=2, robot='robot_b'),
            report('arrived', confirmed=True, approach='APPROACH_COMPLETE', offset=3, robot='robot_b'),
            report('rescued', confirmed=True, offset=4),
        ]
        with ReportLedger(self.db, evidence_policy='OPTIONAL', allowed_robot_ids=ROBOTS) as ledger:
            for sequence, payload in enumerate(self.reports):
                ledger.receive(payload, envelope(sequence))
                ledger.persist(payload['report_id'])
        self.status = {
            'robot_status': {'schema_version': '1.1', 'robots': [robot_status('robot_a'), robot_status('robot_b', battery=None)]},
            'site_events': {'schema_version': '1.1', 'exit_pass_count': 1,
                            'site_events': [site_event('exit-1'), site_event('quake', 'DISASTER_SIGN', 0)]},
            'target_states': {'schema_version': '1.1', 'targets': [
                target_state('unconfirmed', confirmed=False, rescue_state='NOT_CHECKED'),
                target_state('waiting', rescue_state='RECHECK_NEEDED', reasons=['EXIT_COUNT_INCREASED']),
                target_state('supporting', support='ASSIGNED', robot='robot_b'),
                target_state('rescued', rescue_state='RESCUE_CONFIRMED'),
            ]},
        }

    def view(self, status=None, built_at_ns=T0 + 30 * SECOND):
        return build_view(load_ledger(self.db), self.status if status is None else status, built_at_ns=built_at_ns)


class ComposeStageTests(unittest.TestCase):
    def test_rule_table_rows_are_each_reachable(self):
        cases = {
            'R1': (False, None, None, 'NOT_ATTEMPTED'),
            'R2': (True, 'RESCUE_CONFIRMED', 'NONE', 'NOT_ATTEMPTED'),
            'R3': (True, 'NOT_CHECKED', 'ASSIGNED', 'NOT_ATTEMPTED'),
            'R4': (True, None, None, 'APPROACH_COMPLETE'),
            'R5': (True, 'RECHECK_NEEDED', 'FAILED', 'UNREACHABLE'),
        }
        self.assertEqual({rule for rule, _, _ in STAGE_RULES}, set(cases))
        for rule, condition, stage in STAGE_RULES:
            with self.subTest(rule=rule, condition=condition):
                got_stage, got_rule, _ = compose_stage(*cases[rule])
                self.assertEqual((got_stage, got_rule), (stage, rule))
        self.assertEqual({stage for _, _, stage in STAGE_RULES}, set(STAGES))

    def test_every_contract_rescue_state_maps_without_extending_vocabulary(self):
        expected = {'NOT_CHECKED': 'NOT_RESCUED', 'RECHECK_NEEDED': 'NOT_RESCUED', 'RESCUE_CONFIRMED': 'RESCUED'}
        self.assertEqual(set(expected), set(RESCUE_STATES))
        for state, stage in expected.items():
            self.assertEqual(compose_stage(True, state, 'NONE', 'NOT_ATTEMPTED')[0], stage)

    def test_unconfirmed_wins_and_flags_inconsistent_inputs(self):
        stage, rule, warnings = compose_stage(False, 'RESCUE_CONFIRMED', 'ARRIVED', 'APPROACH_COMPLETE')
        self.assertEqual((stage, rule), ('UNCONFIRMED', 'R1'))
        self.assertTrue(warnings)

    def test_known_support_state_overrides_report_approach(self):
        # Support explicitly ended (NONE) after an approach: the target is waiting again, not supported.
        self.assertEqual(compose_stage(True, 'NOT_CHECKED', 'NONE', 'APPROACH_COMPLETE')[0], 'NOT_RESCUED')

    def test_values_outside_vocabulary_are_rejected(self):
        for args in ((True, 'SAVED', None, None), (True, None, 'MAYBE', None), ('yes', None, None, None)):
            with self.subTest(args=args), self.assertRaises(ConsoleError):
                compose_stage(*args)


class LedgerViewTests(LedgerFixture):
    def test_targets_match_latest_ledger_report_one_to_one(self):
        view = self.view()
        ledger = load_ledger(self.db)
        latest = {}
        for item in ledger['reports']:
            latest[item['payload']['target_id']] = item  # read order is received order
        self.assertEqual({row['target_id'] for row in view['targets']}, set(latest))
        for row in view['targets']:
            item = latest[row['target_id']]
            with self.subTest(target=row['target_id']):
                self.assertEqual(row['report_id'], item['payload']['report_id'])
                self.assertEqual(row['approach_result'], item['payload']['approach_result'])
                self.assertEqual(row['receipt_status'], item['receipt']['status'])
                self.assertEqual(row['observed_at_ns'], item['payload']['observed_at_ns'])
        waiting = next(row for row in view['targets'] if row['target_id'] == uid('waiting'))
        self.assertEqual(waiting['report_count'], 2)
        self.assertEqual(waiting['rescue_reasons'], ['EXIT_COUNT_INCREASED'])

    def test_four_stages_are_composed_from_ledger_and_states(self):
        stages = {row['target_id']: (row['stage'], row['rule']) for row in self.view()['targets']}
        self.assertEqual(stages, {
            uid('unconfirmed'): ('UNCONFIRMED', 'R1'),
            uid('waiting'): ('NOT_RESCUED', 'R5'),
            uid('supporting'): ('SUPPORTING', 'R3'),
            uid('arrived'): ('SUPPORTING', 'R4'),
            uid('rescued'): ('RESCUED', 'R2'),
        })
        self.assertEqual(self.view()['stage_counts'],
                         {'UNCONFIRMED': 1, 'NOT_RESCUED': 1, 'SUPPORTING': 2, 'RESCUED': 1})

    def test_position_source_is_labelled(self):
        rows = {row['target_id']: row for row in self.view()['targets']}
        self.assertEqual(rows[uid('arrived')]['position']['source'], 'report')
        self.assertEqual(rows[uid('arrived')]['position']['x'], 1.0)
        self.assertEqual(rows[uid('waiting')]['position']['source'], 'target_state')
        self.assertIn('구조 상태 입력 없음', rows[uid('arrived')]['warnings'])

    def test_events_carry_source_time_and_ledger_time_for_kpi04(self):
        view = self.view()
        ledger = load_ledger(self.db)
        self.assertEqual(view['event_total'], len(self.reports) + 2)
        report_rows = {row['ref_id']: row for row in view['events'] if row['kind'] == 'REPORT'}
        for item in ledger['reports']:
            row = report_rows[item['payload']['report_id']]
            self.assertEqual(row['event_ns'], item['payload']['observed_at_ns'])
            self.assertEqual(row['received_at_ns'], item['received_at_ns'])
        times = [row['event_ns'] for row in view['events']]
        self.assertEqual(times, sorted(times, reverse=True))
        self.assertEqual(view['built_at_ns'], T0 + 30 * SECOND)

    def test_exit_count_is_taken_from_read_api_not_recounted(self):
        status = dict(self.status, site_events={'schema_version': '1.1', 'exit_pass_count': 3,
                                                'site_events': [site_event('exit-1')]})
        view = self.view(status)
        self.assertEqual(view['exit_count'], 3)
        self.assertFalse(any('3.5.5' in w for w in view['warnings']))

    def test_exit_count_zero_with_rescues_is_not_a_warning(self):
        # One fixed camera: no EXIT_PASS producer, the count stays 0 (db-schema 2절); rescues are normal.
        status = dict(self.status, site_events={'schema_version': '1.1', 'exit_pass_count': 0, 'site_events': []})
        view = self.view(status)
        self.assertEqual(view['stage_counts']['RESCUED'], 1)
        self.assertEqual(view['exit_count'], 0)
        self.assertEqual([w for w in view['warnings'] if '출구' in w or '3.5.5' in w], [])

    def test_mission_mode_comes_from_the_first_disaster_sign(self):
        events = [site_event('quake-late', 'DISASTER_SIGN', 7), site_event('exit-1'),
                  site_event('quake-early', 'DISASTER_SIGN', 2)]
        mission = self.view(dict(self.status, site_events={'schema_version': '1.0', 'exit_pass_count': 0,
                                                           'site_events': events}))['mission']
        self.assertEqual((mission['mode'], mission['label'], mission['sign_count']), ('DISASTER', '재난', 2))
        self.assertEqual(mission['trigger_event_id'], uid('quake-early'))
        self.assertEqual(mission['trigger_ns'], T0 + 2 * SECOND)

    def test_mission_mode_is_normal_without_signs_and_unknown_without_input(self):
        calm = self.view(dict(self.status, site_events={'schema_version': '1.0', 'exit_pass_count': 0,
                                                        'site_events': [site_event('exit-1')]}))
        self.assertEqual((calm['mission']['mode'], calm['mission']['trigger_ns']), ('NORMAL', None))
        unknown = self.view({})
        self.assertIsNone(unknown['mission']['mode'])
        self.assertEqual(unknown['mission']['label'], '모름')

    def test_robot_rows_are_stale_past_the_display_limit(self):
        fresh = self.view(built_at_ns=T0 + 20 * SECOND + int(STATUS_STALE_S * SECOND))
        self.assertEqual({row['stale'] for row in fresh['robots']}, {False})
        late = self.view(built_at_ns=T0 + 20 * SECOND + int(STATUS_STALE_S * SECOND) + 1)
        self.assertEqual({row['stale'] for row in late['robots']}, {True})
        self.assertEqual({row['stale'] for row in self.view({})['robots']}, {True})

    def test_robots_follow_ledger_deployment_and_unknowns_stay_none(self):
        view = self.view()
        self.assertEqual([row['robot_id'] for row in view['robots']], list(ROBOTS))
        robot_b = view['robots'][1]
        self.assertIsNone(robot_b['battery_percent'])
        self.assertIsNone(robot_b['battery_age_s'])
        self.assertEqual(view['robots'][0]['battery_percent'], 81.0)
        self.assertEqual(view['robots'][0]['age_s'], 10.0)
        self.assertEqual(view['robots'][0]['received_age_s'], 10.0)

    def test_missing_status_inputs_are_visible_not_defaulted(self):
        view = self.view({})
        self.assertEqual({row['has_status'] for row in view['robots']}, {False})
        self.assertIsNone(view['exit_count'])
        for key in ('robot_status', 'site_events', 'target_states'):
            self.assertIn(f'{key} 입력 없음', view['warnings'])

    def test_robot_outside_deployment_is_not_shown(self):
        status = dict(self.status, robot_status={'schema_version': '1.1', 'robots': [robot_status('robot_z')]})
        view = self.view(status)
        self.assertNotIn('robot_z', [row['robot_id'] for row in view['robots']])
        self.assertTrue(any('robot_z' in w for w in view['warnings']))

    def test_reading_never_changes_or_creates_ledger_files(self):
        before = digest_dir(self.dir)
        view = self.view()
        render_page(view)
        self.assertEqual(digest_dir(self.dir), before)

    def test_missing_ledger_fails_without_creating_it(self):
        missing = self.dir / 'absent.sqlite3'
        with self.assertRaises(ConsoleError) as caught:
            load_ledger(missing)
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')
        self.assertFalse(missing.exists())

    def test_status_fixture_rejects_unknown_sections(self):
        path = self.dir / 'status.json'
        path.write_text(json.dumps({'robots': []}), encoding='utf-8')
        with self.assertRaises(ConsoleError):
            load_status(path)
        path.write_text(json.dumps(self.status), encoding='utf-8')
        self.assertEqual(load_status(path), self.status)


class OfflineRehearsalLedgerTests(unittest.TestCase):
    """E2: the ledger written by the offline rehearsal renders as one screen."""

    def test_offline_demo_ledger_renders(self):
        from rescue_bringup.offline_demo import run_scenario
        with tempfile.TemporaryDirectory() as folder:
            shutil.copytree(ROOT / 'config', Path(folder) / 'config')
            run_scenario(Path(folder) / 'config' / 'deployment.example.json', Path(folder) / 'run')
            ledger = load_ledger(Path(folder) / 'run' / 'operations.sqlite3')
            view = build_view(ledger, {}, built_at_ns=T0)
            html = render_page(view)
        self.assertEqual(len(view['targets']), len({r['payload']['target_id'] for r in ledger['reports']}))
        self.assertGreater(len(view['targets']), 0)
        for row in view['targets']:
            self.assertIn(row['target_id'], html)
        self.assertEqual(html.count('class="event"'), len(ledger['reports']))

    def test_example_status_fixture_matches_offline_ledger(self):
        example = ROOT / 'src' / 'rescue_console' / 'examples' / 'status.example.json'
        if not example.exists():
            self.skipTest('source checkout only: example fixture is not installed')
        from rescue_bringup.offline_demo import run_scenario
        with tempfile.TemporaryDirectory() as folder:
            shutil.copytree(ROOT / 'config', Path(folder) / 'config')
            run_scenario(Path(folder) / 'config' / 'deployment.example.json', Path(folder) / 'run')
            view = build_view(load_ledger(Path(folder) / 'run' / 'operations.sqlite3'),
                              load_status(example), built_at_ns=T0)
        self.assertEqual({row['has_status'] for row in view['robots']}, {True})
        self.assertFalse([w for w in view['warnings'] if '입력 없음' in w or '보고가 없는' in w])
        self.assertEqual(view['exit_count'], 1)


class SiteStateSourceTests(LedgerFixture):
    """--site-state: the console reads the operations site-state DB (#34) and never writes it."""

    def setUp(self):
        super().setUp()
        self.site_db = self.dir / 'site_state.sqlite3'
        write_site_state(self.site_db, self.status)

    def test_site_state_db_gives_the_fixture_shapes(self):
        status = load_site_state(self.site_db)
        self.assertEqual(set(status), {'robot_status', 'site_events', 'target_states'})
        self.assertEqual([r['robot_id'] for r in status['robot_status']['robots']], list(ROBOTS))
        self.assertEqual(status['site_events']['exit_pass_count'], 1)
        db_view = build_view(load_ledger(self.db), status, built_at_ns=T0 + 30 * SECOND)
        fixture_view = self.view()
        for key in ('targets', 'stage_counts', 'exit_count', 'mission'):
            self.assertEqual(db_view[key], fixture_view[key], key)
        self.assertEqual([w for w in db_view['warnings'] if '입력 없음' in w], [])

    def test_site_state_read_changes_nothing_and_missing_db_is_not_created(self):
        before = digest_dir(self.dir)
        current_view(self.db, site_state_path=self.site_db)
        self.assertEqual(digest_dir(self.dir), before)
        with self.assertRaises(ConsoleError) as caught:
            load_site_state(self.dir / 'absent.sqlite3')
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')
        self.assertFalse((self.dir / 'absent.sqlite3').exists())

    def test_ledger_file_is_not_a_site_state_db(self):
        with self.assertRaises(ConsoleError) as caught:
            load_site_state(self.db)
        self.assertEqual(caught.exception.detail_code, 'STORAGE_ERROR')
        self.assertIn('POLICY_MISMATCH', str(caught.exception))

    def test_site_state_and_fixture_together_are_refused(self):
        with self.assertRaises(ConsoleError) as caught:
            current_view(self.db, 'status.json', site_state_path=self.site_db)
        self.assertEqual(caught.exception.detail_code, 'INVALID_INPUT')


if __name__ == '__main__':
    unittest.main()
