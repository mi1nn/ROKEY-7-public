"""E2 component rehearsal; no installed ROS, model, hardware or remote hosts."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import unittest

from rescue_bringup.offline_demo import OBSERVED_AT_NS, run_scenario, uid
from rescue_bringup.profile import ProfileError
from rescue_operations.ledger import ReportOutbox, read_reports


ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('rescue_bringup', 'rescue_robot', 'rescue_mapping', 'rescue_perception',
            'rescue_coordination', 'rescue_operations', 'rescue_contracts')


class OfflineDemoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        shutil.copytree(ROOT / 'config', self.directory / 'config')
        self.profile = self.directory / 'config' / 'deployment.example.json'
        self.output = self.directory / 'run'

    def run_demo(self):
        return run_scenario(self.profile, self.output)

    def cli(self, profile=None, output=None):
        env = dict(os.environ)
        sources = [ROOT / 'src' / name for name in PACKAGES if (ROOT / 'src' / name).is_dir()]
        env.pop('PYTHONPATH', None)
        if sources:
            env['PYTHONPATH'] = os.pathsep.join(str(path) for path in sources)
        env['PYTHONDONTWRITEBYTECODE'] = '1'
        return subprocess.run(
            [sys.executable, '-m', 'rescue_bringup.offline_demo', str(profile or self.profile),
             '--output', str(output or self.output)],
            cwd=self.directory, env=env, capture_output=True, text=True, timeout=20,
        )

    def test_both_roles_are_proposals_with_automatic_approach_blocked(self):
        result = self.run_demo()
        self.assertTrue(result['passed'])
        self.assertTrue(all(result['checks'].values()))
        self.assertEqual({('robot_a', 'robot_b'), ('robot_b', 'robot_a')},
                         {(p['observer'], p['responder']) for p in result['proposals']})
        self.assertEqual(2, len({p['source_observation'] for p in result['proposals']}))
        expected = {'robot_a': ((2., 0., 0.), (3., 0., 0.), (2., 0., 0.)),
                    'robot_b': ((3., 0., 0.), (4., 0., 0.), (4., 0., 0.))}
        for proposal in result['proposals']:
            target, mission_goal, destination_goal = expected[proposal['observer']]
            self.assertEqual(target, proposal['target_mission_xyz'])
            self.assertEqual(mission_goal, proposal['candidate_mission_xyz'])
            self.assertEqual(destination_goal, proposal['candidate_destination_xyz'])
            self.assertTrue(proposal['coordinates_valid'])
            self.assertFalse(proposal['auto_approach_eligible'])
            self.assertFalse(proposal['approach_command_sent'])
            self.assertNotEqual(proposal['target_mission_xyz'], proposal['candidate_mission_xyz'])
        self.assertEqual((0, 1, False, False, 'NOT_RUN'),
                         (result['physical_robots_used'], result['master_processes_used'],
                          result['model_inference_run'], result['ros_run'], result['field_acceptance']))
        self.assertFalse(result['configured_hosts_used_as_processes'])
        self.assertEqual(4, len(result['configured_deployment']['hosts']))

    def test_a_revision_invalidates_only_a_observation_and_all_dependent_goals(self):
        result = self.run_demo()
        revision = result['revision_check']
        self.assertEqual('NEEDS_REOBSERVATION', revision['a_status'])
        self.assertEqual('VALID', revision['b_status'])
        self.assertEqual([uid('robot_a/observation')], revision['invalidated_observation_ids'])
        # A appears once as observer and once as destination: both goals depend on A.
        self.assertEqual([False, False], revision['goal_valid_after_revision'])
        self.assertTrue(result['checks']['stale_republication_rejected'])
        self.assertTrue(result['checks']['expired_corridor_unknown'])

    def test_synthetic_search_results_do_not_claim_real_commands(self):
        result = self.run_demo()
        self.assertEqual({'robot_a', 'robot_b'}, {s['robot_id'] for s in result['synthetic_search']})
        for search in result['synthetic_search']:
            self.assertEqual('SEARCH_AREA', search['task_kind'])
            self.assertEqual([('SEND', uid(search['robot_id'] + '/command')),
                              ('CANCEL', uid(search['robot_id'] + '/command'))], search['synthetic_intents'])
            self.assertEqual('CANCELED', search['result']['outcome'])
            self.assertEqual('CANCELED', search['result']['action_status'])
            self.assertTrue(search['terminal_received'])
            self.assertFalse(search['search_result_before_stop_samples'])
            self.assertTrue(search['result']['stop_confirmed'])
            self.assertFalse(search['real_command_sent'])
            self.assertEqual('SYNTHETIC_ACTION_AND_ODOMETRY', search['evidence_source'])

    def test_exact_epoch_reaches_actual_mapping_and_one_ns_shift_is_rejected(self):
        result = self.run_demo()
        # Assert the literal expected stamp, not only agreement among wrong fields.
        self.assertEqual(1700000000123456789, OBSERVED_AT_NS)
        self.assertEqual(2, len(result['timestamp_checks']))
        for check in result['timestamp_checks']:
            for field in ('perception_observed_at_ns', 'source_observed_at_ns',
                          'tf_query_time_ns', 'projected_observed_at_ns'):
                self.assertIs(type(check[field]), int)
                self.assertEqual(1700000000123456789, check[field])
            self.assertEqual(1700000000123456790, check['shifted_tf_query_time_ns'])
            self.assertEqual({'code': 'STALE_TRANSFORM',
                              'detail_code': 'rescue_mapping.TF_TIME_MISMATCH'}, check['shifted_tf_error'])

    def test_stop_then_new_generation_and_real_receipt_promote_reservation_only(self):
        result = self.run_demo()
        for handoff in result['synthetic_search']:
            self.assertEqual(['SEARCH_SEND', 'CANCEL_REQUEST', 'TERMINAL_CANCELED', 'STOP_CONFIRMED',
                              'NEW_GENERATION', 'APPROACH_ADMISSION', 'ALLOCATOR_RECORD_ADMISSION',
                              'APPROACH_SEND_BLOCKED'], handoff['sequence'])
            search, approach = handoff['search_admission'], handoff['approach_admission']
            self.assertTrue(search['accepted'])
            self.assertTrue(approach['accepted'])
            for key in ('robot_id', 'robot_boot_id', 'authority_epoch'):
                self.assertEqual(search[key], approach[key])
            self.assertGreater(approach['generation'], search['generation'])
            self.assertEqual(handoff['new_generation'], approach['generation'])
            self.assertNotEqual(search['command_id'], approach['command_id'])
            self.assertNotEqual(search['task_id'], approach['task_id'])
            self.assertFalse(handoff['allocator_active_before_admission'])
            self.assertTrue(handoff['allocator_recorded_admission'])
            self.assertTrue(handoff['allocator_active_after_admission'])
            self.assertFalse(handoff['approach_dispatched'])
            self.assertEqual([], handoff['approach_intents'])
            self.assertEqual('STOPPING', handoff['approach_executor_state'])
            self.assertEqual('GUARD_INVALID', handoff['approach_stop_cause'])

    def test_evidence_bytes_and_two_reports_survive_receipt_loss_and_reopen(self):
        result = self.run_demo()
        self.assertEqual({'states': ['PERSISTED', 'PERSISTED'], 'attempts': [2, 1],
                          'state_after_lost_receipt': 'RETRY_PENDING'}, result['outbox_recovery'])
        self.assertTrue(result['checks']['declared_evidence_waits_for_actual_bytes'])
        reports = read_reports(self.output / 'operations.sqlite3')['reports']
        self.assertEqual(2, len(reports))
        with ReportOutbox(self.output / 'outbox.sqlite3', allowed_robot_ids=('robot_a', 'robot_b')) as outbox:
            self.assertEqual([], outbox.pending())
            for item in reports:
                payload, receipt = item['payload'], item['receipt']
                self.assertEqual('PERSISTED', receipt['status'])
                self.assertEqual('UNCONFIRMED', payload['confirmation_result'])
                self.assertEqual('NOT_ATTEMPTED', payload['approach_result'])
                self.assertIsNone(payload['position'])
                self.assertEqual(OBSERVED_AT_NS, payload['observed_at_ns'])
                self.assertEqual(outbox.get(payload['report_id'])['receipt'], receipt)
        db_path = (self.output / 'operations.sqlite3').resolve().as_uri() + '?mode=ro'
        with sqlite3.connect(db_path, uri=True) as db:
            blobs = dict(db.execute('SELECT evidence_id, content FROM evidence'))
        self.assertEqual(2, len(blobs))
        for item in reports:
            ref = item['payload']['evidence_refs'][0]
            self.assertEqual(item['payload']['observation_id'], ref['observation_id'])
            self.assertEqual(hashlib.sha256(blobs[ref['evidence_id']]).hexdigest(), ref['sha256'])
            self.assertTrue(blobs[ref['evidence_id']].startswith(b'\x89PNG\r\n\x1a\n'))
        saved = json.loads((self.output / 'summary.json').read_text())
        self.assertEqual(result['profile_sha256'], hashlib.sha256(self.profile.read_bytes()).hexdigest())
        self.assertEqual('E2_SYNTHETIC_IN_PROCESS', saved['evidence_level'])
        self.assertIn('no camera', saved['synthetic_evidence'])

    def test_existing_output_is_refused_without_overwriting(self):
        self.run_demo()
        before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.output.iterdir() if p.is_file()}
        with self.assertRaises(FileExistsError):
            self.run_demo()
        after = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in self.output.iterdir() if p.is_file()}
        self.assertEqual(before, after)

    def test_invalid_profile_does_not_create_output(self):
        data = json.loads(self.profile.read_text())
        data['hosts'].pop()
        self.profile.write_text(json.dumps(data))
        with self.assertRaises(ProfileError):
            self.run_demo()
        self.assertFalse(self.output.exists())
        response = self.cli()
        self.assertEqual(2, response.returncode)
        self.assertIn('OFFLINE REHEARSAL FAILED', response.stderr)
        self.assertNotIn('Traceback', response.stderr)
        self.assertFalse(self.output.exists())

    def test_valid_profile_with_other_robot_names_is_not_silently_ignored(self):
        data = json.loads(self.profile.read_text())
        data['robots'][0]['id'] = 'robot_x'
        params = self.profile.parent / data['robots'][0]['parameters_file']
        values = json.loads(params.read_text())
        values['robot_id'] = 'robot_x'
        params.write_text(json.dumps(values))
        self.profile.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'requires logical robot_a and robot_b'):
            self.run_demo()
        self.assertFalse(self.output.exists())

    def test_module_cli_runs_and_existing_path_fails_cleanly(self):
        response = self.cli()
        self.assertEqual(0, response.returncode, response.stderr)
        self.assertIn('E2 SYNTHETIC PASS', response.stdout)
        self.assertIn('E3/E4 NOT_RUN', response.stdout)
        self.assertIn('automatic approach BLOCKED', response.stdout)
        self.assertTrue(json.loads((self.output / 'summary.json').read_text())['passed'])
        repeated = self.cli()
        self.assertEqual(2, repeated.returncode)
        self.assertNotIn('Traceback', repeated.stderr)


if __name__ == '__main__':
    unittest.main()
