"""PR10 review regressions for profile-bound report ownership."""
from pathlib import Path
import tempfile
import unittest
import sqlite3
from rescue_operations.ledger import OperationsError, ReportLedger, ReportOutbox, read_reports
from rescue_operations.schema import report
from test_operations import sample_report, sample_envelope


class OperationsReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.payload = sample_report()
        self.payload.update(source_robot='robot3', robot_id='robot7')
        self.allowed = ('robot3', 'robot7')

    def test_schema_accepts_valid_identifier_without_fixed_membership(self):
        self.assertEqual('robot3', report(self.payload)['source_robot'])

    def test_profile_ids_reach_persisted_report_and_outbox(self):
        with ReportLedger(self.root/'ledger.db', evidence_policy='OPTIONAL', allowed_robot_ids=self.allowed) as ledger:
            with ReportOutbox(self.root/'outbox.db', allowed_robot_ids=self.allowed) as outbox:
                outbox.enqueue(self.payload)
                receipt = ledger.receive(self.payload, sample_envelope())
                self.assertEqual('PERSISTED', ledger.persist(receipt.report_id).status)
        snapshot = read_reports(self.root/'ledger.db')
        self.assertEqual(list(self.allowed), snapshot['allowed_robot_ids'])
        self.assertEqual('robot7', snapshot['reports'][0]['payload']['robot_id'])

    def test_unknown_source_or_executor_robot_is_rejected_before_storage(self):
        with ReportLedger(self.root/'ledger.db', evidence_policy='OPTIONAL', allowed_robot_ids=self.allowed) as ledger:
            with ReportOutbox(self.root/'outbox.db', allowed_robot_ids=self.allowed) as outbox:
                for field in ('source_robot', 'robot_id'):
                    payload = dict(self.payload, **{field:'outsider'})
                    for operation in (lambda: ledger.receive(payload, sample_envelope()), lambda: outbox.enqueue(payload)):
                        with self.assertRaises(OperationsError) as caught:
                            operation()
                        self.assertEqual('WRONG_ROBOT', caught.exception.code)
                self.assertEqual([], ledger.list_reports())
                self.assertEqual([], outbox.pending())

    def test_allowed_ids_are_immutable_and_pinned_on_reopen(self):
        allowed = list(self.allowed)
        with ReportLedger(self.root/'ledger.db', evidence_policy='OPTIONAL', allowed_robot_ids=allowed) as ledger:
            allowed.append('outsider')
            with self.assertRaises(AttributeError):
                ledger.allowed_robot_ids = ('robot3', 'outsider')
            with self.assertRaises(OperationsError):
                ledger.receive(dict(self.payload, robot_id='outsider'), sample_envelope())
        with ReportLedger(self.root/'ledger.db', evidence_policy='OPTIONAL', allowed_robot_ids=tuple(reversed(self.allowed))):
            pass
        with self.assertRaises(OperationsError) as caught:
            ReportLedger(self.root/'ledger.db', evidence_policy='OPTIONAL', allowed_robot_ids=('robot3','other'))
        self.assertEqual('rescue_operations.POLICY_MISMATCH', caught.exception.detail_code)
        with ReportOutbox(self.root/'outbox.db', allowed_robot_ids=self.allowed):
            pass
        with self.assertRaises(OperationsError):
            ReportOutbox(self.root/'outbox.db', allowed_robot_ids=('robot3','other'))

    def test_allowed_id_policy_is_required_and_bounded(self):
        with self.assertRaises(TypeError):
            ReportLedger(self.root/'missing.db', evidence_policy='OPTIONAL')
        with self.assertRaises(TypeError):
            ReportOutbox(self.root/'missing.db')
        for values in ((), ('robot3','robot3'), ('bad/id','robot7'), 'robot3'):
            with self.assertRaises(OperationsError):
                ReportOutbox(self.root/'bad.db', allowed_robot_ids=values)

    def test_operations_error_uses_common_envelope_without_losing_detail(self):
        error = OperationsError('STORAGE_ERROR', 'fixture')
        self.assertEqual('REPORT_NOT_PERSISTED', error.code)
        self.assertEqual('rescue_operations.STORAGE_ERROR', error.detail_code)
        self.assertEqual('rescue_operations', error.component)

    def test_missing_robot_policy_is_not_silently_adopted_on_reopen(self):
        path = self.root/'legacy.db'
        with sqlite3.connect(path) as db:
            db.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            db.executemany('INSERT INTO metadata VALUES (?, ?)',
                           [('schema','1.0'), ('role','operations-ledger'), ('evidence_policy','OPTIONAL')])
        with self.assertRaises(OperationsError) as caught:
            ReportLedger(path, evidence_policy='OPTIONAL', allowed_robot_ids=self.allowed)
        self.assertEqual('rescue_operations.POLICY_MISMATCH', caught.exception.detail_code)
        with self.assertRaises(OperationsError):
            read_reports(path)
        with sqlite3.connect(path) as db:
            self.assertEqual(3, db.execute('SELECT count(*) FROM metadata').fetchone()[0])

    def test_schema_robot_identifiers_follow_profile_syntax(self):
        for robot in ('bad/id', 'two words', '', '1robot', 'a'*65, None):
            for key in ('source_robot', 'robot_id'):
                with self.subTest(robot=robot, key=key), self.assertRaises(OperationsError):
                    report(dict(self.payload, **{key: robot}))
