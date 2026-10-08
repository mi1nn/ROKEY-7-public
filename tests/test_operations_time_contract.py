"""PR10 regressions: report, receipt and evidence use common time/UUID ranges."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from rescue_contracts import MAX_TIMESTAMP_NS
from rescue_operations.ledger import OperationsError, Receipt, ReportOutbox
from rescue_operations.schema import envelope, evidence_ref, identity, report
from test_operations import sample_report, sample_envelope, add_evidence, new_id

NIL = '00000000-0000-0000-0000-000000000000'


class OperationsTimeContractTests(unittest.TestCase):
    def assertInvalid(self, operation):
        with self.assertRaises(OperationsError) as caught:
            operation()
        self.assertEqual('INVALID_INPUT', caught.exception.code)
        self.assertEqual('rescue_operations.INVALID_INPUT', caught.exception.detail_code)

    def test_report_times_are_int64_while_generation_and_revisions_remain_uint64(self):
        payload = sample_report()
        payload.update(observed_at_ns=MAX_TIMESTAMP_NS+1, confirmed_at_ns=MAX_TIMESTAMP_NS+2,
                       reported_at_ns=MAX_TIMESTAMP_NS+3)
        self.assertInvalid(lambda: report(payload))
        payload.update(observed_at_ns=MAX_TIMESTAMP_NS-2, confirmed_at_ns=MAX_TIMESTAMP_NS-1,
                       reported_at_ns=MAX_TIMESTAMP_NS, generation=2**64-1)
        payload['position']['map_revision'] = 2**64-1
        self.assertEqual(MAX_TIMESTAMP_NS, report(payload)['reported_at_ns'])

    def test_envelope_and_evidence_times_are_positive_int64(self):
        env = sample_envelope()
        env.update(sequence=2**64-1, published_at_ns=MAX_TIMESTAMP_NS+1)
        self.assertInvalid(lambda: envelope(env))
        env['published_at_ns'] = MAX_TIMESTAMP_NS
        self.assertEqual(2**64-1, envelope(env)['sequence'])
        ref, _ = add_evidence(sample_report())
        self.assertInvalid(lambda: evidence_ref(dict(ref, acquired_at_ns=MAX_TIMESTAMP_NS+1)))
        self.assertEqual(MAX_TIMESTAMP_NS, evidence_ref(dict(ref, acquired_at_ns=MAX_TIMESTAMP_NS))['acquired_at_ns'])

    def test_nil_uuid_cannot_cross_report_envelope_or_evidence_boundaries(self):
        for key in ('report_id', 'target_id', 'observation_id', 'source_boot_id', 'task_id',
                    'confirmation_observation_id'):
            with self.subTest(field=key):
                self.assertInvalid(lambda: report(dict(sample_report(), **{key: NIL})))
        payload = sample_report()
        payload['position']['map_session_id'] = NIL
        self.assertInvalid(lambda: report(payload))
        self.assertInvalid(lambda: envelope(dict(sample_envelope(), publisher_boot_id=NIL)))
        ref, _ = add_evidence(sample_report())
        for key in ('evidence_id', 'observation_id'):
            with self.subTest(field=key):
                self.assertInvalid(lambda: evidence_ref(dict(ref, **{key: NIL})))

    def test_invalid_persisted_receipt_is_not_accepted_by_outbox(self):
        payload = report(sample_report())
        valid = Receipt(new_id(), payload['report_id'], identity(payload), 'PERSISTED', 1,
                        MAX_TIMESTAMP_NS, None)
        with tempfile.TemporaryDirectory() as directory:
            with ReportOutbox(Path(directory)/'outbox.db', allowed_robot_ids=('robot_a', 'robot_b')) as outbox:
                outbox.enqueue(payload)
                for invalid in (replace(valid, persisted_at_ns=MAX_TIMESTAMP_NS+1),
                                replace(valid, receipt_id=NIL)):
                    with self.subTest(receipt=invalid):
                        self.assertIsNone(outbox.deliver(payload['report_id'], sample_envelope(),
                                                         lambda *_: invalid))
                        pending = outbox.get(payload['report_id'])
                        self.assertEqual('RETRY_PENDING', pending['state'])
                        self.assertIn('rescue_operations.INVALID_INPUT', pending['last_error'])
                self.assertEqual('PERSISTED', outbox.deliver(payload['report_id'], sample_envelope(),
                                                           lambda *_: valid).status)
