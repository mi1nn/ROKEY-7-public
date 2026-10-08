"""PR9 CT-12: real SQLite reopen, evidence transactions and receipt loss."""

import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest
import uuid

from rescue_operations.ledger import (LedgerTransport, OperationsError,
                                      ReportLedger, ReportOutbox, read_reports)
from rescue_operations.schema import MAX_EVIDENCE_BYTES, report


def new_id():
    return str(uuid.uuid4())


def sample_report():
    return {
        "schema_version": "1.0", "report_id": new_id(), "target_id": new_id(),
        "observation_id": new_id(), "source_robot": "robot_a", "source_camera": "front_rgb",
        "source_boot_id": new_id(), "clock_domain_id": "offline-fixture",
        "robot_id": "robot_b", "task_id": new_id(), "generation": 1,
        "observation_result": "DETECTED", "confirmation_result": "CONFIRMED",
        "confirmation_observation_id": new_id(), "approach_result": "APPROACH_COMPLETE",
        "position": {"x": 1.0, "y": 2.0, "z": 0.0, "frame_id": "mission",
                     "map_session_id": new_id(), "map_revision": 2, "transform_revision": 3,
                     "authority_id": "mapping"},
        "observed_at_ns": 100, "confirmed_at_ns": 200, "reported_at_ns": 300,
        "reason": "Synthetic stop and confirmation fixture; not a physical measurement",
        "evidence_refs": [], "model_id": "synthetic-detector", "model_sha256": "a" * 64,
        "deployment_version": "offline-test",
    }


def sample_envelope(sequence=0):
    return {"schema_version": "1.0", "publisher_id": "coordinator",
            "publisher_boot_id": new_id(), "sequence": sequence, "published_at_ns": 400 + sequence}


def add_evidence(payload, confirmation=True):
    content = b"synthetic evidence bytes, not a photograph"
    ref = {"evidence_id": new_id(), "sha256": hashlib.sha256(content).hexdigest(),
           "media_type": "image/png",
           "observation_id": payload["confirmation_observation_id"] if confirmation else payload["observation_id"],
           "acquired_at_ns": payload["confirmed_at_ns"] if confirmation else payload["observed_at_ns"]}
    payload["evidence_refs"].append(ref)
    return ref, content


class OperationsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.db = self.root / "operations.sqlite3"
        self.sender = self.root / "outbox.sqlite3"
        self.payload = sample_report()

    def test_received_is_not_persisted_and_restart_retains_receipt(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            received = ledger.receive(self.payload, sample_envelope())
            self.assertEqual("RECEIVED", received.status)
            self.assertIsNone(received.persisted_at_ns)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            self.assertEqual(received.to_dict(), ledger.get(self.payload["report_id"])["receipt"])
            persisted = ledger.persist(self.payload["report_id"])
            self.assertEqual("PERSISTED", persisted.status)
            self.assertEqual(2, persisted.revision)
            self.assertNotEqual(received.receipt_id, persisted.receipt_id)
        self.assertEqual(persisted.to_dict(), read_reports(self.db)["reports"][0]["receipt"])

    def test_envelope_changes_are_retries_not_collisions(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            initial = ledger.receive(self.payload, sample_envelope())
            self.assertEqual(initial, ledger.receive(self.payload, sample_envelope(200)))
            final = ledger.persist(self.payload["report_id"])
            self.assertEqual(final, ledger.receive(self.payload, sample_envelope(500)))
            self.assertEqual(1, len(ledger.list_reports()))

    def test_report_id_collision_does_not_mutate_original(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            original = ledger.receive(self.payload, sample_envelope())
            changed = copy.deepcopy(self.payload)
            changed["reason"] = "changed meaning"
            with self.assertRaisesRegex(OperationsError, "ID_CONTENT_CONFLICT"):
                ledger.receive(changed, sample_envelope(1))
            self.assertEqual(original.to_dict(), ledger.get(self.payload["report_id"])["receipt"])
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox:
            outbox.enqueue(self.payload)
            with self.assertRaisesRegex(OperationsError, "ID_CONTENT_CONFLICT"):
                outbox.enqueue(changed)

    def test_required_evidence_waits_for_actual_bytes_then_survives_reopen(self):
        ref, content = add_evidence(self.payload)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            ledger.receive(self.payload, sample_envelope())
            waiting = ledger.persist(self.payload["report_id"])
            self.assertEqual("PENDING_EVIDENCE", waiting.status)
            self.assertEqual(waiting, ledger.persist(self.payload["report_id"]))
            with self.assertRaisesRegex(OperationsError, "EVIDENCE_HASH_MISMATCH"):
                ledger.store_evidence(ref, b"damaged upload")
            self.assertEqual(waiting, ledger.persist(self.payload["report_id"]))
            ledger.store_evidence(ref, content)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            final = ledger.persist(self.payload["report_id"])
            self.assertEqual("PERSISTED", final.status)
            self.assertEqual(3, final.revision)
            self.assertEqual(final, ledger.persist(self.payload["report_id"]))
        with sqlite3.connect(self.db) as db:
            self.assertEqual(content, db.execute("SELECT content FROM evidence").fetchone()[0])

    def test_missing_ref_and_original_photo_do_not_satisfy_confirmation_policy(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            ledger.receive(self.payload, sample_envelope())
            self.assertEqual("PENDING_EVIDENCE", ledger.persist(self.payload["report_id"]).status)
            other = sample_report()
            ref, content = add_evidence(other, confirmation=False)
            ledger.store_evidence(ref, content)
            ledger.receive(other, sample_envelope())
            self.assertEqual("PENDING_EVIDENCE", ledger.persist(other["report_id"]).status)

    def test_optional_policy_does_not_persist_dangling_declared_evidence(self):
        add_evidence(self.payload)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            ledger.receive(self.payload, sample_envelope())
            self.assertEqual("PENDING_EVIDENCE", ledger.persist(self.payload["report_id"]).status)

    def test_evidence_id_metadata_collision_rejected(self):
        ref, content = add_evidence(self.payload)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            ledger.store_evidence(ref, content)
            ledger.store_evidence(ref, content)
            conflicting = dict(ref, acquired_at_ns=999)
            with self.assertRaisesRegex(OperationsError, "ID_CONTENT_CONFLICT"):
                ledger.store_evidence(conflicting, content)
            self.payload["evidence_refs"][0] = dict(ref, media_type="image/jpeg")
            ledger.receive(self.payload, sample_envelope())
            with self.assertRaisesRegex(OperationsError, "ID_CONTENT_CONFLICT"):
                ledger.persist(self.payload["report_id"])

    def test_upload_transaction_failure_does_not_claim_stored(self):
        ref, content = add_evidence(self.payload)
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            ledger.receive(self.payload, sample_envelope())
            with sqlite3.connect(self.db) as db:
                db.execute("CREATE TRIGGER fail_upload BEFORE INSERT ON evidence BEGIN SELECT RAISE(ABORT, 'fixture failure'); END")
            with self.assertRaisesRegex(OperationsError, "STORAGE_ERROR"):
                ledger.store_evidence(ref, content)
            self.assertEqual("PENDING_EVIDENCE", ledger.persist(self.payload["report_id"]).status)
            with sqlite3.connect(self.db) as db:
                self.assertEqual(0, db.execute("SELECT COUNT(*) FROM evidence").fetchone()[0])
                db.execute("DROP TRIGGER fail_upload")
            ledger.store_evidence(ref, content)
            self.assertEqual("PERSISTED", ledger.persist(self.payload["report_id"]).status)

    def test_report_transaction_failure_rolls_back_and_retry_succeeds(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            with sqlite3.connect(self.db) as db:
                db.execute("CREATE TRIGGER fail_report BEFORE INSERT ON reports BEGIN SELECT RAISE(ABORT, 'fixture failure'); END")
            with self.assertRaisesRegex(OperationsError, "STORAGE_ERROR"):
                ledger.receive(self.payload, sample_envelope())
            self.assertEqual([], ledger.list_reports())
            with sqlite3.connect(self.db) as db:
                db.execute("DROP TRIGGER fail_report")
            self.assertEqual("RECEIVED", ledger.receive(self.payload, sample_envelope()).status)

    def test_persist_write_failure_never_returns_persisted_receipt(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            original = ledger.receive(self.payload, sample_envelope())
            with sqlite3.connect(self.db) as db:
                db.execute("CREATE TRIGGER fail_persist BEFORE UPDATE ON reports BEGIN SELECT RAISE(ABORT, 'fixture failure'); END")
            with self.assertRaisesRegex(OperationsError, "STORAGE_ERROR"):
                ledger.persist(self.payload["report_id"])
            self.assertEqual(original.to_dict(), ledger.get(self.payload["report_id"])["receipt"])
            with sqlite3.connect(self.db) as db:
                db.execute("DROP TRIGGER fail_persist")
            self.assertEqual("PERSISTED", ledger.persist(self.payload["report_id"]).status)

    def test_outbox_receipt_write_failure_replays_committed_remote_report(self):
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            outbox.enqueue(self.payload)
            with sqlite3.connect(self.sender) as db:
                db.execute("CREATE TRIGGER fail_ack BEFORE UPDATE OF receipt_json ON outbox BEGIN SELECT RAISE(ABORT, 'fixture failure'); END")
            with self.assertRaisesRegex(OperationsError, "STORAGE_ERROR"):
                outbox.deliver(self.payload["report_id"], sample_envelope(), LedgerTransport(ledger))
            self.assertEqual("QUEUED", outbox.get(self.payload["report_id"])["state"])
            self.assertEqual("PERSISTED", ledger.get(self.payload["report_id"])["receipt"]["status"])
            with sqlite3.connect(self.sender) as db:
                db.execute("DROP TRIGGER fail_ack")
            self.assertEqual("PERSISTED", outbox.deliver(self.payload["report_id"], sample_envelope(1), LedgerTransport(ledger)).status)
            self.assertEqual(1, len(ledger.list_reports()))

    def test_receipt_loss_after_commit_retries_without_duplicate(self):
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            outbox.enqueue(self.payload)
            transport = LedgerTransport(ledger)
            def lose_receipt(payload, env):
                transport(payload, env)
                raise TimeoutError("simulated response lost after commit")
            self.assertIsNone(outbox.deliver(self.payload["report_id"], sample_envelope(), lose_receipt))
            self.assertEqual("RETRY_PENDING", outbox.get(self.payload["report_id"])["state"])
            self.assertEqual("PERSISTED", ledger.get(self.payload["report_id"])["receipt"]["status"])
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            final = outbox.deliver(self.payload["report_id"], sample_envelope(1), LedgerTransport(ledger))
            self.assertEqual("PERSISTED", final.status)
            self.assertEqual([], outbox.pending())
            self.assertEqual(2, outbox.get(self.payload["report_id"])["attempts"])
            self.assertEqual(1, len(ledger.list_reports()))
            self.assertEqual(final, outbox.deliver(self.payload["report_id"], sample_envelope(2), lambda *_: self.fail("must not resend")))

    def test_received_and_pending_receipts_remain_retryable(self):
        ref, content = add_evidence(self.payload)
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            outbox.enqueue(self.payload)
            got = outbox.deliver(self.payload["report_id"], sample_envelope(), ledger.receive)
            self.assertEqual("RECEIVED", got.status)
            self.assertEqual([self.payload["report_id"]], outbox.pending())
            got = outbox.deliver(self.payload["report_id"], sample_envelope(1), LedgerTransport(ledger))
            self.assertEqual("PENDING_EVIDENCE", got.status)
            ledger.store_evidence(ref, content)
            self.assertEqual("PERSISTED", outbox.deliver(self.payload["report_id"], sample_envelope(2), LedgerTransport(ledger)).status)

    def test_wrong_report_receipt_cannot_finish_outbox(self):
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            outbox.enqueue(self.payload)
            wrong = LedgerTransport(ledger)(sample_report(), sample_envelope())
            self.assertIsNone(outbox.deliver(self.payload["report_id"], sample_envelope(), lambda *_: wrong))
            self.assertEqual("RETRY_PENDING", outbox.get(self.payload["report_id"])["state"])
            self.assertIn("ID_CONTENT_CONFLICT", outbox.get(self.payload["report_id"])["last_error"])

    def test_stale_receipt_revision_does_not_regress_outbox(self):
        ref, _ = add_evidence(self.payload)
        with ReportOutbox(self.sender, allowed_robot_ids=("robot_a", "robot_b")) as outbox, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="CONFIRMATION") as ledger:
            outbox.enqueue(self.payload)
            early = ledger.receive(self.payload, sample_envelope())
            pending = ledger.persist(self.payload["report_id"])
            outbox.deliver(self.payload["report_id"], sample_envelope(), lambda *_: pending)
            self.assertEqual(pending, outbox.deliver(self.payload["report_id"], sample_envelope(), lambda *_: early))
            self.assertEqual("PENDING_EVIDENCE", outbox.get(self.payload["report_id"])["state"])

    def test_concurrent_ledger_instances_deduplicate(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as first, ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as second:
            with ThreadPoolExecutor(max_workers=4) as pool:
                receipts = list(pool.map(lambda i: (first if i % 2 else second).receive(self.payload, sample_envelope(i)), range(20)))
            self.assertTrue(all(value == receipts[0] for value in receipts))
            self.assertEqual(1, len(first.list_reports()))

    def test_policy_and_role_cannot_change_on_reopen(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OBSERVATION"):
            pass
        with self.assertRaisesRegex(OperationsError, "POLICY_MISMATCH"):
            ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL")
        with self.assertRaisesRegex(OperationsError, "POLICY_MISMATCH"):
            ReportOutbox(self.db, allowed_robot_ids=("robot_a", "robot_b"))

    def test_invalid_schema_numeric_ids_and_medical_claims_rejected(self):
        changes = [("schema_version", "2.0"), ("schema_version", "1.1"), ("report_id", "not-uuid"),
                   ("generation", True), ("observed_at_ns", 0), ("confirmed_at_ns", 50),
                   ("reported_at_ns", 199), ("approach_result", "ALIVE"), ("reason", "x" * 1025)]
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            for key, value in changes:
                with self.subTest(key=key, value=value):
                    bad = copy.deepcopy(self.payload)
                    bad[key] = value
                    with self.assertRaises(OperationsError):
                        ledger.receive(bad, sample_envelope())
            for value in (float("nan"), float("inf"), True, 10**500):
                bad = copy.deepcopy(self.payload)
                bad["position"]["x"] = value
                with self.assertRaises(OperationsError):
                    ledger.receive(bad, sample_envelope())
            bad = dict(self.payload, extra="undeclared")
            with self.assertRaises(OperationsError):
                ledger.receive(bad, sample_envelope())
            self.assertEqual([], ledger.list_reports())

    def test_evidence_cannot_relabel_an_old_photo_as_confirmation(self):
        ref, _ = add_evidence(self.payload, confirmation=False)
        ref["observation_id"] = self.payload["confirmation_observation_id"]
        with self.assertRaises(OperationsError):
            report(self.payload)

    def test_payload_bounds_and_immutable_input_snapshot(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            ledger.receive(self.payload, sample_envelope())
            self.payload["position"]["x"] = 999.0
            self.assertEqual(1.0, ledger.get(self.payload["report_id"])["payload"]["position"]["x"])
            with self.assertRaises(OperationsError):
                ledger.receive(dict(self.payload, reason="x" * 65536), sample_envelope())
            ref, _ = add_evidence(self.payload)
            with self.assertRaises(OperationsError):
                ledger.store_evidence(ref, b"x" * (MAX_EVIDENCE_BYTES + 1))

    def test_unreachable_report_is_recorded_without_claiming_approach_complete(self):
        self.payload.update(approach_result="UNREACHABLE", confirmation_result="UNCONFIRMED",
                            confirmation_observation_id=None, confirmed_at_ns=None, position=None,
                            reason="Synthetic route blocked")
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            final = LedgerTransport(ledger)(self.payload, sample_envelope())
            self.assertEqual("PERSISTED", final.status)
            self.assertEqual("UNREACHABLE", ledger.get(self.payload["report_id"])["payload"]["approach_result"])

    def test_readonly_cli_exports_and_does_not_create_missing_database(self):
        with ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL") as ledger:
            LedgerTransport(ledger)(self.payload, sample_envelope())
        result = subprocess.run([sys.executable, "-m", "rescue_operations", str(self.db)],
                                text=True, capture_output=True, check=False)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("PERSISTED", json.loads(result.stdout)["reports"][0]["receipt"]["status"])
        missing = self.root / "missing.sqlite3"
        result = subprocess.run([sys.executable, "-m", "rescue_operations", str(missing)],
                                text=True, capture_output=True, check=False)
        self.assertEqual(2, result.returncode)
        self.assertNotIn("Traceback", result.stderr)
        self.assertFalse(missing.exists())

    def test_invalid_paths_and_closed_db_normalize_errors(self):
        for path in (":memory:", "\x00", self.root):
            with self.subTest(path=str(path)), self.assertRaises(OperationsError):
                ReportLedger(path, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL")
        ledger = ReportLedger(self.db, allowed_robot_ids=("robot_a", "robot_b"), evidence_policy="OPTIONAL")
        ledger.close()
        ledger.close()
        with self.assertRaisesRegex(OperationsError, "STORAGE_ERROR"):
            ledger.list_reports()


if __name__ == "__main__":
    unittest.main()
