"""구조 보고 저장 장부(RescueReport -> ReportReceipt)의 E2 시험. 모두 합성 입력이다.

조정기는 저장 영수증(PERSISTED, 보고의 record_ids 전부 포함)을 받을 때까지 같은 보고를 다시 보낸다
(rescue_coordination.ros_node ``reports_due``·``on_receipt``). 그래서 같은 보고는 한 번만 저장하고 같은
영수증을 돌려줘야 하며, 저장하지 못한 것은 REJECTED 와 이유로 알려야 한다(성공으로 바꾸지 않음).
"""
import tempfile
import unittest
from pathlib import Path
from uuid import UUID

from rescue_operations.schema import OperationsError
from rescue_operations.report_store import RescueReportStore, receipt_fields

ENV = {"schema_version": "0.1", "publisher_id": "central.coordinator", "publisher_boot_id": str(UUID(int=900)),
       "sequence": 3, "published_at_ns": 1700000000_000000000}


def report(**overrides):
    """message_to_ordereddict(RescueReport) 모양(조정기 publish_report 가 채우는 필드)."""
    base = {
        "envelope": dict(ENV), "report_id": str(UUID(int=1)), "target_id": str(UUID(int=2)), "task_id": "",
        "generation": 0, "record_ids": [str(UUID(int=10)), str(UUID(int=11))],
        "locations": [{"location_id": str(UUID(int=10))}, {"location_id": str(UUID(int=11))}],
        "site_events": [], "rechecks": [], "routes": [], "approvals": [], "task_admissions": [],
        "has_task_result": False, "task_result": {}, "confirmation_state": "CONFIRMED",
        "approach_state": "UNASSIGNED", "rescue_state": "NOT_CHECKED", "evidence_refs": [],
        "first_issued_at_ns": 1700000000_000000000, "meaning_sha256": "a" * 64,
    }
    base.update(overrides)
    return base


class ReportStoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "reports.sqlite"
        self.store = RescueReportStore(self.path)

    def tearDown(self):
        self.store.close()
        self.tmp.cleanup()

    def test_new_report_is_persisted_with_all_record_ids(self):
        r = self.store.save(report())
        self.assertEqual((r.state, r.report_id, r.meaning_sha256, r.storage_revision, r.reason_code),
                         ("PERSISTED", str(UUID(int=1)), "a" * 64, 1, ""))
        self.assertEqual(r.included_record_ids, (str(UUID(int=10)), str(UUID(int=11))))
        self.assertIsInstance(r.persisted_at_ns, int)
        UUID(r.receipt_id)

    def test_resend_stores_once_and_returns_same_receipt(self):
        first = self.store.save(report())
        again = self.store.save(report(envelope=dict(ENV, sequence=4, published_at_ns=ENV["published_at_ns"] + 5)))
        self.assertEqual(first, again)  # 전송 envelope 만 다른 재전송
        self.assertEqual(len(self.store.list_reports()), 1)

    def test_resend_after_reopen_returns_same_receipt(self):
        first = self.store.save(report())
        self.store.close()
        self.store = RescueReportStore(self.path)
        self.assertEqual(self.store.save(report()), first)

    def test_same_id_different_content_is_rejected_and_original_kept(self):
        first = self.store.save(report())
        r = self.store.save(report(record_ids=[str(UUID(int=10))]))
        self.assertEqual((r.state, r.reason_code, r.included_record_ids, r.persisted_at_ns),
                         ("REJECTED", "ID_CONTENT_CONFLICT", (), None))
        self.assertEqual(self.store.save(report()), first)

    def test_invalid_reports_are_rejected_and_not_stored(self):
        bad = (dict(record_ids=[]), dict(record_ids=["x"]), dict(record_ids=[str(UUID(int=10))] * 2),
               dict(target_id=""), dict(meaning_sha256="A" * 64), dict(first_issued_at_ns=0),
               dict(report_id="r1"))
        for change in bad:
            r = self.store.save(report(**change))
            self.assertEqual((r.state, r.reason_code, r.included_record_ids), ("REJECTED", "INVALID_INPUT", ()),
                             change)
        missing = report()
        del missing["record_ids"]
        self.assertEqual(self.store.save(missing).reason_code, "INVALID_INPUT")
        self.assertEqual(self.store.list_reports(), [])

    def test_storage_failure_is_rejected_not_persisted(self):
        self.store.close()
        r = self.store.save(report())
        self.assertEqual((r.state, r.reason_code, r.included_record_ids), ("REJECTED", "STORAGE_ERROR", ()))

    def test_bad_db_path_fails_at_start(self):
        with self.assertRaises(OperationsError):
            RescueReportStore(Path(self.tmp.name) / "missing-dir" / "reports.sqlite")


class ReceiptFieldsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = RescueReportStore(Path(self.tmp.name) / "reports.sqlite")

    def tearDown(self):
        self.store.close()
        self.tmp.cleanup()

    def test_persisted_receipt_fields(self):
        f = receipt_fields(self.store.save(report()))
        self.assertEqual((f["state"], f["has_persisted_at"], f["storage_revision"], f["reason_code"]),
                         ("PERSISTED", True, 1, ""))
        self.assertEqual(f["included_record_ids"], [str(UUID(int=10)), str(UUID(int=11))])
        self.assertGreater(f["persisted_at_ns"], 0)

    def test_rejected_receipt_claims_nothing(self):
        f = receipt_fields(self.store.save(report(record_ids=[])))
        self.assertEqual((f["state"], f["has_persisted_at"], f["persisted_at_ns"], f["included_record_ids"],
                          f["storage_revision"], f["reason_code"]), ("REJECTED", False, 0, [], 0, "INVALID_INPUT"))
        self.assertEqual(f["report_id"], str(UUID(int=1)))


if __name__ == "__main__":
    unittest.main()
