"""조정기 구조 보고(RescueReport, IDL) 저장 장부와 영수증(ReportReceipt) 필드. ROS 없음.

조정기는 대상을 찾으면 ``/rescue/reports`` 로 보고를 보내고, 영수증 PERSISTED 가 보고의
``record_ids`` 를 전부 담아 돌아올 때까지 같은 보고를 다시 보낸다(rescue_coordination.ros_node
``reports_due``·``on_receipt``). 그 영수증이 있어야 출동 승인이 열린다(``persisted_report_id``).

- 같은 ``report_id`` 와 같은 내용은 한 번만 저장하고, 다시 오면 처음 영수증을 그대로 돌려준다.
  전송 envelope(sequence·발행 시각)는 재전송마다 다르므로 내용 비교에서 뺀다.
- 같은 ``report_id`` 인데 내용이 다르면 덮어쓰지 않고 REJECTED(ID_CONTENT_CONFLICT).
- 형식이 틀리면 REJECTED(INVALID_INPUT), DB 를 쓰지 못하면 REJECTED(STORAGE_ERROR). 거절 영수증은
  아무 기록도 저장했다고 주장하지 않는다(included_record_ids 비움, persisted_at 없음).
- 기존 ``ledger.ReportLedger`` 는 schema 1.0 보고 형식(observation_id·model_sha256 등)을 요구해서 IDL 보고를
  그대로 넣을 수 없다. 없는 필드를 지어내지 않고 장부를 따로 둔다. DB 기반(``_Database``)은 같다.
- DB 파일은 site_state DB 와 다른 파일이어야 한다(파일당 쓰는 프로세스 하나, db-schema).
"""
from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import time
import uuid

from .ledger import _Database
from .schema import OperationsError, canonical, require, sha, timestamp, uid

_ROLE = "rescue-report-store"
# 조정기가 채우는 필드 중 장부가 검사하는 것. 나머지 필드는 내용 그대로 저장한다.
_REQUIRED = ("report_id", "target_id", "record_ids", "meaning_sha256", "first_issued_at_ns")


@dataclass(frozen=True)
class StoredReceipt:
    report_id: str
    receipt_id: str
    state: str  # PERSISTED 또는 REJECTED
    meaning_sha256: str
    included_record_ids: tuple
    persisted_at_ns: int | None
    storage_revision: int
    reason_code: str


def _reason(exc):
    return exc.detail_code.rsplit(".", 1)[-1]


def _validate(payload):
    """보고 dict -> (envelope 를 뺀 내용, 내용 해시). 틀리면 OperationsError(INVALID_INPUT)."""
    require(isinstance(payload, Mapping), "report must be an object")
    item = json.loads(canonical({k: v for k, v in payload.items() if k != "envelope"}))
    require(all(key in item for key in _REQUIRED), f"report must have {', '.join(_REQUIRED)}")
    uid(item["report_id"], "report_id")
    uid(item["target_id"], "target_id")
    records = item["record_ids"]
    require(isinstance(records, list) and records, "record_ids must be a nonempty list")
    for record in records:
        uid(record, "record_ids")
    require(len(set(records)) == len(records), "record_ids must be unique")
    sha(item["meaning_sha256"], "meaning_sha256")
    timestamp(item["first_issued_at_ns"], "first_issued_at_ns")
    return item, hashlib.sha256(canonical(item).encode()).hexdigest()


def _stored(row):
    return StoredReceipt(row["report_id"], row["receipt_id"], "PERSISTED", row["meaning_sha256"],
                         tuple(json.loads(row["record_ids_json"])), row["persisted_at_ns"], row["revision"], "")


def _rejected(payload, code):
    get = payload.get if isinstance(payload, Mapping) else (lambda key: None)
    report_id, meaning = get("report_id"), get("meaning_sha256")
    return StoredReceipt(report_id if isinstance(report_id, str) else "", str(uuid.uuid4()), "REJECTED",
                         meaning if isinstance(meaning, str) else "", (), None, 0, code)


class RescueReportStore(_Database):
    """조정기 RescueReport 장부(쓰는 프로세스 하나). ``save`` 는 예외 대신 거절 영수증을 돌려준다."""

    def __init__(self, db_path):
        super().__init__(db_path, _ROLE, tables=("""CREATE TABLE IF NOT EXISTS rescue_reports (
            report_id TEXT PRIMARY KEY, content_sha256 TEXT NOT NULL, meaning_sha256 TEXT NOT NULL,
            target_id TEXT NOT NULL, record_ids_json TEXT NOT NULL, payload_json TEXT NOT NULL,
            envelope_json TEXT NOT NULL, receipt_id TEXT NOT NULL, revision INTEGER NOT NULL,
            received_at_ns INTEGER NOT NULL, persisted_at_ns INTEGER NOT NULL)""",))

    def save(self, payload):
        try:
            item, digest = _validate(payload)
            envelope_json = canonical(payload.get("envelope"))
            report_id = item["report_id"]
            with self._transaction() as db:
                row = db.execute("SELECT * FROM rescue_reports WHERE report_id = ?", (report_id,)).fetchone()
                if row is not None:
                    require(row["content_sha256"] == digest, "report_id has different content", "ID_CONTENT_CONFLICT")
                    return _stored(row)
                now = timestamp(time.time_ns(), "persisted_at_ns")
                db.execute("INSERT INTO rescue_reports VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                           (report_id, digest, item["meaning_sha256"], item["target_id"],
                            canonical(item["record_ids"]), canonical(item), envelope_json,
                            str(uuid.uuid4()), 1, now, now))
                row = db.execute("SELECT * FROM rescue_reports WHERE report_id = ?", (report_id,)).fetchone()
            # 영수증은 commit 뒤에만 만든다(PERSISTED = 이 파일에 저장됨).
            return _stored(row)
        except OperationsError as exc:
            return _rejected(payload, _reason(exc))

    def list_reports(self, *, limit=100):
        require(type(limit) is int and 1 <= limit <= 1000, "limit must be 1..1000")
        rows = self._read("""SELECT report_id, target_id, meaning_sha256, record_ids_json, receipt_id,
            persisted_at_ns FROM rescue_reports ORDER BY received_at_ns, report_id LIMIT ?""", (limit,))
        return [dict(row) | {"record_ids": json.loads(row["record_ids_json"])} for row in rows]


def receipt_fields(receipt):
    """StoredReceipt -> ReportReceipt 필드(Envelope 제외). 거절은 저장 시각·기록 ID 를 싣지 않는다."""
    return {
        "receipt_id": receipt.receipt_id, "report_id": receipt.report_id,
        "meaning_sha256": receipt.meaning_sha256, "included_record_ids": list(receipt.included_record_ids),
        "state": receipt.state, "persisted_at_ns": receipt.persisted_at_ns or 0,
        "has_persisted_at": receipt.persisted_at_ns is not None, "storage_revision": receipt.storage_revision,
        "reason_code": receipt.reason_code,
    }
