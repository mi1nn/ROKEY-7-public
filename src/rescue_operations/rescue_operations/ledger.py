"""Durable local report/receipt and sender outbox boundaries (schema 1.0)."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import sqlite3
import threading
import time
from typing import Protocol
import uuid

from .schema import (EVIDENCE_POLICIES, MAX_EVIDENCE_BYTES, OperationsError,
                     allowed_robots, canonical, envelope, evidence_ref, identity, report,
                     require, timestamp, uid, uint)


@dataclass(frozen=True)
class Receipt:
    receipt_id: str
    report_id: str
    payload_sha256: str
    status: str
    revision: int
    persisted_at_ns: int | None
    error_code: str | None

    def to_dict(self):
        return asdict(self)


class ReportTransport(Protocol):
    """Adapters must bound their own I/O duration and return the verified receipt."""

    def __call__(self, payload: dict, transport_envelope: dict) -> Receipt: ...


def _path(value):
    try:
        require(str(value) not in ("", ":memory:") and "\x00" not in str(value), "a durable local DB path is required")
        path = Path(value).resolve()
        require(not path.is_dir() and path.parent.is_dir(), "DB parent must exist and DB path must be a file")
        return path
    except OperationsError:
        raise
    except (TypeError, ValueError, OSError, RuntimeError) as exc:
        raise OperationsError("INVALID_INPUT", "invalid DB path") from exc


def _receipt(row):
    return Receipt(row["receipt_id"], row["report_id"], row["payload_sha256"],
                   row["state"], row["revision"], row["persisted_at_ns"], row["error_code"])


def _limit(limit):
    require(type(limit) is int and 1 <= limit <= 1000, "limit must be 1..1000")
    return limit


class _Database:
    @property
    def allowed_robot_ids(self):
        return self._allowed_robot_ids

    def __init__(self, db_path, role, metadata=None):
        self.path = _path(db_path)
        self._lock = threading.RLock()
        self._db = None
        try:
            self._db = sqlite3.connect(self.path, timeout=1.0, isolation_level=None, check_same_thread=False)
            self._db.row_factory = sqlite3.Row
            self._db.execute("PRAGMA busy_timeout = 1000")
            self._db.execute("PRAGMA journal_mode = WAL")
            self._db.execute("PRAGMA synchronous = FULL")
            with self._transaction() as db:
                db.execute("CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
                wanted = {"schema": "1.0", "role": role, **(metadata or {})}
                current = dict(db.execute("SELECT key, value FROM metadata"))
                if current:
                    require(current == wanted, "DB role/schema/policy cannot change on reopen", "POLICY_MISMATCH")
                else:
                    db.executemany("INSERT INTO metadata VALUES (?, ?)", wanted.items())
        except (sqlite3.Error, OperationsError) as exc:
            self.close()
            if isinstance(exc, OperationsError):
                raise
            raise OperationsError("STORAGE_ERROR", "cannot open report database") from exc

    @contextmanager
    def _transaction(self):
        with self._lock:
            require(self._db is not None, "database is closed", "STORAGE_ERROR")
            try:
                self._db.execute("BEGIN IMMEDIATE")
                yield self._db
                self._db.commit()
            except BaseException as exc:
                self._db.rollback()
                if isinstance(exc, sqlite3.Error):
                    raise OperationsError("STORAGE_ERROR", "database transaction failed") from exc
                raise

    def _read(self, sql, args=()):
        with self._lock:
            require(self._db is not None, "database is closed", "STORAGE_ERROR")
            try:
                return self._db.execute(sql, args).fetchall()
            except sqlite3.Error as exc:
                raise OperationsError("STORAGE_ERROR", "database read failed") from exc

    def close(self):
        with self._lock:
            if self._db is not None:
                self._db.close()
                self._db = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class ReportLedger(_Database):
    """Operations-owned SQLite writer; never share this DB file between PCs."""

    def __init__(self, db_path, *, evidence_policy, allowed_robot_ids):
        require(isinstance(evidence_policy, str) and evidence_policy in EVIDENCE_POLICIES,
                "choose OPTIONAL, OBSERVATION or CONFIRMATION evidence policy")
        self.evidence_policy = evidence_policy
        self._allowed_robot_ids = allowed_robots(allowed_robot_ids)
        super().__init__(db_path, "operations-ledger", {"evidence_policy": evidence_policy,
                         "allowed_robot_ids": canonical(self.allowed_robot_ids)})
        try:
            with self._transaction() as db:
                db.execute("""CREATE TABLE IF NOT EXISTS reports (
                    report_id TEXT PRIMARY KEY, payload_sha256 TEXT NOT NULL,
                    payload_json TEXT NOT NULL, envelope_json TEXT NOT NULL,
                    state TEXT NOT NULL, revision INTEGER NOT NULL, receipt_id TEXT NOT NULL,
                    received_at_ns INTEGER NOT NULL, persisted_at_ns INTEGER, error_code TEXT)""")
                db.execute("""CREATE TABLE IF NOT EXISTS evidence (
                    evidence_id TEXT PRIMARY KEY, ref_json TEXT NOT NULL,
                    content BLOB NOT NULL, stored_at_ns INTEGER NOT NULL)""")
        except BaseException:
            self.close()
            raise

    def receive(self, payload, transport_envelope):
        payload, transport_envelope = report(payload), envelope(transport_envelope)
        _report_membership(payload, self.allowed_robot_ids)
        digest, report_id = identity(payload), payload["report_id"]
        with self._transaction() as db:
            row = db.execute("SELECT * FROM reports WHERE report_id = ?", (report_id,)).fetchone()
            if row is not None:
                require(row["payload_sha256"] == digest, "report_id has different meaning payload", "ID_CONTENT_CONFLICT")
                return _receipt(row)
            db.execute("INSERT INTO reports VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                       (report_id, digest, canonical(payload), canonical(transport_envelope),
                        "RECEIVED", 1, str(uuid.uuid4()), timestamp(time.time_ns(), "received_at_ns"), None, None))
            result = _receipt(db.execute("SELECT * FROM reports WHERE report_id = ?", (report_id,)).fetchone())
        # Receipt is returned only after commit; RECEIVED is still not report completion.
        return result

    def store_evidence(self, ref, content):
        ref = evidence_ref(ref)
        require(type(content) is bytes and 0 < len(content) <= MAX_EVIDENCE_BYTES,
                "evidence must be bytes of size 1..4194304")
        require(hashlib.sha256(content).hexdigest() == ref["sha256"], "evidence bytes differ from SHA-256", "EVIDENCE_HASH_MISMATCH")
        # This is byte integrity only, not proof of camera source, image decoding or a person.
        ref_json = canonical(ref)
        with self._transaction() as db:
            existing = db.execute("SELECT ref_json FROM evidence WHERE evidence_id = ?", (ref["evidence_id"],)).fetchone()
            if existing is not None:
                require(existing["ref_json"] == ref_json, "evidence_id has different metadata", "ID_CONTENT_CONFLICT")
            else:
                db.execute("INSERT INTO evidence VALUES (?, ?, ?, ?)",
                           (ref["evidence_id"], ref_json, content, timestamp(time.time_ns(), "stored_at_ns")))
        return {"evidence_id": ref["evidence_id"], "sha256": ref["sha256"], "bytes_stored": len(content)}

    def persist(self, report_id):
        uid(report_id, "report_id")
        with self._transaction() as db:
            row = db.execute("SELECT * FROM reports WHERE report_id = ?", (report_id,)).fetchone()
            require(row is not None, "unknown report_id", "NOT_FOUND")
            if row["state"] == "PERSISTED":
                return _receipt(row)
            payload = json.loads(row["payload_json"])
            refs = payload["evidence_refs"]
            missing = False
            for ref in refs:
                saved = db.execute("SELECT ref_json FROM evidence WHERE evidence_id = ?", (ref["evidence_id"],)).fetchone()
                if saved is None:
                    missing = True
                else:
                    require(saved["ref_json"] == canonical(ref), "registered evidence differs from report reference", "ID_CONTENT_CONFLICT")
            if self.evidence_policy != "OPTIONAL":
                key = "observation_id" if self.evidence_policy == "OBSERVATION" else "confirmation_observation_id"
                if payload[key] is None or not any(ref["observation_id"] == payload[key] for ref in refs):
                    missing = True
            state = "PENDING_EVIDENCE" if missing else "PERSISTED"
            if row["state"] != state:
                db.execute("""UPDATE reports SET state = ?, revision = revision + 1,
                    receipt_id = ?, persisted_at_ns = ?, error_code = ? WHERE report_id = ?""",
                           (state, str(uuid.uuid4()), None if missing else timestamp(time.time_ns(), "persisted_at_ns"),
                            "PENDING_EVIDENCE" if missing else None, report_id))
            result = _receipt(db.execute("SELECT * FROM reports WHERE report_id = ?", (report_id,)).fetchone())
        return result

    def get(self, report_id):
        uid(report_id, "report_id")
        rows = self._read("SELECT * FROM reports WHERE report_id = ?", (report_id,))
        require(bool(rows), "unknown report_id", "NOT_FOUND")
        return _public_report(rows[0])

    def list_reports(self, *, limit=100):
        rows = self._read("SELECT * FROM reports ORDER BY received_at_ns, report_id LIMIT ?", (_limit(limit),))
        return [_public_report(row) for row in rows]


class LedgerTransport:
    """In-process API adapter; remote transports must preserve the same boundary."""

    def __init__(self, ledger: ReportLedger):
        self.ledger = ledger

    def __call__(self, payload: dict, transport_envelope: dict) -> Receipt:
        received = self.ledger.receive(payload, transport_envelope)
        return self.ledger.persist(received.report_id)


class ReportOutbox(_Database):
    """Sender-owned DB. Each delivery is one bounded attempt, never an implicit loop."""

    def __init__(self, db_path, *, allowed_robot_ids):
        self._allowed_robot_ids = allowed_robots(allowed_robot_ids)
        super().__init__(db_path, "report-outbox", {
            "allowed_robot_ids": canonical(self.allowed_robot_ids)})
        try:
            with self._transaction() as db:
                db.execute("""CREATE TABLE IF NOT EXISTS outbox (
                    report_id TEXT PRIMARY KEY, payload_sha256 TEXT NOT NULL,
                    payload_json TEXT NOT NULL, state TEXT NOT NULL, attempts INTEGER NOT NULL,
                    receipt_json TEXT, last_error TEXT)""")
        except BaseException:
            self.close()
            raise

    def enqueue(self, payload):
        payload = report(payload)
        _report_membership(payload, self.allowed_robot_ids)
        digest, report_id = identity(payload), payload["report_id"]
        with self._transaction() as db:
            row = db.execute("SELECT payload_sha256 FROM outbox WHERE report_id = ?", (report_id,)).fetchone()
            if row is not None:
                require(row["payload_sha256"] == digest, "queued report_id has different meaning", "ID_CONTENT_CONFLICT")
            else:
                db.execute("INSERT INTO outbox VALUES (?, ?, ?, 'QUEUED', 0, NULL, NULL)",
                           (report_id, digest, canonical(payload)))
        return self.get(report_id)

    def get(self, report_id):
        uid(report_id, "report_id")
        rows = self._read("SELECT * FROM outbox WHERE report_id = ?", (report_id,))
        require(bool(rows), "unknown report_id", "NOT_FOUND")
        row = dict(rows[0])
        row["payload"] = json.loads(row.pop("payload_json"))
        receipt_json = row.pop("receipt_json")
        row["receipt"] = json.loads(receipt_json) if receipt_json is not None else None
        return row

    def pending(self, *, limit=100):
        return [row["report_id"] for row in self._read(
            "SELECT report_id FROM outbox WHERE state != 'PERSISTED' ORDER BY report_id LIMIT ?", (_limit(limit),))]

    def deliver(self, report_id, transport_envelope, transport: ReportTransport):
        """Return receipt or None on adapter failure; only verified PERSISTED ends retries."""
        uid(report_id, "report_id")
        transport_envelope = envelope(transport_envelope)
        row = self.get(report_id)
        if row["state"] == "PERSISTED":
            return Receipt(**row["receipt"])
        # Attempt recorded before I/O; process death after remote commit remains retryable.
        with self._transaction() as db:
            db.execute("UPDATE outbox SET attempts = attempts + 1 WHERE report_id = ?", (report_id,))
        received = None
        error = None
        try:
            received = transport(row["payload"], transport_envelope)
            _validate_receipt(received, report_id, row["payload_sha256"])
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"[:1024]
        with self._transaction() as db:
            latest = db.execute("SELECT * FROM outbox WHERE report_id = ?", (report_id,)).fetchone()
            # A late response/failure must not regress a concurrently accepted receipt.
            previous = json.loads(latest["receipt_json"]) if latest["receipt_json"] else None
            if latest["state"] == "PERSISTED":
                return Receipt(**previous)
            if error is not None:
                db.execute("UPDATE outbox SET state = 'RETRY_PENDING', last_error = ? WHERE report_id = ?", (error, report_id))
                return None
            if previous and received.revision < previous["revision"]:
                return Receipt(**previous)
            if previous and received.revision == previous["revision"]:
                require(received.to_dict() == previous, "same receipt revision changed", "ID_CONTENT_CONFLICT")
            db.execute("UPDATE outbox SET state = ?, receipt_json = ?, last_error = NULL WHERE report_id = ?",
                       (received.status, canonical(received.to_dict()), report_id))
        return received


def _report_membership(payload, allowed_robot_ids):
    for key in ("source_robot", "robot_id"):
        require(payload[key] in allowed_robot_ids, key + " is outside the configured deployment", "WRONG_ROBOT")


def _validate_receipt(receipt, report_id, digest):
    require(isinstance(receipt, Receipt), "transport must return Receipt")
    uid(receipt.receipt_id, "receipt_id")
    require(receipt.report_id == report_id and receipt.payload_sha256 == digest,
            "receipt report identity differs", "ID_CONTENT_CONFLICT")
    uint(receipt.revision, "receipt revision", positive=True)
    require(receipt.status in ("RECEIVED", "PENDING_EVIDENCE", "PERSISTED"), "invalid receipt state")
    if receipt.status == "PERSISTED":
        timestamp(receipt.persisted_at_ns, "persisted_at_ns")
        require(receipt.error_code is None, "persisted receipt cannot contain an error")
    else:
        require(receipt.persisted_at_ns is None, "non-persisted receipt cannot have persisted_at_ns")
        require(receipt.error_code == ("PENDING_EVIDENCE" if receipt.status == "PENDING_EVIDENCE" else None),
                "receipt error code disagrees with state")


def _public_report(row):
    return {"payload": json.loads(row["payload_json"]), "receipt": _receipt(row).to_dict(),
            "first_envelope": json.loads(row["envelope_json"]), "received_at_ns": row["received_at_ns"]}


def read_reports(db_path, *, limit=100, report_id=None):
    """Read-only snapshot for a local CLI/API; does not create a missing database."""
    path = _path(db_path)
    limit = _limit(limit)
    if report_id is not None:
        uid(report_id, "report_id")
    db = None
    try:
        db = sqlite3.connect(path.as_uri() + "?mode=ro", uri=True, timeout=1.0)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only = ON")
        metadata = dict(db.execute("SELECT key, value FROM metadata"))
        require(metadata.get("role") == "operations-ledger" and metadata.get("schema") == "1.0",
                "not an operations schema 1.0 database", "POLICY_MISMATCH")
        try:
            configured_robots = allowed_robots(json.loads(metadata["allowed_robot_ids"]))
        except (KeyError, ValueError, TypeError) as exc:
            raise OperationsError("POLICY_MISMATCH", "DB has no valid immutable robot policy") from exc
        if report_id is None:
            rows = db.execute("SELECT * FROM reports ORDER BY received_at_ns, report_id LIMIT ?", (limit,)).fetchall()
        else:
            rows = db.execute("SELECT * FROM reports WHERE report_id = ?", (report_id,)).fetchall()
            require(bool(rows), "unknown report_id", "NOT_FOUND")
        return {"schema_version": "1.0", "evidence_policy": metadata["evidence_policy"],
                "allowed_robot_ids": list(configured_robots),
                "reports": [_public_report(row) for row in rows]}
    except sqlite3.Error as exc:
        raise OperationsError("STORAGE_ERROR", "cannot read report database") from exc
    finally:
        if db is not None:
            db.close()
