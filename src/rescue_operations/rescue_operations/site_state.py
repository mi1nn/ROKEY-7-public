"""Operations-owned site state for the control screen (BRD Part I 3.7.1~3.7.4).

A separate SQLite file from the report ledger, so the fixed report schema 1.0
and its reopen policy are untouched. One writer (operations); readers open it
read-only and never create it. Rows are latest snapshots, except site events,
which are kept as an append-only history.

A robot snapshot is for display only; it is not live readiness evidence (C-05).
Poses in different frames must not be compared by distance. The exit count is
the number of distinct EXIT_PASS events stored here; recognition removes track
duplicates before it publishes, and nothing here removes them again (TS-04).
"""

from dataclasses import asdict
import json
import math
import sqlite3
import time

from rescue_contracts import CANONICAL_CODES, RESCUE_STATES, ROBOT_ROLES, SiteEvent

from .ledger import _Database, _limit, _path
from .schema import (OperationsError, allowed_robots, canonical, exact, require, text, timestamp,
                     uid, uint)

SCHEMA_VERSION = "1.1"
# A 1.0 file is upgraded when the writer opens it: missing tables are added, nothing is dropped.
UPGRADES_FROM = frozenset({"1.0"})
ROLE = "operations-site-state"
UNKNOWN = "UNKNOWN"
# Support status per target [제안, 관제·화면 2026-10-08]; screen-side vocabulary, not a contract code.
SUPPORT_STATES = frozenset({"NONE", "ASSIGNED", "ARRIVED", "FAILED"})
ROBOT_KEYS = {"robot_id", "robot_boot_id", "state_sequence", "role", "stamp_ns", "pose", "motion_state",
              "readiness", "battery_percent", "battery_stamp_ns", "active_task_id", "generation"}
POSE_KEYS = {"x", "y", "yaw", "frame_id"}
TARGET_KEYS = {"target_id", "position", "detection_confirmed", "rescue_state", "rescue_reasons", "verdict_id",
               "support_state", "support_robot_id", "updated_at_ns"}
TARGET_POSITION_KEYS = {"x", "y", "frame_id", "map_session_id", "map_revision"}
ERROR_KEYS = {"at_ns", "robot_id", "task_id", "code", "detail"}


# 1.0: robot_status, site_events, target_states. 1.1 adds the last four (db-schema 4절).
TABLES = (
    """CREATE TABLE IF NOT EXISTS robot_status (
        robot_id TEXT PRIMARY KEY, robot_boot_id TEXT NOT NULL, state_sequence INTEGER NOT NULL,
        stamp_ns INTEGER NOT NULL, status_json TEXT NOT NULL, received_at_ns INTEGER NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS site_events (
        event_id TEXT PRIMARY KEY, kind TEXT NOT NULL, observed_at_ns INTEGER NOT NULL,
        event_json TEXT NOT NULL, received_at_ns INTEGER NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS target_states (
        target_id TEXT PRIMARY KEY, updated_at_ns INTEGER NOT NULL, state_json TEXT NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS error_log (
        seq INTEGER PRIMARY KEY, at_ns INTEGER NOT NULL, robot_id TEXT, task_id TEXT,
        code TEXT NOT NULL, detail TEXT NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS mission_state (
        singleton INTEGER PRIMARY KEY CHECK (singleton = 1), mode TEXT NOT NULL, phase TEXT NOT NULL,
        trigger_event_id TEXT, started_at_ns INTEGER, generation INTEGER NOT NULL,
        updated_at_ns INTEGER NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS operator_commands (
        command_id TEXT PRIMARY KEY, kind TEXT NOT NULL, operator_id TEXT NOT NULL, robot_id TEXT,
        target_id TEXT, segment_index INTEGER, route_revision INTEGER, requested_at_ns INTEGER NOT NULL,
        accepted INTEGER NOT NULL, result TEXT, reason TEXT)""",
    """CREATE TABLE IF NOT EXISTS map_snapshots (
        snapshot_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, revision INTEGER NOT NULL,
        path TEXT NOT NULL, sha256 TEXT NOT NULL, at_ns INTEGER NOT NULL)""",
)


def _finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), label + " must be finite")
    return float(value)


def _member(value, allowed, label):
    require(isinstance(value, str) and value in allowed, label + " is outside the configured deployment", "WRONG_ROBOT")
    return value


def _pair(first, second, label):
    require((first is None) == (second is None), label + " must be both present or both None")


def _robot_status(value, allowed):
    status = dict(exact(value, ROBOT_KEYS, "robot status"))
    _member(status["robot_id"], allowed, "robot_id")
    uid(status["robot_boot_id"], "robot_boot_id")
    uint(status["state_sequence"], "state_sequence")
    require(status["role"] == UNKNOWN or status["role"] in ROBOT_ROLES, "role must be SCOUT, SUPPORT or UNKNOWN")
    timestamp(status["stamp_ns"], "stamp_ns")
    if status["pose"] is not None:
        pose = exact(status["pose"], POSE_KEYS, "pose")
        status["pose"] = {"x": _finite(pose["x"], "pose.x"), "y": _finite(pose["y"], "pose.y"),
                          "yaw": _finite(pose["yaw"], "pose.yaw"), "frame_id": text(pose["frame_id"], "pose.frame_id")}
    text(status["motion_state"], "motion_state", 64)
    text(status["readiness"], "readiness", 64)
    _pair(status["battery_percent"], status["battery_stamp_ns"], "battery_percent/battery_stamp_ns")
    if status["battery_percent"] is not None:
        status["battery_percent"] = _finite(status["battery_percent"], "battery_percent")
        require(0.0 <= status["battery_percent"] <= 100.0, "battery_percent must be 0..100")
        timestamp(status["battery_stamp_ns"], "battery_stamp_ns")
    _pair(status["active_task_id"], status["generation"], "active_task_id/generation")
    if status["active_task_id"] is not None:
        uid(status["active_task_id"], "active_task_id")
        uint(status["generation"], "generation")
    return status


def _target_state(value, allowed):
    state = dict(exact(value, TARGET_KEYS, "target state"))
    uid(state["target_id"], "target_id")
    if state["position"] is not None:
        position = exact(state["position"], TARGET_POSITION_KEYS, "position")
        state["position"] = {"x": _finite(position["x"], "position.x"), "y": _finite(position["y"], "position.y"),
                             "frame_id": text(position["frame_id"], "position.frame_id"),
                             "map_session_id": uid(position["map_session_id"], "position.map_session_id"),
                             "map_revision": uint(position["map_revision"], "position.map_revision")}
    require(type(state["detection_confirmed"]) is bool, "detection_confirmed must be boolean")
    require(state["rescue_state"] in RESCUE_STATES, "rescue_state must be one of RESCUE_STATES")
    reasons = state["rescue_reasons"]
    # The closed reason set belongs to the coordination policy; here only its shape is checked.
    require(isinstance(reasons, list) and all(isinstance(r, str) and r.isidentifier() for r in reasons)
            and len(set(reasons)) == len(reasons), "rescue_reasons must be unique identifiers")
    require(bool(reasons) == (state["rescue_state"] == "RECHECK_NEEDED"),
            "rescue_reasons are present exactly when RECHECK_NEEDED")
    state["rescue_reasons"] = sorted(reasons)
    if state["verdict_id"] is not None:
        uid(state["verdict_id"], "verdict_id")
    require(state["support_state"] in SUPPORT_STATES, "support_state must be NONE, ASSIGNED, ARRIVED or FAILED")
    require((state["support_robot_id"] is None) == (state["support_state"] == "NONE"),
            "support_robot_id is None exactly when support_state is NONE")
    if state["support_robot_id"] is not None:
        _member(state["support_robot_id"], allowed, "support_robot_id")
    timestamp(state["updated_at_ns"], "updated_at_ns")
    return state


def _error_entry(value, allowed):
    entry = exact(value, ERROR_KEYS, "error entry")
    timestamp(entry["at_ns"], "at_ns")
    if entry["robot_id"] is not None:
        _member(entry["robot_id"], allowed, "robot_id")
    if entry["task_id"] is not None:
        uid(entry["task_id"], "task_id")
    # C-09: consumers branch on the common code; the qualified cause belongs in detail.
    require(entry["code"] in CANONICAL_CODES, "code must be a C-09 common code (rescue_contracts.CANONICAL_CODES)")
    text(entry["detail"], "detail", 1024)
    return entry


class SiteStateLedger(_Database):
    """Operations-owned writer for the screen's site state; never share this file between PCs."""

    def __init__(self, db_path, *, allowed_robot_ids):
        self._allowed_robot_ids = allowed_robots(allowed_robot_ids)
        super().__init__(db_path, ROLE, {"allowed_robot_ids": canonical(self.allowed_robot_ids)},
                         schema=SCHEMA_VERSION, upgrades_from=UPGRADES_FROM, tables=TABLES)

    def record_robot_status(self, status):
        """Store the latest snapshot. Returns False for a stale or identical resend."""
        status = _robot_status(status, self.allowed_robot_ids)
        encoded = canonical(status)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM robot_status WHERE robot_id = ?", (status["robot_id"],)).fetchone()
            if row is not None:
                if row["robot_boot_id"] == status["robot_boot_id"]:
                    if status["state_sequence"] == row["state_sequence"]:
                        require(row["status_json"] == encoded, "same boot and sequence with different content",
                                "ID_CONTENT_CONFLICT")
                        return False
                    if status["state_sequence"] < row["state_sequence"]:
                        return False
                elif status["stamp_ns"] <= row["stamp_ns"]:
                    return False  # a late message from another boot must not replace a newer snapshot
            db.execute("INSERT OR REPLACE INTO robot_status VALUES (?, ?, ?, ?, ?, ?)",
                       (status["robot_id"], status["robot_boot_id"], status["state_sequence"], status["stamp_ns"],
                        encoded, timestamp(time.time_ns(), "received_at_ns")))
        return True

    def record_site_event(self, event):
        """Append one site-camera event. Returns False for an identical resend."""
        require(isinstance(event, SiteEvent), "SiteEvent required")
        encoded = canonical(asdict(event))
        with self._transaction() as db:
            row = db.execute("SELECT event_json FROM site_events WHERE event_id = ?", (event.event_id,)).fetchone()
            if row is not None:
                require(row["event_json"] == encoded, "event_id has different content", "ID_CONTENT_CONFLICT")
                return False
            db.execute("INSERT INTO site_events VALUES (?, ?, ?, ?, ?)",
                       (event.event_id, event.kind, event.observed_at_ns, encoded,
                        timestamp(time.time_ns(), "received_at_ns")))
        return True

    def record_target_state(self, state):
        """Store the latest state per target. Only a newer ``updated_at_ns`` replaces it."""
        state = _target_state(state, self.allowed_robot_ids)
        encoded = canonical(state)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM target_states WHERE target_id = ?", (state["target_id"],)).fetchone()
            if row is not None:
                if state["updated_at_ns"] == row["updated_at_ns"]:
                    require(row["state_json"] == encoded, "same target and time with different content",
                            "ID_CONTENT_CONFLICT")
                    return False
                if state["updated_at_ns"] < row["updated_at_ns"]:
                    return False
            db.execute("INSERT OR REPLACE INTO target_states VALUES (?, ?, ?)",
                       (state["target_id"], state["updated_at_ns"], encoded))
        return True


    def record_error(self, entry):
        """Append one failure (BRD 3.7.4). Every call is a new row; returns its ``seq``."""
        entry = _error_entry(entry, self.allowed_robot_ids)
        with self._transaction() as db:
            cursor = db.execute("INSERT INTO error_log (at_ns, robot_id, task_id, code, detail) VALUES (?, ?, ?, ?, ?)",
                                (entry["at_ns"], entry["robot_id"], entry["task_id"], entry["code"], entry["detail"]))
        return cursor.lastrowid


def _read(db_path, query):
    """Open read-only, check role/schema/policy, run ``query(db)``; never create the file."""
    path = _path(db_path)
    db = None
    try:
        db = sqlite3.connect(path.as_uri() + "?mode=ro", uri=True, timeout=1.0)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only = ON")
        metadata = dict(db.execute("SELECT key, value FROM metadata"))
        require(metadata.get("role") == ROLE and metadata.get("schema") == SCHEMA_VERSION,
                "not an operations site-state schema " + SCHEMA_VERSION + " database", "POLICY_MISMATCH")
        try:
            allowed_robots(json.loads(metadata["allowed_robot_ids"]))
        except (KeyError, ValueError, TypeError) as exc:
            raise OperationsError("POLICY_MISMATCH", "DB has no valid robot policy") from exc
        return {"schema_version": SCHEMA_VERSION, **query(db)}
    except sqlite3.Error as exc:
        raise OperationsError("STORAGE_ERROR", "cannot read site-state database") from exc
    finally:
        if db is not None:
            db.close()


def read_robot_status(db_path):
    def query(db):
        rows = db.execute("SELECT status_json, received_at_ns FROM robot_status ORDER BY robot_id").fetchall()
        return {"robots": [json.loads(row["status_json"]) | {"received_at_ns": row["received_at_ns"]}
                           for row in rows]}
    return _read(db_path, query)


def read_site_events(db_path, *, limit=100):
    limit = _limit(limit)

    def query(db):
        count = db.execute("SELECT COUNT(DISTINCT event_id) FROM site_events WHERE kind = 'EXIT_PASS'").fetchone()[0]
        rows = db.execute("SELECT event_json FROM site_events ORDER BY observed_at_ns, event_id LIMIT ?",
                          (limit,)).fetchall()
        return {"exit_pass_count": count, "site_events": [json.loads(row["event_json"]) for row in rows]}
    return _read(db_path, query)


def read_target_states(db_path):
    def query(db):
        rows = db.execute("SELECT state_json FROM target_states ORDER BY target_id").fetchall()
        return {"targets": [json.loads(row["state_json"]) for row in rows]}
    return _read(db_path, query)


def read_error_log(db_path, *, limit=100):
    """Newest first; ``limit`` keeps the latest N entries."""
    limit = _limit(limit)

    def query(db):
        rows = db.execute("SELECT seq, at_ns, robot_id, task_id, code, detail FROM error_log "
                          "ORDER BY at_ns DESC, seq DESC LIMIT ?", (limit,)).fetchall()
        return {"errors": [dict(row) for row in rows]}
    return _read(db_path, query)
