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

from collections.abc import Mapping
from dataclasses import MISSING, asdict, fields
import json
import math
import sqlite3
import time

from rescue_contracts import CANONICAL_CODES, RESCUE_STATES, ROBOT_ROLES, SYSTEM_MODES, SiteEvent

from .ledger import _Database, _limit, _path
from .schema import (OperationsError, allowed_robots, canonical, exact, require, sha, text, timestamp,
                     uid, uint)

SCHEMA_VERSION = "1.3"
# A 1.0 file is upgraded when the writer opens it: missing tables are added, nothing is dropped.
UPGRADES_FROM = frozenset({"1.0", "1.1", "1.2"})  # 1.1 -> 1.2 rebuilds mission_state; 1.3 adds exit_locations
ROLE = "operations-site-state"
UNKNOWN = "UNKNOWN"
# Support status per target [제안, 관제·화면 2026-10-08]; screen-side vocabulary, not a contract code.
SUPPORT_STATES = frozenset({"NONE", "ASSIGNED", "ARRIVED", "FAILED"})
# Mission badge and flow shown on the screen [제안, console-and-db-plan 1절·3절 2026-10-10]. Display
# vocabulary written by operations from coordinator results; the coordinator owns the decisions.
# Mode is the contract system mode (S-00: NORMAL, DISASTER, CLEARING) plus SystemState's MISSION_FINISHED,
# which the coordinator sends exactly when the phase is FINISHED (C-10, contracts 207행, #88).
FINISHED_MODE = "MISSION_FINISHED"
MISSION_MODES = SYSTEM_MODES | {FINISHED_MODE}
# SystemState.SEGMENT_PHASE_* (1차 IDL, #88); the coordinator maps its own words at the node (C-10).
SEGMENT_PHASES = frozenset({"IDLE", "EVALUATING", "AWAITING_APPROVAL", "EXECUTING", "STOPPING", "STOPPED",
                            "UNKNOWN"})
MISSION_PHASES = frozenset({"IDLE", "DISASTER", "SEARCHING", "TARGET_FOUND", "DISPATCH_APPROVED", "SEGMENT_MOVING",
                            "AWAITING_CONTINUE", "ARRIVED", "CANDIDATE", "RECHECKING", "RESCUE_CONFIRMED",
                            "RECHECK_NEEDED", "RETURNING", "FINISHED"})
# Commands the console may send. Every operation of the 1차 IDL service SubmitOperatorCommand (#77·#98:
# START_SEARCH … SET_CAMERA_WATCH) plus the console plan's other buttons (PAUSE·RESUME·CANCEL·STOP_ROBOT,
# console-and-db-plan 4절 [제안], STOP goes through RequestStop). The gate that accepts or rejects them
# is elsewhere; this table only keeps what was asked and what was answered.
OPERATOR_COMMAND_KINDS = frozenset({"START_SEARCH", "DISPATCH_SUPPORT", "CONTINUE_SEGMENT", "REQUEST_RECHECK",
                                    "RETURN_HOME", "FINISH_MISSION", "RESET_STOP", "SET_CAMERA_WATCH",
                                    "PAUSE", "RESUME", "CANCEL", "STOP_ROBOT"})
ROBOT_KEYS = {"robot_id", "robot_boot_id", "state_sequence", "role", "stamp_ns", "pose", "motion_state",
              "readiness", "battery_percent", "battery_stamp_ns", "active_task_id", "generation"}
# Optional robot keys from 1차 IDL RobotStatus (2026-10-11); a row or writer without them reads as None.
ROBOT_DEFAULTS = {"dock_state": None, "segment_index": None}
DOCK_STATES = frozenset({"DOCKED", "UNDOCKED", "UNKNOWN"})  # RobotStatus.DOCK_STATE_*
POSE_KEYS = {"x", "y", "yaw", "frame_id"}
TARGET_KEYS = {"target_id", "position", "detection_confirmed", "rescue_state", "rescue_reasons", "verdict_id",
               "support_state", "support_robot_id", "updated_at_ns"}
# Optional target key (#115): the estimate revision the console shows and sends back with DISPATCH_SUPPORT.
# A row stored before it, or a writer without it, reads as None.
# persisted_report_id (#118): the report of this revision the coordinator saw PERSISTED, else None;
# the console enables AMR2 dispatch only with it (the coordinator also requires it).
TARGET_DEFAULTS = {"target_revision": None, "persisted_report_id": None}
TARGET_POSITION_KEYS = {"x", "y", "frame_id", "map_session_id", "map_revision"}
ERROR_KEYS = {"at_ns", "robot_id", "task_id", "code", "detail"}
COMMAND_KEYS = {"command_id", "kind", "operator_id", "robot_id", "target_id", "segment_index", "route_revision",
                "requested_at_ns", "accepted", "result", "reason"}
MAP_KEYS = {"snapshot_id", "robot_id", "revision", "path", "sha256", "at_ns"}
# 1.3: one VALID QR exit observation (/rescue/exit_location, class_label exit_qr; 작전 OPS-144·150).
EXIT_CLASS_LABEL = "exit_qr"
EXIT_KEYS = {"location_id", "observed_at_ns", "received_at_ns", "map_session_id", "map_revision",
             "transform_revision", "mission_frame", "x", "y", "class_label"}
MISSION_KEYS = {"mode", "phase", "trigger_event_id", "started_at_ns", "generation", "updated_at_ns"}
# The AMR2 dispatch (schema 1.2): all None without a dispatch, all present with one.
DISPATCH_DEFAULTS = {"target_id": None, "target_revision": None, "route_revision": None, "segment_index": None,
                     "segment_count": None, "segment_phase": None}
# SystemState.reason: the coordinator's cause code when it holds the mission (#91 CLA-02), else None.
OPTIONAL_MISSION = DISPATCH_DEFAULTS | {"reason": None}
MISSION_COLUMNS = ("mode", "phase", "trigger_event_id", "started_at_ns", "generation", "updated_at_ns",
                   *OPTIONAL_MISSION)


MISSION_DDL = """CREATE TABLE IF NOT EXISTS mission_state (
        singleton INTEGER PRIMARY KEY CHECK (singleton = 1), mode TEXT NOT NULL, phase TEXT NOT NULL,
        trigger_event_id TEXT, started_at_ns INTEGER, generation INTEGER, updated_at_ns INTEGER NOT NULL,
        target_id TEXT, target_revision INTEGER, route_revision INTEGER, segment_index INTEGER,
        segment_count INTEGER, segment_phase TEXT, reason TEXT)"""
# 1.0: robot_status, site_events, target_states. 1.1 adds four (db-schema 4절); 1.2 reshapes mission_state;
# 1.3 adds exit_locations (a 1.2 file only gets the new table).
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
    MISSION_DDL,
    """CREATE TABLE IF NOT EXISTS operator_commands (
        command_id TEXT PRIMARY KEY, kind TEXT NOT NULL, operator_id TEXT NOT NULL, robot_id TEXT,
        target_id TEXT, segment_index INTEGER, route_revision INTEGER, requested_at_ns INTEGER NOT NULL,
        accepted INTEGER NOT NULL, result TEXT, reason TEXT)""",
    """CREATE TABLE IF NOT EXISTS map_snapshots (
        snapshot_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, revision INTEGER NOT NULL,
        path TEXT NOT NULL, sha256 TEXT NOT NULL, at_ns INTEGER NOT NULL)""",
    """CREATE TABLE IF NOT EXISTS exit_locations (
        location_id TEXT PRIMARY KEY, observed_at_ns INTEGER NOT NULL, received_at_ns INTEGER NOT NULL,
        map_session_id TEXT NOT NULL, map_revision INTEGER NOT NULL, transform_revision INTEGER NOT NULL,
        mission_frame TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL, class_label TEXT NOT NULL)""",
)


def _finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), label + " must be finite")
    return float(value)


def _member(value, allowed, label):
    require(isinstance(value, str) and value in allowed, label + " is outside the configured deployment", "WRONG_ROBOT")
    return value


def _pair(first, second, label):
    require((first is None) == (second is None), label + " must be both present or both None")


def _pose(value):
    if value is None:
        return None
    pose = exact(value, POSE_KEYS, "pose")
    return {"x": _finite(pose["x"], "pose.x"), "y": _finite(pose["y"], "pose.y"),
            "yaw": _finite(pose["yaw"], "pose.yaw"), "frame_id": text(pose["frame_id"], "pose.frame_id")}


def _robot_status(value, allowed):
    require(isinstance(value, Mapping), "robot status must be an object")
    status = dict(exact(ROBOT_DEFAULTS | dict(value), ROBOT_KEYS | ROBOT_DEFAULTS.keys(), "robot status"))
    _member(status["robot_id"], allowed, "robot_id")
    uid(status["robot_boot_id"], "robot_boot_id")
    uint(status["state_sequence"], "state_sequence")
    require(status["role"] == UNKNOWN or status["role"] in ROBOT_ROLES, "role must be SCOUT, SUPPORT or UNKNOWN")
    timestamp(status["stamp_ns"], "stamp_ns")
    status["pose"] = _pose(status["pose"])
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
    require(status["dock_state"] is None or status["dock_state"] in DOCK_STATES,
            "dock_state must be DOCKED, UNDOCKED, UNKNOWN or None")
    if status["segment_index"] is not None:
        uint(status["segment_index"], "segment_index")
    return status


def _stored_status(status_json):
    """A stored robot snapshot with every current key; keys added later read as None."""
    return ROBOT_DEFAULTS | json.loads(status_json)


# SiteEvent fields added after rows were stored (target_id/watch_id, #48) read back as their defaults.
EVENT_DEFAULTS = {f.name: f.default for f in fields(SiteEvent) if f.default is not MISSING}


def _stored_event(event_json):
    """A stored event with every current SiteEvent field; keys added later take their defaults."""
    return EVENT_DEFAULTS | json.loads(event_json)


def _target_state(value, allowed):
    require(isinstance(value, Mapping), "target state must be an object")
    state = dict(exact(TARGET_DEFAULTS | dict(value), TARGET_KEYS | TARGET_DEFAULTS.keys(), "target state"))
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
    # TargetEstimate (#96) has no reason field, so a RECHECK_NEEDED target may come without one.
    require(not reasons or state["rescue_state"] == "RECHECK_NEEDED", "rescue_reasons are only for RECHECK_NEEDED")
    state["rescue_reasons"] = sorted(reasons)
    if state["verdict_id"] is not None:
        uid(state["verdict_id"], "verdict_id")
    require(state["support_state"] in SUPPORT_STATES, "support_state must be NONE, ASSIGNED, ARRIVED or FAILED")
    require((state["support_robot_id"] is None) == (state["support_state"] == "NONE"),
            "support_robot_id is None exactly when support_state is NONE")
    if state["support_robot_id"] is not None:
        _member(state["support_robot_id"], allowed, "support_robot_id")
    timestamp(state["updated_at_ns"], "updated_at_ns")
    if state["target_revision"] is not None:
        uint(state["target_revision"], "target_revision")
    if state["persisted_report_id"] is not None:
        text(state["persisted_report_id"], "persisted_report_id")
    return state


def _stored_target(state_json):
    """A stored target state with every current key; keys added later read as None."""
    return TARGET_DEFAULTS | json.loads(state_json)


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


def _migrate(db, old_schema):
    """1.1 -> 1.2: mission_state gets the dispatch columns and a nullable generation (SQLite cannot relax
    NOT NULL in place, so the one-row table is rebuilt). A 1.0 file has no mission_state yet."""
    if old_schema == "1.1":
        db.execute("ALTER TABLE mission_state RENAME TO mission_state_1_1")
        db.execute(MISSION_DDL)
        db.execute("""INSERT INTO mission_state (singleton, mode, phase, trigger_event_id, started_at_ns, generation,
            updated_at_ns) SELECT singleton, mode, phase, trigger_event_id, started_at_ns, generation, updated_at_ns
            FROM mission_state_1_1""")
        db.execute("DROP TABLE mission_state_1_1")


def _mission(value):
    require(isinstance(value, Mapping), "mission state must be an object")
    mission = dict(exact(OPTIONAL_MISSION | dict(value), MISSION_KEYS | OPTIONAL_MISSION.keys(), "mission state"))
    require(mission["mode"] in MISSION_MODES, "mode must be one of SYSTEM_MODES (S-00)")
    require(mission["phase"] in MISSION_PHASES, "phase must be one of MISSION_PHASES")
    _pair(mission["trigger_event_id"], mission["started_at_ns"], "trigger_event_id/started_at_ns")
    if mission["trigger_event_id"] is not None:
        uid(mission["trigger_event_id"], "trigger_event_id")
        timestamp(mission["started_at_ns"], "started_at_ns")
    if mission["generation"] is not None:
        uint(mission["generation"], "generation")
    timestamp(mission["updated_at_ns"], "updated_at_ns")
    if mission["reason"] is not None:
        text(mission["reason"], "reason")
    require(mission["mode"] != FINISHED_MODE or mission["phase"] == "FINISHED",
            "MISSION_FINISHED only with phase FINISHED (C-10)")
    dispatch = [mission[key] for key in DISPATCH_DEFAULTS]
    require(all(v is None for v in dispatch) or all(v is not None for v in dispatch),
            "dispatch fields are all present or all None")
    if mission["target_id"] is not None:
        uid(mission["target_id"], "target_id")
        for key in ("target_revision", "route_revision", "segment_index", "segment_count"):
            uint(mission[key], key)
        require(mission["segment_index"] <= mission["segment_count"], "segment_index must not pass segment_count")
        require(mission["segment_phase"] in SEGMENT_PHASES, "segment_phase must be one of SEGMENT_PHASES")
    return mission


def _command(value, allowed):
    command = exact(value, COMMAND_KEYS, "operator command")
    uid(command["command_id"], "command_id")
    require(command["kind"] in OPERATOR_COMMAND_KINDS, "kind must be one of OPERATOR_COMMAND_KINDS")
    text(command["operator_id"], "operator_id")
    if command["robot_id"] is not None:
        _member(command["robot_id"], allowed, "robot_id")
    if command["target_id"] is not None:
        uid(command["target_id"], "target_id")
    for key in ("segment_index", "route_revision"):
        if command[key] is not None:
            uint(command[key], key)
    timestamp(command["requested_at_ns"], "requested_at_ns")
    require(type(command["accepted"]) is bool, "accepted must be boolean")
    if command["result"] is not None:
        text(command["result"], "result", 64)
    if command["reason"] is not None:
        text(command["reason"], "reason", 1024)
    require(command["accepted"] or command["reason"] is not None, "a rejected command needs a reason")
    return command


def _map_snapshot(value, allowed):
    snapshot = exact(value, MAP_KEYS, "map snapshot")
    uid(snapshot["snapshot_id"], "snapshot_id")
    _member(snapshot["robot_id"], allowed, "robot_id")
    uint(snapshot["revision"], "revision")
    text(snapshot["path"], "path", 1024)
    sha(snapshot["sha256"], "sha256")
    timestamp(snapshot["at_ns"], "at_ns")
    return snapshot


def _exit_location(value):
    exit_ = exact(value, EXIT_KEYS, "exit location")
    uid(exit_["location_id"], "location_id")
    timestamp(exit_["observed_at_ns"], "observed_at_ns")
    timestamp(exit_["received_at_ns"], "received_at_ns")
    uid(exit_["map_session_id"], "map_session_id")
    uint(exit_["map_revision"], "map_revision")
    uint(exit_["transform_revision"], "transform_revision")
    text(exit_["mission_frame"], "mission_frame")
    for key in ("x", "y"):
        _finite(exit_[key], key)
    require(exit_["class_label"] == EXIT_CLASS_LABEL, "class_label must be " + EXIT_CLASS_LABEL)
    return exit_


class SiteStateLedger(_Database):
    """Operations-owned writer for the screen's site state; never share this file between PCs."""

    def __init__(self, db_path, *, allowed_robot_ids):
        self._allowed_robot_ids = allowed_robots(allowed_robot_ids)
        super().__init__(db_path, ROLE, {"allowed_robot_ids": canonical(self.allowed_robot_ids)},
                         schema=SCHEMA_VERSION, upgrades_from=UPGRADES_FROM, tables=TABLES, migrate=_migrate)

    def record_robot_status(self, status):
        """Store the latest snapshot. Returns False for a stale or identical resend."""
        status = _robot_status(status, self.allowed_robot_ids)
        encoded = canonical(status)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM robot_status WHERE robot_id = ?", (status["robot_id"],)).fetchone()
            if row is not None:
                if row["robot_boot_id"] == status["robot_boot_id"]:
                    if status["state_sequence"] == row["state_sequence"]:
                        # The pose comes on its own topic (record_robot_pose); it is not this status's content.
                        require(canonical(_stored_status(row["status_json"]) | {"pose": None})
                                == canonical(status | {"pose": None}),
                                "same boot and sequence with different content", "ID_CONTENT_CONFLICT")
                        return False
                    if status["state_sequence"] < row["state_sequence"]:
                        return False
                elif status["stamp_ns"] <= row["stamp_ns"]:
                    return False  # a late message from another boot must not replace a newer snapshot
            db.execute("INSERT OR REPLACE INTO robot_status VALUES (?, ?, ?, ?, ?, ?)",
                       (status["robot_id"], status["robot_boot_id"], status["state_sequence"], status["stamp_ns"],
                        encoded, timestamp(time.time_ns(), "received_at_ns")))
        return True

    def record_robot_pose(self, robot_id, pose):
        """Replace only the pose of the robot's current snapshot (None hides it).

        False when there is no snapshot yet or the pose is unchanged. received_at_ns is kept: it
        is when the status arrived, and a newer pose must not make an old status look fresh.
        """
        _member(robot_id, self.allowed_robot_ids, "robot_id")
        pose = _pose(pose)
        with self._transaction() as db:
            row = db.execute("SELECT status_json FROM robot_status WHERE robot_id = ?", (robot_id,)).fetchone()
            if row is None:
                return False
            stored = _stored_status(row["status_json"])
            if stored["pose"] == pose:
                return False
            db.execute("UPDATE robot_status SET status_json = ? WHERE robot_id = ?",
                       (canonical(stored | {"pose": pose}), robot_id))
        return True

    def record_site_event(self, event):
        """Append one site-camera event. Returns False for an identical resend."""
        require(isinstance(event, SiteEvent), "SiteEvent required")
        encoded = canonical(asdict(event))
        with self._transaction() as db:
            row = db.execute("SELECT event_json FROM site_events WHERE event_id = ?", (event.event_id,)).fetchone()
            if row is not None:
                require(canonical(_stored_event(row["event_json"])) == encoded, "event_id has different content",
                        "ID_CONTENT_CONFLICT")
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
                    require(canonical(_stored_target(row["state_json"])) == encoded,
                            "same target and time with different content",
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

    def record_operator_command(self, command):
        """Append one console command and its answer, rejections included. Returns False for a resend."""
        command = _command(command, self.allowed_robot_ids)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM operator_commands WHERE command_id = ?",
                             (command["command_id"],)).fetchone()
            if row is not None:
                require(_command_row(row) == command, "command_id has different content", "ID_CONTENT_CONFLICT")
                return False
            db.execute("INSERT INTO operator_commands VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                       (command["command_id"], command["kind"], command["operator_id"], command["robot_id"],
                        command["target_id"], command["segment_index"], command["route_revision"],
                        command["requested_at_ns"], int(command["accepted"]), command["result"], command["reason"]))
        return True

    def record_map_snapshot(self, snapshot):
        """Append one stored map image (IF-084). Returns False for a resend."""
        snapshot = _map_snapshot(snapshot, self.allowed_robot_ids)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM map_snapshots WHERE snapshot_id = ?", (snapshot["snapshot_id"],)).fetchone()
            if row is not None:
                require(dict(row) == snapshot, "snapshot_id has different content", "ID_CONTENT_CONFLICT")
                return False
            db.execute("INSERT INTO map_snapshots VALUES (?, ?, ?, ?, ?, ?)",
                       (snapshot["snapshot_id"], snapshot["robot_id"], snapshot["revision"], snapshot["path"],
                        snapshot["sha256"], snapshot["at_ns"]))
        return True

    def record_exit_location(self, value):
        """Append one QR exit observation. Returns False for a resend; other content for the same id conflicts."""
        exit_ = _exit_location(value)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM exit_locations WHERE location_id = ?", (exit_["location_id"],)).fetchone()
            if row is not None:
                # A later resend keeps the first arrival: only the observation itself must match.
                same = {k: v for k, v in dict(row).items() if k != "received_at_ns"}
                require(same == {k: v for k, v in exit_.items() if k != "received_at_ns"},
                        "location_id has different content", "ID_CONTENT_CONFLICT")
                return False
            db.execute("INSERT INTO exit_locations VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                       tuple(exit_[k] for k in ("location_id", "observed_at_ns", "received_at_ns", "map_session_id",
                                                "map_revision", "transform_revision", "mission_frame", "x", "y",
                                                "class_label")))
        return True

    def record_mission_state(self, mission):
        """Replace the single mission row. Only a newer ``updated_at_ns`` replaces it."""
        mission = _mission(mission)
        with self._transaction() as db:
            row = db.execute("SELECT * FROM mission_state WHERE singleton = 1").fetchone()
            if row is not None:
                if mission["updated_at_ns"] == row["updated_at_ns"]:
                    require({key: row[key] for key in MISSION_COLUMNS} == mission,
                            "same mission time with different content", "ID_CONTENT_CONFLICT")
                    return False
                if mission["updated_at_ns"] < row["updated_at_ns"]:
                    return False
            db.execute(f"INSERT OR REPLACE INTO mission_state (singleton, {', '.join(MISSION_COLUMNS)}) "
                       f"VALUES (1{', ?' * len(MISSION_COLUMNS)})", [mission[key] for key in MISSION_COLUMNS])
        return True


def _command_row(row):
    return {key: row[key] for key in COMMAND_KEYS} | {"accepted": bool(row["accepted"])}


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


def read_allowed_robot_ids(db_path):
    """The deployment's robots as the writer fixed them in the DB (#147: a card per robot before any status)."""
    def query(db):
        metadata = dict(db.execute("SELECT key, value FROM metadata"))
        return {"allowed_robot_ids": sorted(allowed_robots(json.loads(metadata["allowed_robot_ids"])))}
    return _read(db_path, query)


def read_robot_status(db_path):
    def query(db):
        rows = db.execute("SELECT status_json, received_at_ns FROM robot_status ORDER BY robot_id").fetchall()
        return {"robots": [_stored_status(row["status_json"]) | {"received_at_ns": row["received_at_ns"]}
                           for row in rows]}
    return _read(db_path, query)


def read_site_events(db_path, *, limit=100):
    limit = _limit(limit)

    def query(db):
        count = db.execute("SELECT COUNT(DISTINCT event_id) FROM site_events WHERE kind = 'EXIT_PASS'").fetchone()[0]
        rows = db.execute("SELECT event_json FROM site_events ORDER BY observed_at_ns, event_id LIMIT ?",
                          (limit,)).fetchall()
        return {"exit_pass_count": count, "site_events": [_stored_event(row["event_json"]) for row in rows]}
    return _read(db_path, query)


def read_target_states(db_path):
    def query(db):
        rows = db.execute("SELECT state_json FROM target_states ORDER BY target_id").fetchall()
        return {"targets": [_stored_target(row["state_json"]) for row in rows]}
    return _read(db_path, query)


def read_error_log(db_path, *, limit=100):
    """Newest first; ``limit`` keeps the latest N entries."""
    limit = _limit(limit)

    def query(db):
        rows = db.execute("SELECT seq, at_ns, robot_id, task_id, code, detail FROM error_log "
                          "ORDER BY at_ns DESC, seq DESC LIMIT ?", (limit,)).fetchall()
        return {"errors": [dict(row) for row in rows]}
    return _read(db_path, query)


def read_mission_state(db_path):
    """The single mission row, or None before operations has written one."""
    def query(db):
        row = db.execute("SELECT * FROM mission_state WHERE singleton = 1").fetchone()
        return {"mission": None if row is None else {key: row[key] for key in MISSION_COLUMNS}}
    return _read(db_path, query)


def read_operator_commands(db_path, *, limit=100):
    """Newest first; ``limit`` keeps the latest N commands."""
    limit = _limit(limit)

    def query(db):
        rows = db.execute("SELECT * FROM operator_commands ORDER BY requested_at_ns DESC, command_id DESC LIMIT ?",
                          (limit,)).fetchall()
        return {"commands": [_command_row(row) for row in rows]}
    return _read(db_path, query)


def read_map_snapshots(db_path, *, limit=100):
    """Newest first. The image bytes stay in the file at ``path``; check them against ``sha256``."""
    limit = _limit(limit)

    def query(db):
        rows = db.execute("SELECT * FROM map_snapshots ORDER BY at_ns DESC, snapshot_id DESC LIMIT ?",
                          (limit,)).fetchall()
        return {"maps": [dict(row) for row in rows]}
    return _read(db_path, query)


def read_exit_locations(db_path, *, limit=100):
    """Newest ``observed_at_ns`` first, ties by ``location_id`` (console, OPS-150); ``received_at_ns`` is for display."""
    limit = _limit(limit)

    def query(db):
        rows = db.execute("SELECT * FROM exit_locations ORDER BY observed_at_ns DESC, location_id ASC LIMIT ?",
                          (limit,)).fetchall()
        return {"exits": [dict(row) for row in rows]}
    return _read(db_path, query)
