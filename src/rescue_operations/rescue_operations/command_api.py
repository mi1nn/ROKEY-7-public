"""Operator command API core: console HTTP JSON -> coordinator SubmitOperatorCommand (no ROS import).

Shape agreed on #115 (issuecomment-6118222924). The console sends a command it made the
id for; this checks it, asks the coordinator through ``send`` and answers:

- 200 with the coordinator's answer (accepted, refused, or DUPLICATE with its first answer),
- 400 when operations refuses the request itself (never sent to the coordinator),
- 503 / 504 when the coordinator did not answer. Those bodies carry no ``accepted`` on
  purpose: the coordinator may still have taken the command, and the console shows a body
  without ``accepted`` as 「전달됐는지 모름」 instead of a refusal.

Answers and operations refusals go to ``operator_commands`` (refusals too, console-and-db-plan
4절). A missing answer goes to ``error_log`` only; a resend with the same command_id gets
the coordinator's first answer and is recorded then. The rclpy node (ros_node.py) owns the
service client and serializes ledger writes with its subscriptions.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import time

from .schema import OperationsError, require, text, uid, uint
from .site_state import OPERATOR_COMMAND_KINDS, list_runs, read_operator_commands_range

COMMAND_PATH = "/api/operator-commands"
WEBCAM_PATH = "/api/webcam"  # GET: plugged cameras; POST {device, operator_id}: switch webcam_node (OPS-183)
RESET_PATH = "/api/runs/reset"  # POST {command_id, operator_id[, note]}: the scenario reset (초기화, 1.5)
# A robot status older than this cannot show the robot is at rest: the reset is refused. Same value as the
# console's stale robot card (rescue_console view.STATUS_STALE_S) [제안, 2026-10-15].
RESET_STATUS_STALE_NS = 5_000_000_000
# Refusal reasons in the order they are checked (작전·웹개발 2026-10-15). The first one that holds is the answer.
RESET_MOVING = frozenset({"MOVING", "STOPPING", "STOP_UNCONFIRMED"})  # anything but STOPPED counts as moving
MAX_BODY_BYTES = 8192
# Per-kind request fields = rescue_coordination mission.ALL_COMMAND_FIELDS (the coordinator's gate).
# Phase 1 takes these six; PAUSE/RESUME/CANCEL/STOP_ROBOT and the other IDL operations are refused.
# SET_CAMERA_WATCH carries no ROI: the console has no ROI picker, so roi_xyxy_normalized goes as zeros
# ("not given") and the coordinator uses its deployment ROI or refuses (C-11 row 2).
COMMAND_FIELDS = {
    "START_SEARCH": (),
    "DISPATCH_SUPPORT": ("target_id", "target_revision"),
    "CONTINUE_SEGMENT": ("target_id", "target_revision", "route_revision", "segment_index"),
    "REQUEST_RECHECK": ("target_id",),
    "RETURN_HOME": (),
    "SET_CAMERA_WATCH": ("target_id",),
}
BASE_FIELDS = ("command_id", "kind", "operator_id")
UINT32_MAX = (1 << 32) - 1
NO_ANSWER = "ACCEPT_TIMEOUT"  # C-09 common code for a coordinator that did not answer


class CommandUnavailable(Exception):
    """The coordinator service is not there; nothing was sent."""


class CommandTimeout(Exception):
    """The request went out and no answer came in time; it may have been taken."""


def parse_command(raw):
    """Request body bytes -> checked command dict. Raises OperationsError(INVALID_INPUT)."""
    require(isinstance(raw, bytes) and len(raw) <= MAX_BODY_BYTES, f"body must be at most {MAX_BODY_BYTES} bytes")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, ValueError) as exc:
        raise OperationsError("INVALID_INPUT", "body is not JSON") from exc
    require(isinstance(value, dict), "body must be a JSON object")
    kind = value.get("kind")
    require(kind in COMMAND_FIELDS, f"kind must be one of {sorted(COMMAND_FIELDS)}")
    require(set(value) == set(BASE_FIELDS + COMMAND_FIELDS[kind]),
            f"{kind} takes exactly {sorted(BASE_FIELDS + COMMAND_FIELDS[kind])}")
    uid(value["command_id"], "command_id")
    text(value["operator_id"], "operator_id")
    if "target_id" in value:
        uid(value["target_id"], "target_id")
    for name in ("target_revision", "route_revision"):
        if name in value:
            uint(value[name], name)
    if "segment_index" in value:
        uint(value["segment_index"], "segment_index")
        require(value["segment_index"] <= UINT32_MAX, "segment_index must be uint32")
    return dict(value)


def service_fields(command, *, issued_at_ns, timeout_ns):
    """Checked command -> SubmitOperatorCommand request fields (#115 「operations 가 채우는 것」).

    Fields the six operations do not read stay empty; robot_id is empty because the
    coordinator picks the robot by role. The envelope is filled by the node.
    """
    return dict(command_id=command["command_id"], operation=command["kind"], operator_id=command["operator_id"],
                robot_id="", target_id=command.get("target_id", ""), expected_task_id="", expected_generation=0,
                expected_target_revision=command.get("target_revision", 0), route_id="",
                route_revision=command.get("route_revision", 0), segment_index=command.get("segment_index", 0),
                issued_at_ns=issued_at_ns, deadline_ns=issued_at_ns + timeout_ns, roi_xyxy_normalized=[0.0] * 4,
                association_evidence_id="")


def _optional(value):
    return value or None


class CommandGate:
    """One command in, one (HTTP status, JSON body) out. ``send(fields)`` returns the coordinator's
    reply dict (accepted, request_state, reason_code, created_task_id) or raises
    CommandUnavailable / CommandTimeout."""

    def __init__(self, ledger, send, *, timeout_ns):
        self.ledger = ledger
        self.send = send
        self.timeout_ns = timeout_ns
        # command_id -> first issue time. The coordinator compares the whole command, issued_at_ns
        # included, so a resend must carry the first time to get DUPLICATE (kept for this process).
        self._issued = {}

    def handle(self, raw, *, received_at_ns):
        try:
            command = parse_command(raw)
        except OperationsError as exc:
            return self._refuse(raw, exc, received_at_ns)
        issued_at_ns = self._issued.setdefault(command["command_id"], received_at_ns)
        fields = service_fields(command, issued_at_ns=issued_at_ns, timeout_ns=self.timeout_ns)
        try:
            reply = self.send(fields)
        except (CommandUnavailable, CommandTimeout) as exc:
            status = 503 if isinstance(exc, CommandUnavailable) else 504
            self._log(received_at_ns, f"operator_command {command['command_id']} {command['kind']}: "
                                      f"no coordinator answer ({status}): {exc}", NO_ANSWER)
            return status, {"error": NO_ANSWER, "request_id": command["command_id"], "detail": str(exc)}
        accepted, state = bool(reply["accepted"]), reply["request_state"]
        # A refusal always names a reason (DB rule and the console's answer shape).
        reason = _optional(reply["reason_code"]) or (None if accepted else "UNKNOWN")
        self._record(command, issued_at_ns, accepted=accepted, result=state, reason=reason,
                     duplicate=state == "DUPLICATE")
        return 200, {"accepted": accepted, "request_id": command["command_id"], "reason": reason,
                     "request_state": state, "created_task_id": _optional(reply["created_task_id"])}

    def _refuse(self, raw, exc, received_at_ns):
        """400. The command row needs a usable id, kind and operator; otherwise error_log keeps it."""
        try:
            value = json.loads(raw)
            command_id = uid(value["command_id"], "command_id")
            require(value["kind"] in OPERATOR_COMMAND_KINDS, "kind")
            text(value["operator_id"], "operator_id")
        except Exception:  # noqa: BLE001 - anything unusable just means no command row
            command_id = None
        detail = f"{exc.code}: {exc.detail}"
        if command_id is None:
            self._log(received_at_ns, f"operator_command refused: {detail}", exc.code)
        else:
            row = dict(command_id=command_id, kind=value["kind"], operator_id=value["operator_id"], robot_id=None,
                       target_id=None, segment_index=None, route_revision=None, requested_at_ns=received_at_ns,
                       accepted=False, result="REFUSED_BY_OPERATIONS", reason=exc.code)
            try:
                self.ledger.record_operator_command(row)
            except OperationsError as conflict:  # same id seen with other content: keep the first row
                self._log(received_at_ns, f"operator_command {command_id}: {conflict.detail}", conflict.code)
        return 400, {"accepted": False, "request_id": command_id, "reason": exc.code, "detail": exc.detail}

    def _record(self, command, received_at_ns, *, accepted, result, reason, duplicate):
        row = dict(command_id=command["command_id"], kind=command["kind"], operator_id=command["operator_id"],
                   robot_id=None, target_id=command.get("target_id"), segment_index=command.get("segment_index"),
                   route_revision=command.get("route_revision"), requested_at_ns=received_at_ns, accepted=accepted,
                   result=result, reason=reason)
        try:
            self.ledger.record_operator_command(row)
        except OperationsError as exc:
            if not (duplicate and exc.code == "ID_CONTENT_CONFLICT"):  # a resend keeps the first row
                self._log(received_at_ns, f"operator_command {command['command_id']}: {exc.detail}",
                          exc.code)

    def _log(self, at_ns, detail, code):
        self.ledger.record_error(dict(at_ns=at_ns, robot_id=None, task_id=None, code=code, detail=detail[:1024]))


class WebcamSwitch:
    """The console's camera pick (OPS-183): plugged ``by-id`` cameras and switching webcam_node's device_path.

    Only a plugged camera is sent; anything else is refused with the list (never kept quietly on the old one).
    A switch restarts the quake judge and clears the exit watch in webcam_node, so it is written to error_log and
    shows in the console's command and error history. error_log takes C-09 common codes only, so the detail starts
    with CAMERA_SWITCHED / CAMERA_SWITCH_REFUSED: SENSOR_UNAVAILABLE (the judge is empty while it restarts),
    NOT_FOUND (not plugged in), INVALID_INPUT (webcam_node refused), DRIVER_UNAVAILABLE (webcam_node did not answer).
    """

    def __init__(self, ledger, *, list_cameras, get_device, set_device, by_id_dir):
        self.ledger, self.list_cameras, self.get_device, self.set_device = ledger, list_cameras, get_device, set_device
        self.by_id_dir = by_id_dir

    def _name(self, path):
        return path[len(self.by_id_dir):] if path and path.startswith(self.by_id_dir) else path

    def state(self):
        try:
            current = self._name(self.get_device())
        except (CommandUnavailable, CommandTimeout):
            current = None
        return 200, {"cameras": self.list_cameras(), "current": current}

    def switch(self, raw, *, received_at_ns):
        try:
            require(isinstance(raw, bytes) and len(raw) <= MAX_BODY_BYTES, "body too large")
            value = json.loads(raw)
            require(isinstance(value, dict), "body must be a JSON object")
            device, operator = value.get("device"), text(value.get("operator_id"), "operator_id")
            require(isinstance(device, str) and device, "device must be a camera name")
        except (OperationsError, UnicodeDecodeError, ValueError) as exc:
            return 400, {"switched": False, "reason": "INVALID_INPUT", "detail": str(exc)}
        cameras = self.list_cameras()
        if device not in cameras:
            self._log(received_at_ns, "NOT_FOUND", f"CAMERA_SWITCH_REFUSED {operator}: {device} is not plugged in "
                                                   f"({cameras})")
            return 400, {"switched": False, "reason": "UNKNOWN_DEVICE", "cameras": cameras}
        before = self.state()[1]["current"]
        try:
            ok, reason = self.set_device(self.by_id_dir + device)
        except (CommandUnavailable, CommandTimeout) as exc:
            self._log(received_at_ns, "DRIVER_UNAVAILABLE",
                      f"CAMERA_SWITCH_REFUSED {operator}: {device}: webcam_node did not answer: {exc}")
            return 503, {"switched": False, "reason": "WEBCAM_NODE_UNAVAILABLE", "detail": str(exc)}
        if not ok:
            self._log(received_at_ns, "INVALID_INPUT", f"CAMERA_SWITCH_REFUSED {operator}: {device}: {reason}")
            return 200, {"switched": False, "reason": reason, "current": before}
        self._log(received_at_ns, "SENSOR_UNAVAILABLE",
                  f"CAMERA_SWITCHED {operator}: {before} -> {device} — 카메라 전환, 지진 판정 다시 시작·출구 감시 풀림")
        return 200, {"switched": True, "current": device}

    def _log(self, at_ns, code, detail):
        self.ledger.record_error(dict(at_ns=at_ns, robot_id=None, task_id=None, code=code, detail=detail[:1024]))


class RunReset:
    """The console's reset button (초기화): checks the robots are at rest, then has the ledger start a new run.

    It lives in db_writer's process and uses db_writer's ledger, the one writer of the file. The run id is
    read from the DB on every write (site_state.SiteStateLedger.run_id), so even a reset made through another
    connection is what the next row is written under. Refused, with the reason in operator_commands, when a
    configured robot has no status or one older than RESET_STATUS_STALE_NS (ROBOT_STATUS_STALE), is not
    STOPPED (ROBOT_MOVING), has a task (TASK_ACTIVE) or is not DOCKED (ROBOT_NOT_DOCKED). Nothing falls back:
    a refused reset changes nothing else. The answer has the operator command shape; ``run_id`` is added.
    ``coordinator_boot_id()`` gives the boot of the coordinator heard last (ros_ingest); the new run stops
    recording it, so its late or latched messages do not land in the new run.
    """

    def __init__(self, ledger, *, coordinator_boot_id):
        self.ledger = ledger
        self.coordinator_boot_id = coordinator_boot_id

    def refusal(self, now_ns):
        statuses = {status["robot_id"]: status for status in self.ledger.robot_snapshots()}
        robots = [statuses.get(robot) for robot in self.ledger.allowed_robot_ids]
        checks = (("ROBOT_STATUS_STALE", lambda s: s is None or now_ns - s["received_at_ns"] > RESET_STATUS_STALE_NS),
                  ("ROBOT_MOVING", lambda s: s["motion_state"] != "STOPPED"),
                  ("TASK_ACTIVE", lambda s: s["active_task_id"] is not None),
                  ("ROBOT_NOT_DOCKED", lambda s: s["dock_state"] != "DOCKED"))
        for reason, failing in checks:
            if any(failing(status) for status in robots):
                return reason
        return None

    def handle(self, raw, *, received_at_ns):
        try:
            require(isinstance(raw, bytes) and len(raw) <= MAX_BODY_BYTES, f"body must be at most {MAX_BODY_BYTES} bytes")
            value = json.loads(raw)
            require(isinstance(value, dict) and set(value) in ({"command_id", "operator_id"},
                                                               {"command_id", "operator_id", "note"}),
                    "RESET_RUN takes command_id, operator_id and an optional note")
            command_id, operator = uid(value["command_id"], "command_id"), text(value["operator_id"], "operator_id")
            note = value.get("note")
            if note is not None:
                text(note, "note", 1024)
        except (OperationsError, UnicodeDecodeError, ValueError) as exc:
            code = exc.code if isinstance(exc, OperationsError) else "INVALID_INPUT"
            return 400, {"accepted": False, "request_id": None, "reason": code, "detail": str(exc)}
        first = self._first_answer(command_id)
        if first is not None:
            return 200, first
        row = dict(command_id=command_id, kind="RESET_RUN", operator_id=operator, robot_id=None, target_id=None,
                   segment_index=None, route_revision=None, requested_at_ns=received_at_ns)
        try:
            reason = self.refusal(received_at_ns)
            if reason is not None:
                self.ledger.record_operator_command(row | dict(accepted=False, result="REJECTED", reason=reason))
                return 200, self._answer(command_id, False, reason, "REJECTED", None)
            _, new = self.ledger.reset_run(at_ns=received_at_ns, note=note,
                                           previous_coordinator_boot_id=self.coordinator_boot_id(),
                                           command=row | dict(accepted=True, result="ACCEPTED", reason=None))
        except OperationsError as exc:  # the DB failed: nobody knows if it reset, so no ``accepted``
            return 503, {"error": exc.code, "request_id": command_id, "detail": exc.detail}
        return 200, self._answer(command_id, True, None, "ACCEPTED", new)

    def _first_answer(self, command_id):
        """A resend gets the first answer (DUPLICATE) and does not reset again."""
        for run in self._runs():
            for row in read_operator_commands_range(self.ledger.path, run_id=run["run_id"])["commands"]:
                if row["command_id"] == command_id:
                    created = self._next_run(run["run_id"]) if row["accepted"] else None
                    return self._answer(command_id, row["accepted"], row["reason"], "DUPLICATE", created)
        return None

    def _runs(self):
        return list_runs(self.ledger.path)["runs"]

    def _next_run(self, run_id):
        ids = [run["run_id"] for run in self._runs()]
        index = ids.index(run_id)
        return ids[index + 1] if index + 1 < len(ids) else None

    @staticmethod
    def _answer(command_id, accepted, reason, state, run_id):
        return {"accepted": accepted, "request_id": command_id, "reason": reason, "request_state": state,
                "created_task_id": None, "run_id": run_id}


def make_server(host, port, gate, *, log=None, webcam=None, reset=None):
    """A threading HTTP server: POST COMMAND_PATH -> gate.handle; with ``webcam`` also GET/POST WEBCAM_PATH;
    with ``reset`` also POST RESET_PATH. Not started."""

    class Handler(BaseHTTPRequestHandler):
        def _answer(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):  # noqa: N802 - http.server naming
            if self.path != WEBCAM_PATH or webcam is None:
                return self._answer(404, {"error": "NOT_FOUND"})
            self._answer(*webcam.state())

        def do_POST(self):  # noqa: N802 - http.server naming
            served = {COMMAND_PATH: gate.handle}
            if webcam is not None:
                served[WEBCAM_PATH] = webcam.switch
            if reset is not None:
                served[RESET_PATH] = reset.handle
            if self.path not in served:
                return self._answer(404, {"error": "NOT_FOUND"})
            try:
                length = int(self.headers.get("Content-Length", ""))
            except ValueError:
                return self._answer(411, {"error": "INVALID_INPUT", "detail": "Content-Length required"})
            if not 0 <= length <= MAX_BODY_BYTES:
                return self._answer(413, {"error": "INVALID_INPUT", "detail": f"body over {MAX_BODY_BYTES} bytes"})
            status, body = served[self.path](self.rfile.read(length), received_at_ns=time.time_ns())
            self._answer(status, body)

        def log_message(self, fmt, *args):
            if log is not None:
                log(fmt % args)

    return ThreadingHTTPServer((host, port), Handler)
