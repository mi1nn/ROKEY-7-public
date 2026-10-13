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
from .site_state import OPERATOR_COMMAND_KINDS

COMMAND_PATH = "/api/operator-commands"
MAX_BODY_BYTES = 8192
# Per-kind request fields = rescue_coordination mission.COMMAND_FIELDS (the coordinator's gate).
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


def make_server(host, port, gate, *, log=None):
    """A threading HTTP server with one route, POST COMMAND_PATH -> gate.handle. Not started."""

    class Handler(BaseHTTPRequestHandler):
        def _answer(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):  # noqa: N802 - http.server naming
            if self.path != COMMAND_PATH:
                return self._answer(404, {"error": "NOT_FOUND"})
            try:
                length = int(self.headers.get("Content-Length", ""))
            except ValueError:
                return self._answer(411, {"error": "INVALID_INPUT", "detail": "Content-Length required"})
            if not 0 <= length <= MAX_BODY_BYTES:
                return self._answer(413, {"error": "INVALID_INPUT", "detail": f"body over {MAX_BODY_BYTES} bytes"})
            status, body = gate.handle(self.rfile.read(length), received_at_ns=time.time_ns())
            self._answer(status, body)

        def log_message(self, fmt, *args):
            if log is not None:
                log(fmt % args)

    return ThreadingHTTPServer((host, port), Handler)
