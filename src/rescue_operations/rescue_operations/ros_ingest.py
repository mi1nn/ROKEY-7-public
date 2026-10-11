"""DB writer adapter core: 1차 IDL messages -> site-state records (no ROS import).

Messages are read by field name only, so ROS message objects and plain attribute
objects are handled alike; the rclpy node (ros_node.py) subscribes and calls this.
Every refused or conflicting message is kept out of the tables it would have
changed and recorded in error_log with the receive time and the message identity,
so a failure is visible on the screen instead of silently dropped (AGENTS 7절).
The DB file failing itself cannot be written to that same DB; the caller logs it.
"""
from rescue_contracts import RescueError, SiteEvent

from .schema import OperationsError, require

# C-10 (2026-10-10 작전 결정): a SiteEvent is made only from a VALID view.
VISIBLE = "VALID"
MAX_DETAIL = 1024


def _optional(value):
    """1차 IDL has no null for strings; an empty string means absent."""
    return None if value == "" else value


def _invalid(exc, label):
    if isinstance(exc, OperationsError):
        return exc
    return OperationsError("INVALID_INPUT", label + ": " + getattr(exc, "detail", str(exc)))


def site_event_from_msg(msg):
    """rescue_interfaces/SiteEvent -> rescue_contracts.SiteEvent. Raises OperationsError(INVALID_INPUT).

    Dropped on purpose (not in the Python contract): source_boot_id, metrics, policy_version,
    visibility_state (only VALID passes), the envelope and the evidence's other fields.
    """
    try:
        require(msg.visibility_state == VISIBLE, "site event visibility_state must be VALID (C-10)")
        time = msg.time
        return SiteEvent(msg.event_id, msg.camera_id, msg.kind, time.observed_at_ns, msg.evidence.evidence_id,
                         msg.evidence.sha256, msg.confidence, time.clock_domain_id, time.stamp_semantics,
                         time.clock_state, time.clock_error_bound_ns if time.has_clock_error_bound else None,
                         target_id=_optional(msg.target_id), watch_id=_optional(msg.watch_id))
    except (OperationsError, RescueError, AttributeError, TypeError) as exc:
        raise _invalid(exc, "site event") from exc


class SiteStateIngest:
    """Writes converted messages through one SiteStateLedger; failures go to error_log."""

    def __init__(self, ledger):
        self.ledger = ledger

    def on_site_event(self, msg, *, received_at_ns):
        """True stored, False identical resend, None refused (and logged)."""
        try:
            return self.ledger.record_site_event(site_event_from_msg(msg))
        except OperationsError as exc:
            self._log(exc, received_at_ns, f"site_event {getattr(msg, 'event_id', None)!r}")
            return None

    def _log(self, exc, received_at_ns, what, robot_id=None, task_id=None):
        detail = f"{what}: {exc.detail_code}: {exc.detail}"[:MAX_DETAIL]
        self.ledger.record_error(dict(at_ns=received_at_ns, robot_id=robot_id, task_id=task_id, code=exc.code,
                                      detail=detail))
