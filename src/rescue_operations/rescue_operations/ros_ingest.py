"""DB writer adapter core: 1차 IDL messages -> site-state records (no ROS import).

Messages are read by field name only, so ROS message objects and plain attribute
objects are handled alike; the rclpy node (ros_node.py) subscribes and calls this.
Every refused or conflicting message is kept out of the tables it would have
changed and recorded in error_log with the receive time and the message identity,
so a failure is visible on the screen instead of silently dropped (AGENTS 7절).
The DB file failing itself cannot be written to that same DB; the caller logs it.
"""
import math

from rescue_contracts import RescueError, SiteEvent

from .schema import OperationsError, require

# C-10 (2026-10-10 작전 결정): a SiteEvent is made only from a VALID view.
VISIBLE = "VALID"
# RobotPose.MISSION_POSE_STATUS_VALID: only a valid mission-frame pose is shown.
POSE_VALID = "VALID"
UNKNOWN_ROLE = "UNKNOWN"
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


def pose_from_msg(msg):
    """rescue_interfaces/RobotPose -> DB pose, or None unless the mission pose is VALID (display only, C-05)."""
    try:
        if msg.mission_pose_status != POSE_VALID:
            return None
        stamped = msg.mission_pose
        p, q = stamped.pose.pose.position, stamped.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"x": float(p.x), "y": float(p.y), "yaw": yaw, "frame_id": stamped.header.frame_id}
    except (AttributeError, TypeError, ValueError) as exc:
        raise _invalid(exc, "robot pose") from exc


def robot_status_from_msg(msg, *, role, pose):
    """rescue_interfaces/RobotStatus -> site_state robot snapshot (validated when recorded).

    Not in the 1차 message: role (from the deployment) and pose (from RobotPose). Absent values
    are None, never a placeholder number: battery when battery_known is false (its time is the
    status time), task/generation when active_task_id is empty, segment_index without an active
    task [제안], dock_state when empty. Dropped: authority_epoch, task_state, stop_*, lease,
    sensor_quality, nav2_state, dock_observed_at_ns.
    """
    try:
        task = _optional(msg.active_task_id)
        battery = msg.battery_known
        return dict(robot_id=msg.robot_id, robot_boot_id=msg.robot_boot_id, state_sequence=msg.state_sequence,
                    role=role, stamp_ns=msg.observed_at_ns, pose=pose, motion_state=msg.motion_state,
                    readiness=msg.readiness, battery_percent=msg.battery_fraction * 100.0 if battery else None,
                    battery_stamp_ns=msg.observed_at_ns if battery else None, active_task_id=task,
                    generation=None if task is None else msg.generation, dock_state=_optional(msg.dock_state),
                    segment_index=None if task is None else msg.segment_index)
    except (AttributeError, TypeError) as exc:
        raise _invalid(exc, "robot status") from exc


class SiteStateIngest:
    """Writes converted messages through one SiteStateLedger; failures go to error_log."""

    def __init__(self, ledger, *, roles=None):
        self.ledger = ledger
        self.roles = dict(roles or {})  # robot_id -> SCOUT/SUPPORT from the deployment profile
        self._poses = {}  # robot_id -> latest pose (None once the latest is not VALID)

    def on_robot_pose(self, msg, *, received_at_ns):
        """True a VALID pose is kept, False the robot's pose is now hidden, None refused (and logged)."""
        robot = getattr(msg, "robot_id", None)
        try:
            pose = pose_from_msg(msg)
        except OperationsError as exc:
            self._log(exc, received_at_ns, f"robot_pose {robot!r}", robot_id=robot)
            return None
        self._poses[robot] = pose
        return pose is not None

    def on_robot_status(self, msg, *, received_at_ns):
        """True stored, False stale or identical resend, None refused (and logged)."""
        robot = getattr(msg, "robot_id", None)
        try:
            status = robot_status_from_msg(msg, role=self.roles.get(robot, UNKNOWN_ROLE),
                                           pose=self._poses.get(robot))
            return self.ledger.record_robot_status(status)
        except OperationsError as exc:
            what = f"robot_status {robot!r} seq {getattr(msg, 'state_sequence', None)!r}"
            self._log(exc, received_at_ns, what, robot_id=robot)
            return None

    def on_site_event(self, msg, *, received_at_ns):
        """True stored, False identical resend, None refused (and logged)."""
        try:
            return self.ledger.record_site_event(site_event_from_msg(msg))
        except OperationsError as exc:
            self._log(exc, received_at_ns, f"site_event {getattr(msg, 'event_id', None)!r}")
            return None

    def _log(self, exc, received_at_ns, what, robot_id=None, task_id=None):
        detail = f"{what}: {exc.detail_code}: {exc.detail}"[:MAX_DETAIL]
        if robot_id not in self.ledger.allowed_robot_ids:
            robot_id = None  # an unknown robot is the error itself; error_log keeps configured robots only
        self.ledger.record_error(dict(at_ns=received_at_ns, robot_id=robot_id, task_id=task_id, code=exc.code,
                                      detail=detail))
