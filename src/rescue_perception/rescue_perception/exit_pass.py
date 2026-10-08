"""Fixed camera B: exit-line passes -> SiteEvent(EXIT_PASS) (E2 recorded input only).

No ROS, model, storage or counter. One event per subject crossing from the inside
side to the exit side of a configured image line. Counting and persistence belong
to coordination/operations; mode changes are never issued here.
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns

from .adapter import SourceBinding, rgb_frame, stamp_ns
from .core import Detection, PerceptionError

# Stable namespace: replaying the same recorded frame yields the same event and
# evidence IDs. The producer issues both IDs (contracts C-05); the ledger stores
# the bytes under evidence_id and verifies evidence_sha256.
EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/exit_pass/v1")


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _contract(check, value, label):
    try:
        return check(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _finite(value, name):
    _require(type(value) in (int, float) and math.isfinite(value), "INVALID_INPUT", name)


@dataclass(frozen=True)
class ExitLinePolicy:
    """Exit geometry and association limits for one fixed camera (pixels, ns).

    Deployment values, not chosen here. ``line`` is two image points; the exit side
    is the side where cross(line, point) has the sign ``exit_side``. A point within
    ``band_px`` of the line belongs to neither side (hysteresis against flicker).
    Untracked detections are linked to the nearest live anonymous subject within
    ``match_radius_px`` and ``max_gap_ns``.
    """
    line: tuple[tuple[float, float], tuple[float, float]]
    exit_side: int
    band_px: float
    count_labels: tuple[str, ...]
    min_confidence: float
    match_radius_px: float
    max_gap_ns: int

    def __post_init__(self):
        _require(type(self.line) in (list, tuple) and len(self.line) == 2 and
                 all(type(p) in (list, tuple) and len(p) == 2 for p in self.line), "INVALID_INPUT", "line")
        line = tuple(tuple(p) for p in self.line)
        for value in (*line[0], *line[1]):
            _finite(value, "line coordinate")
        _require(line[0] != line[1], "INVALID_INPUT", "degenerate line")
        object.__setattr__(self, "line", line)
        _require(self.exit_side in (1, -1) and type(self.exit_side) is int, "INVALID_INPUT", "exit_side")
        for name in ("band_px", "match_radius_px"):
            _finite(getattr(self, name), name)
            _require(getattr(self, name) > 0, "INVALID_INPUT", name)
        _require(type(self.count_labels) in (list, tuple) and bool(self.count_labels) and
                 all(isinstance(x, str) and x.strip() for x in self.count_labels), "INVALID_INPUT", "count_labels")
        object.__setattr__(self, "count_labels", tuple(self.count_labels))
        _finite(self.min_confidence, "min_confidence")
        _require(0 <= self.min_confidence <= 1, "INVALID_INPUT", "min_confidence")
        _contract(duration_ns, self.max_gap_ns, "max_gap_ns")
        _require(self.max_gap_ns > 0, "INVALID_INPUT", "max_gap_ns must be positive")


@dataclass(frozen=True)
class FrameOutcome:
    """``status`` is PROCESSED, or DUPLICATE/REORDERED for a frame that changed nothing."""
    status: str
    events: tuple[SiteEvent, ...]


class _Subject:
    __slots__ = ("side", "counted", "last_ns", "anchor")

    def __init__(self, last_ns, anchor):
        self.side, self.counted, self.last_ns, self.anchor = None, False, last_ns, anchor


class ExitPassDetector:
    """One fixed camera, one process. State only moves forward in sensor time.

    A subject (track ID, or an anonymous subject for untracked detections) emits at
    most one EXIT_PASS in its lifetime: it must first be seen clearly on the inside
    side, then clearly on the exit side. Round trips, standing on the line and
    detection flicker add nothing. Frames at or before the last processed stamp are
    ignored. Counted track IDs are retained for the detector's lifetime (no eviction).
    """

    def __init__(self, binding, policy):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, ExitLinePolicy), "INVALID_INPUT", "policy")
        self.camera_id = _contract(camera_id, binding.source_camera, "camera_id")
        self.binding, self.policy = binding, policy
        self._last_ns = None
        self._tracks = {}
        self._anonymous = {}
        self._next_anonymous = 0

    def _side(self, anchor):
        (x0, y0), (x1, y1) = self.policy.line
        dx, dy = x1 - x0, y1 - y0
        signed = (dx * (anchor[1] - y0) - dy * (anchor[0] - x0)) / math.hypot(dx, dy)
        if abs(signed) < self.policy.band_px:
            return None
        return 1 if signed > 0 else -1

    def _anonymous_subject(self, anchor, stamp, claimed):
        best, best_distance = None, None
        for key, subject in self._anonymous.items():
            if key in claimed:
                continue
            distance = math.dist(anchor, subject.anchor)
            if distance <= self.policy.match_radius_px and (best is None or distance < best_distance):
                best, best_distance = key, distance
        if best is None:
            best = self._next_anonymous
            self._next_anonymous += 1
            self._anonymous[best] = _Subject(stamp, anchor)
        claimed.add(best)
        return best, self._anonymous[best]

    def update(self, rgb, detections, detection_header):
        """Process one recorded frame (ROS-shaped Image) and the detections made on it.

        ``detection_header`` must carry this frame's exact stamp and frame_id. The
        time basis (clock domain, stamp semantics, clock state/bound) is copied from
        the binding as-is; UNKNOWN is recorded, not rejected. ``time_basis_declared``
        is only the precondition; ordering and freshness are decided by consumers.
        """
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        _require(stamp_ns(detection_header, "detection") == frame.observed_at_ns and
                 detection_header.frame_id == frame.frame_id, "INVALID_INPUT",
                 "detection header does not match the RGB frame")
        _require(type(detections) in (list, tuple) and all(isinstance(d, Detection) for d in detections),
                 "INVALID_INPUT", "detections")
        tracks = [d.track_id for d in detections if d.tracking_mode == "TRACKING"]
        _require(len(tracks) == len(set(tracks)), "INVALID_INPUT", "track_id repeated within one frame")
        stamp = frame.observed_at_ns
        if self._last_ns is not None and stamp <= self._last_ns:
            return FrameOutcome("DUPLICATE" if stamp == self._last_ns else "REORDERED", ())
        # Validate the whole frame before touching any state (CDX-30-01): a rejected
        # frame leaves no trace, so a corrected retry of the same stamp still counts.
        counted = [d for d in detections
                   if d.class_label in self.policy.count_labels and d.confidence >= self.policy.min_confidence]
        for detection in counted:
            _require(detection.bbox[2] <= frame.width and detection.bbox[3] <= frame.height,
                     "INVALID_INPUT", "bbox outside image")
        self._last_ns = stamp
        # Anonymous subjects past the gap can never be matched again; drop them.
        self._anonymous = {k: v for k, v in self._anonymous.items()
                           if stamp - v.last_ns <= self.policy.max_gap_ns}
        evidence_id = str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame"))
        events, claimed = [], set()
        for detection in counted:
            x0, _, x1, y1 = detection.bbox
            anchor = ((x0 + x1) / 2, y1)  # bottom centre: where the person stands
            if detection.tracking_mode == "TRACKING":
                key = ("track", detection.track_id)
                subject = self._tracks.setdefault(detection.track_id, _Subject(stamp, anchor))
                detail = f"method=TRACK track_id={detection.track_id}"
            else:
                number, subject = self._anonymous_subject(anchor, stamp, claimed)
                key = ("anonymous", number)
                detail = (f"method=HYSTERESIS mode={detection.tracking_mode} subject={number} "
                          f"match_radius_px={self.policy.match_radius_px} max_gap_ns={self.policy.max_gap_ns}")
            subject.last_ns, subject.anchor = stamp, anchor
            side = self._side(anchor)
            if side is None:
                continue
            crossed = subject.side == -self.policy.exit_side and side == self.policy.exit_side
            subject.side = side
            if crossed and not subject.counted:
                subject.counted = True
                event_id = str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/{key[0]}/{key[1]}"))
                events.append(SiteEvent(
                    event_id, self.camera_id, "EXIT_PASS", stamp, evidence_id, evidence_sha256,
                    float(detection.confidence), frame.clock_domain_id, frame.stamp_semantics,
                    self.binding.clock_state, self.binding.clock_error_bound_ns, detail))
        return FrameOutcome("PROCESSED", tuple(events))
