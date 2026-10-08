"""Recorded-input perception contracts; no ROS, model loading, or movement."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import re
from statistics import median
from uuid import UUID

from rescue_contracts import (RescueError, canonical_code, duration_ns, robot_id,
                              timestamp_ns)


class PerceptionError(RescueError):
    def __init__(self, reason, detail=""):
        super().__init__(canonical_code("rescue_perception", reason), detail,
                         component="rescue_perception", detail_code=reason)


def _timestamp(value, label):
    try:
        return timestamp_ns(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _duration(value, label):
    try:
        return duration_ns(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _robot(value):
    try:
        return robot_id(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"source_robot: {exc.detail}") from exc


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _text(value, name):
    _require(isinstance(value, str) and bool(value.strip()), "INVALID_INPUT", name)


def _number(value, name, *, minimum=None, positive=False):
    try:
        finite = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        finite = False
    _require(finite, "INVALID_INPUT", name)
    if minimum is not None:
        _require(value >= minimum, "INVALID_INPUT", name)
    if positive:
        _require(value > 0, "INVALID_INPUT", name)


def _integer(value, name, *, minimum=0):
    _require(type(value) is int and value >= minimum, "INVALID_INPUT", name)


def _uuid(value, name):
    _text(value, name)
    try:
        canonical = str(UUID(value))
    except (ValueError, AttributeError) as exc:
        raise PerceptionError("INVALID_INPUT", name) from exc
    _require(value == canonical and UUID(value).int != 0, "INVALID_INPUT", name)


def _sha(value):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
             "INVALID_INPUT", "sha256 must be lowercase hexadecimal")


def _canonical(value):
    """Normalize equivalent JSON number values without touching ID strings."""
    if type(value) is float and value.is_integer():
        return int(value)
    if isinstance(value, dict):
        return {key: _canonical(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    return value


@dataclass(frozen=True)
class ModelManifest:
    model_id: str
    sha256: str
    class_labels: tuple[str, ...]

    def __post_init__(self):
        _text(self.model_id, "model_id")
        _sha(self.sha256)
        _require(type(self.class_labels) in (list, tuple), "INVALID_INPUT", "class_labels")
        object.__setattr__(self, "class_labels", tuple(self.class_labels))
        _require(bool(self.class_labels), "INVALID_INPUT", "empty classes")
        for label in self.class_labels:
            _text(label, "class label")
        _require(len(set(self.class_labels)) == len(self.class_labels), "INVALID_INPUT", "duplicate classes")


@dataclass(frozen=True)
class FrameMeta:
    frame_id: str
    observed_at_ns: int
    clock_domain_id: str
    stamp_semantics: str
    width: int
    height: int
    calibration_id: str

    def __post_init__(self):
        for name in ("frame_id", "clock_domain_id", "calibration_id"):
            _text(getattr(self, name), name)
        _timestamp(self.observed_at_ns, "observed_at_ns")
        _integer(self.width, "width", minimum=1)
        _integer(self.height, "height", minimum=1)
        _require(self.stamp_semantics in ("CAPTURE", "DRIVER_RECEIVE", "UNKNOWN"),
                 "INVALID_INPUT", "stamp_semantics")


@dataclass(frozen=True)
class CameraIntrinsics:
    frame_id: str
    calibration_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    rectified: bool

    def __post_init__(self):
        _text(self.frame_id, "frame_id")
        _text(self.calibration_id, "calibration_id")
        _integer(self.width, "width", minimum=1)
        _integer(self.height, "height", minimum=1)
        _number(self.fx, "fx", positive=True)
        _number(self.fy, "fy", positive=True)
        _number(self.cx, "cx")
        _number(self.cy, "cy")
        _require(type(self.rectified) is bool, "INVALID_INPUT", "rectified")


@dataclass(frozen=True)
class DepthFrame:
    meta: FrameMeta
    encoding: str
    unit: str
    rows: tuple[tuple[float, ...], ...]

    def __post_init__(self):
        _require(isinstance(self.meta, FrameMeta), "INVALID_INPUT", "depth metadata")
        _require((self.encoding, self.unit) in (("16UC1", "mm"), ("32FC1", "m")),
                 "UNSUPPORTED_DEPTH", "encoding/unit pair")
        _require(type(self.rows) in (list, tuple), "INVALID_INPUT", "depth rows")
        _require(all(type(row) in (list, tuple) for row in self.rows), "INVALID_INPUT", "depth row")
        rows = tuple(tuple(row) for row in self.rows)
        _require(len(rows) == self.meta.height and all(len(row) == self.meta.width for row in rows),
                 "INVALID_INPUT", "depth dimensions")
        # E2 recorded-input boundary: full-frame Python validation is intentionally
        # not a realtime adapter. Promotion requires bounded ROI/array ingestion,
        # equivalent encoding/invalid-depth checks, and measured two-stream latency.
        for row in rows:
            for value in row:
                _require(type(value) in (int, float), "INVALID_INPUT", "depth sample type")
                if self.encoding == "32FC1" and type(value) is int:
                    _number(value, "32FC1 integer sample")
                if self.encoding == "16UC1":
                    _require(type(value) is int and 0 <= value <= 65535,
                             "INVALID_INPUT", "16UC1 sample")
        object.__setattr__(self, "rows", rows)


@dataclass(frozen=True)
class AlignmentEvidence:
    evidence_id: str
    calibration_id: str
    rgb_frame_id: str
    depth_frame_id: str
    verified: bool

    def __post_init__(self):
        for name in ("evidence_id", "calibration_id", "rgb_frame_id", "depth_frame_id"):
            _text(getattr(self, name), name)
        _require(type(self.verified) is bool, "INVALID_INPUT", "verified")


@dataclass(frozen=True)
class DepthPolicy:
    max_stamp_skew_ns: int
    min_depth_m: float
    max_depth_m: float
    min_valid_pixels: int
    min_valid_fraction: float
    max_depth_spread_m: float

    def __post_init__(self):
        _duration(self.max_stamp_skew_ns, "max_stamp_skew_ns")
        _number(self.min_depth_m, "min_depth_m", positive=True)
        _number(self.max_depth_m, "max_depth_m", positive=True)
        _require(self.max_depth_m > self.min_depth_m, "INVALID_INPUT", "depth range")
        _integer(self.min_valid_pixels, "min_valid_pixels", minimum=1)
        _number(self.min_valid_fraction, "min_valid_fraction", positive=True)
        _require(self.min_valid_fraction <= 1, "INVALID_INPUT", "valid fraction")
        _number(self.max_depth_spread_m, "max_depth_spread_m", minimum=0)


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    track_id: str | None
    tracking_mode: str

    def __post_init__(self):
        _integer(self.class_id, "class_id")
        _text(self.class_label, "class_label")
        _number(self.confidence, "confidence", minimum=0)
        _require(self.confidence <= 1, "INVALID_INPUT", "confidence")
        _require(type(self.bbox) in (list, tuple) and len(self.bbox) == 4,
                 "INVALID_INPUT", "bbox")
        object.__setattr__(self, "bbox", tuple(self.bbox))
        for value in self.bbox:
            _number(value, "bbox coordinate", minimum=0)
        x0, y0, x1, y1 = self.bbox
        _require(x0 < x1 and y0 < y1, "INVALID_INPUT", "empty bbox")
        _require(self.tracking_mode in ("DETECTION_ONLY", "TRACKING", "TRACKING_FALLBACK"),
                 "INVALID_INPUT", "tracking_mode")
        if self.tracking_mode == "TRACKING":
            _text(self.track_id, "track_id required in TRACKING mode")
        else:
            _require(self.track_id is None, "INVALID_INPUT", "non-tracking mode cannot contain track_id")


@dataclass(frozen=True)
class LocalizationResult:
    point_optical: tuple[float, float, float] | None
    reason: str
    valid_pixels: int
    alignment_evidence_id: str | None

    def __post_init__(self):
        _require(self.reason in ("LOCATED", "CLOCK_UNCERTAIN", "UNSYNCHRONIZED",
                                "ALIGNMENT_UNVERIFIED", "CALIBRATION_MISMATCH",
                                "DIMENSION_MISMATCH", "DEPTH_INSUFFICIENT", "DEPTH_DISCONTINUITY"),
                 "INVALID_INPUT", "localization reason")
        _integer(self.valid_pixels, "valid_pixels")
        if self.alignment_evidence_id is not None:
            _text(self.alignment_evidence_id, "alignment_evidence_id")
        if self.point_optical is not None:
            _require(type(self.point_optical) in (tuple, list) and len(self.point_optical) == 3,
                     "INVALID_INPUT", "optical point")
            object.__setattr__(self, "point_optical", tuple(self.point_optical))
            for value in self.point_optical:
                _number(value, "optical coordinate")
            _number(self.point_optical[2], "optical depth", positive=True)
            _require(self.reason == "LOCATED" and self.valid_pixels > 0 and
                     self.alignment_evidence_id is not None, "INVALID_INPUT", "location provenance")
        else:
            _require(self.reason != "LOCATED", "INVALID_INPUT", "missing optical point")


    @property
    def error_code(self):
        return None if self.reason == "LOCATED" else canonical_code("rescue_perception", self.reason)

    @property
    def detail_code(self):
        return None if self.reason == "LOCATED" else "rescue_perception." + self.reason


def locate_detection(rgb, depth, intrinsics, alignment, detection, policy):
    """Strict registered, rectified, same-resolution ROI median; optical XYZ only.

    Entire integer-enclosing bbox is the ROI. Bounds are rejected, never rescaled.
    No point from this routine is a mission position or a navigation permission.
    """
    for value, cls in ((rgb, FrameMeta), (depth, DepthFrame), (intrinsics, CameraIntrinsics),
                       (alignment, AlignmentEvidence), (detection, Detection), (policy, DepthPolicy)):
        _require(isinstance(value, cls), "INVALID_INPUT", cls.__name__)
    def unavailable(reason, n=0):
        return LocalizationResult(None, reason, n, alignment.evidence_id)
    if rgb.clock_domain_id != depth.meta.clock_domain_id or "UNKNOWN" in (
            rgb.stamp_semantics, depth.meta.stamp_semantics) or rgb.stamp_semantics != depth.meta.stamp_semantics:
        return unavailable("CLOCK_UNCERTAIN")
    if abs(rgb.observed_at_ns - depth.meta.observed_at_ns) > policy.max_stamp_skew_ns:
        return unavailable("UNSYNCHRONIZED")
    if not alignment.verified or not intrinsics.rectified:
        return unavailable("ALIGNMENT_UNVERIFIED")
    if not (alignment.rgb_frame_id == rgb.frame_id == intrinsics.frame_id and
            alignment.depth_frame_id == depth.meta.frame_id and
            alignment.calibration_id == rgb.calibration_id == depth.meta.calibration_id == intrinsics.calibration_id):
        return unavailable("CALIBRATION_MISMATCH")
    if not (rgb.width == depth.meta.width == intrinsics.width and
            rgb.height == depth.meta.height == intrinsics.height):
        return unavailable("DIMENSION_MISMATCH")
    x0, y0, x1, y1 = detection.bbox
    _require(x1 <= rgb.width and y1 <= rgb.height, "INVALID_INPUT", "bbox outside image")
    left, top, right, bottom = math.floor(x0), math.floor(y0), math.ceil(x1), math.ceil(y1)
    scale = 0.001 if depth.encoding == "16UC1" else 1.0
    # E2 ROI list allocation; no realtime throughput claim (see DepthFrame above).
    values = [depth.rows[y][x] * scale for y in range(top, bottom) for x in range(left, right)]
    valid = [value for value in values if math.isfinite(value) and policy.min_depth_m <= value <= policy.max_depth_m]
    if len(valid) < policy.min_valid_pixels or len(valid) / len(values) < policy.min_valid_fraction:
        return unavailable("DEPTH_INSUFFICIENT", len(valid))
    if max(valid) - min(valid) > policy.max_depth_spread_m:
        return unavailable("DEPTH_DISCONTINUITY", len(valid))
    z = median(valid)
    # Pixel centres range from 0 to width-1; bbox edges use [0, width].
    u, v = (x0 + x1) / 2 - 0.5, (y0 + y1) / 2 - 0.5
    point = ((u - intrinsics.cx) * z / intrinsics.fx,
             (v - intrinsics.cy) * z / intrinsics.fy, z)
    return LocalizationResult(point, "LOCATED", len(valid), alignment.evidence_id)


@dataclass(frozen=True)
class TimePolicy:
    max_age_ns: int
    max_future_ns: int
    max_clock_error_ns: int
    stamp_delay_bound_ns: int

    def __post_init__(self):
        _duration(self.max_age_ns, "max_age_ns")
        _require(self.max_age_ns > 0, "INVALID_INPUT", "max_age_ns must be positive")
        for name in ("max_future_ns", "max_clock_error_ns", "stamp_delay_bound_ns"):
            _duration(getattr(self, name), name)


@dataclass(frozen=True)
class Observation:
    observation_id: str
    source_robot: str
    source_camera: str
    source_boot_id: str
    frame: FrameMeta
    model: ModelManifest
    detection: Detection
    localization: LocalizationResult
    valid_until_ns: int
    clock_state: str
    clock_error_bound_ns: int | None
    evidence_id: str
    evidence_sha256: str

    def __post_init__(self):
        _uuid(self.observation_id, "observation_id")
        _uuid(self.source_boot_id, "source_boot_id")
        _robot(self.source_robot)
        for name in ("source_camera", "evidence_id"):
            _text(getattr(self, name), name)
        _sha(self.evidence_sha256)
        for name, cls in (("frame", FrameMeta), ("model", ModelManifest),
                          ("detection", Detection), ("localization", LocalizationResult)):
            _require(isinstance(getattr(self, name), cls), "INVALID_INPUT", name)
        _require(self.detection.class_id < len(self.model.class_labels) and
                 self.model.class_labels[self.detection.class_id] == self.detection.class_label,
                 "MODEL_CLASS_MISMATCH", "class id/label absent from manifest")
        _require(self.detection.bbox[2] <= self.frame.width and self.detection.bbox[3] <= self.frame.height,
                 "INVALID_INPUT", "bbox outside image")
        _timestamp(self.valid_until_ns, "valid_until_ns")
        _require(self.valid_until_ns > self.observed_at_ns, "INVALID_INPUT", "validity interval")
        _require(self.clock_state in ("SYNCED", "UNKNOWN"), "INVALID_INPUT", "clock_state")
        if self.clock_error_bound_ns is not None:
            _duration(self.clock_error_bound_ns, "clock_error_bound_ns")

    @property
    def observed_at_ns(self):
        return self.frame.observed_at_ns

    @property
    def payload_hash(self):
        return hashlib.sha256(json.dumps(_canonical(asdict(self)), sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).hexdigest()


def make_observation(**kwargs):
    """Build an immutable source observation. Never replace the FrameMeta stamp."""
    return Observation(**kwargs)


def validate_freshness(observation, *, now_ns, receiver_clock_domain, receiver_clock_error_bound_ns, policy):
    _require(isinstance(observation, Observation) and isinstance(policy, TimePolicy), "INVALID_INPUT", "observation/policy")
    _timestamp(now_ns, "now_ns")
    _text(receiver_clock_domain, "receiver_clock_domain")
    _duration(receiver_clock_error_bound_ns, "receiver_clock_error_bound_ns")
    _require(observation.frame.clock_domain_id == receiver_clock_domain and
             observation.frame.stamp_semantics != "UNKNOWN" and observation.clock_state == "SYNCED" and
             observation.clock_error_bound_ns is not None, "CLOCK_UNCERTAIN", "time basis")
    uncertainty = observation.clock_error_bound_ns + receiver_clock_error_bound_ns
    _require(uncertainty <= policy.max_clock_error_ns, "CLOCK_UNCERTAIN", "clock bound")
    _require(observation.observed_at_ns - now_ns <= policy.max_future_ns, "FUTURE_OBSERVATION", "stamp")
    upper_age = now_ns - observation.observed_at_ns + uncertainty + policy.stamp_delay_bound_ns
    _require(upper_age <= policy.max_age_ns and now_ns + uncertainty + policy.stamp_delay_bound_ns < observation.valid_until_ns,
             "STALE_OBSERVATION", "age or original expiry")
    return upper_age


@dataclass(frozen=True)
class Publication:
    observation: Observation
    publisher_id: str
    publisher_boot_id: str
    sequence: int
    published_at_ns: int
    schema_version: str

    def __post_init__(self):
        _require(isinstance(self.observation, Observation), "INVALID_INPUT", "observation")
        _text(self.publisher_id, "publisher_id")
        _uuid(self.publisher_boot_id, "publisher_boot_id")
        _integer(self.sequence, "sequence")
        _require(self.sequence < 2**64, "INVALID_INPUT", "sequence overflow")
        _timestamp(self.published_at_ns, "published_at_ns")
        _require(self.schema_version == "1.0", "INCOMPATIBLE_SCHEMA", "only 1.0 implemented")


@dataclass(frozen=True)
class Admission:
    status: str
    observation: Observation

    def __post_init__(self):
        _require(self.status in ("ACCEPTED", "DUPLICATE", "REORDERED") and
                 isinstance(self.observation, Observation), "INVALID_INPUT", "admission")


class ObservationRegistry:
    """One receiving process; duplicates/reordering never refresh stream timers.

    IDs retained for this registry's lifetime. A durable adapter must restore the
    dedup ledger before accepting replay after restart. No eviction is implicit.
    """
    def __init__(self):
        self._observations = {}
        self._history = []
        self._stream_latest = {}
        self._clock = None
        self._wall = None
        self._clock_fault_latched = False

    def ingest(self, observation, *, now_ns, received_steady, receiver_clock_domain,
               receiver_clock_error_bound_ns, policy):
        _require(isinstance(observation, Observation), "INVALID_INPUT", "observation")
        _number(received_steady, "received_steady", minimum=0)
        _timestamp(now_ns, "now_ns")
        if ((self._clock is not None and received_steady < self._clock) or
                (self._wall is not None and now_ns < self._wall)):
            self._clock_fault_latched = True
        _require(not self._clock_fault_latched, "CLOCK_UNCERTAIN",
                 "receiver time regressed; recover clocks and create a new receiver session")
        # Receiver clock readings are independent of payload admission. Even a
        # rejected stale/colliding message must not hide a later clock rollback.
        self._clock = received_steady
        self._wall = now_ns
        previous = self._observations.get(observation.observation_id)
        if previous is not None:
            _require(previous.payload_hash == observation.payload_hash, "ID_CONTENT_CONFLICT", "immutable observation")
        validate_freshness(observation, now_ns=now_ns, receiver_clock_domain=receiver_clock_domain,
                           receiver_clock_error_bound_ns=receiver_clock_error_bound_ns, policy=policy)
        if previous is not None:
            return Admission("DUPLICATE", previous)
        key = (observation.source_robot, observation.source_camera, observation.source_boot_id)
        latest = self._stream_latest.get(key)
        self._observations[observation.observation_id] = observation
        if latest is not None and observation.observed_at_ns < latest[0]:
            admission = Admission("REORDERED", observation)
            self._history.append(admission)
            return admission
        # Other detections from this same frame are new observations, but cannot
        # refresh the stream gap timer. This preserves multi-person frames.
        if latest is None or observation.observed_at_ns > latest[0]:
            self._stream_latest[key] = (observation.observed_at_ns, received_steady)
        admission = Admission("ACCEPTED", observation)
        self._history.append(admission)
        return admission

    def history(self):
        """Immutable intake history, including reordered discoveries; no event authorization."""
        return tuple(self._history)

    def last_new_receive(self, source_robot, source_camera, source_boot_id):
        entry = self._stream_latest.get((source_robot, source_camera, source_boot_id))
        return None if entry is None else entry[1]


@dataclass(frozen=True)
class CandidateEvent:
    event_id: str
    observation_id: str
    candidate_event_ids: tuple[str, ...]
    status: str = "CANDIDATE"


class ConservativeEventRegistry:
    """Retain separate candidates; associations need later geometric confirmation.

    Only ObservationRegistry ACCEPTED admissions are eligible. This module never
    concludes same-person identity from track number, class, or confidence.
    """
    def __init__(self):
        self._by_observation = {}
        self._events = {}
        self._hashes = {}

    def register(self, admission, *, event_id, candidate_event_ids=()):
        _require(isinstance(admission, Admission), "INVALID_INPUT", "validated admission required")
        _uuid(event_id, "event_id")
        _require(type(candidate_event_ids) in (list, tuple), "INVALID_INPUT", "candidate_event_ids")
        candidates = tuple(candidate_event_ids)
        for candidate in candidates:
            _uuid(candidate, "candidate_event_id")
            _require(candidate in self._events, "INVALID_INPUT", "unknown candidate event")
        _require(len(set(candidates)) == len(candidates) and event_id not in candidates,
                 "INVALID_INPUT", "candidate references")
        existing = self._by_observation.get(admission.observation.observation_id)
        if existing is not None:
            _require(self._hashes[existing.observation_id] == admission.observation.payload_hash and
                     existing.event_id == event_id and existing.candidate_event_ids == candidates,
                     "ID_CONTENT_CONFLICT", "event registration")
            return existing
        _require(admission.status == "ACCEPTED", "STALE_OBSERVATION", "not a new stream observation")
        _require(event_id not in self._events, "ID_CONTENT_CONFLICT", "event_id already used")
        event = CandidateEvent(event_id, admission.observation.observation_id, candidates)
        self._events[event_id] = event
        self._by_observation[event.observation_id] = event
        self._hashes[event.observation_id] = admission.observation.payload_hash
        return event
