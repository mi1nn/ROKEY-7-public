"""Recorded-input perception contracts; no ROS, model loading, or movement."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import re
from statistics import median
from uuid import UUID


class PerceptionError(ValueError):
    def __init__(self, code, detail):
        self.code = code
        super().__init__(f"{code}: {detail}")


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
    observed_at: float
    clock_domain_id: str
    stamp_semantics: str
    width: int
    height: int
    calibration_id: str

    def __post_init__(self):
        for name in ("frame_id", "clock_domain_id", "calibration_id"):
            _text(getattr(self, name), name)
        _number(self.observed_at, "observed_at", positive=True)
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
    max_stamp_skew_s: float
    min_depth_m: float
    max_depth_m: float
    min_valid_pixels: int
    min_valid_fraction: float
    max_depth_spread_m: float

    def __post_init__(self):
        _number(self.max_stamp_skew_s, "max_stamp_skew_s", minimum=0)
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
        if self.track_id is not None:
            _text(self.track_id, "track_id")


@dataclass(frozen=True)
class LocalizationResult:
    point_optical: tuple[float, float, float] | None
    reason: str
    valid_pixels: int
    alignment_evidence_id: str | None

    def __post_init__(self):
        _text(self.reason, "localization reason")
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
    if abs(rgb.observed_at - depth.meta.observed_at) > policy.max_stamp_skew_s:
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
    max_age_s: float
    max_future_s: float
    max_clock_error_s: float
    sensor_delay_bound_s: float

    def __post_init__(self):
        _number(self.max_age_s, "max_age_s", positive=True)
        for name in ("max_future_s", "max_clock_error_s", "sensor_delay_bound_s"):
            _number(getattr(self, name), name, minimum=0)


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
    valid_until: float
    clock_state: str
    clock_error_bound: float | None
    evidence_id: str
    evidence_sha256: str

    def __post_init__(self):
        _uuid(self.observation_id, "observation_id")
        _uuid(self.source_boot_id, "source_boot_id")
        for name in ("source_robot", "source_camera", "evidence_id"):
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
        _number(self.valid_until, "valid_until", positive=True)
        _require(self.valid_until > self.observed_at, "INVALID_INPUT", "validity interval")
        _require(self.clock_state in ("SYNCED", "UNKNOWN"), "INVALID_INPUT", "clock_state")
        if self.clock_error_bound is not None:
            _number(self.clock_error_bound, "clock_error_bound", minimum=0)

    @property
    def observed_at(self):
        return self.frame.observed_at

    @property
    def payload_hash(self):
        return hashlib.sha256(json.dumps(_canonical(asdict(self)), sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).hexdigest()


def make_observation(**kwargs):
    """Build an immutable source observation. Never replace the FrameMeta stamp."""
    return Observation(**kwargs)


def validate_freshness(observation, *, now, receiver_clock_domain, receiver_clock_error_bound, policy):
    _require(isinstance(observation, Observation) and isinstance(policy, TimePolicy), "INVALID_INPUT", "observation/policy")
    _number(now, "now", positive=True)
    _text(receiver_clock_domain, "receiver_clock_domain")
    _number(receiver_clock_error_bound, "receiver_clock_error_bound", minimum=0)
    _require(observation.frame.clock_domain_id == receiver_clock_domain and
             observation.frame.stamp_semantics != "UNKNOWN" and observation.clock_state == "SYNCED" and
             observation.clock_error_bound is not None, "CLOCK_UNCERTAIN", "time basis")
    uncertainty = observation.clock_error_bound + receiver_clock_error_bound
    _require(uncertainty <= policy.max_clock_error_s, "CLOCK_UNCERTAIN", "clock bound")
    _require(observation.observed_at - now <= policy.max_future_s, "FUTURE_OBSERVATION", "stamp")
    upper_age = now - observation.observed_at + uncertainty + policy.sensor_delay_bound_s
    _require(upper_age <= policy.max_age_s and now + uncertainty + policy.sensor_delay_bound_s < observation.valid_until,
             "STALE_OBSERVATION", "age or original expiry")
    return upper_age


@dataclass(frozen=True)
class Publication:
    observation: Observation
    publisher_id: str
    publisher_boot_id: str
    sequence: int
    published_at: float
    schema_version: str

    def __post_init__(self):
        _require(isinstance(self.observation, Observation), "INVALID_INPUT", "observation")
        _text(self.publisher_id, "publisher_id")
        _uuid(self.publisher_boot_id, "publisher_boot_id")
        _integer(self.sequence, "sequence")
        _require(self.sequence < 2**64, "INVALID_INPUT", "sequence overflow")
        _number(self.published_at, "published_at", positive=True)
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
        self._stream_latest = {}
        self._clock = None
        self._wall = None

    def ingest(self, observation, *, now, received_steady, receiver_clock_domain,
               receiver_clock_error_bound, policy):
        _require(isinstance(observation, Observation), "INVALID_INPUT", "observation")
        _number(received_steady, "received_steady", minimum=0)
        _require(self._clock is None or received_steady >= self._clock, "INVALID_INPUT", "monotonic receive regression")
        _number(now, "now", positive=True)
        _require(self._wall is None or now >= self._wall, "CLOCK_UNCERTAIN", "wall time regressed; reset session")
        previous = self._observations.get(observation.observation_id)
        if previous is not None:
            _require(previous.payload_hash == observation.payload_hash, "ID_CONTENT_CONFLICT", "immutable observation")
        validate_freshness(observation, now=now, receiver_clock_domain=receiver_clock_domain,
                           receiver_clock_error_bound=receiver_clock_error_bound, policy=policy)
        self._clock = received_steady
        self._wall = now
        if previous is not None:
            return Admission("DUPLICATE", previous)
        key = (observation.source_robot, observation.source_camera, observation.source_boot_id)
        latest = self._stream_latest.get(key)
        self._observations[observation.observation_id] = observation
        if latest is not None and observation.observed_at < latest[0]:
            return Admission("REORDERED", observation)
        # Other detections from this same frame are new observations, but cannot
        # refresh the stream gap timer. This preserves multi-person frames.
        if latest is None or observation.observed_at > latest[0]:
            self._stream_latest[key] = (observation.observed_at, received_steady)
        return Admission("ACCEPTED", observation)

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
