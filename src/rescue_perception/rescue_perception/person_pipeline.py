"""Robot camera person pipeline: detector interface -> RGB-D localization -> Observation.

E2 only. The model sits behind ``Detector`` (Q-18 open: no weights or runtime are
chosen here). ``RecordedDetections`` replays detections recorded per frame stamp,
which is how a bag or a labelled recording drives the pipeline without a model.
Each person detection becomes an Observation on every frame (stream freshness,
LL-06) and is classified CONFIRMED/UNCONFIRMED by ``ConfirmationPolicy`` (Q-21
values, no defaults). Within one camera stream a track is reported as a new
candidate once, and again only when it upgrades to CONFIRMED; merging candidates
across observations is coordination's TargetEstimate, not done here.
"""
from dataclasses import dataclass
import math
from typing import Protocol
from uuid import NAMESPACE_URL, uuid5

from .adapter import ObservationPolicy, SourceBinding, observe, stamp_ns
from .core import (AlignmentEvidence, DepthPolicy, Detection, ModelManifest, Observation,
                   PerceptionError)

NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/person_pipeline/v1")
TRACKING_MODES = ("DETECTION_ONLY", "TRACKING", "TRACKING_FALLBACK")


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


@dataclass(frozen=True)
class DetectorOutput:
    """One raw model output on one frame. ``track_id`` only in TRACKING mode."""
    class_label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    track_id: str | None


@dataclass(frozen=True)
class Inference:
    """What a detector ran on a frame. Empty ``outputs`` means "ran, found nothing"."""
    model: ModelManifest
    tracking_mode: str
    outputs: tuple[DetectorOutput, ...]

    def __post_init__(self):
        _require(isinstance(self.model, ModelManifest), "INVALID_INPUT", "model")
        _require(self.tracking_mode in TRACKING_MODES, "INVALID_INPUT", "tracking_mode")
        _require(type(self.outputs) in (list, tuple) and all(isinstance(o, DetectorOutput) for o in self.outputs),
                 "INVALID_INPUT", "outputs")
        object.__setattr__(self, "outputs", tuple(self.outputs))


class Detector(Protocol):
    def infer(self, rgb, observed_at_ns: int) -> Inference | None:
        """Return the inference for this frame, or None if the model did not run on it."""


class RecordedDetections:
    """Detections recorded per exact frame stamp (int ns). Missing stamp -> None (not run)."""

    def __init__(self, model, tracking_mode, by_stamp):
        _require(type(by_stamp) is dict and all(type(k) is int for k in by_stamp), "INVALID_INPUT",
                 "by_stamp keys must be int ns stamps")
        self._inferences = {stamp: Inference(model, tracking_mode, outputs) for stamp, outputs in by_stamp.items()}

    def infer(self, rgb, observed_at_ns):
        return self._inferences.get(observed_at_ns)


@dataclass(frozen=True)
class ConfirmationPolicy:
    """Q-21 values from the deployment. CONFIRMED needs both, never one alone."""
    person_labels: tuple[str, ...]
    confirm_confidence_min: float

    def __post_init__(self):
        _require(type(self.person_labels) in (list, tuple) and bool(self.person_labels) and
                 all(isinstance(x, str) and x.strip() for x in self.person_labels), "INVALID_INPUT", "person_labels")
        object.__setattr__(self, "person_labels", tuple(self.person_labels))
        _require(type(self.confirm_confidence_min) in (int, float) and math.isfinite(self.confirm_confidence_min)
                 and 0 <= self.confirm_confidence_min <= 1, "INVALID_INPUT", "confirm_confidence_min")


@dataclass(frozen=True)
class PersonCandidate:
    """``status`` CONFIRMED (confidence and position both pass) or UNCONFIRMED with reasons."""
    observation: Observation
    status: str
    reasons: tuple[str, ...]
    new_candidate: bool


@dataclass(frozen=True)
class FrameResult:
    """``status``: PROCESSED, NOT_INFERRED (model did not run), or DUPLICATE/REORDERED."""
    status: str
    candidates: tuple[PersonCandidate, ...]


class PersonPipeline:
    """One robot camera stream in one process."""

    def __init__(self, *, binding, alignment, rectified, depth_policy, observation_policy,
                 confirmation_policy, detector):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is not None, "INVALID_INPUT",
                 "robot camera binding requires source_robot")
        for value, cls in ((alignment, AlignmentEvidence), (depth_policy, DepthPolicy),
                           (observation_policy, ObservationPolicy), (confirmation_policy, ConfirmationPolicy)):
            _require(isinstance(value, cls), "INVALID_INPUT", cls.__name__)
        _require(type(rectified) is bool and callable(getattr(detector, "infer", None)), "INVALID_INPUT",
                 "rectified/detector")
        self.binding, self.alignment, self.rectified = binding, alignment, rectified
        self.depth_policy, self.observation_policy = depth_policy, observation_policy
        self.confirmation, self.detector = confirmation_policy, detector
        self._last_ns = None
        self._reported = {}  # track_id -> best status already reported as a candidate

    def process(self, rgb, depth, camera_info):
        stamp = stamp_ns(rgb.header, "rgb")
        if self._last_ns is not None and stamp <= self._last_ns:
            return FrameResult("DUPLICATE" if stamp == self._last_ns else "REORDERED", ())
        inference = self.detector.infer(rgb, stamp)
        if inference is None:
            return FrameResult("NOT_INFERRED", ())
        _require(isinstance(inference, Inference), "INVALID_INPUT", "detector must return Inference or None")
        self._last_ns = stamp
        candidates = []
        for index, output in enumerate(inference.outputs):
            if output.class_label not in self.confirmation.person_labels:
                continue
            track = output.track_id if inference.tracking_mode == "TRACKING" else None
            _require(inference.tracking_mode == "TRACKING" or output.track_id is None, "INVALID_INPUT",
                     "track_id outside TRACKING mode")
            _require(output.class_label in inference.model.class_labels, "MODEL_CLASS_MISMATCH", output.class_label)
            detection = Detection(inference.model.class_labels.index(output.class_label), output.class_label,
                                  output.confidence, output.bbox, track, inference.tracking_mode)
            key = f"{self.binding.source_boot_id}/{stamp}/{index}"
            result = observe(rgb, depth, camera_info, detection, rgb.header, binding=self.binding,
                             alignment=self.alignment, rectified=self.rectified, model=inference.model,
                             depth_policy=self.depth_policy, observation_policy=self.observation_policy,
                             observation_id=str(uuid5(NAMESPACE, key)),
                             evidence_id=str(uuid5(NAMESPACE, f"{self.binding.source_boot_id}/{stamp}/frame")))
            reasons = []
            if output.confidence < self.confirmation.confirm_confidence_min:
                reasons.append("LOW_CONFIDENCE")
            if result.localization.reason != "LOCATED":
                reasons.append(result.localization.reason)
            status = "UNCONFIRMED" if reasons else "CONFIRMED"
            candidates.append(PersonCandidate(result.observation, status, tuple(reasons),
                                              self._is_new(track, status)))
        return FrameResult("PROCESSED", tuple(candidates))

    def _is_new(self, track, status):
        """Untracked detections cannot be linked across frames: each is a new candidate."""
        if track is None:
            return True
        previous = self._reported.get(track)
        if previous == "CONFIRMED" or previous == status:
            return False
        self._reported[track] = status
        return True
