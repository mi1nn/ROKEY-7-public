"""Recorded RGB-D observation and conservative candidate-event core (E2 only)."""
from .core import (
    Admission, AlignmentEvidence, CameraIntrinsics, CandidateEvent,
    ConservativeEventRegistry, DepthFrame, DepthPolicy, Detection, FrameMeta,
    LocalizationResult, ModelManifest, Observation, ObservationRegistry,
    PerceptionError, Publication, TimePolicy, locate_detection,
    make_observation, validate_freshness,
)

__all__ = [
    "Admission", "AlignmentEvidence", "CameraIntrinsics", "CandidateEvent",
    "ConservativeEventRegistry", "DepthFrame", "DepthPolicy", "Detection", "FrameMeta",
    "LocalizationResult", "ModelManifest", "Observation", "ObservationRegistry",
    "PerceptionError", "Publication", "TimePolicy", "locate_detection",
    "make_observation", "validate_freshness",
]
