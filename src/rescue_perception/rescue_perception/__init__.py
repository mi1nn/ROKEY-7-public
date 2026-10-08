"""Recorded RGB-D observation core, ROS-shaped input adapter, candidate events (E2 only)."""
from .core import (
    Admission, AlignmentEvidence, CameraIntrinsics, CandidateEvent,
    ConservativeEventRegistry, DepthFrame, DepthPolicy, Detection, FrameMeta,
    LocalizationResult, ModelManifest, Observation, ObservationRegistry,
    PerceptionError, Publication, TimePolicy, locate_detection,
    make_observation, validate_freshness,
)
from .adapter import (
    AdapterResult, ObservationPolicy, SourceBinding, camera_intrinsics, depth_frame,
    detection_from_vision_msgs, observe, rgb_frame, stamp_ns,
)
from .exit_pass import ExitLinePolicy, ExitPassDetector, FrameOutcome

__all__ = [
    "Admission", "AlignmentEvidence", "CameraIntrinsics", "CandidateEvent",
    "ConservativeEventRegistry", "DepthFrame", "DepthPolicy", "Detection", "FrameMeta",
    "LocalizationResult", "ModelManifest", "Observation", "ObservationRegistry",
    "PerceptionError", "Publication", "TimePolicy", "locate_detection",
    "make_observation", "validate_freshness",
    "AdapterResult", "ObservationPolicy", "SourceBinding", "camera_intrinsics", "depth_frame",
    "detection_from_vision_msgs", "observe", "rgb_frame", "stamp_ns",
    "ExitLinePolicy", "ExitPassDetector", "FrameOutcome",
]
