"""Recorded RGB-D input adapter: ROS-shaped messages -> perception core (E2 only).

No ROS import. Inputs are objects with the field names of ``sensor_msgs/Image``,
``sensor_msgs/CameraInfo`` and ``vision_msgs/Detection2D`` (rclpy messages, bag
deserialized messages, or test doubles). Source identity, stamp meaning,
calibration and alignment evidence are injected by the caller; nothing here
guesses them from topic names or message contents.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import struct

from rescue_contracts import RescueError, duration_ns, robot_id, ros_time_to_ns, uuid_string

from .core import (AlignmentEvidence, CameraIntrinsics, DepthFrame, DepthPolicy, Detection,
                   FrameMeta, LocalizationResult, ModelManifest, Observation, PerceptionError,
                   locate_detection, make_observation)

# encoding -> (core unit, struct code, bytes per sample). Only the two encodings the
# core accepts; anything else (mono16, compressedDepth, 8-bit) is rejected, not converted.
DEPTH_ENCODINGS = {"16UC1": ("mm", "H", 2), "32FC1": ("m", "f", 4)}
TRACKING_MODES = ("DETECTION_ONLY", "TRACKING", "TRACKING_FALLBACK")


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _text(value, name):
    _require(isinstance(value, str) and bool(value.strip()), "INVALID_INPUT", name)


def _contract(check, value, label):
    """Apply a shared rescue_contracts validator; report it as a perception input error."""
    try:
        return check(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


@dataclass(frozen=True)
class SourceBinding:
    """Deployment facts for one camera, supplied from outside the adapter.

    ``source_robot`` is a profile robot ID for a robot camera, or ``None`` for any
    other source (e.g. a fixed camera). The current Observation contract only has
    ``source_robot``; a non-robot source therefore stops at LocalizationResult until
    the shared contract defines its identity. Stamp semantics are per stream because
    RGB and Depth may be stamped differently; ``UNKNOWN`` is the honest default.
    """
    source_robot: str | None
    source_camera: str
    source_boot_id: str
    clock_domain_id: str
    rgb_stamp_semantics: str
    depth_stamp_semantics: str
    calibration_id: str
    clock_state: str
    clock_error_bound_ns: int | None

    def __post_init__(self):
        if self.source_robot is not None:
            _contract(robot_id, self.source_robot, "source_robot")
        for name in ("source_camera", "clock_domain_id", "calibration_id"):
            _text(getattr(self, name), name)
        _contract(uuid_string, self.source_boot_id, "source_boot_id")
        for name in ("rgb_stamp_semantics", "depth_stamp_semantics"):
            _require(getattr(self, name) in ("CAPTURE", "DRIVER_RECEIVE", "UNKNOWN"),
                     "INVALID_INPUT", name)
        _require(self.clock_state in ("SYNCED", "UNKNOWN"), "INVALID_INPUT", "clock_state")
        if self.clock_error_bound_ns is not None:
            _contract(duration_ns, self.clock_error_bound_ns, "clock_error_bound_ns")


@dataclass(frozen=True)
class ObservationPolicy:
    """How long a source observation stays valid after its own sensor stamp.

    Deployment value, not chosen here (T-01 family, docs/process/acceptance-plan.md).
    """
    validity_ns: int

    def __post_init__(self):
        _contract(duration_ns, self.validity_ns, "validity_ns")
        _require(self.validity_ns > 0, "INVALID_INPUT", "validity_ns must be positive")


@dataclass(frozen=True)
class AdapterResult:
    """Localization is always present; observation is None only for a non-robot source."""
    localization: LocalizationResult
    observation: Observation | None
    observation_blocked: str | None

    def __post_init__(self):
        _require((self.observation is None) == (self.observation_blocked is not None),
                 "INVALID_INPUT", "observation xor blocked reason")


def stamp_ns(header, label):
    """Exact int64 ns from ``header.stamp.sec/nanosec``; never through float seconds."""
    try:
        return ros_time_to_ns(header.stamp.sec, header.stamp.nanosec)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label} stamp: {exc.detail}") from exc


def _dimensions(msg, label):
    _require(type(msg.width) is int and type(msg.height) is int and msg.width > 0 and msg.height > 0,
             "INVALID_INPUT", f"{label} dimensions")


def frame_meta(image, binding, stamp_semantics, label):
    _dimensions(image, label)
    _text(image.header.frame_id, f"{label} frame_id")
    return FrameMeta(image.header.frame_id, stamp_ns(image.header, label), binding.clock_domain_id,
                     stamp_semantics, image.width, image.height, binding.calibration_id)


def _payload(image, label):
    _require(isinstance(image.data, (bytes, bytearray, memoryview)) or hasattr(image.data, "tobytes"),
             "INVALID_INPUT", f"{label} data must be bytes-like")
    data = image.data.tobytes() if hasattr(image.data, "tobytes") else bytes(image.data)
    _require(type(image.step) is int and len(data) == image.step * image.height,
             "INVALID_INPUT", f"{label} data length != step * height")
    return data


def rgb_frame(image, binding):
    """RGB metadata plus SHA-256 of the exact pixel payload (observation evidence)."""
    meta = frame_meta(image, binding, binding.rgb_stamp_semantics, "rgb")
    return meta, hashlib.sha256(_payload(image, "rgb")).hexdigest()


def depth_frame(image, binding):
    """Decode a raw 16UC1/32FC1 Image honouring endianness and row padding.

    E2 path: the result is a full-frame DepthFrame that the core validates pixel by
    pixel in Python. It is not a realtime ingestion path.
    """
    _require(image.encoding in DEPTH_ENCODINGS, "UNSUPPORTED_DEPTH", f"depth encoding {image.encoding!r}")
    unit, code, size = DEPTH_ENCODINGS[image.encoding]
    _require(image.is_bigendian in (0, 1, False, True), "INVALID_INPUT", "is_bigendian")
    meta = frame_meta(image, binding, binding.depth_stamp_semantics, "depth")
    data = _payload(image, "depth")
    _require(image.step >= image.width * size, "INVALID_INPUT", "depth step shorter than row")
    row = struct.Struct((">" if image.is_bigendian else "<") + code * image.width)
    rows = tuple(row.unpack_from(data, y * image.step) for y in range(image.height))
    return DepthFrame(meta, image.encoding, unit, rows)


def camera_intrinsics(info, binding, *, rectified):
    """Pinhole intrinsics for the image the detector actually saw.

    ``rectified`` is the caller's evidence that the subscribed image is rectified
    (e.g. an ``image_rect`` stream); it is not inferred from ``d``. Rectified images
    use the projection matrix ``p`` (ROS CameraInfo convention); a non-zero ``p``
    translation (secondary stereo camera) is rejected. Unrectified images keep ``k``
    and the core then refuses to localize (ALIGNMENT_UNVERIFIED).
    """
    _require(type(rectified) is bool, "INVALID_INPUT", "rectified")
    _dimensions(info, "camera_info")
    _text(info.header.frame_id, "camera_info frame_id")
    _require(info.binning_x in (0, 1) and info.binning_y in (0, 1), "INVALID_INPUT",
             "binned CameraInfo unsupported")
    roi = info.roi
    _require((roi.x_offset, roi.y_offset, roi.width, roi.height) in
             ((0, 0, 0, 0), (0, 0, info.width, info.height)), "INVALID_INPUT", "CameraInfo ROI unsupported")
    undistort = undistortion_id = None
    if rectified:
        p = tuple(info.p)
        _require(len(p) == 12 and p[3] == 0 and p[7] == 0 and p[11] == 0, "INVALID_INPUT",
                 "rectified projection must be a primary camera (zero translation)")
        fx, cx, fy, cy = p[0], p[2], p[5], p[6]
    else:
        k = tuple(info.k)
        _require(len(k) == 9, "INVALID_INPUT", "camera_info k")
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        prepared = _point_undistorter(info)
        if prepared is not None:   # box points go onto the P grid, where the aligned depth is
            undistort, undistortion_id, (fx, cx, fy, cy) = prepared
    return CameraIntrinsics(info.header.frame_id, binding.calibration_id, info.width, info.height,
                            float(fx), float(fy), float(cx), float(cy), rectified, undistort, undistortion_id)


# CameraInfo distortion models OpenCV undistortPoints takes, with their coefficient counts.
UNDISTORT_MODELS = {"plumb_bob": (5,), "rational_polynomial": (8,)}


def _point_undistorter(info):
    """(undistort(points), "<model>:<sha12(D)>", P intrinsics) for a supported model, else None.

    Unsupported models (e.g. equidistant) or malformed D/P stay unrectified: the core then refuses
    to localize (ALIGNMENT_UNVERIFIED). OpenCV/NumPy are imported only here.
    """
    model, d, p = info.distortion_model, tuple(info.d), tuple(info.p)
    if model not in UNDISTORT_MODELS or len(d) not in UNDISTORT_MODELS[model]:
        return None
    if len(p) != 12 or p[3] != 0 or p[7] != 0 or p[11] != 0 or not all(math.isfinite(x) for x in d + p):
        return None
    import cv2
    import numpy as np
    k_matrix = np.array(info.k, dtype=np.float64).reshape(3, 3)
    d_vector = np.array(d, dtype=np.float64)
    p_matrix = np.array(p, dtype=np.float64).reshape(3, 4)

    def undistort(points):
        raw = np.array([list(points)], dtype=np.float64).reshape(-1, 1, 2)
        return [tuple(float(c) for c in row) for row in cv2.undistortPoints(raw, k_matrix, d_vector,
                                                                              P=p_matrix).reshape(-1, 2)]
    digest = hashlib.sha256(json.dumps([model, list(d)]).encode()).hexdigest()[:12]
    return undistort, f"{model}:{digest}", (p[0], p[2], p[5], p[6])


def detection_from_vision_msgs(det, model, *, tracking_mode):
    """``vision_msgs/Detection2D`` -> core Detection with an explicit tracking mode.

    Exactly one hypothesis is required (no silent best-of). ``det.id`` is the track
    ID in TRACKING mode and must be empty otherwise, so an untracked detector cannot
    look tracked and a tracker ID is never dropped quietly.
    """
    _require(isinstance(model, ModelManifest), "INVALID_INPUT", "model manifest")
    _require(tracking_mode in TRACKING_MODES, "INVALID_INPUT", "tracking_mode")
    _require(len(det.results) == 1, "INVALID_INPUT", "exactly one hypothesis per detection")
    hypothesis = det.results[0].hypothesis
    label = hypothesis.class_id
    _require(label in model.class_labels, "MODEL_CLASS_MISMATCH", f"label {label!r} not in manifest")
    track = det.id if det.id != "" else None
    _require(tracking_mode == "TRACKING" or track is None, "INVALID_INPUT",
             "detection id present outside TRACKING mode")
    centre, sx, sy = det.bbox.center.position, det.bbox.size_x, det.bbox.size_y
    bbox = (centre.x - sx / 2, centre.y - sy / 2, centre.x + sx / 2, centre.y + sy / 2)
    return Detection(model.class_labels.index(label), label, hypothesis.score, bbox,
                     track if tracking_mode == "TRACKING" else None, tracking_mode)


def observe(rgb, depth, camera_info, detection, detection_header, *, binding, alignment, rectified,
            model, depth_policy, observation_policy, observation_id, evidence_id):
    """One recorded RGB/Depth/CameraInfo/detection tuple -> localization (+ observation).

    ``detection_header`` must carry the RGB frame's exact stamp and frame_id, so a box
    from another frame cannot be paired with this image (LL-06). Localization failures
    are kept as Observations without a point (discovery is not discarded). Malformed
    input raises PerceptionError; it never becomes a position.
    """
    for value, cls in ((binding, SourceBinding), (alignment, AlignmentEvidence), (model, ModelManifest),
                       (depth_policy, DepthPolicy), (observation_policy, ObservationPolicy),
                       (detection, Detection)):
        _require(isinstance(value, cls), "INVALID_INPUT", cls.__name__)
    rgb_meta, evidence_sha256 = rgb_frame(rgb, binding)
    _require(stamp_ns(detection_header, "detection") == rgb_meta.observed_at_ns and
             detection_header.frame_id == rgb_meta.frame_id, "INVALID_INPUT",
             "detection header does not match the RGB frame")
    localization = locate_detection(rgb_meta, depth_frame(depth, binding),
                                    camera_intrinsics(camera_info, binding, rectified=rectified),
                                    alignment, detection, depth_policy)
    if binding.source_robot is None:
        return AdapterResult(localization, None, "SOURCE_IDENTITY_UNDEFINED")
    observation = make_observation(
        observation_id=observation_id, source_robot=binding.source_robot,
        source_camera=binding.source_camera, source_boot_id=binding.source_boot_id, frame=rgb_meta,
        model=model, detection=detection, localization=localization,
        valid_until_ns=rgb_meta.observed_at_ns + observation_policy.validity_ns,
        clock_state=binding.clock_state, clock_error_bound_ns=binding.clock_error_bound_ns,
        evidence_id=evidence_id, evidence_sha256=evidence_sha256)
    return AdapterResult(localization, observation, None)
