"""AMR RGB-D settings, YOLO observations and depth decoding/calibration.

Map projection is kept separately in map_projection.py for later integration.
"""
from dataclasses import dataclass, replace
import math
import hashlib
import json
import re
import struct
from pathlib import Path
from importlib.resources import files

from .core import AlignmentEvidence, DepthPolicy
from .person_pipeline import ConfirmationPolicy, DetectorOutput, Inference
from .adapter import ObservationPolicy, SourceBinding
from .camera_utils import SETTINGS as CAMERA_SETTINGS, validate_config


# User settings, 2026-10-12: share RGB settings and always select robot6 correction.
# UNKNOWN clock/alignment values remain deliberate until measured.
SETTINGS = {
    **CAMERA_SETTINGS,
    'depth_topic': 'camera/depth/image_raw/compressedDepth',
    'depth_transport': 'compressedDepth',
    'depth_byte_order': 'little',
    'camera_info_topic': 'camera/camera_info',
    'observations_topic': '/rescue/observations',
    'robot_id': 'robot_a',
    'physical_robot_id': 'robot6',
    'depth_calibration_path': 'package:robot6_depth.json',
    'camera_id': 'amr_rgbd',
    'model_id': 'survivor_yolo26n_640',
    'confirm_confidence_min': 0.8,
    'rgb_optical_frame': 'camera_rgb_optical',
    'depth_optical_frame': 'camera_rgb_optical',
    'calibration_id': 'UNCONFIGURED',
    'alignment_evidence_id': 'UNCONFIGURED',
    'rgb_rectified': False,
    'rgb_depth_alignment_verified': False,
    'rgb_stamp_semantics': 'UNKNOWN',
    'depth_stamp_semantics': 'UNKNOWN',
    'clock_domain_id': 'UNCONFIGURED',
    'clock_state': 'UNKNOWN',
    'clock_error_bound_ns': -1,
    'publisher_id': 'amr.perception',
    'envelope_version': '0.1',
    'sync_queue_size': 3,
    'sync_slop_ns': 20000000,
    'camera_info_max_age_ns': 1000000000,
    'observation_validity_ns': 2000000000,
    'min_depth_m': 0.2,
    'max_depth_m': 5.0,
    'min_valid_pixels': 10,
    'min_valid_fraction': 0.5,
    'max_depth_spread_m': 0.5,
    'sensor_variance_xyz': [0.04, 0.04, 0.04],
}

def target_config(v, boot_id):
    validate_config(v)
    for key, default in SETTINGS.items():
        if key not in v or type(v[key]) is not type(default):
            raise ValueError(f'missing/invalid setting: {key}')
        if isinstance(v[key], str) and not v[key].strip():
            raise ValueError(f'empty setting: {key}')
    if v['depth_transport'] not in ('raw', 'compressedDepth') or v['depth_byte_order'] not in ('little', 'big'):
        raise ValueError('depth_transport must be raw/compressedDepth; specify publisher byte order')
    for key in ('sync_queue_size', 'camera_info_max_age_ns', 'observation_validity_ns'):
        if v[key] <= 0:
            raise ValueError(f'{key} must be positive')
    topics = [v[key] for key in ('image_topic', 'depth_topic', 'camera_info_topic', 'annotated_topic',
                                 'observations_topic')]
    if len(topics) != len(set(topics)):
        raise ValueError('all input/output topics must differ')
    bound = v['clock_error_bound_ns']
    if bound < -1:
        raise ValueError('clock_error_bound_ns must be >=0 or -1 (UNKNOWN)')
    binding = SourceBinding(v['robot_id'], v['camera_id'], boot_id, v['clock_domain_id'],
        v['rgb_stamp_semantics'], v['depth_stamp_semantics'], v['calibration_id'], v['clock_state'],
        bound if bound >= 0 else None)
    alignment = AlignmentEvidence(v['alignment_evidence_id'], v['calibration_id'],
        v['rgb_optical_frame'], v['depth_optical_frame'], v['rgb_depth_alignment_verified'])
    depth = DepthPolicy(v['sync_slop_ns'], v['min_depth_m'], v['max_depth_m'],
                        v['min_valid_pixels'], v['min_valid_fraction'], v['max_depth_spread_m'])
    if v['depth_calibration_path'] != 'UNCONFIGURED':
        calibration = load_depth_calibration(v['depth_calibration_path'], physical_robot_id=v['physical_robot_id'])
        depth = replace(depth, scale=calibration.scale, offset_m=calibration.offset_m)
        calibration_id = f'{binding.calibration_id}|depth:{calibration.calibration_id}:{calibration.sha256}'
        binding = replace(binding, calibration_id=calibration_id)
        alignment = replace(alignment, calibration_id=calibration_id)
    expiry = ObservationPolicy(v['observation_validity_ns'])
    confirmation = ConfirmationPolicy((v['model_label'],), v['confirm_confidence_min'])
    if v['confirm_confidence_min'] < v['model_conf']:
        raise ValueError('confirm_confidence_min must not be below model_conf')
    if v['envelope_version'] != '0.1':
        raise ValueError('only the current ROS wire envelope version 0.1 is supported')
    covariance = covariance_matrix(v['sensor_variance_xyz'])
    return binding, alignment, depth, expiry, confirmation, covariance


def inference_from_result(result, manifest):
    if result.boxes is None:
        raise ValueError('YOLO result has no detection boxes')
    xyxy = result.boxes.xyxy.tolist()
    scores = result.boxes.conf.tolist()
    classes = result.boxes.cls.tolist()
    if not len(xyxy) == len(scores) == len(classes):
        raise ValueError('YOLO box/score/class lengths differ')
    outputs = []
    for bbox, score, class_id in zip(xyxy, scores, classes):
        if not math.isfinite(class_id) or int(class_id) != class_id or not 0 <= int(class_id) < len(manifest.class_labels):
            raise ValueError('YOLO class ID outside model manifest')
        outputs.append(DetectorOutput(manifest.class_labels[int(class_id)], float(score), tuple(bbox), None))
    return Inference(manifest, 'DETECTION_ONLY', tuple(outputs))


class PreparedDetector:
    """One already-run inference for the exact RGB stamp; no hidden second inference."""
    def __init__(self):
        self.stamp = self.inference = None
    def prepare(self, stamp, inference):
        self.stamp, self.inference = stamp, inference
    def infer(self, rgb, observed_at_ns):
        return self.inference if observed_at_ns == self.stamp else None


def covariance_matrix(diagonal):
    if len(diagonal) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in diagonal):
        raise ValueError('three finite positive variances required (m^2)')
    return (float(diagonal[0]), 0., 0., 0., float(diagonal[1]), 0., 0., 0., float(diagonal[2]))


def fill_envelope(message, *, version, publisher_id, boot_id, sequence, now_ns):
    message.schema_version = version
    message.publisher_id = publisher_id
    message.publisher_boot_id = boot_id
    message.sequence = sequence
    message.published_at_ns = now_ns


def fill_observation(message, observation, covariance):
    o = observation
    message.observation_id = o.observation_id
    message.source_robot, message.source_camera, message.source_boot_id = o.source_robot, o.source_camera, o.source_boot_id
    message.time.observed_at_ns, message.time.valid_until_ns = o.observed_at_ns, o.valid_until_ns
    message.time.stamp_semantics, message.time.clock_domain_id = o.frame.stamp_semantics, o.frame.clock_domain_id
    message.time.clock_state = o.clock_state
    message.time.has_clock_error_bound = o.clock_error_bound_ns is not None
    message.time.clock_error_bound_ns = o.clock_error_bound_ns if o.clock_error_bound_ns is not None else 0
    message.class_label, message.confidence = o.detection.class_label, float(o.detection.confidence)
    message.model_id, message.model_sha256 = o.model.model_id, o.model.sha256
    message.tracking_mode, message.has_track_id = o.detection.tracking_mode, o.detection.track_id is not None
    message.track_id = o.detection.track_id or ''
    message.bbox_xyxy_pixels = list(float(x) for x in o.detection.bbox)
    message.sensor_frame = o.frame.frame_id
    message.localization_status = 'VALID' if o.localization.point_optical is not None else 'UNKNOWN'
    # UNKNOWN uses nonfinite sentinels; the status is mandatory, never a valid origin.
    point = o.localization.point_optical or (math.nan, math.nan, math.nan)
    message.sensor_position.x, message.sensor_position.y, message.sensor_position.z = point
    message.position_covariance = list(covariance)
    message.depth_status = o.localization.reason
    e = message.evidence
    e.evidence_id, e.sha256, e.mime_type = o.evidence_id, o.evidence_sha256, 'application/octet-stream'
    e.source_id, e.source_boot_id, e.observed_at_ns = o.source_camera, o.source_boot_id, o.observed_at_ns
    return message


# Robot-specific correction and compressedDepth decoding are kept in this
# existing target boundary module. OpenCV/NumPy remain lazy imports.
@dataclass(frozen=True)
class DepthCalibration:
    calibration_id: str
    physical_robot_id: str
    scale: float
    offset_m: float
    sha256: str


def load_depth_calibration(path, *, physical_robot_id):
    def resource_bytes(name):
        if not isinstance(name, str) or not name or Path(name).name != name:
            raise ValueError('depth calibration resource must be a file name')
        return files('rescue_perception').joinpath('config').joinpath(name).read_bytes()

    path = str(path)
    payload = resource_bytes(path[len('package:'):]) if path.startswith('package:') else Path(path).read_bytes()
    raw = json.loads(payload)
    for key in ('calibration_id', 'physical_robot_id', 'source_url', 'date'):
        if not isinstance(raw.get(key), str) or not raw[key].strip():
            raise ValueError(f'depth calibration requires {key}')
    if raw['physical_robot_id'] != physical_robot_id:
        raise ValueError('depth calibration physical robot mismatch')
    scale, offset = raw.get('scale'), raw.get('offset_m')
    if type(scale) not in (float, int) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('depth calibration scale must be positive and finite')
    if type(offset) not in (float, int) or not math.isfinite(offset):
        raise ValueError('depth calibration offset_m must be finite')
    if raw.get('input_unit') != 'm' or raw.get('input_state') != 'raw_uncorrected':
        raise ValueError('depth calibration expects raw uncorrected metric depth')
    return DepthCalibration(raw['calibration_id'], physical_robot_id, float(scale), float(offset),
                            hashlib.sha256(payload).hexdigest())


def decode_depth_png(message, *, byte_order, bridge):
    import cv2
    import numpy as np
    fmt = message.format
    match = re.fullmatch(r'\s*(16UC1|32FC1);\s*compressedDepth(?:\s+png)?\s*', fmt)
    if not match:
        raise ValueError(f'only 16UC1/32FC1 compressedDepth PNG is supported, got {fmt!r}')
    if byte_order not in ('little', 'big'):
        raise ValueError('depth publisher byte order must be little or big')
    header = struct.Struct(('<' if byte_order == 'little' else '>') + 'iff')
    payload = bytes(message.data)
    if len(payload) <= header.size or payload[header.size:header.size+8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('compressedDepth header/PNG payload missing')
    mode, factor, offset = header.unpack_from(payload)
    if mode != 0:
        raise ValueError('unsupported compressedDepth header mode')
    depth = cv2.imdecode(np.frombuffer(payload[header.size:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError('compressedDepth PNG must decode to a uint16 single-channel array')
    if match.group(1) == '32FC1':
        if not math.isfinite(factor) or factor <= 0 or not math.isfinite(offset):
            raise ValueError('invalid float-depth reconstruction parameters')
        denominator = depth.astype(np.float32) - offset
        valid = (depth != 0) & (denominator > 0)
        restored = np.full(depth.shape, np.nan, dtype=np.float32)
        restored[valid] = factor / denominator[valid]
        depth = restored
    image = bridge.cv2_to_imgmsg(depth, encoding=match.group(1))
    from copy import deepcopy
    image.header = deepcopy(message.header)
    return image
