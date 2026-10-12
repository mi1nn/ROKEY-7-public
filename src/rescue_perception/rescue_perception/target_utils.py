"""ROS-free AMR target adapters: YOLO results, observation wire fields, guarded projection.

Reuse PersonPipeline for RGB-D checks and rescue_mapping for rigid transforms.
Map/alignment metadata must come from the mapping owner, never from detection.
"""
from dataclasses import dataclass, replace
import math
import hashlib
import json
import re
import struct
from pathlib import Path
from importlib.resources import files
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import ros_time_to_ns, timestamp_ns, uuid_string
from rescue_mapping import FrameAlignment, RigidTransform
from .core import AlignmentEvidence, DepthPolicy, TimePolicy, validate_freshness
from .person_pipeline import ConfirmationPolicy, DetectorOutput, Inference
from .adapter import ObservationPolicy, SourceBinding
from .camera_utils import SETTINGS as CAMERA_SETTINGS, validate_config


# User settings, 2026-10-12: share RGB settings and always select robot6 correction.
# UNKNOWN clock/alignment values are deliberate until measured; locations stay blocked.
SETTINGS = {
    **CAMERA_SETTINGS,
    'depth_topic': 'camera/depth/image_raw/compressedDepth',
    'depth_transport': 'compressedDepth',
    'depth_byte_order': 'little',
    'camera_info_topic': 'camera/camera_info',
    'observations_topic': '/rescue/observations',
    'locations_topic': '/rescue/target_locations',
    'map_state_topic': 'mapping/map_state',
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
    'receiver_clock_error_ns': 0,
    'publisher_id': 'amr.perception',
    'envelope_version': '0.1',
    'sync_queue_size': 3,
    'sync_slop_ns': 20000000,
    'camera_info_max_age_ns': 1000000000,
    'observation_validity_ns': 2000000000,
    'max_map_age_ns': 1000000000,
    'max_age_ns': 1000000000,
    'max_future_ns': 0,
    'max_clock_error_ns': 20000000,
    'stamp_delay_bound_ns': 0,
    'min_depth_m': 0.2,
    'max_depth_m': 5.0,
    'min_valid_pixels': 10,
    'min_valid_fraction': 0.5,
    'max_depth_spread_m': 0.5,
    'sensor_variance_xyz': [0.04, 0.04, 0.04],
    'alignment_authority': 'UNCONFIGURED',
    'max_alignment_position_error_m': 0.1,
    'max_alignment_rotation_error_rad': 0.1,
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
    for key in ('sync_queue_size', 'camera_info_max_age_ns', 'observation_validity_ns', 'max_map_age_ns'):
        if v[key] <= 0:
            raise ValueError(f'{key} must be positive')
    topics = [v[key] for key in ('image_topic', 'depth_topic', 'camera_info_topic', 'annotated_topic',
                                 'observations_topic', 'locations_topic', 'map_state_topic')]
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
    freshness = TimePolicy(v['max_age_ns'], v['max_future_ns'], v['max_clock_error_ns'], v['stamp_delay_bound_ns'])
    if v['envelope_version'] != '0.1':
        raise ValueError('only the current ROS wire envelope version 0.1 is supported')
    covariance = covariance_matrix(v['sensor_variance_xyz'])
    if v['receiver_clock_error_ns'] < 0:
        raise ValueError('receiver_clock_error_ns must be nonnegative')
    return binding, alignment, depth, expiry, confirmation, freshness, covariance


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


def rotate_covariance(covariance, transform):
    rotation = RigidTransform((0., 0., 0.), transform.rotation_xyzw)
    columns = [rotation.apply(axis) for axis in ((1.,0.,0.), (0.,1.,0.), (0.,0.,1.))]
    r = [[columns[j][i] for j in range(3)] for i in range(3)]
    return tuple(sum(r[i][k] * covariance[k*3+l] * r[j][l] for k in range(3) for l in range(3))
                 for i in range(3) for j in range(3))


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


@dataclass(frozen=True)
class ProjectionMetadata:
    """Read-only, externally supplied mapping snapshot, with an explicit expiry.

    ``covariance_verified`` and evidence ID attest supplied variances and alignment
    error bounds. Neither this adapter nor a TF lookup measures those uncertainties.
    """
    alignment: FrameAlignment
    covariance_verified: bool
    covariance_evidence_id: str
    sensor_variance_xyz: tuple[float, float, float]

    @classmethod
    def from_dict(cls, values):
        a = dict(values['alignment'])
        if a.get('status') != 'VALID':
            raise ValueError('mapping snapshot must explicitly declare a VALID alignment')
        variances = tuple(values['sensor_variance_xyz'])
        covariance_matrix(variances)
        a['transform'] = RigidTransform(**a['transform'])
        if values.get('covariance_verified') is not True or not values.get('covariance_evidence_id', '').strip():
            raise ValueError('mapping snapshot must identify verified covariance evidence')
        return cls(FrameAlignment(**a), True, values['covariance_evidence_id'], variances)


class MapStateTracker:
    """A delayed lower revision/retired session cannot restore an old map basis."""
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.current = None
        self.retired = set()

    def update(self, message):
        from copy import deepcopy
        if message.robot_id != self.robot_id:
            raise ValueError('MapState robot mismatch')
        uuid_string(message.map_session_id)
        timestamp_ns(message.generated_at_ns)
        if type(message.map_revision) is not int or not 0 <= message.map_revision < 2**64:
            raise ValueError('MapState revision must be uint64')
        if not message.current_native_map_frame:
            raise ValueError('MapState frame is required')
        previous = self.current
        if message.map_session_id in self.retired:
            raise ValueError('retired map session replay')
        if previous is not None:
            if message.generated_at_ns < previous.generated_at_ns:
                raise ValueError('reordered MapState')
            if message.map_session_id == previous.map_session_id:
                if message.map_revision < previous.map_revision:
                    raise ValueError('map revision decreased')
                if message.current_native_map_frame != previous.current_native_map_frame:
                    raise ValueError('map frame changed within the same session')
            else:
                self.retired.add(previous.map_session_id)
        self.current = deepcopy(message)
        return self.current


def check_projection_inputs(observation, metadata, map_state, *, now_ns, max_map_age_ns,
                            receiver_clock_error_ns, time_policy):
    a = metadata.alignment
    validate_freshness(observation, now_ns=now_ns, receiver_clock_domain=a.clock_domain_id,
                       receiver_clock_error_bound_ns=receiver_clock_error_ns, policy=time_policy)
    if observation.localization.point_optical is None:
        raise ValueError('no valid sensor position')
    if not metadata.covariance_verified or not metadata.covariance_evidence_id:
        raise ValueError('covariance not verified')
    if a.status != 'VALID' or not a.valid_from_ns <= observation.observed_at_ns <= now_ns < a.valid_until_ns:
        raise ValueError('alignment invalid, expired, or not valid at observation time')
    if observation.valid_until_ns > a.valid_until_ns:
        raise ValueError('alignment expires before the original observation')
    if map_state is None:
        raise ValueError('fresh MapState is required')
    uuid_string(map_state.map_session_id)
    timestamp_ns(map_state.generated_at_ns)
    basis = (map_state.robot_id, map_state.current_native_map_frame, map_state.map_session_id, map_state.map_revision)
    if basis != (observation.source_robot, a.map_frame, a.map_session_id, a.map_revision) or a.source_robot != observation.source_robot:
        raise ValueError('mapping snapshot does not match current robot/frame/session/revision')
    if map_state.correction_detection_status != 'KNOWN' or map_state.localization_quality != 'VALID':
        raise ValueError('map correction detection/localization quality not verified')
    uncertainty = receiver_clock_error_ns + observation.clock_error_bound_ns
    if map_state.generated_at_ns > observation.observed_at_ns + uncertainty:
        raise ValueError('MapState generated after observation; map basis at capture is not established')
    if map_state.generated_at_ns > now_ns + uncertainty or now_ns - map_state.generated_at_ns + uncertainty > max_map_age_ns:
        raise ValueError('MapState is stale or from the future')


def stamped_transform(message, *, source_frame, target_frame, observed_at_ns):
    if message.child_frame_id != source_frame or message.header.frame_id != target_frame:
        raise ValueError('TF frame mismatch')
    stamp = ros_time_to_ns(message.header.stamp.sec, message.header.stamp.nanosec)
    if stamp != observed_at_ns:
        raise ValueError('TF result must be queried at the original RGB stamp; latest/static-zero result is not accepted')
    t, q = message.transform.translation, message.transform.rotation
    return RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def fill_location(message, observation_message, observation, metadata, sensor_to_map, sensor_covariance):
    a = metadata.alignment
    message.location_id = str(uuid5(NAMESPACE_URL,
        f'ROKEY-7/location/{observation.observation_id}/{a.map_session_id}/{a.map_revision}/{a.transform_revision}'))
    message.observation = observation_message
    f = message.frame_ref
    f.robot_id, f.native_frame, f.mission_frame = observation.source_robot, a.map_frame, a.mission_frame
    f.map_session_id, f.map_revision, f.transform_revision = a.map_session_id, a.map_revision, a.transform_revision
    f.alignment_status = a.status
    message.projection_status = 'VALID'
    point = a.transform.apply(sensor_to_map.apply(observation.localization.point_optical))
    message.mission_position.x, message.mission_position.y, message.mission_position.z = point
    covariance = rotate_covariance(rotate_covariance(sensor_covariance, sensor_to_map), a.transform)
    # Conservative positional bound includes the configured alignment rotation error lever arm.
    radius = math.dist(sensor_to_map.apply(observation.localization.point_optical), (0., 0., 0.))
    alignment_error = a.position_error_m + 2 * radius * math.sin(min(a.rotation_error_rad, math.pi) / 2)
    message.covariance = [value + (alignment_error ** 2 if index in (0,4,8) else 0.)
                          for index, value in enumerate(covariance)]
    if not all(math.isfinite(value) for value in message.covariance):
        raise ValueError('derived covariance is not finite')
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
