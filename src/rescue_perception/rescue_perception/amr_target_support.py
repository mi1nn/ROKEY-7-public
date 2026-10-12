"""ROS-free AMR target adapters: YOLO results, observation wire fields, guarded projection.

Reuse PersonPipeline for RGB-D checks and rescue_mapping for rigid transforms.
Map/alignment metadata must come from the mapping owner, never from detection.
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import ros_time_to_ns, timestamp_ns, uuid_string
from rescue_mapping import FrameAlignment, RigidTransform
from .core import validate_freshness
from .person_pipeline import DetectorOutput, Inference


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
