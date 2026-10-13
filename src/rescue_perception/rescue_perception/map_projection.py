"""Deferred sensor-to-map/mission projection helpers; not wired into amr_target.

Prepared on 2026-10-12 for later integration with existing rescue_mapping.
No ROS node, subscription, publisher or TF lookup is created here. The mapping
owner must supply verified map/alignment/covariance and capture-time transforms.
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import ros_time_to_ns, timestamp_ns, uuid_string
from rescue_mapping import FrameAlignment, RigidTransform
from .core import validate_freshness
from .target_utils import covariance_matrix


def rotate_covariance(covariance, transform):
    rotation = RigidTransform((0., 0., 0.), transform.rotation_xyzw)
    columns = [rotation.apply(axis) for axis in ((1.,0.,0.), (0.,1.,0.), (0.,0.,1.))]
    r = [[columns[j][i] for j in range(3)] for i in range(3)]
    return tuple(sum(r[i][k] * covariance[k*3+l] * r[j][l] for k in range(3) for l in range(3))
                 for i in range(3) for j in range(3))



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
