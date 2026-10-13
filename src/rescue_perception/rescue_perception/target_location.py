"""Confirmed camera observation + capture-time TF map<-camera -> TargetLocation (B2, 2026-10-13).

ROS-free. The node (target_locator.py) looks up the TF at the RGB stamp and hands it here.
The map basis (session, revision, transform revision) and the map -> mission alignment are
declared by the field configuration, the same values the coordinator holds as
goal_map_session_id/goal_map_revision/goal_transform_revision; this module does not measure them.
A location is VALID only from a VALID camera point and a TF stamped at the capture time;
anything else is refused with a reason and nothing is published.

OPS-103 (2026-10-13, CDX-142-01·02): the producer also refuses an unusable time basis (C-03:
not SYNCED, no error bound, UNKNOWN stamp semantics, another clock domain, expired, future stamp),
a capture outside the declared alignment authority window (C-04; the authority is the profile
map_basis plus the robot's declared saved-map identity, #132/#145), and a covariance that is not
symmetric, finite, non-negative on the diagonal and positive semi-definite (C-02).
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import ros_time_to_ns, uuid_string
from rescue_mapping import RigidTransform
from .map_projection import rotate_covariance


class LocationRefused(ValueError):
    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


@dataclass(frozen=True)
class LocationBasis:
    robot_id: str
    map_frame: str
    mission_frame: str
    map_session_id: str
    map_revision: int
    transform_revision: int
    alignment: RigidTransform  # mission <- map
    alignment_position_error_m: float
    alignment_rotation_error_rad: float
    alignment_xy_yaw: tuple
    clock_domain_id: str           # the measured clock domain of the observations (C-03)
    alignment_authority_id: str    # who declared the map -> mission alignment (C-04)
    alignment_valid_from_ns: int
    alignment_valid_until_ns: int

    @classmethod
    def from_values(cls, v):
        for key in ('robot_id', 'map_frame', 'mission_frame', 'clock_domain_id', 'alignment_authority_id'):
            if type(v[key]) is not str or not v[key].strip():
                raise ValueError(f'{key} must be a non-empty string')
        uuid_string(v['map_session_id'])
        for key in ('map_revision', 'transform_revision'):
            if type(v[key]) is not int or not 0 <= v[key] < 2**64:
                raise ValueError(f'{key} must be uint64')
        xy_yaw = list(v['alignment_xy_yaw'])
        if len(xy_yaw) != 3 or not all(type(x) is float and math.isfinite(x) for x in xy_yaw):
            raise ValueError('alignment_xy_yaw must be three finite floats [x, y, yaw]')
        for key in ('alignment_position_error_m', 'alignment_rotation_error_rad'):
            if type(v[key]) is not float or not math.isfinite(v[key]) or v[key] < 0:
                raise ValueError(f'{key} must be finite and >= 0')
        start, end = v['alignment_valid_from_ns'], v['alignment_valid_until_ns']
        if type(start) is not int or type(end) is not int or not 0 <= start < end:
            raise ValueError('alignment validity needs integer ns with valid_from < valid_until')
        return cls(v['robot_id'], v['map_frame'], v['mission_frame'], v['map_session_id'], v['map_revision'],
                   v['transform_revision'], RigidTransform.planar(*xy_yaw), v['alignment_position_error_m'],
                   v['alignment_rotation_error_rad'], tuple(xy_yaw), v['clock_domain_id'],
                   v['alignment_authority_id'], start, end)


@dataclass(frozen=True)
class Located:
    position: tuple
    covariance: tuple


def time_refusal(t, basis, now_ns):
    """The C-03 reason this observation time cannot be used now, or None (OPS-103 decision 1)."""
    if t.clock_state != 'SYNCED':
        return 'CLOCK_NOT_SYNCED'
    if not t.has_clock_error_bound:
        return 'NO_CLOCK_ERROR_BOUND'
    if t.stamp_semantics == 'UNKNOWN':
        return 'STAMP_SEMANTICS_UNKNOWN'
    if t.clock_domain_id != basis.clock_domain_id:
        return 'CLOCK_DOMAIN_MISMATCH'
    if now_ns > t.valid_until_ns:
        return 'OBSERVATION_EXPIRED'
    if t.observed_at_ns > now_ns + t.clock_error_bound_ns:
        return 'FUTURE_STAMP'
    return None


def covariance_ok(c, tolerance=1e-12):
    """3x3 row-major: finite, symmetric, diagonal >= 0 and every principal minor >= 0 (PSD)."""
    if len(c) != 9 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in c):
        return False
    m = [[float(c[3 * i + j]) for j in range(3)] for i in range(3)]
    if any(abs(m[i][j] - m[j][i]) > tolerance * max(1.0, abs(m[i][j])) for i in range(3) for j in range(i)):
        return False
    if any(m[i][i] < -tolerance for i in range(3)):
        return False
    minors = [m[i][i] * m[j][j] - m[i][j] * m[j][i] for i, j in ((0, 1), (0, 2), (1, 2))]
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    return all(x >= -tolerance for x in minors) and det >= -tolerance


def locate(observation, tf, basis, *, now_ns):
    """observation: TargetObservation; tf: TransformStamped map_frame <- observation.sensor_frame.

    ``now_ns``: this host's clock (the measured clock domain ``basis.clock_domain_id``)."""
    reason = time_refusal(observation.time, basis, now_ns)
    if reason is not None:
        raise LocationRefused(reason, f'observed={observation.time.observed_at_ns} now={now_ns}')
    if not basis.alignment_valid_from_ns <= observation.time.observed_at_ns < basis.alignment_valid_until_ns:
        raise LocationRefused('ALIGNMENT_NOT_VALID_AT_CAPTURE', basis.alignment_authority_id)
    if now_ns >= basis.alignment_valid_until_ns:
        raise LocationRefused('ALIGNMENT_EXPIRED', basis.alignment_authority_id)
    if not covariance_ok(tuple(observation.position_covariance)):
        raise LocationRefused('COVARIANCE_INVALID', str(list(observation.position_covariance)))
    if observation.source_robot != basis.robot_id:
        raise LocationRefused('SOURCE_ROBOT_MISMATCH', observation.source_robot)
    if observation.localization_status != 'VALID':
        raise LocationRefused('NO_SENSOR_POSITION', observation.localization_status)
    p = observation.sensor_position
    point = (p.x, p.y, p.z)
    if not all(math.isfinite(x) for x in point):
        raise LocationRefused('NONFINITE_POSITION')
    if tf.header.frame_id != basis.map_frame:
        raise LocationRefused('TF_FRAME_MISMATCH', tf.header.frame_id)
    if tf.child_frame_id != observation.sensor_frame:
        raise LocationRefused('TF_CHILD_MISMATCH', tf.child_frame_id)
    stamp = ros_time_to_ns(tf.header.stamp.sec, tf.header.stamp.nanosec)
    if stamp != observation.time.observed_at_ns:
        raise LocationRefused('TF_STAMP_MISMATCH', f'tf={stamp} observed={observation.time.observed_at_ns}')
    t, q = tf.transform.translation, tf.transform.rotation
    sensor_to_map = RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
    in_map = sensor_to_map.apply(point)
    position = basis.alignment.apply(in_map)
    covariance = rotate_covariance(rotate_covariance(tuple(observation.position_covariance), sensor_to_map),
                                   basis.alignment)
    # Same conservative bound as map_projection.fill_location: declared alignment position
    # error plus the rotation error lever arm. AMCL pose uncertainty is not included.
    radius = math.dist(in_map, (0., 0., 0.))
    error = basis.alignment_position_error_m + 2 * radius * math.sin(min(basis.alignment_rotation_error_rad,
                                                                         math.pi) / 2)
    covariance = tuple(value + (error ** 2 if index in (0, 4, 8) else 0.) for index, value in enumerate(covariance))
    if not all(math.isfinite(x) for x in position):
        raise LocationRefused('NONFINITE_POSITION', 'derived value')
    if not covariance_ok(covariance, tolerance=1e-9):
        raise LocationRefused('COVARIANCE_INVALID', 'derived value')
    return Located(position, covariance)


def fill_location(message, observation, located, basis):
    # The alignment value and its authority are part of the content, so changing them makes a new ID
    # instead of an ID_CONTENT_CONFLICT at the coordinator.
    alignment = ','.join(f'{x:.9g}' for x in basis.alignment_xy_yaw)
    message.location_id = str(uuid5(NAMESPACE_URL, f'ROKEY-7/location/{observation.observation_id}/'
                                    f'{basis.map_session_id}/{basis.map_revision}/{basis.transform_revision}/'
                                    f'{basis.alignment_authority_id}/{alignment}'))
    message.observation = observation
    f = message.frame_ref
    f.robot_id, f.native_frame, f.mission_frame = basis.robot_id, basis.map_frame, basis.mission_frame
    f.map_session_id, f.map_revision, f.transform_revision = (basis.map_session_id, basis.map_revision,
                                                              basis.transform_revision)
    f.alignment_status = 'VALID'
    message.projection_status = 'VALID'
    message.mission_position.x, message.mission_position.y, message.mission_position.z = located.position
    message.covariance = list(located.covariance)
    return message
