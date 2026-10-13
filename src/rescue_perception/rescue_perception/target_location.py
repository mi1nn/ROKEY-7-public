"""Confirmed camera observation + capture-time TF map<-camera -> TargetLocation (B2, 2026-10-13).

ROS-free. The node (target_locator.py) looks up the TF at the RGB stamp and hands it here.
The map basis (session, revision, transform revision) and the map -> mission alignment are
declared by the field configuration, the same values the coordinator holds as
goal_map_session_id/goal_map_revision/goal_transform_revision; this module does not measure them.
A location is VALID only from a VALID camera point and a TF stamped at the capture time;
anything else is refused with a reason and nothing is published.
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

    @classmethod
    def from_values(cls, v):
        for key in ('robot_id', 'map_frame', 'mission_frame'):
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
        return cls(v['robot_id'], v['map_frame'], v['mission_frame'], v['map_session_id'], v['map_revision'],
                   v['transform_revision'], RigidTransform.planar(*xy_yaw), v['alignment_position_error_m'],
                   v['alignment_rotation_error_rad'])


@dataclass(frozen=True)
class Located:
    position: tuple
    covariance: tuple


def locate(observation, tf, basis):
    """observation: TargetObservation; tf: TransformStamped map_frame <- observation.sensor_frame."""
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
    if not all(math.isfinite(x) for x in position + covariance):
        raise LocationRefused('NONFINITE_POSITION', 'derived value')
    return Located(position, covariance)


def fill_location(message, observation, located, basis):
    message.location_id = str(uuid5(NAMESPACE_URL, f'ROKEY-7/location/{observation.observation_id}/'
                                    f'{basis.map_session_id}/{basis.map_revision}/{basis.transform_revision}'))
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
