"""ROS-independent rigid transforms; quaternion order is x, y, z, w."""

from dataclasses import dataclass
import math
from rescue_contracts import (RescueError, canonical_code, robot_id,
                              timestamp_ns, uuid_string as common_uuid_string)


class MappingError(RescueError):
    """A value cannot be used under the coordinate contract."""

    def __init__(self, code: str, detail: str = ""):
        super().__init__(canonical_code("rescue_mapping", code), detail,
                         component="rescue_mapping", detail_code=code)


def robot_identifier(value):
    try:
        return robot_id(value)
    except RescueError as exc:
        raise MappingError("INVALID_INPUT", str(exc)) from exc


def name(value, label):
    if not isinstance(value, str) or not value or any(c.isspace() or ord(c) < 32 for c in value):
        raise MappingError("INVALID_INPUT", label + " must be a nonempty identifier")
    return value


def integer(value, label, minimum=0):
    if type(value) is not int or value < minimum:
        raise MappingError("INVALID_INPUT", label + " must be an integer >= " + str(minimum))
    return value


def uint64(value, label):
    integer(value, label)
    if value > 2**64 - 1:
        raise MappingError("INVALID_INPUT", label + " exceeds uint64")
    return value


def uuid_string(value, label):
    try:
        return common_uuid_string(value)
    except RescueError as exc:
        raise MappingError("INVALID_INPUT", label + ": " + exc.detail) from exc


def timestamp(value, label):
    try:
        return timestamp_ns(value)
    except RescueError as exc:
        raise MappingError("INVALID_INPUT", label + ": " + exc.detail) from exc


def finite(value, label, minimum=None):
    if type(value) not in (int, float):
        raise MappingError("INVALID_INPUT", label + " must be numeric")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise MappingError("INVALID_INPUT", label + " is not finite") from exc
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise MappingError("INVALID_INPUT", label + " is out of range")
    return result


def vector(value, size, label):
    if not isinstance(value, (tuple, list)) or len(value) != size:
        raise MappingError("INVALID_INPUT", label + " has the wrong dimension")
    return tuple(finite(item, label) for item in value)


@dataclass(frozen=True)
class RigidTransform:
    """Map a source point to a target: p_target = R(q) p_source + t.

    Lists are defensively copied; finite nonzero quaternions are normalized.
    The constructor does not establish which frames or time this transform has.
    """

    translation_xyz: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        translation = vector(self.translation_xyz, 3, "translation_xyz")
        rotation = vector(self.rotation_xyzw, 4, "rotation_xyzw")
        scale = max(abs(v) for v in rotation)
        if scale == 0:
            raise MappingError("INVALID_QUATERNION")
        # Scale first: a subnormal hypot can round before division and produce
        # a non-unit quaternion; a very large hypot can overflow altogether.
        scaled = tuple(v / scale for v in rotation)
        norm = math.hypot(*scaled)
        object.__setattr__(self, "translation_xyz", translation)
        object.__setattr__(self, "rotation_xyzw", tuple(v / norm for v in scaled))

    @classmethod
    def identity(cls):
        return cls((0, 0, 0), (0, 0, 0, 1))

    @classmethod
    def planar(cls, x, y, yaw, z=0):
        yaw = finite(yaw, "yaw")
        return cls((x, y, z), (0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))

    def apply(self, point_xyz):
        px, py, pz = vector(point_xyz, 3, "point_xyz")
        x, y, z, w = self.rotation_xyzw
        # Twice q_vector cross point, followed by the second cross product.
        cx, cy, cz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
        tx, ty, tz = self.translation_xyz
        return vector((px + w * cx + y * cz - z * cy + tx,
                       py + w * cy + z * cx - x * cz + ty,
                       pz + w * cz + x * cy - y * cx + tz), 3, "transformed point")

    def inverse(self):
        x, y, z, w = self.rotation_xyzw
        inverse_rotation = RigidTransform((0, 0, 0), (-x, -y, -z, w))
        translation = inverse_rotation.apply(tuple(-v for v in self.translation_xyz))
        return RigidTransform(translation, inverse_rotation.rotation_xyzw)
