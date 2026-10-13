"""Occupancy-grid identity and free-footprint rule shared by robot and coordinator (OPS-117).

Pure functions on the OccupancyGrid fields (frame, size, resolution, planar origin x/y/yaw from
planar_origin, cell values); no ROS. The robot (rescue_robot.live_sources.Grid and planar,
map_occupancy_sha256.py) and the coordinator's frozen-map check call these, so the same grid gives the same hash and the
same MapState snapshot id on both sides. Invalid grids raise ValueError.
"""
import hashlib
import json
import math
import uuid

SNAPSHOT_NAMESPACE = uuid.uuid5(uuid.NAMESPACE_URL, 'rescue_robot/map_state/snapshot')  # same map -> same id


def planar_origin(px, py, pz, qx, qy, qz, qw):
    """(x, y, yaw) of a planar pose (z 0, rotation about z only, unit quaternion); else ValueError."""
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in (px, py, pz, qx, qy, qz, qw))
            or pz != 0 or qx != 0 or qy != 0 or not math.isclose(qz * qz + qw * qw, 1., abs_tol=1e-6)):
        raise ValueError('finite normalized planar pose required')
    return float(px), float(py), 2 * math.atan2(qz, qw)


def _checked(frame, width, height, resolution, origin, data):
    data = tuple(data)
    if (not isinstance(frame, str) or not frame or type(width) is not int or type(height) is not int
            or width <= 0 or height <= 0 or not isinstance(resolution, (int, float)) or isinstance(resolution, bool)
            or not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in origin)
            or len(data) != width * height or any(type(v) is not int or not -1 <= v <= 100 for v in data)):
        raise ValueError('invalid occupancy grid')
    return float(resolution), tuple(float(v) for v in origin), data


def occupancy_sha256(*, frame, width, height, resolution, origin, data):
    """Content hash of one grid; the publication stamp and load time are deliberately not part of it."""
    resolution, origin, data = _checked(frame, width, height, resolution, origin, data)
    payload = [frame, width, height, resolution, list(origin), data]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def map_snapshot_id(occupancy_sha256, map_session_id, map_revision, transform_revision):
    """MapState.snapshot_id (UUID): the map content and the basis it is published under."""
    return str(uuid.uuid5(SNAPSHOT_NAMESPACE, f'{occupancy_sha256}:{map_session_id}:{map_revision}:'
                                              f'{transform_revision}'))


def footprint_clear(*, frame, width, height, resolution, origin, data, x, y, radius):
    """True only if every cell touching the disc (x, y, radius) in the grid frame is inside the map and free (0)."""
    resolution, (ox, oy, yaw), data = _checked(frame, width, height, resolution, origin, data)
    c, s = math.cos(yaw), math.sin(yaw)
    ix = math.floor((c * (x - ox) + s * (y - oy)) / resolution)
    iy = math.floor((-s * (x - ox) + c * (y - oy)) / resolution)
    extent = math.ceil(radius / resolution) + 1
    for cy in range(iy - extent, iy + extent + 1):
        for cx in range(ix - extent, ix + extent + 1):
            lx, ly = (cx + .5) * resolution, (cy + .5) * resolution
            px, py = ox + c * lx - s * ly, oy + s * lx + c * ly
            if math.hypot(px - x, py - y) <= radius + resolution / math.sqrt(2):
                if not (0 <= cx < width and 0 <= cy < height) or data[cy * width + cx] != 0:
                    return False
    return True


def _xy_yaw(value):
    if (len(value) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                                   for v in value)):
        raise ValueError('planar pose must be three finite numbers (x, y, yaw)')
    return tuple(float(v) for v in value)


def compose_planar(start, relative):
    """Map pose of ``relative`` given in the frame of ``start`` (both (x, y, yaw)); yaw in [-pi, pi].

    Coordinates measured in advance are relative to the TB-6 start pose (OPS-141); the start pose
    (first TB-6 SLAM pose of the run, map frame) turns them into map coordinates.
    """
    (sx, sy, syaw), (rx, ry, ryaw) = _xy_yaw(start), _xy_yaw(relative)
    c, s = math.cos(syaw), math.sin(syaw)
    yaw = syaw + ryaw
    return sx + c * rx - s * ry, sy + s * rx + c * ry, math.atan2(math.sin(yaw), math.cos(yaw))


def relative_planar(start, pose):
    """Inverse of compose_planar: ``pose`` (map) expressed relative to ``start``."""
    (sx, sy, syaw), (px, py, pyaw) = _xy_yaw(start), _xy_yaw(pose)
    c, s = math.cos(syaw), math.sin(syaw)
    dx, dy, yaw = px - sx, py - sy, pyaw - syaw
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw), math.cos(yaw))
