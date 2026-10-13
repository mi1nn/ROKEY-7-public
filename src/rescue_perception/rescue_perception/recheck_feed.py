"""amr_target frame/scan -> recheck window inputs (2026-10-13). Pure; no ROS, no model.

amr_target calls these only while the ObserveTarget server has an open window (recheck_server).
"""
from dataclasses import dataclass
import math
from statistics import median

from rescue_mapping import RigidTransform
from .recheck import frame_coverage, inverse


@dataclass(frozen=True)
class Candidate:
    status: str          # CONFIRMED / UNCONFIRMED (person_pipeline)
    point_optical: tuple | None
    bbox: tuple          # raw RGB pixel box [x0, y0, x1, y1)


def rigid_from_tf(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def scan_fields(scan):
    return dict(ranges=list(scan.ranges), angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                range_min=scan.range_min, range_max=scan.range_max)


def depth_median(rows, u, v, half):
    """Median of the finite positive depths (m) in the window around pixel (u, v), or None."""
    values = []
    for y in range(max(0, v - half), min(len(rows), v + half + 1)):
        row = rows[y]
        for x in range(max(0, u - half), min(len(row), u + half + 1)):
            value = float(row[x])
            if math.isfinite(value) and value > 0:
                values.append(value)
    return median(values) if values else None


def frame_evidence(point_map, map_from_optical, camera, depth_at, candidates, policy):
    """(coverage, confirmed points in the map, unconfirmed detection on the point) for one frame."""
    coverage = frame_coverage(point_map, map_from_optical, camera, depth_at, policy)[0]
    confirmed, unconfirmed = [], False
    x, y, z = inverse(map_from_optical).apply(point_map)
    pixel = None if z <= 0 else (camera.fx * x / z + camera.cx, camera.fy * y / z + camera.cy)
    for c in candidates:
        in_map = None if c.point_optical is None else map_from_optical.apply(c.point_optical)
        if c.status == 'CONFIRMED' and in_map is not None:
            confirmed.append(in_map)
            continue
        near = in_map is not None and math.dist(in_map, point_map) <= policy.presence_radius_m
        x0, y0, x1, y1 = c.bbox
        on_box = pixel is not None and x0 <= pixel[0] < x1 and y0 <= pixel[1] < y1
        unconfirmed = unconfirmed or near or on_box
    return coverage, confirmed, unconfirmed


# Recheck window settings of amr_target: typed parameters without defaults (prefix recheck_ so they do
# not mix with the detection depth range). Policy names follow recheck.RecheckPolicy.
RECHECK_PARAMS = {
    'recheck_action_name': 'STRING', 'recheck_map_frame': 'STRING', 'recheck_scan_topic': 'STRING',
    'recheck_max_window_ns': 'INTEGER', 'recheck_validity_ns': 'INTEGER',
    'recheck_presence_radius_m': 'DOUBLE', 'recheck_lidar_radius_m': 'DOUBLE', 'recheck_min_frames': 'INTEGER',
    'recheck_min_scans': 'INTEGER', 'recheck_min_lidar_rays': 'INTEGER', 'recheck_min_lidar_hits': 'INTEGER',
    'recheck_image_margin_px': 'INTEGER', 'recheck_min_depth_m': 'DOUBLE', 'recheck_max_depth_m': 'DOUBLE',
    'recheck_occlusion_tolerance_m': 'DOUBLE', 'recheck_depth_window_px': 'INTEGER',
}


def recheck_values(values):
    """-> (RecheckPolicy, other settings) or ValueError for a missing or bad value."""
    from .recheck import RecheckPolicy
    missing = sorted(name for name in RECHECK_PARAMS if values.get(name) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    for name in ('recheck_action_name', 'recheck_map_frame', 'recheck_scan_topic'):
        if type(values[name]) is not str or not values[name].strip():
            raise ValueError(f'{name} must be a non-empty STRING')
    for name in ('recheck_max_window_ns', 'recheck_validity_ns'):
        if type(values[name]) is not int or values[name] <= 0:
            raise ValueError(f'{name} must be a positive INTEGER')
    policy_names = ('presence_radius_m', 'lidar_radius_m', 'min_frames', 'min_scans', 'min_lidar_rays',
                    'min_lidar_hits', 'image_margin_px', 'min_depth_m', 'max_depth_m', 'occlusion_tolerance_m',
                    'depth_window_px')
    policy = RecheckPolicy(**{name: values['recheck_' + name] for name in policy_names})
    return policy, {name: values[name] for name in ('recheck_action_name', 'recheck_map_frame', 'recheck_scan_topic',
                                                    'recheck_max_window_ns', 'recheck_validity_ns')}


def depth_rows_m(array, encoding, scale, offset_m):
    """Decoded depth image -> metres with the robot depth calibration; invalid pixels become NaN."""
    import numpy as np
    raw = np.asarray(array, dtype=np.float64) * (0.001 if encoding == '16UC1' else 1.0)
    valid = np.isfinite(raw) & (raw > 0)
    return np.where(valid, raw * scale + offset_m, np.nan)
