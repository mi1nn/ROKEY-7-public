"""TB-6 recheck of a watched target point -> RecheckObservation states (2026-10-13). Pure, no ROS.

Rescue completion = fixed-webcam disappearance candidate + this recheck (owner decision 2026-10-13,
Q-10; contracts C-11 rows 7·9; SDD ObserveTarget Rules). The window holds only samples received
after the goal was accepted. An empty detection list is never ABSENT on its own: the point must be
in the image, in depth range and not occluded in enough frames. LiDAR CLEAR needs rays that pass
through the disc around the point. Whatever is not shown stays UNKNOWN.
"""
from dataclasses import dataclass, field
import math

from rescue_mapping import RigidTransform


@dataclass(frozen=True)
class RecheckPolicy:
    presence_radius_m: float      # a confirmed detection this close to the point = PRESENT
    lidar_radius_m: float         # disc around the point checked by the LiDAR
    min_frames: int               # frames with the point in view needed for ABSENT / coverage VALID
    min_scans: int                # scans with a CLEAR/BLOCKED answer needed for a LiDAR state
    min_lidar_rays: int           # rays through the disc needed for one scan to be CLEAR
    min_lidar_hits: int           # returns inside the disc needed for one scan to be BLOCKED
    image_margin_px: int          # the projected point must be this far inside the image
    min_depth_m: float
    max_depth_m: float
    occlusion_tolerance_m: float  # depth this much shorter than the point = something in front
    depth_window_px: int          # half size of the depth window around the projected pixel

    def __post_init__(self):
        for name in ('presence_radius_m', 'lidar_radius_m', 'min_depth_m', 'max_depth_m', 'occlusion_tolerance_m'):
            value = getattr(self, name)
            if type(value) is not float or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and > 0')
        for name in ('min_frames', 'min_scans', 'min_lidar_rays', 'min_lidar_hits'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f'{name} must be an integer >= 1')
        for name in ('image_margin_px', 'depth_window_px'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError(f'{name} must be an integer >= 0')
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError('max_depth_m must be above min_depth_m')


@dataclass(frozen=True)
class CameraModel:
    """Pinhole intrinsics of the rectified RGB image (CameraInfo K)."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def inverse(transform):
    """target<-source -> source<-target."""
    x, y, z, w = transform.rotation_xyzw
    conjugate = RigidTransform((0.0, 0.0, 0.0), (-x, -y, -z, w))
    tx, ty, tz = conjugate.apply(transform.translation_xyz)
    return RigidTransform((-tx, -ty, -tz), (-x, -y, -z, w))


def frame_coverage(point_map, map_from_optical, camera, depth_at, policy):
    """(VALID|INVALID|UNKNOWN, reason) for one RGB-D frame and the watched point (map frame).

    ``depth_at(u, v, half_window)`` returns the median valid depth (m, calibrated) around the pixel or None.
    """
    x, y, z = inverse(map_from_optical).apply(point_map)
    if z <= 0:
        return 'INVALID', 'BEHIND_CAMERA'
    u, v = camera.fx * x / z + camera.cx, camera.fy * y / z + camera.cy
    m = policy.image_margin_px
    if not (m <= u <= camera.width - 1 - m and m <= v <= camera.height - 1 - m):
        return 'INVALID', 'OUT_OF_IMAGE'
    if not policy.min_depth_m <= z <= policy.max_depth_m:
        return 'INVALID', 'OUT_OF_DEPTH_RANGE'
    measured = depth_at(int(round(u)), int(round(v)), policy.depth_window_px)
    if measured is None or not math.isfinite(measured):
        return 'UNKNOWN', 'NO_DEPTH'
    if measured < z - policy.occlusion_tolerance_m:
        return 'INVALID', 'OCCLUDED'
    return 'VALID', 'IN_VIEW'


def scan_state(scan, map_from_laser, point_map, policy):
    """(CLEAR|BLOCKED|UNKNOWN, reason) for one LaserScan (dict of its fields) and the watched point.

    Uses the scan plane only (2-D). For each ray that meets the disc around the point, the ray-circle
    entry/exit distances decide (#165 review CDX-165-02): a return inside [entry, exit] is a hit, one
    before the entry is occluded, one beyond the exit passes. No return (+inf, or a finite value above
    range_max, REP-117) passes only when the exit is within range_max. -inf (something closer than
    range_min, REP-117), NaN and finite values below range_min say nothing (CDX-165-01). CLEAR needs
    every ray toward the disc to pass: any occluded or unreadable ray leaves it UNKNOWN.
    """
    px, py, _ = inverse(map_from_laser).apply(point_map)
    distance = math.hypot(px, py)
    radius = policy.lidar_radius_m
    if distance <= radius:
        return 'UNKNOWN', 'INSIDE_DISC'
    hits = passes = occluded = unreadable = out_of_range = 0
    for index, rng in enumerate(scan['ranges']):
        angle = scan['angle_min'] + index * scan['angle_increment']
        along = px * math.cos(angle) + py * math.sin(angle)          # closest approach along the ray
        across_sq = distance * distance - along * along
        if along <= 0 or across_sq >= radius * radius:
            continue                                                 # the ray misses the disc
        half = math.sqrt(radius * radius - across_sq)
        entry, exit_ = along - half, along + half
        if rng == math.inf or (math.isfinite(rng) and rng > scan['range_max']):
            if exit_ <= scan['range_max']:
                passes += 1                                          # nothing returned up to past the disc
            else:
                out_of_range += 1
        elif math.isfinite(rng) and rng >= scan['range_min']:
            if rng < entry:
                occluded += 1
            elif rng <= exit_:
                hits += 1
            else:
                passes += 1
        else:
            unreadable += 1                                          # -inf, NaN or below range_min
    if hits + passes + occluded + unreadable + out_of_range == 0:
        return 'UNKNOWN', 'NOT_IN_SCAN'
    if hits >= policy.min_lidar_hits:
        return 'BLOCKED', 'RETURNS_IN_DISC'
    if occluded:
        return 'UNKNOWN', 'OCCLUDED'
    if unreadable:
        return 'UNKNOWN', 'UNREADABLE_RANGE'
    if hits == 0 and passes >= policy.min_lidar_rays and not out_of_range:
        return 'CLEAR', 'RAYS_PASS_DISC'
    if out_of_range and not passes:
        return 'UNKNOWN', 'OUT_OF_RANGE'
    return 'UNKNOWN', 'TOO_FEW_RAYS'


@dataclass
class RecheckWindow:
    """Samples received at or after ``start_ns`` (goal accepted) for one watched point."""
    point_map: tuple
    policy: RecheckPolicy
    start_ns: int
    frames: list = field(default_factory=list)   # (received_ns, coverage, near_confirmed, unconfirmed_at_point)
    scans: list = field(default_factory=list)    # (received_ns, state)

    def add_frame(self, *, received_ns, coverage, confirmed_points, unconfirmed_at_point):
        if received_ns < self.start_ns:
            return False
        near = any(math.dist(p, self.point_map) <= self.policy.presence_radius_m for p in confirmed_points)
        self.frames.append((received_ns, coverage, near, bool(unconfirmed_at_point)))
        return True

    def add_scan(self, *, received_ns, state):
        if received_ns < self.start_ns:
            return False
        self.scans.append((received_ns, state))
        return True

    def end_ns(self):
        stamps = [f[0] for f in self.frames] + [s[0] for s in self.scans]
        return max(stamps) if stamps else None


def window_states(window):
    """(camera_state, lidar_state, coverage_state) of a closed window (C-11 row 9 vocabulary)."""
    policy = window.policy
    in_view = sum(1 for f in window.frames if f[1] == 'VALID')
    out_of_view = sum(1 for f in window.frames if f[1] == 'INVALID')
    if any(f[2] for f in window.frames):
        camera = 'PRESENT'
    elif in_view >= policy.min_frames and not any(f[3] for f in window.frames):
        camera = 'ABSENT'
    else:
        camera = 'UNKNOWN'
    states = [s[1] for s in window.scans]
    if 'BLOCKED' in states:
        lidar = 'BLOCKED'
    elif states.count('CLEAR') >= policy.min_scans:
        lidar = 'CLEAR'
    else:
        lidar = 'UNKNOWN'
    if in_view >= policy.min_frames and lidar != 'UNKNOWN':
        coverage = 'VALID'
    elif out_of_view >= policy.min_frames and in_view < policy.min_frames:
        coverage = 'INVALID'
    else:
        coverage = 'UNKNOWN'
    return camera, lidar, coverage
