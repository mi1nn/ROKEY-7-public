"""Exit QR on TB-6 RGB-D (OPS-136/138, owner decision 2026-10-13 evening).

TB-6 registers the exit where it reads the exit QR on patrol. A QR counts only when its content
equals the field value exactly, in ``required_frames`` consecutive frames, each with a depth-backed
position, all within ``position_tolerance_m`` of the first. Any other frame (no QR, other content,
no depth) restarts the run: a misread must never make an exit. The position is the QR centre:
corners undistorted onto the aligned-depth grid (#164), median depth inside the inner half of the
code, back-projected. Pure functions; amr_target wires them into its RGB-D frame callback.

Message (no new IDL, OPS-138): a TargetObservation with class_label ``exit_qr`` on
EXIT_OBSERVATIONS_TOPIC; a second target_locator turns it into a TargetLocation on
EXIT_LOCATION_TOPIC with the TB-6 map session/revision. confidence 1.0 means the content matched,
not a probability. model_id is ``opencv.QRCodeDetector/<cv2 version>``, model_sha256 the installed
cv2 binary. One observation per map revision (the coordinator re-bases older ones at the freeze).
"""
from dataclasses import dataclass
import math
from statistics import median

GRID = 9   # depth samples per side inside the inner half of the code
CLASS_LABEL = 'exit_qr'
EXIT_OBSERVATIONS_TOPIC = '/rescue/exit_observations'
EXIT_LOCATION_TOPIC = '/rescue/exit_location'
EXIT_PARAMS = {'exit_qr_payload': 'STRING', 'exit_qr_required_frames': 'INTEGER',
               'exit_qr_position_tolerance_m': 'DOUBLE', 'exit_qr_min_valid_depth_ratio': 'DOUBLE',
               'exit_qr_every_n_frames': 'INTEGER',
               'exit_map_state_topic': 'STRING'}


@dataclass(frozen=True)
class ExitQrPolicy:
    payload: str                  # field file value, no default
    required_frames: int          # consecutive matching frames [proposal 5]
    position_tolerance_m: float   # spread allowed within one run [proposal 0.2]
    min_valid_depth_ratio: float  # share of depth samples that must be valid [proposal 0.5]
    every_n_frames: int           # check one frame in n: about 35 ms per 704x704 frame (OPS-141) [proposal 6]

    def __post_init__(self):
        if type(self.payload) is not str or not self.payload.strip():
            raise ValueError('payload must be a non-empty string')
        if type(self.required_frames) is not int or self.required_frames < 1:
            raise ValueError('required_frames must be an integer >= 1')
        if type(self.position_tolerance_m) is not float or not math.isfinite(self.position_tolerance_m) \
                or self.position_tolerance_m <= 0:
            raise ValueError('position_tolerance_m must be a finite float > 0')
        if type(self.min_valid_depth_ratio) is not float or not 0 < self.min_valid_depth_ratio <= 1:
            raise ValueError('min_valid_depth_ratio must be in (0, 1]')
        if type(self.every_n_frames) is not int or self.every_n_frames < 1:
            raise ValueError('every_n_frames must be an integer >= 1')


class ExitQrConfirmer:
    """Counts consecutive checked frames that read the exit QR at one place."""

    def __init__(self, policy):
        self.policy = policy
        self.run = []
        self.frames = 0

    def due(self):
        """True for the frames to check (one in every_n_frames); the others are not seen and keep the run."""
        self.frames += 1
        return (self.frames - 1) % self.policy.every_n_frames == 0

    def add(self, payload, point):
        """One frame: decoded content (or None) and its sensor point (or None). True on the confirming frame."""
        if payload != self.policy.payload or point is None:
            self.run = []
            return False
        if self.run and math.dist(point, self.run[0]) > self.policy.position_tolerance_m:
            self.run = []
        self.run.append(point)
        if len(self.run) < self.policy.required_frames:
            return False
        self.run = []
        return True


def detect_qr(bgr):
    """(content, four (u, v) corners) of a decoded QR in a BGR image, or None."""
    import cv2
    text, points, _ = cv2.QRCodeDetector().detectAndDecode(bgr)
    if not text or points is None:
        return None
    return text, tuple((float(u), float(v)) for u, v in points.reshape(-1, 2))


def qr_point(corners, rows_m, *, fx, fy, cx, cy, undistort, min_valid_ratio):
    """QR centre in the optical frame (x, y, z) from depth in metres (NaN/0 invalid), or None."""
    corners = list(undistort(corners)) if undistort is not None else list(corners)
    cu = sum(u for u, _ in corners) / 4
    cv = sum(v for _, v in corners) / 4
    inner = [(cu + (u - cu) / 2, cv + (v - cv) / 2) for u, v in corners]   # away from the quiet-zone edge
    (a, b, c, d), samples, total = inner, [], 0
    for i in range(GRID):
        s = i / (GRID - 1)
        for j in range(GRID):
            t = j / (GRID - 1)
            u = (1 - s) * ((1 - t) * a[0] + t * b[0]) + s * ((1 - t) * d[0] + t * c[0])
            v = (1 - s) * ((1 - t) * a[1] + t * b[1]) + s * ((1 - t) * d[1] + t * c[1])
            x, y = int(round(u)), int(round(v))
            total += 1
            if 0 <= y < len(rows_m) and 0 <= x < len(rows_m[y]):
                value = float(rows_m[y][x])
                if math.isfinite(value) and value > 0:
                    samples.append(value)
    if not samples or len(samples) < min_valid_ratio * total:
        return None
    z = median(samples)
    return ((cu - cx) * z / fx, (cv - cy) * z / fy, z)


def exit_values(values):
    """Field values (no defaults) -> (ExitQrPolicy, TB-6 MapState topic) or ValueError."""
    missing = sorted(name for name in EXIT_PARAMS if values.get(name) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    topic = values['exit_map_state_topic']
    if type(topic) is not str or not topic.strip():
        raise ValueError('exit_map_state_topic must be a non-empty STRING')
    return ExitQrPolicy(values['exit_qr_payload'], values['exit_qr_required_frames'],
                        values['exit_qr_position_tolerance_m'], values['exit_qr_min_valid_depth_ratio'],
                        values['exit_qr_every_n_frames']), topic


def detector_identity():
    """(model_id, sha256 of the installed cv2 binary) for the exit observations."""
    import hashlib
    import os
    import cv2
    path = cv2.__file__
    if not path.endswith(('.so', '.pyd')):   # pip layout: cv2/__init__.py loads cv2/python-3.x/cv2*.so
        found = [os.path.join(d, f) for d, _, files in os.walk(os.path.dirname(path)) for f in files
                 if f.startswith('cv2') and f.endswith(('.so', '.pyd'))]
        if len(found) != 1:
            raise RuntimeError(f'cv2 binary not identified: {found}')
        path = found[0]
    with open(path, 'rb') as handle:
        return f'opencv.QRCodeDetector/{cv2.__version__}', hashlib.sha256(handle.read()).hexdigest()


def fill_exit_observation(message, *, observation_id, binding, frame_id, observed_at_ns, validity_ns, corners, point,
                          covariance, model_id, model_sha256, evidence_id, evidence_sha256):
    """TargetObservation fields for a confirmed exit QR; ``binding`` already carries this frame's clock."""
    message.observation_id = observation_id
    message.source_robot, message.source_camera = binding.source_robot, binding.source_camera
    message.source_boot_id = binding.source_boot_id
    t = message.time
    t.observed_at_ns, t.valid_until_ns = observed_at_ns, observed_at_ns + validity_ns
    t.stamp_semantics, t.clock_domain_id = binding.rgb_stamp_semantics, binding.clock_domain_id
    t.clock_state = binding.clock_state
    t.has_clock_error_bound = binding.clock_error_bound_ns is not None
    t.clock_error_bound_ns = binding.clock_error_bound_ns or 0
    message.class_label, message.confidence = CLASS_LABEL, 1.0   # content matched, not a probability
    message.model_id, message.model_sha256 = model_id, model_sha256
    message.tracking_mode, message.has_track_id, message.track_id = 'DETECTION_ONLY', False, ''
    us, vs = [u for u, _ in corners], [v for _, v in corners]
    message.bbox_xyxy_pixels = [min(us), min(vs), max(us), max(vs)]
    message.sensor_frame = frame_id
    message.localization_status, message.depth_status = 'VALID', 'LOCATED'
    p = message.sensor_position
    p.x, p.y, p.z = point
    message.position_covariance = list(covariance)
    e = message.evidence
    e.evidence_id, e.sha256, e.mime_type = evidence_id, evidence_sha256, 'application/octet-stream'
    e.source_id, e.source_boot_id, e.observed_at_ns = binding.source_camera, binding.source_boot_id, observed_at_ns
    return message


class ExitSender:
    """One exit observation per TB-6 map (session, revision); ``None`` = no VALID map known yet."""

    def __init__(self):
        self.sent = ()   # nothing sent

    def send(self, map_key):
        if self.sent != () and self.sent == map_key:
            return False
        self.sent = map_key
        return True
