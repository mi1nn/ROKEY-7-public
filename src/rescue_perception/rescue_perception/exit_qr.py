"""Exit QR on TB-6 RGB-D (OPS-136/138, owner decision 2026-10-13 evening; several exits OPS-154/155/157).

TB-6 registers each exit where it reads that exit's QR while searching the map. The field file
lists the exit contents (e.g. EXIT-1, EXIT-2); a QR counts only when its content equals one of them
exactly, in ``required_frames`` consecutive checked frames, each with a depth-backed position, all
within ``position_tolerance_m`` of the first. Each exit counts on its own: a checked frame without
that exit (or without its depth, or with it read twice) restarts its run only. Other content is
never an exit: a misread must never make one. The position is the QR centre:
corners undistorted onto the aligned-depth grid (#164), median depth inside the inner half of the
code, back-projected. Pure functions; amr_target wires them into its RGB-D frame callback.

Message (no new IDL, OPS-138/157): a TargetObservation with class_label ``exit_qr`` and the exit
content as ``track_id`` (tracking_mode TRACKING: one name for the exit over frames and revisions) on
EXIT_OBSERVATIONS_TOPIC; a second target_locator turns it into a TargetLocation on
EXIT_LOCATION_TOPIC with the TB-6 map session/revision. confidence 1.0 means the content matched,
not a probability. model_id is ``opencv.QRCodeDetector/<cv2 version>``, model_sha256 the installed
cv2 binary. One observation per exit and map revision (the coordinator re-bases older ones at the freeze).
"""
from dataclasses import dataclass
import math
from statistics import median

GRID = 9   # depth samples per side inside the inner half of the code
CLASS_LABEL = 'exit_qr'
EXIT_OBSERVATIONS_TOPIC = '/rescue/exit_observations'
EXIT_LOCATION_TOPIC = '/rescue/exit_location'
EXIT_PARAMS = {'exit_qr_payloads': 'STRING_ARRAY', 'exit_qr_required_frames': 'INTEGER',
               'exit_qr_position_tolerance_m': 'DOUBLE', 'exit_qr_min_valid_depth_ratio': 'DOUBLE',
               'exit_qr_every_n_frames': 'INTEGER',
               'exit_map_state_topic': 'STRING'}


@dataclass(frozen=True)
class ExitQrPolicy:
    payloads: tuple               # exit contents from the field file (exact text, distinct), no default
    required_frames: int          # consecutive matching frames [proposal 5]
    position_tolerance_m: float   # spread allowed within one run [proposal 0.2]
    min_valid_depth_ratio: float  # share of depth samples that must be valid [proposal 0.5]
    every_n_frames: int           # check one frame in n: about 35 ms per 704x704 frame (OPS-141) [proposal 6]

    def __post_init__(self):
        if (type(self.payloads) is not tuple or not self.payloads
                or any(type(p) is not str or not p.strip() for p in self.payloads)
                or len(set(self.payloads)) != len(self.payloads)):
            raise ValueError('payloads must be a non-empty tuple of distinct non-empty strings')
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
    """Counts, for each exit, consecutive checked frames that read its QR at one place."""

    def __init__(self, policy):
        self.policy = policy
        self.runs = {payload: [] for payload in policy.payloads}
        self.frames = 0

    def due(self):
        """True for the frames to check (one in every_n_frames); the others are not seen and keep the run."""
        self.frames += 1
        return (self.frames - 1) % self.policy.every_n_frames == 0

    def add(self, readings):
        """One checked frame: {listed exit: sensor point or None}; ``{}`` = no exit read.

        Returns the exits confirmed on this frame, in field-file order."""
        unknown = set(readings) - set(self.runs)
        if unknown:
            raise ValueError(f'not a listed exit: {sorted(unknown)}')
        confirmed = []
        for payload, run in self.runs.items():
            point = readings.get(payload)
            if point is None:
                run.clear()
                continue
            if run and math.dist(point, run[0]) > self.policy.position_tolerance_m:
                run.clear()
            run.append(point)
            if len(run) >= self.policy.required_frames:
                run.clear()
                confirmed.append(payload)
        return tuple(confirmed)


def detect_qrs(bgr):
    """((content, four (u, v) corners), ...) of every decoded QR in a BGR image; codes not decoded are left out."""
    import cv2
    ok, texts, points, _ = cv2.QRCodeDetector().detectAndDecodeMulti(bgr)
    if not ok or points is None:
        return ()
    return tuple((text, tuple((float(u), float(v)) for u, v in corners.reshape(-1, 2)))
                 for text, corners in zip(texts, points) if text)


def split_qrs(found, payloads):
    """({listed exit: corners}, other contents, listed exits read more than once) of one frame.

    Exact text only. An exit read twice in one frame has no single place and is left out."""
    counts = {}
    for text, _ in found:
        counts[text] = counts.get(text, 0) + 1
    allowed = {text: corners for text, corners in found if text in payloads and counts[text] == 1}
    unknown = tuple(text for text, _ in found if text not in payloads)
    duplicated = tuple(p for p in payloads if counts.get(p, 0) > 1)
    return allowed, unknown, duplicated


def payloads_argument(text):
    """Launch argument ``EXIT-1,EXIT-2`` -> ('EXIT-1', 'EXIT-2'); exact text, an empty or repeated item is refused."""
    items = tuple(text.split(','))
    if any(not item.strip() for item in items) or len(set(items)) != len(items):
        raise ValueError(f'exit_qr_payloads must be distinct non-empty comma-separated contents, got {text!r}')
    return items


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
    return ExitQrPolicy(tuple(values['exit_qr_payloads']), values['exit_qr_required_frames'],
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


def fill_exit_observation(message, *, observation_id, binding, frame_id, observed_at_ns, validity_ns, payload, corners,
                          point, covariance, model_id, model_sha256, evidence_id, evidence_sha256):
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
    message.tracking_mode, message.has_track_id, message.track_id = 'TRACKING', True, payload   # which exit
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
    """One observation per exit and TB-6 map (session, revision); map ``None`` = no VALID map known yet."""

    def __init__(self):
        self.sent = {}   # exit -> map key last sent

    def send(self, payload, map_key):
        if payload in self.sent and self.sent[payload] == map_key:
            return False
        self.sent[payload] = map_key
        return True
