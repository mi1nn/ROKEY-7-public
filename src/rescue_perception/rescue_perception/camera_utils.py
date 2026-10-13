"""ROS-free validation and one-frame processing for the AMR YOLO preview.

Preview only: no localization, tracking, observation admission or motion.
Settings are supplied by the nodes; hardware evidence is checked separately.
"""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import re
import subprocess

# User settings, 2026-10-12: one robot6/GPU deployment, no external YAML.
# Controller deployment: Ubuntu 24.04, ROS 2 Jazzy, RTX 4070.
# Topic/frame names still require live verification; no sensor settings are changed.
SETTINGS = {
    'image_type': 'compressed',
    'annotated_topic': 'perception/annotated',
    'model_path': 'assets/models/best.pt',
    'model_sha256': '0a149876a59fabb06bd57d8d146a4938a8fe2a74e030982941c28c431a4dcb74',
    'model_label': 'survivor',
    'model_imgsz': 640,
    'model_conf': 0.5,
    'device': '0',
    'show': False,
    'no_frame_warn_s': 5.0,
    'log_every': 10,
}

# Field-dependent names are typed ROS parameters with no default (B1, 2026-10-13): the
# robot6 OAK-D topic was written three different ways and none is verified on the robot.
FIELD_PARAMS = {'image_topic': 'STRING'}
STAMP_SEMANTICS = ('CAPTURE', 'DRIVER_RECEIVE', 'UNKNOWN')   # ObservationTime vocabulary

PARAMS = {
    'image_topic': 'STRING', 'image_type': 'STRING', 'annotated_topic': 'STRING',
    'model_path': 'STRING', 'model_sha256': 'STRING', 'model_label': 'STRING',
    'model_imgsz': 'INTEGER', 'model_conf': 'DOUBLE', 'device': 'STRING',
    'show': 'BOOL', 'no_frame_warn_s': 'DOUBLE', 'log_every': 'INTEGER',
}


def field_values(settings, supplied, names):
    """settings + the required field parameters; missing, empty or mistyped -> ValueError."""
    missing = sorted(name for name in names if supplied.get(name) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    for name, kind in names.items():
        value = supplied[name]
        if kind == 'BOOL':
            if type(value) is not bool:
                raise ValueError(f'{name} must be a BOOL')
        elif type(value) is not str or not value.strip():
            raise ValueError(f'{name} must be a non-empty {kind}')
    for name in ('rgb_stamp_semantics', 'depth_stamp_semantics'):
        if name in names and supplied[name] not in STAMP_SEMANTICS:
            raise ValueError(f'{name} must be one of {STAMP_SEMANTICS}')
    return {**settings, **{name: supplied[name] for name in names}}


def read_field_parameters(node, names):
    """Declare typed parameters without a value; an unset one comes back as None."""
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.parameter import Parameter
    supplied = {}
    for name, kind in names.items():
        node.declare_parameter(name, Parameter.Type[kind],
                               ParameterDescriptor(description='field value; no default'))
        try:
            supplied[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            supplied[name] = None
    return supplied


def validate_config(values):
    missing = sorted(key for key in PARAMS if values.get(key) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    for key, kind in PARAMS.items():
        value = values[key]
        expected = {'STRING': str, 'INTEGER': int, 'DOUBLE': float, 'BOOL': bool}[kind]
        if type(value) is not expected:
            raise ValueError(f'{key} must be {kind}')
        if kind == 'STRING' and not value.strip():
            raise ValueError(f'{key} must not be empty')
    if values['image_type'] not in ('raw', 'compressed'):
        raise ValueError('image_type must be raw or compressed; check ros2 topic type')
    if values['image_topic'] == values['annotated_topic']:
        raise ValueError('input and annotated topics must differ')
    if not re.fullmatch(r'[0-9a-fA-F]{64}', values['model_sha256']):
        raise ValueError('model_sha256 must be the full 64-digit SHA-256')
    for key in ('model_imgsz', 'log_every'):
        if values[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if not math.isfinite(values['model_conf']) or not 0 <= values['model_conf'] <= 1:
        raise ValueError('model_conf must be finite and in [0, 1]')
    if not math.isfinite(values['no_frame_warn_s']) or values['no_frame_warn_s'] <= 0:
        raise ValueError('no_frame_warn_s must be finite and positive')
    return dict(values)


def validate_rgb_compressed_format(value):
    """Accept JPEG/PNG RGB previews; reject depth payloads before RGB decoding."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError('RGB compressed message must declare its format')
    fmt = value.lower()
    if 'compresseddepth' in fmt or re.match(r'\s*(16uc1|32fc1|mono16)\b', fmt):
        raise ValueError('depth compression is not RGB; use a dedicated depth decoder')
    if not re.search(r'\b(jpeg|jpg|png)\b', fmt):
        raise ValueError(f'unsupported RGB compressed format: {value!r}; expected JPEG or PNG')


def verify_model(path, expected_sha):
    """Require an existing local .pt file; never request automatic model download."""
    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file() or path.suffix != '.pt':
        raise ValueError('model_path must be an existing local .pt file')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != expected_sha.lower():
        raise ValueError(f'model SHA-256 mismatch: actual={actual} expected={expected_sha}')
    return str(path), actual


def validate_labels(names, expected):
    labels = list(names.values()) if isinstance(names, dict) else list(names)
    if labels != [expected]:
        raise ValueError(f'expected single class {expected!r}, model has {names!r}')


def git_sha():
    try:
        directory = str(Path(__file__).resolve().parent)
        sha = subprocess.run(['git', '-C', directory, 'rev-parse', 'HEAD'],
            capture_output=True, text=True, check=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', '-C', directory, 'status', '--porcelain'],
            capture_output=True, text=True, check=True, timeout=5).stdout.strip()
        return sha + ('+dirty' if dirty else '')
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


class PreviewProcessor:
    """Dependency-injected boundary. Exceptions propagate; failure produces no image.

    Preserve the source header exactly (integer sec/nanosec and frame_id).
    No new timestamp or successful empty result is fabricated on inference failure.
    """
    def __init__(self, model, bridge, values):
        self.model, self.bridge = model, bridge
        self.values = validate_config(values)

    def infer(self, message):
        if self.values['image_type'] == 'raw':
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        else:
            validate_rgb_compressed_format(getattr(message, 'format', None))
            frame = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding='bgr8')
        results = self.model.predict(
            frame, imgsz=self.values['model_imgsz'], conf=self.values['model_conf'],
            device=self.values['device'], verbose=False)
        if len(results) != 1 or results[0].boxes is None:
            raise ValueError('expected one object detection result with boxes')
        return frame, results[0]

    def process(self, message):
        frame, result = self.infer(message)
        annotated = result.plot(labels=True, conf=True)
        image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        image.header = deepcopy(message.header)
        return image, annotated, len(result.boxes)
