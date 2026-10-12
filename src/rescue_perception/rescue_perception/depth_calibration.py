"""Load an explicitly selected, physical-robot-bound depth calibration profile.

Robot6 selection authorized by the user on 2026-10-12, referencing PR #126.
No hardware configuration is changed. Missing/mismatched profiles fail startup.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from importlib.resources import files


@dataclass(frozen=True)
class DepthCalibration:
    calibration_id: str
    physical_robot_id: str
    scale: float
    offset_m: float
    sha256: str


def load_depth_calibration(path, *, physical_robot_id):
    def resource_bytes(name):
        if not isinstance(name, str) or not name or Path(name).name != name:
            raise ValueError('depth calibration resource must be a file name')
        return files('rescue_perception').joinpath('config').joinpath(name).read_bytes()

    path = str(path)
    payload = resource_bytes(path[len('package:'):]) if path.startswith('package:') else Path(path).read_bytes()
    raw = json.loads(payload)
    if 'profile_resource' in raw:
        payload = resource_bytes(raw['profile_resource'])
        if hashlib.sha256(payload).hexdigest() != raw.get('profile_sha256'):
            raise ValueError('depth calibration resource hash mismatch')
        raw = json.loads(payload)
    for key in ('calibration_id', 'physical_robot_id', 'source_url', 'date'):
        if not isinstance(raw.get(key), str) or not raw[key].strip():
            raise ValueError(f'depth calibration requires {key}')
    if raw['physical_robot_id'] != physical_robot_id:
        raise ValueError('depth calibration physical robot mismatch')
    scale, offset = raw.get('scale'), raw.get('offset_m')
    if type(scale) not in (float, int) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('depth calibration scale must be positive and finite')
    if type(offset) not in (float, int) or not math.isfinite(offset):
        raise ValueError('depth calibration offset_m must be finite')
    if raw.get('input_unit') != 'm' or raw.get('input_state') != 'raw_uncorrected':
        raise ValueError('depth calibration expects raw uncorrected metric depth')
    return DepthCalibration(raw['calibration_id'], physical_robot_id, float(scale), float(offset),
                            hashlib.sha256(payload).hexdigest())
