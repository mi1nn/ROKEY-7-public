"""Occupancy map (map_server YAML + binary PGM) -> PNG bytes and map-frame -> image pixels.

Standard library only (no PyYAML, Pillow or numpy). A map that cannot be read exactly is a
ConsoleError; nothing is guessed, defaulted or redrawn.
"""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import struct
import threading
import time
import zlib

from .view import ConsoleError

# Markers are drawn only for poses in this frame. The saved map is the pre-quake reference
# map whose frame is the mission frame candidate (assets/manifest/pre_quake_map.json, Q-09).
MAP_FRAME = 'map'
# rescue_operations map_snapshot writes this next to live_map.yaml while the map topic has a publisher.
ALIVE_FILE = 'live_map.alive'
# map_server YAML keys (nav2_map_server map_io). Any other key is refused, not ignored.
YAML_KEYS = frozenset({'image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'})
REQUIRED_YAML_KEYS = ('image', 'resolution', 'origin')
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def _invalid(message):
    return ConsoleError('INVALID_INPUT', f'map: {message}')


def _scalar(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in '\'"':
        return text[1:-1]
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text


def parse_map_yaml(text):
    """Flat `key: value` lines, values scalar or `[a, b, c]`. Nested YAML is refused."""
    result = {}
    for number, line in enumerate(text.splitlines(), 1):
        line = line.split('#', 1)[0].rstrip()
        if not line:
            continue
        key, sep, value = line.partition(':')
        key = key.strip()
        if not sep or line[0].isspace() or not value.strip():
            raise _invalid(f'yaml line {number} is not a flat "key: value"')
        if key not in YAML_KEYS:
            raise _invalid(f'unsupported yaml key {key!r}')
        if key in result:
            raise _invalid(f'duplicate yaml key {key!r}')
        value = value.strip()
        if value.startswith('['):
            if not value.endswith(']'):
                raise _invalid(f'yaml line {number}: unclosed list')
            value = [_scalar(item) for item in value[1:-1].split(',')]
        else:
            value = _scalar(value)
        result[key] = value
    missing = [key for key in REQUIRED_YAML_KEYS if key not in result]
    if missing:
        raise _invalid(f'yaml missing {missing}')
    return result


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise _invalid(f'{label} must be a finite number')
    return float(value)


def read_pgm(data):
    """Binary PGM (P5, maxval <= 255) -> (width, height, pixels top row first, maxval)."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            end = data.find(b'\n', pos)
            if end < 0:
                raise _invalid('pgm header ends inside a comment')
            pos = end + 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace() and data[pos:pos + 1] != b'#':
            pos += 1
        if start == pos:
            raise _invalid('pgm header is truncated')
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise _invalid(f'only binary PGM (P5) is supported, got {tokens[0][:8]!r}')
    try:
        width, height, maxval = (int(token) for token in tokens[1:])
    except ValueError as exc:
        raise _invalid('pgm size or maxval is not an integer') from exc
    if width <= 0 or height <= 0 or not 0 < maxval <= 255:
        raise _invalid(f'pgm size {width}x{height} or maxval {maxval} out of range')
    if pos >= len(data) or not data[pos:pos + 1].isspace():
        raise _invalid('pgm header must end with one whitespace byte')
    pixels = data[pos + 1:]
    if len(pixels) != width * height:
        raise _invalid(f'pgm raster has {len(pixels)} bytes, expected {width * height}')
    return width, height, pixels, maxval


def encode_png(width, height, pixels):
    """8-bit grayscale PNG, filter type 0 on every row."""
    if len(pixels) != width * height:
        raise _invalid('pixel count does not match the size')
    raw = b''.join(b'\x00' + pixels[row * width:(row + 1) * width] for row in range(height))

    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))

    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return PNG_SIGNATURE + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')


@dataclass(frozen=True)
class MapImage:
    meta: dict      # what the screen model carries (no bytes)
    png: bytes


def _read(path):
    try:
        return path.read_bytes()
    except OSError as exc:
        raise ConsoleError('STORAGE_ERROR', f'map file unreadable: {path.name}: {exc.strerror}') from exc


def load_map(yaml_path):
    """Read a map_server map once. The PGM is shown as saved (negate only flips it so dark = occupied)."""
    yaml_path = Path(yaml_path)
    yaml_bytes = _read(yaml_path)
    try:
        spec = parse_map_yaml(yaml_bytes.decode('utf-8'))
    except UnicodeDecodeError as exc:
        raise _invalid('yaml is not UTF-8') from exc
    origin = spec['origin']
    if not isinstance(origin, list) or len(origin) != 3:
        raise _invalid('origin must be [x, y, yaw]')
    origin = [_number(value, 'origin') for value in origin]
    if origin[2] != 0.0:
        # A rotated origin would need a rotated image; refuse instead of drawing markers in the wrong place.
        raise _invalid(f'origin yaw {origin[2]} is not supported (only 0)')
    resolution = _number(spec['resolution'], 'resolution')
    if resolution <= 0:
        raise _invalid('resolution must be positive')
    negate = spec.get('negate', 0)
    if negate not in (0, 1):
        raise _invalid('negate must be 0 or 1')
    if not isinstance(spec['image'], str) or not spec['image']:
        raise _invalid('image must be a file name')
    image_path = Path(spec['image'])
    image_path = image_path if image_path.is_absolute() else yaml_path.parent / image_path
    pgm_bytes = _read(image_path)
    width, height, pixels, maxval = read_pgm(pgm_bytes)
    if maxval != 255:
        pixels = bytes(value * 255 // maxval for value in pixels)
    if negate:
        pixels = bytes(255 - value for value in pixels)
    pgm_sha256 = hashlib.sha256(pgm_bytes).hexdigest()
    meta = {
        'source': yaml_path.name,
        'image': image_path.name,
        'width_px': width,
        'height_px': height,
        'resolution': resolution,
        'origin': origin,
        'frame_id': MAP_FRAME,
        'mode': spec.get('mode'),
        'yaml_sha256': hashlib.sha256(yaml_bytes).hexdigest(),
        'pgm_sha256': pgm_sha256,
        # Cache key: a different map file on restart gets a different URL.
        'url': f'map.png?v={pgm_sha256[:12]}',
    }
    return MapImage(meta=meta, png=encode_png(width, height, pixels))


class LiveMap:
    """--live-map: the SLAM map file (rescue_operations map_snapshot) re-read whenever it changes.

    The YAML's modification time is when the map was received. The map is STALE when both that and
    ``live_map.alive`` (refreshed while the topic still has a publisher, so a frozen map that is not
    re-sent stays current, OPS-117/118) are older than ``stale_s``.
    A missing map is MISSING and a map that fails to load is ERROR, keeping the last good image
    with the error said on screen: an old picture is never shown as if it were current.
    """

    def __init__(self, yaml_path, *, stale_s, clock=time.time_ns):
        if type(stale_s) not in (int, float) or not math.isfinite(stale_s) or stale_s <= 0:
            raise _invalid('live map stale seconds must be a positive number')
        self.path, self.stale_ns, self.clock = Path(yaml_path), int(stale_s * 1e9), clock
        self.image, self._key, self._error = None, None, None
        self._lock = threading.Lock()

    def current(self):
        """(MapImage or None, {'state': OK|STALE|ERROR|MISSING, 'received_at_ns', 'age_s', 'message'})."""
        with self._lock:
            try:
                stat = self.path.stat()
            except OSError as exc:
                if self.image is None:
                    return None, {'state': 'MISSING', 'received_at_ns': None, 'age_s': None,
                                  'message': f'실시간 지도 없음 — {self.path.name} 를 아직 받지 못함'}
                return self.image, {'state': 'ERROR', 'received_at_ns': None, 'age_s': None,
                                    'message': f'지도 읽기 실패: {exc.strerror} — 마지막으로 읽은 지도를 보임'}
            key = (stat.st_mtime_ns, stat.st_size)
            if key != self._key:
                self._key = key
                try:
                    self.image, self._error = load_map(self.path), None
                except ConsoleError as exc:
                    self._error = exc.detail
            received = stat.st_mtime_ns
            age_s = round((self.clock() - received) / 1e9, 1)
            if self._error is not None:
                kept = ' — 마지막으로 읽은 지도를 보임' if self.image is not None else ''
                return self.image, {'state': 'ERROR', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 읽기 실패: {self._error}{kept}'}
            try:
                alive = (self.path.parent / ALIVE_FILE).stat().st_mtime_ns
            except OSError:
                alive = received
            heard = max(received, alive)
            if self.clock() - heard > self.stale_ns:
                silent = round((self.clock() - heard) / 1e9, 1)
                return self.image, {'state': 'STALE', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 낡음: {silent:g}초 동안 지도 발행 없음'}
            return self.image, {'state': 'OK', 'received_at_ns': received, 'age_s': age_s, 'message': ''}


def world_to_pixel(meta, x, y):
    """Map-frame metres -> image coordinates (column from the left, row from the top), continuous.

    `origin` is the lower-left corner of the lower-left cell (map_server), so the origin is at
    (0, height) and the far corner at (width, 0). Returns None when the point is off the image.
    """
    # Rounded to 1/100 px before the edge test, so a point exactly on the far edge is not lost
    # to float error (0.2 / 0.05 = 3.9999999999999996).
    column = round((x - meta['origin'][0]) / meta['resolution'], 2)
    row = round(meta['height_px'] - (y - meta['origin'][1]) / meta['resolution'], 2)
    if not (0 <= column <= meta['width_px'] and 0 <= row <= meta['height_px']):
        return None
    return column, row


EXIT_NAME = '출구(QR)'


def map_overlay(meta, robots, targets, exit_=None):
    """Screen-model `map` section: markers for poses in MAP_FRAME that land on the image.

    robots / targets are the screen-model rows (`pose` / `position` may be None). Poses in another
    frame are listed in `frame_mismatch` and never projected; on-frame poses outside the image are
    listed in `off_map`, not clamped to the edge.
    """
    overlay = dict(meta, available=True, robots=[], targets=[], exits=[], off_map=[], frame_mismatch=[], no_pose=[],
                   note=f'기준 지도 {meta["source"]} ({meta["width_px"]}×{meta["height_px"]} px, '
                        f'{meta["resolution"]} m/px) — 좌표 map_session 일치는 확인하지 않음(Q-09)')
    entries = [('robot', row['robot_id'], row.get('pose'), {'yaw': (row.get('pose') or {}).get('yaw'),
                                                            'role': row.get('role'), 'name': row.get('name')}) for row in robots]
    entries += [('target', row['target_id'], row.get('position'), {'stage': row['stage']}) for row in targets]
    if exit_ is not None:   # the newest exit_qr observation (#237)
        entries.append(('exit', EXIT_NAME, exit_, {'observed_at_ns': exit_['observed_at_ns'],
                                                   'map_revision': exit_['map_revision']}))
    for kind, ident, pose, extra in entries:
        if pose is None:
            if kind == 'robot':   # listed, not placed: no pose is ever made up (TB-5 before the map is fixed)
                overlay['no_pose'].append({'id': ident, 'name': extra['name'] or ident})
            continue
        where = {'kind': kind, 'id': ident, 'x': pose['x'], 'y': pose['y'], 'frame_id': pose['frame_id']}
        if pose['frame_id'] != MAP_FRAME:
            overlay['frame_mismatch'].append(where)
            continue
        pixel = world_to_pixel(meta, pose['x'], pose['y'])
        if pixel is None:
            overlay['off_map'].append(where)
            continue
        overlay[kind + 's'].append(dict(extra, id=ident, px=pixel[0], py=pixel[1]))
    return overlay
