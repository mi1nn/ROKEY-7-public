"""점유 지도(map_server YAML + 이진 PGM) -> PNG 바이트, 그리고 map frame 좌표 -> 이미지 화소.

표준 라이브러리만 쓴다(PyYAML, Pillow, numpy 없음). 정확히 읽을 수 없는 지도는
ConsoleError 다. 추측하거나 기본값으로 채우거나 다시 그리지 않는다.
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

# 이 frame 의 위치만 지도에 표지로 그린다. 저장된 지도는 재난 전 기준 지도이고,
# 그 frame 이 임무 frame 후보다(assets/manifest/pre_quake_map.json, Q-09).
MAP_FRAME = 'map'
# rescue_operations map_snapshot 이 지도 토픽에 발행자가 있는 동안 live_map.yaml 옆에 이 파일을 쓴다.
ALIVE_FILE = 'live_map.alive'
# map_server YAML 키(nav2_map_server map_io). 다른 키는 무시하지 않고 거절한다.
YAML_KEYS = frozenset({'image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'})
REQUIRED_YAML_KEYS = ('image', 'resolution', 'origin')
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def _invalid(message):
    return ConsoleError('INVALID_INPUT', f'map: {message}')


def _scalar(text):
    """YAML 스칼라 하나: 따옴표가 있으면 안의 글, 아니면 int, 아니면 float, 아니면 그대로의 글.

    map_server 의 평평한 키에는 이것으로 충분하다. 여기에는 YAML 불리언·null·이스케이프 시퀀스가 없다.
    """
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
    """평평한 `key: value` 줄을 읽는다. 값은 스칼라 또는 `[a, b, c]`. 중첩 YAML 은 거절한다."""
    result = {}
    for number, line in enumerate(text.splitlines(), 1):
        # '#' 뒤는 따옴표 안이라도 모두 주석으로 본다: 이 평평한 파서의 단순화다.
        line = line.split('#', 1)[0].rstrip()
        if not line:
            continue
        key, sep, value = line.partition(':')
        key = key.strip()
        # 들여쓴 줄은 중첩 YAML 이다: 최상위 키로 읽지 않고 거절한다.
        if not sep or line[0].isspace() or not value.strip():
            raise _invalid(f'yaml line {number} is not a flat "key: value"')
        if key not in YAML_KEYS:
            raise _invalid(f'unsupported yaml key {key!r}')
        if key in result:
            raise _invalid(f'duplicate yaml key {key!r}')
        value = value.strip()
        if value.startswith('['):   # `origin: [x, y, yaw]` 같은 flow 목록. 쉼표로 나눠 스칼라로 만든다
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
    """이진 PGM(P5, maxval <= 255) -> (폭, 높이, 위 행부터의 화소, maxval)."""
    # 머리: 공백으로 나뉜 토큰 4개(매직, 폭, 높이, maxval). 그 사이에 '#' 주석이 끼어 있을 수 있다.
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
    # maxval 이 255 를 넘으면 표본 하나가 2 byte 라는 뜻인데, 이 읽기 함수는 그것을 다루지 않는다.
    if width <= 0 or height <= 0 or not 0 < maxval <= 255:
        raise _invalid(f'pgm size {width}x{height} or maxval {maxval} out of range')
    # 머리는 공백 1 byte 로 정확히 끝난다. 더 건너뛰면 값이 공백 코드인 화소를 먹어 버릴 수 있다.
    if pos >= len(data) or not data[pos:pos + 1].isspace():
        raise _invalid('pgm header must end with one whitespace byte')
    pixels = data[pos + 1:]
    # 크기가 정확해야 한다: 모자라거나 덧붙은 래스터는 자르거나 채우지 않고 거절한다.
    if len(pixels) != width * height:
        raise _invalid(f'pgm raster has {len(pixels)} bytes, expected {width * height}')
    return width, height, pixels, maxval


def encode_png(width, height, pixels):
    """8-bit 회색조 PNG. 모든 행에 필터 형식 0 을 쓴다."""
    if len(pixels) != width * height:
        raise _invalid('pixel count does not match the size')
    raw = b''.join(b'\x00' + pixels[row * width:(row + 1) * width] for row in range(height))

    def chunk(kind, body):
        # PNG 청크: big-endian 본문 길이, 4-byte 형식, 본문, 형식 + 본문에 대한 CRC-32.
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))

    # IHDR: 폭, 높이, 비트 깊이 8, 색 형식 0(회색조), 압축 0, 필터 0, 인터레이스 없음.
    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return PNG_SIGNATURE + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')


@dataclass(frozen=True)
class MapImage:
    """읽어 둔 지도 하나(frozen): /map.png 로 내보내는 PNG 와 화면 모델이 싣는 `meta` dict.

    `meta`(키는 load_map 참고)에는 이미지 바이트가 없어서, 폴링하는 화면 JSON 이 작게 유지된다.
    """
    meta: dict      # 화면 모델이 싣는 것(바이트 없음)
    png: bytes


def _read(path):
    try:
        return path.read_bytes()
    except OSError as exc:
        raise ConsoleError('STORAGE_ERROR', f'map file unreadable: {path.name}: {exc.strerror}') from exc


def load_map(yaml_path):
    """map_server 지도를 한 번 읽는다. PGM 은 저장된 그대로 보인다(negate 는 어두운 쪽 = 점유가 되게 뒤집기만 한다)."""
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
        # 회전한 origin 은 회전한 이미지가 있어야 한다: 표지를 엉뚱한 곳에 그리지 않고 거절한다.
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
    # 상대 이미지 이름은 map_server 처럼 YAML 이 있는 폴더를 기준으로 푼다.
    image_path = image_path if image_path.is_absolute() else yaml_path.parent / image_path
    pgm_bytes = _read(image_path)
    width, height, pixels, maxval = read_pgm(pgm_bytes)
    # occupied_thresh / free_thresh / mode 는 받아들이지만 화소에 적용하지 않는다: 화소는 저장된 그대로 보이고,
    # 0..255 로 늘이고 negate 로 뒤집기만 한다. map_snapshot 파일은 이미 trinary 다
    # (0 점유, 254 빈칸, 205 미지).
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
        # YAML 에는 frame 이 없다: 이 지도를 MAP_FRAME 으로 보고, 그 frame 의 위치만 그린다(map_overlay).
        'frame_id': MAP_FRAME,
        'mode': spec.get('mode'),
        'yaml_sha256': hashlib.sha256(yaml_bytes).hexdigest(),
        'pgm_sha256': pgm_sha256,
        # 캐시 키: 다시 시작했을 때 지도 파일이 다르면 URL 도 달라진다.
        'url': f'map.png?v={pgm_sha256[:12]}',
    }
    return MapImage(meta=meta, png=encode_png(width, height, pixels))


class LiveMap:
    """--live-map: SLAM 지도 파일(rescue_operations map_snapshot)을 바뀔 때마다 다시 읽는다.

    YAML 의 수정 시각이 지도를 받은 시각이다. 그 시각과
    ``live_map.alive``(토픽에 아직 발행자가 있는 동안 갱신되므로, 다시 보내지 않는 고정 지도도
    현재로 남는다, OPS-117/118)가 둘 다 ``stale_s`` 보다 오래되면 지도는 STALE 이다.
    지도가 없으면 MISSING, 읽기에 실패하면 ERROR 이며, 마지막으로 잘 읽은 이미지를 남기고
    오류를 화면에 함께 알린다: 옛 그림을 지금 것처럼 보이지 않는다.
    """

    def __init__(self, yaml_path, *, stale_s, clock=time.time_ns):
        """경로와 낡음 기준만 기억한다. 첫 current() 호출 전에는 아무것도 읽지 않는다.

        stale_s 는 기본값이 없고(현장에서 정하는 값) 유한한 양수여야 하며, 아니면 INVALID_INPUT.
        `clock` 은 파일 mtime 과 비교하므로 벽시계 ns 를 준다(같은 PC 를 전제). 시험은 가짜를 넘긴다.
        """
        if type(stale_s) not in (int, float) or not math.isfinite(stale_s) or stale_s <= 0:
            raise _invalid('live map stale seconds must be a positive number')
        self.path, self.stale_ns, self.clock = Path(yaml_path), int(stale_s * 1e9), clock
        # 마지막으로 잘 읽은 지도, 마지막으로 읽은 파일의 (mtime_ns, size), 그 읽기의 오류 문구(읽혔으면 None).
        self.image, self._key, self._error = None, None, None
        self._lock = threading.Lock()

    def current(self):
        """(MapImage 또는 None, {'state': OK|STALE|ERROR|MISSING, 'received_at_ns', 'age_s', 'message'}) 를 돌려준다."""
        with self._lock:
            try:
                stat = self.path.stat()
            except OSError as exc:
                if self.image is None:
                    return None, {'state': 'MISSING', 'received_at_ns': None, 'age_s': None,
                                  'message': f'실시간 지도 없음 — {self.path.name} 를 아직 받지 못함'}
                return self.image, {'state': 'ERROR', 'received_at_ns': None, 'age_s': None,
                                    'message': f'지도 읽기 실패: {exc.strerror} — 마지막으로 읽은 지도를 보임'}
            # map_snapshot 은 YAML 을 rename 으로 바꾸므로, (mtime, size) 가 새로우면 다 쓴 새 파일을 읽는다는 뜻이다.
            key = (stat.st_mtime_ns, stat.st_size)
            if key != self._key:
                self._key = key     # 읽기에 실패해도 저장해서, 깨진 파일을 폴링마다 다시 해석하지 않는다
                try:
                    self.image, self._error = load_map(self.path), None
                except ConsoleError as exc:
                    # self.image 는 마지막으로 잘 읽은 지도를 그대로 두고, 오류는 그 옆에 함께 알린다.
                    self._error = exc.detail
            received = stat.st_mtime_ns
            age_s = round((self.clock() - received) / 1e9, 1)
            # ERROR 를 STALE 보다 먼저 본다: 파일 자체가 최근 것이어도 읽기 실패는 알린다.
            if self._error is not None:
                kept = ' — 마지막으로 읽은 지도를 보임' if self.image is not None else ''
                return self.image, {'state': 'ERROR', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 읽기 실패: {self._error}{kept}'}
            try:
                alive = (self.path.parent / ALIVE_FILE).stat().st_mtime_ns
            except OSError:
                alive = received    # 읽을 수 있는 alive 파일이 없다: 지도 자체의 시각만으로 판단한다
            # 지도 토픽의 마지막 신호: 새로 받은 지도와 발행자 생존 touch 중 늦은 쪽.
            heard = max(received, alive)
            if self.clock() - heard > self.stale_ns:
                silent = round((self.clock() - heard) / 1e9, 1)
                return self.image, {'state': 'STALE', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 낡음: {silent:g}초 동안 지도 발행 없음'}
            return self.image, {'state': 'OK', 'received_at_ns': received, 'age_s': age_s, 'message': ''}


def world_to_pixel(meta, x, y):
    """map frame 미터 -> 이미지 좌표(왼쪽부터의 열, 위부터의 행), 연속 값.

    `origin` 은 왼쪽 아래 칸의 왼쪽 아래 모서리이므로(map_server), 원점은
    (0, height), 반대쪽 모서리는 (width, 0) 에 있다. 점이 이미지 밖이면 None 을 돌려준다.
    """
    # 가장자리 검사 전에 1/100 px 로 반올림해서, 반대쪽 가장자리에 딱 놓인 점을
    # 부동소수점 오차로 잃지 않는다(예: (0.3 - 0.1) / 0.05 = 3.9999999999999996).
    column = round((x - meta['origin'][0]) / meta['resolution'], 2)
    row = round(meta['height_px'] - (y - meta['origin'][1]) / meta['resolution'], 2)
    if not (0 <= column <= meta['width_px'] and 0 <= row <= meta['height_px']):
        return None
    return column, row


EXIT_NAME = '출구(QR)'


def map_overlay(meta, robots, targets, exit_=None):
    """화면 모델의 `map` 부분: MAP_FRAME 에 있고 이미지 안에 들어오는 위치의 표지.

    robots / targets 는 화면 모델의 행이고(`pose` / `position` 은 None 일 수 있다), exit_ 는 최신 출구(QR) 관측이다.
    다른 frame 의 위치는 `frame_mismatch` 에 넣고 투영하지 않는다. frame 은 맞지만 이미지 밖인 위치는 가장자리로
    끌어오지 않고 `off_map` 에, 위치가 없는 로봇은 `no_pose` 에 넣는다. 출구 표지는 `exits` 에 들어간다.
    """
    overlay = dict(meta, available=True, robots=[], targets=[], exits=[], off_map=[], frame_mismatch=[], no_pose=[],
                   note=f'기준 지도 {meta["source"]} ({meta["width_px"]}×{meta["height_px"]} px, '
                        f'{meta["resolution"]} m/px) — 좌표 map_session 일치는 확인하지 않음(Q-09)')
    entries = [('robot', row['robot_id'], row.get('pose'), {'yaw': (row.get('pose') or {}).get('yaw'),
                                                            'role': row.get('role'), 'name': row.get('name')}) for row in robots]
    entries += [('target', row['target_id'], row.get('position'), {'stage': row['stage']}) for row in targets]
    if exit_ is not None:   # 가장 최근의 exit_qr 관측(#237)
        entries.append(('exit', EXIT_NAME, exit_, {'observed_at_ns': exit_['observed_at_ns'],
                                                   'map_revision': exit_['map_revision']}))
    for kind, ident, pose, extra in entries:
        if pose is None:
            if kind == 'robot':   # 목록에만 넣고 그리지 않는다: 위치를 지어내지 않는다(지도가 고정되기 전의 TB-5)
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
