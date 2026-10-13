"""점유 격자 지도(map_server YAML + 바이너리 PGM)의 PNG 변환 및 월드(map) 좌표와 픽셀 좌표 변환.

외부 라이브러리 없이 표준 라이브러리만 사용한다(PyYAML, Pillow, numpy 미사용). 형식 오류 지도는
ConsoleError 예외로 처리하며 임의 보간이나 기본값 대체 및 추정 렌더링을 하지 않는다.
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

# 이 frame과 일치하는 위치만 지도에 마커로 표시한다. 저장된 지도는 재난 전 기준 지도이며,
# 해당 frame이 임무 frame 후보다(assets/manifest/pre_quake_map.json, Q-09).
MAP_FRAME = 'map'
# rescue_operations map_snapshot이 지도 토픽 발행자를 감지하는 동안 live_map.yaml 옆에 기록하는 파일.
ALIVE_FILE = 'live_map.alive'
# map_server YAML 허용 키 목록(nav2_map_server map_io). 정의되지 않은 키는 무시하지 않고 거부한다.
YAML_KEYS = frozenset({'image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'})
REQUIRED_YAML_KEYS = ('image', 'resolution', 'origin')
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def _invalid(message):
    return ConsoleError('INVALID_INPUT', f'map: {message}')


def _scalar(text):
    """YAML 스칼라 파싱: 따옴표 문자열, 정수, 부동소수점, 일반 문자열 순으로 변환한다.

    map_server의 단층 키 파싱에는 이 정도로 충분하며, 불리언·null·이스케이프 시퀀스는 지원하지 않는다.
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
    """단층 `key: value` 형식을 파싱한다. 값은 스칼라 또는 `[a, b, c]` 리스트이며 중첩 구조는 거부한다."""
    result = {}
    for number, line in enumerate(text.splitlines(), 1):
        # '#' 이후는 따옴표 내부라도 모두 주석으로 처리한다: 단층 파서의 단순화 규칙이다.
        line = line.split('#', 1)[0].rstrip()
        if not line:
            continue
        key, sep, value = line.partition(':')
        key = key.strip()
        # 들여쓰기된 줄은 중첩 YAML이므로 최상위 키로 허용하지 않고 거부한다.
        if not sep or line[0].isspace() or not value.strip():
            raise _invalid(f'yaml line {number} is not a flat "key: value"')
        if key not in YAML_KEYS:
            raise _invalid(f'unsupported yaml key {key!r}')
        if key in result:
            raise _invalid(f'duplicate yaml key {key!r}')
        value = value.strip()
        if value.startswith('['):   # `origin: [x, y, yaw]` 형태의 리스트. 쉼표로 분리해 스칼라로 변환한다.
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
    """바이너리 PGM(P5, maxval <= 255) -> (너비, 높이, 상단 행부터의 픽셀 바이트열, maxval)."""
    # 헤더: 공백으로 구분된 토큰 4개(매직 넘버, 너비, 높이, maxval). 중간에 '#' 주석이 포함될 수 있다.
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
    # maxval이 255를 초과하면 샘플당 2바이트 포맷이므로, 본 리더에서는 지원하지 않는다.
    if width <= 0 or height <= 0 or not 0 < maxval <= 255:
        raise _invalid(f'pgm size {width}x{height} or maxval {maxval} out of range')
    # 헤더는 공백 1바이트로 정확히 끝나야 한다. 추가로 건너뛰면 공백 코드와 일치하는 픽셀 데이터가 유실될 수 있다.
    if pos >= len(data) or not data[pos:pos + 1].isspace():
        raise _invalid('pgm header must end with one whitespace byte')
    pixels = data[pos + 1:]
    # 래스터 크기가 일치해야 한다. 바이트가 부족하거나 초과하면 잘라내거나 패딩하지 않고 거부한다.
    if len(pixels) != width * height:
        raise _invalid(f'pgm raster has {len(pixels)} bytes, expected {width * height}')
    return width, height, pixels, maxval


def encode_png(width, height, pixels):
    """8비트 그레이스케일 PNG 인코딩. 모든 행에 필터 타입 0(None)을 적용한다."""
    if len(pixels) != width * height:
        raise _invalid('pixel count does not match the size')
    raw = b''.join(b'\x00' + pixels[row * width:(row + 1) * width] for row in range(height))

    def chunk(kind, body):
        # PNG 청크: 빅엔디안 본문 길이, 4바이트 청크 타입, 본문, 타입+본문 CRC-32 체크섬.
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))

    # IHDR: 너비, 높이, 비트 심도 8, 컬러 타입 0(그레이스케일), 압축 0, 필터 0, 인터레이스 없음.
    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return PNG_SIGNATURE + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')


@dataclass(frozen=True)
class MapImage:
    """불변 지도 데이터: /map.png로 제공할 PNG 바이너리와 화면 모델용 `meta` 딕셔너리.

    `meta`(키 구성은 load_map 참조)에는 이미지 바이너리를 제외하여 폴링 화면 JSON 크기를 작게 유지한다.
    """
    meta: dict      # 화면 모델 탑재용 메타데이터(바이너리 제외)
    png: bytes


def _read(path):
    try:
        return path.read_bytes()
    except OSError as exc:
        raise ConsoleError('STORAGE_ERROR', f'map file unreadable: {path.name}: {exc.strerror}') from exc


def load_map(yaml_path):
    """map_server 지도 파일을 1회 로드한다. PGM 원본을 유지하며 negate 설정 시 점유 반전만 처리한다."""
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
        # yaw 회전이 적용된 origin은 회전된 이미지가 필요하므로 오정렬을 방지하기 위해 거부한다.
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
    # 상대 경로 이미지 파일명은 map_server 사양에 맞춰 YAML 파일이 위치한 디렉터리를 기준으로 해석한다.
    image_path = image_path if image_path.is_absolute() else yaml_path.parent / image_path
    pgm_bytes = _read(image_path)
    width, height, pixels, maxval = read_pgm(pgm_bytes)
    # occupied_thresh/free_thresh/mode 파라미터는 수용하되 픽셀에는 적용하지 않는다. 픽셀은 원본 값을 유지하며,
    # 0..255 스케일링 및 negate 반전만 수행한다. map_snapshot 파일은 이미 삼치(trinary) 값이다
    # (0 점유, 254 빈 공간, 205 미탐색 영역).
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
        # YAML에는 frame 정보가 없다: 본 지도를 MAP_FRAME으로 간주하고 해당 frame 좌표만 표시한다(map_overlay).
        'frame_id': MAP_FRAME,
        'mode': spec.get('mode'),
        'yaml_sha256': hashlib.sha256(yaml_bytes).hexdigest(),
        'pgm_sha256': pgm_sha256,
        # 캐시 무효화 키: 재시작 후 지도 파일이 변경되면 URL도 변경되어 브라우저 캐시를 갱신한다.
        'url': f'map.png?v={pgm_sha256[:12]}',
    }
    return MapImage(meta=meta, png=encode_png(width, height, pixels))


class LiveMap:
    """--live-map: SLAM 지도 파일(rescue_operations map_snapshot)이 갱신될 때마다 다시 로드한다.

    YAML 수정 시각을 지도 수신 시각으로 본다. 해당 시각과
    ``live_map.alive``(지도 토픽 발행자가 유지되는 동안 갱신되므로, 재발행이 없는 정적 지도도
    최신 상태로 유지됨, OPS-117/118)가 모두 ``stale_s``를 초과하면 STALE로 판정한다.
    지도가 없으면 MISSING, 로드 실패 시 ERROR 상태가 되며, 마지막 정상 로드 이미지를 유지하면서
    오류 상태를 화면에 함께 보고한다. 이전 이미지를 최신 정상 상태인 것처럼 표시하지 않는다.
    """

    def __init__(self, yaml_path, *, stale_s, clock=time.time_ns):
        """경로와 낡음 판정 기준만 등록한다. 최초 current() 호출 전에는 파일을 읽지 않는다.

        stale_s는 기본값이 없고(현장 환경에 따라 지정) 유한한 양수여야 하며, 위반 시 INVALID_INPUT.
        `clock`은 파일 mtime과 비교하므로 동일 호스트 기준 벽시계 ns를 사용한다. 테스트에서는 모의 시계를 주입한다.
        """
        if type(stale_s) not in (int, float) or not math.isfinite(stale_s) or stale_s <= 0:
            raise _invalid('live map stale seconds must be a positive number')
        self.path, self.stale_ns, self.clock = Path(yaml_path), int(stale_s * 1e9), clock
        # 마지막 정상 로드 지도, 마지막 로드 파일의 (mtime_ns, size), 로드 오류 메시지(성공 시 None).
        self.image, self._key, self._error = None, None, None
        self._lock = threading.Lock()

    def current(self):
        """(MapImage 또는 None, {'state': OK|STALE|ERROR|MISSING, 'received_at_ns', 'age_s', 'message'}) 반환."""
        with self._lock:
            try:
                stat = self.path.stat()
            except OSError as exc:
                if self.image is None:
                    return None, {'state': 'MISSING', 'received_at_ns': None, 'age_s': None,
                                  'message': f'실시간 지도 없음 — {self.path.name} 를 아직 받지 못함'}
                return self.image, {'state': 'ERROR', 'received_at_ns': None, 'age_s': None,
                                    'message': f'지도 읽기 실패: {exc.strerror} — 마지막으로 읽은 지도를 보임'}
            # map_snapshot은 원자적 rename으로 YAML을 교체하므로 (mtime, size) 변경은 쓰기 완료를 의미한다.
            key = (stat.st_mtime_ns, stat.st_size)
            if key != self._key:
                self._key = key     # 로드에 실패해도 키를 기록하여 매 폴링마다 손상된 파일을 재해석하지 않는다
                try:
                    self.image, self._error = load_map(self.path), None
                except ConsoleError as exc:
                    # self.image는 마지막 정상 로드 지도를 유지하고, 오류는 별도로 기록해 화면에 알린다.
                    self._error = exc.detail
            received = stat.st_mtime_ns
            age_s = round((self.clock() - received) / 1e9, 1)
            # ERROR를 STALE보다 먼저 검사한다: 파일 mtime이 최신이어도 읽기 실패는 즉시 보고한다.
            if self._error is not None:
                kept = ' — 마지막으로 읽은 지도를 보임' if self.image is not None else ''
                return self.image, {'state': 'ERROR', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 읽기 실패: {self._error}{kept}'}
            try:
                alive = (self.path.parent / ALIVE_FILE).stat().st_mtime_ns
            except OSError:
                alive = received    # 유효한 alive 파일이 없으면 지도 파일 자체의 수정 시각만으로 판정한다
            # 지도 토픽의 최종 수신 신호: 최신 지도 수신 시각과 발행자 생존 touch 시각 중 더 늦은 쪽.
            heard = max(received, alive)
            if self.clock() - heard > self.stale_ns:
                silent = round((self.clock() - heard) / 1e9, 1)
                return self.image, {'state': 'STALE', 'received_at_ns': received, 'age_s': age_s,
                                    'message': f'지도 낡음: {silent:g}초 동안 지도 발행 없음'}
            return self.image, {'state': 'OK', 'received_at_ns': received, 'age_s': age_s, 'message': ''}


def world_to_pixel(meta, x, y):
    """map frame 좌표(미터) -> 이미지 좌표(좌측 기준 열, 상단 기준 행) 연속값 변환.

    `origin`은 좌하단 그리드의 좌하단 모서리이므로(map_server), 원점은
    (0, height), 반대편 대각 모서리는 (width, 0)에 위치한다. 이미지 경계 밖이면 None을 반환한다.
    """
    # 경계 검사 전 1/100 픽셀 단위로 반올림하여 반대편 경계에 걸친 좌표가
    # 부동소수점 오차로 탈락하지 않도록 보호한다(예: (0.3 - 0.1) / 0.05 = 3.9999999999999996).
    column = round((x - meta['origin'][0]) / meta['resolution'], 2)
    row = round(meta['height_px'] - (y - meta['origin'][1]) / meta['resolution'], 2)
    if not (0 <= column <= meta['width_px'] and 0 <= row <= meta['height_px']):
        return None
    return column, row


EXIT_NAME = '출구(QR)'


def map_overlay(meta, robots, targets, exit_=None, passages=()):
    """화면 모델의 `map` 영역 생성: MAP_FRAME과 일치하고 이미지 범위 내에 위치한 개체 마커 표시.

    robots/targets는 화면 모델 행 목록이며(`pose`/`position`은 None 가능), exit_는 최신 출구(QR) 관측이다.
    타 frame 좌표는 `frame_mismatch`에 분리하고 투영하지 않는다. frame은 맞으나 이미지 범위 밖인 좌표는
    임의로 경계에 맞추지 않고 `off_map`에, 위치 미확인 로봇은 `no_pose`에 분류한다. 출구 마커는 `exits`에 포함된다.
    """
    overlay = dict(meta, available=True, robots=[], targets=[], exits=[], passages=[], off_map=[], frame_mismatch=[], no_pose=[],
                   note=f'기준 지도 {meta["source"]} ({meta["width_px"]}×{meta["height_px"]} px, '
                        f'{meta["resolution"]} m/px) — 좌표 map_session 일치는 확인하지 않음(Q-09)')
    entries = [('robot', row['robot_id'], row.get('pose'), {'yaw': (row.get('pose') or {}).get('yaw'),
                                                            'role': row.get('role'), 'name': row.get('name')}) for row in robots]
    entries += [('target', row['target_id'], row.get('position'), {'stage': row['stage']}) for row in targets]
    if exit_ is not None:   # 가장 최근 관측된 exit_qr 관측치(#237)
        entries.append(('exit', EXIT_NAME, exit_, {'observed_at_ns': exit_['observed_at_ns'],
                                                   'map_revision': exit_['map_revision']}))
    entries += [('passage', row['id'], row, {'state': row['state'], 'label': row['label'], 'current': row['current'],
                                              'robot_id': row['robot_id'], 'map_revision': row['map_revision']})
                for row in passages]   # 터널 판정(#262, OPS-189)
    for kind, ident, pose, extra in entries:
        if pose is None:
            if kind == 'robot':   # 목록에만 포함하고 지도에 그리지 않음: 미확인 위치를 임의 생성하지 않음(지도 확정 전 TB-5)
                overlay['no_pose'].append({'id': ident, 'name': extra['name'] or ident})
            continue
        where = {'kind': kind, 'id': ident, 'x': pose['x'], 'y': pose['y'], 'frame_id': pose['frame_id'],
                 'label': extra.get('label')}
        if pose['frame_id'] != MAP_FRAME:
            overlay['frame_mismatch'].append(where)
            continue
        pixel = world_to_pixel(meta, pose['x'], pose['y'])
        if pixel is None:
            overlay['off_map'].append(where)
            continue
        overlay[kind + 's'].append(dict(extra, id=ident, px=pixel[0], py=pixel[1]))
    return overlay
