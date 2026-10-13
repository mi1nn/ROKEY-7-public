"""지진 판정: 웹캠 화면 '전체'가 짧은 시간에 앞뒤로 여러 번 흔들리면 지진 사건(DISASTER_SIGN)을 하나 낸다.

생각하는 방법
    카메라가 흔들리면 화면 속 모든 것이 같은 방향으로 같이 밀린다.
    사람이 지나가면 화면의 일부만 움직이고, 조명이 바뀌면 밝기만 바뀐다.
    그래서 "화면 전체가 같은 방향으로 움직였나"와 "그 방향이 3초 안에 4번 이상 바뀌었나"를 본다.

프레임 한 장마다 하는 일
    1. 화면을 작은 회색 격자로 줄이고 4×4 = 16개 블록으로 나눈다.
    2. 블록마다 직전 프레임과 비교해 얼마나 밀렸는지(이동량)를 찾는다.
    3. 판단할 수 없는 화면이면 UNKNOWN(판단 보류)을 돌려준다.
       - 무늬 있는 블록이 너무 적다 (렌즈가 막힘, 너무 어두움)
       - 흔들리기 전 원래 장면이 거의 안 보인다 (손으로 가림)
       - 직전 프레임과 맞춘 블록이 너무 적다 (너무 크게 흔들림)
       판단 보류는 지금까지 센 횟수를 더하지도 지우지도 않는다.
    4. 네 사분면이 모두 같은 방향으로 움직였을 때만 '화면 전체 이동'으로 인정한다.
    5. 이동 방향이 직전과 반대면 '방향 전환' 1회. 3초 안에 4회 이상이면 지진 조건이 참이다.
    6. 조건이 0.2초 이어지면 사건 1건. 사건 뒤에는 2초 동안 조용해야 다시 낼 수 있다.

숫자(3초, 4회 등)는 코드에 없고 모두 YAML 설정(DisasterSignPolicy)에서 받는다.
이 파일은 판정만 한다. 사건을 받아 재난 모드로 바꾸는 일은 조정기가 한다.
numpy·OpenCV 없이 순수 Python 이라 ROS 없이 바로 시험할 수 있다.
"""
from dataclasses import dataclass
import math
from statistics import median
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns

from .adapter import SourceBinding, rgb_frame
from .core import PerceptionError

# 용어
#   격자      : 프레임에서 8 픽셀마다 한 점씩 뽑은 회색 밝기 표. 640×480 화면 → 80열 × 60행
#   블록      : 격자 가장자리 여백(3칸)을 뺀 나머지를 4×4 로 나눈 한 칸. 블록마다 이동량을 따로 잰다
#   칸        : 거리 단위. 1칸 = 격자 한 칸 = 8 픽셀
#   기준 장면 : 흔들리기 전의 화면. 지금 화면에 이 장면이 보이는지로 '가려졌나'를 판단한다
#   방향 전환 : 이번 이동이 직전 이동과 반대 방향인 것
#
# 예시로 따라가기 (0.2초마다 1장)
#   t=0.0s  첫 프레임. 비교할 것이 없으니 기준 장면으로 저장만 한다
#   t=0.2s  카메라가 오른쪽으로 밀림. 16블록 대부분이 (+2, 0) 이동 → 네 사분면 모두 같음 → 화면 이동 (+2, 0)
#   t=0.4s  왼쪽으로 밀림 (-2, 0). 직전 (+2, 0) 과 반대 → 방향 전환 1회
#   t=0.6s ~ 1.0s  오른쪽·왼쪽 반복 → t=1.0s 에 전환 4회 → 지진 조건 참
#   t=1.2s  조건이 0.2초 이어짐 → DISASTER_SIGN 사건 1건
#   사람이 지나가면  → 일부 블록만 움직여 네 사분면이 같은 이동에 동의하지 않음 → 정상
#   손으로 가리면    → 원래 장면이 보이는 블록이 30% 미만 → 판단 보류(UNKNOWN), 센 횟수는 그대로

EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/disaster_sign/v1")
# 컬러 → 회색 밝기 계산에 쓰는 가중치(표준 값). bgr8 은 채널 순서가 거꾸로라 가중치도 거꾸로 둔다.
_LUMA = {"rgb8": (0.299, 0.587, 0.114), "bgr8": (0.114, 0.587, 0.299)}
_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1}


# 아래 네 함수는 값 검사 도우미. 조건이 틀리면 PerceptionError 를 낸다.
def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _contract(check, value, label):
    try:
        return check(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _finite(value, name, low, high):
    _require(type(value) in (int, float) and math.isfinite(value) and low <= value <= high,
             "INVALID_INPUT", name)


def _count(value, name, low):
    _require(type(value) is int and value >= low, "INVALID_INPUT", name)


@dataclass(frozen=True)
class DisasterSignPolicy:
    """지진 판정에 쓰는 숫자 모음. 모두 YAML 에서 받고 기본값은 없다. (괄호 안은 예시 YAML 값)

    거리 단위 '칸' = 격자 한 칸 = sample_stride 픽셀.
    - sample_stride (8)          : 격자를 만들 때 몇 픽셀마다 한 점을 뽑을지
    - blocks_x / blocks_y (4, 4) : 격자를 나눌 블록 수. 사분면을 나누려면 짝수여야 한다
    - search_cells (3)           : 한 프레임 사이에 찾아볼 최대 이동(칸). ±3칸
    - texture_min (4.0)          : 블록 밝기 표준편차가 이 값 이상이면 '무늬 있음'
    - match_min (0.6)            : 직전 프레임과 비교 점수가 이 값 이상이면 '맞췄다'
    - valid_ratio_min (0.5)      : 무늬 있는 블록·맞춘 블록이 이 비율보다 적으면 판단 보류
    - agree_tol (1.0)            : 화면 이동과 '같은 이동'으로 쳐 주는 오차(칸)
    - shift_min (1.0)            : 이보다 작은 이동은 잡음으로 보고 방향을 기록하지 않는다
    - scene_match_min (0.6)      : 블록이 기준 장면과 이 점수 이상으로 맞으면 '원래 장면이 보인다'
    - visible_min (0.3)          : 원래 장면이 보이는 블록이 이 비율보다 적으면 판단 보류(가려짐)
    - rebase_ns (10 s)           : 바뀐 화면이 이 시간 넘게 그대로면 카메라를 옮긴 것으로 보고 기준 장면을 바꾼다
    - shakes_min (4)             : 시간 창 안 방향 전환이 이 횟수 이상이면 지진 조건 참
    - window_ns (3 s)            : 방향 전환을 세는 시간 창
    - hold_ns (0.2 s)            : 조건이 이만큼 이어져야 사건을 낸다
    - release_ns (2 s)           : 사건 뒤 조건이 이만큼 거짓이어야 다시 사건을 낼 수 있다
    - max_gap_ns (0.5 s)         : 프레임 사이가 이보다 벌어지면 센 것을 모두 버리고 처음부터
    """
    sample_stride: int
    blocks_x: int
    blocks_y: int
    search_cells: int
    texture_min: float
    match_min: float
    valid_ratio_min: float
    agree_tol: float
    shift_min: float
    scene_match_min: float
    visible_min: float
    rebase_ns: int
    shakes_min: int
    window_ns: int
    hold_ns: int
    release_ns: int
    max_gap_ns: int

    # 값 범위 검사. 하나라도 틀리면 노드가 기동하지 않는다.
    def __post_init__(self):
        _count(self.sample_stride, "sample_stride", 1)
        for name in ("blocks_x", "blocks_y"):
            _count(getattr(self, name), name, 2)
            _require(getattr(self, name) % 2 == 0, "INVALID_INPUT", f"{name} must be even (four quadrants)")
        _count(self.search_cells, "search_cells", 1)
        _finite(self.texture_min, "texture_min", 0, 255)
        _require(self.texture_min > 0, "INVALID_INPUT", "texture_min must be positive")
        _finite(self.match_min, "match_min", -1, 1)
        _finite(self.valid_ratio_min, "valid_ratio_min", 0, 1)
        _require(self.valid_ratio_min > 0, "INVALID_INPUT", "valid_ratio_min must be positive")
        _finite(self.agree_tol, "agree_tol", 0, 2 * self.search_cells)
        _finite(self.shift_min, "shift_min", 0, 2 * self.search_cells)
        _require(self.shift_min > 0, "INVALID_INPUT", "shift_min must be positive")
        _finite(self.scene_match_min, "scene_match_min", -1, 1)
        _finite(self.visible_min, "visible_min", 0, 1)
        _count(self.shakes_min, "shakes_min", 1)
        for name in ("window_ns", "hold_ns", "release_ns", "max_gap_ns", "rebase_ns"):
            _contract(duration_ns, getattr(self, name), name)
        _require(self.window_ns > 0 and self.max_gap_ns > 0, "INVALID_INPUT",
                 "window_ns/max_gap_ns must be positive")
        _require(self.hold_ns <= self.window_ns, "INVALID_INPUT", "hold_ns must not exceed window_ns")


@dataclass(frozen=True)
class SignFrame:
    """프레임 한 장의 판정 결과. webcam_node 가 이것을 보고 카메라 상태와 사건을 발행한다.

    - status   : PROCESSED(판정함), UNKNOWN(판단 보류), DUPLICATE·REORDERED(같은·이른 시각이라 무시)
    - reason   : 방향을 기록하지 않은 이유. 기록했으면 None
                 FIRST_FRAME      첫 프레임
                 OCCLUDED         무늬 있는 블록이 너무 적음 (렌즈 막힘, 어두움)
                 SCENE_CHANGED    원래 장면이 거의 안 보임 (손 가림)
                 QUADRANT_COVERED 절반도 안 보이면서 한 사분면이 통째로 가려짐
                 MATCH_FAILED     직전 프레임과 맞추지 못함 (너무 크게 흔들림)
                 PARTIAL_MOTION   화면 일부만 움직임 (사람 통과)
                 BELOW_NOISE      움직임이 너무 작음 (잡음)
    - shift    : 화면 전체 이동 (dx, dy) 칸. 재지 못했으면 None
    - reversals: 최근 3초 안의 방향 전환 횟수
    - condition: 지진 조건이 참인가
    - event    : 이번 프레임에서 낸 사건. 없으면 None
    - textured_ratio / matched_ratio / visible_ratio: 무늬 있는 / 직전과 맞춘 / 원래 장면이 보이는 블록 비율(측정 도구용)
    """
    status: str
    reason: str | None
    shift: tuple | None
    reversals: int
    condition: bool
    event: SiteEvent | None
    textured_ratio: float | None = None
    matched_ratio: float | None = None
    visible_ratio: float | None = None


def _gray_grid(image, stride):
    """프레임 → 회색 밝기 격자(2차원 리스트). stride 픽셀마다 한 점만 뽑아 계산량을 약 1/64 로 줄인다."""
    _require(image.encoding in _CHANNELS, "INVALID_INPUT", f"unsupported encoding {image.encoding!r}")
    channels = _CHANNELS[image.encoding]
    data = image.data.tobytes() if hasattr(image.data, "tobytes") else bytes(image.data)
    _require(image.step >= image.width * channels, "INVALID_INPUT", "step shorter than row")
    weights = _LUMA.get(image.encoding)
    grid = []
    for y in range(0, image.height, stride):
        # data 는 픽셀이 한 줄로 이어진 바이트. (x, y) 픽셀의 첫 바이트 위치 = y * step + x * channels
        base, row = y * image.step, []
        for x in range(0, image.width, stride):
            i = base + x * channels
            # 밝기 = 컬러면 세 채널에 가중치를 곱해 더한 값, 흑백(mono8)이면 그 한 채널 값 그대로
            row.append(data[i] if weights is None else
                       weights[0] * data[i] + weights[1] * data[i + 1] + weights[2] * data[i + 2])
        grid.append(row)
    return grid


def _shape(grid):
    return len(grid), len(grid[0])


def _block_size(grid, policy):
    """블록 한 개의 (높이, 너비) 칸 수.

    격자 가장자리에 search_cells 만큼 여백을 남기고 나머지를 4×4 로 나눈다.
    여백이 있어야 ±3칸 옮겨 볼 때 격자 밖으로 나가지 않는다.
    예) 80×60 격자, 여백 3 → 74×54 → 블록 하나 = 18열 × 13행
    """
    s = policy.search_cells
    rows, cols = len(grid) - 2 * s, len(grid[0]) - 2 * s
    bh = rows // policy.blocks_y if rows > 0 else 0
    bw = cols // policy.blocks_x if cols > 0 else 0
    _require(bh >= 2 and bw >= 2, "INVALID_INPUT", "image too small for sample_stride/blocks/search_cells")
    return bh, bw


def _offsets(search):
    """찾아볼 이동 후보 (dx, dy) 전부. search 3 이면 -3~3 × -3~3 = 49개.

    작은 이동부터 정렬한다. 점수가 같으면 덜 움직인 쪽을 고르게 하려는 것이다.
    """
    span = range(-search, search + 1)
    return sorted(((dx, dy) for dy in span for dx in span),
                  key=lambda d: (d[0] ** 2 + d[1] ** 2, d[1], d[0]))


def _block_shifts(prev, cur, policy):
    """블록 16개 각각이 직전 프레임에서 얼마나 밀렸는지 찾는다.

    돌려주는 값: 블록마다 (bx, by, 무늬있음, dx, dy, 점수). 무늬가 없으면 dx, dy, 점수는 None.
    (dx, dy) 는 '지금 블록이 직전 프레임에서 어디에 있었나' = 카메라가 움직인 방향이다.
    """
    s = policy.search_cells
    bh, bw = _block_size(cur, policy)
    n, offsets, out = bh * bw, _offsets(s), []
    for by in range(policy.blocks_y):
        for bx in range(policy.blocks_x):
            # (y0, x0) = 이 블록의 왼쪽 위 칸. 여백 s 칸 다음부터 블록 크기만큼씩 건너뛴다.
            y0, x0 = s + by * bh, s + bx * bw
            # a = 블록 안 밝기 값들을 한 줄로 편 목록에서 평균을 뺀 것. norm_a = a 의 크기.
            a = [cur[y][x] for y in range(y0, y0 + bh) for x in range(x0, x0 + bw)]
            mean = sum(a) / n
            a = [v - mean for v in a]
            norm_a = math.sqrt(sum(v * v for v in a))
            # 밝기 표준편차(norm_a / √n)가 작으면 흰 벽·어둠처럼 무늬가 없는 블록이다.
            # 어디로 옮겨도 똑같아 보여서 이동을 잴 수 없으므로 '무늬 없음'으로 두고 넘어간다.
            if norm_a / math.sqrt(n) < policy.texture_min:
                out.append((bx, by, False, None, None, None))
                continue
            out.append((bx, by, True) + _best_zncc(prev, a, norm_a, y0, x0, bh, bw, offsets))
    return out


def _best_zncc(prev, a, norm_a, y0, x0, bh, bw, offsets):
    """블록 a 를 직전 프레임(prev)의 여러 위치와 비교해 가장 닮은 위치 (dx, dy, 점수)를 찾는다.

    점수 ZNCC: 두 조각에서 각각 평균을 빼고, 크기로 나눈 뒤, 같은 자리끼리 곱해 더한 값. -1 ~ 1.
    평균을 빼고 크기로 나누기 때문에 밝기가 통째로 바뀌어도 무늬가 같으면 점수가 같다.
    → 조명을 켜고 꺼도 '움직였다'고 착각하지 않는다.
        원래 조각      50, 100, 150
        불을 켬(2배)  100, 200, 300  → 점수 1.0 (같은 무늬)
        다른 장면     150,  50, 100  → 점수 -0.5
    비교할 수 있는 위치가 하나도 없으면 점수 -2 를 돌려준다.
    """
    n, best = bh * bw, (None, None, -2.0)
    for dx, dy in offsets:
        # b = 직전 프레임에서 (dx, dy) 만큼 옮긴 자리의 같은 크기 조각
        b = [prev[y][x] for y in range(y0 + dy, y0 + dy + bh) for x in range(x0 + dx, x0 + dx + bw)]
        sb = sum(b)
        var_b = sum(v * v for v in b) - sb * sb / n  # b 가 흩어진 정도(분산 × n)
        if var_b <= 1e-9:  # 완전히 평평한 조각은 비교할 수 없다
            continue
        # a 는 이미 평균이 0 이라 b 의 평균을 빼지 않아도 결과가 같다
        score = sum(p * q for p, q in zip(a, b)) / (norm_a * math.sqrt(var_b))
        if score > best[2]:  # 더 높을 때만 바꾼다 → 같은 점수면 먼저 본 작은 이동이 남는다
            best = (dx, dy, score)
    return best


def _visible_blocks(ref, cur, policy):
    """기준 장면(흔들리기 전 화면)이 아직 보이는 블록들의 (bx, by) 집합.

    블록마다 기준 장면의 ±3칸 안에서 가장 닮은 점수가 scene_match_min(0.6) 이상이면 '보인다'.
    직전 프레임이 아니라 기준 장면과 비교하는 이유: 손이 렌즈 앞에 머물면 직전 프레임에도 손이 있어서
    '잘 맞는다'고 나온다. 흔들리기 전 장면과 비교해야 '원래 보이던 것이 지금도 보이나'를 알 수 있다.
    """
    s = policy.search_cells
    bh, bw = _block_size(cur, policy)
    n, offsets, visible = bh * bw, _offsets(s), set()
    for by in range(policy.blocks_y):
        for bx in range(policy.blocks_x):
            y0, x0 = s + by * bh, s + bx * bw
            a = [cur[y][x] for y in range(y0, y0 + bh) for x in range(x0, x0 + bw)]
            mean = sum(a) / n
            a = [v - mean for v in a]
            norm_a = math.sqrt(sum(v * v for v in a))
            if norm_a > 1e-9 and _best_zncc(ref, a, norm_a, y0, x0, bh, bw, offsets)[2] >= policy.scene_match_min:
                visible.add((bx, by))
    return visible


def _whole_view(matched, policy):
    """블록 이동량들로 '화면 전체 이동' (dx, dy) 를 정한다. 일부만 움직였으면 None.

    1. 블록 이동량의 중앙값을 화면 이동으로 본다(일부 블록이 튀어도 다수를 따른다).
    2. 네 사분면 모두에 그 이동과 비슷한(±agree_tol) 블록이 하나 이상 있어야 인정한다.
    예) 카메라 흔들림: 거의 모든 블록이 (+2, 0) → 중앙값 (+2, 0), 네 사분면 모두 있음 → (+2, 0)
        사람이 왼쪽 절반을 덮고 움직임: 왼쪽 8블록 (+3, 0), 오른쪽 8블록 (0, 0)
        → 중앙값 x = 1.5 → 어느 블록도 1.5 ± 1 안에 없음 → None
    """
    mx, my = median(b[3] for b in matched), median(b[4] for b in matched)
    # 사분면 번호: (bx * 2 // blocks_x, by * 2 // blocks_y) → (0,0) (1,0) (0,1) (1,1) 중 하나
    agreeing = {(b[0] * 2 // policy.blocks_x, b[1] * 2 // policy.blocks_y) for b in matched
                if abs(b[3] - mx) <= policy.agree_tol and abs(b[4] - my) <= policy.agree_tol}
    return (mx, my) if len(agreeing) == 4 else None


class DisasterSignDetector:
    """지진 판정기. 카메라 1대에 하나. update() 에 프레임을 시간 순서대로 넣는다.

    흔들림 한 번에 사건은 최대 1건이다. 같은 시각이나 더 이른 시각의 프레임은 아무것도 바꾸지 않는다.
    """

    def __init__(self, binding, policy):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, DisasterSignPolicy), "INVALID_INPUT", "policy")
        self.camera_id = _contract(camera_id, binding.source_camera, "camera_id")
        self.binding, self.policy = binding, policy
        # 프레임 사이에 기억하는 값들
        self._last_ns = None        # 마지막으로 처리한 프레임 시각
        self._prev_grid = None      # 직전 프레임 격자 (이동량 비교용)
        self._ref_grid = None       # 기준 장면 = 흔들리기 전 화면 (가림 판단용)
        self._still_since = None    # 화면이 멈춰 있기 시작한 시각 (기준 장면 갱신용)
        self._changed_since = None  # 가려진 화면이 그대로 이어지기 시작한 시각 (카메라 옮김 판단용)
        self._last_dir = None       # 마지막으로 기록한 이동 방향 (dx, dy)
        self._reversal_ns = []      # 최근 3초 안 방향 전환이 일어난 시각들
        self._true_since = None     # 지진 조건이 참이 된 시각
        self._false_since = None    # 지진 조건이 거짓이 된 시각
        self._armed = True          # 지금 사건을 낼 수 있는가 (사건 뒤에는 False)

    def _reset_motion(self):
        self._prev_grid, self._ref_grid, self._last_dir, self._reversal_ns = None, None, None, []
        self._still_since, self._changed_since = None, None
        self._true_since, self._false_since = None, None

    def update(self, rgb):
        """★ 프레임 한 장 판정. 위에서부터 '판단할 수 없는 경우'를 걸러 내고, 남은 프레임에서만 방향 전환을 센다."""
        # 입력 형식(encoding·크기·시각)을 검사하고, 증거로 쓸 원 픽셀의 sha256 을 구한다.
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        stamp = frame.observed_at_ns
        # 같은 시각(DUPLICATE)·더 이른 시각(REORDERED) 프레임은 무시한다.
        if self._last_ns is not None and stamp <= self._last_ns:
            return SignFrame("DUPLICATE" if stamp == self._last_ns else "REORDERED", None, None,
                             len(self._reversal_ns), False, None)
        # 격자 만들기와 크기 검사를 상태를 바꾸기 전에 한다. 잘못된 프레임은 아무 흔적도 남기지 않는다.
        grid = _gray_grid(rgb, self.policy.sample_stride)
        _block_size(grid, self.policy)
        if self._last_ns is not None and stamp - self._last_ns > self.policy.max_gap_ns:
            self._reset_motion()  # 프레임이 한동안 끊겼다: 그 사이를 모르므로 처음부터
        elif self._prev_grid is not None and _shape(grid) != _shape(self._prev_grid):
            self._reset_motion()  # 해상도가 바뀌었다: 직전 프레임과 비교할 수 없으므로 처음부터
        self._last_ns = stamp
        prev, self._prev_grid = self._prev_grid, grid  # 다음 프레임은 이번 프레임과 비교한다(보류여도)
        # 3초 창을 벗어난 방향 전환 기록은 지운다 → 남은 개수 = 최근 3초 안의 전환 횟수
        self._reversal_ns = [t for t in self._reversal_ns if t > stamp - self.policy.window_ns]
        if prev is None:
            self._ref_grid = grid  # 첫 프레임: 기준 장면으로 저장만 한다
            return self._judge(frame, evidence_sha256, "FIRST_FRAME", None, None, None)

        # 블록 16개의 이동량. textured = 무늬 있는 블록, matched = 그중 직전 프레임과 잘 맞은 블록
        blocks = _block_shifts(prev, grid, self.policy)
        textured = [b for b in blocks if b[2]]
        matched = [b for b in textured if b[5] >= self.policy.match_min]
        textured_ratio, matched_ratio = len(textured) / len(blocks), len(matched) / len(blocks)
        vmin = self.policy.valid_ratio_min
        # 아래 UNKNOWN 들은 '이번 프레임은 판단하지 않는다'는 뜻이다. 센 횟수와 시간 기록을 건드리지 않는다.
        # 보류 ① 무늬 있는 블록이 절반 미만: 렌즈가 막혔거나 너무 어둡다
        if textured_ratio < vmin:
            self._still_since, self._changed_since = None, None
            return SignFrame("UNKNOWN", "OCCLUDED", None, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio)
        visible = _visible_blocks(self._ref_grid, grid, self.policy)
        visible_ratio = len(visible) / len(blocks)
        # 화면 이동은 원래 장면이 보이는 블록으로만 정한다. 손·사람이 덮은 블록의 움직임은 흔들림으로 세지 않는다.
        scene_blocks = [b for b in matched if (b[0], b[1]) in visible]
        shift = _whole_view(scene_blocks, self.policy) if matched_ratio >= vmin and scene_blocks else None
        # frame_shift = 덮은 물체까지 포함한 이동. 화면이 가려졌을 때 '카메라를 옮긴 건가' 판단에만 쓴다.
        frame_shift = _whole_view(matched, self.policy) if matched_ratio >= vmin and matched else None
        visible_quadrants = {(bx * 2 // self.policy.blocks_x, by * 2 // self.policy.blocks_y) for bx, by in visible}
        quadrant_hidden = len(visible_quadrants) < 4 and visible_ratio < vmin
        # 보류 ② 원래 장면이 30% 미만 보임(SCENE_CHANGED), 또는 절반도 안 보이면서 한 사분면이 통째로 가려짐
        #        (QUADRANT_COVERED, 네 사분면 확인을 할 수 없음). 덜 가려졌으면(멀리 지나가는 사람) 판정을 계속한다.
        if visible_ratio < self.policy.visible_min or quadrant_hidden:
            reason = "SCENE_CHANGED" if visible_ratio < self.policy.visible_min else "QUADRANT_COVERED"
            self._still_since = None
            if frame_shift is not None and math.hypot(*frame_shift) < self.policy.shift_min:
                # 가려진(바뀐) 화면이 움직임 없이 10초 넘게 이어지면 카메라를 옮긴 것으로 보고 새 기준 장면으로 삼는다.
                self._changed_since = stamp if self._changed_since is None else self._changed_since
                if stamp - self._changed_since >= self.policy.rebase_ns:
                    self._ref_grid, self._changed_since = grid, None
            else:
                self._changed_since = None
            return SignFrame("UNKNOWN", reason, frame_shift, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio, visible_ratio)
        self._changed_since = None
        # 보류 ③ 직전 프레임과 맞춘 블록이 절반 미만: 너무 크게 흔들려 이동을 재지 못했다
        if matched_ratio < vmin:
            self._still_since = None
            return SignFrame("UNKNOWN", "MATCH_FAILED", None, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio, visible_ratio)

        # 여기부터는 판정하는 프레임이다.
        if shift is None or math.hypot(*shift) >= self.policy.shift_min:
            self._still_since = None  # 움직였으니 '멈춰 있음' 기록을 끊는다
        if shift is None:
            # 화면 일부만 움직였다(사람 통과) → 지진 조건을 강제로 거짓으로 둔다
            return self._judge(frame, evidence_sha256, "PARTIAL_MOTION", None, textured_ratio, matched_ratio,
                               visible_ratio, force_false=True)
        if math.hypot(*shift) < self.policy.shift_min:
            # 1칸 미만 움직임은 잡음이다. 방향을 기록하지 않는다.
            self._still_since = stamp if self._still_since is None else self._still_since
            if not self._reversal_ns and stamp - self._still_since >= self.policy.window_ns:
                # 흔들림 없이 3초 넘게 멈춰 있으면 지금 화면을 새 기준 장면으로 삼는다(조명·물건 배치 변화 반영).
                # 잠깐 멈춘 손이 기준 장면이 되지 않게 '오래 멈춰 있을 때만' 바꾼다.
                self._ref_grid = grid
            return self._judge(frame, evidence_sha256, "BELOW_NOISE", shift, textured_ratio, matched_ratio,
                               visible_ratio)
        # ★ 방향 전환 세기: 이번 이동과 직전 방향의 내적(dx1·dx2 + dy1·dy2)이 음수면 반대 방향이다.
        #   예) 직전 오른쪽 (2, 0), 이번 왼쪽 (-2, 0) → 내적 -4 → 전환 1회
        if self._last_dir is not None and shift[0] * self._last_dir[0] + shift[1] * self._last_dir[1] < 0:
            self._reversal_ns.append(stamp)
        self._last_dir = shift
        return self._judge(frame, evidence_sha256, None, shift, textured_ratio, matched_ratio, visible_ratio)

    def _judge(self, frame, evidence_sha256, reason, shift, textured_ratio, matched_ratio, visible_ratio=None,
               force_false=False):
        """지진 조건을 정하고 사건을 낼지 결정한다.

        조건 = 최근 3초 안 방향 전환이 4회 이상. 조건이 0.2초 이어지면 사건 1건을 내고 '쉬는 중'(armed=False)이 된다.
        쉬는 중에는 조건이 참이어도 사건을 내지 않는다. 조건이 2초 동안 거짓이면 다시 낼 수 있게 된다.
        예) t=1.0s 전환 4회 → t=1.2s 사건 → 흔들림이 멈춰도 3초 창 안의 전환이 빠질 때까지는 조건 참
            → 전환이 4회 밑으로 내려간 때부터 2초 뒤 다시 사건을 낼 수 있음
        """
        stamp, p = frame.observed_at_ns, self.policy
        reversals = len(self._reversal_ns)
        condition = not force_false and reversals >= p.shakes_min
        event = None
        if condition:
            self._false_since = None
            self._true_since = stamp if self._true_since is None else self._true_since
            if self._armed and stamp - self._true_since >= p.hold_ns:
                self._armed = False
                event = self._event(frame, evidence_sha256, shift, reversals)
        else:
            self._true_since = None
            self._false_since = stamp if self._false_since is None else self._false_since
            if not self._armed and stamp - self._false_since >= p.release_ns:
                self._armed = True
        return SignFrame("PROCESSED", reason, shift, reversals, condition, event, textured_ratio, matched_ratio,
                         visible_ratio)

    def _event(self, frame, evidence_sha256, shift, reversals):
        """지진 사건(SiteEvent) 만들기. detail 에 판정 근거(전환 횟수·이동·설정 값)를 적어 로그로 확인할 수 있게 한다."""
        p, stamp = self.policy, frame.observed_at_ns
        detail = (f"reversals={reversals}>={p.shakes_min} window_ns={p.window_ns} "
                  f"shift={'n/a' if shift is None else f'({shift[0]},{shift[1]})'} "
                  f"onset_ns={self._true_since} hold_ns={p.hold_ns} stride={p.sample_stride} "
                  f"blocks={p.blocks_x}x{p.blocks_y} search={p.search_cells}")
        # 사건 ID 는 카메라·관측 시각으로 만든다(uuid5) → 카메라와 시각까지 같은 입력이면 같은 ID 가 나온다.
        # 노드에서는 시각이 '받은 시각'이라, 같은 영상을 다시 돌려도 시각이 다르면 ID 도 다르다.
        # confidence 1.0 은 확률이 아니라 '규칙 조건을 만족했다'는 뜻이다.
        return SiteEvent(str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/disaster")), self.camera_id,
                         "DISASTER_SIGN", stamp, str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame")),
                         evidence_sha256, 1.0, frame.clock_domain_id, frame.stamp_semantics,
                         self.binding.clock_state, self.binding.clock_error_bound_ns, detail)
