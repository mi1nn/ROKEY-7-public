"""고정 웹캠: 화면 전체 흔들림 횟수 -> SiteEvent(DISASTER_SIGN) (E2 전용).

모델·ROS·저장소·모드 전환 없음. 판정 로직은 numpy/OpenCV 없이 순수 Python 으로
쓴다(추가 설치 없이 import·시험되도록). OpenCV 는 프레임을 넣어 주는 카메라
노드·측정 도구 쪽에서만 쓴다. PR #33 설계 v3 합의(2026-10-09):

1. 회색조 격자로 줄인다(``_gray_grid``).
2. 블록마다 직전 프레임과 ZNCC(평균을 빼고 정규화한 상관)로 이동량을 찾는다.
   밝기·대비가 화면 전체로 바뀌어도 점수가 그대로라 조명 변화가 이동으로 보이지 않는다.
3. 영상 정상 확인: 무늬 있는 블록(가림·어두움)이나 매칭된 블록이 너무 적으면
   UNKNOWN. UNKNOWN 프레임은 흔들림 횟수를 더하지도 지우지도 않는다.
4. 화면 전체 확인: 원래 장면이 보이는 블록(5)의 이동량 중앙값을 화면 이동으로 보고, 네
   사분면 모두에 그와 같은 방향의 블록이 있어야 한다. 아니면 부분 움직임(사람 통과)이라 거짓.
5. 장면 확인: 카메라가 흔들리면 보이는 장면은 같고 위치만 밀린다. 렌즈 앞의 손처럼
   가까운 물체가 화면 전체를 덮으면 장면 자체가 바뀐다. 그래서 흔들리기 전 장면(기준
   장면)을 기억해 두고, 블록마다 지금 화면이 그것을 조금 옮긴 것과 ZNCC 로 맞는지 본다.
   맞는 블록이 ``visible_min`` 미만이거나 한 사분면에 하나도 없으면 가려진 것으로 보고
   UNKNOWN(SCENE_CHANGED / QUADRANT_COVERED). 손에도 무늬가
   있어 무늬 기준만으로는 가림을 못 잡기 때문이다(2026-10-10 실측).
6. 이동이 ``shift_min`` 이상이면 방향을 기록하고, 직전 방향과 반대면(내적 < 0)
   방향 전환 1회. ``window_ns`` 안에서 ``shakes_min`` 회 이상이면 조건 참.

모든 값은 DisasterSignPolicy 로 받는다(Q-11, 기본값 없음). 진도 7 은 시나리오
근거일 뿐 웹캠으로 재지 않는다. 사건은 증거이고 모드 전환은 협업·관제가 한다(계약 C-01).
"""
# 용어
#   격자      : 프레임을 sample_stride(8) 픽셀 간격으로 뽑은 회색 밝기 표. 640×480 → 80×60
#   블록      : 격자를 4×4 로 나눈 한 칸. 블록마다 이동량을 따로 잰다
#   칸        : 거리 단위. 1칸 = 격자 한 칸 = 8 픽셀
#   기준 장면 : 흔들리기 전의 화면. 지금 화면에 이 장면이 보이는지로 가림을 판단한다
#   방향 전환 : 이번 이동이 직전 이동과 반대 방향인 것. 3초 안에 4번이면 지진 조건
# 읽는 순서: DisasterSignDetector.update() → _block_shifts·_best_zncc → _visible_blocks → _whole_view → _judge

# 한 장면으로 따라가기 (예시 값, 0.2초마다 1장)
#   t=0.0s  첫 프레임 → 기준 장면으로 저장 (FIRST_FRAME)
#   t=0.2s  카메라가 오른쪽으로 밀림: 16블록 대부분이 (+2, 0) → 네 사분면 모두 동의 → 화면 이동 (+2, 0), 방향 기록
#   t=0.4s  왼쪽으로 밀림 (-2, 0) → 직전 (+2, 0) 과 반대 → 방향 전환 1회
#   t=0.6s ~ 1.0s  오른쪽·왼쪽 반복 → t=1.0s 에 전환 4회 → 조건 참
#   t=1.2s  조건이 0.2초 유지됨 → DISASTER_SIGN 사건 1건 (이후 조용해질 때까지 다시 내지 않음)
#   사람이 지나가면 → 사람이 있는 블록만 움직여 네 사분면이 같은 이동에 동의하지 않음 → 정상
#   손으로 렌즈를 가리면 → 기준 장면이 보이는 블록이 30% 미만 → UNKNOWN(판단 보류), 전환 횟수는 그대로

from dataclasses import dataclass
import math
from statistics import median
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns

from .adapter import SourceBinding, rgb_frame
from .core import PerceptionError

EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/disaster_sign/v1")
# 8비트 컬러의 회색조 가중치(ITU-R BT.601). 채널 순서는 encoding 에 따른다.
_LUMA = {"rgb8": (0.299, 0.587, 0.114), "bgr8": (0.114, 0.587, 0.299)}
_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1}


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
    """Q-11 값. 배포 프로파일이 넣는다. 기본값은 하나도 없다.

    거리 단위는 격자 칸(1칸 = ``sample_stride`` 픽셀).
    - ``blocks_x``/``blocks_y``: 격자를 나눌 블록 수. 사분면이 있어야 하므로 짝수.
    - ``search_cells``: 프레임당 찾아볼 최대 이동(칸).
    - ``texture_min``: 블록 밝기 표준편차가 이 이상이면 "무늬 있음".
    - ``match_min``: 최고 ZNCC 점수가 이 이상이면 "매칭됨".
    - ``valid_ratio_min``: 무늬 있는 블록·매칭된 블록이 각각 이 비율 이상이어야 정상 영상.
    - ``agree_tol``: 블록이 중앙값 이동과 "같은 방향"으로 인정되는 축별 허용 오차(칸).
    - ``shift_min``: 방향을 기록하는 최소 이동(칸). 정지 상태 잡음 바닥값으로 정한다.
    - ``scene_match_min``: 블록마다 지금 화면과 기준 장면의 ZNCC 가 이 이상이어야 "보임".
      기준 장면에서 ±``search_cells`` 칸 안의 가장 잘 맞는 위치로 비교한다. 흔들림 없이 ``window_ns`` 이상
      멈춰 있을 때 새로 고친다.
    - ``visible_min``: 기준 장면이 보이는 블록이 이 비율 미만이면(화면 대부분이 가려짐)
      UNKNOWN(SCENE_CHANGED). 보이는 블록이 ``valid_ratio_min`` 미만이면서 한 사분면에 하나도
      없으면 사분면 확인을 할 수 없으므로 UNKNOWN(QUADRANT_COVERED). 그보다 덜 가려지면
      (조금 떨어져 지나가는 사람) 판정을 계속한다(2026-10-10 사용자 확인).
    - ``rebase_ns``: 기준 장면과 다른 화면이 이만큼 안정적으로(잡음 수준 이동) 이어지면
      카메라가 옮겨진 것으로 보고 새 기준 장면으로 삼는다. 그보다 오래 멈춘 손은 장면이
      된다(한계).
    - ``shakes_min``/``window_ns``: 이 시간 창 안에 방향 전환이 이 횟수 이상이면 조건 참.
    - ``hold_ns``: 조건이 이만큼 이어져야 사건 1개. ``window_ns`` 이하(PR #33, 2026-10-09).
    - ``release_ns``: 사건 뒤 조건이 이만큼 거짓이어야 다시 낼 수 있다.
    - ``max_gap_ns``: 프레임 간격이 이보다 길면 횟수·구간·직전 프레임을 모두 버린다.
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

    # 값 범위 검사. 하나라도 틀리면 PerceptionError → 노드가 기동하지 않는다.
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
    """프레임 하나의 판정 결과.

    - ``status``: PROCESSED(판정함), UNKNOWN(판정 안 함), DUPLICATE, REORDERED.
    - ``reason``: 방향을 기록하지 않은 이유. FIRST_FRAME, OCCLUDED(무늬 부족),
      MATCH_FAILED(매칭 부족), SCENE_CHANGED(기준 장면이 거의 안 보임, 렌즈 앞 물체),
      QUADRANT_COVERED(절반도 안 보이면서 한 사분면에 기준 장면이 하나도 안 보임),
      PARTIAL_MOTION(일부만 움직임), BELOW_NOISE(잡음 수준).
      방향을 기록했으면 None.
    - ``shift``: 화면 전체 이동 (dx, dy) 칸. 재지 못했으면 None.
    - ``reversals``: 이 프레임 뒤 시간 창 안의 방향 전환 횟수.
    - ``textured_ratio``/``matched_ratio``/``visible_ratio``: 무늬 있는/직전 프레임과 매칭된/
      기준 장면이 보이는 블록 비율(측정 도구용).
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
    """``stride`` 픽셀 간격으로 뽑은 회색조 격자(2차원 리스트). #33 그대로."""
    _require(image.encoding in _CHANNELS, "INVALID_INPUT", f"unsupported encoding {image.encoding!r}")
    channels = _CHANNELS[image.encoding]
    data = image.data.tobytes() if hasattr(image.data, "tobytes") else bytes(image.data)
    _require(image.step >= image.width * channels, "INVALID_INPUT", "step shorter than row")
    weights = _LUMA.get(image.encoding)
    grid = []
    # stride 픽셀마다 한 점만 뽑아 회색 밝기로 바꾼다(밝기 = 세 채널 가중합). 계산량이 약 1/64 로 준다.
    for y in range(0, image.height, stride):
        base, row = y * image.step, []
        for x in range(0, image.width, stride):
            i = base + x * channels
            row.append(data[i] if weights is None else
                       weights[0] * data[i] + weights[1] * data[i + 1] + weights[2] * data[i + 2])
        grid.append(row)
    return grid


def _shape(grid):
    return len(grid), len(grid[0])


def _block_size(grid, policy):
    """블록 한 칸의 (높이, 너비). 가장자리에 ``search_cells`` 만큼 여백을 남긴다.

    여백이 있어야 어느 방향으로 찾아도 직전 프레임 밖으로 나가지 않는다.
    """
    s = policy.search_cells
    rows, cols = len(grid) - 2 * s, len(grid[0]) - 2 * s
    bh = rows // policy.blocks_y if rows > 0 else 0
    bw = cols // policy.blocks_x if cols > 0 else 0
    _require(bh >= 2 and bw >= 2, "INVALID_INPUT", "image too small for sample_stride/blocks/search_cells")
    return bh, bw


def _offsets(search):
    """찾아볼 이동 후보. 작은 이동부터 정렬해 점수가 같으면 덜 움직인 쪽을 고른다."""
    # search 가 3 이면 (-3..3) × (-3..3) = 49 개 후보.
    span = range(-search, search + 1)
    return sorted(((dx, dy) for dy in span for dx in span),
                  key=lambda d: (d[0] ** 2 + d[1] ** 2, d[1], d[0]))


def _block_shifts(prev, cur, policy):
    """블록마다 (bx, by, 무늬있음, dx, dy, 점수) 를 돌려준다.

    현재 블록을 직전 프레임에서 (dx, dy) 만큼 옮긴 자리와 비교한다. 즉 dx, dy 는
    카메라가 움직인 방향이다. 무늬가 없는 블록은 dx, dy, 점수가 None.
    """
    s = policy.search_cells
    bh, bw = _block_size(cur, policy)
    n, offsets, out = bh * bw, _offsets(s), []
    # 블록 위치: 격자 가장자리 s칸(여백)을 빼고 남은 부분을 4×4 로 나눈다. (y0, x0) = 블록의 왼쪽 위 칸.
    #   예) 640×480, stride 8 → 격자 60행×80열, s=3 → 54행×74열 → 블록 하나 = 13행 × 18열 (bh=13, bw=18)
    for by in range(policy.blocks_y):
        for bx in range(policy.blocks_x):
            y0, x0 = s + by * bh, s + bx * bw
            # a = 블록 안 밝기 값을 한 줄로 편 목록(13×18 = 234개, n = 234). norm_a = 평균을 뺀 a 의 길이(크기).
            # 현재 블록에서 평균을 뺀 값과 그 크기(표준편차 * sqrt(n))
            a = [cur[y][x] for y in range(y0, y0 + bh) for x in range(x0, x0 + bw)]
            mean = sum(a) / n
            a = [v - mean for v in a]
            norm_a = math.sqrt(sum(v * v for v in a))
            # 표준편차(= norm_a / √n)가 작으면 무늬 없는 블록(흰 벽, 어둠)이다.
            # 어디로 옮겨도 똑같아 보여 이동을 잴 수 없으므로 '무늬 없음'으로 두고 건너뛴다.
            if norm_a / math.sqrt(n) < policy.texture_min:
                out.append((bx, by, False, None, None, None))
                continue
            out.append((bx, by, True) + _best_zncc(prev, a, norm_a, y0, x0, bh, bw, offsets))
    return out


# ZNCC(평균을 뺀 정규화 상관): 두 조각에서 각각 평균을 빼고 크기로 나눈 뒤 곱해 더한 값. -1 ~ 1.
#   1 에 가까울수록 같은 무늬다. 밝기·대비가 바뀌어도 점수가 그대로라 조명 변화를 이동으로 착각하지 않는다.
#   예) [50, 100, 150] 과 불을 켜 두 배가 된 [100, 200, 300] → 1.0 / 다른 장면 [150, 50, 100] → -0.5
def _best_zncc(prev, a, norm_a, y0, x0, bh, bw, offsets):
    """평균을 뺀 조각 a 를 prev 의 (y0+dy, x0+dx) 자리들과 ZNCC 로 비교해 최고 (dx, dy, 점수).

    ZNCC 는 -1 ~ 1 이다. 비교할 수 있는 자리가 없으면 점수 -2.
    """
    n, best = bh * bw, (None, None, -2.0)
    # 후보 이동마다 직전 프레임에서 그만큼 옮긴 자리의 조각 b 를 꺼내 점수를 매긴다.
    for dx, dy in offsets:
        # b = 직전 프레임에서 (dx, dy) 만큼 옮긴 자리의 같은 크기 조각
        b = [prev[y][x] for y in range(y0 + dy, y0 + dy + bh) for x in range(x0 + dx, x0 + dx + bw)]
        sb = sum(b)
        # var_b = b 의 분산 × n (평균을 빼는 계산을 한 줄로 줄인 식)
        var_b = sum(v * v for v in b) - sb * sb / n
        if var_b <= 1e-9:  # 직전 자리가 완전히 평평하면 비교할 수 없다
            continue
        # a 는 이미 평균 0 이라 b 의 평균을 빼지 않아도 분자가 같다
        # score = (a·b) / (|a|·|b|). 같은 무늬면 1, 상관없는 무늬면 0 근처, 밝기가 뒤집힌 무늬면 -1
        score = sum(p * q for p, q in zip(a, b)) / (norm_a * math.sqrt(var_b))
        # 더 높은 점수일 때만 바꾼다 → 점수가 같으면 먼저 본(더 작은) 이동이 남는다.
        if score > best[2]:
            best = (dx, dy, score)
    return best


# 직전 프레임이 아니라 '기준 장면'과 비교하는 이유: 손이 렌즈 앞에 머물면 직전 프레임에도 손이 있어서
# 직전 프레임과는 잘 맞는다고 나온다. 흔들리기 전 장면과 비교해야 '원래 보이던 것이 지금도 보이나'를 알 수 있다.
def _visible_blocks(ref, cur, policy):
    """기준 장면이 아직 보이는 블록들의 (bx, by) 집합.

    격자를 이동량 측정과 같은 블록으로 나누고, 블록마다 기준 장면에서 ±``search_cells`` 칸 안의
    최고 ZNCC 가 ``scene_match_min`` 이상이면 "보임". 무늬가 없는 블록은 보임으로 세지 않는다.
    손처럼 무늬가 있는 물체가 가려도 원래 장면과 다르므로 "안 보임" 이 된다.
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
            # 기준 장면의 ±3칸 안에서 가장 잘 맞는 점수가 scene_match_min(0.6) 이상이면 '이 블록에는 원래 장면이 보인다'.
            if norm_a > 1e-9 and _best_zncc(ref, a, norm_a, y0, x0, bh, bw, offsets)[2] >= policy.scene_match_min:
                visible.add((bx, by))
    return visible


# 예) 카메라 흔들림: 16블록 대부분이 (+2, 0) → 중앙값 (+2, 0), 네 사분면 모두에 (+2, 0) 블록이 있음 → (+2, 0)
#     사람이 화면 왼쪽 절반을 덮고 움직임: 왼쪽 8블록 (+3, 0), 오른쪽 8블록 (0, 0) → 중앙값 x = 1.5
#     → 어느 블록도 1.5 ± 1칸 안에 없음 → 동의하는 사분면 0개 → None(부분 움직임)
def _whole_view(matched, policy):
    """매칭된 블록들로 화면 전체 이동 (dx, dy) 을 구한다. 부분 움직임이면 None.

    중앙값을 화면 이동으로 보고, 네 사분면마다 그 이동과 ``agree_tol`` 안으로
    맞는 블록이 하나 이상 있어야 한다. 사람이 화면 대부분을 가려도 멈춰 있는
    사분면이 남으므로 걸러진다.
    """
    # 중앙값을 쓰는 이유: 손·사람이 일부 블록을 끌고 가도 다수 블록의 움직임이 화면 이동이 된다.
    mx, my = median(b[3] for b in matched), median(b[4] for b in matched)
    # 중앙값과 ±agree_tol 안으로 맞는 블록이 속한 사분면을 모은다.
    # (bx * 2 // blocks_x, by * 2 // blocks_y) 는 각각 0 또는 1 → (0,0) (1,0) (0,1) (1,1) 네 사분면.
    agreeing = {(b[0] * 2 // policy.blocks_x, b[1] * 2 // policy.blocks_y) for b in matched
                if abs(b[3] - mx) <= policy.agree_tol and abs(b[4] - my) <= policy.agree_tol}
    return (mx, my) if len(agreeing) == 4 else None


class DisasterSignDetector:
    """고정 카메라 1대, 프로세스 1개. 상태는 센서 시각 순서로만 나아간다.

    한 번의 흔들림 구간에 사건은 최대 1개(hold 충족 시). release 조건을 채워야
    다시 낼 수 있다. 같은 시각·더 이른 시각 프레임은 아무것도 바꾸지 않는다.
    """

    def __init__(self, binding, policy):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, DisasterSignPolicy), "INVALID_INPUT", "policy")
        self.camera_id = _contract(camera_id, binding.source_camera, "camera_id")
        self.binding, self.policy = binding, policy
        # 판정기가 프레임 사이에 기억하는 값들. update() 가 매 프레임 읽고 고친다.
        self._last_ns = None        # 마지막으로 처리한 프레임 시각
        self._prev_grid = None      # 직전 프레임 격자(비교 기준)
        self._ref_grid = None       # 기준 장면(흔들리지 않을 때의 화면)
        self._still_since = None    # 화면이 잡음 수준으로 멈춰 있기 시작한 시각
        self._changed_since = None  # 기준 장면과 다른 화면이 안정적으로 이어지기 시작한 시각
        self._last_dir = None       # 마지막으로 기록한 이동 방향 (dx, dy)
        self._reversal_ns = []      # 시간 창 안의 방향 전환 시각들
        self._true_since = None     # 조건이 참이 된 시각(hold 계산)
        self._false_since = None    # 조건이 거짓이 된 시각(release 계산)
        self._armed = True          # 사건을 낼 수 있는 상태인가

    def _reset_motion(self):
        self._prev_grid, self._ref_grid, self._last_dir, self._reversal_ns = None, None, None, []
        self._still_since, self._changed_since = None, None
        self._true_since, self._false_since = None, None

    def update(self, rgb):
        # rgb_frame: 입력 형식(encoding·크기·시각)을 검사하고, 증거로 쓸 원 픽셀의 sha256 을 구한다.
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        stamp = frame.observed_at_ns
        # 같은 시각(DUPLICATE)·더 이른 시각(REORDERED) 프레임은 무시한다. 판정 상태는 시간 순서로만 나아간다.
        if self._last_ns is not None and stamp <= self._last_ns:
            return SignFrame("DUPLICATE" if stamp == self._last_ns else "REORDERED", None, None,
                             len(self._reversal_ns), False, None)
        # 상태를 바꾸기 전에 디코딩·크기 검사를 끝낸다: 거절된 프레임은 흔적을 남기지 않는다.
        grid = _gray_grid(rgb, self.policy.sample_stride)
        _block_size(grid, self.policy)
        if self._last_ns is not None and stamp - self._last_ns > self.policy.max_gap_ns:
            self._reset_motion()  # 빠진 프레임은 어느 쪽 증거도 아니다
        elif self._prev_grid is not None and _shape(grid) != _shape(self._prev_grid):
            self._reset_motion()  # 해상도가 바뀌면 직전 프레임과 비교할 수 없다: 처음부터 다시
        self._last_ns = stamp
        prev, self._prev_grid = self._prev_grid, grid  # UNKNOWN 이어도 다음 비교 기준은 이 프레임
        # 3초(window_ns) 창을 벗어난 방향 전환 기록은 지운다 → 남은 개수 = '최근 3초 안의 전환 횟수'.
        self._reversal_ns = [t for t in self._reversal_ns if t > stamp - self.policy.window_ns]
        # 첫 프레임: 비교할 직전 프레임이 없다. 기준 장면으로 저장만 한다.
        if prev is None:
            self._ref_grid = grid  # 처음 보는 화면을 기준 장면으로
            return self._judge(frame, evidence_sha256, "FIRST_FRAME", None, None, None)

        # 블록 16개의 이동량. textured = 무늬 있는 블록, matched = 그중 직전 프레임과 잘 맞은 블록(ZNCC ≥ match_min).
        blocks = _block_shifts(prev, grid, self.policy)
        textured = [b for b in blocks if b[2]]
        matched = [b for b in textured if b[5] >= self.policy.match_min]
        textured_ratio, matched_ratio = len(textured) / len(blocks), len(matched) / len(blocks)
        vmin = self.policy.valid_ratio_min
        # 이하 UNKNOWN 은 판정하지 않는다는 뜻: 횟수·hold·release 구간을 건드리지 않는다.
        # [보류 ①] 무늬 있는 블록이 절반 미만 → 렌즈가 막혔거나 너무 어둡다. 판단하지 않는다(UNKNOWN / OCCLUDED).
        if textured_ratio < vmin:
            self._still_since, self._changed_since = None, None
            return SignFrame("UNKNOWN", "OCCLUDED", None, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio)
        # 기준 장면이 아직 보이는 블록. 손·사람이 덮은 블록은 여기서 빠진다.
        visible = _visible_blocks(self._ref_grid, grid, self.policy)
        visible_ratio = len(visible) / len(blocks)
        # 화면 이동은 원래 장면이 보이는 블록으로만 정한다: 카메라가 흔들리면 움직이는 것은 장면이다.
        # 손·사람이 덮은 블록은 투표에서 뺀다(덮은 물체의 움직임을 흔들림으로 세지 않도록).
        scene_blocks = [b for b in matched if (b[0], b[1]) in visible]
        # shift       : 원래 장면이 보이는 블록만으로 구한 화면 이동. 판정에 쓴다.
        # frame_shift : 매칭된 블록 전부로 구한 이동. 화면이 가려졌을 때 '카메라를 옮긴 건가' 참고용이다.
        shift = _whole_view(scene_blocks, self.policy) if matched_ratio >= vmin and scene_blocks else None
        frame_shift = _whole_view(matched, self.policy) if matched_ratio >= vmin and matched else None
        # 기준 장면이 보이는 사분면. 한 사분면이 통째로 가려지면 '네 사분면이 같이 움직였나'를 확인할 수 없다.
        visible_quadrants = {(bx * 2 // self.policy.blocks_x, by * 2 // self.policy.blocks_y) for bx, by in visible}
        quadrant_hidden = len(visible_quadrants) < 4 and visible_ratio < vmin
        # [보류 ②] 원래 장면이 30% 미만 보임(SCENE_CHANGED), 또는 절반도 안 보이면서 한 사분면이 통째로 가려짐(QUADRANT_COVERED).
        if visible_ratio < self.policy.visible_min or quadrant_hidden:
            # 원래 장면이 거의 안 보이거나(손 등 가까운 물체, 카메라가 다른 곳을 봄), 절반도 안 보이면서
            # 한 구역이 통째로 가려져 네 사분면 확인을 할 수 없다: 판단하지 않는다. 화면 대부분이 보이면
            # 한 구역이 가려져도(위쪽으로 지나가는 사람, 그림자) 판정을 계속한다(2026-10-10 실측).
            reason = "SCENE_CHANGED" if visible_ratio < self.policy.visible_min else "QUADRANT_COVERED"
            self._still_since = None
            if frame_shift is not None and math.hypot(*frame_shift) < self.policy.shift_min:
                # 바뀐 화면이 rebase_ns 이상 안정적이면 카메라가 옮겨진 것으로 보고 새 기준 장면으로.
                self._changed_since = stamp if self._changed_since is None else self._changed_since
                if stamp - self._changed_since >= self.policy.rebase_ns:
                    self._ref_grid, self._changed_since = grid, None
            else:
                self._changed_since = None
            # 원래 장면이 안 보이므로 이동은 화면 전체(덮은 물체 포함)의 값을 참고로만 싣는다.
            return SignFrame("UNKNOWN", reason, frame_shift, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio, visible_ratio)
        self._changed_since = None
        # [보류 ③] 직전 프레임과 맞춘 블록이 절반 미만 → 너무 크게 흔들려 이동을 못 쟀다(UNKNOWN / MATCH_FAILED).
        if matched_ratio < vmin:
            self._still_since = None
            return SignFrame("UNKNOWN", "MATCH_FAILED", None, len(self._reversal_ns), False, None,
                             textured_ratio, matched_ratio, visible_ratio)

        # 여기부터는 판정하는 프레임이다. shift = 화면 전체 이동 (dx, dy) 칸, None 이면 일부만 움직임.
        if shift is None or math.hypot(*shift) >= self.policy.shift_min:
            self._still_since = None  # 멈춤이 끊겼다
        # 일부만 움직임(사람 통과) → 조건을 강제로 거짓으로 둔다(PARTIAL_MOTION).
        if shift is None:
            return self._judge(frame, evidence_sha256, "PARTIAL_MOTION", None, textured_ratio, matched_ratio,
                               visible_ratio, force_false=True)
        # 1칸(8 픽셀) 미만 이동은 잡음으로 본다(BELOW_NOISE). 방향을 기록하지 않는다.
        if math.hypot(*shift) < self.policy.shift_min:
            self._still_since = stamp if self._still_since is None else self._still_since
            if not self._reversal_ns and stamp - self._still_since >= self.policy.window_ns:
                # 흔들림 없이 시간 창 이상 멈춰 있을 때만 기준 장면을 새로 고친다(조명·배치 변화 반영).
                # 잠깐 멈춘 손을 기준 장면으로 삼지 않기 위해서다.
                self._ref_grid = grid
            return self._judge(frame, evidence_sha256, "BELOW_NOISE", shift, textured_ratio, matched_ratio,
                               visible_ratio)
        # ★ 방향 전환: 내적 = dx1·dx2 + dy1·dy2. 음수면 두 이동이 반대 방향이다.
        #   예) 직전 오른쪽 (2, 0), 이번 왼쪽 (-2, 0) → 내적 -4 < 0 → 전환 1회
        if self._last_dir is not None and shift[0] * self._last_dir[0] + shift[1] * self._last_dir[1] < 0:
            self._reversal_ns.append(stamp)  # 직전 방향과 반대: 방향 전환 1회
        self._last_dir = shift
        return self._judge(frame, evidence_sha256, None, shift, textured_ratio, matched_ratio, visible_ratio)

    # 시간 순 예)
    #   t=1.0s 전환이 4회가 됨 → _true_since = 1.0
    #   t=1.2s 조건이 0.2초 유지 → 사건 1건, _armed = False (더 흔들려도 사건 안 냄)
    #   흔들림이 멈춰도 3초 창 안의 전환 기록이 빠질 때까지는 조건이 참이다.
    #   전환이 4회 밑으로 내려간 때부터 2초가 지나면 _armed = True → 다음 지진은 다시 사건을 낸다.
    def _judge(self, frame, evidence_sha256, reason, shift, textured_ratio, matched_ratio, visible_ratio=None,
               force_false=False):
        """조건을 정하고 hold/release 로 사건을 낼지 정한다(#33 과 같은 규칙)."""
        stamp, p = frame.observed_at_ns, self.policy
        reversals = len(self._reversal_ns)
        # 조건: 최근 3초 안 방향 전환 ≥ shakes_min(4). 부분 움직임(force_false)이면 무조건 거짓.
        condition = not force_false and reversals >= p.shakes_min
        event = None
        if condition:
            self._false_since = None
            self._true_since = stamp if self._true_since is None else self._true_since
            # 조건이 hold(0.2 s) 동안 이어졌고 아직 사건을 안 냈으면(armed) 사건 1건. 내고 나면 armed = False.
            if self._armed and stamp - self._true_since >= p.hold_ns:
                self._armed = False
                event = self._event(frame, evidence_sha256, shift, reversals)
        else:
            self._true_since = None
            self._false_since = stamp if self._false_since is None else self._false_since
            # 사건 뒤 조건이 release(2 s) 동안 거짓이면 다시 사건을 낼 수 있게 한다(armed = True). 흔들림 한 번에 사건 1건.
            if not self._armed and stamp - self._false_since >= p.release_ns:
                self._armed = True
        return SignFrame("PROCESSED", reason, shift, reversals, condition, event, textured_ratio, matched_ratio,
                         visible_ratio)

    def _event(self, frame, evidence_sha256, shift, reversals):
        p, stamp = self.policy, frame.observed_at_ns
        # detail = 사건 근거(전환 횟수·이동·설정 값). 로그와 메시지에 남아 '왜 지진으로 봤나'를 설명한다.
        detail = (f"reversals={reversals}>={p.shakes_min} window_ns={p.window_ns} "
                  f"shift={'n/a' if shift is None else f'({shift[0]},{shift[1]})'} "
                  f"onset_ns={self._true_since} hold_ns={p.hold_ns} stride={p.sample_stride} "
                  f"blocks={p.blocks_x}x{p.blocks_y} search={p.search_cells}")
        # 규칙 기반 사건이라 확률이 아니다: 1.0 은 "정책 조건 충족" (#33 작전 결정, C-05).
        # 사건 ID = uuid5(카메라/시각). 같은 영상을 다시 돌리면 같은 ID 가 나온다(재현 가능).
        return SiteEvent(str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/disaster")), self.camera_id,
                         "DISASTER_SIGN", stamp, str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame")),
                         evidence_sha256, 1.0, frame.clock_domain_id, frame.stamp_semantics,
                         self.binding.clock_state, self.binding.clock_error_bound_ns, detail)
