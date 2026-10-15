"""webcam_node 를 돕는 작은 판단들. ROS 없이 시험할 수 있게 노드 파일에서 따로 뺐다.

1. WatchSlot        : 감시 걸기·해제 요청(SetCameraWatch)을 받아 수락할지 거절할지 정한다. 카메라당 감시는 하나.
2. stream_state     : 지진 판정 결과 → 카메라 상태(CameraStatus 의 ONLINE / DEGRADED / UNKNOWN)
3. scene_visibility : 지진 판정 결과 → 출구 판정에 넘길 시야(VALID / OCCLUDED / UNKNOWN)
4. boxes_from_xyxyn : YOLO 출력 숫자 → DetectionBox 목록
5. status_items · draw_status : 판정 화면 위 상태창(지진·인형·출구 3칸, 한글, 화면용, 판정에 안 씀)

2·3 은 '화면이 가려졌나'를 따로 재지 않고 지진 판정이 이미 잰 결과를 다시 쓴다.
한계: 사람이 감시 영역(ROI)만 살짝 가리는 경우는 화면 전체로는 보이는 상태라 VALID 로 들어간다.
후보는 확정이 아니라 로봇이 다시 확인하라는 신호라서, 이 경우는 로봇 재확인이 걸러 준다.
"""
from dataclasses import dataclass
import math

from .core import PerceptionError
from .target_watch import DetectionBox, TargetWatchDetector, WatchSpec

ASSOCIATION_METHODS = ("CALIBRATED_PROJECTION", "OPERATOR_ROI")
# 지진 판정이 '화면 대부분이 가려짐'으로 판단 보류한 이유들
_COVERED = ("OCCLUDED", "SCENE_CHANGED", "QUADRANT_COVERED")
# 좌표가 0 ~ 1 을 살짝 벗어나도 봐 주는 오차(부동소수 계산 오차)
_CLIP_TOL = 1e-3


@dataclass(frozen=True)
class WatchRequest:
    """SetCameraWatch 요청에서 판단에 쓰는 칸들.

    command_id: 요청 ID(같은 요청 재전송을 알아보는 데 씀), expected_watch_revision: 요청자가 알고 있는 지금 칸 번호,
    roi_xyxy_normalized: 감시 영역, enable: True 면 걸기 / False 면 해제, deadline_ns: 이 시각이 지나면 거절.
    """
    command_id: str
    camera_id: str
    watch_id: str
    target_id: str
    expected_watch_revision: int
    roi_xyxy_normalized: tuple
    association_method: str
    association_evidence_id: str
    enable: bool
    issued_at_ns: int
    deadline_ns: int


@dataclass(frozen=True)
class WatchReply:
    """SetCameraWatch 응답. reason_code 는 ACCEPTED 또는 거절 이유."""
    accepted: bool
    watch_id: str
    watch_revision: int
    reason_code: str
    detail: str = ""  # 거절 이유 설명(로그용). 응답 메시지에는 실리지 않는다


class WatchSlot:
    """카메라당 감시 칸 하나와 그 감시의 출구 판정기.

    revision(칸 번호)은 감시를 걸거나 해제할 때마다 1씩 오른다. 0 은 아직 아무것도 안 걸린 상태다.
    예) 처음 0 → 걸기(expected 0) 수락 → 1 → 해제(expected 1) 수락 → 2 → 다시 걸기(expected 2) 수락 → 3
        이미 1 인데 expected 0 으로 걸려고 하면 REVISION_MISMATCH 로 거절한다(낡은 정보로 바꾸는 것을 막음).
    """

    def __init__(self, binding, policy):
        self._binding = binding
        self._policy = policy
        self.revision = 0         # 감시 칸 번호
        self.watch = None         # 지금 걸린 감시(WatchSpec). 없으면 None
        self.detector = None      # 그 감시의 출구 판정기. 없으면 None → webcam_node 가 YOLO 를 돌리지 않는다
        self._replies = {}        # command_id 별 처음 응답(재전송 대비)
        self._image_size = None   # 마지막으로 본 영상 크기

    def handle(self, req, *, now_ns):
        """요청 하나 처리. 같은 command_id 가 다시 오면(재전송) 처음 응답을 그대로 돌려줘 감시가 두 번 걸리지 않게 한다."""
        if req.command_id in self._replies:
            return self._replies[req.command_id]
        # 처음 보는 요청이면 판단하고
        reply = self._decide(req, now_ns)
        # 응답을 command_id 로 기억해 둔다 (빈 ID 는 기억하지 않음)
        if req.command_id:
            self._replies[req.command_id] = reply
        return reply

    def _reject(self, req, code, detail=""):
        """거절 응답. 칸 번호는 지금 값 그대로 알려 준다."""
        return WatchReply(False, req.watch_id, self.revision, code, detail)

    def _decide(self, req, now_ns):
        """거절 조건을 위에서부터 차례로 본다. 거절하면 칸은 그대로 둔다."""
        if req.camera_id != self._binding.source_camera:  # 다른 카메라에 보낸 요청
            return self._reject(req, "CAMERA_MISMATCH")
        if type(req.deadline_ns) is not int or req.deadline_ns <= now_ns:  # 기한이 지난 요청
            return self._reject(req, "EXPIRED")
        if not req.command_id or req.association_method not in ASSOCIATION_METHODS:  # 형식 오류
            return self._reject(req, "INVALID_INPUT", "command_id empty or unknown association_method")
        if req.expected_watch_revision != self.revision:  # 요청자가 알고 있는 칸 번호가 지금과 다름
            return self._reject(req, "REVISION_MISMATCH")
        # ROI 를 비교·저장하기 좋게 튜플로 바꾼다
        roi = tuple(req.roi_xyxy_normalized)
        # 해제: 지금 걸린 감시와 watch_id·ROI 가 정확히 같아야 한다
        if not req.enable:
            if self.watch is None or req.watch_id != self.watch.watch_id or roi != self.watch.roi_xyxy_normalized:
                return self._reject(req, "WATCH_MISMATCH")
            # 감시를 지우고 칸 번호 +1
            self._clear()
            return WatchReply(True, req.watch_id, self.revision, "ACCEPTED")
        # 걸기: ID·ROI 형식을 검사하고, 칸 번호를 1 올리고, 새 출구 판정기를 만든다(이전 무장·추적 상태는 버림)
        try:
            watch = WatchSpec(camera_id=req.camera_id, watch_id=req.watch_id, target_id=req.target_id,
                              roi_xyxy_normalized=roi, watch_revision=self.revision + 1)
        except PerceptionError as exc:
            return self._reject(req, "INVALID_INPUT", exc.detail)
        # 칸 번호 +1, 새 감시와 새 판정기로 바꾼다
        self.revision += 1
        self.watch = watch
        self.detector = TargetWatchDetector(self._binding, self._policy, watch)
        return WatchReply(True, watch.watch_id, self.revision, "ACCEPTED")

    def camera_changed(self):
        """카메라를 바꿨다(OPS-183): ROI 가 다른 화면을 가리키므로 활성 감시를 푼다."""
        self._clear()
        # 새 카메라의 첫 프레임 크기를 새로 기록하도록 비운다
        self._image_size = None

    def _clear(self):
        """감시를 지운다. 해제도 '바뀜'이라 칸 번호는 1 올린다."""
        self.revision += 1
        self.watch = None
        self.detector = None

    def observe_image_size(self, width, height):
        """영상 크기를 기록한다. 크기가 바뀌어서 감시를 해제했으면 True.

        ROI 는 화면 비율 좌표라 해상도가 바뀌면 다른 곳을 가리키게 된다. 그래서 감시를 지운다.
        """
        size = (width, height)
        # 처음 보는 크기면 '바뀐 것'이 아니다
        changed = self._image_size is not None and size != self._image_size
        self._image_size = size
        if changed and self.watch is not None:
            self._clear()
            return True
        return False


def stream_state(sign_frame, size, expected_size):
    """카메라 상태 (stream_state, quality_reason) 정하기.

    - 해상도가 YAML 에서 요청한 것과 다름 → DEGRADED, SIZE_MISMATCH (예: 640×480 요청, 1280×720 받음)
    - 지진 판정이 '화면 대부분 가림'으로 보류함 → UNKNOWN, 그 이유
    - 그 외 → ONLINE
    (카메라를 못 열거나 못 읽은 OFFLINE 은 webcam_node 가 직접 보낸다)
    """
    if tuple(size) != tuple(expected_size):
        return "DEGRADED", "SIZE_MISMATCH"
    if sign_frame.reason in _COVERED:
        return "UNKNOWN", sign_frame.reason
    return "ONLINE", ""


def scene_visibility(sign_frame, *, stream_online):
    """지진 판정 결과 → 출구 판정에 넘길 시야.

    - 영상이 끊겼거나, 판정하지 않은 프레임(DUPLICATE 등) → UNKNOWN
    - OCCLUDED · SCENE_CHANGED · QUADRANT_COVERED (화면 대부분 가림) → OCCLUDED
    - MATCH_FAILED (너무 크게 흔들려 판단 보류) → UNKNOWN
    - 그 외 판정한 프레임 → VALID
    """
    if not stream_online or sign_frame.status not in ("PROCESSED", "UNKNOWN"):
        return "UNKNOWN"
    if sign_frame.reason in _COVERED:
        return "OCCLUDED"
    if sign_frame.status == "UNKNOWN":
        return "UNKNOWN"
    return "VALID"


def _coord(value):
    """좌표 하나 검사. 0 ~ 1 을 0.001 이내로 살짝 벗어난 값만 잘라 맞추고, 크게 벗어나거나 NaN 이면 오류."""
    v = float(value)
    if not math.isfinite(v) or v < -_CLIP_TOL or v > 1 + _CLIP_TOL:
        raise PerceptionError("INVALID_INPUT", "box coordinate must be normalized (0..1)")
    return min(1.0, max(0.0, v))


def boxes_from_xyxyn(rows, scores):
    """YOLO 출력 → DetectionBox 튜플.

    xyxyn = (왼쪽 x, 위 y, 오른쪽 x, 아래 y) 를 화면 너비·높이로 나눈 값(0 ~ 1).
    예) rows = [[0.10, 0.50, 0.25, 0.90]], scores = [0.93] → (DetectionBox(0.10, 0.50, 0.25, 0.90, 0.93),)
    넓이가 0 인 상자는 버리고, 상자 수와 점수 수가 다르면 오류.
    """
    rows, scores = list(rows), list(scores)
    if len(rows) != len(scores):
        raise PerceptionError("INVALID_INPUT", "boxes and scores must have the same length")
    boxes = []
    for row, score in zip(rows, scores):
        if len(row) != 4:
            raise PerceptionError("INVALID_INPUT", "box must have 4 coordinates")
        # 좌표 네 개를 검사하고 0 ~ 1 로 맞춘다
        x0, y0, x1, y1 = (_coord(v) for v in row)
        if x0 >= x1 or y0 >= y1:
            continue
        boxes.append(DetectionBox(x0, y0, x1, y1, float(score)))
    return tuple(boxes)




# 상태창 색(BGR). 어두운 띠 위에서 읽히는 밝은 색.
RED, ORANGE, GREEN, GREY, WHITE = (80, 80, 255), (0, 170, 255), (110, 220, 110), (170, 170, 170), (255, 255, 255)
_FONTS = {}   # (글꼴 경로, 크기) → ImageFont


def status_items(*, quake_now, quake_seen, boxes, score_min, presence, watched):
    """판정 화면 위 상태창 3칸: [(제목, 값, BGR 색)] 지진 · 인형 · 출구 (2026-10-15 사용자 요청: 한글, 필요한 것만).

    화면용이다. 판정·사건에는 쓰지 않는다.
    - boxes None: 지진 전이면 YOLO 를 안 돌린 것(대기), 지진 뒤면 YOLO 실패다(빈 목록으로 꾸미지 않는다).
    - presence: 출구 판정기 결과. 감시가 걸렸으면 「출구 · 감시」, 지진 뒤 화면용 판정기면 「출구 · 화면용」(기록 안 됨).
    """
    if quake_now:
        quake = ('흔들리는 중', RED)
    elif quake_seen:
        quake = ('감지됨', ORANGE)
    else:
        quake = ('없음', WHITE)
    if boxes is None:
        doll = ('탐지기 오류', RED) if quake_seen or watched else ('대기', GREY)
    else:
        best = max((b.score for b in boxes if b.score >= score_min), default=None)
        doll = (f'보임 {round(best * 100)}%', GREEN) if best is not None else ('안 보임', GREY)
    if presence is None:
        return [('지진',) + quake, ('인형',) + doll, ('출구', '대기', GREY)]
    if presence.state == 'ABSENT_CANDIDATE':
        exit_ = ('출구로 사라짐', RED)
    elif presence.state == 'PRESENT':
        exit_ = (f'안 보인 지 {presence.absent_ns / 1e9:.1f}초', ORANGE) if presence.absent_ns else ('화면 안에 있음', GREEN)
    else:
        exit_ = {'NOT_VIA_EXIT': ('다른 쪽으로 사라짐', ORANGE), 'NOT_YET_SEEN': ('인형 찾는 중', GREY),
                 'OCCLUDED': ('화면 가림', ORANGE)}.get(presence.reason, ('판정 불가', GREY))
    return [('지진',) + quake, ('인형',) + doll, ('출구 · 감시' if watched else '출구 · 화면용',) + exit_]


def draw_status(bgr, items, font_path):
    """bgr 위쪽에 상태창 띠를 그린 새 이미지. 칸마다 왼쪽 색 막대 + 작은 회색 제목 + 큰 색 글자.

    한글은 Hershey 로 못 그려 PIL 과 CJK 글꼴(debug_font_path)로 그린다(overhead.render_view 와 같은 방식).
    """
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    height, width = bgr.shape[:2]
    band = max(40, width // 13)   # 640 폭에서 49 px
    out = bgr.copy()
    dark = out.copy()
    cv2.rectangle(dark, (0, 0), (width, band), (25, 25, 25), -1)
    out = cv2.addWeighted(dark, 0.82, out, 0.18, 0)
    canvas = Image.fromarray(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(canvas)
    sizes = (max(10, band * 3 // 10), max(12, band * 9 // 20))   # 제목, 값
    fonts = []
    for size in sizes:
        if (font_path, size) not in _FONTS:
            _FONTS[font_path, size] = ImageFont.truetype(font_path, size)
        fonts.append(_FONTS[font_path, size])
    column = width // len(items)
    for n, (title, value, colour) in enumerate(items):
        x, rgb = n * column, tuple(reversed(colour))
        draw.rectangle((x + 6, 7, x + 10, band - 7), fill=rgb)
        draw.text((x + 18, 4), title, font=fonts[0], fill=(170, 170, 170))
        draw.text((x + 18, 4 + sizes[0] + 1), value, font=fonts[1], fill=rgb)
    return cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)
