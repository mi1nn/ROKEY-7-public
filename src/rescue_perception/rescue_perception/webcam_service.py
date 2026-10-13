"""고정 웹캠 노드의 판정 경계: 감시 설정 수락, 영상 상태 -> 시야, 탐지 상자 변환 (E2 전용).

ROS·카메라·모델 없이 import·시험된다. ``webcam_node`` 는 이 모듈의 결과를 메시지에 옮긴다.

1. ``WatchSlot``: 카메라당 감시 칸 하나(SDD SetCameraWatch Rules, C-11 경계 검사 2026-10-10).
   - 거절: 다른 카메라(CAMERA_MISMATCH), 기한 지남(EXPIRED), ``expected_watch_revision`` 이
     지금 칸 번호와 다름(REVISION_MISMATCH), 형식 오류·빈 열거값(INVALID_INPUT), 해제 요청이
     활성 감시의 ``watch_id``·ROI 와 다름(WATCH_MISMATCH). 거절은 칸을 바꾸지 않는다.
   - 수락하면 칸 번호(``watch_revision``)를 1 올린다. 0 은 비어 있음이다. 같은 ``command_id`` 를
     다시 받으면 처음 응답을 그대로 돌려준다(재전송 멱등).
   - 영상 크기가 바뀌면 ROI 가 다른 화면을 가리키므로 감시를 지운다(SDD Rules: image_size 변경 시 무효).
2. ``scene_visibility``: 지진 판정(``DisasterSignDetector``)이 이미 재는 화면 전체 가림을 대상
   감시의 시야로 쓴다. 영상이 끊겼으면 UNKNOWN, 화면 대부분이 가려졌으면(OCCLUDED·SCENE_CHANGED·
   QUADRANT_COVERED) OCCLUDED, 크게 흔들려 직전 프레임과 못 맞췄으면(MATCH_FAILED) UNKNOWN.
   **한계:** 사람이 ROI 만 가리는 국소 가림은 잡지 못해 VALID 로 들어간다(OPS-74-02). 후보는 재확인의
   트리거일 뿐이고 AMR1 재관측이 확정한다(KPI-06).
3. ``boxes_from_xyxyn``: 모델의 정규화 상자(x0, y0, x1, y1)와 점수를 ``DetectionBox`` 로 바꾼다.
   부동소수 오차로 0~1 을 살짝 벗어난 좌표만 자르고, 면적 0 상자는 버린다. NaN·길이 불일치는 거절.
"""
from dataclasses import dataclass
import math

from .core import PerceptionError
from .target_watch import DetectionBox, TargetWatchDetector, WatchSpec

ASSOCIATION_METHODS = ("CALIBRATED_PROJECTION", "OPERATOR_ROI")
# 지진 판정이 "화면 대부분이 가려짐"으로 보류한 이유(disaster_sign.SignFrame.reason).
_COVERED = ("OCCLUDED", "SCENE_CHANGED", "QUADRANT_COVERED")
# 좌표를 0~1 로 자를 때 허용하는 부동소수 오차.
_CLIP_TOL = 1e-3


@dataclass(frozen=True)
class WatchRequest:
    """SetCameraWatch 요청에서 판정에 쓰는 필드(Envelope 제외)."""
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
    accepted: bool
    watch_id: str
    watch_revision: int
    reason_code: str
    detail: str = ""  # 거절 이유(로그용). 서비스 응답 필드는 아니다


class WatchSlot:
    """카메라당 감시 칸 하나와 그 칸의 소실 판정기."""

    def __init__(self, binding, policy):
        # revision  : 감시 칸 번호(0 = 비어 있음). 걸거나 해제할 때마다 1 오른다.
        # watch     : 지금 걸린 감시(WatchSpec), detector: 그 감시의 출구 판정기
        # _replies  : command_id 별 처음 응답(재전송 대비)
        self._binding = binding
        self._policy = policy
        self.revision = 0
        self.watch = None
        self.detector = None
        self._replies = {}
        self._image_size = None

    def handle(self, req, *, now_ns):
        # 같은 command_id 가 다시 오면(네트워크 재전송) 처음 응답을 그대로 돌려준다. 감시가 두 번 걸리지 않는다.
        if req.command_id in self._replies:
            return self._replies[req.command_id]
        reply = self._decide(req, now_ns)
        if req.command_id:
            self._replies[req.command_id] = reply
        return reply

    def _reject(self, req, code, detail=""):
        return WatchReply(False, req.watch_id, self.revision, code, detail)

    def _decide(self, req, now_ns):
        # 거절 조건을 순서대로 본다. 거절하면 칸은 그대로다.
        if req.camera_id != self._binding.source_camera:
            return self._reject(req, "CAMERA_MISMATCH")
        if type(req.deadline_ns) is not int or req.deadline_ns <= now_ns:
            return self._reject(req, "EXPIRED")
        if not req.command_id or req.association_method not in ASSOCIATION_METHODS:
            return self._reject(req, "INVALID_INPUT", "command_id empty or unknown association_method")
        # 요청자는 '지금 칸 번호'를 알고 있어야 한다. 모르는 채로 바꾸면(두 곳이 동시에 바꾸는 경우) 거절.
        if req.expected_watch_revision != self.revision:
            return self._reject(req, "REVISION_MISMATCH")
        roi = tuple(req.roi_xyxy_normalized)
        # 해제: 지금 걸린 감시와 watch_id·ROI 가 정확히 같아야 한다.
        if not req.enable:
            if self.watch is None or req.watch_id != self.watch.watch_id or roi != self.watch.roi_xyxy_normalized:
                return self._reject(req, "WATCH_MISMATCH")
            self._clear()
            return WatchReply(True, req.watch_id, self.revision, "ACCEPTED")
        # 걸기: ID·ROI 형식을 검사하고, 칸 번호를 1 올린 뒤 새 출구 판정기를 만든다(이전 무장·추적 상태는 버림).
        try:
            watch = WatchSpec(camera_id=req.camera_id, watch_id=req.watch_id, target_id=req.target_id,
                              roi_xyxy_normalized=roi, watch_revision=self.revision + 1)
        except PerceptionError as exc:
            return self._reject(req, "INVALID_INPUT", exc.detail)
        self.revision += 1
        self.watch = watch
        self.detector = TargetWatchDetector(self._binding, self._policy, watch)
        return WatchReply(True, watch.watch_id, self.revision, "ACCEPTED")

    def _clear(self):
        self.revision += 1
        self.watch = None
        self.detector = None

    def observe_image_size(self, width, height):
        """영상 크기를 기록한다. 바뀌어서 활성 감시를 지웠으면 True."""
        size = (width, height)
        changed = self._image_size is not None and size != self._image_size
        self._image_size = size
        if changed and self.watch is not None:
            self._clear()
            return True
        return False


def stream_state(sign_frame, size, expected_size):
    """CameraStatus 의 (stream_state, quality_reason). SDD 계약: 정지 영상·끊김·가림은 UNKNOWN.

    해상도가 요청과 다르면 DEGRADED(SIZE_MISMATCH), 화면 대부분이 가려졌으면(지진 판정의 OCCLUDED·
    SCENE_CHANGED·QUADRANT_COVERED) UNKNOWN 과 그 이유, 아니면 ONLINE.
    """
    if tuple(size) != tuple(expected_size):
        return "DEGRADED", "SIZE_MISMATCH"
    if sign_frame.reason in _COVERED:
        return "UNKNOWN", sign_frame.reason
    return "ONLINE", ""


# 지진 판정 이유 → 시야
#   OCCLUDED · SCENE_CHANGED · QUADRANT_COVERED → OCCLUDED (화면 대부분 가림)
#   MATCH_FAILED(너무 크게 흔들림)              → UNKNOWN
#   그 외 판정한 프레임                          → VALID
def scene_visibility(sign_frame, *, stream_online):
    """지진 판정 한 프레임 -> 대상 감시의 시야(VALID/OCCLUDED/UNKNOWN)."""
    if not stream_online or sign_frame.status not in ("PROCESSED", "UNKNOWN"):
        return "UNKNOWN"
    if sign_frame.reason in _COVERED:
        return "OCCLUDED"
    if sign_frame.status == "UNKNOWN":
        return "UNKNOWN"
    return "VALID"


# 0 ~ 1 을 0.001 이내로 살짝 벗어난 값(부동소수 오차)만 잘라 맞추고, 크게 벗어나거나 NaN 이면 오류.
def _coord(value):
    v = float(value)
    if not math.isfinite(v) or v < -_CLIP_TOL or v > 1 + _CLIP_TOL:
        raise PerceptionError("INVALID_INPUT", "box coordinate must be normalized (0..1)")
    return min(1.0, max(0.0, v))


def boxes_from_xyxyn(rows, scores):
    """모델 출력(정규화 xyxy 행, 점수) -> DetectionBox 튜플."""
    rows, scores = list(rows), list(scores)
    if len(rows) != len(scores):
        raise PerceptionError("INVALID_INPUT", "boxes and scores must have the same length")
    boxes = []
    for row, score in zip(rows, scores):
        if len(row) != 4:
            raise PerceptionError("INVALID_INPUT", "box must have 4 coordinates")
        x0, y0, x1, y1 = (_coord(v) for v in row)
        # 넓이가 0 인 상자(점·선)는 버린다.
        if x0 >= x1 or y0 >= y1:
            continue
        boxes.append(DetectionBox(x0, y0, x1, y1, float(score)))
    return tuple(boxes)
