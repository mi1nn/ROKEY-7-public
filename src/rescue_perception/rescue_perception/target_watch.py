"""고정 웹캠: 구조 지점 감시 영역의 인형 유무 -> SiteEvent(TARGET_DISAPPEARED_CANDIDATE) (E2 전용).

Q-26 결정(2026-10-11): 웹캠은 지진 감지와 구조 지점의 대상 소실 후보를 맡는다. 계약은
docs/design/contracts.md C-11, IDL 은 SetCameraWatch·TargetPresence(#77). 모델·ROS·저장소 없음.
인형 상자는 탐지기(웹캠 survivor 모델)가 주고, 이 모듈은 상자와 감시 영역만으로 판정한다.

1. 감시(WatchSpec)는 coordinator 가 발급한 watch_id·target_id 와 정규화 ROI 를 담는다. 카메라당 1개.
2. 프레임마다 ROI 안에 인형이 있는지 본다: 점수 ≥ ``score_min`` 이고 상자 면적 중 ROI 안 비율이
   ``roi_overlap_min`` 이상인 상자가 하나라도 있으면 "있음".
3. 무장: 인형이 ``present_confirm_frames`` 연속 보여야 소실을 잴 수 있다. 한 번도 못 봤으면
   (ROI 를 잘못 잡았을 수 있다) 후보를 내지 않고 UNKNOWN(``NOT_YET_SEEN``)으로 알린다.
4. 소실: 무장 뒤 VALID 프레임에서 계속 "없음"이 ``absent_hold_ns`` 이상, ``absent_min_frames`` 이상
   이어지고, **감시 중인 인형이 출구 쪽으로 사라졌을 때만** ABSENT_CANDIDATE(사용자 결정 2026-10-12,
   Q-26 추가 항목). 인형은 화면 안에서 따라간다: 직전 위치에서 ``track_max_jump`` 이내의 가장 가까운
   상자가 같은 인형이다. 따라가던 인형이 화면에서 사라졌고 마지막 중심이 ``exit_roi_xyxy_normalized``
   안이면 출구 쪽이다. 아니면(반대쪽·제자리에서 사라짐, 화면 안 다른 곳에 있음) UNKNOWN(``NOT_VIA_EXIT``)
   으로 운영자 판단에 넘긴다. 출구 영역을 화면 전체로 두면 방향을 보지 않는다.
   사건은 watch 하나에 하나(C-11 2·3행, 같은 watch 는 같은 event_id).
5. 판정 불가: 가림(OCCLUDED)·영상 상태 불명(UNKNOWN)·탐지기 결과 없음은 state UNKNOWN 이고 소실
   시간을 처음부터 다시 잰다. 후보를 만들지 않는다 — 관제 웹에서 운영자가 영상을 보고 판단한다
   (Q-26, C-11 10행 REQUEST_RECHECK). 프레임 간격이 ``max_gap_ns`` 보다 길어도 다시 잰다.

``visibility`` 는 호출하는 쪽(웹캠 노드)이 정한다: 정지 영상·끊김·가림 판정(CameraStatus,
TargetPresence.visibility_state). 이 모듈은 받은 값을 그대로 판정 조건으로 쓴다.
사건은 재확인의 트리거일 뿐 완료 증거가 아니다(KPI-06). 모든 값은 ``TargetWatchPolicy`` 로
받는다(기본값 없음).
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns, uuid_string

from .adapter import SourceBinding, rgb_frame
from .core import PerceptionError

EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/target_watch/v1")
VISIBILITY_STATES = ("VALID", "OCCLUDED", "UNKNOWN")


def _require(condition, code, detail):
    if not condition:
        raise PerceptionError(code, detail)


def _contract(check, value, label):
    try:
        return check(value)
    except RescueError as exc:
        raise PerceptionError("INVALID_INPUT", f"{label}: {exc.detail}") from exc


def _unit(value, name, low=0.0, high=1.0):
    _require(type(value) in (int, float) and math.isfinite(value) and low <= value <= high, "INVALID_INPUT", name)


@dataclass(frozen=True)
class DetectionBox:
    """탐지기가 준 인형 상자 하나. 정규화 좌표(0~1, x0 < x1, y0 < y1)와 점수."""
    x0: float
    y0: float
    x1: float
    y1: float
    score: float

    def __post_init__(self):
        for name in ("x0", "y0", "x1", "y1", "score"):
            _unit(getattr(self, name), name)
        _require(self.x0 < self.x1 and self.y0 < self.y1, "INVALID_INPUT", "box must have x0 < x1, y0 < y1")


@dataclass(frozen=True)
class WatchSpec:
    """SetCameraWatch 로 받은 감시 하나(C-11 1·2행). ID 는 coordinator 가 발급한 canonical UUID."""
    camera_id: str
    watch_id: str
    target_id: str
    roi_xyxy_normalized: tuple
    watch_revision: int

    def __post_init__(self):
        _contract(camera_id, self.camera_id, "camera_id")
        _contract(uuid_string, self.watch_id, "watch_id")
        _contract(uuid_string, self.target_id, "target_id")
        r = self.roi_xyxy_normalized
        _require(type(r) is tuple and len(r) == 4, "INVALID_INPUT", "roi_xyxy_normalized must be a 4-tuple")
        for v in r:
            _unit(v, "roi_xyxy_normalized")
        _require(r[0] < r[2] and r[1] < r[3], "INVALID_INPUT", "roi must have x0 < x1, y0 < y1")
        _require(type(self.watch_revision) is int and self.watch_revision >= 0, "INVALID_INPUT", "watch_revision")


@dataclass(frozen=True)
class TargetWatchPolicy:
    """소실 판정 값. 배포 프로파일이 넣는다. 기본값은 하나도 없다.

    - ``score_min``: 이 점수 이상인 상자만 인형으로 본다.
    - ``roi_overlap_min``: 상자 면적 중 ROI 안에 든 비율이 이 이상이면 "ROI 안에 있음"(0 초과).
    - ``present_confirm_frames``: 무장에 필요한 연속 "있음" 프레임 수.
    - ``absent_hold_ns``/``absent_min_frames``: 소실 후보에 필요한 연속 "없음" 시간과 프레임 수(둘 다).
    - ``max_gap_ns``: 프레임 간격이 이보다 길면 소실 시간을 처음부터 다시 잰다.
    - ``exit_roi_xyxy_normalized``: 화면에서 출구 쪽 영역(정규화). 인형이 여기서 사라져야 후보다.
    - ``track_max_jump``: 한 프레임 사이 같은 인형으로 볼 최대 중심 이동(정규화 거리, 0 초과 √2 이하).
    """
    score_min: float
    roi_overlap_min: float
    present_confirm_frames: int
    absent_hold_ns: int
    absent_min_frames: int
    max_gap_ns: int
    exit_roi_xyxy_normalized: tuple
    track_max_jump: float

    def __post_init__(self):
        _unit(self.score_min, "score_min")
        _unit(self.roi_overlap_min, "roi_overlap_min")
        _require(self.roi_overlap_min > 0, "INVALID_INPUT", "roi_overlap_min must be positive")
        for name in ("present_confirm_frames", "absent_min_frames"):
            _require(type(getattr(self, name)) is int and getattr(self, name) >= 1, "INVALID_INPUT", name)
        for name in ("absent_hold_ns", "max_gap_ns"):
            _contract(duration_ns, getattr(self, name), name)
            _require(getattr(self, name) > 0, "INVALID_INPUT", f"{name} must be positive")
        r = self.exit_roi_xyxy_normalized
        _require(type(r) is tuple and len(r) == 4, "INVALID_INPUT", "exit_roi_xyxy_normalized must be a 4-tuple")
        for v in r:
            _unit(v, "exit_roi_xyxy_normalized")
        _require(r[0] < r[2] and r[1] < r[3], "INVALID_INPUT", "exit roi must have x0 < x1, y0 < y1")
        _unit(self.track_max_jump, "track_max_jump", high=math.sqrt(2))
        _require(self.track_max_jump > 0, "INVALID_INPUT", "track_max_jump must be positive")


@dataclass(frozen=True)
class PresenceFrame:
    """프레임 하나의 판정(TargetPresence 에 대응).

    - ``status``: PROCESSED, DUPLICATE, REORDERED.
    - ``state``: PRESENT(있음, 또는 무장 뒤 소실을 재는 중), ABSENT_CANDIDATE, UNKNOWN.
    - ``reason``: UNKNOWN·재시작의 이유(NOT_YET_SEEN, OCCLUDED, VISIBILITY_UNKNOWN,
      DETECTOR_UNAVAILABLE, FRAME_GAP) 또는 None.
    - ``absent_frames``/``absent_ns``: 지금 이어지는 "없음" 프레임 수와 시간.
    - ``consecutive_valid_frames``: 연속 VALID 프레임 수.
    - ``event``: 이 프레임에서 낸 소실 후보(watch 당 최대 1개).
    """
    status: str
    state: str | None
    visibility_state: str | None
    reason: str | None
    absent_frames: int
    absent_ns: int
    consecutive_valid_frames: int
    event: SiteEvent | None


def _inside_fraction(box, roi):
    ix = max(0.0, min(box.x1, roi[2]) - max(box.x0, roi[0]))
    iy = max(0.0, min(box.y1, roi[3]) - max(box.y0, roi[1]))
    return (ix * iy) / ((box.x1 - box.x0) * (box.y1 - box.y0))


def _center(box):
    return ((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2)


class TargetWatchDetector:
    """웹캠 1대의 활성 감시 1개. 상태는 센서 시각 순서로만 나아간다."""

    def __init__(self, binding, policy, watch):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, TargetWatchPolicy), "INVALID_INPUT", "policy")
        _require(isinstance(watch, WatchSpec), "INVALID_INPUT", "watch")
        cam = _contract(camera_id, binding.source_camera, "camera_id")
        _require(watch.camera_id == cam, "INVALID_INPUT", "watch camera_id differs from the camera binding")
        self.binding, self.policy, self.watch, self.camera_id = binding, policy, watch, cam
        self._last_ns = None
        self._present_run = 0       # 연속 "있음" 프레임 (무장 전)
        self._armed = False
        self._absent_since = None   # 이어지는 "없음"의 시작 시각
        self._absent_frames = 0
        self._valid_run = 0
        self._emitted = False       # watch 하나에 후보 하나
        self._track = None          # 감시 중인 인형의 마지막 중심

    @property
    def track(self):
        """따라가는 인형의 마지막 중심(정규화 x, y). 아직 못 봤으면 None."""
        return self._track

    def update(self, rgb, boxes, *, visibility):
        """``boxes``: 이 프레임의 인형 상자 튜플, 탐지기가 결과를 못 냈으면 None.
        ``visibility``: VALID / OCCLUDED / UNKNOWN (웹캠 노드가 정함).
        """
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        _require(visibility in VISIBILITY_STATES, "INVALID_INPUT", "visibility")
        _require(boxes is None or (type(boxes) is tuple and all(isinstance(b, DetectionBox) for b in boxes)),
                 "INVALID_INPUT", "boxes must be a tuple of DetectionBox or None")
        stamp = frame.observed_at_ns
        if self._last_ns is not None and stamp <= self._last_ns:
            return PresenceFrame("DUPLICATE" if stamp == self._last_ns else "REORDERED", None, None, None,
                                 self._absent_frames, self._absent_ns(stamp), self._valid_run, None)
        gap = self._last_ns is not None and stamp - self._last_ns > self.policy.max_gap_ns
        self._last_ns = stamp
        if gap:
            self._restart_absence()
        roi = self.watch.roi_xyxy_normalized
        scored = [b for b in boxes or () if b.score >= self.policy.score_min]
        in_roi = [b for b in scored if _inside_fraction(b, roi) >= self.policy.roi_overlap_min]
        # 가림 중에도 모델이 낸 상자로 따라간다: 옮기는 사람이 화면을 가린 채 인형을 들고 나간다(2026-10-12 실기).
        seen = self._follow(in_roi, scored)

        if visibility != "VALID" or boxes is None:
            # 판정 불가: 후보를 만들지 않고 소실 시간을 다시 잰다(운영자 판단 대상).
            self._valid_run = 0
            self._present_run = 0
            self._restart_absence()
            reason = ("OCCLUDED" if visibility == "OCCLUDED" else "VISIBILITY_UNKNOWN" if visibility == "UNKNOWN"
                      else "DETECTOR_UNAVAILABLE")
            return self._result("UNKNOWN", visibility, reason, stamp, None)

        self._valid_run += 1
        present = bool(in_roi)
        if present:
            self._present_run += 1
            if self._present_run >= self.policy.present_confirm_frames:
                self._armed = True
            self._restart_absence()
            return self._result("PRESENT", visibility, "FRAME_GAP" if gap else None, stamp, None)

        self._present_run = 0
        if not self._armed:
            return self._result("UNKNOWN", visibility, "NOT_YET_SEEN", stamp, None)
        if self._absent_since is None:
            self._absent_since = stamp
        self._absent_frames += 1
        absent_ns = stamp - self._absent_since
        if absent_ns < self.policy.absent_hold_ns or self._absent_frames < self.policy.absent_min_frames:
            return self._result("PRESENT", visibility, "FRAME_GAP" if gap else None, stamp, None)
        event = None
        if not self._emitted:
            x0, y0, x1, y1 = self.policy.exit_roi_xyxy_normalized
            via_exit = not seen and self._track is not None and x0 <= self._track[0] <= x1 and y0 <= self._track[1] <= y1
            if not via_exit:
                return self._result("UNKNOWN", visibility, "NOT_VIA_EXIT", stamp, None)
            self._emitted = True
            event = self._event(frame, evidence_sha256, absent_ns)
        return self._result("ABSENT_CANDIDATE", visibility, None, stamp, event)

    def _follow(self, in_roi, scored):
        """감시 중인 인형의 위치를 갱신한다. 이 프레임에서 그 인형이 보였으면 True."""
        if in_roi:
            self._track = _center(max(in_roi, key=lambda b: b.score))
            return True
        if self._track is None or not scored:
            return False
        tx, ty = self._track
        nearest = min(scored, key=lambda b: math.dist(_center(b), (tx, ty)))
        if math.dist(_center(nearest), (tx, ty)) > self.policy.track_max_jump:
            return False
        self._track = _center(nearest)
        return True

    def _restart_absence(self):
        self._absent_since, self._absent_frames = None, 0

    def _absent_ns(self, stamp):
        return 0 if self._absent_since is None else max(0, stamp - self._absent_since)

    def _result(self, state, visibility, reason, stamp, event):
        return PresenceFrame("PROCESSED", state, visibility, reason, self._absent_frames, self._absent_ns(stamp),
                             self._valid_run, event)

    def _event(self, frame, evidence_sha256, absent_ns):
        p, w, stamp = self.policy, self.watch, frame.observed_at_ns
        detail = (f"absent_since_ns={self._absent_since} absent_ns={absent_ns} absent_frames={self._absent_frames} "
                  f"hold_ns={p.absent_hold_ns} min_frames={p.absent_min_frames} score_min={p.score_min} "
                  f"roi_overlap_min={p.roi_overlap_min} roi={w.roi_xyxy_normalized} watch_revision={w.watch_revision} "
                  f"last_seen_center=({self._track[0]:.3f},{self._track[1]:.3f}) exit_roi={p.exit_roi_xyxy_normalized}")
        # 규칙 기반 사건이라 확률이 아니다: 1.0 은 "정책 조건 충족" (C-05, disaster_sign 과 같은 규칙).
        return SiteEvent(str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{w.watch_id}/candidate")), self.camera_id,
                         "TARGET_DISAPPEARED_CANDIDATE", stamp,
                         str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame")), evidence_sha256, 1.0,
                         frame.clock_domain_id, frame.stamp_semantics, self.binding.clock_state,
                         self.binding.clock_error_bound_ns, detail, target_id=w.target_id, watch_id=w.watch_id)
