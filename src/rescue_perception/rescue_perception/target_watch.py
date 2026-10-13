"""출구 구조 판정: 감시하던 인형이 '출구 쪽으로' 화면에서 사라지면 구조 후보 사건(TARGET_DISAPPEARED_CANDIDATE)을 낸다.

생각하는 방법
    구조된 사람(인형)은 출구로 옮겨진다. 그래서 인형이 화면에서 사라졌고, 마지막으로 본 위치가
    출구 쪽이면 '구출됐을 수 있다'는 후보를 낸다. 후보는 확정이 아니다. 받은 쪽이 로봇으로 다시 확인한다.
    반대쪽으로 사라졌거나, 화면이 가려져 잘 모르겠으면 후보를 내지 않고 UNKNOWN 으로 남긴다(운영자가 판단).

프레임 한 장마다 하는 일
    1. YOLO 가 준 인형 상자 중 점수가 충분하고 감시 영역(ROI) 안에 있는 것만 고른다.
    2. 감시 중인 인형의 위치를 따라간다(추적). 화면이 가려진 프레임에서도 위치는 계속 갱신한다.
    3. 화면이 가려졌거나 YOLO 결과가 없으면 판정할 수 없다 → UNKNOWN, 사라진 시간을 처음부터 다시 잰다.
    4. 인형이 보이면 PRESENT. 3장 연속 보이면 '무장'(이제 사라짐을 잴 수 있음).
    5. 무장 뒤 인형이 3초 이상 그리고 10장 이상 안 보이면, 마지막 위치가 출구 영역 안인지 본다.
       안이면 ABSENT_CANDIDATE + 후보 사건(감시 하나에 한 번만), 아니면 UNKNOWN / NOT_VIA_EXIT.

인형 상자는 YOLO(webcam_node)가 주고, '화면이 가려졌나'(시야)는 지진 판정 결과로 webcam_node 가 정해서 넣어 준다.
숫자는 모두 YAML 설정(TargetWatchPolicy)에서 받는다.
"""
from dataclasses import dataclass
import math
from uuid import NAMESPACE_URL, uuid5

from rescue_contracts import RescueError, SiteEvent, camera_id, duration_ns, uuid_string

from .adapter import SourceBinding, rgb_frame
from .core import PerceptionError

# 용어
#   ROI(감시 영역) : 화면에서 인형을 지켜볼 범위. 좌표는 화면 비율 [x0, y0, x1, y1] (0 ~ 1). [0, 0, 1, 1] = 화면 전체
#   무장           : 인형을 연속으로 확인해서 '이제 사라짐을 잴 수 있다'가 된 상태
#   추적           : 감시 중인 인형의 중심 위치를 프레임마다 따라가는 것
#   출구 영역      : 인형이 이쪽으로 사라져야 후보가 된다. 예시 값 [0, 0, 0.2, 1] = 화면 왼쪽 20%
#   시야           : VALID(잘 보임) / OCCLUDED(화면 대부분 가림) / UNKNOWN(영상 상태 모름)
#
# 예시로 따라가기 (0.2초마다 1장, 출구 = 화면 왼쪽 20%)
#   감시 시작 → 인형이 3장 연속 보임 → 무장, 상태 PRESENT
#   사람이 인형을 들고 왼쪽으로 이동 → 추적 위치가 (0.50, 0.60) → (0.30, 0.60) → (0.05, 0.60) 으로 따라감
#   인형이 화면 왼쪽 밖으로 나감 → 상자 없음 → 사라진 시간 재기 시작 (아직 PRESENT)
#   3초 넘게 계속 없음 → 마지막 위치 (0.05, 0.60) 가 출구 영역 안 → ABSENT_CANDIDATE + 후보 1건
#   오른쪽으로 들고 나갔다면 → 마지막 위치 (0.95, 0.60) 은 출구 밖 → UNKNOWN / NOT_VIA_EXIT (후보 없음)
#   도중에 사람이 렌즈를 가리면 → 그 프레임은 UNKNOWN, 사라진 시간은 처음부터 다시 (추적 위치는 계속 갱신)

EVENT_NAMESPACE = uuid5(NAMESPACE_URL, "rescue_perception/target_watch/v1")
VISIBILITY_STATES = ("VALID", "OCCLUDED", "UNKNOWN")


# 아래 세 함수는 값 검사 도우미. 조건이 틀리면 PerceptionError 를 낸다.
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
    """YOLO 가 찾은 인형 상자 하나. 좌표는 화면 비율(0 ~ 1), x0 < x1, y0 < y1. score 는 YOLO 점수."""
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
    """감시 하나. 조정기가 SetCameraWatch 로 보낸 내용이다.

    watch_id·target_id: 조정기가 만든 ID(UUID). 후보 사건에 그대로 실려 '어느 대상의 후보인지' 알려 준다.
    roi_xyxy_normalized: 감시 영역. watch_revision: 감시 칸 번호(걸 때마다 1씩 오름).
    """
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
    """출구 판정에 쓰는 숫자 모음. 모두 YAML 에서 받고 기본값은 없다. (괄호 안은 예시 YAML 값)

    - score_min (0.25)               : 이 점수 이상인 상자만 인형으로 본다
    - roi_overlap_min (0.5)          : 상자 면적의 이 비율 이상이 ROI 안이면 'ROI 안에 있다'
    - present_confirm_frames (3)     : 무장에 필요한 연속 '보임' 프레임 수
    - absent_hold_ns (3 s)           : 후보가 되려면 이 시간 이상 계속 안 보여야 한다
    - absent_min_frames (10)         : 후보가 되려면 이 장수 이상 계속 안 보여야 한다 (시간·장수 둘 다)
    - max_gap_ns (1 s)               : 프레임 사이가 이보다 벌어지면 사라진 시간을 처음부터
    - exit_roi_xyxy_normalized ([0, 0, 0.2, 1]) : 출구 영역. 화면 전체로 두면 방향을 보지 않는다
    - track_max_jump (0.3)           : 마지막 추적 위치에서 이 거리(화면 비율) 안의 상자만 같은 인형으로 본다
    """
    score_min: float
    roi_overlap_min: float
    present_confirm_frames: int
    absent_hold_ns: int
    absent_min_frames: int
    max_gap_ns: int
    exit_roi_xyxy_normalized: tuple
    track_max_jump: float

    # 값 범위 검사. 하나라도 틀리면 노드가 기동하지 않는다.
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
    """프레임 한 장의 판정 결과. webcam_node 가 이것을 TargetPresence 메시지로 보낸다.

    - status          : PROCESSED(판정함), DUPLICATE·REORDERED(같은·이른 시각이라 무시)
    - state           : PRESENT(보임, 또는 사라진 시간 재는 중) / ABSENT_CANDIDATE(출구로 사라짐) / UNKNOWN(판정 불가·애매)
    - reason          : UNKNOWN 인 이유 등. NOT_YET_SEEN(아직 무장 전), OCCLUDED(가림), VISIBILITY_UNKNOWN(영상 상태 모름),
                        DETECTOR_UNAVAILABLE(YOLO 실패), NOT_VIA_EXIT(출구 아닌 쪽으로 사라짐), FRAME_GAP(프레임 끊김 뒤)
    - absent_frames / absent_ns : 지금까지 연속으로 안 보인 장수와 시간
    - consecutive_valid_frames  : 연속으로 잘 보인(VALID) 장수
    - event           : 이번 프레임에서 낸 후보 사건. 없으면 None
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
    """상자 면적 중 ROI 와 겹친 부분의 비율. 예) 상자의 절반이 ROI 안이면 0.5"""
    ix = max(0.0, min(box.x1, roi[2]) - max(box.x0, roi[0]))  # 겹친 부분의 가로 길이
    iy = max(0.0, min(box.y1, roi[3]) - max(box.y0, roi[1]))  # 겹친 부분의 세로 길이
    return (ix * iy) / ((box.x1 - box.x0) * (box.y1 - box.y0))


def _center(box):
    return ((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2)


class TargetWatchDetector:
    """출구 판정기. 감시 하나에 하나(감시를 새로 걸면 새로 만든다). update() 에 프레임을 시간 순서대로 넣는다."""

    def __init__(self, binding, policy, watch):
        _require(isinstance(binding, SourceBinding) and binding.source_robot is None, "INVALID_INPUT",
                 "fixed camera binding must have source_robot=None")
        _require(isinstance(policy, TargetWatchPolicy), "INVALID_INPUT", "policy")
        _require(isinstance(watch, WatchSpec), "INVALID_INPUT", "watch")
        cam = _contract(camera_id, binding.source_camera, "camera_id")
        _require(watch.camera_id == cam, "INVALID_INPUT", "watch camera_id differs from the camera binding")
        self.binding, self.policy, self.watch, self.camera_id = binding, policy, watch, cam
        # 프레임 사이에 기억하는 값들
        self._last_ns = None
        self._present_run = 0       # 연속으로 보인 장수 (무장 전에 센다)
        self._armed = False         # 무장했나 = 사라짐을 잴 수 있나
        self._absent_since = None   # 안 보이기 시작한 시각
        self._absent_frames = 0     # 연속으로 안 보인 장수
        self._valid_run = 0         # 연속으로 잘 보인(VALID) 장수
        self._emitted = False       # 후보를 이미 냈나 (감시 하나에 한 번)
        self._track = None          # 감시 중인 인형의 마지막 중심 (x, y)

    @property
    def track(self):
        """따라가는 인형의 마지막 중심(화면 비율 x, y). 아직 못 봤으면 None. 판정 화면에 점으로 그린다."""
        return self._track

    def update(self, rgb, boxes, *, visibility):
        """★ 프레임 한 장 판정.

        boxes      : 이 프레임의 인형 상자들(DetectionBox 튜플). YOLO 가 실패했으면 None.
        visibility : VALID / OCCLUDED / UNKNOWN. webcam_node 가 지진 판정 결과로 정해 넣는다.
        """
        frame, evidence_sha256 = rgb_frame(rgb, self.binding)
        _require(visibility in VISIBILITY_STATES, "INVALID_INPUT", "visibility")
        _require(boxes is None or (type(boxes) is tuple and all(isinstance(b, DetectionBox) for b in boxes)),
                 "INVALID_INPUT", "boxes must be a tuple of DetectionBox or None")
        stamp = frame.observed_at_ns
        # 같은 시각·더 이른 시각 프레임은 무시한다(상태를 바꾸지 않음).
        if self._last_ns is not None and stamp <= self._last_ns:
            return PresenceFrame("DUPLICATE" if stamp == self._last_ns else "REORDERED", None, None, None,
                                 self._absent_frames, self._absent_ns(stamp), self._valid_run, None)
        # 프레임이 1초 넘게 끊겼으면 그 사이를 모르므로 사라진 시간을 처음부터 잰다.
        gap = self._last_ns is not None and stamp - self._last_ns > self.policy.max_gap_ns
        self._last_ns = stamp
        if gap:
            self._restart_absence()
        # (1) 인형 상자 고르기: 점수 ≥ 0.25 인 상자(scored) → 그중 면적의 50% 이상이 ROI 안인 상자(in_roi)
        #     boxes 가 None(YOLO 실패)이어도 여기서는 빈 목록으로 다루고, 판정 불가 처리는 (3) 에서 한다.
        roi = self.watch.roi_xyxy_normalized
        scored = [b for b in boxes or () if b.score >= self.policy.score_min]
        in_roi = [b for b in scored if _inside_fraction(b, roi) >= self.policy.roi_overlap_min]
        # (2) 추적: 인형 위치를 갱신하고, 이번 프레임에 그 인형이 보였는지(seen)를 받는다.
        #     화면이 가려진 프레임에서도 갱신한다. 사람이 몸으로 화면을 가린 채 인형을 들고 나가는 경우가 실제로 있었다.
        seen = self._follow(in_roi, scored)

        # (3) 판정 불가: 화면 가림·영상 상태 모름·YOLO 실패 → UNKNOWN, 사라진 시간 리셋.
        #     애매한 프레임에서는 후보를 만들지 않는다. 운영자가 영상을 보고 판단한다.
        if visibility != "VALID" or boxes is None:
            self._valid_run = 0
            self._present_run = 0
            self._restart_absence()
            reason = ("OCCLUDED" if visibility == "OCCLUDED" else "VISIBILITY_UNKNOWN" if visibility == "UNKNOWN"
                      else "DETECTOR_UNAVAILABLE")
            return self._result("UNKNOWN", visibility, reason, stamp, None)

        # 여기부터는 잘 보이는(VALID) 프레임이다.
        self._valid_run += 1
        present = bool(in_roi)  # ROI 안 인형 상자가 하나라도 있으면 True
        # (4) 인형이 보임 → PRESENT. 3장 연속이면 무장한다.
        if present:
            self._present_run += 1
            if self._present_run >= self.policy.present_confirm_frames:
                self._armed = True
            self._restart_absence()
            return self._result("PRESENT", visibility, "FRAME_GAP" if gap else None, stamp, None)

        # (5) 인형이 안 보임. 아직 무장 전이면 후보를 내지 않는다(NOT_YET_SEEN).
        #     처음부터 한 번도 제대로 못 봤다면 ROI 를 잘못 잡았을 수도 있기 때문이다.
        self._present_run = 0
        if not self._armed:
            return self._result("UNKNOWN", visibility, "NOT_YET_SEEN", stamp, None)
        # (6) 사라진 시간·장수를 센다. 3초와 10장을 둘 다 넘기 전에는 아직 PRESENT(재는 중).
        #     0.2초마다 1장이면 3초 = 15장이라 10장 조건은 저절로 넘는다. 프레임이 느려질 때를 위해 둘 다 본다.
        if self._absent_since is None:
            self._absent_since = stamp
        self._absent_frames += 1
        absent_ns = stamp - self._absent_since
        if absent_ns < self.policy.absent_hold_ns or self._absent_frames < self.policy.absent_min_frames:
            return self._result("PRESENT", visibility, "FRAME_GAP" if gap else None, stamp, None)
        # (7) ★ 출구 판정: 충분히 오래 안 보였다. 마지막으로 본 위치가 출구 영역 안인가?
        #     not seen    : 이번 프레임에 그 인형이 화면 어디에도 안 보인다 (화면 안 다른 곳에 있으면 나간 게 아니다)
        #     self._track : 마지막으로 본 중심 (x, y)
        event = None
        if not self._emitted:
            x0, y0, x1, y1 = self.policy.exit_roi_xyxy_normalized
            via_exit = not seen and self._track is not None and x0 <= self._track[0] <= x1 and y0 <= self._track[1] <= y1
            if not via_exit:
                return self._result("UNKNOWN", visibility, "NOT_VIA_EXIT", stamp, None)
            self._emitted = True  # 후보 사건은 감시 하나당 한 번만 낸다(인형이 다시 보이면 상태는 PRESENT 로 돌아간다).
            event = self._event(frame, evidence_sha256, absent_ns)
        return self._result("ABSENT_CANDIDATE", visibility, None, stamp, event)

    def _follow(self, in_roi, scored):
        """감시 중인 인형의 위치를 갱신한다. 이번 프레임에 그 인형이 보였으면 True.

        1. ROI 안 상자가 있으면 그중 점수가 가장 높은 상자가 그 인형이다.
        2. 없으면 직전 위치에서 가장 가까운 상자를 고른다(사람이 들고 옮기는 중일 수 있다).
           단, 마지막 추적 위치(몇 프레임 전일 수 있음)에서 거리 0.3(track_max_jump, 화면 비율) 넘게 떨어져 있으면 다른 물체로 본다.
        예) 직전 위치 (0.40, 0.60), 이번 상자 중심이 (0.35, 0.62) 와 (0.90, 0.20)
            → 가까운 (0.35, 0.62) 를 고르고, 거리 약 0.05 ≤ 0.3 이라 같은 인형으로 본다.
        """
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
        """구조 후보 사건(SiteEvent) 만들기. detail 에 판정 근거(사라진 시간·장수·마지막 위치·출구 영역)를 적는다."""
        p, w, stamp = self.policy, self.watch, frame.observed_at_ns
        detail = (f"absent_since_ns={self._absent_since} absent_ns={absent_ns} absent_frames={self._absent_frames} "
                  f"hold_ns={p.absent_hold_ns} min_frames={p.absent_min_frames} score_min={p.score_min} "
                  f"roi_overlap_min={p.roi_overlap_min} roi={w.roi_xyxy_normalized} watch_revision={w.watch_revision} "
                  f"last_seen_center=({self._track[0]:.3f},{self._track[1]:.3f}) exit_roi={p.exit_roi_xyxy_normalized}")
        # 사건 ID 는 카메라·watch_id 로 만든다(uuid5) → 같은 감시는 언제나 같은 ID 라 받는 쪽이 중복을 알아챈다.
        # confidence 1.0 은 확률이 아니라 '규칙 조건을 만족했다'는 뜻이다.
        return SiteEvent(str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{w.watch_id}/candidate")), self.camera_id,
                         "TARGET_DISAPPEARED_CANDIDATE", stamp,
                         str(uuid5(EVENT_NAMESPACE, f"{self.camera_id}/{stamp}/frame")), evidence_sha256, 1.0,
                         frame.clock_domain_id, frame.stamp_semantics, self.binding.clock_state,
                         self.binding.clock_error_bound_ns, detail, target_id=w.target_id, watch_id=w.watch_id)
