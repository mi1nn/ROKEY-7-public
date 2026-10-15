"""고정 웹캠 노드의 판정 경계(감시 설정 수락·영상 상태 → 시야·탐지 상자 변환)의 E2 시험. 모두 합성 입력이다.

ROS·카메라·모델을 쓰지 않는다. 노드(`webcam_node`)는 이 모듈의 결과를 메시지에 옮기기만 한다.
숫자는 시험용이며 정책 확정값이 아니다.
"""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from uuid import UUID

from rescue_contracts import canonical_code

from rescue_perception import PerceptionError, SourceBinding
from rescue_perception.disaster_sign import SignFrame
from rescue_perception.target_watch import TargetWatchPolicy
from rescue_perception.webcam_service import (
    WatchRequest, WatchSlot, boxes_from_xyxyn, scene_visibility,
)

NOW = 1700000000_000000000
WATCH = str(UUID(int=8))
TARGET = str(UUID(int=7))
ROI = (0.4, 0.4, 0.6, 0.6)


def request(**overrides):
    base = dict(command_id=str(UUID(int=1)), camera_id="cam_fixed", watch_id=WATCH, target_id=TARGET,
                expected_watch_revision=0, roi_xyxy_normalized=ROI, association_method="OPERATOR_ROI",
                association_evidence_id="", enable=True, issued_at_ns=NOW - 10, deadline_ns=NOW + 10**9)
    base.update(overrides)
    return WatchRequest(**base)


def sign(status="PROCESSED", reason=None):
    return SignFrame(status=status, reason=reason, shift=None, reversals=0, condition=False, event=None)


class WatchSlotTests(unittest.TestCase):
    def setUp(self):
        binding = SourceBinding(None, "cam_fixed", str(UUID(int=300)), "site-clock", "DRIVER_RECEIVE", "UNKNOWN",
                                "cam-fixed-cal", "UNKNOWN", None)
        policy = TargetWatchPolicy(score_min=0.5, roi_overlap_min=0.5, present_confirm_frames=2,
                                   absent_hold_ns=300, absent_min_frames=3, max_gap_ns=150,
                                   exit_roi_xyxy_normalized=(0.0, 0.0, 1.0, 1.0), track_max_jump=0.3)
        self.slot = WatchSlot(binding, policy)

    def test_empty_slot_has_no_detector(self):
        self.assertEqual((self.slot.revision, self.slot.detector, self.slot.watch), (0, None, None))

    def test_enable_accepts_and_bumps_revision(self):
        reply = self.slot.handle(request(), now_ns=NOW)
        self.assertEqual((reply.accepted, reply.watch_id, reply.watch_revision, reply.reason_code),
                         (True, WATCH, 1, "ACCEPTED"))
        self.assertEqual(self.slot.watch.target_id, TARGET)
        self.assertEqual(self.slot.watch.watch_revision, 1)
        self.assertIsNotNone(self.slot.detector)

    def test_same_command_is_idempotent(self):
        first = self.slot.handle(request(), now_ns=NOW)
        again = self.slot.handle(request(), now_ns=NOW + 5)
        self.assertEqual(first, again)
        self.assertEqual(self.slot.revision, 1)

    def test_rejections_leave_slot_unchanged(self):
        cases = {
            "CAMERA_MISMATCH": request(camera_id="cam_other"),
            "EXPIRED": request(deadline_ns=NOW),
            "REVISION_MISMATCH": request(expected_watch_revision=3),
            "INVALID_INPUT": request(association_method=""),
        }
        for i, (code, req) in enumerate(cases.items()):
            req = replace(req, command_id=str(UUID(int=100 + i)))
            reply = self.slot.handle(req, now_ns=NOW)
            self.assertEqual((reply.accepted, reply.reason_code, reply.watch_revision), (False, code, 0), code)
        for i, bad in enumerate((dict(roi_xyxy_normalized=(0.6, 0.4, 0.4, 0.6)), dict(target_id=""),
                                 dict(watch_id="w1"), dict(roi_xyxy_normalized=(0.0, 0.0, 1.2, 1.0)))):
            reply = self.slot.handle(request(command_id=str(UUID(int=200 + i)), **bad), now_ns=NOW)
            self.assertEqual((reply.accepted, reply.reason_code), (False, "INVALID_INPUT"), bad)
        self.assertEqual((self.slot.revision, self.slot.detector), (0, None))

    def test_invalid_input_reply_says_which_field(self):
        # 2026-10-12 실기: watch_id 를 UUID 가 아닌 값으로 보냈더니 노드 로그에 INVALID_INPUT 만 남았다.
        reply = self.slot.handle(request(watch_id="w-doll-1"), now_ns=NOW)
        self.assertEqual(reply.reason_code, "INVALID_INPUT")
        self.assertIn("watch_id", reply.detail)

    def test_replace_needs_current_revision(self):
        self.slot.handle(request(), now_ns=NOW)
        stale = self.slot.handle(request(command_id=str(UUID(int=2)), expected_watch_revision=0,
                                         roi_xyxy_normalized=(0.1, 0.1, 0.3, 0.3)), now_ns=NOW)
        self.assertEqual((stale.accepted, stale.reason_code), (False, "REVISION_MISMATCH"))
        new = self.slot.handle(request(command_id=str(UUID(int=3)), expected_watch_revision=1,
                                       roi_xyxy_normalized=(0.1, 0.1, 0.3, 0.3)), now_ns=NOW)
        self.assertEqual((new.accepted, new.watch_revision), (True, 2))
        self.assertEqual(self.slot.watch.roi_xyxy_normalized, (0.1, 0.1, 0.3, 0.3))

    def test_disable_must_match_active_watch(self):
        self.slot.handle(request(), now_ns=NOW)
        for i, bad in enumerate((dict(watch_id=str(UUID(int=9))), dict(roi_xyxy_normalized=(0.1, 0.1, 0.3, 0.3)))):
            reply = self.slot.handle(request(command_id=str(UUID(int=50 + i)), enable=False,
                                             expected_watch_revision=1, **bad), now_ns=NOW)
            self.assertEqual((reply.accepted, reply.reason_code), (False, "WATCH_MISMATCH"), bad)
        off = self.slot.handle(request(command_id=str(UUID(int=60)), enable=False, expected_watch_revision=1),
                               now_ns=NOW)
        self.assertEqual((off.accepted, off.watch_revision), (True, 2))
        self.assertEqual((self.slot.watch, self.slot.detector), (None, None))

    def test_disable_without_active_watch_is_rejected(self):
        reply = self.slot.handle(request(enable=False), now_ns=NOW)
        self.assertEqual((reply.accepted, reply.reason_code), (False, "WATCH_MISMATCH"))

    def test_image_size_change_clears_watch(self):
        self.slot.handle(request(), now_ns=NOW)
        self.assertFalse(self.slot.observe_image_size(640, 480))  # 처음 본 크기
        self.assertFalse(self.slot.observe_image_size(640, 480))
        self.assertTrue(self.slot.observe_image_size(1280, 720))  # 바뀜 → 감시 무효
        self.assertEqual((self.slot.watch, self.slot.detector, self.slot.revision), (None, None, 2))


class SceneVisibilityTests(unittest.TestCase):
    def test_offline_stream_is_unknown(self):
        self.assertEqual(scene_visibility(sign(), stream_online=False), "UNKNOWN")

    def test_whole_view_cover_is_occluded(self):
        for reason in ("OCCLUDED", "SCENE_CHANGED", "QUADRANT_COVERED"):
            self.assertEqual(scene_visibility(sign("UNKNOWN", reason), stream_online=True), "OCCLUDED", reason)

    def test_match_failed_is_unknown(self):
        # 크게 흔들리면 직전 프레임과 맞추지 못한다. 인형이 있는지 판단할 근거가 아니다.
        self.assertEqual(scene_visibility(sign("UNKNOWN", "MATCH_FAILED"), stream_online=True), "UNKNOWN")

    def test_normal_view_is_valid(self):
        for reason in (None, "FIRST_FRAME", "BELOW_NOISE", "PARTIAL_MOTION"):
            self.assertEqual(scene_visibility(sign("PROCESSED", reason), stream_online=True), "VALID", reason)

    def test_duplicate_or_unknown_status_is_unknown(self):
        for status in ("DUPLICATE", "REORDERED"):
            self.assertEqual(scene_visibility(sign(status), stream_online=True), "UNKNOWN", status)


class BoxTests(unittest.TestCase):
    def test_rows_become_boxes(self):
        boxes = boxes_from_xyxyn([(0.1, 0.2, 0.3, 0.4)], [0.9])
        self.assertEqual(len(boxes), 1)
        self.assertEqual((boxes[0].x0, boxes[0].y1, boxes[0].score), (0.1, 0.4, 0.9))
        self.assertIsInstance(boxes, tuple)

    def test_slight_overflow_is_clipped_and_empty_boxes_dropped(self):
        # 모델이 낸 좌표가 부동소수 오차로 1 을 조금 넘을 수 있다. 면적 0 상자는 버린다.
        boxes = boxes_from_xyxyn([(-0.0001, 0.5, 1.00001, 0.9), (0.3, 0.3, 0.3, 0.6)], [0.8, 0.7])
        self.assertEqual([(b.x0, b.x1) for b in boxes], [(0.0, 1.0)])

    def test_no_detections_is_empty_tuple(self):
        self.assertEqual(boxes_from_xyxyn([], []), ())

    def test_length_mismatch_or_nan_is_rejected(self):
        with self.assertRaises(PerceptionError) as ctx:
            boxes_from_xyxyn([(0.1, 0.1, 0.2, 0.2)], [])
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", "INVALID_INPUT"))
        with self.assertRaises(PerceptionError):
            boxes_from_xyxyn([(float("nan"), 0.1, 0.2, 0.2)], [0.5])


class NodeParamsExampleTests(unittest.TestCase):
    """예시 파라미터 파일이 노드가 요구하는 키와 정확히 같고, 그 값으로 두 정책이 만들어지는지.

    파일은 설치된 패키지에서 읽는다(설치본 시험에는 src 가 없다, installed-wheels CI)."""

    def test_example_yaml_matches_node_params(self):
        from importlib.resources import files
        import re
        from rescue_perception.webcam_node import PARAMS, policies
        path = files("rescue_perception").joinpath("config/webcam_node.example.yaml")
        values = {}
        for line in path.read_text().splitlines():
            m = re.match(r"\s{4}(\w+):\s*([^#]+?)\s*(#.*)?$", line)
            if m:
                kind, raw = PARAMS.get(m.group(1)), m.group(2)
                values[m.group(1)] = (int(raw) if kind == "INTEGER" else float(raw) if kind == "DOUBLE"
                                      else [float(x) for x in raw.strip("[]").split(",")] if kind == "DOUBLE_ARRAY"
                                      else raw)
        self.assertEqual(set(values), set(PARAMS))
        policies(values)  # 값 검사는 각 정책이 한다

    def test_camera_is_opened_by_stable_device_path(self):
        # 2026-10-12 실기: 카메라 번호(device_index)는 꽂는 순서로 바뀌었다(USB 웹캠이 0 이 아니라 2).
        from rescue_perception.webcam_node import PARAMS
        self.assertNotIn("device_index", PARAMS)
        self.assertEqual(PARAMS.get("device_path"), "STRING")



class SpinShutdownTests(unittest.TestCase):
    """2026-10-12 실기: 감시 중 SIGINT 가 프레임 처리 도중에 오면 발행이 RCLError 를 내 exit 1 로 끝났다."""

    class FakeRclpy:
        def __init__(self, exc, ok):
            self.exc, self._ok = exc, ok

        def spin(self, node):
            raise self.exc

        def ok(self):
            return self._ok

    class Log:
        def __init__(self):
            self.lines = []

        def info(self, msg):
            self.lines.append(msg)

    def test_error_after_shutdown_is_a_normal_stop(self):
        from rescue_perception.webcam_node import spin_until_shutdown
        log = self.Log()
        spin_until_shutdown(self.FakeRclpy(RuntimeError("publisher's context is invalid"), ok=False), None, log)
        self.assertEqual(log.lines, ["webcam_node stopping"])

    def test_error_while_running_still_raises(self):
        from rescue_perception.webcam_node import spin_until_shutdown
        with self.assertRaises(RuntimeError):
            spin_until_shutdown(self.FakeRclpy(RuntimeError("bug"), ok=True), None, self.Log())

    def test_ctrl_c_is_a_normal_stop(self):
        from rescue_perception.webcam_node import spin_until_shutdown
        log = self.Log()
        spin_until_shutdown(self.FakeRclpy(KeyboardInterrupt(), ok=True), None, log)
        self.assertEqual(log.lines, ["webcam_node stopping"])




class StreamStateTests(unittest.TestCase):
    """CameraStatus 계약(SDD): 정지 영상·끊김·가림은 UNKNOWN. 2026-10-12 실기: 렌즈를 손으로 가려도 ONLINE 이었다."""

    def test_covered_scene_is_unknown_with_reason(self):
        from rescue_perception.webcam_service import stream_state
        for reason in ("OCCLUDED", "SCENE_CHANGED", "QUADRANT_COVERED"):
            self.assertEqual(stream_state(sign("UNKNOWN", reason), (640, 480), (640, 480)), ("UNKNOWN", reason))

    def test_normal_and_size_mismatch(self):
        from rescue_perception.webcam_service import stream_state
        self.assertEqual(stream_state(sign(), (640, 480), (640, 480)), ("ONLINE", ""))
        self.assertEqual(stream_state(sign("UNKNOWN", "MATCH_FAILED"), (640, 480), (640, 480)), ("ONLINE", ""))
        self.assertEqual(stream_state(sign(), (320, 240), (640, 480)), ("DEGRADED", "SIZE_MISMATCH"))

class DebugImageTests(unittest.TestCase):
    """2026-10-12 사용자 요청: 노드로 시험할 때도 판정을 화면으로 본다(웹캠 -> 노드 -> 토픽 -> 대시보드)."""

    def test_node_has_debug_image_topic_param(self):
        from rescue_perception.webcam_node import PARAMS
        self.assertEqual(PARAMS.get("debug_image_topic"), "STRING")

    def test_detector_exposes_tracked_position(self):
        from rescue_perception.target_watch import WatchSpec
        binding = SourceBinding(None, "cam_fixed", str(UUID(int=300)), "site-clock", "DRIVER_RECEIVE", "UNKNOWN",
                                "cam-fixed-cal", "UNKNOWN", None)
        policy = TargetWatchPolicy(score_min=0.5, roi_overlap_min=0.5, present_confirm_frames=2, absent_hold_ns=300,
                                   absent_min_frames=3, max_gap_ns=150, exit_roi_xyxy_normalized=(0.0, 0.0, 1.0, 1.0),
                                   track_max_jump=0.3)
        from rescue_perception.target_watch import TargetWatchDetector
        det = TargetWatchDetector(binding, policy, WatchSpec("cam_fixed", WATCH, TARGET, ROI, 1))
        self.assertIsNone(det.track)
        image = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1700000000, nanosec=0), frame_id="x"),
                                width=8, height=8, encoding="mono8", is_bigendian=0, step=8, data=bytes(64))
        det.update(image, boxes_from_xyxyn([[0.45, 0.45, 0.55, 0.55]], [0.9]), visibility="VALID")
        self.assertEqual(det.track, (0.5, 0.5))


class StatusItemsTests(unittest.TestCase):
    """2026-10-15 사용자 요청: 웹캠 판정 화면 위에 지진·인형·출구 상태를 한글로 띄운다(화면용, 판정에 안 씀)."""

    def items(self, **kw):
        from rescue_perception.webcam_service import status_items
        base = dict(quake_now=False, quake_seen=False, boxes=None, score_min=0.5,
                    presence=None, watched=False)
        base.update(kw)
        return [(title, value) for title, value, _ in status_items(**base)]

    def presence(self, state, reason=None, absent_ns=0):
        return SimpleNamespace(state=state, reason=reason, absent_ns=absent_ns)

    def test_before_quake_three_boxes(self):
        self.assertEqual(self.items(), [("지진", "없음"), ("인형", "대기"), ("출구", "대기")])

    def test_quake_now_and_latched(self):
        self.assertEqual(self.items(quake_now=True)[0], ("지진", "흔들리는 중"))
        self.assertEqual(self.items(quake_seen=True)[0], ("지진", "감지됨"))

    def test_doll_seen_uses_best_passing_score(self):
        boxes = boxes_from_xyxyn([[0.1, 0.1, 0.2, 0.2], [0.3, 0.3, 0.4, 0.4]], [0.93, 0.2])
        self.assertEqual(self.items(quake_seen=True, boxes=boxes)[1], ("인형", "보임 93%"))
        self.assertEqual(self.items(quake_seen=True, boxes=boxes[1:])[1], ("인형", "안 보임"))
        self.assertEqual(self.items(quake_seen=True, boxes=None)[1], ("인형", "탐지기 오류"))

    def test_exit_title_says_view_or_watch(self):
        self.assertEqual(self.items(presence=self.presence("ABSENT_CANDIDATE"))[2], ("출구 · 화면용", "출구로 사라짐"))
        self.assertEqual(self.items(presence=self.presence("ABSENT_CANDIDATE"), watched=True)[2],
                         ("출구 · 감시", "출구로 사라짐"))
        self.assertEqual(self.items(presence=self.presence("PRESENT", absent_ns=1_500_000_000))[2][1], "안 보인 지 1.5초")
        self.assertEqual(self.items(presence=self.presence("PRESENT"))[2][1], "화면 안에 있음")
        self.assertEqual(self.items(presence=self.presence("UNKNOWN", "NOT_VIA_EXIT"))[2][1], "다른 쪽으로 사라짐")
        self.assertEqual(self.items(presence=self.presence("UNKNOWN", "NOT_YET_SEEN"))[2][1], "인형 찾는 중")
        self.assertEqual(self.items(presence=self.presence("UNKNOWN", "OCCLUDED"))[2][1], "화면 가림")

    def test_status_band_paints_the_top_only(self):
        try:
            import cv2  # noqa: F401
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV not installed: status band NOT_RUN")
        from pathlib import Path
        fonts = sorted(Path("/usr/share/fonts").glob("**/NotoSansCJK-*.ttc"))
        if not fonts:
            self.skipTest("no CJK font: status band NOT_RUN")
        from rescue_perception.webcam_service import draw_status, status_items
        bgr = np.full((480, 640, 3), 200, dtype=np.uint8)
        out = draw_status(bgr, status_items(quake_now=True, quake_seen=True, boxes=(),
                                            score_min=0.5, presence=None, watched=False), str(fonts[0]))
        self.assertEqual(out.shape, bgr.shape)
        self.assertTrue((out[:20] != 200).any())     # 위 띠에 그렸다
        self.assertTrue((out[200:] == 200).all())    # 아래는 그대로


if __name__ == "__main__":
    unittest.main()
