"""고정 웹캠 노드의 판정 경계(감시 설정 수락·영상 상태 → 시야·탐지 상자 변환)의 E2 시험. 모두 합성 입력이다.

ROS·카메라·모델을 쓰지 않는다. 노드(`webcam_node`)는 이 모듈의 결과를 메시지에 옮기기만 한다.
숫자는 시험용이며 정책 확정값이 아니다.
"""
from dataclasses import replace
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
                                   absent_hold_ns=300, absent_min_frames=3, max_gap_ns=150)
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
    """예시 파라미터 파일이 노드가 요구하는 키와 정확히 같고, 그 값으로 두 정책이 만들어지는지."""

    def test_example_yaml_matches_node_params(self):
        import re
        from pathlib import Path
        from rescue_perception.webcam_node import PARAMS, policies
        path = Path(__file__).resolve().parents[1] / "src/rescue_perception/config/webcam_node.example.yaml"
        values = {}
        for line in path.read_text().splitlines():
            m = re.match(r"\s{4}(\w+):\s*([^#]+?)\s*(#.*)?$", line)
            if m:
                kind, raw = PARAMS.get(m.group(1)), m.group(2)
                values[m.group(1)] = int(raw) if kind == "INTEGER" else float(raw) if kind == "DOUBLE" else raw
        self.assertEqual(set(values), set(PARAMS))
        policies(values)  # 값 검사는 각 정책이 한다


if __name__ == "__main__":
    unittest.main()
