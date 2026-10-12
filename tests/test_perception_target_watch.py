"""고정 웹캠 구조 지점 소실 판정(TARGET_DISAPPEARED_CANDIDATE)의 E2 시험. 모두 합성 입력이다.

카메라·모델·ROS·저장소를 쓰지 않는다. 프레임은 8x8 회색 영상(증거 해시용)이고, 인형 위치는
탐지기가 준 정규화 상자(DetectionBox)로 넣는다. 감시 영역(ROI)은 (0.4, 0.4)~(0.6, 0.6).
모든 숫자는 시험용이며 정책 확정값이 아니다(Q-26).
"""
from dataclasses import replace
import hashlib
from types import SimpleNamespace as NS
import unittest
from uuid import UUID, uuid5

from rescue_contracts import canonical_code

from rescue_perception import PerceptionError, SourceBinding
from rescue_perception.target_watch import (
    EVENT_NAMESPACE, DetectionBox, TargetWatchDetector, TargetWatchPolicy, WatchSpec,
)

T0_SEC, T0_NSEC = 1700000000, 123456789
T0 = T0_SEC * 10**9 + T0_NSEC
PIXELS = bytes(range(64))
TARGET = str(UUID(int=7))
WATCH = str(UUID(int=8))


def image(k):
    return NS(header=NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id="cam_a/rgb_optical"),
              width=8, height=8, encoding="mono8", is_bigendian=0, step=8, data=PIXELS)


DOLL = (DetectionBox(0.45, 0.45, 0.55, 0.55, 0.9),)       # ROI 한가운데
OUTSIDE = (DetectionBox(0.0, 0.0, 0.1, 0.1, 0.9),)        # ROI 밖
NONE = ()


def at(cx):
    """가로 중심이 cx 인 인형 상자(세로는 ROI 가운데)."""
    return (DetectionBox(round(cx - 0.05, 3), 0.45, round(cx + 0.05, 3), 0.55, 0.9),)


class TargetWatchTests(unittest.TestCase):
    def setUp(self):
        self.binding = SourceBinding(None, "cam_a", str(UUID(int=300)), "site-clock", "CAPTURE", "CAPTURE",
                                     "cam-a-cal", "SYNCED", 5000000)
        self.policy = TargetWatchPolicy(score_min=0.5, roi_overlap_min=0.5, present_confirm_frames=2,
                                        absent_hold_ns=300, absent_min_frames=3, max_gap_ns=150,
                                        exit_roi_xyxy_normalized=(0.0, 0.0, 1.0, 1.0), track_max_jump=0.3)
        self.watch = WatchSpec(camera_id="cam_a", watch_id=WATCH, target_id=TARGET,
                               roi_xyxy_normalized=(0.4, 0.4, 0.6, 0.6), watch_revision=1)

    def detector(self, **policy):
        return TargetWatchDetector(self.binding, replace(self.policy, **policy), self.watch)

    def run_frames(self, det, frames):
        """frames: (k, boxes, visibility)."""
        return [det.update(image(k), boxes, visibility=v) for k, boxes, v in frames]

    def seq(self, start, boxes_list, visibility="VALID"):
        return [(start + i * 100, b, visibility) for i, b in enumerate(boxes_list)]

    def events(self, results):
        return [r.event for r in results if r.event is not None]

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))

    def test_doll_in_roi_is_present(self):
        results = self.run_frames(self.detector(), self.seq(0, [DOLL] * 3))
        self.assertEqual([r.state for r in results], ["PRESENT"] * 3)
        self.assertEqual(self.events(results), [])

    def test_doll_removed_raises_one_candidate_with_watch_link(self):
        # 2프레임 확인 → 인형 사라짐: 300ns·3프레임 이상 계속 안 보이면 후보 1개
        results = self.run_frames(self.detector(), self.seq(0, [DOLL, DOLL] + [NONE] * 6))
        self.assertEqual([r.state for r in results],
                         ["PRESENT", "PRESENT", "PRESENT", "PRESENT", "PRESENT", "ABSENT_CANDIDATE",
                          "ABSENT_CANDIDATE", "ABSENT_CANDIDATE"])
        events = self.events(results)
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual((event.kind, event.camera_id, event.target_id, event.watch_id, event.confidence),
                         ("TARGET_DISAPPEARED_CANDIDATE", "cam_a", TARGET, WATCH, 1.0))
        # 처음 안 보인 k=2(200) 부터 300ns 지난 k=5(500)
        self.assertEqual(event.observed_at_ns, T0 + 500)
        self.assertEqual(event.event_id, str(uuid5(EVENT_NAMESPACE, f"cam_a/{WATCH}/candidate")))
        self.assertEqual(event.evidence_id, str(uuid5(EVENT_NAMESPACE, f"cam_a/{T0 + 500}/frame")))
        self.assertEqual(event.evidence_sha256, hashlib.sha256(PIXELS).hexdigest())
        self.assertEqual((event.clock_domain_id, event.stamp_semantics, event.clock_state,
                          event.clock_error_bound_ns), ("site-clock", "CAPTURE", "SYNCED", 5000000))
        self.assertIn(f"absent_since_ns={T0 + 200}", event.detail)
        self.assertIn("watch_revision=1", event.detail)

    def test_brief_absence_is_not_a_candidate(self):
        # 2프레임만 안 보였다가 다시 보임 → 후보 없음
        results = self.run_frames(self.detector(), self.seq(0, [DOLL, DOLL, NONE, NONE, DOLL, DOLL]))
        self.assertEqual(self.events(results), [])
        self.assertEqual(results[-1].state, "PRESENT")

    def test_occlusion_is_unknown_and_never_a_candidate(self):
        # 사람이 가린 동안은 판정 불가. 가림이 길어도 후보를 만들지 않는다(운영자 판단 대상)
        frames = self.seq(0, [DOLL, DOLL]) + self.seq(200, [NONE] * 8, "OCCLUDED")
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.state for r in results[2:]}, {"UNKNOWN"})
        self.assertEqual({r.reason for r in results[2:]}, {"OCCLUDED"})
        self.assertEqual(self.events(results), [])

    def test_absence_restarts_after_occlusion(self):
        # 안 보임 2프레임 → 가림 → 다시 정상: 소실 시간은 처음부터 다시 잰다
        frames = (self.seq(0, [DOLL, DOLL, NONE, NONE]) + [(400, NONE, "OCCLUDED")]
                  + self.seq(500, [NONE] * 4))
        results = self.run_frames(self.detector(), frames)
        events = self.events(results)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].observed_at_ns, T0 + 800)  # 500 부터 300ns

    def test_unknown_stream_is_unknown(self):
        results = self.run_frames(self.detector(), self.seq(0, [DOLL, DOLL]) + [(200, NONE, "UNKNOWN")])
        self.assertEqual((results[-1].state, results[-1].reason), ("UNKNOWN", "VISIBILITY_UNKNOWN"))

    def test_detector_without_result_is_unknown(self):
        results = self.run_frames(self.detector(), self.seq(0, [DOLL, DOLL]) + [(200, None, "VALID")])
        self.assertEqual((results[-1].state, results[-1].reason), ("UNKNOWN", "DETECTOR_UNAVAILABLE"))

    def test_never_seen_doll_does_not_arm(self):
        # ROI 를 잘못 잡아 인형을 한 번도 못 봤으면 소실 후보를 내지 않고 판정 불가로 알린다
        results = self.run_frames(self.detector(), self.seq(0, [NONE] * 8))
        self.assertEqual({(r.state, r.reason) for r in results}, {("UNKNOWN", "NOT_YET_SEEN")})
        self.assertEqual(self.events(results), [])

    def test_doll_outside_roi_or_low_score_counts_as_absent(self):
        weak = (DetectionBox(0.45, 0.45, 0.55, 0.55, 0.2),)
        results = self.run_frames(self.detector(), self.seq(0, [DOLL, DOLL, OUTSIDE, weak, NONE, NONE]))
        self.assertEqual(len(self.events(results)), 1)

    def test_partial_overlap_uses_box_fraction_inside_roi(self):
        # 상자의 절반 이상이 ROI 안이면 있음, 아니면 없음 (roi_overlap_min=0.5)
        half_in = (DetectionBox(0.5, 0.45, 0.7, 0.55, 0.9),)    # 50% 안
        mostly_out = (DetectionBox(0.55, 0.45, 0.75, 0.55, 0.9),)  # 25% 안
        det = self.detector()
        states = [r.state for r in self.run_frames(det, self.seq(0, [DOLL, DOLL, half_in]))]
        self.assertEqual(states[-1], "PRESENT")
        r = self.run_frames(det, self.seq(300, [mostly_out] * 3))
        self.assertEqual(r[-1].absent_frames, 3)

    def test_one_candidate_per_watch(self):
        frames = self.seq(0, [DOLL, DOLL] + [NONE] * 5 + [DOLL] * 3 + [NONE] * 5)
        self.assertEqual(len(self.events(self.run_frames(self.detector(), frames))), 1)

    def test_frame_gap_restarts_absence(self):
        frames = self.seq(0, [DOLL, DOLL, NONE, NONE]) + self.seq(600, [NONE] * 3)
        results = self.run_frames(self.detector(), frames)
        self.assertEqual(results[4].reason, "FRAME_GAP")
        self.assertEqual(self.events(results), [])  # 600 부터 다시 재서 아직 300ns 안 됨

    def test_duplicate_and_reordered_frames_change_nothing(self):
        det = self.detector()
        self.run_frames(det, self.seq(0, [DOLL, DOLL, NONE]))
        dup = det.update(image(200), NONE, visibility="VALID")
        old = det.update(image(100), NONE, visibility="VALID")
        self.assertEqual([dup.status, old.status], ["DUPLICATE", "REORDERED"])
        self.assertEqual(det.update(image(300), NONE, visibility="VALID").absent_frames, 2)

    def test_replay_yields_identical_results(self):
        frames = self.seq(0, [DOLL, DOLL, NONE]) + [(300, NONE, "OCCLUDED")] + self.seq(400, [NONE] * 5)
        self.assertEqual(self.run_frames(self.detector(), frames), self.run_frames(self.detector(), frames))

    # -- 출구 방향 (Q-26 추가 항목, 사용자 결정 2026-10-12: 출구 쪽으로 사라질 때만 후보) --------------
    # 출구 영역은 화면 오른쪽 띠. 위 시험들의 출구는 화면 전체(어느 방향이든 출구)다.

    def exit_detector(self):
        return self.detector(exit_roi_xyxy_normalized=(0.8, 0.0, 1.0, 1.0))

    def test_doll_carried_out_through_exit_is_candidate(self):
        results = self.run_frames(self.exit_detector(), self.seq(0, [at(0.5), at(0.5), at(0.7), at(0.9)] + [NONE] * 4))
        events = self.events(results)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].observed_at_ns, T0 + 500)
        self.assertIn("last_seen_center=(0.9", events[0].detail)

    def test_doll_carried_out_the_other_way_is_not_a_candidate(self):
        results = self.run_frames(self.exit_detector(), self.seq(0, [at(0.5), at(0.5), at(0.3), at(0.1)] + [NONE] * 4))
        self.assertEqual(self.events(results), [])
        self.assertEqual((results[-1].state, results[-1].reason), ("UNKNOWN", "NOT_VIA_EXIT"))

    def test_doll_moved_within_frame_is_not_a_candidate(self):
        # 출구 영역 안으로 옮겨 놓아도 화면에 보이는 동안은 사라진 것이 아니다
        results = self.run_frames(self.exit_detector(), self.seq(0, [at(0.5), at(0.5), at(0.7)] + [at(0.9)] * 6))
        self.assertEqual(self.events(results), [])
        self.assertEqual((results[-1].state, results[-1].reason), ("UNKNOWN", "NOT_VIA_EXIT"))

    def test_other_doll_does_not_hide_the_watched_doll_leaving(self):
        other = DetectionBox(0.05, 0.45, 0.15, 0.55, 0.9)
        frames = [(b + (other,)) for b in (at(0.5), at(0.5), at(0.7), at(0.9))] + [(other,)] * 4
        self.assertEqual(len(self.events(self.run_frames(self.exit_detector(), self.seq(0, frames)))), 1)

    def test_vanishing_in_place_is_not_a_candidate(self):
        # 출구가 아닌 곳에서 갑자기 안 보이면(탐지 실패·사람이 가림) 후보 없이 운영자 판단으로 넘긴다
        results = self.run_frames(self.exit_detector(), self.seq(0, [at(0.5), at(0.5)] + [NONE] * 6))
        self.assertEqual(self.events(results), [])
        self.assertEqual(results[-1].reason, "NOT_VIA_EXIT")

    def test_invalid_inputs_are_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: TargetWatchDetector(
            replace(self.binding, source_robot="robot_a"), self.policy, self.watch))
        self.assertCode("INVALID_INPUT", lambda: TargetWatchDetector(
            self.binding, self.policy, replace(self.watch, camera_id="cam_b")))  # 바인딩과 다른 카메라
        for bad in (dict(watch_id="w1"), dict(target_id=str(UUID(int=0))), dict(roi_xyxy_normalized=(0.6, 0.4, 0.4, 0.6)),
                    dict(roi_xyxy_normalized=(0.0, 0.0, 1.2, 1.0)), dict(roi_xyxy_normalized=[0.1, 0.1, 0.2, 0.2]),
                    dict(watch_revision=-1)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.watch, **bad))
        for bad in (dict(score_min=1.5), dict(roi_overlap_min=0.0), dict(present_confirm_frames=0),
                    dict(absent_hold_ns=0), dict(absent_min_frames=0), dict(max_gap_ns=0), dict(absent_hold_ns=1.0)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.policy, **bad))
        for bad in (dict(exit_roi_xyxy_normalized=(0.8, 0.0, 0.7, 1.0)), dict(exit_roi_xyxy_normalized=[0.8, 0.0, 1.0, 1.0]),
                    dict(track_max_jump=0.0), dict(track_max_jump=2.0)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.policy, **bad))
        for bad in (dict(x0=0.6), dict(y1=1.5), dict(score=-0.1), dict(x0=float("nan"))):
            box = dict(x0=0.4, y0=0.4, x1=0.5, y1=0.5, score=0.9)
            box.update(bad)
            self.assertCode("INVALID_INPUT", lambda box=box: DetectionBox(**box))
        det = self.detector()
        self.assertCode("INVALID_INPUT", lambda: det.update(image(0), [DOLL[0]], visibility="VALID"))
        self.assertCode("INVALID_INPUT", lambda: det.update(image(0), DOLL, visibility="HIDDEN"))


if __name__ == "__main__":
    unittest.main()
