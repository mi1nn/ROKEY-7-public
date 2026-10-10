"""고정 웹캠 지진 징후(흔들림 횟수)의 E2 시험. 모두 합성 영상이다.

카메라·bag·ROS·저장소를 쓰지 않는다. 장면은 48x48 의사 난수 무늬이고, 보는 창의
위치(ox, oy)를 옮겨 카메라 흔들림을, 밝기를 곱해 조명 변화를, 일부 영역만 옮겨
사람 통과를 흉내 낸다. 모든 숫자는 시험용이며 Q-11 값이 아니다.
"""
from dataclasses import replace
import hashlib
from types import SimpleNamespace as NS
import unittest
from uuid import UUID, uuid5

from rescue_contracts import canonical_code

from rescue_perception import PerceptionError, SourceBinding
from rescue_perception.disaster_sign import EVENT_NAMESPACE, DisasterSignDetector, DisasterSignPolicy

T0_SEC, T0_NSEC = 1700000000, 123456789
T0 = T0_SEC * 10**9 + T0_NSEC


def uid(n):
    return str(UUID(int=n))


def scene(size=48, ox=8, oy=8):
    """고정된 의사 난수 무늬(값 30..120)를 (ox, oy) 위치의 size x size 창으로 본 것."""
    return [[30 + ((x * 7919 + y * 104729) ** 2 // 13) % 91 for x in range(ox, ox + size)]
            for y in range(oy, oy + size)]


def lit(rows, gain):
    """같은 장면, 다른 조명: 모든 픽셀에 같은 배율. 아무것도 움직이지 않는다."""
    return [[min(255, round(v * gain)) for v in row] for row in rows]


def moved_where(mask, dx):
    """mask 가 참인 픽셀만 dx 만큼 옮긴 장면(화면 일부만 움직임 = 사람 통과)."""
    still, moving = scene(), scene(ox=8 + dx)
    return [[moving[y][x] if mask(x, y) else still[y][x] for x in range(48)] for y in range(48)]


def hand(ox=0, size=48):
    """렌즈 바로 앞의 손: 장면과 전혀 다른 무늬가 화면 전체를 덮는다(값 40..100)."""
    return [[40 + ((x * 31337 + y * 7331) ** 2 // 7) % 61 for x in range(ox, ox + size)] for y in range(size)]


COVERED = [[10] * 48 for _ in range(48)]


def image(k, rows, encoding="mono8"):
    if encoding == "mono8":
        data = bytes(v for row in rows for v in row)
    else:
        data = bytes(c for row in rows for v in row for c in (v, v, v))
    channels = 1 if encoding == "mono8" else 3
    height, width = len(rows), len(rows[0])
    return NS(header=NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id="cam_a/rgb_optical"),
              width=width, height=height, encoding=encoding, is_bigendian=0, step=width * channels, data=data)


def shake(start, n, first=8):
    """창을 좌우로 2칸씩 왕복: first, first+2, first, ... (100 ns 간격)."""
    return [(start + k * 100, scene(ox=first + 2 * (k % 2))) for k in range(n)]


def still(start, n, ox=8):
    return [(start + k * 100, scene(ox=ox)) for k in range(n)]


class DisasterSignTests(unittest.TestCase):
    def setUp(self):
        self.binding = SourceBinding(None, "cam_a", uid(300), "site-clock", "CAPTURE", "CAPTURE",
                                     "cam-a-cal", "SYNCED", 5000000)
        # 격자 48x48, 여백 2칸 -> 블록 4x4 개(각 11x11 칸).
        self.policy = DisasterSignPolicy(sample_stride=1, blocks_x=4, blocks_y=4, search_cells=2,
                                         texture_min=5.0, match_min=0.8, valid_ratio_min=0.5, agree_tol=1.0,
                                         shift_min=1.5, scene_match_min=0.5, shakes_min=3, window_ns=1000, hold_ns=100,
                                         release_ns=300, max_gap_ns=150)

    def detector(self, **policy):
        return DisasterSignDetector(self.binding, replace(self.policy, **policy))

    def shake_detector(self):
        return self.detector()

    def run_frames(self, detector, frames):
        return [detector.update(image(k, rows)) for k, rows in frames]

    def events(self, results):
        return [r.event for r in results if r.event is not None]

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))

    def test_still_scene_raises_nothing(self):
        results = self.run_frames(self.detector(), still(0, 5))
        self.assertEqual(self.events(results), [])
        self.assertEqual(results[0].reason, "FIRST_FRAME")
        self.assertEqual((results[1].shift, results[1].reason, results[1].reversals), ((0, 0), "BELOW_NOISE", 0))

    def test_known_shift_is_measured(self):
        # 창을 오른쪽 2칸, 위로 1칸 옮기면 (dx, dy) = (2, -1): 카메라가 움직인 방향
        results = self.run_frames(self.detector(), [(0, scene()), (100, scene(ox=10, oy=7))])
        self.assertEqual(results[1].shift, (2, -1))
        self.assertEqual((results[1].textured_ratio, results[1].matched_ratio), (1.0, 1.0))

    def test_whole_view_shake_emits_one_event_with_exact_stamp(self):
        results = self.run_frames(self.detector(), shake(0, 8))
        # k=1 첫 방향 기록, k=2,3,4 에서 방향 전환 -> k=4 에 3회(조건 참), hold 100 ns -> k=5 에 사건
        self.assertEqual([r.reversals for r in results], [0, 0, 1, 2, 3, 4, 5, 6])
        events = self.events(results)
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual((event.kind, event.camera_id, event.confidence), ("DISASTER_SIGN", "cam_a", 1.0))
        self.assertEqual(event.observed_at_ns, T0 + 500)
        self.assertEqual(event.evidence_id, str(uuid5(EVENT_NAMESPACE, f"cam_a/{T0 + 500}/frame")))
        expected = bytes(v for row in scene(ox=10) for v in row)
        self.assertEqual(event.evidence_sha256, hashlib.sha256(expected).hexdigest())
        self.assertEqual((event.clock_domain_id, event.stamp_semantics, event.clock_state,
                          event.clock_error_bound_ns), ("site-clock", "CAPTURE", "SYNCED", 5000000))
        self.assertIn("reversals=4>=3", event.detail)
        self.assertIn(f"onset_ns={T0 + 400}", event.detail)

    def test_fewer_reversals_than_needed_raise_nothing(self):
        results = self.run_frames(self.detector(), shake(0, 4) + still(400, 3, ox=10))  # 마지막 자리에서 멈춤
        self.assertEqual(max(r.reversals for r in results), 2)
        self.assertEqual(self.events(results), [])

    def test_lighting_change_alone_raises_nothing(self):
        # 카메라는 그대로, 조명만 어두워지고 밝아지고 깜빡임: 움직인 게 없으니 사건 없음
        # (PR #33 검토 2026-10-09: 엣지 지표는 조명에 뒤집힌다).
        gains = (1.0, 0.4, 1.0, 0.4, 2.0, 0.4, 1.0, 0.4, 2.0, 1.0)
        frames = [(k * 100, lit(scene(), gain)) for k, gain in enumerate(gains)]
        self.assertEqual(self.events(self.run_frames(self.shake_detector(), frames)), [])

    def test_brightness_offset_alone_raises_nothing(self):
        # 주변 조명이 켜져 모든 픽셀이 같은 값만큼 밝아지거나 어두워짐(곱이 아니라 덧셈)
        offsets = (0, 60, 0, -25, 60, 0, -25, 60, 0, 60)
        frames = [(k * 100, [[v + d for v in row] for row in scene()]) for k, d in enumerate(offsets)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.shift for r in results[1:]}, {(0, 0)})
        self.assertEqual(self.events(results), [])

    def test_steady_pan_is_not_shaking(self):
        # 한 방향으로 계속 밀림(카메라가 천천히 돌아감): 이동은 크지만 방향 전환이 없다
        frames = [(k * 100, scene(ox=8 + 2 * k)) for k in range(8)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.shift for r in results[1:]}, {(2, 0)})
        self.assertEqual(max(r.reversals for r in results), 0)
        self.assertEqual(self.events(results), [])

    def test_one_quadrant_moving_is_partial_motion(self):
        # 왼쪽 위 사분면만 왕복(사람이 지나감): 나머지 세 사분면은 멈춰 있다
        top_left = lambda x, y: x < 24 and y < 24
        frames = [(k * 100, moved_where(top_left, 2 * (k % 2))) for k in range(8)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.reason for r in results[1:]}, {"PARTIAL_MOTION"})
        self.assertEqual(self.events(results), [])

    def test_three_quadrants_moving_is_still_partial_motion(self):
        # 가까운 물체가 화면 대부분을 덮고 움직여도 멈춘 사분면 하나가 남으면 흔들림이 아니다
        all_but_bottom_right = lambda x, y: x < 24 or y < 24
        frames = [(k * 100, moved_where(all_but_bottom_right, 2 * (k % 2))) for k in range(8)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.reason for r in results[1:]}, {"PARTIAL_MOTION"})
        self.assertEqual(self.events(results), [])

    def test_hand_moving_right_in_front_of_lens_is_not_shaking(self):
        # 손이 렌즈 가까이서 화면 전체를 덮고 왔다 갔다 함(2026-10-10 실측 오탐): 16칸이 모두
        # 같이 움직이지만 보이는 것이 원래 장면이 아니다 -> 세지 않는다.
        frames = still(0, 3) + [(300 + k * 100, hand(ox=2 * (k % 2))) for k in range(10)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual(self.events(results), [])
        self.assertEqual(max(r.reversals for r in results), 0)

    def test_hand_pausing_briefly_does_not_become_the_scene(self):
        # 손이 렌즈 앞에서 잠깐 멈췄다가 다시 움직임: 잠깐 멈춘 손 화면을 기준 장면으로
        # 삼으면 그다음 손 움직임을 흔들림으로 센다(합성 영상 재생에서 발견, 2026-10-10).
        frames = (still(0, 3) + [(300, hand()), (400, hand()), (500, hand())]
                  + [(600 + k * 100, hand(ox=2 * ((k + 1) % 2))) for k in range(8)])
        results = self.run_frames(self.detector(), frames)
        self.assertEqual(self.events(results), [])
        self.assertEqual(max(r.reversals for r in results), 0)

    def test_camera_moved_to_a_new_spot_becomes_the_new_scene(self):
        # 카메라가 밀려 새 자리에 놓이고 시간 창 이상 멈추면 그 화면이 기준 장면이 된다.
        # 그 뒤의 흔들림은 새 기준 장면으로 확인해 센다.
        frames = still(0, 3) + still(300, 13, ox=20) + shake(1600, 6, first=20)
        events = self.events(self.run_frames(self.detector(), frames))
        self.assertEqual([e.observed_at_ns for e in events], [T0 + 2100])

    def test_jitter_below_noise_floor_is_not_counted(self):
        frames = [(k * 100, scene(ox=8 + k % 2)) for k in range(8)]  # 1칸 왕복 < shift_min 1.5
        results = self.run_frames(self.detector(), frames)
        self.assertEqual({r.reason for r in results[1:]}, {"BELOW_NOISE"})
        self.assertEqual(max(r.reversals for r in results), 0)
        self.assertEqual(self.events(results), [])

    def test_covered_view_is_unknown_and_keeps_the_count(self):
        frames = shake(0, 3) + [(300, COVERED), (400, scene(ox=10))] + shake(500, 4)
        results = self.run_frames(self.detector(), frames)
        self.assertEqual([(r.status, r.reason) for r in results[3:5]],
                         [("UNKNOWN", "OCCLUDED"), ("UNKNOWN", "MATCH_FAILED")])  # 가림, 가림 직후
        self.assertEqual([r.reversals for r in results[2:5]], [1, 1, 1])  # 더하지도 지우지도 않음
        events = self.events(results)
        self.assertEqual([e.observed_at_ns for e in events], [T0 + 800])

    def test_release_must_hold_before_rearming(self):
        d = self.detector(release_ns=500)
        first = self.events(self.run_frames(d, shake(0, 6)))  # 사건 500
        # 멈추면 1400 에 조건 거짓(창 안 전환 2회) -> 1800 에 다시 참: 거짓 300 ns < release 500
        early_results = self.run_frames(d, still(600, 9) + shake(1500, 6))
        early = self.events(early_results)
        # 2900 에 거짓 -> 3400 에 다시 낼 수 있음 -> 3800 에 참, hold 100 -> 3900 사건
        late = self.events(self.run_frames(d, still(2100, 14) + shake(3500, 6)))
        self.assertEqual((len(first), len(early), len(late)), (1, 0, 1))
        self.assertEqual([r.condition for r in early_results if r.event is None][8:12],
                         [False, False, False, False])  # 1400..1700 은 거짓이었다
        self.assertEqual(late[0].observed_at_ns, T0 + 3900)

    def test_frame_gap_resets_the_count(self):
        frames = shake(0, 4) + shake(500, 6)  # 300 -> 500 간격 200 > max_gap 150
        results = self.run_frames(self.detector(), frames)
        self.assertEqual((results[4].reason, results[4].reversals), ("FIRST_FRAME", 0))
        self.assertEqual([e.observed_at_ns for e in self.events(results)], [T0 + 1000])

    def test_hold_counts_from_when_the_condition_became_true(self):
        results = self.run_frames(self.detector(hold_ns=200), shake(0, 8))
        self.assertEqual([e.observed_at_ns for e in self.events(results)], [T0 + 600])

    def test_duplicate_and_reordered_frames_change_nothing(self):
        d = self.detector()
        self.run_frames(d, shake(0, 3))
        self.assertEqual(d.update(image(200, scene())).status, "DUPLICATE")
        self.assertEqual(d.update(image(50, scene(ox=10))).status, "REORDERED")
        rest = [(300, scene(ox=10)), (400, scene()), (500, scene(ox=10))]  # 왕복을 그대로 이어감
        self.assertEqual([e.observed_at_ns for e in self.events(self.run_frames(d, rest))], [T0 + 500])

    def test_colour_encodings_use_luma(self):
        frames = [(0, scene()), (100, scene(ox=10, oy=7))]
        for encoding in ("rgb8", "bgr8"):
            d = self.detector()
            colour = [d.update(image(k, rows, encoding)) for k, rows in frames]
            self.assertEqual(colour[1].shift, (2, -1))

    def test_channel_order_matters(self):
        # 무늬가 첫 바이트에만 있다: rgb8 에서는 빨강(가중치 0.299, 표준편차 약 7.8 >= 5),
        # bgr8 에서는 파랑(0.114, 약 3.0 < 5) -> 무늬 부족.
        def coloured(k, rows, encoding):
            data = bytes(c for row in rows for v in row for c in (v, 0, 0))
            return NS(header=NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id="cam_a/rgb_optical"),
                      width=48, height=48, encoding=encoding, is_bigendian=0, step=144, data=data)
        for encoding, status in (("rgb8", "PROCESSED"), ("bgr8", "UNKNOWN")):
            d = self.detector()
            d.update(coloured(0, scene(), encoding))
            self.assertEqual(d.update(coloured(100, scene(ox=10), encoding)).status, status)

    def test_replay_yields_identical_events(self):
        self.assertEqual(self.events(self.run_frames(self.detector(), shake(0, 8))),
                         self.events(self.run_frames(self.detector(), shake(0, 8))))

    def test_rejected_frame_leaves_no_state(self):
        # CDX-30-01 과 같은 종류: 디코딩에서 거절된 프레임이 시각을 올리거나 구간을
        # 초기화하면 안 된다. 고친 프레임을 다시 넣으면 정상 처리돼야 한다.
        d = self.detector()
        self.run_frames(d, shake(0, 5))  # k=4(400)에 조건 참
        bad = image(600, scene())
        bad.encoding = "16UC1"  # 적용됐다면 간격 200 > 150 으로 구간이 초기화됐을 것
        self.assertCode("INVALID_INPUT", lambda: d.update(bad))
        retry = d.update(image(500, scene(ox=10)))
        self.assertEqual(retry.status, "PROCESSED")
        self.assertIsNotNone(retry.event)

    def test_frame_size_change_restarts_instead_of_crashing(self):
        # 웹캠이 켜지면서 해상도를 바꾸면 첫 프레임과 다음 프레임 크기가 다르다(2026-10-10 실측
        # IndexError). 크기가 바뀌면 직전 프레임과 비교하지 않고 처음부터 다시 센다.
        d = self.detector()
        self.run_frames(d, [(k * 100, scene(size=40, ox=8 + 2 * (k % 2))) for k in range(4)])
        result = d.update(image(400, scene()))  # 40x40 -> 48x48
        self.assertEqual((result.status, result.reason, result.reversals), ("PROCESSED", "FIRST_FRAME", 0))
        self.assertEqual(d.update(image(500, scene(ox=10))).shift, (2, 0))

    def test_rejected_frame_then_retry_of_same_stamp(self):
        d = self.detector()
        self.run_frames(d, still(0, 1))
        bad = image(100, scene())
        bad.encoding = "yuv422"
        self.assertCode("INVALID_INPUT", lambda: d.update(bad))
        self.assertEqual(d.update(image(100, scene())).status, "PROCESSED")

    def test_invalid_inputs_are_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: DisasterSignDetector(replace(self.binding, source_robot="robot_a"),
                                                                      self.policy))
        self.assertCode("INVALID_INPUT", lambda: DisasterSignDetector(replace(self.binding, source_camera="front"),
                                                                      self.policy))
        depth = image(0, COVERED)
        depth.encoding = "16UC1"
        self.assertCode("INVALID_INPUT", lambda: self.detector().update(depth))
        self.assertCode("INVALID_INPUT", lambda: self.detector(sample_stride=16).update(image(0, scene())))
        for bad in (dict(blocks_x=3), dict(blocks_y=0), dict(search_cells=0), dict(texture_min=0.0),
                    dict(match_min=1.5), dict(valid_ratio_min=0.0), dict(shift_min=0.0), dict(shakes_min=0),
                    dict(hold_ns=1001), dict(hold_ns=1.0), dict(window_ns=0), dict(max_gap_ns=0),
                    dict(sample_stride=0), dict(agree_tol=-1.0), dict(scene_match_min=1.5)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.policy, **bad))


if __name__ == "__main__":
    unittest.main()
