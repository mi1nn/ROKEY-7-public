"""E2 tests for fixed camera A disaster signs on SYNTHETIC frames.

No camera, bag, ROS or storage is run (2026-10-08: no recording exists). Frames
are 16x16 synthetic images: a 4-px checkerboard (texture), the same board shifted
by 2 px (whole-view motion), and a uniform frame (covered lens). All numbers are
fixtures, never Q-11 values.
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


def uid(n):
    return str(UUID(int=n))


def board(shift=0):
    return [[200 if ((x + shift) // 4 + y // 4) % 2 else 0 for x in range(16)] for y in range(16)]


COVERED = [[10] * 16 for _ in range(16)]


def scene(size=48, ox=8, oy=8):
    """Fixed pseudo-random texture (values 30..120) seen through a size x size window at (ox, oy)."""
    return [[30 + ((x * 7919 + y * 104729) ** 2 // 13) % 91 for x in range(ox, ox + size)]
            for y in range(oy, oy + size)]


def lit(rows, gain):
    """Same scene under different lighting: every pixel scaled, nothing moves."""
    return [[min(255, round(v * gain)) for v in row] for row in rows]


def image(k, rows, encoding="mono8"):
    if encoding == "mono8":
        data = bytes(v for row in rows for v in row)
    else:
        data = bytes(c for row in rows for v in row for c in (v, v, v))
    channels = 1 if encoding == "mono8" else 3
    height, width = len(rows), len(rows[0])
    return NS(header=NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC + k), frame_id="cam_a/rgb_optical"),
              width=width, height=height, encoding=encoding, is_bigendian=0, step=width * channels, data=data)


class DisasterSignTests(unittest.TestCase):
    def setUp(self):
        self.binding = SourceBinding(None, "cam_a", uid(300), "site-clock", "CAPTURE", "CAPTURE",
                                     "cam-a-cal", "SYNCED", 5000000)
        self.policy = DisasterSignPolicy(sample_stride=1, edge_gradient_min=50.0, use_shake=False,
                                         shake_change_min=0.5, use_occlusion=True, occlusion_density_max=0.05,
                                         rule="ANY", hold_ns=200, release_ns=300, max_gap_ns=150)

    def detector(self, **policy):
        return DisasterSignDetector(self.binding, replace(self.policy, **policy))

    def shake_detector(self):
        return self.detector(use_shake=True, use_occlusion=False)

    def run_frames(self, detector, frames):
        return [detector.update(image(k, rows)) for k, rows in frames]

    def events(self, results):
        return [r.event for r in results if r.event is not None]

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))

    def test_static_texture_raises_nothing(self):
        results = self.run_frames(self.detector(use_shake=True), [(k * 100, board()) for k in range(5)])
        self.assertEqual(self.events(results), [])
        self.assertEqual(results[0].shake, None)
        self.assertEqual(results[1].shake, 0.0)
        self.assertGreater(results[1].edge_density, 0.05)

    def test_occlusion_held_long_enough_emits_one_event_with_exact_stamp(self):
        results = self.run_frames(self.detector(), [(k * 100, COVERED) for k in range(5)])
        events = self.events(results)
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual((event.kind, event.camera_id, event.confidence), ("DISASTER_SIGN", "cam_a", 1.0))
        self.assertEqual(event.observed_at_ns, 1700000000123456989)
        self.assertEqual(event.evidence_id, str(uuid5(EVENT_NAMESPACE, "cam_a/1700000000123456989/frame")))
        self.assertEqual(event.evidence_sha256, hashlib.sha256(bytes([10]) * 256).hexdigest())
        self.assertEqual((event.clock_domain_id, event.stamp_semantics, event.clock_state,
                          event.clock_error_bound_ns), ("site-clock", "CAPTURE", "SYNCED", 5000000))
        self.assertIn("rule=ANY", event.detail)
        self.assertIn("onset_ns=1700000000123456789", event.detail)
        self.assertEqual(results[2].edge_density, 0.0)

    def test_short_occlusion_does_not_emit(self):
        results = self.run_frames(self.detector(), [(0, COVERED), (100, COVERED), (199, board())])
        self.assertEqual(self.events(results), [])

    def test_release_must_hold_before_rearming(self):
        d = self.detector()
        covered = lambda start: [(start + k * 100, COVERED) for k in range(3)]
        clear = lambda start, n: [(start + k * 100, board()) for k in range(n)]
        first = self.events(self.run_frames(d, covered(0)))
        early = self.events(self.run_frames(d, clear(300, 3) + covered(600)))  # clear run 200 ns < release
        late = self.events(self.run_frames(d, clear(900, 4) + covered(1300)))  # clear run 300 ns = release
        self.assertEqual((len(first), len(early), len(late)), (1, 0, 1))

    def test_whole_view_motion_triggers_shake_indicator(self):
        frames = [(k * 100, board(2 * (k % 2))) for k in range(4)]
        results = self.run_frames(self.detector(use_shake=True, use_occlusion=False), frames)
        # Horizontal block edges stay put under a sideways shift, so the change ratio
        # is partial (28/43 edge cells), still above the fixture threshold 0.5.
        self.assertEqual([r.shake for r in results], [None, 28 / 43, 28 / 43, 28 / 43])
        events = self.events(results)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].observed_at_ns, 1700000000123457089)  # onset at k=1, hold 200 ns

    def test_all_rule_needs_every_selected_indicator(self):
        both = dict(use_shake=True, use_occlusion=True, rule="ALL")
        shaking = [(k * 100, board(2 * (k % 2))) for k in range(5)]
        covered = [(k * 100, COVERED) for k in range(5)]
        self.assertEqual(self.events(self.run_frames(self.detector(**both), shaking)), [])
        self.assertEqual(self.events(self.run_frames(self.detector(**both), covered)), [])
        self.assertEqual(len(self.events(self.run_frames(self.detector(use_shake=True, rule="ANY"), shaking))), 1)

    def test_frame_gap_restarts_the_hold(self):
        frames = [(0, COVERED), (100, COVERED), (300, COVERED), (400, COVERED), (500, COVERED)]
        results = self.run_frames(self.detector(), frames)
        self.assertEqual([r.event is not None for r in results], [False, False, False, False, True])

    def test_duplicate_and_reordered_frames_change_nothing(self):
        d = self.detector()
        self.run_frames(d, [(0, COVERED), (100, COVERED)])
        self.assertEqual(d.update(image(100, COVERED)).status, "DUPLICATE")
        self.assertEqual(d.update(image(50, board())).status, "REORDERED")
        self.assertIsNotNone(d.update(image(200, COVERED)).event)

    def test_colour_encodings_use_luma(self):
        mono = self.detector().update(image(0, board()))
        for encoding in ("rgb8", "bgr8"):
            colour = self.detector().update(image(0, board(), encoding))
            self.assertEqual(colour.edge_density, mono.edge_density)

    def test_channel_order_matters(self):
        # Texture only in the first byte: red in rgb8 (luma 0.299*200 = 59.8 >= 50),
        # blue in bgr8 (0.114*200 = 22.8 < 50).
        data = bytes(c for row in board() for v in row for c in (v, 0, 0))
        def coloured(encoding):
            return NS(header=NS(stamp=NS(sec=T0_SEC, nanosec=T0_NSEC), frame_id="cam_a/rgb_optical"),
                      width=16, height=16, encoding=encoding, is_bigendian=0, step=48, data=data)
        self.assertGreater(self.detector().update(coloured("rgb8")).edge_density, 0.05)
        self.assertEqual(self.detector().update(coloured("bgr8")).edge_density, 0.0)

    def test_replay_yields_identical_events(self):
        frames = [(k * 100, COVERED) for k in range(3)]
        self.assertEqual(self.events(self.run_frames(self.detector(), frames)),
                         self.events(self.run_frames(self.detector(), frames)))

    def test_rejected_frame_leaves_no_state(self):
        # Same class as CDX-30-01: a frame rejected while decoding must not advance
        # the stamp or reset the runs, so a corrected retry is processed normally.
        d = self.detector()
        self.run_frames(d, [(0, COVERED), (100, COVERED)])
        bad = image(300, COVERED)
        bad.encoding = "16UC1"  # gap 200 > max_gap 150 would reset the hold if applied
        self.assertCode("INVALID_INPUT", lambda: d.update(bad))
        retry = d.update(image(200, COVERED))
        self.assertEqual(retry.status, "PROCESSED")
        self.assertIsNotNone(retry.event)

    def test_rejected_frame_then_retry_of_same_stamp(self):
        d = self.detector()
        self.run_frames(d, [(0, COVERED)])
        bad = image(100, COVERED)
        bad.encoding = "yuv422"
        self.assertCode("INVALID_INPUT", lambda: d.update(bad))
        self.assertEqual(d.update(image(100, COVERED)).status, "PROCESSED")

    def test_lighting_change_alone_raises_nothing(self):
        # Lights dimmed/brightened/flickering with a still camera: nothing moved, so
        # no disaster sign (PR #33 review, 2026-10-09: edge indicators flip on lighting).
        gains = (1.0, 0.4, 1.0, 0.4, 2.0, 0.4, 1.0, 0.4, 2.0, 1.0)
        frames = [(k * 100, lit(scene(), gain)) for k, gain in enumerate(gains)]
        self.assertEqual(self.events(self.run_frames(self.shake_detector(), frames)), [])

    def test_invalid_inputs_are_rejected(self):
        self.assertCode("INVALID_INPUT", lambda: DisasterSignDetector(replace(self.binding, source_robot="robot_a"),
                                                                      self.policy))
        self.assertCode("INVALID_INPUT", lambda: DisasterSignDetector(replace(self.binding, source_camera="front"),
                                                                      self.policy))
        depth = image(0, COVERED)
        depth.encoding = "16UC1"
        self.assertCode("INVALID_INPUT", lambda: self.detector().update(depth))
        self.assertCode("INVALID_INPUT", lambda: self.detector(sample_stride=16).update(image(0, board())))
        for bad in (dict(use_shake=False, use_occlusion=False), dict(rule="OR"), dict(hold_ns=1.0),
                    dict(edge_gradient_min=0.0), dict(shake_change_min=1.5), dict(max_gap_ns=0),
                    dict(sample_stride=0), dict(use_shake=1)):
            self.assertCode("INVALID_INPUT", lambda bad=bad: replace(self.policy, **bad))


if __name__ == "__main__":
    unittest.main()
