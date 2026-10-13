"""E2 recorded-input adapter tests on SYNTHETIC ROS-shaped messages.

No bag recorded by the robot/mapping lane existed on 2026-10-08, so every message
below is constructed here. No ROS, camera, model, TF or robot is run. All numbers
are test fixtures, never hardware defaults.
"""
from dataclasses import replace
import hashlib
import math
import struct
from types import SimpleNamespace as NS
import unittest
from uuid import UUID

from rescue_contracts import canonical_code

from rescue_perception import (
    AlignmentEvidence, DepthPolicy, Detection, ModelManifest, ObservationPolicy, ObservationRegistry,
    PerceptionError, SourceBinding, TimePolicy, camera_intrinsics, depth_frame,
    detection_from_vision_msgs, observe,
)

EPOCH_SEC, EPOCH_NSEC = 1700000000, 123456789
EPOCH_NS = 1700000000123456789  # literal on purpose: no arithmetic on the expected value
FRAME = "robot_a/rgb_optical"


def uid(n):
    return str(UUID(int=n))


def header(sec=EPOCH_SEC, nanosec=EPOCH_NSEC, frame_id=FRAME):
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame_id)


def rgb_image(pixel=0, **hdr):
    return NS(header=header(**hdr), width=4, height=4, encoding="rgb8", is_bigendian=0,
              step=12, data=bytes([pixel]) * 48)


def depth_image(values=None, *, encoding="16UC1", big=False, pad=0, width=4, height=4, **hdr):
    code, size = {"16UC1": ("H", 2), "32FC1": ("f", 4)}.get(encoding, ("H", 2))
    values = values if values is not None else [[2000] * width for _ in range(height)]
    row_fmt = (">" if big else "<") + code * width
    data = b"".join(struct.pack(row_fmt, *row) + b"\0" * pad for row in values)
    return NS(header=header(**hdr), width=width, height=height, encoding=encoding,
              is_bigendian=int(big), step=width * size + pad, data=data)


def camera_info(*, k=(2.0, 0, 1.5, 0, 2.0, 1.5, 0, 0, 1), p=None, frame_id=FRAME, binning=0, roi=None,
                model="plumb_bob", d=(0.0,) * 5):
    p = p if p is not None else (2.0, 0, 1.5, 0, 0, 2.0, 1.5, 0, 0, 0, 1, 0)
    return NS(header=header(frame_id=frame_id), width=4, height=4, k=k, p=p, d=d,
              distortion_model=model, binning_x=binning, binning_y=binning,
              roi=roi or NS(x_offset=0, y_offset=0, width=0, height=0))


def detection2d(label="person", score=0.9, track="", centre=(2.0, 2.0), size=(4.0, 4.0), results=None):
    hyp = NS(hypothesis=NS(class_id=label, score=score))
    return NS(header=header(), id=track, results=[hyp] if results is None else results,
              bbox=NS(center=NS(position=NS(x=centre[0], y=centre[1])), size_x=size[0], size_y=size[1]))


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.binding = SourceBinding("robot_a", "front", uid(100), "test-clock", "CAPTURE", "CAPTURE",
                                     "test-cal", "SYNCED", 10000000)
        self.alignment = AlignmentEvidence("test-alignment", "test-cal", FRAME, FRAME, True)
        self.model = ModelManifest("test-model", "a" * 64, ("person", "dummy"))
        self.depth_policy = DepthPolicy(20000000, 0.5, 5.0, 2, 0.5, 0.5)
        self.observation_policy = ObservationPolicy(2000000000)
        self.time_policy = TimePolicy(2000000000, 20000000, 40000000, 10000000)
        self.detection = Detection(0, "person", 0.9, (0, 0, 4, 4), None, "DETECTION_ONLY")

    def run_adapter(self, n=1, rgb=None, depth=None, info=None, detection=None, det_header=None, **kw):
        args = dict(binding=self.binding, alignment=self.alignment, rectified=True, model=self.model,
                    depth_policy=self.depth_policy, observation_policy=self.observation_policy,
                    observation_id=uid(n), evidence_id=f"synthetic-{n}")
        args.update(kw)
        return observe(rgb or rgb_image(), depth or depth_image(), info or camera_info(),
                       detection or self.detection, det_header or header(), **args)

    def ingest(self, registry, observation, steady, now_ns=EPOCH_NS + 500000000):
        return registry.ingest(observation, now_ns=now_ns, received_steady=steady,
                               receiver_clock_domain="test-clock",
                               receiver_clock_error_bound_ns=10000000, policy=self.time_policy)

    def assertCode(self, code, fn):
        with self.assertRaises(PerceptionError) as ctx:
            fn()
        self.assertEqual(ctx.exception.code, canonical_code("rescue_perception", code))
        return ctx.exception

    # Stamp preservation -------------------------------------------------------
    def test_header_stamp_reaches_observation_without_losing_a_nanosecond(self):
        result = self.run_adapter()
        self.assertEqual(result.observation.observed_at_ns, 1700000000123456789)
        self.assertEqual(result.observation.frame.observed_at_ns, EPOCH_NS)
        self.assertEqual(result.observation.valid_until_ns, 1700000002123456789)
        self.assertEqual(result.localization.point_optical, (0.0, 0.0, 2.0))

    def test_float_header_stamp_is_rejected_not_rounded(self):
        self.assertCode("INVALID_INPUT", lambda: self.run_adapter(rgb=rgb_image(sec=1700000000.0)))

    def test_detection_from_another_frame_is_rejected_by_one_nanosecond(self):
        self.assertCode("INVALID_INPUT", lambda: self.run_adapter(det_header=header(nanosec=EPOCH_NSEC + 1)))
        self.assertCode("INVALID_INPUT", lambda: self.run_adapter(det_header=header(frame_id="other")))

    # Depth decoding ------------------------------------------------------------
    def test_16uc1_little_endian_with_row_padding(self):
        values = [[1000 * y + x for x in range(1, 5)] for y in range(1, 5)]
        frame = depth_frame(depth_image(values, pad=3), self.binding)
        self.assertEqual(frame.rows, tuple(tuple(row) for row in values))
        self.assertEqual((frame.encoding, frame.unit), ("16UC1", "mm"))

    def test_32fc1_big_endian_keeps_nan_as_missing(self):
        values = [[math.nan, 2.0, 2.0, 2.0]] + [[2.0] * 4] * 3
        frame = depth_frame(depth_image(values, encoding="32FC1", big=True), self.binding)
        self.assertTrue(math.isnan(frame.rows[0][0]))
        self.assertEqual(frame.rows[1], (2.0, 2.0, 2.0, 2.0))
        result = self.run_adapter(depth=depth_image(values, encoding="32FC1", big=True))
        self.assertEqual(result.localization.valid_pixels, 15)

    def test_unsupported_encoding_and_bad_length_are_rejected(self):
        self.assertCode("UNSUPPORTED_DEPTH", lambda: self.run_adapter(depth=depth_image(encoding="mono16")))
        short = depth_image()
        short.data = short.data[:-1]
        self.assertCode("INVALID_INPUT", lambda: self.run_adapter(depth=short))
        listed = depth_image()
        listed.data = list(listed.data)
        self.assertCode("INVALID_INPUT", lambda: self.run_adapter(depth=listed))

    # Failure reasons keep the discovery without a position --------------------
    def assertUnlocated(self, reason, result):
        self.assertEqual(result.localization.reason, reason)
        self.assertIsNone(result.localization.point_optical)
        self.assertIsNotNone(result.observation)
        self.assertIsNone(result.observation.localization.point_optical)

    def test_bad_time_alignment_or_depth_never_produce_a_point(self):
        unknown = replace(self.binding, depth_stamp_semantics="UNKNOWN")
        self.assertUnlocated("CLOCK_UNCERTAIN", self.run_adapter(binding=unknown))
        mixed = replace(self.binding, depth_stamp_semantics="DRIVER_RECEIVE")
        self.assertUnlocated("CLOCK_UNCERTAIN", self.run_adapter(binding=mixed))
        late = depth_image(nanosec=EPOCH_NSEC + 20000001)
        self.assertUnlocated("UNSYNCHRONIZED", self.run_adapter(depth=late))
        edge = depth_image(nanosec=EPOCH_NSEC + 20000000)
        self.assertEqual(self.run_adapter(depth=edge).localization.reason, "LOCATED")
        unverified = replace(self.alignment, verified=False)
        self.assertUnlocated("ALIGNMENT_UNVERIFIED", self.run_adapter(alignment=unverified))
        # Unrectified: refused only when the CameraInfo model cannot undistort the box points (2026-10-13).
        fisheye = camera_info(model="equidistant", d=(0.0,) * 4)
        self.assertUnlocated("ALIGNMENT_UNVERIFIED", self.run_adapter(rectified=False, info=fisheye))
        self.assertUnlocated("CALIBRATION_MISMATCH", self.run_adapter(info=camera_info(frame_id="other")))
        small = depth_image([[2000] * 2] * 2, width=2, height=2)
        self.assertUnlocated("DIMENSION_MISMATCH", self.run_adapter(depth=small))
        holes = depth_image([[0] * 4] * 4)
        self.assertUnlocated("DEPTH_INSUFFICIENT", self.run_adapter(depth=holes))
        edge_bg = depth_image([[1000, 1000, 4000, 4000]] * 4)
        self.assertUnlocated("DEPTH_DISCONTINUITY", self.run_adapter(depth=edge_bg))

    # CameraInfo ----------------------------------------------------------------
    def test_rectified_stream_uses_projection_not_raw_k(self):
        info = camera_info(k=(9.0, 0, 9.0, 0, 9.0, 9.0, 0, 0, 1))
        intrinsics = camera_intrinsics(info, self.binding, rectified=True)
        self.assertEqual((intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy), (2.0, 2.0, 1.5, 1.5))
        fisheye = camera_info(k=(9.0, 0, 9.0, 0, 9.0, 9.0, 0, 0, 1), model="equidistant", d=(0.0,) * 4)
        raw = camera_intrinsics(fisheye, self.binding, rectified=False)
        self.assertEqual((raw.fx, raw.cx, raw.rectified, raw.undistort), (9.0, 9.0, False, None))

    def test_secondary_stereo_binning_and_roi_are_rejected(self):
        right = camera_info(p=(2.0, 0, 1.5, -0.1, 0, 2.0, 1.5, 0, 0, 0, 1, 0))
        self.assertCode("INVALID_INPUT", lambda: camera_intrinsics(right, self.binding, rectified=True))
        self.assertCode("INVALID_INPUT", lambda: camera_intrinsics(camera_info(binning=2), self.binding,
                                                                   rectified=True))
        roi = NS(x_offset=1, y_offset=0, width=2, height=2)
        self.assertCode("INVALID_INPUT", lambda: camera_intrinsics(camera_info(roi=roi), self.binding,
                                                                   rectified=True))

    # Detection mapping -----------------------------------------------------------
    def test_vision_msgs_detection_requires_explicit_tracking_mode(self):
        det = detection_from_vision_msgs(detection2d(centre=(3.0, 1.0), size=(2.0, 2.0)), self.model,
                                         tracking_mode="DETECTION_ONLY")
        self.assertEqual((det.class_id, det.bbox, det.track_id), (0, (2.0, 0.0, 4.0, 2.0), None))
        tracked = detection_from_vision_msgs(detection2d(track="0"), self.model, tracking_mode="TRACKING")
        self.assertEqual(tracked.track_id, "0")
        self.assertCode("INVALID_INPUT", lambda: detection_from_vision_msgs(
            detection2d(track="7"), self.model, tracking_mode="TRACKING_FALLBACK"))
        self.assertCode("INVALID_INPUT", lambda: detection_from_vision_msgs(
            detection2d(), self.model, tracking_mode="TRACKING"))
        self.assertCode("INVALID_INPUT", lambda: detection_from_vision_msgs(
            detection2d(), self.model, tracking_mode=None))
        two = [NS(hypothesis=NS(class_id="person", score=0.9))] * 2
        self.assertCode("INVALID_INPUT", lambda: detection_from_vision_msgs(
            detection2d(results=two), self.model, tracking_mode="DETECTION_ONLY"))
        self.assertCode("MODEL_CLASS_MISMATCH", lambda: detection_from_vision_msgs(
            detection2d(label="doll"), self.model, tracking_mode="DETECTION_ONLY"))

    # Source identity -------------------------------------------------------------
    def test_non_robot_source_stops_at_localization(self):
        fixed = replace(self.binding, source_robot=None, source_camera="fixed_b")
        result = self.run_adapter(binding=fixed)
        self.assertEqual(result.localization.reason, "LOCATED")
        self.assertIsNone(result.observation)
        self.assertEqual(result.observation_blocked, "SOURCE_IDENTITY_UNDEFINED")

    def test_binding_and_policy_use_shared_contract_validators(self):
        self.assertCode("INVALID_INPUT", lambda: replace(self.binding, source_robot="1robot"))
        self.assertCode("INVALID_INPUT", lambda: replace(self.binding, source_boot_id=uid(0)))
        self.assertCode("INVALID_INPUT", lambda: replace(self.binding, source_boot_id=uid(0xabc).upper()))
        self.assertCode("INVALID_INPUT", lambda: replace(self.binding, clock_error_bound_ns=1.0))
        self.assertCode("INVALID_INPUT", lambda: ObservationPolicy(0))
        self.assertCode("INVALID_INPUT", lambda: ObservationPolicy(2.0))

    # Evidence --------------------------------------------------------------------
    def test_evidence_hash_is_the_exact_rgb_payload(self):
        result = self.run_adapter(rgb=rgb_image(pixel=7))
        self.assertEqual(result.observation.evidence_sha256, hashlib.sha256(bytes([7]) * 48).hexdigest())
        other = self.run_adapter(rgb=rgb_image(pixel=8))
        self.assertNotEqual(result.observation.payload_hash, other.observation.payload_hash)

    # Late, reordered and duplicate input (LL-06) ---------------------------------
    def test_duplicate_and_reordered_input_do_not_refresh_stream_or_validity(self):
        registry = ObservationRegistry()
        first = self.run_adapter(1).observation
        self.assertEqual(self.ingest(registry, first, steady=10.0).status, "ACCEPTED")
        key = ("robot_a", "front", uid(100))
        self.assertEqual(registry.last_new_receive(*key), 10.0)

        replay = self.run_adapter(1).observation  # same recorded tuple decoded again
        self.assertEqual(replay.payload_hash, first.payload_hash)
        self.assertEqual(self.ingest(registry, replay, steady=11.0).status, "DUPLICATE")
        self.assertEqual(registry.last_new_receive(*key), 10.0)

        older = self.run_adapter(2, rgb=rgb_image(nanosec=EPOCH_NSEC - 1), depth=depth_image(
            nanosec=EPOCH_NSEC - 1), det_header=header(nanosec=EPOCH_NSEC - 1)).observation
        self.assertEqual(self.ingest(registry, older, steady=12.0).status, "REORDERED")
        self.assertEqual(registry.last_new_receive(*key), 10.0)

        # Validity comes from the sensor stamp: decoding or receiving later changes nothing.
        self.assertEqual(replay.valid_until_ns, first.valid_until_ns)
        self.assertCode("STALE_OBSERVATION", lambda: self.ingest(
            ObservationRegistry(), first, steady=13.0, now_ns=1700000002123456789))


if __name__ == "__main__":
    unittest.main()
