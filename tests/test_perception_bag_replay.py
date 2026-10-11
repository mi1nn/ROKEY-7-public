"""E2 tests for rescue_perception.bag_replay (pairing and config) without ROS.

Messages are SYNTHETIC plain objects with ROS field names. Reading a real rosbag2
file (tools/perception_bag_replay.py) needs ROS 2 and is run locally (PR
evidence), not here. The config mirrors tools/perception_bag_replay.synthetic.json.
"""
import json
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest

from rescue_perception import bag_replay as tool

RAW_CONFIG = {
    "topics": {
        "rgb": "/synthetic/rgb",
        "depth": "/synthetic/depth",
        "camera_info": "/synthetic/camera_info",
        "detections": "/synthetic/detections"
    },
    "binding": {
        "source_robot": "robot_a",
        "source_camera": "front",
        "source_boot_id": "00000000-0000-0000-0000-000000000064",
        "clock_domain_id": "synthetic-bag",
        "rgb_stamp_semantics": "CAPTURE",
        "depth_stamp_semantics": "CAPTURE",
        "calibration_id": "synthetic-cal",
        "clock_state": "SYNCED",
        "clock_error_bound_ns": 10000000
    },
    "alignment": {
        "evidence_id": "synthetic-alignment",
        "calibration_id": "synthetic-cal",
        "rgb_frame_id": "synthetic/rgb_optical",
        "depth_frame_id": "synthetic/rgb_optical",
        "verified": True
    },
    "rectified": True,
    "model": {
        "model_id": "synthetic-recorded-detections",
        "sha256": "0000000000000000000000000000000000000000000000000000000000000000",
        "class_labels": [
            "person"
        ]
    },
    "tracking_mode": "DETECTION_ONLY",
    "depth_policy": {
        "max_stamp_skew_ns": 20000000,
        "min_depth_m": 0.5,
        "max_depth_m": 5.0,
        "min_valid_pixels": 2,
        "min_valid_fraction": 0.5,
        "max_depth_spread_m": 0.5
    },
    "observation_policy": {
        "validity_ns": 2000000000
    },
    "observation_namespace": "6f1a7c1e-2a5b-4f0e-9a51-2d8f7f3c9b10"
}
SEC = 1700000000
FRAME = "synthetic/rgb_optical"


def header(nanosec):
    return NS(stamp=NS(sec=SEC, nanosec=nanosec), frame_id=FRAME)


def rgb(nanosec):
    return NS(header=header(nanosec), width=4, height=4, encoding="rgb8", is_bigendian=0, step=12, data=bytes(48))


def depth(nanosec, mm=2000):
    return NS(header=header(nanosec), width=4, height=4, encoding="16UC1", is_bigendian=0, step=8,
              data=b"".join(struct.pack("<4H", *[mm] * 4) for _ in range(4)))


INFO = NS(header=header(0), width=4, height=4, k=(2.0, 0, 1.5, 0, 2.0, 1.5, 0, 0, 1),
          p=(2.0, 0, 1.5, 0, 0, 2.0, 1.5, 0, 0, 0, 1, 0), binning_x=0, binning_y=0,
          roi=NS(x_offset=0, y_offset=0, width=0, height=0))


def detections(nanosec, label="person"):
    det = NS(header=header(nanosec), id="", results=[NS(hypothesis=NS(class_id=label, score=0.9))],
             bbox=NS(center=NS(position=NS(x=2.0, y=2.0)), size_x=4.0, size_y=4.0))
    return NS(header=header(nanosec), detections=[det])


class BagReplayTests(unittest.TestCase):
    def load(self, raw):
        with tempfile.NamedTemporaryFile("w", suffix=".json") as handle:
            json.dump(raw, handle)
            handle.flush()
            return tool.load_config(handle.name)

    def setUp(self):
        self.cfg = self.load(RAW_CONFIG)

    def run_records(self, records):
        return list(tool.replay_records(records, self.cfg))

    def test_detection_array_pairs_exact_rgb_and_nearest_depth(self):
        rows = self.run_records([("camera_info", INFO), ("rgb", rgb(123456789)), ("depth", depth(123457789)),
                                 ("depth", depth(173456789)), ("detections", detections(123456789))])
        self.assertEqual(rows, [{"stamp_ns": 1700000000123456789, "index": 0, "result": "OBSERVATION",
                                 "reason": "LOCATED", "point_optical": (0.0, 0.0, 2.0),
                                 "observed_at_ns": 1700000000123456789}])

    def test_unpaired_arrays_are_reported_not_dropped(self):
        rows = self.run_records([("camera_info", INFO), ("rgb", rgb(1000)), ("depth", depth(1000)),
                                 ("detections", detections(2000)),
                                 ("rgb", rgb(900000000)), ("detections", detections(900000000))])
        self.assertEqual([(r["result"], r["reason"]) for r in rows],
                         [("UNPAIRED", "NO_RGB_WITH_DETECTION_STAMP"), ("UNPAIRED", "NO_DEPTH_WITHIN_SKEW")])
        no_info = self.run_records([("rgb", rgb(1000)), ("depth", depth(1000)), ("detections", detections(1000))])
        self.assertEqual(no_info[0]["reason"], "NO_CAMERA_INFO")

    def test_localization_failure_and_rejection_are_rows(self):
        rows = self.run_records([("camera_info", INFO), ("rgb", rgb(1000)), ("depth", depth(1000, mm=0)),
                                 ("detections", detections(1000)), ("rgb", rgb(2000)), ("depth", depth(2000)),
                                 ("detections", detections(2000, label="dummy"))])
        self.assertEqual((rows[0]["result"], rows[0]["reason"], rows[0]["point_optical"]),
                         ("OBSERVATION", "DEPTH_INSUFFICIENT", None))
        self.assertEqual((rows[1]["result"], rows[1]["reason"]),
                         ("REJECTED", "rescue_perception.MODEL_CLASS_MISMATCH"))

    def test_config_is_strict(self):
        raw = RAW_CONFIG
        for broken in ({**raw, "extra": 1}, {k: v for k, v in raw.items() if k != "depth_policy"},
                       {**raw, "topics": {**raw["topics"], "imu": "/imu"}}):
            with self.assertRaises(ValueError):
                self.load(broken)


if __name__ == "__main__":
    unittest.main()
