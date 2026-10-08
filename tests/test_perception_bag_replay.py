"""E2 tests for tools/perception_bag_replay.py pairing and config logic without ROS.

Messages are SYNTHETIC plain objects with ROS field names. Reading a real rosbag2
file needs ROS 2 and is run locally (PR evidence), not here.
"""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("perception_bag_replay", ROOT / "tools" / "perception_bag_replay.py")
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

CONFIG = ROOT / "tools" / "perception_bag_replay.synthetic.json"
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
    def setUp(self):
        self.cfg = tool.load_config(CONFIG)

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
        raw = json.loads(CONFIG.read_text(encoding="utf-8"))
        for broken in ({**raw, "extra": 1}, {k: v for k, v in raw.items() if k != "depth_policy"},
                       {**raw, "topics": {**raw["topics"], "imu": "/imu"}}):
            with tempfile.NamedTemporaryFile("w", suffix=".json") as handle:
                json.dump(broken, handle)
                handle.flush()
                with self.assertRaises(ValueError):
                    tool.load_config(handle.name)


if __name__ == "__main__":
    unittest.main()
