#!/usr/bin/env python3
"""Replay a rosbag2 recording through the rescue_perception input adapter (E2).

    perception_bag_replay.py write-synthetic OUT_DIR        # needs ROS 2 (rosbag2_py)
    perception_bag_replay.py replay BAG_DIR --config CFG    # needs ROS 2 (rosbag2_py)

Reads RGB/Depth ``sensor_msgs/Image``, ``sensor_msgs/CameraInfo`` and
``vision_msgs/Detection2DArray`` from the bag, deserializes them with rclpy exactly
as a subscriber would, pairs each detection array with the RGB frame of the same
stamp and the nearest Depth frame, and prints one JSON line per detection. Every
binding/policy value comes from CFG (JSON); there are no defaults. No ROS graph,
robot or model is involved. Bags and their hashes stay outside the repository
(assets/manifest/ records real recordings).
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for package in ("rescue_contracts", "rescue_perception"):
    sys.path.insert(0, str(ROOT / "src" / package))

from rescue_perception.bag_replay import load_config, replay_records  # noqa: E402
from rescue_contracts import RescueError  # noqa: E402

def _ros():
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message, serialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - depends on the host
        raise RuntimeError(f"ROS 2 Python packages are required: {exc}") from exc
    return rosbag2_py, serialize_message, deserialize_message, get_message


def read_bag(path, topics):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"bag path does not exist: {path}")
    rosbag2_py, _, deserialize_message, get_message = _ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    role_of = {topic: role for role, topic in topics.items()}
    missing = [topic for topic in role_of if topic not in types]
    if missing:
        raise ValueError(f"topics missing from bag: {missing}")
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in role_of:
            yield role_of[topic], deserialize_message(data, get_message(types[topic]))


def write_synthetic(out_dir):
    """Small synthetic bag: 3 frames, 4x4 RGB/16UC1 depth. Frame 2 has depth holes."""
    import struct
    rosbag2_py, serialize_message, _, _ = _ros()
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out_dir)), rosbag2_py.ConverterOptions("", ""))
    topics = {"rgb": ("/synthetic/rgb", "sensor_msgs/msg/Image"),
              "depth": ("/synthetic/depth", "sensor_msgs/msg/Image"),
              "camera_info": ("/synthetic/camera_info", "sensor_msgs/msg/CameraInfo"),
              "detections": ("/synthetic/detections", "vision_msgs/msg/Detection2DArray")}
    for index, (name, kind) in enumerate(topics.values()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=index, name=name, type=kind, serialization_format="cdr"))

    def stamped(msg, sec, nanosec, frame="synthetic/rgb_optical"):
        msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id = sec, nanosec, frame
        return msg

    info = stamped(CameraInfo(), 1700000000, 0)
    info.width, info.height, info.distortion_model = 4, 4, "plumb_bob"
    info.k = [2.0, 0.0, 1.5, 0.0, 2.0, 1.5, 0.0, 0.0, 1.0]
    info.p = [2.0, 0.0, 1.5, 0.0, 0.0, 2.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    writer.write(topics["camera_info"][0], serialize_message(info), 1700000000000000000)
    for frame, nanosec in enumerate((123456789, 223456789, 323456789)):
        receive_ns = 1700000000000000000 + nanosec + 5000000
        rgb = stamped(Image(), 1700000000, nanosec)
        rgb.width, rgb.height, rgb.encoding, rgb.step, rgb.data = 4, 4, "rgb8", 12, bytes([frame]) * 48
        depth = stamped(Image(), 1700000000, nanosec + 1000)
        mm = 0 if frame == 1 else 2000
        depth.width, depth.height, depth.encoding, depth.step = 4, 4, "16UC1", 8
        depth.data = b"".join(struct.pack("<4H", *[mm] * 4) for _ in range(4))
        det = Detection2D()
        det.header = rgb.header
        det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.size_x, det.bbox.size_y = 2.0, 2.0, 4.0, 4.0
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id, hyp.hypothesis.score = "person", 0.9
        det.results = [hyp]
        array = Detection2DArray()
        array.header, array.detections = rgb.header, [det]
        for role, msg in (("rgb", rgb), ("depth", depth), ("detections", array)):
            writer.write(topics[role][0], serialize_message(msg), receive_ns)
    del writer
    return {role: name for role, (name, _) in topics.items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("write-synthetic").add_argument("out_dir")
    replay = sub.add_parser("replay")
    replay.add_argument("bag")
    replay.add_argument("--config", required=True)
    args = parser.parse_args(argv)
    if args.command == "write-synthetic":
        print(json.dumps({"written": args.out_dir, "topics": write_synthetic(args.out_dir), "synthetic": True}))
        return 0
    try:
        cfg = load_config(args.config)
        seen, failed = False, False
        for row in replay_records(read_bag(args.bag, cfg["topics"]), cfg):
            seen = True
            failed |= row["result"] not in ("OBSERVATION", "EMPTY_DETECTIONS") or row["reason"] not in (
                "LOCATED", "NO_DETECTIONS")
            print(json.dumps(row, ensure_ascii=False))
        if not seen:
            print(json.dumps({"result": "EMPTY_REPLAY", "reason": "NO_REPLAY_ROWS"}))
        return 0 if seen and not failed else 1
    except (OSError, ValueError, TypeError, KeyError, AttributeError, RuntimeError, RescueError) as exc:
        print(json.dumps({"result": "INPUT_ERROR", "reason": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
