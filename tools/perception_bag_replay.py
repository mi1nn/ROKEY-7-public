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

from rescue_contracts import RescueError  # noqa: E402
from rescue_perception import (AlignmentEvidence, DepthPolicy, ModelManifest, ObservationPolicy,  # noqa: E402
                               SourceBinding, detection_from_vision_msgs, observe, stamp_ns)

ROLES = ("rgb", "depth", "camera_info", "detections")


def load_config(path):
    """Strict JSON config -> adapter objects. Unknown or missing keys are errors."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    expected = {"topics", "binding", "alignment", "rectified", "model", "tracking_mode", "depth_policy",
                "observation_policy", "observation_namespace"}
    if set(raw) != expected or set(raw["topics"]) != set(ROLES):
        raise ValueError(f"config keys must be exactly {sorted(expected)} and topics {list(ROLES)}")
    model = raw["model"]
    return {
        "topics": raw["topics"],
        "binding": SourceBinding(**raw["binding"]),
        "alignment": AlignmentEvidence(**raw["alignment"]),
        "rectified": raw["rectified"],
        "model": ModelManifest(model["model_id"], model["sha256"], tuple(model["class_labels"])),
        "tracking_mode": raw["tracking_mode"],
        "depth_policy": DepthPolicy(**raw["depth_policy"]),
        "observation_policy": ObservationPolicy(**raw["observation_policy"]),
        "observation_namespace": raw["observation_namespace"],
    }


def pair_frames(records, max_skew_ns):
    """records: (role, msg) in bag order -> (rgb, depth, camera_info, detection_array) tuples.

    RGB is matched to a detection array by the exact header stamp (the detector
    ran on that frame). Depth is the nearest stamp within ``max_skew_ns``; the
    core still judges the pair. CameraInfo is the latest seen before the array.
    Unmatched arrays are reported with a reason instead of being dropped.
    """
    rgb, depth, info, pending = {}, [], None, []
    for role, msg in records:
        if role == "rgb":
            rgb[stamp_ns(msg.header, "rgb")] = msg
        elif role == "depth":
            depth.append((stamp_ns(msg.header, "depth"), msg))
        elif role == "camera_info":
            info = msg
        elif role == "detections":
            pending.append((stamp_ns(msg.header, "detections"), msg, info))
    for stamp, array, latest_info in pending:
        nearest = min(depth, key=lambda item: abs(item[0] - stamp), default=None)
        if stamp not in rgb:
            yield stamp, None, "NO_RGB_WITH_DETECTION_STAMP"
        elif nearest is None or abs(nearest[0] - stamp) > max_skew_ns:
            yield stamp, None, "NO_DEPTH_WITHIN_SKEW"
        elif latest_info is None:
            yield stamp, None, "NO_CAMERA_INFO"
        else:
            yield stamp, (rgb[stamp], nearest[1], latest_info, array), None


def replay_records(records, cfg):
    from uuid import UUID, uuid5
    namespace = UUID(cfg["observation_namespace"])
    for stamp, frames, unmatched in pair_frames(records, cfg["depth_policy"].max_stamp_skew_ns):
        if frames is None:
            yield {"stamp_ns": stamp, "result": "UNPAIRED", "reason": unmatched}
            continue
        rgb, depth, info, array = frames
        for index, det in enumerate(array.detections):
            row = {"stamp_ns": stamp, "index": index}
            try:
                detection = detection_from_vision_msgs(det, cfg["model"], tracking_mode=cfg["tracking_mode"])
                result = observe(rgb, depth, info, detection, det.header, binding=cfg["binding"],
                                 alignment=cfg["alignment"], rectified=cfg["rectified"], model=cfg["model"],
                                 depth_policy=cfg["depth_policy"], observation_policy=cfg["observation_policy"],
                                 observation_id=str(uuid5(namespace, f"{stamp}/{index}")),
                                 evidence_id=f"bag:{stamp}")
            except RescueError as exc:
                row.update(result="REJECTED", reason=exc.detail_code)
            else:
                loc = result.localization
                row.update(result="OBSERVATION" if result.observation else "LOCALIZATION_ONLY",
                           reason=loc.reason, point_optical=loc.point_optical,
                           observed_at_ns=result.observation.observed_at_ns if result.observation else None)
            yield row


def _ros():
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message, serialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - depends on the host
        raise SystemExit(f"ROS 2 Python packages are required: {exc}") from exc
    return rosbag2_py, serialize_message, deserialize_message, get_message


def read_bag(path, topics):
    rosbag2_py, _, deserialize_message, get_message = _ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    role_of = {topic: role for role, topic in topics.items()}
    missing = [topic for topic in role_of if topic not in types]
    if missing:
        raise SystemExit(f"topics missing from bag: {missing}")
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
    cfg = load_config(args.config)
    rows = list(replay_records(read_bag(args.bag, cfg["topics"]), cfg))
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
