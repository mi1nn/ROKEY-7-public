"""Pure pairing/config logic for replaying rosbag2 recordings through the adapter (E2).

No ROS import: callers deserialize messages (``tools/perception_bag_replay.py`` does
it with rclpy) and pass ``(role, msg)`` records in bag order.
"""
import json
from pathlib import Path
from uuid import UUID, uuid5

from rescue_contracts import RescueError

from .adapter import ObservationPolicy, SourceBinding, detection_from_vision_msgs, observe, stamp_ns
from .core import AlignmentEvidence, DepthPolicy, ModelManifest

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
