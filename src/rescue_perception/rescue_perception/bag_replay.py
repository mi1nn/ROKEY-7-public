"""Pure pairing/config logic for replaying rosbag2 recordings through the adapter (E2).

No ROS import: callers deserialize messages (``tools/perception_bag_replay.py`` does
it with rclpy) and pass ``(role, msg)`` records in bag order.
"""
import json
from pathlib import Path
from uuid import UUID, uuid5

from rescue_contracts import RescueError, duration_ns

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
    topics = tuple(raw["topics"].values())
    if any(not isinstance(topic, str) or not topic.strip() for topic in topics) or len(set(topics)) != len(ROLES):
        raise ValueError("each replay role requires a nonempty distinct topic")
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


def _checked_stamp(msg, role):
    try:
        return stamp_ns(msg.header, role), None
    except (RescueError, AttributeError) as exc:
        return None, {"result": "REJECTED", "reason": "UNUSABLE_TIMESTAMP", "role": role,
                      "detail": str(exc)}


def pair_frames(records, max_skew_ns):
    """Bag order -> (stamp, frames or None, diagnostic or None).

    Offline whole-recording matching, never a realtime synchronizer. RGB matches
    the exact detection-array stamp; Depth is nearest within the configured skew
    (ties use the first bag record). CameraInfo is the latest seen before the array.
    Bad header stamps produce diagnostics and are excluded, never replaced with
    bag receive time or the replay host clock. Valid records continue to be read.
    """
    duration_ns(max_skew_ns)
    rgb, depth, info, pending = {}, [], None, []
    for record_index, (role, msg) in enumerate(records):
        if role not in ROLES:
            raise ValueError(f"unknown replay role: {role!r}")
        stamp, error = _checked_stamp(msg, role)
        if error:
            yield None, None, {**error, "record_index": record_index}
            continue
        if role == "rgb":
            rgb[stamp] = msg
        elif role == "depth":
            depth.append((stamp, msg))
        elif role == "camera_info":
            info = msg
        elif role == "detections":
            pending.append((stamp, msg, info))
    for stamp, array, latest_info in pending:
        nearest = min(depth, key=lambda item: abs(item[0] - stamp), default=None)
        error = {"result": "UNPAIRED"}
        if stamp not in rgb:
            error["reason"] = "NO_RGB_WITH_DETECTION_STAMP"
        elif nearest is None or abs(nearest[0] - stamp) > max_skew_ns:
            error.update(reason="NO_DEPTH_WITHIN_SKEW",
                         nearest_depth_stamp_ns=nearest[0] if nearest else None,
                         depth_skew_ns=abs(nearest[0] - stamp) if nearest else None,
                         max_stamp_skew_ns=max_skew_ns)
        elif latest_info is None:
            error["reason"] = "NO_CAMERA_INFO"
        else:
            yield stamp, (rgb[stamp], nearest[1], latest_info, array), None
            continue
        yield stamp, None, error


def replay_records(records, cfg):
    namespace = UUID(cfg["observation_namespace"])
    for stamp, frames, unmatched in pair_frames(records, cfg["depth_policy"].max_stamp_skew_ns):
        if frames is None:
            yield {"stamp_ns": stamp, **unmatched}
            continue
        rgb, depth, info, array = frames
        if not array.detections:
            yield {"stamp_ns": stamp, "result": "EMPTY_DETECTIONS", "reason": "NO_DETECTIONS"}
        for index, det in enumerate(array.detections):
            row = {"stamp_ns": stamp, "index": index}
            _, error = _checked_stamp(det, "detection")
            if error:
                yield {**row, **error}
                continue
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
