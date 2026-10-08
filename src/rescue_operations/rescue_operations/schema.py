"""Fixed local report schema 1.0; no ROS or network dependencies."""

import hashlib
import json
import math
import re
import uuid
from collections.abc import Mapping
from rescue_contracts import RescueError, canonical_code, robot_id, robot_ids

MAX_JSON_BYTES = 65536
MAX_EVIDENCE_BYTES = 4 * 1024 * 1024
MAX_EVIDENCE_REFS = 8
UINT64_MAX = (1 << 64) - 1
EVIDENCE_POLICIES = {"OPTIONAL", "OBSERVATION", "CONFIRMATION"}
REPORT_KEYS = {
    "schema_version", "report_id", "target_id", "observation_id",
    "source_robot", "source_camera", "source_boot_id", "clock_domain_id",
    "robot_id", "task_id", "generation", "observation_result",
    "confirmation_result", "confirmation_observation_id", "approach_result",
    "position", "observed_at_ns", "confirmed_at_ns", "reported_at_ns", "reason",
    "evidence_refs", "model_id", "model_sha256", "deployment_version",
}
REF_KEYS = {"evidence_id", "sha256", "media_type", "observation_id", "acquired_at_ns"}
ENVELOPE_KEYS = {"schema_version", "publisher_id", "publisher_boot_id", "sequence", "published_at_ns"}
POSITION_KEYS = {"x", "y", "z", "frame_id", "map_session_id", "map_revision", "transform_revision", "authority_id"}


class OperationsError(RescueError):
    """A bounded input, identity, policy or storage failure with a stable code."""

    def __init__(self, code, message=""):
        super().__init__(canonical_code("rescue_operations", code), message,
                         component="rescue_operations", detail_code=code)


def report_robot_id(value):
    try:
        return robot_id(value)
    except RescueError as exc:
        raise OperationsError("INVALID_INPUT", str(exc)) from exc


def allowed_robots(values):
    try:
        return robot_ids(values)
    except RescueError as exc:
        raise OperationsError("INVALID_INPUT", str(exc)) from exc


def require(condition, message, code="INVALID_INPUT"):
    if not condition:
        raise OperationsError(code, message)


def text(value, label, maximum=256):
    require(isinstance(value, str) and 0 < len(value) <= maximum and "\x00" not in value,
            f"{label} must be a nonempty string of at most {maximum} characters")
    return value


def uint(value, label, positive=False):
    require(type(value) is int and (1 if positive else 0) <= value <= UINT64_MAX,
            f"{label} must be {'positive ' if positive else ''}uint64")
    return value


def uid(value, label):
    text(value, label, 36)
    try:
        valid = str(uuid.UUID(value)) == value
    except (ValueError, AttributeError):
        valid = False
    require(valid, f"{label} must be a canonical lowercase UUID")
    return value


def sha(value, label):
    require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value),
            f"{label} must be a lowercase SHA-256")
    return value


def canonical(value):
    try:
        encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                             ensure_ascii=False, allow_nan=False)
        require(len(encoded.encode("utf-8")) <= MAX_JSON_BYTES, "JSON exceeds 65536 bytes")
        return encoded
    except OperationsError:
        raise
    except (TypeError, ValueError, RecursionError, UnicodeError) as exc:
        raise OperationsError("INVALID_INPUT", "not bounded finite JSON") from exc


def exact(value, keys, label):
    require(isinstance(value, Mapping), f"{label} must be an object")
    require(set(value) == keys, f"{label} fields must match schema 1.0")
    return dict(value)


def version(value):
    require(value == "1.0", "only schema 1.0 is supported; minor extensions are not implicit",
            "INCOMPATIBLE_SCHEMA")


def evidence_ref(value):
    ref = exact(value, REF_KEYS, "evidence_ref")
    uid(ref["evidence_id"], "evidence_id")
    uid(ref["observation_id"], "evidence observation_id")
    sha(ref["sha256"], "evidence sha256")
    require(ref["media_type"] in ("image/png", "image/jpeg"), "unsupported evidence media_type")
    uint(ref["acquired_at_ns"], "acquired_at_ns", positive=True)
    canonical(ref)
    return ref


def envelope(value):
    item = exact(value, ENVELOPE_KEYS, "envelope")
    version(item["schema_version"])
    text(item["publisher_id"], "publisher_id")
    uid(item["publisher_boot_id"], "publisher_boot_id")
    uint(item["sequence"], "sequence")
    uint(item["published_at_ns"], "published_at_ns", positive=True)
    canonical(item)
    return item


def report(value):
    """Validate and detach input; all returned fields contribute to its identity hash."""
    # Round trip bounds nested input and removes aliases before field normalization.
    item = exact(json.loads(canonical(value)), REPORT_KEYS, "report")
    version(item["schema_version"])
    for key in ("report_id", "target_id", "observation_id", "source_boot_id", "task_id"):
        uid(item[key], key)
    for key in ("source_robot", "robot_id"):
        report_robot_id(item[key])
    for key in ("source_camera", "clock_domain_id", "model_id", "deployment_version"):
        text(item[key], key)
    sha(item["model_sha256"], "model_sha256")
    uint(item["generation"], "generation")
    for key in ("observed_at_ns", "reported_at_ns"):
        uint(item[key], key, positive=True)
    require(item["observed_at_ns"] <= item["reported_at_ns"], "report predates observation")
    require(item["observation_result"] == "DETECTED", "only DETECTED observation_result is supported")
    require(item["confirmation_result"] in ("CONFIRMED", "UNCONFIRMED"), "invalid confirmation_result")
    if item["confirmation_result"] == "CONFIRMED":
        uid(item["confirmation_observation_id"], "confirmation_observation_id")
        require(item["confirmation_observation_id"] != item["observation_id"], "confirmation requires a separate observation")
        uint(item["confirmed_at_ns"], "confirmed_at_ns", positive=True)
        require(item["observed_at_ns"] <= item["confirmed_at_ns"] <= item["reported_at_ns"],
                "confirmation timestamp is out of order")
    else:
        require(item["confirmation_observation_id"] is None and item["confirmed_at_ns"] is None,
                "UNCONFIRMED cannot carry confirmation evidence")
    require(item["approach_result"] in ("APPROACH_COMPLETE", "UNREACHABLE", "FAILED", "CANCELED", "NOT_ATTEMPTED"),
            "invalid approach_result; medical or physical rescue claims are not supported")
    text(item["reason"], "reason", 1024)
    if item["position"] is not None:
        pos = exact(item["position"], POSITION_KEYS, "position")
        for key in ("x", "y", "z"):
            val = pos[key]
            require(type(val) in (int, float), f"position.{key} must be numeric")
            try:
                val = float(val)
            except (ValueError, OverflowError) as exc:
                raise OperationsError("INVALID_INPUT", f"invalid position.{key}") from exc
            require(math.isfinite(val), f"position.{key} must be finite")
            pos[key] = 0.0 if val == 0 else val
        for key in ("frame_id", "authority_id"):
            text(pos[key], key)
        uid(pos["map_session_id"], "map_session_id")
        for key in ("map_revision", "transform_revision"):
            uint(pos[key], key)
        item["position"] = pos
    if item["approach_result"] == "APPROACH_COMPLETE":
        require(item["confirmation_result"] == "CONFIRMED" and item["position"] is not None,
                "APPROACH_COMPLETE requires confirmation and position")
    refs = item["evidence_refs"]
    require(isinstance(refs, list) and len(refs) <= MAX_EVIDENCE_REFS, "evidence_refs must contain at most 8 items")
    expected = {item["observation_id"]: item["observed_at_ns"]}
    if item["confirmation_observation_id"] is not None:
        expected[item["confirmation_observation_id"]] = item["confirmed_at_ns"]
    item["evidence_refs"] = [evidence_ref(ref) for ref in refs]
    require(len({ref["evidence_id"] for ref in refs}) == len(refs), "duplicate evidence_id")
    for ref in refs:
        require(expected.get(ref["observation_id"]) == ref["acquired_at_ns"],
                "evidence must reference its original or confirmation observation and exact acquisition stamp")
    canonical(item)
    return item


def identity(payload):
    return hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()
