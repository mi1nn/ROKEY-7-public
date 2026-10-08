"""Validate an inert deployment profile without importing ROS or opening a socket.

This format describes host ownership and configuration identity only. It is not
the rescue message contract and does not establish operational safety limits.

ROBOT_COUNT preserves the two-AMR requirement. HOST_TIERS and
REQUIRE_ALL_HOSTS_USED encode the current proposed deployment baseline while
CLA-01 (four required hosts versus four available hosts) remains unresolved.
These policy values do not confirm a hardware allocation or a runtime topology.
"""

from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any
from uuid import UUID


ROBOT_COUNT = 2
HOST_TIERS = {"high": 2, "mid": 2}
REQUIRE_ALL_HOSTS_USED = True

# Schema v2 (2026-10-08, BRD v0.1 / work bundle 1): mission inputs. Roles are
# profile data, never code constants; exactly one SCOUT and one SUPPORT robot.
SUPPORTED_SCHEMA_VERSIONS = (1, 2)
ROBOT_ROLES = ("SCOUT", "SUPPORT")
AREA_BASIS_KINDS = ("PRIOR_MAP", "MAP_REVISION_BOUND")
MAX_SEARCH_AREAS = 64

MAX_CONFIG_BYTES = 65_536
IDENTIFIER = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}\Z")
NAMESPACE = re.compile(r"/[A-Za-z][A-Za-z0-9_]{0,63}\Z")
ROS_NAME = re.compile(r"/[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\Z")
# TF frame IDs (C-04): no leading slash; robot-scoped frames like robot_b/map are valid.
FRAME_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\Z")


class ProfileError(ValueError):
    """A configuration cannot be accepted as this foundation's profile."""


@dataclass(frozen=True)
class Host:
    id: str
    tier: str


@dataclass(frozen=True)
class RobotParameters:
    robot_id: str
    namespace: str
    use_sim_time: bool
    # v2 only (None under v1): ROS names the command adapter must use. They are
    # required to live under the robot namespace so two robots cannot share a path.
    nav_action: str | None = None
    odom_topic: str | None = None
    dock_status_topic: str | None = None


@dataclass(frozen=True)
class AreaBasis:
    kind: str
    map_session_id: str
    map_revision: int
    transform_revision: int


@dataclass(frozen=True)
class SearchArea:
    id: str
    mission_xy: tuple[float, float]
    basis: AreaBasis


@dataclass(frozen=True)
class HoldPose:
    frame_id: str
    x: float
    y: float
    yaw: float
    map_session_id: str
    map_revision: int


@dataclass(frozen=True)
class Mission:
    roles: tuple[tuple[str, str], ...]          # (robot_id, role) sorted by robot_id
    search_areas: tuple[SearchArea, ...]
    hold_positions: tuple[tuple[str, HoldPose], ...]

    def role_of(self, robot_id: str) -> str:
        return dict(self.roles)[robot_id]

    def hold_of(self, robot_id: str) -> HoldPose | None:
        return dict(self.hold_positions).get(robot_id)


@dataclass(frozen=True)
class Robot:
    id: str
    namespace: str
    command_host: str
    slam_host: str
    perception_host: str
    parameters_file: str
    parameters: RobotParameters


@dataclass(frozen=True)
class Deployment:
    id: str
    hosts: tuple[Host, ...]
    robots: tuple[Robot, ...]
    coordination_host: str
    operations_host: str
    schema_version: int = 1
    mission: Mission | None = None


def _object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ProfileError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _nonfinite(value: str) -> None:
    raise ProfileError(f"non-finite JSON number is forbidden: {value}")


def _read_json(path: Path) -> Any:
    try:
        if not path.is_file():
            raise ProfileError(f"configuration must be a regular file: {path}")
        with path.open("rb") as stream:
            raw = stream.read(MAX_CONFIG_BYTES + 1)
        if len(raw) > MAX_CONFIG_BYTES:
            raise ProfileError(f"configuration exceeds {MAX_CONFIG_BYTES} bytes: {path}")
        return json.loads(raw.decode("utf-8"), object_pairs_hook=_object,
                          parse_constant=_nonfinite)
    except ProfileError:
        raise
    except (OSError, ValueError, RecursionError) as exc:
        raise ProfileError(f"cannot read JSON configuration {path}: {exc}") from exc


def _fields(value: Any, expected: set[str], label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProfileError(f"{label} must be an object")
    actual = set(value)
    if actual != expected:
        raise ProfileError(f"{label}: missing={sorted(expected - actual)}, "
                           f"unknown={sorted(actual - expected)}")
    return value


def _version(value: Any, label: str) -> int:
    if type(value) is not int or value not in SUPPORTED_SCHEMA_VERSIONS:
        raise ProfileError(f"{label}.schema_version must be one of {list(SUPPORTED_SCHEMA_VERSIONS)}")
    return value


def _identifier(value: Any, label: str, namespace: bool = False) -> str:
    pattern = NAMESPACE if namespace else IDENTIFIER
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise ProfileError(f"invalid {label}: expected {'/namespace' if namespace else 'identifier'}")
    return value


def _list(value: Any, length: int, label: str) -> list[Any]:
    if not isinstance(value, list) or len(value) != length:
        raise ProfileError(f"{label} must contain exactly {length} entries")
    return value


def _unique(values: list[str], label: str) -> None:
    if len(set(values)) != len(values):
        raise ProfileError(f"duplicate {label}")


def _parameter_path(root: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise ProfileError("parameters_file must be a relative JSON path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts or relative.suffix != ".json":
        raise ProfileError("parameters_file must be a relative .json path without '..'")
    try:
        resolved = (root / relative).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise ProfileError(f"parameters_file cannot be resolved: {value!r}") from exc
    if not resolved.is_relative_to(root) or not resolved.is_file():
        raise ProfileError("parameters_file must be a file inside the profile directory")
    return resolved


def _finite(value: Any, label: str) -> float:
    # A JSON integer can exceed float range; isfinite()/float() then raise OverflowError.
    try:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ProfileError(f"{label} must be a finite number")
        return float(value)
    except OverflowError as exc:
        raise ProfileError(f"{label} must be a finite number") from exc


def _frame_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or not FRAME_ID.fullmatch(value):
        raise ProfileError(f"invalid {label}: expected a TF frame id such as mission or robot_b/map")
    return value


def _uint64(value: Any, label: str) -> int:
    if type(value) is not int or not 0 <= value < 2**64:
        raise ProfileError(f"{label} must be an unsigned 64-bit integer")
    return value


def _uuid(value: Any, label: str) -> str:
    try:
        parsed = UUID(value) if isinstance(value, str) else None
    except ValueError:
        parsed = None
    if parsed is None or str(parsed) != value or parsed.int == 0:
        raise ProfileError(f"{label} must be a canonical nonzero UUID")
    return value


def _ros_name(value: Any, label: str, namespace: str) -> str:
    if not isinstance(value, str) or not ROS_NAME.fullmatch(value) or not value.startswith(namespace + "/"):
        raise ProfileError(f"{label} must be a ROS name under {namespace}/")
    return value


def _parameters(path: Path, robot_id: str, namespace: str, schema_version: int) -> RobotParameters:
    base = {"schema_version", "robot_id", "namespace", "runtime_enabled", "use_sim_time"}
    names = {"nav_action", "odom_topic", "dock_status_topic"}
    value = _fields(_read_json(path), base | (names if schema_version == 2 else set()), "robot parameters")
    if _version(value["schema_version"], "robot parameters") != schema_version:
        raise ProfileError("robot parameters schema_version must match the deployment schema_version")
    if value["robot_id"] != robot_id or value["namespace"] != namespace:
        raise ProfileError(f"robot parameters do not belong to {robot_id} ({namespace})")
    if value["runtime_enabled"] is not False:
        raise ProfileError("runtime_enabled must be false: ROS runtime is not implemented")
    if type(value["use_sim_time"]) is not bool:
        raise ProfileError("use_sim_time must be boolean")
    if schema_version == 1:
        return RobotParameters(robot_id, namespace, value["use_sim_time"])
    resolved = {name: _ros_name(value[name], name, namespace) for name in sorted(names)}
    if len(set(resolved.values())) != len(resolved):
        raise ProfileError("nav_action, odom_topic and dock_status_topic must be distinct")
    return RobotParameters(robot_id, namespace, value["use_sim_time"],
                           resolved["nav_action"], resolved["odom_topic"], resolved["dock_status_topic"])


def _hold_pose(value: Any, label: str) -> HoldPose:
    pose = _fields(value, {"frame_id", "x", "y", "yaw", "map_session_id", "map_revision"}, label)
    return HoldPose(_frame_id(pose["frame_id"], label + ".frame_id"),
                    _finite(pose["x"], label + ".x"), _finite(pose["y"], label + ".y"),
                    _finite(pose["yaw"], label + ".yaw"),
                    _uuid(pose["map_session_id"], label + ".map_session_id"),
                    _uint64(pose["map_revision"], label + ".map_revision"))


def _mission(value: Any, robot_ids: list[str]) -> Mission:
    data = _fields(value, {"roles", "search_areas", "hold_positions"}, "mission")
    roles = data["roles"]
    if not isinstance(roles, dict) or set(roles) != set(robot_ids):
        raise ProfileError("mission.roles must name exactly the deployment's robots")
    for robot_id, role in roles.items():
        if not isinstance(role, str) or role not in ROBOT_ROLES:
            raise ProfileError(f"mission.roles[{robot_id}] must be one of {list(ROBOT_ROLES)}")
    if Counter(roles.values()) != Counter(ROBOT_ROLES):
        raise ProfileError("mission.roles must assign exactly one SCOUT and one SUPPORT")
    areas_raw = data["search_areas"]
    if not isinstance(areas_raw, list) or not 1 <= len(areas_raw) <= MAX_SEARCH_AREAS:
        raise ProfileError(f"mission.search_areas must contain 1..{MAX_SEARCH_AREAS} areas")
    areas: list[SearchArea] = []
    for index, entry in enumerate(areas_raw):
        label = f"search_areas[{index}]"
        area = _fields(entry, {"id", "mission_xy", "basis"}, label)
        xy = area["mission_xy"]
        if not isinstance(xy, list) or len(xy) != 2:
            raise ProfileError(f"{label}.mission_xy must be [x, y]")
        basis = _fields(area["basis"], {"kind", "map_session_id", "map_revision", "transform_revision"},
                        label + ".basis")
        if not isinstance(basis["kind"], str) or basis["kind"] not in AREA_BASIS_KINDS:
            raise ProfileError(f"{label}.basis.kind must be one of {list(AREA_BASIS_KINDS)}")
        areas.append(SearchArea(
            _uuid(area["id"], label + ".id"),
            (_finite(xy[0], label + ".mission_xy[0]"), _finite(xy[1], label + ".mission_xy[1]")),
            AreaBasis(basis["kind"], _uuid(basis["map_session_id"], label + ".basis.map_session_id"),
                      _uint64(basis["map_revision"], label + ".basis.map_revision"),
                      _uint64(basis["transform_revision"], label + ".basis.transform_revision"))))
    _unique([area.id for area in areas], "search_areas id")
    holds_raw = data["hold_positions"]
    if not isinstance(holds_raw, dict) or not set(holds_raw) <= set(robot_ids):
        raise ProfileError("mission.hold_positions keys must be deployment robots")
    holds = {robot_id: _hold_pose(pose, f"hold_positions[{robot_id}]") for robot_id, pose in holds_raw.items()}
    support = [robot_id for robot_id, role in roles.items() if role == "SUPPORT"]
    if any(robot_id not in holds for robot_id in support):
        raise ProfileError("mission.hold_positions must include the SUPPORT robot")
    return Mission(tuple(sorted(roles.items())), tuple(areas), tuple(sorted(holds.items())))


def load_deployment(path: str | Path) -> Deployment:
    """Read a bounded profile against the module's current proposed baseline.

    All parameters are data. No launch, import-by-name, shell command, network
    connection, or environment substitution is accepted by this schema.
    A profile symlink is resolved first: relative parameter files are confined
    to the resolved profile's directory, not the symlink's directory.
    """
    try:
        profile_path = Path(path).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise ProfileError(f"profile cannot be resolved: {path!r}") from exc
    raw = _read_json(profile_path)
    schema_version = _version(raw["schema_version"] if isinstance(raw, dict) and "schema_version" in raw else None,
                              "deployment")
    data = _fields(raw, {
        "schema_version", "deployment_id", "hosts", "robots", "services"
    } | ({"mission"} if schema_version == 2 else set()), "deployment")
    deployment_id = _identifier(data["deployment_id"], "deployment_id")
    hosts: list[Host] = []
    for entry in _list(data["hosts"], sum(HOST_TIERS.values()), "hosts"):
        host = _fields(entry, {"id", "tier"}, "host")
        host_id = _identifier(host["id"], "host.id")
        if not isinstance(host["tier"], str) or host["tier"] not in HOST_TIERS:
            raise ProfileError(f"host.tier must be one of {sorted(HOST_TIERS)}")
        hosts.append(Host(host_id, host["tier"]))
    _unique([host.id for host in hosts], "host id")
    if Counter(host.tier for host in hosts) != HOST_TIERS:
        raise ProfileError(f"hosts must have tier counts {HOST_TIERS}")
    known_hosts = {host.id for host in hosts}
    used_hosts: set[str] = set()

    def host_reference(value: Any, label: str) -> str:
        host_id = _identifier(value, label)
        if host_id not in known_hosts:
            raise ProfileError(f"{label} references unknown host: {host_id}")
        used_hosts.add(host_id)
        return host_id

    services = _fields(data["services"], {"coordination_host", "operations_host"}, "services")
    coordination_host = host_reference(services["coordination_host"], "coordination_host")
    operations_host = host_reference(services["operations_host"], "operations_host")
    robots: list[Robot] = []
    for entry in _list(data["robots"], ROBOT_COUNT, "robots"):
        robot = _fields(entry, {
            "id", "namespace", "command_host", "slam_host", "perception_host", "parameters_file"
        }, "robot")
        robot_id = _identifier(robot["id"], "robot.id")
        namespace = _identifier(robot["namespace"], "robot.namespace", namespace=True)
        command_host = host_reference(robot["command_host"], "command_host")
        slam_host = host_reference(robot["slam_host"], "slam_host")
        perception_host = host_reference(robot["perception_host"], "perception_host")
        parameters_path = _parameter_path(profile_path.parent, robot["parameters_file"])
        parameters = _parameters(parameters_path, robot_id, namespace, schema_version)
        robots.append(Robot(robot_id, namespace, command_host, slam_host,
                            perception_host, robot["parameters_file"], parameters))
    _unique([robot.id for robot in robots], "robot id")
    _unique([robot.namespace for robot in robots], "robot namespace")
    if REQUIRE_ALL_HOSTS_USED and known_hosts != used_hosts:
        raise ProfileError(f"hosts without an assigned role: {sorted(known_hosts - used_hosts)}")
    if len({robot.parameters.use_sim_time for robot in robots}) != 1:
        raise ProfileError("all robots must use the same use_sim_time setting")
    mission = _mission(data["mission"], [robot.id for robot in robots]) if schema_version == 2 else None
    return Deployment(deployment_id, tuple(hosts), tuple(robots), coordination_host, operations_host,
                      schema_version, mission)
