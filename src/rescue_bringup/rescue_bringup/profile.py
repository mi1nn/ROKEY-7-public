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
# v2.1 additive blocks (2026-10-08): optional, validated strictly when present.
READINESS_SOURCES = ("odom", "scan", "camera", "wheel_status", "map_to_odom_tf")
SITE_CAMERA_ROLES = ("DISASTER_SIGN", "TARGET_WATCH", "EXIT_PASS")  # what a fixed camera adapter may produce;
# TARGET_WATCH -> SiteEvent TARGET_DISAPPEARED_CANDIDATE. One camera may hold several roles (2026-10-09: one fixed camera).
MAX_SITE_CAMERAS = 8
MAX_DISCOVERY_SERVERS = 8
DOMAIN_ID_MAX = 232
SERVER_ID_MAX = 255
_ABSENT = object()   # optional block not present (JSON null is NOT absent: it is rejected)

MAX_CONFIG_BYTES = 65_536
IDENTIFIER = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}\Z")
NAMESPACE = re.compile(r"/[A-Za-z][A-Za-z0-9_]{0,63}\Z")
ROS_NAME = re.compile(r"/[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\Z")
# TF frame IDs (C-04): no leading slash; robot-scoped frames like robot_b/map are valid.
FRAME_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*\Z")
CAMERA_ID = re.compile(r"cam_[A-Za-z0-9_]{1,60}\Z")  # = rescue_contracts.camera_id


class ProfileError(ValueError):
    """A configuration cannot be accepted as this foundation's profile."""


@dataclass(frozen=True)
class DiscoveryServer:
    server_id: int        # position in ROS_DISCOVERY_SERVER (0..255)
    address: str
    port: int


@dataclass(frozen=True)
class HostNetwork:
    """Expected ROS 2 discovery setup of one PC (recorded, compared at preflight; never applied)."""
    domain_id: int
    discovery_servers: tuple[DiscoveryServer, ...]
    super_client: bool    # must be False: runtime nodes are plain clients


@dataclass(frozen=True)
class Host:
    id: str
    tier: str
    network: HostNetwork | None = None


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
    readiness: "ReadinessParameters | None" = None


@dataclass(frozen=True)
class ReadinessParameters:
    """Inputs of the robot-side readiness monitor (rescue_robot.ros_node parameter names)."""
    version: str
    max_age_s: tuple[tuple[str, float], ...]   # (source, seconds) sorted by source
    scan_topic: str
    tf_topic: str
    map_frame: str
    odom_frame: str
    camera_topics: tuple[str, ...]
    wheel_status_topic: str | None
    watchdog_heartbeat_topic: str | None      # None until the robot-side watchdog exists

    def max_age(self, source: str) -> float:
        return dict(self.max_age_s)[source]


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
    entry_yaw: float | None = None   # arrival heading in the mission frame (v2.1)


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
class SiteCamera:
    id: str        # cam_<id> (rescue_contracts.camera_id)
    host: str
    roles: tuple[str, ...]   # non-empty, unique, each in SITE_CAMERA_ROLES
    source: str              # device/URL string for the adapter; not interpreted here


@dataclass(frozen=True)
class Deployment:
    id: str
    hosts: tuple[Host, ...]
    robots: tuple[Robot, ...]
    coordination_host: str
    operations_host: str
    schema_version: int = 1
    mission: Mission | None = None
    site_cameras: tuple[SiteCamera, ...] = ()


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


def _positive_int(value: Any, label: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ProfileError(f"{label} must be an integer in {low}..{high}")
    return value


def _nonempty(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ProfileError(f"{label} must be a non-empty string without surrounding whitespace")
    return value


def _readiness(value: Any, namespace: str, taken: set[str]) -> ReadinessParameters:
    label = "readiness"
    data = _fields(value, {"version", "max_age_s", "scan_topic", "tf_topic", "map_frame", "odom_frame",
                           "camera_topics", "wheel_status_topic", "watchdog_heartbeat_topic"}, label)
    version = _nonempty(data["version"], label + ".version")
    ages = data["max_age_s"]
    if not isinstance(ages, dict) or set(ages) != set(READINESS_SOURCES):
        raise ProfileError(f"{label}.max_age_s must have exactly the sources {list(READINESS_SOURCES)}")
    max_age = []
    for source in READINESS_SOURCES:
        seconds = _finite(ages[source], f"{label}.max_age_s.{source}")
        if seconds <= 0:
            raise ProfileError(f"{label}.max_age_s.{source} must be positive")
        max_age.append((source, seconds))
    scan = _ros_name(data["scan_topic"], label + ".scan_topic", namespace)
    tf = _ros_name(data["tf_topic"], label + ".tf_topic", namespace)
    cameras_raw = data["camera_topics"]
    if not isinstance(cameras_raw, list):
        raise ProfileError(f"{label}.camera_topics must be a list (may be empty)")
    cameras = tuple(_ros_name(t, f"{label}.camera_topics[{i}]", namespace) for i, t in enumerate(cameras_raw))
    wheel = data["wheel_status_topic"]
    wheel_topic = None if wheel == "" else _ros_name(wheel, label + ".wheel_status_topic", namespace)
    if data["watchdog_heartbeat_topic"] != "":
        raise ProfileError(f"{label}.watchdog_heartbeat_topic must be empty until the robot-side watchdog exists")
    names = [scan, tf, *cameras] + ([wheel_topic] if wheel_topic else [])
    if len(set(names)) != len(names) or set(names) & taken:
        raise ProfileError(f"{label} topics (scan_topic, tf_topic, camera_topics, wheel_status_topic) must be distinct")
    return ReadinessParameters(version, tuple(max_age), scan, tf,
                               _frame_id(data["map_frame"], label + ".map_frame"),
                               _frame_id(data["odom_frame"], label + ".odom_frame"),
                               cameras, wheel_topic, None)


def _network(value: Any, label: str) -> HostNetwork:
    data = _fields(value, {"domain_id", "discovery_servers", "super_client"}, label)
    domain = _positive_int(data["domain_id"], label + ".domain_id", 0, DOMAIN_ID_MAX)
    servers_raw = data["discovery_servers"]
    if not isinstance(servers_raw, list) or not 1 <= len(servers_raw) <= MAX_DISCOVERY_SERVERS:
        raise ProfileError(f"{label}.discovery_servers must contain 1..{MAX_DISCOVERY_SERVERS} servers")
    servers = []
    for index, entry in enumerate(servers_raw):
        item = _fields(entry, {"server_id", "address", "port"}, f"{label}.discovery_servers[{index}]")
        servers.append(DiscoveryServer(
            _positive_int(item["server_id"], f"{label}.discovery_servers[{index}].server_id", 0, SERVER_ID_MAX),
            _nonempty(item["address"], f"{label}.discovery_servers[{index}].address"),
            _positive_int(item["port"], f"{label}.discovery_servers[{index}].port", 1, 65535)))
    ids = [s.server_id for s in servers]
    if len(set(ids)) != len(ids):
        raise ProfileError(f"{label}.discovery_servers server_id must be unique (position = id)")
    if len({(s.address, s.port) for s in servers}) != len(servers):
        raise ProfileError(f"{label}.discovery_servers address:port must be unique")
    if data["super_client"] is not False:
        raise ProfileError(f"{label}.super_client must be false: runtime nodes are plain discovery clients")
    return HostNetwork(domain, tuple(sorted(servers, key=lambda s: s.server_id)), False)


def _parameters(path: Path, robot_id: str, namespace: str, schema_version: int) -> RobotParameters:
    base = {"schema_version", "robot_id", "namespace", "runtime_enabled", "use_sim_time"}
    names = {"nav_action", "odom_topic", "dock_status_topic"}
    raw = _read_json(path)
    readiness_raw = raw.pop("readiness", _ABSENT) if isinstance(raw, dict) and schema_version == 2 else _ABSENT
    value = _fields(raw, base | (names if schema_version == 2 else set()), "robot parameters")
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
    readiness = (_readiness(readiness_raw, namespace, set(resolved.values()))
                 if readiness_raw is not _ABSENT else None)
    return RobotParameters(robot_id, namespace, value["use_sim_time"],
                           resolved["nav_action"], resolved["odom_topic"], resolved["dock_status_topic"],
                           readiness)


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
        entry_yaw_raw = entry.pop("entry_yaw", _ABSENT) if isinstance(entry, dict) else _ABSENT
        area = _fields(entry, {"id", "mission_xy", "basis"}, label)
        entry_yaw = _finite(entry_yaw_raw, label + ".entry_yaw") if entry_yaw_raw is not _ABSENT else None
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
                      _uint64(basis["transform_revision"], label + ".basis.transform_revision")),
            entry_yaw))
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
    cameras_raw = raw.pop("site_cameras", _ABSENT) if isinstance(raw, dict) and schema_version == 2 else _ABSENT
    data = _fields(raw, {
        "schema_version", "deployment_id", "hosts", "robots", "services"
    } | ({"mission"} if schema_version == 2 else set()), "deployment")
    deployment_id = _identifier(data["deployment_id"], "deployment_id")
    hosts: list[Host] = []
    for entry in _list(data["hosts"], sum(HOST_TIERS.values()), "hosts"):
        network_raw = entry.pop("network", _ABSENT) if isinstance(entry, dict) and schema_version == 2 else _ABSENT
        host = _fields(entry, {"id", "tier"}, "host")
        host_id = _identifier(host["id"], "host.id")
        if not isinstance(host["tier"], str) or host["tier"] not in HOST_TIERS:
            raise ProfileError(f"host.tier must be one of {sorted(HOST_TIERS)}")
        network = _network(network_raw, f"hosts[{host_id}].network") if network_raw is not _ABSENT else None
        hosts.append(Host(host_id, host["tier"], network))
    _unique([host.id for host in hosts], "host id")
    seen_servers: dict[tuple[str, int], int] = {}
    for host in hosts:
        for server in (host.network.discovery_servers if host.network else ()):
            key = (server.address, server.port)
            if seen_servers.setdefault(key, server.server_id) != server.server_id:
                raise ProfileError(f"discovery server {server.address}:{server.port} has different server_id across hosts "
                                   f"(position in ROS_DISCOVERY_SERVER is the id; it must be the same everywhere)")
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
    cameras: list[SiteCamera] = []
    if cameras_raw is not _ABSENT:
        if not isinstance(cameras_raw, list) or len(cameras_raw) > MAX_SITE_CAMERAS:
            raise ProfileError(f"site_cameras must be a list of at most {MAX_SITE_CAMERAS}")
        for index, entry in enumerate(cameras_raw):
            label = f"site_cameras[{index}]"
            cam = _fields(entry, {"id", "host", "roles", "source"}, label)
            if not isinstance(cam["id"], str) or not CAMERA_ID.match(cam["id"]):
                raise ProfileError(f"{label}.id must match cam_<identifier>")
            roles_raw = cam["roles"]
            if (not isinstance(roles_raw, list) or not roles_raw
                    or any(not isinstance(r, str) or r not in SITE_CAMERA_ROLES for r in roles_raw)
                    or len(set(roles_raw)) != len(roles_raw)):
                raise ProfileError(f"{label}.roles must be a non-empty list of distinct values from {list(SITE_CAMERA_ROLES)}")
            host_id = _identifier(cam["host"], label + ".host")
            if host_id not in known_hosts:
                raise ProfileError(f"{label}.host references unknown host: {host_id}")
            cameras.append(SiteCamera(cam["id"], host_id, tuple(roles_raw), _nonempty(cam["source"], label + ".source")))
        _unique([cam.id for cam in cameras], "site_cameras id")
    return Deployment(deployment_id, tuple(hosts), tuple(robots), coordination_host, operations_host,
                      schema_version, mission, tuple(cameras))
