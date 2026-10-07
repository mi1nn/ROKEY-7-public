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
from pathlib import Path
import re
from typing import Any


ROBOT_COUNT = 2
HOST_TIERS = {"high": 2, "mid": 2}
REQUIRE_ALL_HOSTS_USED = True

MAX_CONFIG_BYTES = 65_536
IDENTIFIER = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}\Z")
NAMESPACE = re.compile(r"/[A-Za-z][A-Za-z0-9_]{0,63}\Z")


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


def _version(value: Any, label: str) -> None:
    if type(value) is not int or value != 1:
        raise ProfileError(f"{label}.schema_version must be integer 1")


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


def _parameters(path: Path, robot_id: str, namespace: str) -> RobotParameters:
    value = _fields(_read_json(path), {
        "schema_version", "robot_id", "namespace", "runtime_enabled", "use_sim_time"
    }, "robot parameters")
    _version(value["schema_version"], "robot parameters")
    if value["robot_id"] != robot_id or value["namespace"] != namespace:
        raise ProfileError(f"robot parameters do not belong to {robot_id} ({namespace})")
    if value["runtime_enabled"] is not False:
        raise ProfileError("runtime_enabled must be false: ROS runtime is not implemented")
    if type(value["use_sim_time"]) is not bool:
        raise ProfileError("use_sim_time must be boolean")
    return RobotParameters(robot_id, namespace, value["use_sim_time"])


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
    data = _fields(_read_json(profile_path), {
        "schema_version", "deployment_id", "hosts", "robots", "services"
    }, "deployment")
    _version(data["schema_version"], "deployment")
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
        parameters = _parameters(parameters_path, robot_id, namespace)
        robots.append(Robot(robot_id, namespace, command_host, slam_host,
                            perception_host, robot["parameters_file"], parameters))
    _unique([robot.id for robot in robots], "robot id")
    _unique([robot.namespace for robot in robots], "robot namespace")
    if REQUIRE_ALL_HOSTS_USED and known_hosts != used_hosts:
        raise ProfileError(f"hosts without an assigned role: {sorted(known_hosts - used_hosts)}")
    if len({robot.parameters.use_sim_time for robot in robots}) != 1:
        raise ProfileError("all robots must use the same use_sim_time setting")
    return Deployment(deployment_id, tuple(hosts), tuple(robots), coordination_host, operations_host)
