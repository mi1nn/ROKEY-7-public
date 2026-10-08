"""Immutable input, provenance, and result values for pure coordinate logic."""

from dataclasses import dataclass

from .geometry import (MappingError, RigidTransform, finite, integer, name, robot_identifier,
                       timestamp, uint64, uuid_string, vector)


@dataclass(frozen=True)
class SourceObservation:
    observation_id: str
    source_robot: str
    source_camera: str
    source_boot_id: str
    observed_at_ns: int
    source_frame: str
    point_xyz: tuple[float, float, float]
    valid_until_ns: int
    clock_domain_id: str
    map_session_id: str
    map_revision: int

    def __post_init__(self):
        robot_identifier(self.source_robot)
        for field in ("source_robot", "source_camera", "source_frame", "clock_domain_id"):
            name(getattr(self, field), field)
        for field in ("observation_id", "source_boot_id", "map_session_id"):
            uuid_string(getattr(self, field), field)
        timestamp(self.observed_at_ns, "observed_at_ns")
        timestamp(self.valid_until_ns, "valid_until_ns")
        integer(self.valid_until_ns, "valid_until_ns", self.observed_at_ns + 1)
        uint64(self.map_revision, "map_revision")
        object.__setattr__(self, "point_xyz", vector(self.point_xyz, 3, "point_xyz"))


@dataclass(frozen=True)
class TimedTransform:
    """A transform queried at observation time, not a 'latest TF' fallback.

    EXACT needs one sample at query_time_ns. INTERPOLATED_VALIDATED needs two
    bracketing sample times and a caller-provided validation evidence ID. This
    records the adapter's claim; this library cannot verify a real TF buffer.
    """

    source_robot: str
    source_frame: str
    target_frame: str
    map_session_id: str
    map_revision: int
    clock_domain_id: str
    query_time_ns: int
    transform: RigidTransform
    mode: str
    sample_times_ns: tuple[int, ...]
    evidence_id: str

    def __post_init__(self):
        robot_identifier(self.source_robot)
        for field in ("source_robot", "source_frame", "target_frame",
                      "clock_domain_id", "evidence_id"):
            name(getattr(self, field), field)
        uuid_string(self.map_session_id, "map_session_id")
        uint64(self.map_revision, "map_revision")
        timestamp(self.query_time_ns, "query_time_ns")
        if not isinstance(self.transform, RigidTransform):
            raise MappingError("INVALID_INPUT", "transform")
        if not isinstance(self.sample_times_ns, (tuple, list)):
            raise MappingError("INVALID_INPUT", "sample_times_ns")
        samples = tuple(timestamp(v, "sample time") for v in self.sample_times_ns)
        if self.mode == "EXACT":
            valid = samples == (self.query_time_ns,)
        elif self.mode == "INTERPOLATED_VALIDATED":
            valid = (len(samples) == 2 and samples[0] < samples[1]
                     and samples[0] <= self.query_time_ns <= samples[1])
        else:
            valid = False
        if not valid:
            raise MappingError("INVALID_TF_TIME_EVIDENCE")
        object.__setattr__(self, "sample_times_ns", samples)


@dataclass(frozen=True)
class FrameAlignment:
    source_robot: str
    map_frame: str
    mission_frame: str
    map_session_id: str
    map_revision: int
    transform_revision: int
    authority_id: str
    clock_domain_id: str
    transform: RigidTransform
    valid_from_ns: int
    valid_until_ns: int
    position_error_m: float
    rotation_error_rad: float
    status: str = "VALID"

    def __post_init__(self):
        robot_identifier(self.source_robot)
        for field in ("source_robot", "map_frame", "mission_frame",
                      "authority_id", "clock_domain_id"):
            name(getattr(self, field), field)
        uuid_string(self.map_session_id, "map_session_id")
        for field in ("map_revision", "transform_revision"):
            uint64(getattr(self, field), field)
        timestamp(self.valid_from_ns, "valid_from_ns")
        timestamp(self.valid_until_ns, "valid_until_ns")
        integer(self.valid_until_ns, "valid_until_ns", self.valid_from_ns + 1)
        for field in ("position_error_m", "rotation_error_rad"):
            object.__setattr__(self, field, finite(getattr(self, field), field, 0))
        if self.status not in ("VALID", "DEGRADED", "INVALID"):
            raise MappingError("INVALID_INPUT", "alignment status")
        if not isinstance(self.transform, RigidTransform):
            raise MappingError("INVALID_INPUT", "transform")


@dataclass(frozen=True)
class CoordinateVersion:
    source_robot: str
    map_frame: str
    mission_frame: str
    map_session_id: str
    map_revision: int
    transform_revision: int
    authority_id: str


@dataclass(frozen=True)
class Projection:
    original: SourceObservation
    map_point_xyz: tuple[float, float, float]
    mission_point_xyz: tuple[float, float, float]
    version: CoordinateVersion
    sensor_to_map: TimedTransform
    map_to_mission: FrameAlignment
    tf_evidence_id: str
    tf_mode: str
    tf_sample_times_ns: tuple[int, ...]
    projected_at_ns: int
    valid_until_ns: int
    status: str = "VALID"
    reason: str = ""

    @property
    def auto_approach_eligible(self):
        # No real backend correction detection / TF / executor adapter exists.
        return False


@dataclass(frozen=True)
class ObservationReference:
    observation_id: str
    version: CoordinateVersion


@dataclass(frozen=True)
class GoalSnapshot:
    snapshot_id: str
    destination_robot: str
    destination_version: CoordinateVersion
    mission_point_xyz: tuple[float, float, float]
    destination_point_xyz: tuple[float, float, float]
    observations: tuple[ObservationReference, ...]
    created_at_ns: int
    valid_until_ns: int

    @property
    def auto_approach_eligible(self):
        return False


@dataclass(frozen=True)
class GoalValidation:
    coordinates_valid: bool
    reasons: tuple[str, ...]

    @property
    def auto_approach_eligible(self):
        return False


@dataclass(frozen=True)
class Invalidation:
    source_robot: str
    reason: str
    observation_ids: tuple[str, ...]
    goal_snapshot_ids: tuple[str, ...]


@dataclass(frozen=True)
class MapState:
    source_robot: str
    map_frame: str
    map_session_id: str
    map_revision: int = 0
    map_update_sequence: int = 0
