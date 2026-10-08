"""In-memory coordinate authority. It neither subscribes to TF nor moves robots."""

from dataclasses import replace
from threading import RLock
from uuid import uuid4

from .geometry import MappingError, finite, name, robot_identifier, timestamp, uint64, uuid_string, vector
from .models import (CoordinateVersion, FrameAlignment, GoalSnapshot, GoalValidation,
                     Invalidation, MapState, ObservationReference, Projection,
                     SourceObservation, TimedTransform)


class MappingRegistry:
    """Serialize local updates and keep source-specific coordinate provenance.

    All times are integer nanoseconds in one explicitly named clock domain.
    Alignment error limits must be supplied by a caller; no field thresholds
    are invented here. An input marked VALID is geometric metadata, not proof
    of a working SLAM adapter. Automatic approach eligibility is always false.
    """

    def __init__(self, *, clock_domain_id, alignment_authority,
                 max_position_error_m, max_rotation_error_rad, mission_frame="mission"):
        self.clock_domain_id = name(clock_domain_id, "clock_domain_id")
        self.alignment_authority = name(alignment_authority, "alignment_authority")
        self.mission_frame = name(mission_frame, "mission_frame")
        self.max_position_error_m = finite(max_position_error_m, "max_position_error_m", 0)
        self.max_rotation_error_rad = finite(max_rotation_error_rad, "max_rotation_error_rad", 0)
        self._lock = RLock()
        self._maps = {}
        self._sessions_seen = set()
        self._alignments = {}
        self._last_alignment = {}
        self._originals = {}
        self._bound_tf = {}
        self._projections = {}
        self._history = {}
        self._goals = {}
        self._invalid_goals = set()
        self._last_time = None
        self._clock_fault = False

    def _time(self, now_ns):
        timestamp(now_ns, "now_ns")
        if self._last_time is not None and now_ns < self._last_time:
            self._clock_fault = True
        if self._clock_fault:
            raise MappingError("CLOCK_RESET_REQUIRED", "create a new registry after clock recovery")
        self._last_time = now_ns

    def _map(self, robot):
        robot_identifier(robot)
        if robot not in self._maps:
            raise MappingError("UNKNOWN_ROBOT", robot)
        return self._maps[robot]

    def map_state(self, robot):
        with self._lock:
            return self._map(robot)

    def _invalidate(self, robot, reason, status="NEEDS_REOBSERVATION"):
        affected = []
        for oid, projection in tuple(self._projections.items()):
            if (projection.original.source_robot == robot
                    and projection.status in ("VALID", "STALE_DERIVATION")):
                invalid = replace(projection, status=status, reason=reason)
                self._projections[oid] = invalid
                self._history[oid].append(invalid)
                affected.append(oid)
        goals = []
        for gid, goal in self._goals.items():
            depends = (goal.destination_robot == robot or
                       any(ref.version.source_robot == robot for ref in goal.observations))
            if depends and gid not in self._invalid_goals:
                self._invalid_goals.add(gid)
                goals.append(gid)
        return Invalidation(robot, reason, tuple(sorted(affected)), tuple(sorted(goals)))

    def start_session(self, source_robot, map_frame, map_session_id):
        """Start at revision zero; previously retired session IDs cannot return."""
        with self._lock:
            robot_identifier(source_robot)
            name(map_frame, "map_frame")
            uuid_string(map_session_id, "map_session_id")
            current = self._maps.get(source_robot)
            if current and current.map_session_id == map_session_id:
                if current.map_frame != map_frame:
                    raise MappingError("SESSION_CONTENT_CONFLICT")
                return Invalidation(source_robot, "UNCHANGED", (), ())
            key = (source_robot, map_session_id)
            if key in self._sessions_seen:
                raise MappingError("RETIRED_SESSION")
            event = self._invalidate(source_robot, "MAP_SESSION_CHANGED")
            self._maps[source_robot] = MapState(source_robot, map_frame, map_session_id)
            self._sessions_seen.add(key)
            self._alignments.pop(source_robot, None)
            self._last_alignment.pop(source_robot, None)
            return event

    def advance_map_revision(self, source_robot, map_session_id, map_revision):
        """Explicit correction input; this does not detect a real loop closure."""
        with self._lock:
            state = self._map(source_robot)
            uuid_string(map_session_id, "map_session_id")
            uint64(map_revision, "map_revision")
            if state.map_session_id != map_session_id:
                raise MappingError("MAP_SESSION_MISMATCH")
            if map_revision <= state.map_revision:
                raise MappingError("NONINCREASING_MAP_REVISION")
            self._maps[source_robot] = replace(state, map_revision=map_revision)
            self._alignments.pop(source_robot, None)
            return self._invalidate(source_robot, "MAP_REVISION_CHANGED")

    def record_map_update(self, source_robot, map_session_id, sequence):
        """Occupancy updates do not by themselves mean a coordinate correction."""
        with self._lock:
            state = self._map(source_robot)
            uuid_string(map_session_id, "map_session_id")
            uint64(sequence, "map_update_sequence")
            if state.map_session_id != map_session_id:
                raise MappingError("MAP_SESSION_MISMATCH")
            if sequence < state.map_update_sequence:
                raise MappingError("STALE_MAP_UPDATE")
            self._maps[source_robot] = replace(state, map_update_sequence=sequence)
            return self._maps[source_robot]

    def set_alignment(self, alignment):
        with self._lock:
            if not isinstance(alignment, FrameAlignment):
                raise MappingError("INVALID_INPUT", "alignment")
            state = self._map(alignment.source_robot)
            if alignment.authority_id != self.alignment_authority:
                raise MappingError("WRONG_ALIGNMENT_AUTHORITY")
            if alignment.clock_domain_id != self.clock_domain_id:
                raise MappingError("CLOCK_DOMAIN_MISMATCH")
            if (alignment.map_frame != state.map_frame or
                    alignment.mission_frame != self.mission_frame):
                raise MappingError("FRAME_MISMATCH")
            if (alignment.map_session_id, alignment.map_revision) != (
                    state.map_session_id, state.map_revision):
                raise MappingError("MAP_VERSION_MISMATCH")
            previous = self._last_alignment.get(alignment.source_robot)
            if previous is not None:
                if alignment.transform_revision == previous.transform_revision:
                    if alignment == previous and alignment.source_robot in self._alignments:
                        return Invalidation(alignment.source_robot, "UNCHANGED", (), ())
                    raise MappingError("ALIGNMENT_REVISION_CONFLICT")
                if alignment.transform_revision < previous.transform_revision:
                    raise MappingError("STALE_ALIGNMENT_REVISION")
            usable = (alignment.status == "VALID"
                      and alignment.position_error_m <= self.max_position_error_m
                      and alignment.rotation_error_rad <= self.max_rotation_error_rad)
            status = "STALE_DERIVATION" if usable else "NEEDS_REOBSERVATION"
            event = self._invalidate(alignment.source_robot, "ALIGNMENT_CHANGED", status)
            self._alignments[alignment.source_robot] = alignment
            self._last_alignment[alignment.source_robot] = alignment
            return event

    def invalidate_alignment(self, source_robot, reason="ALIGNMENT_INVALID"):
        with self._lock:
            self._map(source_robot)
            name(reason, "reason")
            self._alignments.pop(source_robot, None)
            return self._invalidate(source_robot, reason)

    def _alignment(self, robot, now_ns):
        self._map(robot)
        alignment = self._alignments.get(robot)
        if alignment is None:
            raise MappingError("ALIGNMENT_UNAVAILABLE")
        if alignment.status != "VALID":
            raise MappingError("ALIGNMENT_" + alignment.status)
        if not alignment.valid_from_ns <= now_ns < alignment.valid_until_ns:
            raise MappingError("ALIGNMENT_EXPIRED_OR_NOT_YET_VALID")
        if (alignment.position_error_m > self.max_position_error_m or
                alignment.rotation_error_rad > self.max_rotation_error_rad):
            raise MappingError("ALIGNMENT_UNCERTAIN")
        return alignment

    def current_alignment(self, source_robot, *, now_ns):
        """Read-only: the usable map->mission alignment now, or MappingError.

        Same checks as projection (VALID, inside validity window, error limits).
        It grants nothing: auto approach eligibility stays false.
        """
        with self._lock:
            self._time(now_ns)
            return self._alignment(source_robot, now_ns)

    @staticmethod
    def _version(alignment):
        return CoordinateVersion(alignment.source_robot, alignment.map_frame,
                                 alignment.mission_frame, alignment.map_session_id,
                                 alignment.map_revision, alignment.transform_revision,
                                 alignment.authority_id)

    def retain_observation(self, observation):
        """Retain the source record even if a subsequent projection fails."""
        with self._lock:
            if not isinstance(observation, SourceObservation):
                raise MappingError("INVALID_INPUT", "observation")
            self._map(observation.source_robot)
            if observation.clock_domain_id != self.clock_domain_id:
                raise MappingError("CLOCK_DOMAIN_MISMATCH")
            previous = self._originals.get(observation.observation_id)
            if previous is not None and previous != observation:
                raise MappingError("ID_CONTENT_CONFLICT")
            if previous is None:
                self._originals[observation.observation_id] = observation
            return self._originals[observation.observation_id]

    def original(self, observation_id):
        with self._lock:
            uuid_string(observation_id, "observation_id")
            if observation_id not in self._originals:
                raise MappingError("UNKNOWN_OBSERVATION")
            return self._originals[observation_id]

    def project(self, observation, sensor_to_map, *, now_ns):
        with self._lock:
            self._time(now_ns)
            original = self.retain_observation(observation)
            state = self._map(original.source_robot)
            if now_ns < original.observed_at_ns:
                raise MappingError("FUTURE_OBSERVATION")
            if now_ns >= original.valid_until_ns:
                raise MappingError("OBSERVATION_EXPIRED")
            if (original.map_session_id, original.map_revision) != (
                    state.map_session_id, state.map_revision):
                raise MappingError("NEEDS_REOBSERVATION")
            if not isinstance(sensor_to_map, TimedTransform):
                raise MappingError("TF_UNAVAILABLE")
            tf = sensor_to_map
            if tf.source_robot != original.source_robot:
                raise MappingError("TF_ROBOT_MISMATCH")
            if tf.source_frame != original.source_frame or tf.target_frame != state.map_frame:
                raise MappingError("TF_FRAME_MISMATCH")
            if tf.clock_domain_id != self.clock_domain_id:
                raise MappingError("CLOCK_DOMAIN_MISMATCH")
            if tf.query_time_ns != original.observed_at_ns:
                raise MappingError("TF_TIME_MISMATCH", "latest TF is not an observation-time lookup")
            if (tf.map_session_id, tf.map_revision) != (state.map_session_id, state.map_revision):
                raise MappingError("TF_MAP_VERSION_MISMATCH")
            if original.observation_id in self._bound_tf and self._bound_tf[original.observation_id] != tf:
                raise MappingError("TF_CONTENT_CONFLICT", "the original sensor-time transform is immutable")
            alignment = self._alignment(original.source_robot, now_ns)
            map_point = tf.transform.apply(original.point_xyz)
            result = Projection(original, map_point, alignment.transform.apply(map_point),
                                self._version(alignment), tf, alignment, tf.evidence_id, tf.mode,
                                tf.sample_times_ns, now_ns,
                                min(original.valid_until_ns, alignment.valid_until_ns))
            self._projections[original.observation_id] = result
            self._bound_tf[original.observation_id] = tf
            self._history.setdefault(original.observation_id, []).append(result)
            return result

    def projection(self, observation_id, *, now_ns):
        with self._lock:
            self._time(now_ns)
            uuid_string(observation_id, "observation_id")
            result = self._projections.get(observation_id)
            if result is None:
                raise MappingError("PROJECTION_UNAVAILABLE")
            if result.status not in ("VALID", "STALE_DERIVATION"):
                return result
            if now_ns >= result.valid_until_ns:
                return replace(result, status="EXPIRED", reason="PROJECTION_EXPIRED")
            try:
                alignment = self._alignment(result.original.source_robot, now_ns)
            except MappingError as exc:
                return replace(result, status="NEEDS_REOBSERVATION",
                               reason=exc.detail_code.removeprefix("rescue_mapping."))
            if result.version != self._version(alignment):
                same_map = (result.version.source_robot == alignment.source_robot
                            and result.version.map_frame == alignment.map_frame
                            and result.version.map_session_id == alignment.map_session_id
                            and result.version.map_revision == alignment.map_revision)
                return replace(result, status="STALE_DERIVATION" if same_map else "NEEDS_REOBSERVATION",
                               reason=result.reason if same_map and result.status == "STALE_DERIVATION"
                               else "VERSION_MISMATCH")
            return result

    def projection_history(self, observation_id):
        with self._lock:
            self.original(observation_id)
            return tuple(self._history.get(observation_id, ()))

    def capture_goal_snapshot(self, observation_ids, destination_robot,
                              mission_point_xyz, *, now_ns):
        """Bind an E2 candidate to every source version and destination version.

        This is not a navigation command or authorization to approach.
        """
        with self._lock:
            self._time(now_ns)
            if not isinstance(observation_ids, (list, tuple)) or not observation_ids:
                raise MappingError("INVALID_INPUT", "observation_ids must be nonempty")
            ids = tuple(uuid_string(v, "observation_id") for v in observation_ids)
            if len(set(ids)) != len(ids):
                raise MappingError("DUPLICATE_OBSERVATION_REFERENCE")
            results = tuple(self.projection(oid, now_ns=now_ns) for oid in ids)
            if any(result.status != "VALID" for result in results):
                raise MappingError("SOURCE_PROJECTION_INVALID")
            alignment = self._alignment(destination_robot, now_ns)
            mission_point = vector(mission_point_xyz, 3, "mission_point_xyz")
            snapshot = GoalSnapshot(
                str(uuid4()), destination_robot, self._version(alignment), mission_point,
                alignment.transform.inverse().apply(mission_point),
                tuple(ObservationReference(p.original.observation_id, p.version) for p in results),
                now_ns, min(alignment.valid_until_ns, *(p.valid_until_ns for p in results)))
            self._goals[snapshot.snapshot_id] = snapshot
            return snapshot

    def validate_goal_snapshot(self, snapshot, *, now_ns):
        """Recheck at acceptance and again immediately before any future publish.

        This process-local check cannot make a remote check-and-publish atomic.
        A future executor must implement that protocol and its stop handling.
        """
        with self._lock:
            if not isinstance(snapshot, GoalSnapshot):
                return GoalValidation(False, ("INVALID_GOAL_SNAPSHOT", "BACKEND_NOT_INTEGRATED"))
            try:
                self._time(now_ns)
                uuid_string(snapshot.snapshot_id, "snapshot_id")
                if self._goals.get(snapshot.snapshot_id) != snapshot:
                    raise MappingError("UNREGISTERED_OR_MODIFIED_GOAL_SNAPSHOT")
                if snapshot.snapshot_id in self._invalid_goals:
                    raise MappingError("GOAL_VERSION_INVALIDATED")
                if now_ns >= snapshot.valid_until_ns:
                    raise MappingError("GOAL_SNAPSHOT_EXPIRED")
                alignment = self._alignment(snapshot.destination_robot, now_ns)
                if snapshot.destination_version != self._version(alignment):
                    raise MappingError("DESTINATION_VERSION_MISMATCH")
                for reference in snapshot.observations:
                    result = self.projection(reference.observation_id, now_ns=now_ns)
                    if result.status != "VALID" or result.version != reference.version:
                        raise MappingError("SOURCE_VERSION_MISMATCH")
            except MappingError as exc:
                return GoalValidation(False, (exc.detail_code.removeprefix("rescue_mapping."),
                                              "BACKEND_NOT_INTEGRATED"))
            return GoalValidation(True, ("BACKEND_NOT_INTEGRATED",))
