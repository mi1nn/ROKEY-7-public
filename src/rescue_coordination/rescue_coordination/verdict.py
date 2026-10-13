"""Rescue judgement from one TB-6 recheck: contracts C-11 rows 7-9 (camera candidate) and 11 (operator).

Pure function; the node core passes the recheck observation and the reference the recheck is linked to.
RESCUE_CONFIRMED needs every condition below; the first one that fails is the RECHECK_NEEDED reason. Nothing is
repaired or defaulted: an unknown state is not ABSENT/CLEAR/VALID, and a missing time basis is not comparable.

1. linked: the observation names the linked task, the target and the SCOUT (row 7);
2. frame: frame_ref on the coordinator's map basis and mission frame, alignment VALID (row 7);
3. time basis: the reference and the observation both declare one (stamp semantics known, SYNCED, an error
   bound), the same clock_domain_id, and the error sum within ``max_clock_error_ns`` (row 8);
4. order: ``sensor_window_start_ns - reference.at_ns > error sum`` (row 8; row 11 with the acceptance time);
5. sensing: camera ABSENT, LiDAR CLEAR, coverage VALID (row 9, BRD 3.5.5).

Row 11 (an operator REQUEST_RECHECK) compares with this coordinator's acceptance time, which needs the
coordinator's own measured clock domain and error. They are not measured on this path, so an operator
recheck is CANDIDATE_TIME_BASIS_UNUSABLE until they are (contract: "측정 전에는 UNKNOWN").
"""
from dataclasses import dataclass

RECHECK_NEEDED_REASONS = ('CANDIDATE_NOT_LINKED', 'FRAME_REVISION_MISMATCH', 'CANDIDATE_TIME_BASIS_UNUSABLE',
                          'ORDER_VIOLATION', 'CAMERA_NOT_ABSENT', 'LIDAR_NOT_CLEAR', 'COVERAGE_NOT_VALID')


@dataclass(frozen=True)
class Reference:
    """What the recheck must come after: the accepted camera candidate, or the operator's acceptance."""
    source: str                    # 'CAMERA' or 'OPERATOR'
    at_ns: int                     # candidate observed_at_ns / coordinator acceptance time
    clock_domain_id: str | None
    clock_error_bound_ns: int | None
    declared: bool                 # the candidate's SiteEvent.time_basis_declared


def _declared(t):
    return t.stamp_semantics != 'UNKNOWN' and t.clock_state == 'SYNCED' and bool(t.has_clock_error_bound)


def judge(observation, reference, *, expected_task_id, target_id, scout_id, basis, mission_frame,
          max_clock_error_ns):
    """(state, reason): ('RESCUE_CONFIRMED', 'CONFIRMED') or ('RECHECK_NEEDED', one of RECHECK_NEEDED_REASONS)."""
    o, f, t = observation, observation.frame_ref, observation.time

    def needed(reason):
        return 'RECHECK_NEEDED', reason

    if (o.task_id, o.target_id, o.robot_id) != (expected_task_id, target_id, scout_id):
        return needed('CANDIDATE_NOT_LINKED')
    if ((f.map_session_id, f.map_revision, f.transform_revision) != tuple(basis) or f.mission_frame != mission_frame
            or f.alignment_status != 'VALID'):
        return needed('FRAME_REVISION_MISMATCH')
    if (reference.source != 'CAMERA' or not reference.declared or reference.clock_error_bound_ns is None
            or not _declared(t) or t.clock_domain_id != reference.clock_domain_id):
        return needed('CANDIDATE_TIME_BASIS_UNUSABLE')
    error_sum = int(t.clock_error_bound_ns) + int(reference.clock_error_bound_ns)
    if error_sum > max_clock_error_ns:
        return needed('CANDIDATE_TIME_BASIS_UNUSABLE')
    if not o.sensor_window_start_ns - reference.at_ns > error_sum:
        return needed('ORDER_VIOLATION')
    if o.camera_state != 'ABSENT':
        return needed('CAMERA_NOT_ABSENT')
    if o.lidar_state != 'CLEAR':
        return needed('LIDAR_NOT_CLEAR')
    if o.coverage_state != 'VALID':
        return needed('COVERAGE_NOT_VALID')
    return 'RESCUE_CONFIRMED', 'CONFIRMED'
