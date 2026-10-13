"""ObserveTarget action server: one recheck window per goal -> RecheckObservation (2026-10-13).

The robot executor sends the goal after the robot stopped at the observation pose (SDD IF-023,
C-11 row 11). The window starts when the goal is accepted; only samples received after that
count. Sensor samples are pushed in by the node that owns the camera and LiDAR inputs
(``add_frame``/``add_scan``, times from ``now_ns``). Cancel or deadline -> outcome UNKNOWN with
every state UNKNOWN, never an absence. The observation time uses this host's measured clock
(webcam_clock, as the webcam does); unmeasured means UNKNOWN.
"""
from dataclasses import dataclass
import math
import threading
import time
from uuid import uuid4

from .recheck import RecheckWindow, window_states
from .target_utils import fill_envelope
from .webcam_clock import observation_clock

STAMP_SEMANTICS = 'CAPTURE'   # samples are timed by their sensor capture stamps (OPS-108)


@dataclass(frozen=True)
class SensorClock:
    """The host whose clock stamps the sensor data (camera / LiDAR driver, TB-6 Pi): its measured sync
    error bound and clock domain, the same field values as the detection launch (sensor_host_*)."""
    error_bound_ns: int
    clock_domain_id: str


def window_margin_ns(record, now_ns, sensor):
    """How much later than the acceptance a capture stamp must be: this host's measured bound (0 when
    unmeasured, no bound claimed) plus the sensor host's."""
    state, bound = observation_clock(record, now_ns)
    return (bound if state == 'SYNCED' else 0) + sensor.error_bound_ns


def capture_clock(record, stamp_ns, sensor, clock_domain_id):
    """(clock_state, bound or None) of a capture stamp: SYNCED only with this host measured and the
    sensor host in the same clock domain; the bound adds both."""
    state, bound = observation_clock(record, stamp_ns)
    if state != 'SYNCED' or sensor.clock_domain_id != clock_domain_id:
        return 'UNKNOWN', None
    return 'SYNCED', bound + sensor.error_bound_ns


class WindowSlot:
    """One recheck window at a time, taken when the goal is accepted (#165 review CDX-165-03).

    The window used to exist only once the goal executed, so two goals accepted before that could
    both run on one window. reserve() in the accept callback, release() when the goal ends."""

    def __init__(self):
        self._lock = threading.Lock()
        self._taken = False

    def reserve(self):
        with self._lock:
            if self._taken:
                return False
            self._taken = True
            return True

    def release(self):
        with self._lock:
            self._taken = False
def deadline_margin_ns(record, now_ns):
    """This host's measured clock error bound now (C-03), or 0 when unmeasured (no bound is claimed).

    The goal deadline comes from the executor's clock, possibly another PC. With a measured bound the
    deadline counts as reached that much earlier. Unmeasured: compared as is, and the observation then
    carries clock UNKNOWN, which C-03 consumers refuse anyway."""
    state, bound = observation_clock(record, now_ns)
    return bound if state == 'SYNCED' else 0


def goal_refusal(*, robot_id, native_frame, point, window_ns, deadline_ns, now_ns, margin_ns, expected_robot,
                 map_frame, max_window_ns):
    """The reason code an ObserveTarget goal is refused, or None."""
    if robot_id != expected_robot:
        return 'ROBOT_MISMATCH'
    if native_frame != map_frame:
        return 'FRAME_MISMATCH'
    if not all(math.isfinite(v) for v in point):
        return 'INVALID_POINT'
    if not 0 < window_ns <= max_window_ns:
        return 'INVALID_WINDOW'
    if deadline_ns:
        if deadline_ns <= now_ns + margin_ns:
            return 'DEADLINE_PASSED'
        if now_ns + window_ns + margin_ns > deadline_ns:
            return 'DEADLINE_TOO_CLOSE'   # the window could not end before the deadline
    return None


class RecheckServer:
    def __init__(self, node, *, action_name, robot_id, map_frame, policy, max_window_ns, validity_ns,
                 clock_domain_id, clock_record, publisher_id, envelope_version, sensor_clock, callback_group=None,
                 now_ns=time.time_ns, log=None):
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rescue_interfaces.action import ObserveTarget
        self._action = ObserveTarget
        self.robot_id, self.map_frame, self.policy = robot_id, map_frame, policy
        self.max_window_ns, self.validity_ns = max_window_ns, validity_ns
        self.clock_domain_id, self.clock_record, self.sensor_clock = clock_domain_id, clock_record, sensor_clock
        self.publisher_id, self.envelope_version = publisher_id, envelope_version
        self.now_ns, self.boot_id = now_ns, str(uuid4())
        self.log = log or node.get_logger()
        self._lock = threading.Lock()
        self._window = None
        self._slot = WindowSlot()
        self._sequence = 0
        self._server = ActionServer(
            node, ObserveTarget, action_name, execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT if self._slot.reserve() else GoalResponse.REJECT,
            cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=callback_group)

    # -- sensor input (called from the owning node's callbacks) ---------------------------------
    def busy(self):
        with self._lock:
            return self._window is not None

    def watched_point(self):
        """The map point of the open window, or None: the caller computes coverage only then."""
        with self._lock:
            return None if self._window is None else self._window.point_map

    def add_frame(self, *, stamp_ns, coverage, confirmed_points, unconfirmed_at_point):
        with self._lock:
            if self._window is not None:
                self._window.add_frame(stamp_ns=stamp_ns, coverage=coverage,
                                       confirmed_points=confirmed_points, unconfirmed_at_point=unconfirmed_at_point)

    def add_scan(self, *, stamp_ns, state):
        with self._lock:
            if self._window is not None:
                self._window.add_scan(stamp_ns=stamp_ns, state=state)

    # -- action ---------------------------------------------------------------------------------
    def _refusal(self, goal, now):
        p = goal.watched_native_point
        return goal_refusal(robot_id=goal.robot_id, native_frame=goal.frame_ref.native_frame, point=(p.x, p.y, p.z),
                            window_ns=goal.requested_window_ns, deadline_ns=goal.deadline_ns, now_ns=now,
                            margin_ns=window_margin_ns(self.clock_record(), now, self.sensor_clock),
                            expected_robot=self.robot_id,
                            map_frame=self.map_frame, max_window_ns=self.max_window_ns)

    def _execute(self, handle):
        try:
            return self._run(handle)
        finally:   # however the goal ends (result, refusal, cancel, exception) the slot is free again
            with self._lock:
                self._window = None
            self._slot.release()

    def _run(self, handle):
        goal, now = handle.request, self.now_ns()
        refusal = self._refusal(goal, now)
        if refusal is not None:
            handle.abort()
            self.log.warning(f'ObserveTarget task={goal.task_id} refused: {refusal}')
            return self._result(goal, 'FAILED', refusal, None, now, now)   # no samples: times are the refusal's
        point = (goal.watched_native_point.x, goal.watched_native_point.y, goal.watched_native_point.z)
        margin = window_margin_ns(self.clock_record(), now, self.sensor_clock)
        with self._lock:
            self._window = RecheckWindow(point, self.policy, start_ns=now, start_margin_ns=margin)
        self.log.info(f'ObserveTarget task={goal.task_id} target={goal.target_id} window '
                      f'{goal.requested_window_ns / 1e9:.1f}s after a {margin / 1e9:.3f}s clock margin point={point}')
        end = now + margin + goal.requested_window_ns   # the window counts from the first admissible capture
        outcome = reason = None
        feedback = self._action.Feedback(request_id=goal.request_id, phase='COLLECTING')
        while outcome is None:
            time.sleep(0.05)
            current = self.now_ns()
            if handle.is_cancel_requested:
                outcome, reason = 'UNKNOWN', 'CANCELED'
            elif goal.deadline_ns and current + deadline_margin_ns(self.clock_record(), current) >= goal.deadline_ns:
                outcome, reason = 'UNKNOWN', 'DEADLINE'
            elif current >= end:
                outcome, reason = 'OBSERVED', ''
            else:
                feedback.envelope.published_at_ns = current
                handle.publish_feedback(feedback)
        with self._lock:
            window, self._window = self._window, None
        span = window.span_ns() or (window.first_ns(), window.first_ns())
        result = self._result(goal, outcome, reason, window, *span)
        if reason == 'CANCELED':
            handle.canceled()
        elif outcome == 'OBSERVED':
            handle.succeed()
        else:
            handle.abort()
        o = result.observation
        self.log.info(f'ObserveTarget task={goal.task_id} {outcome} {reason} camera={o.camera_state} '
                      f'lidar={o.lidar_state} coverage={o.coverage_state} {o.detector_health} clock={o.time.clock_state}')
        return result

    def _result(self, goal, outcome, reason, window, start_ns, end_ns):
        result = self._action.Result(outcome=outcome, reason_code=reason)
        self._sequence += 1
        fill_envelope(result.envelope, version=self.envelope_version, publisher_id=self.publisher_id,
                      boot_id=self.boot_id, sequence=self._sequence, now_ns=self.now_ns())
        o = result.observation
        o.observation_id, o.task_id, o.target_id = str(uuid4()), goal.task_id, goal.target_id
        o.robot_id, o.source_boot_id = goal.robot_id, self.boot_id
        o.frame_ref, o.watched_native_point = goal.frame_ref, goal.watched_native_point
        if outcome == 'OBSERVED':
            o.camera_state, o.lidar_state, o.coverage_state = window_states(window)
        else:   # cancel, deadline, refused goal: never an absence (SDD ObserveTarget Rules)
            o.camera_state = o.lidar_state = o.coverage_state = 'UNKNOWN'
        frames = [] if window is None else window.frames
        scans = [] if window is None else window.scans
        o.detector_health = (f'frames={len(frames)} in_view={sum(1 for f in frames if f[1] == "VALID")} '
                             f'scans={len(scans)}')
        o.sensor_window_start_ns, o.sensor_window_end_ns = start_ns, end_ns
        t = o.time
        t.observed_at_ns, t.valid_until_ns = end_ns, end_ns + self.validity_ns
        t.stamp_semantics, t.clock_domain_id = STAMP_SEMANTICS, self.clock_domain_id
        t.clock_state, bound = capture_clock(self.clock_record(), end_ns, self.sensor_clock, self.clock_domain_id)
        t.has_clock_error_bound, t.clock_error_bound_ns = bound is not None, bound or 0
        return result
