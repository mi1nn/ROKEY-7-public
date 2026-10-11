"""Executor <-> Nav2 command adapter boundary. Imports no ROS package.

One CommandAdapter owns one Executor (one robot) and one NavigationPort (that
robot's NavigateToPose client). It keeps the executor's events apart:

- admission   `submit` -> Executor.admit -> TaskAdmission (the only acceptance
               record a coordinator may use; a Nav2 goal acceptance never makes one)
- send        Executor intent ('SEND', id) -> port.send_goal(id, task.goal_pose)
- cancel      Executor intent ('CANCEL', id) -> port.cancel_goal(id)
- goal reply  port -> on_goal_response -> Executor.goal_response
- cancel reply port -> on_cancel_response; recorded only, it is not a stop
- terminal    port -> on_terminal -> Executor.goal_terminal (execution fence)
- stop proof  odometry -> on_odom -> Executor.observe_motion

Deadline, lease, boot, authority and generation checks stay in Executor
(admit/dispatch/tick/renew). This module only supplies times and evidence.
Callbacks from several threads are serialized by one lock, and every executor
call reads the same local monotonic clock inside that lock.
"""
import math
import threading
import time

from rescue_contracts import RescueError, ros_time_to_ns
from rescue_robot.executor import ContractError, Executor, Readiness

# action_msgs/msg/GoalStatus terminal codes (ROS 2 action design); other codes
# are not terminal and must not fence execution.
GOAL_STATUS_TERMINAL = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
READINESS_FLAGS = ('driver', 'sensors', 'slam', 'nav2', 'undocked', 'watchdog')


class CommandAdapter:
    def __init__(self, executor, port, *, guards, clock_error_bound_s, bind_goal=None,
                 clock=time.monotonic, wall_clock_ns=time.time_ns):
        """`guards(task) -> bool` is re-checked before SEND and on every step
        (map revision etc.; caller-injected). `clock_error_bound_s` is the
        largest accepted |host wall - odom header| offset in local float
        seconds (not C-03's int clock_error_bound_ns); no default.
        `bind_goal(task) -> GoalPose` runs right before dispatch and gives the
        pose actually sent (e.g. MissionGoalBinder.bind: mission -> robot map);
        if it raises, nothing is sent and the guard fails. Without it the
        task's own goal_pose is sent unchanged. The Task is never modified."""
        if not isinstance(executor, Executor):
            raise ContractError('executor type')
        if not callable(guards):
            raise ContractError('guards callable required')
        if type(clock_error_bound_s) not in (int, float) or not 0 < clock_error_bound_s < math.inf:
            raise ContractError('clock_error_bound_s')
        self.clock_error_bound_s = float(clock_error_bound_s)
        self.executor = executor
        self.port = port
        self.guards = guards
        self.bind_goal = bind_goal if bind_goal is not None else (lambda task: task.goal_pose)
        self.clock = clock
        self.wall_clock_ns = wall_clock_ns
        self.lock = threading.RLock()
        self.goals = {}
        self.sent = set()
        self.cancel_requested = set()
        self.status = {}
        self.events = []
        self._reported = set()

    @property
    def robot_id(self):
        return self.executor.robot_id

    # ---- evidence inputs -------------------------------------------------
    def observe_status(self, **flags):
        """Record fresh readiness flags (driver, sensors, slam, nav2, undocked, watchdog)."""
        with self.lock:
            now = self.clock()
            for name, value in flags.items():
                if name not in READINESS_FLAGS or type(value) is not bool:
                    raise ContractError('readiness flag ' + str(name))
                self.status[name] = (value, now)

    def on_dock_status(self, is_docked):
        if type(is_docked) is not bool:
            raise ContractError('is_docked')
        self.observe_status(undocked=not is_docked)

    def readiness(self, now):
        """All six flags, stamped by the oldest; any missing flag is not ready."""
        if any(name not in self.status for name in READINESS_FLAGS):
            return Readiness(0.0, False, False, False, False, False, False)
        oldest = min(stamp for _, stamp in self.status.values())
        return Readiness(oldest, *(self.status[name][0] for name in READINESS_FLAGS))

    def on_odom(self, *, stamp_sec, stamp_nanosec, linear, angular):
        """nav_msgs/Odometry header + twist -> one motion sample.

        [미확인] The header is robot system time; its offset from this host's
        wall clock is moved onto the local monotonic axis:
            sample_steady = received_steady - (wall_now - header_ns) / 1e9
        This assumes robot and PC clocks agree within clock_error_bound_s (C-03;
        not measured yet, Q-14/Q-15). An offset beyond that bound, or an
        invalid stamp, is not used as a sample: Executor.invalidate_stop_evidence
        restarts the stop window, so missing evidence surfaces as
        STOP_EVIDENCE_STALE / STOP_UNCONFIRMED.
        """
        with self.lock:
            received = self.clock()
            wall = self.wall_clock_ns()
            speed = math.hypot(*linear)
            turn = math.hypot(*angular)
            try:
                offset = (wall - ros_time_to_ns(stamp_sec, stamp_nanosec)) / 1e9
                sample = received - offset if abs(offset) <= self.clock_error_bound_s else None
            except RescueError:
                sample = None
            try:
                if sample is None or sample < 0:
                    self.events.append(('ODOM_UNUSABLE', None))
                    self.executor.invalidate_stop_evidence(now=received)
                    return False
                return self.executor.observe_motion(sample_steady=sample, received_steady=received,
                                                    linear=speed, angular=turn)
            finally:
                self._after()

    # ---- commands ---------------------------------------------------------
    def submit(self, task):
        """Admit a Task; returns the executor's TaskAdmission (the only acceptance record)."""
        with self.lock:
            now = self.clock()
            receipt = self.executor.admit(task, now_wall_ns=self.wall_clock_ns(), now_steady=now,
                                          readiness=self.readiness(now))
            self.events.append(('ADMISSION', task.command_id, receipt.reason))
            return receipt

    def step(self):
        """Periodic tick: dispatch an admitted task or re-check guards/deadlines."""
        with self.lock:
            now = self.clock()
            wall = self.wall_clock_ns()
            ready = self.readiness(now)
            active = self.executor.active
            guards_valid = True if active is None else self.guards(active) is True
            try:
                if active is not None and self.executor.state == 'PLANNING':
                    if guards_valid and active.kind != 'HOLD':
                        try:
                            self.goals[active.command_id] = self.bind_goal(active)
                        except RescueError as exc:
                            self.events.append(('BIND_FAILED', active.command_id, exc.code))
                            guards_valid = False
                    self.executor.dispatch(now_wall_ns=wall, now_steady=now, readiness=ready,
                                           guards_valid=guards_valid)
                else:
                    self.executor.tick(now_wall_ns=wall, now_steady=now, readiness=ready,
                                       guards_valid=guards_valid)
            finally:
                self._after()

    def renew(self, command_id, authority_epoch, generation, sequence, *, lease_duration=None):
        with self.lock:
            return self.executor.renew(command_id, authority_epoch, generation, sequence,
                                       now=self.clock(), lease_duration=lease_duration)

    def request_stop(self, cause, outcome='CANCELED'):
        with self.lock:
            try:
                self.executor.request_stop(cause, now=self.clock(), outcome=outcome)
            finally:
                self._after()

    # ---- action callbacks (from this robot's port only) -------------------
    def on_goal_response(self, command_id, accepted):
        with self.lock:
            if command_id not in self.sent:
                self.events.append(('FOREIGN_GOAL_RESPONSE', command_id))
                return
            self.events.append(('GOAL_RESPONSE', command_id, accepted))
            try:
                self.executor.goal_response(command_id, accepted=accepted, now=self.clock())
            finally:
                self._after()

    def on_cancel_response(self, command_id, accepted):
        """A cancel acknowledgement is not stop evidence; it is only recorded."""
        with self.lock:
            kind = 'CANCEL_RESPONSE' if command_id in self.cancel_requested else 'FOREIGN_CANCEL_RESPONSE'
            self.events.append((kind, command_id, accepted))

    def on_terminal(self, command_id, status_code):
        with self.lock:
            if command_id not in self.sent:
                self.events.append(('FOREIGN_TERMINAL', command_id))
                return False
            status = GOAL_STATUS_TERMINAL.get(status_code)
            if status is None:
                self.events.append(('NON_TERMINAL_STATUS', command_id, status_code))
                return False
            self.events.append(('TERMINAL', command_id, status))
            try:
                return self.executor.goal_terminal(command_id, terminal_status=status, now=self.clock())
            finally:
                self._after()

    def result(self, command_id):
        return self.executor.results.get(command_id)

    # ---- internals ----------------------------------------------------------
    def _after(self):
        for intent, command_id in self.executor.drain_intents():
            if intent == 'SEND':
                self.sent.add(command_id)
                self.events.append(('SEND', command_id))
                self.port.send_goal(command_id, self.goals[command_id])
            else:
                self.cancel_requested.add(command_id)
                self.events.append(('CANCEL_REQUEST', command_id))
                self.port.cancel_goal(command_id)
        for command_id, result in self.executor.results.items():
            if command_id not in self._reported:
                self._reported.add(command_id)
                self.events.append(('RESULT', command_id, result.outcome, result.stop_confirmed))
