"""Mission-frame Task.goal_pose -> robot map goal, bound to one coordinate version.

The registry is any object with `map_state(robot)`, `current_alignment(robot,
now_ns=)` and `mission_frame` (rescue_mapping.MappingRegistry). It is passed in,
not imported, so rescue_robot gains no package dependency.

The goal carries the CoordinateVersion it was validated under
(map_session_id, map_revision, transform_revision of the moving robot).
- `bind(task)` runs before SEND: if that version is current, the alignment's
  inverse (mission -> map) turns the goal into the robot's map frame. The
  result is used for SEND only; the Task itself is never changed.
- `check(task)` is the adapter guard on every step. It fails (-> GUARD_INVALID,
  cancel, stop gate) when the version is no longer current or the alignment is
  unusable, which also catches an alignment change while driving (C-04).
REOBSERVE_TARGET/APPROACH_TARGET additionally need `target_gate(task)` is True;
the caller wires GoalSnapshot coordinates_valid and auto_approach_eligible
(always False today). Nothing here makes approach eligible.
"""
import math
import time

from rescue_contracts import RescueError
from rescue_robot.executor import ContractError, GoalPose

TARGET_KINDS = ('REOBSERVE_TARGET', 'APPROACH_TARGET')


class MissionGoalBinder:
    def __init__(self, registry, robot_id, *, target_gate, wall_clock_ns=time.time_ns):
        if not callable(target_gate):
            raise ContractError('target_gate callable required')
        self.registry = registry
        self.robot_id = robot_id
        self.target_gate = target_gate
        self.wall_clock_ns = wall_clock_ns
        self.last_reason = ''

    def _current(self, task):
        """The usable alignment matching this task's goal version, or raise."""
        if task.robot_id != self.robot_id:
            raise ContractError('task for another robot', 'WRONG_ROBOT')
        goal = task.goal_pose
        if goal.frame_id != self.registry.mission_frame:
            raise ContractError('goal_pose must be in the mission frame')
        state = self.registry.map_state(self.robot_id)
        if (goal.map_session_id, goal.map_revision) != (state.map_session_id, state.map_revision):
            raise ContractError('goal map version is not current', 'FRAME_REVISION_MISMATCH')
        alignment = self.registry.current_alignment(self.robot_id, now_ns=self.wall_clock_ns())
        if (goal.map_session_id, goal.map_revision, goal.transform_revision) != (
                alignment.map_session_id, alignment.map_revision, alignment.transform_revision):
            raise ContractError('goal alignment version is not current', 'FRAME_REVISION_MISMATCH')
        return alignment

    def bind(self, task):
        if task.kind == 'HOLD':
            return None
        alignment = self._current(task)
        goal = task.goal_pose
        to_map = alignment.transform.inverse()
        x, y, _ = to_map.apply((goal.x, goal.y, 0.0))
        hx, hy, _ = to_map.apply((goal.x + math.cos(goal.yaw), goal.y + math.sin(goal.yaw), 0.0))
        return GoalPose(alignment.map_frame, x, y, math.atan2(hy - y, hx - x),
                        goal.map_session_id, goal.map_revision, goal.transform_revision)

    def check(self, task):
        try:
            if task.kind == 'HOLD':
                return True
            self._current(task)
            if task.kind in TARGET_KINDS and self.target_gate(task) is not True:
                raise ContractError('target goal not authorized', 'NOT_AUTHORIZED')
        except RescueError as exc:
            self.last_reason = exc.detail_code
            return False
        self.last_reason = ''
        return True
