"""RETURN_HOME as the coordinator sends it passes the executor's wire check (simulation run 6, 2026-10-16; no ROS).

Run 6 (#407): RETURN_HOME sent 12 times, TB-6's executor answered `ExecuteTask rejected: INVALID_INPUT ... native
frame` 11 times (TASK_REFUSED). The coordinator fills a non-route goal's frame_ref with the mission frame and map
basis only (fill_execute_task_goal: alignment is the mapping's call, never asserted), but validate_task_wire
asked every non-HOLD, non-SEARCH_AREA goal for a full frame key (native frame, alignment VALID). RETURN_HOME, like
SEARCH_AREA, names a destination the robot resolves itself (configured home/<robot_id> or the coordinator's
destination record) against its own current alignment, and validate_resolved_goal checks the resolved pose
against the goal's frame and revisions. So RETURN_HOME gets SEARCH_AREA's check. TB-5's trip to the exit is a
RETURN_HOME too ('home/robot_b'). The profile's home values never reached this check: the goal was refused on
its frame_ref before any destination was looked up.

Run 8 (#418) showed the next refusal (mission frame); the coordinator-flow test below is expectedFailure on main.
"""
import unittest
from types import SimpleNamespace as NS

from rescue_coordination import ros_node
from rescue_robot.executor import ContractError
from rescue_robot.task_wire import validate_task_wire
from test_coordination_ros_core import goal_template
from test_task_goal_fields import uid

NOW = 1_700_000_000_000_000_000
S = 1_000_000_000


def return_home_goal(robot_id='robot_a'):
    fields = dict(command_id=uid(1), task_id=uid(2), robot_id=robot_id, authority_epoch=uid(3), generation=1,
                  kind='RETURN_HOME', issued_at_ns=NOW, deadline_ns=NOW + 60 * S, robot_boot_id=uid(4),
                  policy_version='search-live-20261012', target_id=None, target_revision=None, search_area_ref=None,
                  home_ref=f'home/{robot_id}', lease_id=uid(5), route=None, approval=None,
                  goal_pose=None)   # search._goal_pose: RETURN_HOME has none, the robot resolves its home (5A)
    goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
    lease = NS(lease_id=uid(5), robot_id=robot_id, robot_boot_id=uid(4), coordinator_id='coord', authority_epoch=uid(3),
               generation=1, renewal_sequence=0, issued_at_ns=NOW, valid_until_ns=NOW + 2 * S)
    return goal, lease


class ReturnHomeWire(unittest.TestCase):
    def test_the_coordinator_leaves_native_frame_and_alignment_empty(self):
        goal, _ = return_home_goal()
        self.assertEqual(('', ''), (goal.frame_ref.native_frame, goal.frame_ref.alignment_status))

    @unittest.expectedFailure
    def test_return_home_from_the_coordinator_passes(self):
        for robot in ('robot_a', 'robot_b'):   # TB-6 home, TB-5 exit
            with self.subTest(robot=robot):
                goal, lease = return_home_goal(robot)
                validate_task_wire(goal, lease, now_ns=NOW, clock_error_bound_ns=0)

    def test_still_refused_for_another_robots_frame_or_half_a_basis(self):
        goal, lease = return_home_goal()
        goal.frame_ref.robot_id = 'robot_b'
        with self.assertRaises(ContractError):
            validate_task_wire(goal, lease, now_ns=NOW, clock_error_bound_ns=0)
        goal, lease = return_home_goal()
        goal.frame_ref.mission_frame = 'map'      # a basis is either all there or not at all
        with self.assertRaises(ContractError):
            validate_task_wire(goal, lease, now_ns=NOW, clock_error_bound_ns=0)


class FromTheCoordinatorFlow(unittest.TestCase):
    """Run 8 (#418): `ExecuteTask rejected: INVALID_INPUT ... mission frame` 4 times after #408. #408's test gave the
    goal a pose; the coordinator gives RETURN_HOME none (search._goal_pose, decision 5A), so its frame_ref carries only
    the robot. Here the goal comes from the coordinator's own rescue flow, unchanged, through every executor check."""

    def coordinator_home(self):
        from test_coordination_rescue_flow import RescueFlowTests, recheck, result
        flow = RescueFlowTests('test_a_good_recheck_is_rescue_confirmed_then_tb6_returns_to_the_docking_approach')
        flow.setUp()
        flow.accepted_candidate()
        fields = flow.reobserve()
        flow.core.on_result(fields['task_id'], result(fields, obs=recheck(fields)))
        (home,) = flow.tasks('RETURN_HOME')
        return dict(home, lease_id=uid(5))   # the node's _send adds the lease id

    @unittest.expectedFailure
    def test_the_coordinators_return_home_passes_every_executor_check(self):
        from rescue_robot.executor import GoalPose
        from rescue_robot.task_wire import validate_resolved_goal
        fields = self.coordinator_home()
        self.assertIsNone(fields['goal_pose'])
        goal = ros_node.fill_execute_task_goal(goal_template(), fields, coordinator_id='coord')
        lease = NS(lease_id=uid(5), robot_id=fields['robot_id'], robot_boot_id=fields['robot_boot_id'],
                   coordinator_id='coord', authority_epoch=fields['authority_epoch'], generation=fields['generation'],
                   renewal_sequence=0, issued_at_ns=fields['issued_at_ns'], valid_until_ns=fields['issued_at_ns'] + 2 * S)
        validate_task_wire(goal, lease, now_ns=fields['issued_at_ns'], clock_error_bound_ns=0)
        # the robot's resolver: home/<robot> on its current alignment (task_site_runtime.resolve_goal)
        pose = GoalPose('mission', 1.0, 2.0, 0.5, uid(6), 3, 1)
        self.assertEqual(pose, validate_resolved_goal(goal, pose))

if __name__ == '__main__':
    unittest.main()
