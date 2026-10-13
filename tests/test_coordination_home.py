"""Coordinator home positions from deployment parameters (C5; was ``home_positions=()``, ros_node build_core).

``home_poses`` is [x, y, yaw] per role robot, SCOUT first then SUPPORT, in the mission frame on the
same map basis as the other configured goals (goal_map_*). Left out = no home: RETURN_HOME stays
NO_HOME_POSITION, never a made-up pose. A wrong length is refused at start.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from rescue_coordination.mission import HomePosition

REPO = Path(__file__).resolve().parents[1]
MAP = '22222222-2222-4222-8222-222222222222'
ROLES = {'SCOUT': 'robot_a', 'SUPPORT': 'robot_b'}
BASIS = (MAP, 0, 1)


def values(**changes):
    base = {'profile_path': str(REPO / 'config' / 'deployment.v2.example.json'), 'entry_yaws': [0.0, 1.5],
            'policy_version': 'p', 'executable_bases': ['PRIOR_MAP'], 'task_duration_s': 60.0,
            'lease_duration_s': 10.0, 'executor_policy_version': 'e', 'mission_frame_id': 'mission',
            'authority_epoch': '00000000-0000-4000-8000-000000000004', 'coordinator_id': 'c',
            'envelope_schema_version': 'e', 'status_max_age_s': 5.0, 'tick_period_s': .2,
            'route_segment_goals': [1.0, 0.0, 0.0], 'route_revision': 1, 'observe_goal': [3.0, 1.0, 0.0],
            'goal_map_session_id': MAP, 'goal_map_revision': 0, 'goal_transform_revision': 1,
            'merge_distance_m': .5, 'report_retry_s': 1.0, 'lease_renew_period_s': .2, 'admission_timeout_s': 1.0,
            'lease_message_ttl_s': 1.0, 'route_segment_passage_free': [True], 'camera_watch_roi': None,
            'camera_watch_timeout_s': 2.0, 'home_poses': None, 'target_clock_domain_id': 'site-clock',
            'target_max_age_s': 2.0, 'target_max_future_s': 0.05,
            'support_dispatch': 'OPERATOR', 'approach_standoff_m': 0.8, 'support_pose_max_age_s': 2.0,
            'support_exit_wait_s': 1.0, 'recheck_max_clock_error_s': 2.0}
    return base | changes


class HomePoseTests(unittest.TestCase):
    def test_one_pose_per_role_robot_scout_first_on_the_goal_basis(self):
        homes = ros_node.home_positions([0.5, -1.0, 0.0, 1.5, -1.0, 3.14], ROLES, BASIS)
        self.assertEqual((HomePosition('robot_a', (0.5, -1.0), 0.0, MAP, 0, 1),
                          HomePosition('robot_b', (1.5, -1.0), 3.14, MAP, 0, 1)), homes)

    def test_left_out_or_empty_is_no_home(self):
        self.assertEqual((), ros_node.home_positions(None, ROLES, BASIS))
        self.assertEqual((), ros_node.home_positions([], ROLES, BASIS))

    def test_wrong_length_or_bad_value_is_refused(self):
        for flat in ([0.5, -1.0, 0.0], [0.5, -1.0, 0.0, 1.5, -1.0], [0.] * 9, [float('nan'), 0., 0., 0., 0., 0.]):
            with self.subTest(flat=flat), self.assertRaises(RescueError):
                ros_node.home_positions(flat, ROLES, BASIS)


class MapStateTests(unittest.TestCase):
    """#176: each robot publishes MapState; a VALID localization on a session/revision is the current basis.

    transform_revision is not in MapState: it is the profile map_basis value, the coordinator's goal basis.
    """
    def setUp(self):
        from test_coordination_ros_core import make_core
        self.core = make_core()

    def map_state(self, robot, quality='VALID', **changes):
        from test_coordination_search import MAP
        values = dict(robot_id=robot, map_session_id=MAP, map_revision=3, localization_quality=quality)
        return NS(**(values | changes))

    def test_valid_localization_records_the_basis_with_the_goal_transform_revision(self):
        from test_coordination_ros_core import SCOUT, SUPPORT
        from test_coordination_search import MAP
        self.assertTrue(self.core.on_map_state(self.map_state(SCOUT)))
        self.assertEqual({SCOUT: (MAP, 3, 1)}, self.core.mission.map_basis)
        self.core.on_map_state(self.map_state(SUPPORT, map_revision=4))
        self.assertEqual((MAP, 4, 1), self.core.mission.map_basis[SUPPORT])

    def test_unknown_localization_withdraws_the_basis(self):
        from test_coordination_ros_core import SCOUT
        self.core.on_map_state(self.map_state(SCOUT))
        self.assertFalse(self.core.on_map_state(self.map_state(SCOUT, quality='UNKNOWN')))
        self.assertNotIn(SCOUT, self.core.mission.map_basis)  # never kept as if still valid

    def test_a_robot_outside_the_policy_is_refused(self):
        with self.assertRaises(RescueError):
            self.core.on_map_state(self.map_state('unit_9'))


class BuildCoreTests(unittest.TestCase):
    def test_build_core_passes_the_homes_to_the_mission(self):
        core = ros_node.build_core(values(home_poses=[0.5, -1.0, 0.0, 1.5, -1.0, 0.0]))
        self.assertEqual({'robot_a': (0.5, -1.0), 'robot_b': (1.5, -1.0)},
                         {r: h.mission_xy for r, h in core.mission.homes.items()})
        self.assertEqual(BASIS, core.mission.homes['robot_a'].basis)

    def test_build_core_without_homes_keeps_return_home_refused(self):
        self.assertEqual({}, ros_node.build_core(values()).mission.homes)

    def test_home_poses_parameter_is_typed_and_may_be_left_out(self):
        # May be left out, so declared beside the required PARAMETERS (which launch passes in full).
        self.assertNotIn('home_poses', ros_node.PARAMETERS)
        self.assertEqual('DOUBLE_ARRAY', ros_node.UNSET_OK_PARAMETERS['home_poses'])


if __name__ == '__main__':
    unittest.main()
