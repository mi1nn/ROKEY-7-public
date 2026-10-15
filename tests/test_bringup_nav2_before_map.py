"""TB-5 Nav2 never became active (simulation 2026-10-15, run 232037; field 2026-10-15 17:30 too), no ROS.

TB-5 (FROZEN_MAP_AMCL) has no map frame until the first freeze: its AMCL starts on the frozen map
only then (frozen_localization.launch.py). But robot.launch.py, which carries TB-5's Nav2, runs before
the freeze (field runbook step 5; its executor must publish the MapState the frozen map relay
follows). TurtleBot4 navigation_launch.py starts Nav2 at once (autostart default 'true').

Nav2 Jazzy:
- Costmap2DROS::on_activate waits for global_frame -> robot_base_frame at most
  initial_transform_timeout (default 60 s), then returns FAILURE. The global costmap is in 'map'.
- LifecycleManager::startup logs 'Failed to bring up all requested nodes. Aborting bringup.', keeps
  is_active false and does not try again (only a broken bond restarts it).
So TB-5's Nav2 stays inactive for the run and the executor refuses arm with NAV_NOT_READY (#340).

The first freeze comes when TB-6 finds the doll or ends its search: minutes after the launch, with no
bound before the disaster is declared. No finite TF timeout fixes this. So robot.launch.py starts a
FROZEN_MAP_AMCL robot's Nav2 with autostart false, and rescue_bringup.nav2_start sends STARTUP once
map -> base_link is on that robot's TF. A refused or unanswered STARTUP is logged, and the executor's
readiness keeps Nav2 NOT_READY (NAV2_LIFECYCLE ... inactive), so arm is refused, never made up.
"""
import sys
import unittest
from unittest.mock import patch

from test_bringup_field_launch import FieldCase
from test_bringup_no_backup import REPO, vendor_params
from test_robot_r2_same_map import RclpyLogger

LAUNCH = REPO / 'src' / 'rescue_bringup' / 'launch' / 'robot.launch.py'
NAV2_DEFAULT_TRANSFORM_TIMEOUT_S = 60.0   # nav2_costmap_2d jazzy: declare_parameter("initial_transform_timeout", 60.0)


class Tb5Nav2BeforeTheMap(FieldCase):
    def values(self, robot_id):
        return self.robot(field=self.field(), robot_id=robot_id, occupancy_sha256='')

    def nav2_params(self, values):
        from rescue_bringup.field_launch import nav2_with_passage
        from rescue_bringup.nav2_field import field_nav2
        return nav2_with_passage(field_nav2(vendor_params()), values['passage'])

    def test_tb5_is_on_the_frozen_map_only(self):
        self.assertEqual('FROZEN_MAP_AMCL', self.values('robot_b')['task_input']['task.saved_map.mode'])

    def test_the_global_costmap_needs_the_map_frame_and_waits_60_s(self):
        """What TB-5 meets before the first freeze: no map frame, and Nav2's own wait is 60 s."""
        costmap = self.nav2_params(self.values('robot_b'))['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual('map', costmap['global_frame'])
        self.assertEqual(NAV2_DEFAULT_TRANSFORM_TIMEOUT_S,
                         costmap.get('initial_transform_timeout', NAV2_DEFAULT_TRANSFORM_TIMEOUT_S))

    @unittest.skipUnless(LAUNCH.exists(), 'installed-wheel checkout has no src/ launch files')
    def test_tb5_nav2_is_started_after_its_map_frame_exists(self):
        """robot.launch.py must not auto-start a FROZEN_MAP_AMCL robot's Nav2 at launch (it gives up after 60 s
        and never retries); it is started once map -> base_link is there."""
        values = self.values('robot_b')
        self.assertIn('nav2_autostart', values)
        self.assertFalse(values['nav2_autostart'])
        text = LAUNCH.read_text()
        self.assertIn("'autostart': str(values['nav2_autostart']).lower()", text)
        self.assertIn("executable='nav2_start_after_map'", text)
        self.assertIn('nav2_start_after_map = rescue_bringup.nav2_start:main',
                      (REPO / 'src' / 'rescue_bringup' / 'setup.py').read_text())

    def test_tb5_start_values(self):
        start = self.values('robot_b')['nav2_start']
        self.assertEqual({'manage_service': '/robot5/lifecycle_manager_navigation/manage_nodes', 'map_frame': 'map',
                          'base_frame': 'base_link', 'tf_topic': '/robot5/tf', 'tf_static_topic': '/robot5/tf_static',
                          'poll_period_s': 0.5, 'answer_wait_s': 120.0}, start)
        # the same lifecycle manager the executor asks is_active (#340)
        self.assertEqual(start['manage_service'].rsplit('/', 1)[0],
                         self.values('robot_b')['task_input']['nav2_lifecycle_service'].rsplit('/', 1)[0])

    def test_tb6_nav2_starts_at_launch_as_before(self):
        """TB-6 (SLAM_THEN_FROZEN) has the SLAM map frame when robot.launch.py runs (tb6_slam first)."""
        values = self.values('robot_a')
        self.assertTrue(values['nav2_autostart'])
        self.assertIsNone(values['nav2_start'])


ARGS = dict(service='/robot5/lifecycle_manager_navigation/manage_nodes', map_frame='map', base_frame='base_link',
            answer_wait_s=120.0)


def step(state, has_map=True, service_ready=True, answer=None, sent_for_s=0.0):
    from rescue_bringup.nav2_start import step
    return step(state, has_map=has_map, service_ready=service_ready, answer=answer, sent_for_s=sent_for_s, **ARGS)


class StartAfterTheMap(unittest.TestCase):
    def test_waits_for_the_map_frame_and_says_so_once(self):
        state, said, send = step('START', has_map=False)
        self.assertEqual(('WAITING_MAP', False), (state, send))
        self.assertTrue(said[1].startswith('NAV2_STARTUP_WAITING map -> base_link not on TF yet'), said)
        self.assertEqual(('WAITING_MAP', None, False), step(state, has_map=False))

    def test_waits_for_the_service(self):
        state, said, send = step('WAITING_MAP', service_ready=False)
        self.assertEqual(('WAITING_SERVICE', 'warning', False), (state, said[0], send))
        self.assertEqual(('WAITING_SERVICE', None, False), step(state, service_ready=False))

    def test_sends_startup_once_the_map_frame_is_there(self):
        state, said, send = step('WAITING_MAP')
        self.assertEqual(('SENT', True), (state, send))
        self.assertTrue(said[1].startswith('NAV2_STARTUP_SENT /robot5/lifecycle_manager_navigation/manage_nodes'))
        self.assertEqual(('SENT', None, False), step(state, sent_for_s=10.0))        # waits for the answer

    def test_active(self):
        self.assertEqual(('ACTIVE', ('info', 'NAV2_STARTUP /robot5/lifecycle_manager_navigation/manage_nodes active'),
                          False), step('SENT', answer=True))
        self.assertEqual(('ACTIVE', None, False), step('ACTIVE', has_map=False))

    def test_a_refused_startup_is_an_error_with_the_retry_command_and_never_resent(self):
        state, said, send = step('SENT', answer=False)
        self.assertEqual(('FAILED', 'error', False), (state, said[0], send))
        self.assertIn('NAV2_STARTUP_FAILED', said[1])
        self.assertIn('NAV2_LIFECYCLE inactive, arm refused', said[1])
        self.assertIn('ManageLifecycleNodes "{command: 0}"', said[1])
        self.assertEqual(('FAILED', None, False), step('FAILED'))

    def test_no_answer_in_time_is_an_error(self):
        self.assertEqual(('SENT', None, False), step('SENT', sent_for_s=119.9))
        state, said, send = step('SENT', sent_for_s=120.0)
        self.assertEqual(('NO_ANSWER', 'error', False), (state, said[0], send))
        self.assertIn('NAV2_STARTUP_NO_ANSWER', said[1])
        self.assertEqual(('NO_ANSWER', None, False), step('NO_ANSWER', answer=True))

    def test_every_line_goes_out_with_the_rclpy_rule(self):
        from rescue_bringup.nav2_start import emit
        logger = RclpyLogger()
        for said in (step('START', has_map=False)[1], step('WAITING_MAP', service_ready=False)[1],
                     step('WAITING_MAP')[1], step('SENT', answer=True)[1], step('SENT', answer=False)[1],
                     step('SENT', sent_for_s=200.0)[1]):
            emit(logger, said)
        self.assertEqual(6, len(logger.lines))


class StartupLine(unittest.TestCase):
    def test_git_sha_and_parameters(self):
        """AGENTS 7: every node logs its source git SHA and the settings it runs with when it starts."""
        from rescue_bringup.nav2_start import startup_line
        line = startup_line({'manage_service': '/robot5/lifecycle_manager_navigation/manage_nodes'}, 'abc123')
        self.assertTrue(line.startswith('git_sha=abc123 parameters='), line)
        self.assertIn('/robot5/lifecycle_manager_navigation/manage_nodes', line)
        self.assertIn('log.info(startup_line(values, git_sha()))',
                      (REPO / 'src' / 'rescue_bringup' / 'rescue_bringup' / 'nav2_start.py').read_text())


class FailedStartupIsNotReady(unittest.TestCase):
    """After NAV2_STARTUP_FAILED the manager answers is_active false: the executor says NOT_READY and logs why."""

    def test_executor_readiness_after_a_refused_startup(self):
        from test_robot_nav2_ready import FakeNode, fake_ros
        with patch.dict(sys.modules, fake_ros()):
            from rescue_robot.ros_node import Nav2Port
            node, now = FakeNode(), [100.0]
            service = '/robot5/lifecycle_manager_navigation/is_active'
            port = Nav2Port(node, '/robot5/navigate_to_pose', service, clock=lambda: now[0], max_age_s=3.0)
            self.assertEqual('FAILED', step('SENT', answer=False)[0])
            port.server_ready()
            node.services[service].calls[-1].finish(False)      # the manager after the refused STARTUP
            self.assertFalse(port.server_ready())
            self.assertIn(f'NAV2_LIFECYCLE {service} inactive: nav2 NOT_READY', node.logs)


if __name__ == '__main__':
    unittest.main()
