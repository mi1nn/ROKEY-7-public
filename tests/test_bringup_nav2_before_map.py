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
bound before the disaster is declared. No finite TF timeout fixes this; Nav2 must be started once the
map frame is there. The test marked expectedFailure describes that and fails on main.
"""
import unittest

from test_bringup_field_launch import FieldCase
from test_bringup_no_backup import REPO, vendor_params

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
    @unittest.expectedFailure
    def test_tb5_nav2_is_started_after_its_map_frame_exists(self):
        """robot.launch.py must not auto-start a FROZEN_MAP_AMCL robot's Nav2 at launch (it gives up after 60 s
        and never retries); it is started once map -> base_link is there."""
        values = self.values('robot_b')
        self.assertIn('nav2_autostart', values)       # robot.launch.py passes no autostart today: 'true'
        self.assertFalse(values['nav2_autostart'])
        self.assertIn("'autostart'", LAUNCH.read_text())


if __name__ == '__main__':
    unittest.main()
