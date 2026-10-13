"""Destinations the coordinator hands the robots through ~/task_inputs include their homes (field fix, 2026-10-13).

The robot resolves a RETURN_HOME's ``home_ref`` (``home/<robot_id>``) in the coordinator's live record
(rescue_robot task_site_runtime.resolve_goal, live_clock ``destinations``). The record carried only the SCOUT's
search areas, so TB-5's exit and TB-6's docking approach point could not be found on a real robot. Each
``home_poses`` entry is now in it under ``home/<robot_id>`` with the robot, pose and map basis.
"""
import unittest

from rescue_coordination import live_task_inputs, mission as mission_module
from test_coordination_ros_core import SCOUT, SUPPORT, make_core
from test_coordination_search import MAP


class LiveDestinationTests(unittest.TestCase):
    def test_homes_are_destinations_under_home_robot_id(self):
        core = make_core()
        homes = (mission_module.HomePosition(SCOUT, (0.5, -1.0), 0.0, MAP, 3, 1),
                 mission_module.HomePosition(SUPPORT, (3.0, 0.0), 3.14, MAP, 3, 1))
        core.mission.homes = {h.robot_id: h for h in homes}
        d = live_task_inputs.live_destinations(core)
        self.assertEqual(dict(robot_id=SUPPORT, frame_id=core.search.policy.mission_frame_id, xy_yaw=[3.0, 0.0, 3.14],
                              map_session_id=MAP, map_revision=3, transform_revision=1), d[f'home/{SUPPORT}'])
        self.assertEqual(SCOUT, d[f'home/{SCOUT}']['robot_id'])

    def test_search_areas_stay_and_no_home_means_no_home_entry(self):
        core = make_core()
        d = live_task_inputs.live_destinations(core)
        self.assertFalse([ref for ref in d if ref.startswith('home/')])
        areas = [a.site_id for a in core.search.policy.areas if core.search.policy.executable(a)]
        self.assertEqual(sorted(areas), sorted(d))
        self.assertTrue(all(d[a]['robot_id'] == SCOUT for a in areas))


if __name__ == '__main__':
    unittest.main()
