"""Every simulated robot is spawned through scripts/sim/spawn_extra.launch.py, whose dock has no IR emitter rays.

S10 (2026-10-16): with the dock's IR emitter rays on (dock_description visualize_rays, default true) another robot's lidar
passed through walls near the dock, 50 of 3200 rays in a still scene and 0 with them off (config/sim/README.md).
The launch files need ROS to run, so this reads them as text: the argument must be there and nothing may go round it.
"""
from pathlib import Path
import re
import unittest

SIM = Path(__file__).resolve().parents[1] / 'scripts' / 'sim'
HAS_SIM = (SIM / 'spawn_extra.launch.py').exists()   # the installed-wheel check copies tests/ and config/ only


@unittest.skipUnless(HAS_SIM, 'installed-wheel checkout has no scripts/')
class SpawnPath(unittest.TestCase):
    def test_the_dock_is_made_without_ir_emitter_rays(self):
        text = (SIM / 'spawn_extra.launch.py').read_text()
        self.assertIn("'visualize_rays': 'false'", text)

    def test_the_first_robot_keeps_its_sensors_system_through_the_same_launch(self):
        text = (SIM / 'sim_world.launch.py').read_text()
        self.assertIn('spawn_extra.launch.py', text)
        self.assertIn("('strip_sensors', 'false')", text)

    def test_no_script_spawns_with_the_stock_launch_that_keeps_the_rays(self):
        for path in list(SIM.glob('*.sh')) + list(SIM.glob('*.launch.py')):
            code = '\n'.join(line for line in path.read_text().splitlines() if not line.lstrip().startswith('#'))
            with self.subTest(path.name):
                # a launch call in a script, or the file name as a launch path element in Python (not prose)
                use = r"ros2 launch \S+ turtlebot4_(spawn|gz)\.launch\.py|'turtlebot4_(spawn|gz)\.launch\.py'"
                self.assertIsNone(re.search(use, code),
                                  'spawn through sim_world.launch.py / spawn_extra.launch.py')


if __name__ == '__main__':
    unittest.main()
