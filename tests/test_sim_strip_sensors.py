"""scripts/sim/strip_sensors.py: the second simulated robot loses only the Gazebo Sensors system (2026-10-15).

With a Sensors system in every Create3 model the Gazebo server aborted when the second robot was created
("A scene node with the name sun already exists", 4 of 4 two-robot runs on the simulation PC); with the plugin removed from
the second robot's description 0 aborts. Nothing else in the description may change.
"""
import importlib.util
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'strip_sensors.py'
if not TOOL.exists():   # the installed-wheel check copies tests/ and config/ only
    raise unittest.SkipTest('installed-wheel checkout has no scripts/')
try:
    spec = importlib.util.spec_from_file_location('strip_sensors', TOOL)
    strip_sensors = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(strip_sensors)
except ImportError as exc:   # rclpy / std_msgs not installed (CI without ROS)
    raise unittest.SkipTest(f'ROS Python packages not installed: {exc}')

SENSORS = ('<plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">\n'
           '        <render_engine>ogre</render_engine>\n      </plugin>')
URDF = ('<robot name="turtlebot4">\n  <gazebo>\n    <plugin filename="libgz-sim-pose-publisher-system.so" '
        'name="gz::sim::systems::PosePublisher"><publish_link_pose>true</publish_link_pose></plugin>\n'
        '  </gazebo>\n  <gazebo>\n      ' + SENSORS + '\n  </gazebo>\n'
        '  <gazebo reference="rplidar_link"><sensor name="rplidar" type="gpu_lidar"/></gazebo>\n'
        '</robot>\n')


class Strip(unittest.TestCase):
    def test_only_the_sensors_system_goes(self):
        out, removed = strip_sensors.strip(URDF)
        self.assertEqual(1, removed)
        self.assertNotIn('gz-sim-sensors-system', out)
        self.assertEqual(URDF.replace(SENSORS, ''), out)
        self.assertIn('PosePublisher', out)
        self.assertIn('<sensor name="rplidar" type="gpu_lidar"/>', out)   # the sensors themselves stay

    def test_a_description_without_it_reports_zero(self):
        out, removed = strip_sensors.strip(URDF.replace(SENSORS, ''))
        self.assertEqual(0, removed)


class SimOdom(unittest.TestCase):
    """Simulated odom counted turns 6.4 % too large with the drive wheels colliding as 1.5 cm wide cylinders
    (undock: odom 191 deg, ground truth 179.5 deg), straight lines exact; with spheres of the wheel radius odom and
    truth agree to 0.01 deg [측정 2026-10-16]. The simulation copy changes the two drive wheel collisions only."""
    WHEELS = ''.join(f'<link name="{side}_wheel">\n    <visual><geometry><cylinder length="0.015" radius="0.03575"/>'
                     f'</geometry></visual>\n    <collision>\n      <geometry>\n        '
                     f'<cylinder length="0.015" radius="0.03575"/>\n      </geometry>\n    </collision>\n  </link>\n'
                     for side in ('left', 'right'))
    CASTER = '<link name="caster"><collision><geometry><cylinder length="0.01" radius="0.01"/></geometry></collision></link>'

    def robot(self):
        return URDF.replace('</robot>', self.WHEELS + self.CASTER + '</robot>')

    def test_only_the_two_drive_wheel_collisions_become_spheres(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=True)
        self.assertEqual({'sensors_removed': 0, 'wheels_sphere': 2}, report)
        self.assertEqual(2, out.count('<sphere radius="0.03575"/>'))
        self.assertEqual(2 + 1, out.count('<cylinder '))   # the wheel visuals and the caster stay cylinders
        self.assertIn('gz-sim-sensors-system', out)        # the first robot keeps its Sensors system

    def test_the_second_robot_gets_both_changes(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=False, wheel_sphere=True)
        self.assertEqual({'sensors_removed': 1, 'wheels_sphere': 2}, report)
        self.assertNotIn('gz-sim-sensors-system', out)

    def test_a_description_that_does_not_match_is_refused(self):
        with self.assertRaises(ValueError):   # no drive wheel cylinders: nothing is guessed
            strip_sensors.patch(URDF, keep_sensors=True, wheel_sphere=True)
        with self.assertRaises(ValueError):   # a second robot without a Sensors system to remove
            strip_sensors.patch(self.robot().replace(SENSORS, ''), keep_sensors=False, wheel_sphere=True)

    def test_every_robot_is_spawned_with_the_sphere_wheels(self):
        launch = (REPO / 'scripts' / 'sim' / 'spawn_extra.launch.py').read_text()
        self.assertIn("'--wheel-sphere'", launch)
        self.assertIn("'-topic', 'robot_description_sim'", launch)


if __name__ == '__main__':
    unittest.main()
