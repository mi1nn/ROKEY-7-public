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
        '  <gazebo><plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">\n'
        '    <parameters>/opt/ros/jazzy/share/irobot_create_control/config/control.yaml</parameters>\n'
        '  </plugin></gazebo>\n</robot>\n')
SIM_CONTROL = REPO / 'config' / 'sim' / 'create3_control.yaml'


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
    """Simulated odom counted turns 6.4 % too large (undock: odom 191 deg, ground truth 179.5 deg; commanded 360 deg turn:
    odom 361, truth 339), straight lines exact [측정 2026-10-16]. The simulation copy of the description points
    gz_ros2_control at config/sim/create3_control.yaml, whose only change is wheel_separation_multiplier."""

    def test_the_controller_parameters_are_swapped_and_nothing_else(self):
        out, swapped = strip_sensors.use_control_params(URDF, '/x/create3_control.yaml')
        self.assertEqual(1, swapped)
        self.assertIn('<parameters>/x/create3_control.yaml</parameters>', out)
        self.assertEqual(URDF.replace('/opt/ros/jazzy/share/irobot_create_control/config/control.yaml',
                                      '/x/create3_control.yaml'), out)

    def test_the_first_robot_keeps_its_sensors_system_but_gets_the_sim_parameters(self):
        out, report = strip_sensors.patch(URDF, keep_sensors=True, control_params='/x/c.yaml')
        self.assertIn('gz-sim-sensors-system', out)
        self.assertIn('<parameters>/x/c.yaml</parameters>', out)
        self.assertEqual({'sensors_removed': 0, 'control_swapped': 1}, report)
        out, report = strip_sensors.patch(URDF, keep_sensors=False, control_params='/x/c.yaml')
        self.assertNotIn('gz-sim-sensors-system', out)
        self.assertEqual({'sensors_removed': 1, 'control_swapped': 1}, report)

    def test_a_description_that_does_not_match_is_refused(self):
        with self.assertRaises(ValueError):   # no controller parameters to swap: nothing is guessed
            strip_sensors.patch(URDF.replace('irobot_create_control/config/control.yaml', 'other.yaml'),
                                keep_sensors=True, control_params='/x/c.yaml')
        with self.assertRaises(ValueError):   # a second robot without a Sensors system to remove
            strip_sensors.patch(URDF.replace(SENSORS, ''), keep_sensors=False, control_params='/x/c.yaml')

    def test_the_sim_parameters_differ_from_upstream_only_in_the_multiplier(self):
        upstream = Path('/opt/ros/jazzy/share/irobot_create_control/config/control.yaml')
        if not upstream.exists():
            self.skipTest('irobot_create_control not installed')
        strip = lambda text: [l.split('#')[0].rstrip() for l in text.splitlines() if l.split('#')[0].strip()]  # noqa: E731
        ours, theirs = strip(SIM_CONTROL.read_text()), strip(upstream.read_text())
        changed = [(a, b) for a, b in zip(ours, theirs) if a != b]
        self.assertEqual(len(theirs), len(ours))
        self.assertEqual([('      wheel_separation_multiplier: 1.064', '      wheel_separation_multiplier: 1.0')], changed)


if __name__ == '__main__':
    unittest.main()
