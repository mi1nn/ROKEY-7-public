"""scripts/sim/strip_sensors.py: the second simulated robot loses only the Gazebo Sensors system (2026-10-15).

With a Sensors system in every Create3 model the Gazebo server aborted when the second robot was created
("A scene node with the name sun already exists", 4 of 4 two-robot runs on mu-03); with the plugin removed from
the second robot's description 0 aborts. Nothing else in the description may change.
"""
import importlib.util
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
try:
    spec = importlib.util.spec_from_file_location('strip_sensors', REPO / 'scripts' / 'sim' / 'strip_sensors.py')
    strip_sensors = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(strip_sensors)
except ImportError as exc:   # rclpy / std_msgs not installed (CI without ROS)
    raise unittest.SkipTest(f'ROS Python packages not installed: {exc}')

SENSORS = ('<plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">\n'
           '        <render_engine>ogre</render_engine>\n      </plugin>')
URDF = ('<robot name="turtlebot4">\n  <gazebo>\n    <plugin filename="libgz-sim-pose-publisher-system.so" '
        'name="gz::sim::systems::PosePublisher"><publish_link_pose>true</publish_link_pose></plugin>\n'
        '  </gazebo>\n  <gazebo>\n      ' + SENSORS + '\n  </gazebo>\n'
        '  <gazebo reference="rplidar_link"><sensor name="rplidar" type="gpu_lidar"/></gazebo>\n</robot>\n')


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


if __name__ == '__main__':
    unittest.main()
