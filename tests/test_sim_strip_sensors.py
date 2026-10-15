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
        self.assertEqual({'sensors_removed': 0, 'wheels_sphere': 2, 'depth_camera': 0, 'lidar': 0}, report)
        self.assertEqual(2, out.count('<sphere radius="0.03575"/>'))
        self.assertEqual(2 + 1, out.count('<cylinder '))   # the wheel visuals and the caster stay cylinders
        self.assertIn('gz-sim-sensors-system', out)        # the first robot keeps its Sensors system

    def test_the_second_robot_gets_both_changes(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=False, wheel_sphere=True)
        self.assertEqual({'sensors_removed': 1, 'wheels_sphere': 2, 'depth_camera': 0, 'lidar': 0}, report)
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


class SimDepth(unittest.TestCase):
    """The OAK-D depth in the simulation copy: nearest depth 0.40 m like the real stereo (작전 2026-10-16). The RGB
    image of the same sensor keeps its own clip (0.3 m). The noise is not here: Gazebo did not apply <camera><noise>
    to the rgbd depth (measured std 0.0000 m); wall_clock_relay.py adds it."""
    CAMERA = ('<sensor name="rgbd_camera" type="rgbd_camera">\n      <camera>\n        <horizontal_fov>1.25</horizontal_fov>\n'
              '        <image>\n          <width>320</width>\n          <height>240</height>\n        </image>\n'
              '        <clip>\n          <near>0.3</near>\n          <far>100</far>\n        </clip>\n'
              '        <optical_frame_id>oakd_rgb_camera_optical_frame</optical_frame_id>\n      </camera>\n'
              '      <always_on>1</always_on>\n      <update_rate>30</update_rate>\n    </sensor>')

    def robot(self):
        return URDF.replace('</robot>', '<gazebo reference="oakd_rgb_camera_frame">' + self.CAMERA + '</gazebo></robot>')

    def test_depth_gets_its_own_near_clip(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=False, depth_near_m=0.40)
        self.assertEqual(1, report['depth_camera'])
        camera = out[out.index('<sensor name="rgbd_camera"'):]
        self.assertIn('<depth_camera><clip><near>0.4</near><far>100</far></clip></depth_camera>', camera)
        self.assertNotIn('<noise>', camera)
        self.assertIn('<near>0.3</near>', camera)   # the RGB clip stays

    def test_without_the_options_the_camera_is_untouched(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=False)
        self.assertEqual(self.robot(), out)
        self.assertEqual(0, report['depth_camera'])

    def test_no_rgbd_camera_is_refused_when_asked_to_change_it(self):
        with self.assertRaises(ValueError):
            strip_sensors.patch(URDF, keep_sensors=True, wheel_sphere=False, depth_near_m=0.4)

    def test_every_robot_is_spawned_with_the_real_depth(self):
        launch = (REPO / 'scripts' / 'sim' / 'spawn_extra.launch.py').read_text()
        self.assertIn("'--depth-near', '0.40'", launch)


class SimLidar(unittest.TestCase):
    """The rplidar in the simulation copy: 9 Hz like the real one (/robot6/scan 9 Hz [측정 2026-10-15]), gaussian
    noise sigma 0.01 m [추정], and the Gazebo range floor at 0.10 m: with 0.164 m the GPU lidar let rays whose true
    distance is under about 0.23 m through walls (0.164 / cos 45 on the cube face) [측정 2026-10-16]. The relay then
    drops returns under the real 0.164 m, so the field sees what the real lidar gives."""
    LIDAR = ('<sensor name="rplidar" type="gpu_lidar">\n      <update_rate>62.0</update_rate>\n      <visualize>1</visualize>\n'
             '      <lidar>\n        <scan><horizontal><samples>640</samples></horizontal></scan>\n'
             '        <range>\n          <min>0.164</min>\n          <max>12.0</max>\n          <resolution>0.01</resolution>\n'
             '        </range>\n      </lidar>\n    </sensor>')

    def robot(self):
        return URDF.replace('</robot>', '<gazebo reference="rplidar_link">' + self.LIDAR + '</gazebo></robot>')

    def test_rate_floor_and_noise(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=False,
                                          lidar_rate_hz=9.0, lidar_min_m=0.10, lidar_noise_m=0.01)
        self.assertEqual(1, report['lidar'])
        lidar = out[out.index('<sensor name="rplidar"'):]
        self.assertIn('<update_rate>9</update_rate>', lidar)
        self.assertIn('<min>0.1</min>', lidar)
        self.assertIn('<max>12.0</max>', lidar)
        self.assertIn('<noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>', lidar)

    def test_no_lidar_is_refused_when_asked_to_change_it(self):
        with self.assertRaises(ValueError):
            strip_sensors.patch(URDF, keep_sensors=True, wheel_sphere=False, lidar_rate_hz=9.0, lidar_min_m=0.1,
                                lidar_noise_m=0.01)

    def test_every_robot_is_spawned_with_the_real_lidar(self):
        launch = (REPO / 'scripts' / 'sim' / 'spawn_extra.launch.py').read_text()
        self.assertIn("'--lidar-rate', '9'", launch)
        self.assertIn("'--lidar-min', '0.10'", launch)
        self.assertIn("'--lidar-noise', '0.01'", launch)


class SimRenderRates(unittest.TestCase):
    """Render rates of the sensors nothing downstream reads at full rate (2026-10-16, RTF): the 4 cliff and 7 IR
    intensity ray sensors of each Create3 (62 Hz in the description) and the OAK-D rgbd_camera (30 Hz; the relay passes
    at most 10 Hz depth and 5 Hz RGB). Only update_rate changes; the rplidar keeps its own rate."""
    RAY = ('<sensor name="{}" type="gpu_lidar">\n      <update_rate>62</update_rate>\n      <visualize>False</visualize>\n'
           '      <lidar><scan><horizontal><samples>5</samples></horizontal></scan></lidar>\n    </sensor>')
    NAMES = ('cliff_side_left', 'cliff_side_right', 'cliff_front_left', 'cliff_front_right',
             'ir_intensity_front_center_left', 'ir_intensity_front_center_right', 'ir_intensity_front_left',
             'ir_intensity_front_right', 'ir_intensity_left', 'ir_intensity_right', 'ir_intensity_side_left')

    def robot(self, names=NAMES):
        rays = ''.join(f'<gazebo reference="{n}">' + self.RAY.format(n) + '</gazebo>' for n in names)
        return URDF.replace('</robot>', rays + '<gazebo reference="rplidar_link">' + SimLidar.LIDAR + '</gazebo>'
                            '<gazebo reference="oakd_rgb_camera_frame">' + SimDepth.CAMERA + '</gazebo></robot>')

    def test_cliff_and_ir_rays_get_the_rate_and_nothing_else(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=False, rays_rate_hz=10.0)
        self.assertEqual(11, report['small_rays'])
        self.assertEqual(11, out.count('<update_rate>10</update_rate>'))
        self.assertIn('<update_rate>62.0</update_rate>', out)   # the rplidar is not a small ray sensor
        self.assertIn('<update_rate>30</update_rate>', out)     # nor the camera
        self.assertEqual(self.robot().replace('<update_rate>62</update_rate>', '<update_rate>10</update_rate>'), out)

    def test_camera_rate(self):
        out, report = strip_sensors.patch(self.robot(), keep_sensors=True, wheel_sphere=False, camera_rate_hz=10.0)
        self.assertEqual(1, report['camera_rate'])
        camera = out[out.index('<sensor name="rgbd_camera"'):]
        self.assertIn('<update_rate>10</update_rate>', camera)
        self.assertEqual(11, out.count('<update_rate>62</update_rate>'))

    def test_a_description_that_does_not_match_is_refused(self):
        with self.assertRaises(ValueError):   # a Create3 with one ray sensor fewer: nothing is guessed
            strip_sensors.patch(self.robot(self.NAMES[1:]), keep_sensors=True, wheel_sphere=False, rays_rate_hz=10.0)
        with self.assertRaises(ValueError):
            strip_sensors.patch(URDF, keep_sensors=True, wheel_sphere=False, camera_rate_hz=10.0)

    def test_every_robot_is_spawned_with_the_lower_render_rates(self):
        """RTF of two idle robots: 0.84 at 62 Hz rays / 30 Hz camera, 0.99 at 1 Hz / 10 Hz [측정 2026-10-16]."""
        launch = (REPO / 'scripts' / 'sim' / 'spawn_extra.launch.py').read_text()
        self.assertIn("'--rays-rate', '1'", launch)
        self.assertIn("'--camera-rate', '10'", launch)


if __name__ == '__main__':
    unittest.main()
