"""TB-5 start alignment in the field launch: values from the field run file, node in frozen_localization (no ROS).

Every robots.<id>.start_align.* value is required for a FROZEN_MAP_AMCL robot (no defaults) and goes to the
start_align node unchanged; the SLAM robot (TB-6) has none and gets no node (its AMCL starts at the frozen pose).
"""
import json
import unittest

from test_bringup_field_launch import FIELD, REPO, FieldCase
from test_bringup_map_lifecycle import frozen_field
from test_robot_start_align import CONFIG

LAUNCH = REPO / 'src' / 'rescue_bringup' / 'launch' / 'frozen_localization.launch.py'


class FieldFile(FieldCase):
    def test_repository_field_file_has_every_value_for_tb5_only(self):
        raw = json.loads(FIELD.read_text())
        b = raw['robots']['robot_b']
        self.assertEqual(set(CONFIG), {k[len('start_align.'):] for k in b if k.startswith('start_align.')})
        self.assertFalse([k for k in raw['robots']['robot_a'] if k.startswith('start_align.')])
        self.assertIn('start_align', raw['source'])   # [제안] values say where they come from

    def test_a_missing_or_extra_value_is_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def missing(raw):
            raw['robots']['robot_b'].pop('start_align.max_updates')

        def on_tb6(raw):
            raw['robots']['robot_a']['start_align.max_updates'] = 20

        def not_a_count(raw):
            raw['robots']['robot_b']['start_align.max_updates'] = 2.5
        for change in (missing, on_tb6, not_a_count):
            with self.subTest(change.__name__), self.assertRaises(FieldLaunchError):
                self.field(change)


class LocalizationValues(FieldCase):
    def values(self, robot_id):
        from rescue_bringup.field_launch import frozen_localization_values
        return frozen_localization_values(self.deployment, field=self.field(frozen_field), robot_id=robot_id)

    def test_tb5_gets_the_node_parameters(self):
        v = self.values('robot_b')['start_align']
        for key, value in CONFIG.items():
            self.assertEqual(value, v[key], key)
        self.assertEqual({'map_topic': '/robot5/map', 'scan_topic': '/robot5/scan', 'odom_topic': '/robot5/odom',
                          'dock_topic': '/robot5/dock_status', 'amcl_pose_topic': '/robot5/amcl_pose',
                          'set_initial_pose_service': '/robot5/set_initial_pose',
                          'nomotion_update_service': '/robot5/request_nomotion_update',
                          'status_topic': '/robot5/rescue_start_align', 'base_frame': 'base_link'},
                         {k: v[k] for k in ('map_topic', 'scan_topic', 'odom_topic', 'dock_topic', 'amcl_pose_topic',
                                            'set_initial_pose_service', 'nomotion_update_service', 'status_topic',
                                            'base_frame')})

    def test_the_executor_reads_the_same_verdict_topic(self):
        status = self.values('robot_b')['start_align']['status_topic']
        b = self.robot(field=self.field(frozen_field), robot_id='robot_b', occupancy_sha256='')['task_input']
        self.assertEqual(status, b['task.saved_map.start_align_topic'])
        a = self.robot(field=self.field(frozen_field), occupancy_sha256='')['task_input']
        self.assertNotIn('task.saved_map.start_align_topic', a)

    def test_tb6_has_no_start_alignment(self):
        self.assertIsNone(self.values('robot_a')['start_align'])


@unittest.skipUnless(LAUNCH.exists(), 'installed-wheel checkout has no src/ launch files')
class Launch(unittest.TestCase):
    def test_the_node_runs_beside_amcl_with_the_robot_tf(self):
        text = LAUNCH.read_text()
        group = text[text.index('GroupAction(['):]
        self.assertIn("executable='start_align'", group)              # inside the /tf -> <ns>/tf remap
        self.assertIn("name='rescue_start_align'", group)
        self.assertIn("values['start_align']", text)

    def test_entry_point(self):
        setup = (REPO / 'src' / 'rescue_robot' / 'setup.py').read_text()
        self.assertIn('"start_align = rescue_robot.start_align:main"', setup)


if __name__ == '__main__':
    unittest.main()
