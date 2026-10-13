"""operations PC launch: deployment profile v2.1 -> db_writer and report_node parameters (E2, no ROS).

The launch file (src/rescue_bringup/launch/operations.launch.py) only wraps this. Topic
names must be the ones the coordinator uses, so this checks them against its constants.
"""
from pathlib import Path
import importlib.util
import sys
import tempfile
import unittest
from unittest.mock import patch

from rescue_bringup.profile import load_deployment

REPO = Path(__file__).resolve().parents[1]
PROFILE = REPO / 'config' / 'deployment.v2.example.json'
V1_PROFILE = REPO / 'config' / 'deployment.example.json'


def build(**changes):
    from rescue_bringup.operations_launch import db_writer_parameters
    values = dict(db_path='/var/lib/rescue/site_state.sqlite3', api_port=8090, command_timeout_s=3.0)
    return db_writer_parameters(load_deployment(PROFILE), **(values | changes))


class DbWriterParameters(unittest.TestCase):
    def test_profile_robots_roles_and_topics_become_the_node_parameters(self):
        self.assertEqual({
            'db_path': '/var/lib/rescue/site_state.sqlite3',
            'allowed_robot_ids': ['robot_a', 'robot_b'],
            'robot_roles': ['SCOUT', 'SUPPORT'],
            'status_topics': ['/rescue/robots/robot_a/status', '/rescue/robots/robot_b/status'],
            'pose_topics': ['/rescue/robots/robot_a/pose', '/rescue/robots/robot_b/pose'],
            'site_events_topic': '/rescue/site_events',
            'system_state_topic': '/rescue/system_state',
            'targets_topic': '/rescue/targets',
            'api_host': '127.0.0.1',
            'api_port': 8090,
            'operator_command_service': '/rescue/operator_command',
            'command_timeout_s': 3.0,
        }, build())

    def test_every_parameter_db_writer_requires_is_given_and_accepted(self):
        """The node refuses to start on a missing or inconsistent parameter (AGENTS 8절)."""
        from rescue_operations.ros_node import node_config
        config = node_config(build())
        self.assertEqual({'robot_a': 'SCOUT', 'robot_b': 'SUPPORT'}, config['roles'])

    def test_topics_are_the_coordinators(self):
        from rescue_coordination import ros_node as coordinator
        params = build()
        self.assertEqual((coordinator.SITE_EVENT_TOPIC, coordinator.SYSTEM_STATE_TOPIC, coordinator.TARGETS_TOPIC,
                          coordinator.OPERATOR_COMMAND_SERVICE),
                         (params['site_events_topic'], params['system_state_topic'], params['targets_topic'],
                          params['operator_command_service']))
        self.assertTrue(all(t.startswith(coordinator.ROBOT_PREFIX) for t in params['status_topics']))

    def test_api_listens_on_loopback_only(self):
        """The API has no login of its own; the console on the same PC does that (#125)."""
        self.assertEqual('127.0.0.1', build()['api_host'])

    def test_bad_launch_arguments_are_refused(self):
        from rescue_bringup.operations_launch import LaunchSettingError
        for change in (dict(db_path=''), dict(api_port=0), dict(api_port=70000), dict(api_port='8090'),
                       dict(command_timeout_s=0.0), dict(command_timeout_s=5.0)):
            with self.subTest(change=change), self.assertRaises(LaunchSettingError):
                build(**change)

    def test_a_profile_without_mission_roles_is_refused(self):
        from rescue_bringup.operations_launch import LaunchSettingError, db_writer_parameters
        with self.assertRaises(LaunchSettingError):
            db_writer_parameters(load_deployment(V1_PROFILE), db_path='/x.sqlite3', api_port=8090,
                                 command_timeout_s=3.0)


def build_report(**changes):
    from rescue_bringup.operations_launch import report_node_parameters
    values = dict(db_path='/var/lib/rescue/site_state.sqlite3', report_db_path='/var/lib/rescue/reports.sqlite3',
                  envelope_schema_version='0.1', publisher_id='report-ops')
    return report_node_parameters(**(values | changes))


class ReportNodeParameters(unittest.TestCase):
    """#137 report_node: coordinator RescueReport -> ReportReceipt, on the operations PC."""

    def test_launch_values_become_every_report_node_parameter(self):
        from rescue_operations.report_node import PARAMS
        params = build_report()
        self.assertEqual(set(PARAMS), set(params))
        self.assertEqual({
            'db_path': '/var/lib/rescue/reports.sqlite3',
            'reports_topic': '/rescue/reports',
            'receipts_topic': '/rescue/report_receipts',
            'envelope_schema_version': '0.1',
            'publisher_id': 'report-ops',
        }, params)

    def test_topics_are_the_coordinators(self):
        from rescue_coordination import ros_node as coordinator
        params = build_report()
        self.assertEqual((coordinator.REPORT_TOPIC, coordinator.REPORT_RECEIPT_TOPIC),
                         (params['reports_topic'], params['receipts_topic']))

    def test_values_stay_text(self):
        """A number-like text such as '0.1' must reach the STRING parameter as text."""
        params = build_report()
        self.assertTrue(all(isinstance(value, str) for value in params.values()))

    def test_report_db_must_be_another_file_than_the_site_state_db(self):
        """One writing process per SQLite file (#137)."""
        from rescue_bringup.operations_launch import LaunchSettingError
        for change in (dict(report_db_path='/var/lib/rescue/site_state.sqlite3'),
                       dict(report_db_path='/var/lib/rescue/../rescue/site_state.sqlite3'),
                       dict(report_db_path=''), dict(envelope_schema_version=''), dict(publisher_id='')):
            with self.subTest(change=change), self.assertRaises(LaunchSettingError):
                build_report(**change)


WEBCAM_SETTINGS = {'debug_image_topic': '/rescue/cameras/cam_fixed/debug_image/compressed',
                   'camera_id': 'cam_fixed'}


def relay(**changes):
    from rescue_bringup.operations_launch import mjpeg_relay_parameters
    values = dict(webcam=dict(WEBCAM_SETTINGS), bind_host='127.0.0.1', port='8091', stale_s='2.0')
    return mjpeg_relay_parameters(**(values | changes))


class MjpegRelayParameters(unittest.TestCase):
    """#153 mjpeg_relay: the webcam judge video as MJPEG for the console, on with the webcam only."""

    def test_webcam_settings_and_launch_values_become_every_relay_parameter(self):
        from rescue_perception.mjpeg_relay import PARAMS
        params = relay()
        self.assertEqual(set(PARAMS), set(params))
        self.assertEqual({'image_topic': '/rescue/cameras/cam_fixed/debug_image/compressed',
                          'bind_host': '127.0.0.1', 'port': 8091, 'path': '/cam_fixed.mjpg', 'stale_s': 2.0}, params)

    def test_console_camera_argument(self):
        from rescue_bringup.operations_launch import console_camera_argument
        self.assertEqual('cam_fixed=http://127.0.0.1:8091/cam_fixed.mjpg',
                         console_camera_argument(relay(), camera_id='cam_fixed'))

    def test_parameter_checks_run_without_yaml(self):
        with patch.dict(sys.modules, {'yaml': None}):
            self.assertEqual(8091, relay()['port'])

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'PyYAML is a ROS launch dependency')
    def test_malformed_yaml_is_a_launch_setting_error(self):
        from rescue_bringup.operations_launch import LaunchSettingError, read_webcam_parameters
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'broken.yaml'
            path.write_text('webcam_node: [unterminated')
            with self.assertRaises(LaunchSettingError):
                read_webcam_parameters(path)

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'PyYAML is a ROS launch dependency')
    def test_packaged_webcam_yaml_matches_the_parameter_fixture(self):
        from importlib.resources import files
        from rescue_bringup.operations_launch import read_webcam_parameters
        example = files('rescue_perception').joinpath('config/webcam_node.example.yaml')
        values = read_webcam_parameters(example)
        self.assertEqual(WEBCAM_SETTINGS, {key: values[key] for key in WEBCAM_SETTINGS})

    def test_bad_values_are_refused(self):
        from rescue_bringup.operations_launch import LaunchSettingError
        webcam = dict(WEBCAM_SETTINGS)
        for change in (dict(bind_host=''), dict(port='0'), dict(port='70000'), dict(port='80.5'),
                       dict(stale_s='0'), dict(stale_s='nan'), dict(stale_s=''),
                       dict(webcam=webcam | {'debug_image_topic': ''}), dict(webcam=webcam | {'camera_id': ''}),
                       dict(webcam={k: v for k, v in webcam.items() if k != 'debug_image_topic'})):
            with self.subTest(change={k: v for k, v in change.items() if k != 'webcam'} or 'webcam'), \
                    self.assertRaises(LaunchSettingError):
                relay(**change)

    def test_relay_needs_the_webcam_on_this_pc(self):
        """video_port without webcam_params_file: nothing to relay, refuse instead of an empty stream."""
        from rescue_bringup.operations_launch import LaunchSettingError, relay_requested
        self.assertFalse(relay_requested(video_port='', webcam_params_file=''))
        self.assertFalse(relay_requested(video_port='', webcam_params_file='/w.yaml'))
        self.assertTrue(relay_requested(video_port='8091', webcam_params_file='/w.yaml'))
        with self.assertRaises(LaunchSettingError):
            relay_requested(video_port='8091', webcam_params_file='')

    def test_unreadable_webcam_file_is_refused(self):
        from rescue_bringup.operations_launch import LaunchSettingError, read_webcam_parameters
        with self.assertRaises(LaunchSettingError):
            read_webcam_parameters(REPO / 'absent.yaml')


class MapSnapshotParameters(unittest.TestCase):
    """map_topic/map_dir: the SLAM map relay for the console --live-map (owner 2026-10-13: TB-6 SLAM)."""

    def test_off_without_both(self):
        from rescue_bringup.operations_launch import map_snapshot_parameters
        self.assertIsNone(map_snapshot_parameters(map_topic='', map_dir=''))

    def test_on_with_both_and_the_console_value(self):
        from rescue_bringup.operations_launch import map_snapshot_parameters
        self.assertEqual({'map_topic': '/robot6/map', 'out_dir': '/srv/live'},
                         map_snapshot_parameters(map_topic='/robot6/map', map_dir='/srv/live'))

    def test_one_without_the_other_is_refused(self):
        from rescue_bringup.operations_launch import LaunchSettingError, map_snapshot_parameters
        for bad in (dict(map_topic='/robot6/map', map_dir=''), dict(map_topic='', map_dir='/srv/live'),
                    dict(map_topic='robot6/map', map_dir='/srv/live')):
            with self.subTest(bad=bad), self.assertRaises(LaunchSettingError):
                map_snapshot_parameters(**bad)


class WebcamDevice(unittest.TestCase):
    """webcam_device:= picks which plugged camera the webcam_node opens, without editing its YAML."""

    def setUp(self):
        self.by_id = Path(tempfile.mkdtemp())
        for name in ('usb-HP_Wide_Vision-video-index0', 'usb-HP_Wide_Vision-video-index1',
                     'usb-Web_Camera-video-index0'):
            (self.by_id / name).touch()

    def test_cameras_are_the_capture_nodes_only(self):
        """index1 is the metadata node of the same camera: it cannot be opened for frames."""
        from rescue_bringup.operations_launch import available_cameras
        self.assertEqual(['usb-HP_Wide_Vision-video-index0', 'usb-Web_Camera-video-index0'],
                         available_cameras(self.by_id))
        self.assertEqual([], available_cameras(self.by_id / 'absent'))

    def test_empty_keeps_the_yaml_device(self):
        from rescue_bringup.operations_launch import webcam_device
        self.assertEqual('', webcam_device('', by_id_dir=self.by_id))

    def test_a_name_or_a_path_selects_the_camera(self):
        from rescue_bringup.operations_launch import webcam_device
        path = str(self.by_id / 'usb-Web_Camera-video-index0')
        self.assertEqual(path, webcam_device('usb-Web_Camera-video-index0', by_id_dir=self.by_id))
        self.assertEqual(path, webcam_device(path, by_id_dir=self.by_id))

    def test_a_missing_camera_is_refused_with_the_plugged_ones(self):
        from rescue_bringup.operations_launch import LaunchSettingError, webcam_device
        with self.assertRaises(LaunchSettingError) as caught:
            webcam_device('usb-Unplugged-video-index0', by_id_dir=self.by_id)
        self.assertIn('usb-Web_Camera-video-index0', str(caught.exception))
        self.assertIn('usb-HP_Wide_Vision-video-index0', str(caught.exception))


class LaunchArguments(unittest.TestCase):
    def test_text_launch_arguments_are_parsed_strictly(self):
        from rescue_bringup.operations_launch import LaunchSettingError, parse_launch_arguments
        self.assertEqual(dict(db_path='/x.sqlite3', api_port=8090, command_timeout_s=3.0),
                         parse_launch_arguments(db_path='/x.sqlite3', api_port='8090', command_timeout_s='3.0'))
        for bad in (dict(api_port='80.5'), dict(api_port='port'), dict(command_timeout_s='nan'),
                    dict(command_timeout_s='')):
            with self.subTest(bad=bad), self.assertRaises(LaunchSettingError):
                parse_launch_arguments(**(dict(db_path='/x.sqlite3', api_port='8090', command_timeout_s='3.0') | bad))


if __name__ == '__main__':
    unittest.main()
