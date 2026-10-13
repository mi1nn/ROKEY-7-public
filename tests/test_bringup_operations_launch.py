"""operations PC launch: deployment profile v2.1 -> db_writer and report_node parameters (E2, no ROS).

The launch file (src/rescue_bringup/launch/operations.launch.py) only wraps this. Topic
names must be the ones the coordinator uses, so this checks them against its constants.
"""
from pathlib import Path
import unittest

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
