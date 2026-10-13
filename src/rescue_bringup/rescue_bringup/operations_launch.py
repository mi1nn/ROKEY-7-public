"""operations PC launch values: deployment profile v2.1 -> db_writer and report_node parameters (no ROS import).

``launch/operations.launch.py`` loads the profile, parses its text launch arguments with
``parse_launch_arguments`` and hands ``db_writer_parameters`` to the node. Robot IDs and
roles come from the profile (mission roles), so a field run with other robots changes the
profile file only. Topic and service names are the coordinator's (rescue_coordination
ros_node constants); a test keeps them equal without importing it here.
"""
import math
import os

from .profile import Deployment

SITE_EVENTS_TOPIC = '/rescue/site_events'
SYSTEM_STATE_TOPIC = '/rescue/system_state'
TARGETS_TOPIC = '/rescue/targets'
OPERATOR_COMMAND_SERVICE = '/rescue/operator_command'
REPORTS_TOPIC = '/rescue/reports'  # IF-072, coordinator -> report_node
REPORT_RECEIPTS_TOPIC = '/rescue/report_receipts'  # IF-073, report_node -> coordinator
ROBOT_PREFIX = '/rescue/robots/'
# The command API has no login of its own; the console on the same PC checks ADMIN (#125).
API_HOST = '127.0.0.1'
CONSOLE_TIMEOUT_S = 5.0  # the console gives up after this (#115); the API must answer first


class LaunchSettingError(ValueError):
    """A launch argument or profile that cannot start the operations PC."""


def parse_launch_arguments(*, db_path, api_port, command_timeout_s):
    """Launch arguments arrive as text; parse them strictly (no silent defaults)."""
    try:
        port = int(api_port)
        timeout = float(command_timeout_s)
    except ValueError as exc:
        raise LaunchSettingError(f'api_port must be an integer and command_timeout_s a number: {exc}') from exc
    if not math.isfinite(timeout):
        raise LaunchSettingError('command_timeout_s must be finite')
    return dict(db_path=db_path, api_port=port, command_timeout_s=timeout)


def db_writer_parameters(deployment, *, db_path, api_port, command_timeout_s):
    """Profile + launch values -> every parameter rescue_operations db_writer requires."""
    if not isinstance(deployment, Deployment) or deployment.mission is None:
        raise LaunchSettingError('profile v2 with mission roles is required (config/deployment.v2.example.json)')
    if not isinstance(db_path, str) or not db_path:
        raise LaunchSettingError('db_path is required')
    if type(api_port) is not int or not 1 <= api_port <= 65535:
        raise LaunchSettingError('api_port must be 1..65535')
    if not isinstance(command_timeout_s, float) or not 0.0 < command_timeout_s < CONSOLE_TIMEOUT_S:
        raise LaunchSettingError(f'command_timeout_s must be over 0 and under the console {CONSOLE_TIMEOUT_S:g} s')
    robots = [robot.id for robot in deployment.robots]
    return {
        'db_path': db_path,
        'allowed_robot_ids': robots,
        'robot_roles': [deployment.mission.role_of(robot) for robot in robots],
        'status_topics': [ROBOT_PREFIX + robot + '/status' for robot in robots],
        'pose_topics': [ROBOT_PREFIX + robot + '/pose' for robot in robots],
        'site_events_topic': SITE_EVENTS_TOPIC,
        'system_state_topic': SYSTEM_STATE_TOPIC,
        'targets_topic': TARGETS_TOPIC,
        'api_host': API_HOST,
        'api_port': api_port,
        'operator_command_service': OPERATOR_COMMAND_SERVICE,
        'command_timeout_s': command_timeout_s,
    }


def report_node_parameters(*, db_path, report_db_path, envelope_schema_version, publisher_id):
    """Launch values -> every parameter rescue_operations report_node (#137) requires.

    The report store is its own SQLite file: one writing process per file, so it must not
    be the db_writer site-state file. Every value is text (STRING parameters).
    """
    for name, value in (('report_db_path', report_db_path), ('envelope_schema_version', envelope_schema_version),
                        ('publisher_id', publisher_id)):
        if not isinstance(value, str) or not value:
            raise LaunchSettingError(f'{name} is required')
    if os.path.realpath(report_db_path) == os.path.realpath(db_path):
        raise LaunchSettingError('report_db_path must be another file than the site-state db_path')
    return {
        'db_path': report_db_path,
        'reports_topic': REPORTS_TOPIC,
        'receipts_topic': REPORT_RECEIPTS_TOPIC,
        'envelope_schema_version': envelope_schema_version,
        'publisher_id': publisher_id,
    }
