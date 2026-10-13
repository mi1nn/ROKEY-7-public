"""Coordinator and robot launch values: profile + field run file + explicit arguments (no ROS).

The launch files (src/rescue_bringup/launch/coordinator.launch.py, robot.launch.py) only wrap
rescue_bringup.field_launch. The field run file config/field/search_area.field.json carries the
#132 field values; these tests keep the generated parameters equal to the #132 field scripts
(scripts/field/search-area/start_*.sh), which moved a robot on 2026-10-12.
"""
import ast
import json
from pathlib import Path
import re
import runpy
import subprocess
import sys
import tempfile
import unittest

from rescue_bringup.profile import load_deployment

REPO = Path(__file__).resolve().parents[1]
FIELD = REPO / 'config' / 'field' / 'search_area.field.json'
PROFILE = REPO / 'config' / 'deployment.v2.example.json'
SCRIPTS = REPO / 'scripts' / 'field' / 'search-area'
SHA = 'ab' * 32
NOW_NS = 1_800_000_000_000_000_000
PLACEHOLDERS = dict(route_segment_goals='[0.0, 0.0, 0.0]', observe_goal='[0.0, 0.0, 0.0]',
                    route_segment_passage_free='[false]')


def script_parameters(name):
    """`-p name:=value` of a #132 field script; shell variables are left out (set at run time)."""
    text = (SCRIPTS / name).read_text()
    values = {}
    for raw in re.findall(r"-p (\"[^\"]*\"|'[^']*'|\S+)", text):
        if raw[0] in '"\'':
            raw = raw[1:-1]  # the outer shell quotes only
        key, value = raw.split(':=', 1)
        if '$' not in value:
            try:
                values[key] = json.loads(value)
            except json.JSONDecodeError:
                try:
                    values[key] = ast.literal_eval(value)
                except (ValueError, SyntaxError):
                    values[key] = value
    return values


def field_profile(directory, *, entry_yaws=(0.0, 0.0)):
    """The example profile with the field namespaces (robot_a=/robot6, robot_b=/robot5, #134)
    and the readiness values of #132 start_robot.sh."""
    profile = json.loads(PROFILE.read_text())
    # #132 field map basis: frame map (the example profile's frame is 'mission').
    profile['map_basis']['frame_id'] = 'map'
    for area, yaw in zip(profile['mission']['search_areas'], entry_yaws):
        if yaw is None:
            area.pop('entry_yaw', None)
        else:
            area['entry_yaw'] = yaw
    (directory / 'robots').mkdir()
    for robot, ns in zip(profile['robots'], ('/robot6', '/robot5')):
        robot['namespace'] = ns
        robot['parameters_file'] = f'robots/{robot["id"]}.json'
        params = {
            'schema_version': 2, 'robot_id': robot['id'], 'namespace': ns, 'runtime_enabled': False,
            'use_sim_time': False, 'nav_action': ns + '/navigate_to_pose', 'odom_topic': ns + '/odom',
            'dock_status_topic': ns + '/dock_status',
            'readiness': {'version': 'search-live-20261012',
                          'max_age_s': {'odom': 1.0, 'scan': 2.0, 'camera': 2.0, 'wheel_status': 2.0,
                                        'map_to_odom_tf': 2.0},
                          'scan_topic': ns + '/scan', 'tf_topic': ns + '/tf', 'map_frame': 'map',
                          'odom_frame': 'odom', 'camera_topics': [ns + '/oakd/rgb/image_raw/compressed'],
                          'wheel_status_topic': '', 'watchdog_heartbeat_topic': ''}}
        (directory / robot['parameters_file']).write_text(json.dumps(params))
    path = directory / 'profile.json'
    path.write_text(json.dumps(profile))
    return path


class DependencyTests(unittest.TestCase):
    def test_installed_input_copy_keeps_field_parity_fixtures_without_package_sources(self):
        wheel_check = REPO / 'scripts' / 'check_wheels.py'
        if not wheel_check.is_file():
            self.skipTest('wheel builder is not part of the installed test checkout')
        copy_inputs = runpy.run_path(str(wheel_check))['copy_test_inputs']
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / 'check'
            copy_inputs(target)
            self.assertFalse((target / 'src').exists())
            for name in ('start_coordinator.sh', 'start_robot.sh', 'start_navigation.sh',
                         'nav2_watchdog.launch.py'):
                copied = target / 'scripts' / 'field' / 'search-area' / name
                self.assertEqual((SCRIPTS / name).read_bytes(), copied.read_bytes())

    def test_suite_imports_without_pyyaml(self):
        code = """
import importlib.abc
from pathlib import Path
import sys
root = Path(sys.argv[1])
for package in sorted((root / 'src').glob('rescue_*')):
    sys.path.insert(0, str(package))
sys.path.insert(0, str(root / 'tests'))
class NoYaml(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'yaml' or fullname.startswith('yaml.'):
            raise ModuleNotFoundError('PyYAML unavailable in the stdlib CI environment')
sys.meta_path.insert(0, NoYaml())
import test_bringup_field_launch
"""
        result = subprocess.run([sys.executable, '-I', '-c', code, str(REPO)],
                                capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)


class FieldCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.profile_path = field_profile(self.dir)
        self.deployment = load_deployment(self.profile_path)

    def field(self, change=None):
        from rescue_bringup.field_launch import load_field
        raw = json.loads(FIELD.read_text())
        if change:
            change(raw)
        path = self.dir / 'field.json'
        path.write_text(json.dumps(raw))
        return load_field(path)

    def coordinator(self, field=None, **changes):
        from rescue_bringup.field_launch import coordinator_parameters
        return coordinator_parameters(self.deployment, profile_path=str(self.profile_path),
                                      field=field or self.field(), **(PLACEHOLDERS | changes))

    def robot(self, field=None, **changes):
        from rescue_bringup.field_launch import robot_launch_values
        values = dict(robot_id='robot_a', occupancy_sha256=SHA, now_ns=NOW_NS) | changes
        return robot_launch_values(self.deployment, field=field or self.field(), **values)


class CoordinatorParameters(FieldCase):
    def test_equal_to_the_132_field_script(self):
        expected = script_parameters('start_coordinator.sh')
        expected['profile_path'] = str(self.profile_path)
        expected['camera_watch_timeout_s'] = 2.0  # added after #132 (SetCameraWatch, coordination C2)
        # Added after #132: TB-5 automatic dispatch (user decision 1B, 2026-10-13)
        expected.update(support_dispatch='AUTO', approach_standoff_m=0.8, support_pose_max_age_s=2.0)
        # Added after #132: TargetLocation time checks (coordination OPS-103, 2026-10-13)
        expected.update(target_clock_domain_id='rokey7-utc-field', target_max_age_s=3.0, target_max_future_s=0.05)
        # Added after #132: TB-5 waits at the target before leaving for the exit (user decision 2026-10-13: 15 s)
        expected['support_exit_wait_s'] = 15.0
        # Added after #132: recheck -> verdict clock check (coordination, from measured clock bounds)
        expected['recheck_max_clock_error_s'] = 2.0
        # Added after #132: SLAM map frozen before TB-5 goes (owner decision 2026-10-13, Q-29 ⑦; 작전 OPS-119)
        expected.update(map_mode='SLAM_FREEZE', freeze_clearance_m=0.2, map_freeze_timeout_s=10.0)
        self.assertEqual(expected, self.coordinator())

    def test_one_passage_flag_per_segment_end(self):
        """A length other than the segment count holds every segment (PASSAGE_ASSESSMENT_REQUIRED) at run time;
        it is never what the operator meant, so the launch refuses it before start."""
        from rescue_bringup.field_launch import FieldLaunchError
        two = '[1.0, 2.0, 0.0, 3.0, 4.0, 0.0]'
        self.assertEqual([True, False], self.coordinator(route_segment_goals=two,
                                                         route_segment_passage_free='[true, false]')
                         ['route_segment_passage_free'])
        for flags in ('[true]', '[true, false, false]'):
            with self.subTest(flags=flags), self.assertRaisesRegex(FieldLaunchError, 'one true/false per segment'):
                self.coordinator(route_segment_goals=two, route_segment_passage_free=flags)

    def test_camera_watch_roi_is_an_optional_site_argument(self):
        """Unset by default: the coordinator then refuses camera watch requests (no whole-frame default)."""
        from rescue_bringup.field_launch import FieldLaunchError
        self.assertNotIn('camera_watch_roi', self.coordinator())
        self.assertEqual([0.1, 0.2, 0.5, 0.9], self.coordinator(camera_watch_roi='[0.1, 0.2, 0.5, 0.9]')['camera_watch_roi'])
        for bad in ('[0.1, 0.2, 0.5]', '[]', 'all', '[0, 0, 1, nan]'):
            with self.subTest(bad=bad), self.assertRaises(FieldLaunchError):
                self.coordinator(camera_watch_roi=bad)

    def test_every_parameter_the_coordinator_requires(self):
        from rescue_coordination.ros_node import PARAMETERS
        self.assertTrue(set(PARAMETERS) <= set(self.coordinator()))

    def test_entry_yaws_come_from_the_profile_in_area_order(self):
        from rescue_bringup.field_launch import coordinator_parameters
        (self.dir / 'other').mkdir()
        path = field_profile(self.dir / 'other', entry_yaws=(0.25, -1.5))
        params = coordinator_parameters(load_deployment(path), profile_path=str(path), field=self.field(),
                                        **PLACEHOLDERS)
        self.assertEqual([0.25, -1.5], params['entry_yaws'])

    def test_a_search_area_without_entry_yaw_is_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError, coordinator_parameters
        (self.dir / 'other').mkdir()
        path = field_profile(self.dir / 'other', entry_yaws=(0.0, None))
        with self.assertRaises(FieldLaunchError):
            coordinator_parameters(load_deployment(path), profile_path=str(path), field=self.field(),
                                   **PLACEHOLDERS)

    def test_placeholder_goals_are_explicit_arguments(self):
        """route_segment_goals / observe_goal / route_segment_passage_free have no default (#132 placeholders)."""
        from rescue_bringup.field_launch import FieldLaunchError
        params = self.coordinator(route_segment_goals='[1.0, 2.0, 0.5, 3.0, 4.0, -0.5]',
                                  route_segment_passage_free='[true, false]', observe_goal='[5, 6, 0]')
        self.assertEqual([1.0, 2.0, 0.5, 3.0, 4.0, -0.5], params['route_segment_goals'])
        self.assertEqual([True, False], params['route_segment_passage_free'])
        self.assertEqual([5.0, 6.0, 0.0], params['observe_goal'])
        self.assertTrue(all(type(v) is float for v in params['observe_goal']))
        for bad in (dict(route_segment_goals=''), dict(route_segment_goals='[0.0, 0.0]'),
                    dict(route_segment_goals='[]'), dict(observe_goal='[0.0, 0.0, 0.0, 1.0]'),
                    dict(observe_goal='[0, 0, nan]'), dict(observe_goal='0,0,0'),
                    dict(route_segment_passage_free='[0]'), dict(route_segment_passage_free='false'),
                    dict(route_segment_passage_free='[]'), dict(route_segment_passage_free='[false, false]')):
            with self.subTest(bad=bad), self.assertRaises(FieldLaunchError):
                self.coordinator(**bad)


class RobotParameters(FieldCase):
    def test_task_input_equal_to_the_132_field_script(self):
        expected = script_parameters('start_robot.sh')
        expected.update({'task.saved_map.occupancy_sha256': SHA,
                         'task.saved_map.valid_from_ns': NOW_NS - 3_000_000_000,
                         'task.saved_map.transform_valid_until_ns': NOW_NS + 3_600_000_000_000})
        # Added after #132: automatic target move policy per robot (C1, 2026-10-13). TB-6 (robot_a,
        # SCOUT) takes only REOBSERVE_TARGET (C1-b) and asks the recheck server after the stop.
        expected.update({'task.live.auto_target.enabled': True,
                         'task.live.auto_target.kinds': ['REOBSERVE_TARGET', 'RETURN_HOME'],
                         'task.live.recheck.window_ns': 3_000_000_000,
                         'task.live.recheck.deadline_ns': 10_000_000_000,
                         'task.live.auto_target.goal_clear_radius_m': 0.2,
                         'task.live.auto_target.policy_version': 'auto-target-20261013'})
        # Added after #132: RobotPose validity (pose publisher, 2026-10-13)
        expected['task.live.pose_valid_ns'] = 2_000_000_000
        # The robot Pi clock (sensor host of the AMCL stamps), per robot (2026-10-13)
        expected['task.live.sensor_host_clock_domain_id'] = 'rokey7-utc-field'
        # TB-6 Pi measured 2026-10-13 (chronyc tracking, about 109 ms by the parse_chrony bound) -> 150 ms
        expected['task.live.sensor_host_clock_error_bound_ns'] = 150_000_000
        values = self.robot()
        self.assertEqual('/robot6', values['namespace'])
        self.assertEqual(expected, values['task_input'])

    def test_watchdog_equal_to_the_132_field_launch(self):
        values = self.robot()
        nav = script_parameters('start_navigation.sh')  # none: start_navigation passes launch arguments
        self.assertEqual({}, nav)
        self.assertEqual({
            'namespace': '/robot6',
            'cmd_vel_in_topic': '/robot6/cmd_vel_watchdog_in', 'cmd_vel_out_topic': '/robot6/cmd_vel',
            'cmd_vel_stamped': True,
            'executor_heartbeat_topic': '/robot6/rescue/executor_heartbeat',
            'watchdog_heartbeat_topic': '/robot6/rescue/watchdog_heartbeat',
            'step_period': 0.05, 'watchdog.version': 'search-live-20261012',
            'watchdog.heartbeat_max_age_s': 0.5, 'watchdog.command_max_age_s': 0.5,
            'watchdog.max_linear_mps': 0.3, 'watchdog.max_angular_rps': 1.2,
        }, values['watchdog'])
        self.assertEqual([('collision_monitor:/robot6/cmd_vel', '/robot6/cmd_vel_watchdog_in'),
                          ('docking_server:/robot6/cmd_vel', '/robot6/cmd_vel_watchdog_in')],
                         values['nav2_remaps'])

    def test_heartbeat_topics_match_between_task_input_and_watchdog(self):
        values = self.robot()
        for name in ('executor_heartbeat_topic', 'watchdog_heartbeat_topic'):
            self.assertEqual(values['task_input'][name], values['watchdog'][name])

    def test_tb5_values_in_the_repository_field_file(self):
        """TB-5 (robot_b, SUPPORT): its own publisher, the same mapping authority as TB-6, approach and return
        only, Pi clock measured (150 ms). Decided 2026-10-13; #132 had no robot_b values."""
        b, a = self.robot(robot_id='robot_b')['task_input'], self.robot()['task_input']
        self.assertEqual('/robot5/navigate_to_pose', b['nav_action'])
        self.assertNotEqual(a['task.publisher_id'], b['task.publisher_id'])
        self.assertEqual(a['task.mapping_authority'], b['task.mapping_authority'])
        self.assertTrue(b['task.live.auto_target.enabled'])
        self.assertEqual(['APPROACH_TARGET', 'RETURN_HOME'], b['task.live.auto_target.kinds'])
        # TB-5 Pi measured 2026-10-13 (chronyc tracking, about 107 ms by the parse_chrony bound) -> 150 ms
        self.assertEqual(150_000_000, b['task.live.sensor_host_clock_error_bound_ns'])

    def test_robot_b_needs_its_own_identity_in_the_field_file(self):
        """No publisher_id / mapping_authority is made up for a robot the field file does not name."""
        from rescue_bringup.field_launch import FieldLaunchError
        with self.assertRaisesRegex(FieldLaunchError, 'robot_b'):
            self.robot(field=self.field(lambda raw: raw['robots'].pop('robot_b')), robot_id='robot_b')

        def add_b(raw):
            raw['robots']['robot_b'] = {'task.publisher_id': 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa',
                                        'task.mapping_authority': 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb',
                                        'task.live.auto_target.enabled': True,
                                        'task.live.auto_target.kinds': ['APPROACH_TARGET'],
                                        'task.live.auto_target.goal_clear_radius_m': 0.2,
                                        'task.live.auto_target.policy_version': 'p',
                                        'task.live.sensor_host_clock_domain_id': 'rokey7-utc-field',
                                        'task.live.sensor_host_clock_error_bound_ns': 9_000_000}
        values = self.robot(field=self.field(add_b), robot_id='robot_b')
        # Both policy and sensor clock must stay local to this robot.
        self.assertTrue(values['task_input']['task.live.auto_target.enabled'])
        self.assertEqual(['REOBSERVE_TARGET', 'RETURN_HOME'], self.robot()['task_input']['task.live.auto_target.kinds'])
        self.assertEqual(['APPROACH_TARGET'], values['task_input']['task.live.auto_target.kinds'])
        # Each robot Pi has its own measured clock bound (decision 2026-10-13)
        self.assertEqual(9_000_000, values['task_input']['task.live.sensor_host_clock_error_bound_ns'])
        self.assertEqual(150_000_000, self.robot()['task_input']['task.live.sensor_host_clock_error_bound_ns'])
        self.assertEqual('/robot5', values['namespace'])
        self.assertEqual('/robot5/navigate_to_pose', values['task_input']['nav_action'])
        self.assertEqual('/robot5/map', values['task_input']['task.saved_map.map_topic'])
        self.assertEqual('robot_b', values['task_input']['robot_id'])

    def test_measured_map_hash_and_robot_are_required(self):
        from rescue_bringup.field_launch import FieldLaunchError
        for bad in (dict(occupancy_sha256=''), dict(occupancy_sha256='AB' * 32), dict(occupancy_sha256='ab' * 31),
                    dict(robot_id='robot_c'), dict(now_ns=0)):
            with self.subTest(bad=bad), self.assertRaises(FieldLaunchError):
                self.robot(**bad)


class MapBasisFromProfile(FieldCase):
    """Profile map_basis is the one source of the map basis (shared with the target locator launch)."""

    def reload(self, change):
        raw = json.loads(self.profile_path.read_text())
        change(raw)
        self.profile_path.write_text(json.dumps(raw))
        self.deployment = load_deployment(self.profile_path)

    def test_coordinator_goals_and_robot_alignment_use_the_profile_map_basis(self):
        def other_map(raw):
            raw['map_basis'] = dict(frame_id='map', map_session_id='33333333-3333-4333-8333-333333333333',
                                    map_revision=4, transform_revision=5)
            for area in raw['mission']['search_areas']:
                area['basis'].update(map_session_id='33333333-3333-4333-8333-333333333333', map_revision=4,
                                     transform_revision=5)
            raw['mission']['hold_positions']['robot_b'].update(
                map_session_id='33333333-3333-4333-8333-333333333333', map_revision=4)
        self.reload(other_map)
        coordinator, robot = self.coordinator(), self.robot()['task_input']
        self.assertEqual(('map', '33333333-3333-4333-8333-333333333333', 4, 5),
                         (coordinator['mission_frame_id'], coordinator['goal_map_session_id'],
                          coordinator['goal_map_revision'], coordinator['goal_transform_revision']))
        self.assertEqual(('map', '33333333-3333-4333-8333-333333333333', 4, 5),
                         (robot['task.mission_frame'], robot['task.saved_map.map_session_id'],
                          robot['task.saved_map.map_revision'], robot['task.saved_map.transform_revision']))

    def test_a_profile_without_map_basis_starts_nothing(self):
        from rescue_bringup.field_launch import FieldLaunchError
        self.reload(lambda raw: raw.pop('map_basis'))
        for build in (self.coordinator, self.robot):
            with self.subTest(build=build.__name__), self.assertRaisesRegex(FieldLaunchError, 'map_basis'):
                build()

    def test_empty_camera_topics_are_refused(self):
        """ROS cannot pass an empty array parameter; task_input would see it as missing."""
        from rescue_bringup.field_launch import FieldLaunchError
        robot_file = self.profile_path.parent / 'robots' / 'robot_a.json'
        raw = json.loads(robot_file.read_text())
        raw['readiness']['camera_topics'] = []
        robot_file.write_text(json.dumps(raw))
        self.deployment = load_deployment(self.profile_path)
        with self.assertRaisesRegex(FieldLaunchError, 'camera_topics'):
            self.robot()

    def test_robot_map_frame_must_be_the_map_basis_frame(self):
        from rescue_bringup.field_launch import FieldLaunchError
        self.reload(lambda raw: raw['map_basis'].update(frame_id='mission'))
        with self.assertRaisesRegex(FieldLaunchError, 'map_frame'):
            self.robot()


class HomePoses(FieldCase):
    """RETURN_HOME destinations from the profile home_positions (decision 5A, 2026-10-13)."""

    def with_homes(self, homes):
        raw = json.loads(self.profile_path.read_text())
        raw['home_positions'] = homes
        self.profile_path.write_text(json.dumps(raw))
        self.deployment = load_deployment(self.profile_path)

    def test_coordinator_gets_scout_then_support_home(self):
        self.with_homes({'robot_b': [3.0, -4.0, 0.0], 'robot_a': [1.0, 2.0, 0.5]})
        self.assertEqual([1.0, 2.0, 0.5, 3.0, -4.0, 0.0], self.coordinator()['home_poses'])

    def test_no_homes_passes_nothing(self):
        self.assertNotIn('home_poses', self.coordinator())
        self.assertNotIn('task.live.home_xy_yaw', self.robot()['task_input'])

    def test_one_home_only_is_refused_for_the_coordinator(self):
        from rescue_bringup.field_launch import FieldLaunchError
        self.with_homes({'robot_a': [1.0, 2.0, 0.5]})
        with self.assertRaisesRegex(FieldLaunchError, 'home_positions'):
            self.coordinator()

    def test_robot_gets_its_own_home(self):
        self.with_homes({'robot_b': [3.0, -4.0, 0.0], 'robot_a': [1.0, 2.0, 0.5]})
        self.assertEqual([1.0, 2.0, 0.5], self.robot()['task_input']['task.live.home_xy_yaw'])


class FieldFile(FieldCase):
    def test_values_that_must_agree_between_coordinator_and_robot_are_checked(self):
        from rescue_bringup.field_launch import FieldLaunchError
        changes = {
            'lease duration': lambda r: r['robot'].update({'task.lease.duration_s': 4.0}),
            'authority epoch': lambda r: r['robot'].update({'authority_epoch': 'x'}),
            'coordinator id': lambda r: r['robot'].update({'task.coordinator_id': 'x'}),
            'envelope': lambda r: r['robot'].update({'task.schema_version': 'x'}),
            'clock domain': lambda r: r['robot'].update({'task.lease.clock_domain_id': 'x'}),
            'clock backend': lambda r: r['robot'].update({'task.live.clock_backend': 'chrony'}),
            'executor policy': lambda r: r['robot'].update({'policy_version': 'x'}),
            'coordinator service': lambda r: r['robot'].update({'task.live.coordinator_service': '/x/task_inputs'}),
        }
        for label, change in changes.items():
            with self.subTest(label), self.assertRaises(FieldLaunchError):
                self.field(change)

    def test_missing_extra_and_mistyped_keys_are_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError
        changes = {
            'missing': lambda r: r['coordinator'].pop('tick_period_s'),
            'dispatch mode': lambda r: r['coordinator'].update({'support_dispatch': 'auto'}),
            'standoff not positive': lambda r: r['coordinator'].update({'approach_standoff_m': 0.0}),
            'pose age not positive': lambda r: r['coordinator'].update({'support_pose_max_age_s': -1.0}),
            'target age not positive': lambda r: r['coordinator'].update({'target_max_age_s': 0.0}),
            'target future negative': lambda r: r['coordinator'].update({'target_max_future_s': -0.01}),
            'target clock domain differs': lambda r: r['coordinator'].update({'target_clock_domain_id': 'other'}),
            'exit wait negative': lambda r: r['coordinator'].update({'support_exit_wait_s': -1.0}),
            'recheck clock error not positive': lambda r: r['coordinator'].update({'recheck_max_clock_error_s': 0.0}),
            'map mode unknown': lambda r: r['coordinator'].update({'map_mode': 'SLAM'}),
            'freeze timeout not positive': lambda r: r['coordinator'].update({'map_freeze_timeout_s': 0.0}),
            # The coordinator checks the approach point and homes with the radius the robot checks its goal with.
            'freeze clearance differs from a robot goal radius': lambda r: r['robots']['robot_b'].update(
                {'task.live.auto_target.goal_clear_radius_m': 0.3}),
            'auto target kinds not a list': lambda r: r['robots']['robot_a'].update(
                {'task.live.auto_target.kinds': 'APPROACH_TARGET'}),
            'auto target enabled not bool': lambda r: r['robots']['robot_a'].update({'task.live.auto_target.enabled': 1}),
            'auto target shared, not per robot': lambda r: r['robot'].update({'task.live.auto_target.enabled': True}),
            'pi clock shared, not per robot': lambda r: r['robot'].update(
                {'task.live.sensor_host_clock_error_bound_ns': 5}),
            'extra': lambda r: r['robot'].update({'policy.unknown': 1.0}),
            'int for double': lambda r: r['watchdog'].update({'watchdog.max_linear_mps': 1}),
            'double for int': lambda r: r['robot'].update({'task.poll_period_s': 1}),
            'bool for int': lambda r: r['coordinator'].update({'route_revision': True}),
            'nonfinite': lambda r: r['alignment'].update({'window_s': float('inf')}),
            'format': lambda r: r.update({'format': 'rescue-field-run/2'}),
            'live off': lambda r: r['coordinator'].update({'live_inputs.enabled': False}),
            'file input': lambda r: r['robot'].update({'task.input_mode': 'FILES'}),
        }
        for label, change in changes.items():
            with self.subTest(label), self.assertRaises(FieldLaunchError):
                self.field(change)

    def test_the_repository_field_file_loads(self):
        self.assertEqual('rescue-field-run/1', self.field().raw['format'])


if __name__ == '__main__':
    unittest.main()
