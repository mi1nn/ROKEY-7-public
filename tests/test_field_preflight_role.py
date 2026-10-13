"""E2: scripts/field/preflight_role.py with a fake environment (no ROS, no clock daemon, no robot).

One read-only report per PC role (profile host id): what that host runs is read from the
profile (operations / coordination / site camera / robot command, SLAM, perception host),
and each missing package, module, file, profile block or unsynchronised clock is a FAILED
line before any launch stops on it.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts/field/preflight_role.py'
if not SCRIPT.is_file():
    raise unittest.SkipTest('repo preflight_role script absent (installed-wheel checkout)')
spec = importlib.util.spec_from_file_location('field_preflight_role', SCRIPT)
pr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pr)

SYNCED = {'state': 'SYNCED', 'error_bound_ns': 1_000_000}
# A robot's own field values (robots.<id>): identity and its automatic target-move policy (#166).
ROBOT_B = {'task.publisher_id': 'p', 'task.mapping_authority': 'm', 'task.live.auto_target.enabled': True,
           'task.live.auto_target.kinds': ['APPROACH_TARGET'], 'task.live.auto_target.goal_clear_radius_m': 0.2,
           'task.live.auto_target.policy_version': 'p',
           'task.live.sensor_host_clock_domain_id': 'rokey7-utc-field',
           'task.live.sensor_host_clock_error_bound_ns': -1}
ALL_MODULES = object()


class FakeSpecs:
    """find_spec stand-in: every module present except the ones named missing."""

    def __init__(self, missing=()):
        self.missing, self.asked = set(missing), []

    def __call__(self, name):
        self.asked.append(name)
        return None if name in self.missing else object()


class RoleCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)
        shutil.copytree(REPO / 'config', self.dir / 'config')
        self.profile = self.dir / 'config' / 'deployment.v2.example.json'
        raw = json.loads(self.profile.read_text())
        raw['map_basis']['frame_id'] = 'map'          # the field frame (#132); robots use map_frame 'map'
        self.profile.write_text(json.dumps(raw))
        # A fake sourced install: <prefix>/share/<pkg>/package.xml
        self.prefix = self.dir / 'install'
        for package in ('rescue_bringup', 'turtlebot4_navigation'):
            (self.prefix / 'share' / package).mkdir(parents=True)
            (self.prefix / 'share' / package / 'package.xml').write_text('<package/>')
        self.env = {'ROS_DISTRO': 'jazzy', 'AMENT_PREFIX_PATH': str(self.prefix), 'ROS_DOMAIN_ID': '7',
                    'ROS_DISCOVERY_SERVER': '192.0.2.13:11811;192.0.2.16:11811',
                    'ROS_AUTOMATIC_DISCOVERY_RANGE': ''}
        self.field = REPO / 'config' / 'field' / 'search_area.field.json'
        self.model = self.dir / 'model.pt'
        self.model.write_bytes(b'model bytes')
        self.webcam = self.dir / 'webcam.yaml'
        self.write_webcam()
        self.map = self.dir / 'map.yaml'
        (self.dir / 'map.pgm').write_bytes(b'P5 1 1 255 \x00')
        self.map.write_text(json.dumps(dict(image='map.pgm', resolution=0.05, origin=[0.0, 0.0, 0.0],
                                          negate=0, occupied_thresh=0.65, free_thresh=0.25)))

    def write_webcam(self, drop=(), **changes):
        from rescue_perception.webcam_node import PARAMS
        values = {name: 'x' for name in PARAMS}
        values.update(model_path=str(self.model), model_sha256=hashlib.sha256(b'model bytes').hexdigest())
        if 'clock_backend' in PARAMS:  # measured webcam clock (#146)
            values.update(clock_backend='timesyncd', clock_max_age_ns=3600000000000, clock_drift_ppm=100.0)
        values.update(changes)
        for name in drop:
            values.pop(name)
        self.webcam.write_text(json.dumps({'webcam_node': {'ros__parameters': values}}))  # JSON is YAML

    def run_role(self, role, specs=None, clock=lambda backend, **_: dict(SYNCED), **changes):
        values = dict(env=self.env, profile_path=self.profile, field_path=self.field, webcam_params=self.webcam,
                      map_yaml=self.map, find_spec=specs or FakeSpecs(), clock=clock) | changes
        return {c['id']: c for c in pr.run_role(role, **values)}

    def assertStatus(self, checks, check_id, status):
        self.assertIn(check_id, checks, sorted(checks))
        self.assertEqual(status, checks[check_id]['status'], checks[check_id])


class Duties(RoleCase):
    def test_each_role_checks_what_the_profile_gives_it(self):
        compute = self.run_role('compute_a')
        for check_id in ('module.nav2_msgs', 'module.rescue_robot.task_input_node', 'pkg.turtlebot4_navigation',
                         'module.ultralytics', 'map.files', 'field.robot.robot_a'):
            self.assertStatus(compute, check_id, pr.PASSED)
        operations = self.run_role('operations')
        for check_id in ('module.rescue_operations.report_node', 'module.flask', 'webcam.params', 'webcam.model'):
            self.assertStatus(operations, check_id, pr.PASSED)
        self.assertNotIn('module.nav2_msgs', operations)
        coordination = self.run_role('coordination')
        for check_id in ('module.rescue_coordination.ros_node', 'field.load', 'field.coordinator', 'clock.timesyncd'):
            self.assertStatus(coordination, check_id, pr.PASSED)
        self.assertNotIn('webcam.params', coordination)

    def test_a_good_fake_environment_passes_everything(self):
        from rescue_perception.webcam_node import PARAMS
        servers = {'compute_a': '192.0.2.13:11811', 'compute_b': ';192.0.2.16:11811'}  # profile host network
        raw = json.loads(self.field.read_text())
        raw['robots']['robot_b'] = dict(ROBOT_B)  # not in #132
        field = self.dir / 'field.json'
        field.write_text(json.dumps(raw))
        for role in ('operations', 'coordination', 'compute_a', 'compute_b'):
            with self.subTest(role=role):
                env = self.env | {'ROS_DISCOVERY_SERVER': servers.get(role, self.env['ROS_DISCOVERY_SERVER'])}
                checks = self.run_role(role, env=env, field_path=field)
                failed = {k: c['detail'] for k, c in checks.items() if c['status'] != pr.PASSED}
                if role == 'operations' and 'clock_backend' not in PARAMS:
                    # webcam_node before #146 does not measure its clock: no DISASTER from it
                    self.assertEqual(pr.FAILED, checks['clock.webcam']['status'])
                    failed.pop('clock.webcam')
                self.assertEqual({}, failed)

    def test_robot_b_without_field_values_fails(self):
        raw = json.loads(self.field.read_text())
        raw['robots'].pop('robot_b')
        field = self.dir / 'field_no_b.json'
        field.write_text(json.dumps(raw))
        self.assertStatus(self.run_role('compute_b', field_path=field), 'field.robot.robot_b', pr.FAILED)

    def test_unknown_role_is_a_failure_not_an_empty_report(self):
        checks = self.run_role('camera')
        self.assertStatus(checks, 'profile.role', pr.FAILED)


class Failures(RoleCase):
    def test_json_settings_remain_checked_without_yaml(self):
        with patch.dict(sys.modules, {'yaml': None}):
            self.assertStatus(self.run_role('compute_a'), 'map.files', pr.PASSED)
            self.assertStatus(self.run_role('operations'), 'webcam.model', pr.PASSED)

    def test_native_yaml_without_parser_is_not_run(self):
        self.map.write_text('image: map.pgm\n')
        self.webcam.write_text('webcam_node:\n  ros__parameters: {}\n')
        with patch.dict(sys.modules, {'yaml': None}):
            self.assertStatus(self.run_role('compute_a'), 'map.files', pr.NOT_RUN)
            self.assertStatus(self.run_role('operations'), 'webcam.params', pr.NOT_RUN)

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'native YAML parsing needs PyYAML')
    def test_malformed_yaml_is_failed(self):
        self.map.write_text('image: [unterminated')
        self.webcam.write_text('webcam_node: [unterminated')
        self.assertStatus(self.run_role('compute_a'), 'map.files', pr.FAILED)
        self.assertStatus(self.run_role('operations'), 'webcam.params', pr.FAILED)

    def test_ros_not_sourced(self):
        checks = self.run_role('coordination', env={})
        self.assertStatus(checks, 'ros.distro', pr.FAILED)
        self.assertStatus(checks, 'ros.workspace', pr.FAILED)

    def test_missing_module_or_package(self):
        checks = self.run_role('compute_a', specs=FakeSpecs(missing={'nav2_msgs'}))
        self.assertStatus(checks, 'module.nav2_msgs', pr.FAILED)
        shutil.rmtree(self.prefix / 'share' / 'turtlebot4_navigation')
        self.assertStatus(self.run_role('compute_a'), 'pkg.turtlebot4_navigation', pr.FAILED)

    def test_profile_without_map_basis(self):
        raw = json.loads(self.profile.read_text())
        del raw['map_basis']
        self.profile.write_text(json.dumps(raw))
        self.assertStatus(self.run_role('coordination'), 'profile.map_basis', pr.FAILED)

    def test_broken_profile_stops_the_dependent_checks(self):
        self.profile.write_text('{')
        checks = self.run_role('coordination')
        self.assertStatus(checks, 'profile.load', pr.FAILED)
        self.assertStatus(checks, 'profile.role', pr.NOT_RUN)

    def test_domain_and_discovery_server_must_be_the_profile_host_network(self):
        checks = self.run_role('coordination', env=self.env | {'ROS_DOMAIN_ID': '8'})
        self.assertStatus(checks, 'env.ROS_DOMAIN_ID', pr.FAILED)
        checks = self.run_role('coordination', env=self.env | {'ROS_DISCOVERY_SERVER': '192.0.2.13:11811'})
        self.assertStatus(checks, 'env.ROS_DISCOVERY_SERVER', pr.FAILED)
        checks = self.run_role('compute_a', env=self.env | {'ROS_DISCOVERY_SERVER': '192.0.2.13:11811'})
        self.assertStatus(checks, 'env.ROS_DISCOVERY_SERVER', pr.PASSED)

    def test_unsynchronised_or_unreadable_clock(self):
        checks = self.run_role('coordination', clock=lambda backend, **_: {'state': 'UNKNOWN'})
        self.assertStatus(checks, 'clock.timesyncd', pr.FAILED)

        def broken(backend, **_):
            raise ValueError('clock daemon unavailable')
        self.assertStatus(self.run_role('compute_a', clock=broken), 'clock.timesyncd', pr.FAILED)

    def test_webcam_params_missing_key_or_wrong_model(self):
        from rescue_perception.webcam_node import PARAMS
        self.write_webcam(drop=('model_conf',))
        self.assertStatus(self.run_role('operations'), 'webcam.params', pr.FAILED)
        self.write_webcam(model_sha256='0' * 64)
        self.assertStatus(self.run_role('operations'), 'webcam.model', pr.FAILED)
        self.write_webcam(model_path=str(self.dir / 'absent.pt'))
        self.assertStatus(self.run_role('operations'), 'webcam.model', pr.FAILED)
        clock_keys = [name for name in PARAMS if name.startswith('clock_')]
        self.write_webcam(drop=clock_keys[:1])
        checks = self.run_role('operations')
        self.assertStatus(checks, 'webcam.params', pr.FAILED)
        self.assertIn(clock_keys[0], checks['webcam.params']['detail'])

    def test_webcam_and_map_files_must_be_given(self):
        self.assertStatus(self.run_role('operations', webcam_params=None), 'webcam.params', pr.NOT_RUN)
        self.assertStatus(self.run_role('compute_a', map_yaml=None), 'map.files', pr.NOT_RUN)
        self.assertStatus(self.run_role('coordination', field_path=None), 'field.load', pr.NOT_RUN)

    def test_map_image_must_exist(self):
        (self.dir / 'map.pgm').unlink()
        self.assertStatus(self.run_role('compute_a'), 'map.files', pr.FAILED)

    def test_field_file_without_this_robot(self):
        raw = json.loads(self.field.read_text())
        raw['robots'].pop('robot_a')
        raw['robots']['robot_b'] = dict(ROBOT_B)
        field = self.dir / 'field.json'
        field.write_text(json.dumps(raw))
        self.assertStatus(self.run_role('compute_a', field_path=field), 'field.robot.robot_a', pr.FAILED)

    def test_exit_code_and_report(self):
        checks = pr.run_role('coordination', env={}, profile_path=self.profile, field_path=None,
                             webcam_params=None, map_yaml=None, find_spec=FakeSpecs(), clock=lambda b, **_: SYNCED)
        self.assertEqual(2, pr.summarize(checks))
        lines = pr.format_report(checks).splitlines()
        self.assertTrue(all(line.split()[0] in (pr.PASSED, pr.FAILED, pr.NOT_RUN) for line in lines))


class DiscoveryServers(RoleCase):
    """Plan A (network-discovery.md, 2026-10-13): each PC joins the robots' onboard Pi servers.

    The robot PCs join their own robot's server, coordination and operations join both. The slot
    in ROS_DISCOVERY_SERVER is the server ID, so robots with IDs 6 and 5 give ';;;;;;ip6' style
    values. The IDs here stand in for the values read from the robots.
    """

    A, B = ('198.51.100.6', 11811, 6), ('198.51.100.5', 11811, 5)

    def setUp(self):
        super().setUp()
        raw = json.loads(self.profile.read_text())
        servers = {'compute_a': [self.A], 'compute_b': [self.B], 'coordination': [self.A, self.B],
                   'operations': [self.A, self.B]}
        for host in raw['hosts']:
            host['network']['discovery_servers'] = [dict(server_id=i, address=a, port=p)
                                                    for a, p, i in servers[host['id']]]
        self.profile.write_text(json.dumps(raw))

    @staticmethod
    def value(*servers):
        slots = [''] * (max(i for _, _, i in servers) + 1)
        for address, port, server_id in servers:
            slots[server_id] = f'{address}:{port}'
        return ';'.join(slots)

    def status(self, role, value):
        return self.run_role(role, env=self.env | {'ROS_DISCOVERY_SERVER': value})['env.ROS_DISCOVERY_SERVER']['status']

    def test_two_servers_for_coordination_and_operations(self):
        both = self.value(self.A, self.B)
        self.assertEqual(';;;;;198.51.100.5:11811;198.51.100.6:11811', both)
        for role in ('coordination', 'operations'):
            with self.subTest(role=role):
                self.assertEqual(pr.PASSED, self.status(role, both))
                self.assertEqual(pr.FAILED, self.status(role, self.value(self.A)))       # one server missing
                self.assertEqual(pr.FAILED, self.status(role, '198.51.100.6:11811;198.51.100.5:11811'))  # slots 0, 1

    def test_one_server_for_each_robot_pc(self):
        self.assertEqual(pr.PASSED, self.status('compute_a', self.value(self.A)))
        self.assertEqual(pr.PASSED, self.status('compute_b', self.value(self.B)))
        self.assertEqual(pr.FAILED, self.status('compute_a', self.value(self.B)))        # the other robot
        self.assertEqual(pr.FAILED, self.status('compute_a', self.value(self.A, self.B)))  # extra server
        self.assertEqual(pr.FAILED, self.status('compute_a', '198.51.100.6:11811'))      # slot 0, not ID 6
        self.assertEqual(pr.FAILED, self.status('compute_a', ''))

    def test_robots_with_the_same_server_id_cannot_be_joined_together(self):
        """Two onboard servers with one ID: a PC on both cannot list them (one slot), the profile says so."""
        raw = json.loads(self.profile.read_text())
        for host in raw['hosts']:
            for server in host['network']['discovery_servers']:
                server['server_id'] = 0
        self.profile.write_text(json.dumps(raw))
        checks = self.run_role('coordination')
        self.assertStatus(checks, 'profile.load', pr.FAILED)
        self.assertIn('server_id must be unique', checks['profile.load']['detail'])


if __name__ == '__main__':
    unittest.main()
