"""E2: scripts/field/preflight.py without ROS (pure functions, fake runner, local sockets).

Collected by scripts/run_tests.py; no ROS needed. Alone:

    python3 -I -m unittest discover -s tests -p 'test_field_preflight.py' -v
"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts/field/preflight.py'
if not SCRIPT.is_file():
    # scripts/check_wheels.py copies only tests/ and config/ to test installed wheels;
    # preflight is a repo script, not a wheel. scripts/run_tests.py runs these tests.
    raise unittest.SkipTest('scripts/field/preflight.py absent (installed-wheel checkout)')
spec = importlib.util.spec_from_file_location('field_preflight', SCRIPT)
pf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pf)

CONFIG = {
    'ros': {'distro': 'jazzy', 'rmw_implementation': 'rmw_fastrtps_cpp', 'domain_id': None},
    'discovery_servers': [{'server_id': 3, 'address': 'r3', 'port': 11811},
                          {'server_id': 6, 'address': 'r6', 'port': 11811}],
}
ENV_OK = {'ROS_DISTRO': 'jazzy', 'ROS_DISCOVERY_SERVER': ';;;r3:11811;;;r6:11811'}

LIST_V = """Published topics:
 * /robot_a/cmd_vel [geometry_msgs/msg/Twist] 2 publishers
 * /robot_a/map [nav_msgs/msg/OccupancyGrid] 1 publisher
 * /robot_a/odom [nav_msgs/msg/Odometry] 1 publisher

Subscribed topics:
 * /robot_b/cmd_vel [geometry_msgs/msg/Twist] 1 subscriber
"""


def by_id(checks):
    return {c['id']: c for c in checks}


class DiscoveryParse(unittest.TestCase):
    def test_slot_index_is_server_id(self):
        self.assertEqual(pf.discovery_servers(';;;r3:11811;;;r6:11811'),
                         [{'server_id': 3, 'address': 'r3', 'port': 11811},
                          {'server_id': 6, 'address': 'r6', 'port': 11811}])

    def test_default_port_trailing_semicolon_and_ipv6(self):
        self.assertEqual(pf.discovery_servers('h1;[::1]:11899;'),
                         [{'server_id': 0, 'address': 'h1', 'port': 11811},
                          {'server_id': 1, 'address': '::1', 'port': 11899}])

    def test_bad_port_is_none_not_default(self):
        self.assertEqual(pf.discovery_servers('h:abc'), [{'server_id': 0, 'address': 'h', 'port': None}])
        c = pf.check_servers({'ROS_DISCOVERY_SERVER': 'h:abc'}, 0.1, probe=lambda *a: 'reply')
        self.assertEqual(c[0]['status'], pf.NOT_RUN)

    def test_unset_is_empty(self):
        self.assertEqual(pf.discovery_servers(None), [])
        self.assertEqual(pf.discovery_servers(''), [])

    def test_p4_discovery_host_kept(self):
        self.assertEqual(pf.discovery_host(';;;r3:11811'), 'r3')
        self.assertIsNone(pf.discovery_host(''))


class Summary(unittest.TestCase):
    def test_exit_codes(self):
        ok, bad, nr = (pf.check('x', s, '') for s in (pf.PASSED, pf.FAILED, pf.NOT_RUN))
        self.assertEqual(pf.summarize([ok]), 0)
        self.assertEqual(pf.summarize([ok, nr]), 3)
        self.assertEqual(pf.summarize([ok, nr, bad]), 2)
        self.assertEqual(pf.summarize([]), 3)   # zero checks is never PASSED


class Environment(unittest.TestCase):
    def test_matching_config_passes_and_unset_rmw_is_jazzy_default(self):
        c = by_id(pf.check_environment(ENV_OK, CONFIG))
        self.assertEqual(c['env.ros_distro']['status'], pf.PASSED)
        self.assertEqual(c['env.rmw']['status'], pf.PASSED)
        self.assertEqual(c['env.discovery_server']['status'], pf.PASSED)
        self.assertEqual(c['env.domain_id']['status'], pf.NOT_RUN)

    def test_cyclone_with_discovery_server_fails_even_without_config(self):
        env = dict(ENV_OK, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        self.assertEqual(by_id(pf.check_environment(env, None))['env.rmw']['status'], pf.FAILED)

    def test_missing_server_fails(self):
        env = dict(ENV_OK, ROS_DISCOVERY_SERVER=';;;r3:11811')
        self.assertEqual(by_id(pf.check_environment(env, CONFIG))['env.discovery_server']['status'], pf.FAILED)

    def test_placeholder_config_is_not_run(self):
        config = json.loads((REPO / 'config/hosts/central.example.json').read_text())
        self.assertEqual(by_id(pf.check_environment(ENV_OK, config))['env.discovery_server']['status'], pf.NOT_RUN)

    def test_no_ros_fails_distro(self):
        self.assertEqual(by_id(pf.check_environment({}, None))['env.ros_distro']['status'], pf.FAILED)

    def test_domain_compared_when_given(self):
        config = dict(CONFIG, ros=dict(CONFIG['ros'], domain_id=7))
        self.assertEqual(by_id(pf.check_environment(dict(ENV_OK, ROS_DOMAIN_ID='7'), config))['env.domain_id']['status'],
                         pf.PASSED)
        self.assertEqual(by_id(pf.check_environment(ENV_OK, config))['env.domain_id']['status'], pf.FAILED)


class Install(unittest.TestCase):
    def test_glob_and_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ('nav2_msgs', 'turtlebot4_msgs'):
                (Path(tmp) / 'share' / name).mkdir(parents=True)
                (Path(tmp) / 'share' / name / 'package.xml').write_text('<package/>')
            env = {'AMENT_PREFIX_PATH': tmp, 'PATH': ''}
            c = by_id(pf.check_install(env, {'required_packages': ['nav2_msgs', 'turtlebot4_*', 'irobot_create_msgs']},
                                       which=lambda *a, **k: None))
        self.assertEqual(c['pkg.nav2_msgs']['status'], pf.PASSED)
        self.assertEqual(c['pkg.turtlebot4_*']['measured']['found'], ['turtlebot4_msgs'])
        self.assertEqual(c['pkg.irobot_create_msgs']['status'], pf.FAILED)
        self.assertEqual(c['tool.fastdds']['status'], pf.FAILED)

    def test_unsourced_is_not_run(self):
        c = by_id(pf.check_install({}, None, which=lambda *a, **k: None))
        self.assertTrue(all(v['status'] == pf.NOT_RUN for v in c.values()))


class Server(unittest.TestCase):
    def test_closed_local_port_is_refused(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
        s.close()
        self.assertEqual(pf.udp_probe('127.0.0.1', port, 1.0), 'refused')

    def test_listening_port_gives_no_reply_and_stays_not_run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('127.0.0.1', 0))
            env = {'ROS_DISCOVERY_SERVER': f'127.0.0.1:{s.getsockname()[1]}'}
            c = pf.check_servers(env, 0.3)
        self.assertEqual(c[0]['measured']['result'], 'no_reply')
        self.assertEqual(c[0]['status'], pf.NOT_RUN)   # UDP silence never proves the server is up


class Graph(unittest.TestCase):
    def runner(self, stdout, code=0):
        calls = []

        def run(args, env, timeout):
            calls.append((args, env))
            return subprocess.CompletedProcess(args, code, stdout, 'boom')
        return run, calls

    def test_parse_counts_only_published_block(self):
        self.assertEqual(pf.parse_topic_list_verbose(LIST_V),
                         {'/robot_a/cmd_vel': 2, '/robot_a/map': 1, '/robot_a/odom': 1})

    def test_duplicate_publishers_fail_missing_namespace_not_run(self):
        run, calls = self.runner(LIST_V)
        c = by_id(pf.check_graph({'PATH': ''}, ['/robot_a', 'robot_b'], 1, which=lambda *a, **k: 'ros2', runner=run))
        self.assertEqual(c['graph./robot_a/cmd_vel']['status'], pf.FAILED)
        self.assertEqual(c['graph./robot_a/map']['status'], pf.PASSED)
        self.assertEqual(c['graph./robot_b']['status'], pf.NOT_RUN)
        self.assertEqual(calls[0][1]['ROS_SUPER_CLIENT'], 'True')   # child env only

    def test_parent_env_not_mutated(self):
        env = {'PATH': '', 'ROS_SUPER_CLIENT': 'False'}
        run, _ = self.runner(LIST_V)
        pf.check_graph(env, ['/robot_a'], 1, which=lambda *a, **k: 'ros2', runner=run)
        self.assertEqual(env['ROS_SUPER_CLIENT'], 'False')

    def test_cli_failure_and_absence_are_not_run(self):
        run, _ = self.runner('', code=1)
        c = pf.check_graph({'PATH': ''}, ['/robot_a'], 1, which=lambda *a, **k: 'ros2', runner=run)
        self.assertEqual(c[0]['status'], pf.NOT_RUN)
        c = pf.check_graph({'PATH': ''}, ['/robot_a'], 1, which=lambda *a, **k: None)
        self.assertEqual(c[0]['status'], pf.NOT_RUN)


class HostExamples(unittest.TestCase):
    def test_examples_parse_and_hold_no_real_address(self):
        files = sorted((REPO / 'config/hosts').glob('*.example.json'))
        self.assertEqual({f.name.split('.')[0] for f in files},
                         {'robot_control_1', 'robot_control_2', 'central', 'operations'})
        for f in files:
            d = json.loads(f.read_text())
            self.assertEqual(d['role'], f.name.split('.')[0])
            self.assertEqual(d['ros']['rmw_implementation'], 'rmw_fastrtps_cpp')
            self.assertFalse(d['ros']['super_client_runtime'])
            for s in d['discovery_servers']:
                self.assertTrue(pf.is_placeholder(s['address']), f'{f.name}: real host in example')


class MainJson(unittest.TestCase):
    def test_main_writes_json_and_returns_code(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'r.json'
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                code = pf.main(['--json', str(out)], env={'PATH': ''})
            data = json.loads(out.read_text())
        self.assertEqual(code, 2)                     # ROS_DISTRO missing
        self.assertEqual(data['exit_code'], 2)
        self.assertIn('ROS_DISCOVERY_SERVER', data['environment'])
        self.assertEqual(set(data['host_facts']), {'os', 'python', 'ros_installs'})


if __name__ == '__main__':
    unittest.main()
