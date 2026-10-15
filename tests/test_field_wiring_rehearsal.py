"""scripts/field/wiring-rehearsal without ROS: the safety lines of run.sh and what the fakes leave to our nodes.

The rehearsal itself needs ROS 2, a local Discovery Server and a built workspace (run.sh; record in
docs/evidence/rehearsal-2026-10-14-wiring-one-pc.md).
"""
import importlib.util
from pathlib import Path
import py_compile
import re
import subprocess
import sys
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
HERE = REPO / 'scripts' / 'field' / 'wiring-rehearsal'
if not HERE.is_dir():
    raise unittest.SkipTest('scripts/field/wiring-rehearsal absent (installed-wheel checkout)')
RUN = (HERE / 'run.sh').read_text()


class RunScript(unittest.TestCase):
    def test_bash_syntax_and_python_files_compile(self):
        self.assertEqual(0, subprocess.run(['bash', '-n', str(HERE / 'run.sh')]).returncode)
        for path in sorted(HERE.rglob('*.py')):
            with self.subTest(path=path.name):
                py_compile.compile(str(path), doraise=True)

    def test_only_a_local_discovery_server(self):
        # every inherited server / peer setting is cleared before the local one is set
        self.assertRegex(RUN, r"ROS_DISCOVERY_SERVER\|ROS_SUPER_CLIENT\|ROS_STATIC_PEERS")
        self.assertIn('export ROS_DISCOVERY_SERVER="127.0.0.1:$PORT"', RUN)
        self.assertIn('[ "$ROS_DISCOVERY_SERVER" = "127.0.0.1:$PORT" ] ||', RUN)
        self.assertIn('fast-discovery-server -i 0 -l 127.0.0.1 -p "$PORT"', RUN)
        self.assertIn('[ "$DOMAIN" -ge 10 ] ||', RUN)

    def test_stops_by_process_group_never_by_name(self):
        self.assertIn('set -m', RUN)
        self.assertIn('kill "-$signal" -- "-$pid"', RUN)
        code = '\n'.join(line for line in RUN.splitlines() if not line.lstrip().startswith('#'))
        self.assertNotRegex(code, r'pkill|killall')

    def test_host_shim_only_on_the_executors(self):
        launch = (HERE / 'rehearsal.launch.py').read_text()
        self.assertEqual(1, launch.count('REHEARSAL_HOST_ID'))
        node = launch[launch.index("executable='task_input'"):]
        self.assertIn('REHEARSAL_HOST_ID', node[:node.index('Node(') if 'Node(' in node else len(node)])


class AmclThresholds(unittest.TestCase):
    def test_the_executors_get_the_amcl_update_thresholds_robot_launch_adds(self):
        # #392: a frozen-mode task_input refuses to start without task.saved_map.amcl_update_min_d / _a, which
        # robot.launch.py adds from localization.yaml; the rehearsal adds them from the vendored copy
        launch = (HERE / 'rehearsal.launch.py').read_text()
        self.assertIn('amcl_update_thresholds(yaml.safe_load(LOCALIZATION.read_text()))', launch)
        self.assertIn("parameters=[typed(task)]", launch)

    def test_the_vendored_file_gives_the_thresholds(self):
        try:
            import yaml
        except ImportError:
            self.skipTest('PyYAML not installed (the pure-Python CI job)')
        from rescue_bringup.field_launch import amcl_update_thresholds
        vendored = (REPO / 'src' / 'vendor' / 'turtlebot4' / 'turtlebot4_navigation' / 'config' /
                    'localization.yaml')
        self.assertEqual({'task.saved_map.amcl_update_min_d': 0.25, 'task.saved_map.amcl_update_min_a': 0.2},
                         amcl_update_thresholds(yaml.safe_load(vendored.read_text())))


class Fakes(unittest.TestCase):
    def test_fakes_leave_every_node_of_ours_to_the_launch(self):
        sys.path.insert(0, str(HERE.parent))
        sys.path.insert(0, str(HERE))
        try:
            spec = importlib.util.spec_from_file_location('rehearsal_fake_graph', HERE / 'fake_graph.py')
            fake = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(fake)
            import test_bringup_field_launch as fixtures
            import wiring_check as wc
            from rescue_bringup.field_launch import load_field
            from rescue_bringup.profile import load_deployment
            with tempfile.TemporaryDirectory() as folder:
                deployment = load_deployment(fixtures.field_profile(Path(folder)))
                field = load_field(fixtures.FIELD)
                for phase in ('SEARCHING', 'FROZEN'):
                    ours = fake.ours(deployment, field, phase)
                    expected = {n for lk in wc.expected_links(deployment, field, slam_phase=phase)
                                for n in lk.writers + lk.readers}
                    with self.subTest(phase=phase):
                        self.assertLessEqual(ours, expected)
                        self.assertIn('/robot5/rescue_frozen_map_relay', ours)
                        self.assertEqual(phase == 'FROZEN', '/robot6/rescue_frozen_map_relay' in ours)
                        # what rehearsal.launch.py starts by name
                        launch = (HERE / 'rehearsal.launch.py').read_text()
                        for name in ('rescue_command_watchdog', 'rescue_command_adapter', 'rescue_frozen_map_relay'):
                            self.assertIn(f"name='{name}'", launch)
                        for name in ('operations.launch.py', 'coordinator.launch.py', 'map_freeze.launch.py',
                                     'overhead_passage.launch.py'):
                            self.assertIn(f"'{name}'", launch)
        finally:
            sys.path.remove(str(HERE))
            sys.path.remove(str(HERE.parent))


class MeasuredTf(unittest.TestCase):
    def test_tb6_tf_is_the_measured_one_by_default(self):
        feed = (HERE / 'feed.py').read_text()
        self.assertIn("def __init__(self, node, *, tf='measured')", feed)
        for value in ('0.02000981357266791', '-0.004797844888163447', '-0.14913737773895264',
                      '0.9999651908874512'):   # #291 measured_tb6_tf, 2026-10-14
            self.assertIn(value, feed)
        self.assertRegex(RUN, re.escape('--tf "${REHEARSAL_TF:-measured}"'))


if __name__ == '__main__':
    unittest.main()
