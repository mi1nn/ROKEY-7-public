"""scripts/field/start-align-rehearsal without ROS: syntax and the safety lines of run.sh (record in
docs/evidence/rehearsal-2026-10-15-start-align-one-pc.md)."""
from pathlib import Path
import py_compile
import subprocess
import unittest

HERE = Path(__file__).resolve().parents[1] / 'scripts' / 'field' / 'start-align-rehearsal'
if not HERE.is_dir():
    raise unittest.SkipTest('scripts/field/start-align-rehearsal absent (installed-wheel checkout)')
RUN = (HERE / 'run.sh').read_text()


class RunScript(unittest.TestCase):
    def test_bash_syntax_and_python_files_compile(self):
        self.assertEqual(0, subprocess.run(['bash', '-n', str(HERE / 'run.sh')]).returncode)
        for path in sorted(HERE.glob('*.py')):
            with self.subTest(path=path.name):
                py_compile.compile(str(path), doraise=True)

    def test_local_only_and_stopped_by_process_group(self):
        self.assertIn('unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS', RUN)
        self.assertIn('ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST', RUN)
        self.assertIn('[ "$DOMAIN" -ge 10 ] ||', RUN)
        self.assertIn('set -m', RUN)
        code = '\n'.join(line for line in RUN.splitlines() if not line.lstrip().startswith('#'))
        self.assertNotRegex(code, r'pkill|killall')

    def test_launch_uses_the_field_launch_values(self):
        launch = (HERE / 'align.launch.py').read_text()
        for name in ('frozen_localization_values', 'amcl_parameters', "executable='start_align'",
                     "SetRemap('/tf', ns + '/tf')"):
            self.assertIn(name, launch)


if __name__ == '__main__':
    unittest.main()
