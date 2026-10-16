"""scripts/sim/sim_webcam.py: the fixed webcam for run_full.sh, as runbook step 3 attaches it (재범 2026-10-16 10:07).

run_full.sh started operations.launch.py without the webcam and the console without --camera, so the console's field
webcam panel said there was no webcam address. With SIM_WEBCAM_DEVICE the run copies the webcam YAML into the run folder
(model path made absolute: the YAML's is relative to the repository root and the launch does not run there) and passes
the console the --camera value the launch logs, as the runbook says.
"""
import importlib.util
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'sim_webcam.py'
EXAMPLE = REPO / 'src' / 'rescue_perception' / 'rescue_perception' / 'config' / 'webcam_node.example.yaml'
if not TOOL.exists():   # the installed-wheel check copies tests/, config/ and some scripts/ only
    raise unittest.SkipTest('installed-wheel checkout has no scripts/')
spec = importlib.util.spec_from_file_location('sim_webcam', TOOL)
sim_webcam = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim_webcam)


class WebcamYaml(unittest.TestCase):
    def test_model_path_becomes_absolute_and_nothing_else_changes(self):
        text = EXAMPLE.read_text()
        out = sim_webcam.webcam_yaml(text, '/repo')
        self.assertIn('    model_path: /repo/assets/models/webcam-yolo26n-best.pt', out)
        self.assertNotIn('model_path: assets/', out)
        changed = [(a, b) for a, b in zip(text.splitlines(), out.splitlines()) if a != b]
        self.assertEqual(1, len(changed))
        self.assertEqual(len(text.splitlines()), len(out.splitlines()))

    def test_a_yaml_without_a_relative_model_path_is_refused(self):
        with self.assertRaises(ValueError):
            sim_webcam.webcam_yaml('webcam_node:\n  ros__parameters:\n    camera_id: cam_fixed\n', '/repo')


class ConsoleCamera(unittest.TestCase):
    LOG = ('[INFO] [launch.user]: plugged cameras: usb-Web_Camera_Web_Camera_202400126002-video-index0\n'
           '\x1b[0m[INFO] [launch.user]: console --camera cam_fixed=http://100.64.0.1:18190/cam_fixed.mjpg\x1b[0m\n')

    def test_the_value_the_launch_logs(self):
        self.assertEqual('cam_fixed=http://100.64.0.1:18190/cam_fixed.mjpg', sim_webcam.console_camera(self.LOG))

    def test_no_line_is_none(self):
        self.assertIsNone(sim_webcam.console_camera('[INFO] rescue_db_writer git=abc\n'))

    def test_run_full_attaches_it_only_with_the_device(self):
        script = (REPO / 'scripts' / 'sim' / 'run_full.sh').read_text()
        self.assertIn('SIM_WEBCAM_DEVICE', script)
        self.assertIn('webcam_params_file:=', script)
        self.assertIn('video_bind_host:=$web_host', script)   # the phone browser must reach the relay
        self.assertIn('--camera-stale-s 2.0', script)


if __name__ == '__main__':
    unittest.main()
