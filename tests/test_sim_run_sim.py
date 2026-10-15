"""scripts/sim/run_sim.sh --check-only: the safety and argument checks, without ROS or Gazebo.

Simulation track stage 1 (재범 decision 2026-10-15 evening). A real launch needs Gazebo and a display, so it is
checked by hand and recorded in the PR; what the script refuses before it starts anything is checked here.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'sim' / 'run_sim.sh'
HAS_SCRIPT = SCRIPT.exists()   # the installed-wheel check copies tests/ and config/ only
PARTITION = 'unit_test_partition'   # no real Gazebo runs here, so the same-partition check stays quiet


@unittest.skipUnless(HAS_SCRIPT, 'installed-wheel checkout has no scripts/')
class RunSimChecks(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.worlds = Path(tmp.name)
        (self.worlds / 'tiny.sdf').write_text('<sdf version="1.8"><world name="tiny"/></sdf>\n')
        (self.worlds / 'tiny.spawn.json').write_text(json.dumps(
            {'world': 'tiny', 'robots': {'robot6': {'x': 0.5, 'y': -1.25, 'yaw': 1.5708}}}))

    def run_script(self, *args, **env):
        base = {'PATH': os.environ['PATH'], 'HOME': os.environ.get('HOME', '/tmp'), 'RUN_SIM_WORLD_DIR': str(self.worlds),
                'RUN_SIM_GZ_PARTITION': PARTITION}
        base.update(env)
        return subprocess.run(['bash', str(SCRIPT), '--check-only', *args], capture_output=True, text=True,
                              env=base, timeout=60)

    def test_defaults_are_the_map_seat(self):
        done = self.run_script('--world', 'tiny', RUN_SIM_GZ_PARTITION='')
        if done.returncode != 0:
            # the map seat's own Gazebo may be running on this PC right now: that refusal names the default partition
            self.assertIn('파티션 sim_map 에 이전 Gazebo', done.stderr)
            return
        self.assertIn('domain=77', done.stdout)
        self.assertIn('partition=sim_map', done.stdout)
        self.assertIn('namespace=robot6', done.stdout)
        self.assertIn('ROS_DISCOVERY_SERVER=<unset>', done.stdout)

    def test_the_start_pose_comes_from_spawn_json(self):
        done = self.run_script('--world', 'tiny')
        self.assertIn('x=0.5 y=-1.25 yaw=1.5708', done.stdout)

    def test_a_given_pose_wins_but_only_all_three_together(self):
        done = self.run_script('--world', 'tiny', '--x', '1', '--y', '2', '--yaw', '0.5')
        self.assertEqual(0, done.returncode, done.stderr)
        self.assertIn('x=1 y=2 yaw=0.5', done.stdout)
        done = self.run_script('--world', 'tiny', '--x', '1')
        self.assertEqual(2, done.returncode)
        self.assertIn('셋을 함께', done.stderr)

    def test_another_seat_gets_its_own_domain_partition_and_name(self):
        (self.worlds / 'tiny.spawn.json').write_text(json.dumps(
            {'world': 'tiny', 'robots': {'robot5': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
                                         'robot6': {'x': 0.5, 'y': -1.25, 'yaw': 1.5708}}}))
        done = self.run_script('--world', 'tiny', RUN_SIM_DOMAIN_ID='79', RUN_SIM_GZ_PARTITION='unit_test_full',
                               RUN_SIM_NAMESPACE='robot5')
        self.assertEqual(0, done.returncode, done.stderr)
        self.assertIn('domain=79', done.stdout)
        self.assertIn('partition=unit_test_full', done.stdout)
        self.assertIn('namespace=robot5', done.stdout)
        # --namespace beats the environment
        done = self.run_script('--world', 'tiny', '--namespace', 'robot6', RUN_SIM_NAMESPACE='robot5')
        self.assertIn('namespace=robot6', done.stdout)

    def test_refusals(self):
        refusals = (
            ('field domain', ('--world', 'tiny'), {'RUN_SIM_DOMAIN_ID': '6'}, '70~99'),
            ('domain not a number', ('--world', 'tiny'), {'RUN_SIM_DOMAIN_ID': 'x'}, '70~99'),
            ('bad partition', ('--world', 'tiny'), {'RUN_SIM_GZ_PARTITION': 'Sim-Map'}, 'PARTITION'),
            ('slash in the name', ('--world', 'tiny', '--namespace', '/robot6'), {}, '슬래시'),
            ('robot not in spawn.json', ('--world', 'tiny', '--namespace', 'robot9'), {}, 'robot9'),
            ('no such world', ('--world', 'nowhere'), {}, 'world 파일이 없습니다'),
            ('world name mismatch', ('--world', 'other'), {}, '<world name>'),
            ('a real map is not an argument', ('--world', 'tiny', '--map', 'live_map.yaml'), {}, '지도를 받는 인자는 없습니다'),
            ('pose not a number', ('--world', 'tiny', '--x', 'a', '--y', '1', '--yaw', '0'), {}, '숫자'),
        )
        (self.worlds / 'other.sdf').write_text('<sdf version="1.8"><world name="not_other"/></sdf>\n')
        for name, args, env, message in refusals:
            with self.subTest(name):
                done = self.run_script(*args, **env)
                self.assertEqual(2, done.returncode, done.stdout)
                self.assertIn(message, done.stderr)

    def test_the_arena_world_in_the_repository_has_a_start_pose(self):
        done = subprocess.run(['bash', str(SCRIPT), '--check-only', '--world', 'arena'], capture_output=True, text=True,
                              env={'PATH': os.environ['PATH'], 'HOME': os.environ.get('HOME', '/tmp'),
                                   'RUN_SIM_GZ_PARTITION': PARTITION}, timeout=60)
        if not (REPO / 'config' / 'sim' / 'worlds' / 'arena.sdf').exists():
            self.skipTest('no config/sim (installed-wheel checkout)')
        self.assertEqual(0, done.returncode, done.stderr)
        self.assertIn('namespace=robot6', done.stdout)


if __name__ == '__main__':
    unittest.main()
