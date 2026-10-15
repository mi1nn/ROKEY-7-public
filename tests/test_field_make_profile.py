"""scripts/field/make_field_profile.py: the field deployment profile from the example, the same on every PC.

Until 2026-10-15 the field profile was the example edited by hand on each PC and compared by sha256 afterwards
(#339 기록). The tool writes it from one command; these tests pin what it changes and that the field launches
accept the result.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from rescue_bringup import field_launch
from rescue_bringup.profile import load_deployment

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'make_field_profile.py'
if not SCRIPT.is_file():
    raise unittest.SkipTest('scripts/field/make_field_profile.py absent (installed-wheel checkout)')
SPEC = importlib.util.spec_from_file_location('make_field_profile', SCRIPT)
mfp = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mfp)
FIELD = REPO / 'config' / 'field' / 'search_area.field.json'
SERVER = '192.0.2.50:11811'   # documentation address (RFC 5737), not a field value


class MakeFieldProfile(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.out = Path(self.tmp.name) / 'profile'

    def make(self, **kw):
        return mfp.write(self.out, mfp.build(discovery=SERVER, domain=6, **kw))

    def test_field_namespaces_network_and_basis(self):
        self.make()
        profile = json.loads((self.out / 'profile.json').read_text())
        self.assertEqual({'robot_a': '/robot6', 'robot_b': '/robot5'},
                         {r['id']: r['namespace'] for r in profile['robots']})
        self.assertEqual('rokey7_field', profile['deployment_id'])
        self.assertEqual('map', profile['map_basis']['frame_id'])
        for host in profile['hosts']:
            self.assertEqual({'domain_id': 6, 'super_client': False,
                              'discovery_servers': [{'server_id': 0, 'address': '192.0.2.50', 'port': 11811}]},
                             host['network'])
        self.assertNotIn('home_positions', profile)   # none is made up
        robot_b = json.loads((self.out / 'robots' / 'robot_b.v2.json').read_text())
        self.assertEqual(['/robot5/oakd/rgb/camera_info'], robot_b['readiness']['camera_topics'])   # #342
        self.assertNotIn('/robot_b', json.dumps(robot_b))
        self.assertNotIn('"/robot_a', (self.out / 'profile.json').read_text())

    def test_differs_from_the_10_15_field_files_only_where_meant(self):
        # 10-15 실기 robot_a.v2.json (sha256 7bafe6da..., 실기·배포 자리 전달): the example with the field
        # namespace, camera topic then the full RGB (재범 17:25); #342 made it the RGB camera_info.
        field_robot_a = {
            'schema_version': 2, 'robot_id': 'robot_a', 'namespace': '/robot6', 'runtime_enabled': False,
            'use_sim_time': False, 'nav_action': '/robot6/navigate_to_pose', 'odom_topic': '/robot6/odom',
            'dock_status_topic': '/robot6/dock_status',
            'readiness': {'version': 'example-2026-10-08', 'max_age_s': {
                'odom': 0.5, 'scan': 0.5, 'camera': 1.0, 'wheel_status': 0.5, 'map_to_odom_tf': 1.0},
                'scan_topic': '/robot6/scan', 'tf_topic': '/robot6/tf', 'map_frame': 'map', 'odom_frame': 'odom',
                'camera_topics': ['/robot6/oakd/rgb/image_raw/compressed'],
                'wheel_status_topic': '/robot6/wheel_status', 'watchdog_heartbeat_topic': ''}}
        self.make()
        made = json.loads((self.out / 'robots' / 'robot_a.v2.json').read_text())
        field_robot_a['readiness']['camera_topics'] = ['/robot6/oakd/rgb/camera_info']
        self.assertEqual(field_robot_a, made)
        # the profile: only deployment_id differs from the 10-15 field profile (sha256 8a8efff9...)
        example = json.loads(mfp.EXAMPLE.read_text())
        profile = json.loads((self.out / 'profile.json').read_text())
        changed = sorted(key for key in example if example[key] != profile[key])
        self.assertEqual(['deployment_id', 'hosts', 'map_basis', 'robots'], changed)
        self.assertEqual(example['mission'], profile['mission'])   # hold_positions keep their own frame

    def test_the_field_launches_take_it(self):
        self.make()
        deployment, field = load_deployment(self.out / 'profile.json'), field_launch.load_field(FIELD)
        for robot_id in ('robot_a', 'robot_b'):
            values = field_launch.robot_launch_values(deployment, field=field, robot_id=robot_id,
                                                      occupancy_sha256='', now_ns=1)
            self.assertEqual({'robot_a': '/robot6', 'robot_b': '/robot5'}[robot_id], values['namespace'])
        field_launch.slam_launch_values(deployment, field=field, robot_id='robot_a')
        field_launch.frozen_localization_values(deployment, field=field, robot_id='robot_b')

    def test_same_inputs_same_hashes(self):
        first = mfp.build(discovery=SERVER, domain=6)
        self.assertEqual(first, mfp.build(discovery=SERVER, domain=6))
        hashes = self.make()
        self.assertEqual({name: hashlib.sha256(data).hexdigest() for name, data in first.items()}, hashes)

    def test_measured_homes_only(self):
        self.make(homes=[('robot_a', [1.0, -0.5, 3.1])])
        self.assertEqual({'robot_a': [1.0, -0.5, 3.1]},
                         json.loads((self.out / 'profile.json').read_text())['home_positions'])
        self.assertEqual((1.0, -0.5, 3.1), load_deployment(self.out / 'profile.json').home_of('robot_a'))

    def test_refusals_write_nothing(self):
        self.out.mkdir()
        with self.assertRaisesRegex(mfp.FieldProfileError, 'already exists'):
            self.make()
        self.assertEqual([], list(self.out.iterdir()))
        for kw, pattern in ((dict(discovery='192.0.2.50'), 'address>:<port'), (dict(domain=300), 'domain'),
                            (dict(homes=[('robot_c', [0.0, 0.0, 0.0])]), 'home')):
            args = dict(discovery=SERVER, domain=6) | kw
            with self.subTest(kw=kw), self.assertRaisesRegex(mfp.FieldProfileError, pattern):
                mfp.build(**args)
        with self.assertRaisesRegex(mfp.FieldProfileError, 'robot_id>=x,y,yaw'):
            mfp._home('robot_a=1,2')

    def test_command_line(self):
        from contextlib import redirect_stdout
        import io
        out = io.StringIO()
        with redirect_stdout(out):
            code = mfp.main(['--out', str(self.out), '--discovery', SERVER, '--domain', '6',
                             '--home', 'robot_b=0.2,1.7,2.9'])
        self.assertEqual(0, code)
        self.assertEqual(['profile.json', 'robots/robot_a.v2.json', 'robots/robot_b.v2.json'],
                         [line.split('  ')[1] for line in out.getvalue().splitlines()])
        self.assertEqual(2, mfp.main(['--out', str(self.out), '--discovery', SERVER, '--domain', '6']))


if __name__ == '__main__':
    unittest.main()
