"""TB-6 SLAM as run in the field (실습 기록 #339 B3·B4, 2026-10-15; no ROS).

B3: slam_toolbox only processes a scan after the robot moved minimum_travel_distance / minimum_travel_heading
(TurtleBot4 slam.yaml 0.1 / 0.1), so a TB-6 standing on its start mark gave no /robot6/pose and the freeze had
no start pose. In the field both were set to 0.0 and /robot6/pose came at 2 Hz [측정 2026-10-15].
tb6_slam.launch.py now reads TurtleBot4 slam.yaml as is and changes these two values only (slam_field).

B4: slam_toolbox stayed inactive when launched in the background (launch missed the configure -> inactive
transition event [추정]). slam_active_check reads the lifecycle state after a wait and logs SLAM_ACTIVE or
SLAM_NOT_ACTIVE; an inactive node is sent activate once, logged, and checked again. Nothing else is redone.
"""
import unittest

from test_bringup_field_launch import REPO
from test_bringup_no_backup import load_yaml

VENDOR = REPO / 'src' / 'vendor' / 'turtlebot4' / 'turtlebot4_navigation' / 'config' / 'slam.yaml'
LAUNCH = REPO / 'src' / 'rescue_bringup' / 'launch' / 'tb6_slam.launch.py'


def vendor_params():
    if not VENDOR.exists():
        raise unittest.SkipTest('installed-wheel checkout: no vendor slam.yaml')
    return load_yaml(VENDOR)


class FieldSlam(unittest.TestCase):
    def test_only_the_two_travel_values_change(self):
        from rescue_bringup.slam_field import field_slam
        base = vendor_params()
        out = field_slam(base)
        params, original = out['slam_toolbox']['ros__parameters'], base['slam_toolbox']['ros__parameters']
        self.assertEqual((0.1, 0.1), (original['minimum_travel_distance'], original['minimum_travel_heading']))
        self.assertEqual((0.0, 0.0), (params['minimum_travel_distance'], params['minimum_travel_heading']))
        for key in set(original) | set(params):
            if key not in ('minimum_travel_distance', 'minimum_travel_heading'):
                self.assertEqual(original.get(key), params.get(key), key)
        self.assertEqual(0.1, base['slam_toolbox']['ros__parameters']['minimum_travel_distance'])  # not changed

    def test_an_unexpected_file_is_refused(self):
        from rescue_bringup.slam_field import SlamFieldError, field_slam

        def without(key):
            return {'slam_toolbox': {'ros__parameters': {k: v for k, v in
                                                         {'minimum_travel_distance': 0.1,
                                                          'minimum_travel_heading': 0.1}.items() if k != key}}}
        for name, base in (('no slam_toolbox', {'amcl': {}}), ('no ros__parameters', {'slam_toolbox': {}}),
                           ('no distance', without('minimum_travel_distance')),
                           ('no heading', without('minimum_travel_heading')), ('not a mapping', [])):
            with self.subTest(name), self.assertRaises(SlamFieldError):
                field_slam(base)


class ActiveCheck(unittest.TestCase):
    def judge(self, state, sent=False, activate_once=True):
        from rescue_bringup.slam_active import judge
        return judge(state, node='/robot6/slam_toolbox', activate_once=activate_once, sent=sent)

    def test_active_is_logged_and_nothing_sent(self):
        line, action = self.judge('active')
        self.assertTrue(line.startswith('SLAM_ACTIVE /robot6/slam_toolbox'), line)
        self.assertIsNone(action)

    def test_inactive_is_activated_once_and_logged(self):
        line, action = self.judge('inactive')
        self.assertTrue(line.startswith('SLAM_NOT_ACTIVE /robot6/slam_toolbox state=inactive'), line)
        self.assertIn('activate once', line)
        self.assertEqual('ACTIVATE', action)
        line, action = self.judge('inactive', sent=True)              # still inactive after the one activate
        self.assertTrue(line.startswith('SLAM_NOT_ACTIVE /robot6/slam_toolbox state=inactive'), line)
        self.assertIn('after activate', line)
        self.assertIsNone(action)
        line, action = self.judge('active', sent=True)
        self.assertTrue(line.startswith('SLAM_ACTIVE /robot6/slam_toolbox'), line)
        self.assertIn('after activate', line)

    def test_without_activate_once_only_logs(self):
        line, action = self.judge('inactive', activate_once=False)
        self.assertTrue(line.startswith('SLAM_NOT_ACTIVE'), line)
        self.assertIsNone(action)

    def test_other_states_and_no_answer_are_only_logged(self):
        """configure is launch's job: an unconfigured node or no answer is not redone here."""
        for state in ('unconfigured', 'finalized', 'configuring', None):
            with self.subTest(state=state):
                line, action = self.judge(state)
                self.assertTrue(line.startswith('SLAM_NOT_ACTIVE /robot6/slam_toolbox state='
                                                + (state or 'NO_ANSWER')), line)
                self.assertIsNone(action)


@unittest.skipUnless(LAUNCH.exists(), 'installed-wheel checkout has no src/ launch files')
class Launch(unittest.TestCase):
    def test_launch_uses_the_field_slam_values_and_the_check(self):
        text = LAUNCH.read_text()
        self.assertIn('field_slam(', text)
        self.assertIn("executable='slam_active_check'", text)
        self.assertIn("'slam_active_wait_s'", text)
        self.assertIn("'slam_activate_once'", text)

    def test_entry_point(self):
        setup = (REPO / 'src' / 'rescue_bringup' / 'setup.py').read_text()
        self.assertIn('slam_active_check = rescue_bringup.slam_active:main', setup)


if __name__ == '__main__':
    unittest.main()
