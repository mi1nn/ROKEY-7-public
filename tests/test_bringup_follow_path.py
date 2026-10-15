"""Nav2 FollowPath: Rotation Shim + Regulated Pure Pursuit instead of DWB, on both robots (no ROS).

재범 request 2026-10-14 evening (found in practice): sent by an RViz click to a goal behind the back wall
of a U-shaped dead end, TB-6 kept turning its head left and right inside the U and got stuck, although
the global path correctly went out through the entrance. TurtleBot4's DWB (nav2.yaml) scores GoalDist and
GoalAlign (24 each) toward the goal against PathAlign and PathDist (32) along the path [추정]. Our
launches replace controller_server.FollowPath as they remove the backup recovery (OPS-207); the
TurtleBot4 nav2.yaml itself is not edited. Field use is confirmed only after a supervised before/after
run at the same U spot (NOT_RUN).
"""
import unittest

from test_bringup_no_backup import REPO, load_yaml, vendor_params

SHIM = 'nav2_rotation_shim_controller::RotationShimController'
RPP = 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'


def field(params):
    from rescue_bringup.nav2_field import field_nav2
    return field_nav2(params)


class FollowPath(unittest.TestCase):
    def test_shim_drives_rpp_forward_only(self):
        follow = field(vendor_params())['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(SHIM, follow['plugin'])
        self.assertEqual(RPP, follow['primary_controller'])
        self.assertIs(False, follow['allow_reversing'])        # no reversing (OPS-207)
        self.assertIs(False, follow['use_rotate_to_heading'])  # the shim does the turning (Nav2 docs)
        self.assertIs(True, follow['use_collision_detection'])

    def test_speeds_are_the_turtlebot4_defaults(self):
        """재범 2026-10-16 05:4x 「TB 속도가 실제에 비해 너무 느리다. 실제의 100%로 맞춰라.」 (grade 4): FollowPath drives at the
        TurtleBot4 nav2.yaml limits (DWB max_vel_x / max_vel_theta, the velocity smoother's caps), not below them."""
        base = vendor_params()
        dwb = base['controller_server']['ros__parameters']['FollowPath']
        smoother = base['velocity_smoother']['ros__parameters']
        follow = field(base)['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(dwb['max_vel_x'], follow['desired_linear_vel'])
        self.assertEqual(smoother['max_velocity'][0], follow['desired_linear_vel'])
        self.assertEqual(dwb['max_vel_theta'], follow['rotate_to_heading_angular_vel'])
        self.assertEqual(smoother['max_velocity'][2], follow['rotate_to_heading_angular_vel'])

    def test_no_dwb_key_is_left(self):
        follow = field(vendor_params())['controller_server']['ros__parameters']['FollowPath']
        dwb = vendor_params()['controller_server']['ros__parameters']['FollowPath']
        self.assertFalse({'critics', 'vx_samples', 'sim_time', 'GoalDist.scale', 'GoalAlign.scale'} & set(follow))
        self.assertFalse((set(dwb) - {'plugin'}) & set(follow), set(dwb) & set(follow))

    def test_values_stay_within_the_dwb_limits(self):
        dwb = vendor_params()['controller_server']['ros__parameters']['FollowPath']
        follow = field(vendor_params())['controller_server']['ros__parameters']['FollowPath']
        self.assertLessEqual(follow['desired_linear_vel'], dwb['max_vel_x'])
        self.assertEqual(dwb['max_vel_theta'], follow['rotate_to_heading_angular_vel'])
        self.assertLessEqual(follow['max_angular_accel'], dwb['acc_lim_theta'])
        self.assertTrue(0.3 <= follow['min_lookahead_dist'] <= follow['lookahead_dist']
                        <= follow['max_lookahead_dist'] <= 0.6)
        self.assertAlmostEqual(0.785, follow['angular_dist_threshold'])

    def test_controller_server_common_values_are_kept(self):
        before = vendor_params()['controller_server']['ros__parameters']
        after = field(vendor_params())['controller_server']['ros__parameters']
        for key in set(before) - {'FollowPath'}:
            with self.subTest(key=key):
                self.assertEqual(before[key], after[key])
        self.assertIs(True, after['enable_stamped_cmd_vel'])

    def test_backup_removal_still_happens(self):
        params = field(vendor_params())
        self.assertNotIn('backup', params['behavior_server']['ros__parameters']['behavior_plugins'])
        self.assertIn('no_backup', params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])

    def test_unexpected_controller_parameters_are_refused(self):
        from rescue_bringup.nav2_field import Nav2FieldError
        cases = {
            'no controller_server': lambda p: p.pop('controller_server'),
            'FollowPath not a controller plugin': lambda p: p['controller_server']['ros__parameters'].update(
                controller_plugins=['Other']),
            'FollowPath already another controller': lambda p: p['controller_server']['ros__parameters'][
                'FollowPath'].update(plugin='nav2_mppi_controller::MPPIController'),
        }
        for name, change in cases.items():
            params = vendor_params()
            change(params)
            with self.subTest(name), self.assertRaises(Nav2FieldError):
                field(params)

    def test_the_input_is_not_changed(self):
        params = vendor_params()
        field(params)
        self.assertEqual('dwb_core::DWBLocalPlanner', params['controller_server']['ros__parameters']['FollowPath']['plugin'])


class Launches(unittest.TestCase):
    def test_every_nav2_launch_uses_the_field_parameters(self):
        launch = REPO / 'src' / 'rescue_bringup' / 'launch'
        if not launch.exists():
            self.skipTest('installed-wheel checkout: no launch sources')
        starting = [p for p in sorted(launch.glob('*.launch.py')) if "'nav2.launch.py'" in p.read_text()]
        self.assertEqual(3, len(starting))
        for path in starting:
            with self.subTest(path.name):
                self.assertIn('field_nav2', path.read_text())

    def test_baseline_is_not_the_field_controller(self):
        # config/robots keeps the TurtleBot4 nav2.yaml baseline; the field controller is made at launch only
        baseline = load_yaml(REPO / 'config' / 'robots' / 'turtlebot4_nav2.baseline.yaml')
        self.assertEqual('dwb_core::DWBLocalPlanner',
                         baseline['controller_server']['ros__parameters']['FollowPath']['plugin'])


if __name__ == '__main__':
    unittest.main()
