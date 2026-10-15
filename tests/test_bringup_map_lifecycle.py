"""Field launch values for the map lifecycle: TB-6 SLAM -> frozen map -> both robots AMCL (no ROS).

Decision 2026-10-13 evening (scope Q-29 ⑦, OPS-113/117/124): each robot's map source is in the
field run file (robots.<id>.task.saved_map.mode); no default. TB-6 (robot_a) SLAM_THEN_FROZEN, TB-5
(robot_b) FROZEN_MAP_AMCL. The TB-5 AMCL start comes from robots.robot_b.initial_pose_xy_yaw
(clicked on the TB-6 map in a trial run and read back from /robot5/amcl_pose with
scripts/field/capture_initial_pose.py); until it is measured the TB-5 localization launch
refuses to start. The TB-6 AMCL start is the frozen pose (/rescue/map/frozen_pose/robot_a).
Saved maps (SAVED_MAP_AMCL) stay for tests without robots and the #132 field script.
"""
import json
import math
from pathlib import Path
import runpy
import sys
import unittest

from test_bringup_field_launch import FIELD, NOW_NS, REPO, SHA, FieldCase, HomePoses, saved_maps, tb5_depth, unmeasured

START = [0.4, -0.3, 0.1]
LAUNCH = REPO / 'src' / 'rescue_bringup' / 'launch' / 'frozen_localization.launch.py'
CAPTURE = REPO / 'scripts' / 'field' / 'capture_initial_pose.py'


def frozen_field(raw):
    raw['robots']['robot_b']['initial_pose_xy_yaw'] = list(START)


class MapMode(FieldCase):
    def test_repository_field_file_is_the_slam_freeze_flow(self):
        raw = json.loads(FIELD.read_text())
        self.assertEqual('SLAM_THEN_FROZEN', raw['robots']['robot_a']['task.saved_map.mode'])
        self.assertEqual('FROZEN_MAP_AMCL', raw['robots']['robot_b']['task.saved_map.mode'])
        # The TB-5 start is measured, never made up: 1.86 m right of TB-6 (재범 with a tape, 2026-10-15 17:05).
        self.assertEqual([0.0, -1.86, 0.0], raw['robots']['robot_b']['initial_pose_xy_yaw'])

    def test_tb6_task_input_searches_on_slam_without_a_map_hash(self):
        values = self.robot(field=self.field(), occupancy_sha256='')
        task = values['task_input']
        self.assertEqual('SLAM_THEN_FROZEN', task['task.saved_map.mode'])
        self.assertEqual('', task['task.saved_map.occupancy_sha256'])
        self.assertEqual('/robot6/pose', task['task.saved_map.slam_localization_topic'])
        self.assertEqual('/robot6/amcl_pose', task['task.saved_map.localization_topic'])
        self.assertEqual('/robot6/map', task['task.saved_map.map_topic'])
        # only task_input parameters are passed: the field file's map keys stay in the launch
        self.assertFalse({'initial_pose_xy_yaw', 'map_freeze.pose_max_age_ns'} & set(task))

    def test_tb5_task_input_waits_for_the_frozen_map(self):
        task = self.robot(field=self.field(tb5_depth), robot_id='robot_b', occupancy_sha256='')['task_input']
        self.assertEqual('FROZEN_MAP_AMCL', task['task.saved_map.mode'])
        self.assertNotIn('task.saved_map.slam_localization_topic', task)

    def test_a_map_hash_is_refused_where_the_freeze_decides_it(self):
        from rescue_bringup.field_launch import FieldLaunchError
        for robot_id in ('robot_a', 'robot_b'):
            with self.subTest(robot_id=robot_id), self.assertRaisesRegex(FieldLaunchError, 'freeze'):
                self.robot(field=self.field(), robot_id=robot_id, occupancy_sha256=SHA)

    def test_saved_map_source_still_needs_the_measured_hash(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def saved(raw):
            raw['coordinator']['map_mode'] = 'SAVED_MAP'
            for robot in raw['robots'].values():
                robot['task.saved_map.mode'] = 'SAVED_MAP_AMCL'
                robot.pop('map_freeze.pose_max_age_ns', None)
                for key in [k for k in robot if k == 'initial_pose_xy_yaw' or k.startswith('start_align.')]:
                    robot.pop(key)   # FROZEN_MAP_AMCL only
                robot.pop('task.saved_map.slam_covariance_max_age_ns', None)
        field = self.field(saved)
        self.assertEqual('SAVED_MAP_AMCL', self.robot(field=field)['task_input']['task.saved_map.mode'])
        with self.assertRaises(FieldLaunchError):
            self.robot(field=field, occupancy_sha256='')

    def test_map_source_rules_in_the_field_file(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def drop(key, robot='robot_a'):
            return lambda raw: raw['robots'][robot].pop(key)

        def put(key, value, robot='robot_a'):
            return lambda raw: raw['robots'][robot].__setitem__(key, value)
        bad = {
            'no map source': drop('task.saved_map.mode'),
            'unknown map source': put('task.saved_map.mode', 'SLAM'),
            'two SLAM robots': put('task.saved_map.mode', 'SLAM_THEN_FROZEN', 'robot_b'),
            'frozen map without a SLAM robot': put('task.saved_map.mode', 'FROZEN_MAP_AMCL'),
            'saved and frozen mixed': put('task.saved_map.mode', 'SAVED_MAP_AMCL', 'robot_b'),
            'SLAM robot without freeze pose age': drop('map_freeze.pose_max_age_ns'),
            'freeze pose age not positive': put('map_freeze.pose_max_age_ns', 0),
            'freeze pose age on a non-SLAM robot': put('map_freeze.pose_max_age_ns', 1, 'robot_b'),
            'start pose on the SLAM robot': put('initial_pose_xy_yaw', [0.0, 0.0, 0.0]),
            'start pose not x, y, yaw': put('initial_pose_xy_yaw', [0.0, 0.0], 'robot_b'),
            'start pose not finite numbers': put('initial_pose_xy_yaw', [0.0, 0.0, 'x'], 'robot_b'),
        }
        for name, change in bad.items():
            with self.subTest(name), self.assertRaises(FieldLaunchError):
                self.field(change)


class TB5ExitFromTheCoordinator(FieldCase):
    """SLAM_FREEZE: the TB-5 exit is the QR exit TB-6 finds; the coordinator sends it as destination
    'home/robot_b' (OPS-139). A launch-time profile home would be used until the first coordinator
    record, so the FROZEN_MAP_AMCL robot gets none; TB-6 keeps its docking approach point."""
    with_homes = HomePoses.with_homes

    def test_frozen_map_robot_has_no_launch_home_and_the_slam_robot_keeps_its_own(self):
        self.with_homes({'robot_b': [3.0, -4.0, 0.0], 'robot_a': [1.0, 2.0, 0.5]})
        b = self.robot(field=self.field(tb5_depth), robot_id='robot_b', occupancy_sha256='')['task_input']
        a = self.robot(field=self.field(), occupancy_sha256='')['task_input']
        self.assertNotIn('task.live.home_xy_yaw', b)
        self.assertEqual([1.0, 2.0, 0.5], a['task.live.home_xy_yaw'])

    def test_saved_maps_keep_both_launch_homes(self):
        self.with_homes({'robot_b': [3.0, -4.0, 0.0], 'robot_a': [1.0, 2.0, 0.5]})
        field = self.field(lambda raw: (saved_maps(raw), tb5_depth(raw)))
        self.assertEqual([3.0, -4.0, 0.0], self.robot(field=field, robot_id='robot_b')['task_input']['task.live.home_xy_yaw'])


class CoordinatorRobotPairing(FieldCase):
    """coordinator.map_mode and the robots' map modes are one decision (OPS-126): a mismatch starts nothing."""

    def test_repository_field_file_pairs_slam_freeze_with_slam_then_frozen_robots(self):
        raw = json.loads(FIELD.read_text())
        self.assertEqual('SLAM_FREEZE', raw['coordinator']['map_mode'])
        self.field()

    def test_saved_map_pairs_with_saved_map_robots_only(self):
        def saved(raw):
            raw['coordinator']['map_mode'] = 'SAVED_MAP'
            for robot in raw['robots'].values():
                robot['task.saved_map.mode'] = 'SAVED_MAP_AMCL'
                robot.pop('map_freeze.pose_max_age_ns', None)
                for key in [k for k in robot if k == 'initial_pose_xy_yaw' or k.startswith('start_align.')]:
                    robot.pop(key)   # FROZEN_MAP_AMCL only
                robot.pop('task.saved_map.slam_covariance_max_age_ns', None)
        self.field(saved)

    def test_mismatches_are_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def coordinator_saved(raw):
            raw['coordinator']['map_mode'] = 'SAVED_MAP'

        def robots_saved(raw):
            for robot in raw['robots'].values():
                robot['task.saved_map.mode'] = 'SAVED_MAP_AMCL'
                robot.pop('map_freeze.pose_max_age_ns', None)
                for key in [k for k in robot if k == 'initial_pose_xy_yaw' or k.startswith('start_align.')]:
                    robot.pop(key)   # FROZEN_MAP_AMCL only
                robot.pop('task.saved_map.slam_covariance_max_age_ns', None)

        def no_frozen_robot(raw):
            raw['robots'].pop('robot_b')
        for name, change in (('SAVED_MAP coordinator, SLAM robots', coordinator_saved),
                             ('SLAM_FREEZE coordinator, saved-map robots', robots_saved),
                             ('SLAM_FREEZE without a robot on the frozen map', no_frozen_robot)):
            with self.subTest(name), self.assertRaisesRegex(FieldLaunchError, 'map_mode'):
                self.field(change)


class LocalizationLaunch(FieldCase):
    def values(self, robot_id, field=None):
        from rescue_bringup.field_launch import frozen_localization_values
        return frozen_localization_values(self.deployment, field=field or self.field(frozen_field), robot_id=robot_id)

    def test_tb5_starts_amcl_from_the_measured_start_on_the_frozen_map(self):
        v = self.values('robot_b')
        self.assertEqual(('/robot5', '/robot5/map', '/rescue/map/frozen'),
                         (v['namespace'], v['map_topic'], v['frozen_topic']))
        self.assertEqual(START, v['initial_pose_xy_yaw'])
        self.assertIsNone(v['initial_pose_topic'])

    def test_tb6_starts_amcl_from_the_frozen_pose(self):
        v = self.values('robot_a')
        self.assertEqual('/robot6', v['namespace'])
        self.assertIsNone(v['initial_pose_xy_yaw'])
        self.assertEqual('/rescue/map/frozen_pose/robot_a', v['initial_pose_topic'])

    def test_tb5_without_a_measured_start_does_not_start(self):
        from rescue_bringup.field_launch import FieldLaunchError
        with self.assertRaisesRegex(FieldLaunchError, 'capture_initial_pose'):
            self.values('robot_b', field=self.field(unmeasured))

    def test_saved_map_robots_have_no_frozen_localization(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def saved(raw):
            raw['coordinator']['map_mode'] = 'SAVED_MAP'
            for robot in raw['robots'].values():
                robot['task.saved_map.mode'] = 'SAVED_MAP_AMCL'
                robot.pop('map_freeze.pose_max_age_ns', None)
                for key in [k for k in robot if k == 'initial_pose_xy_yaw' or k.startswith('start_align.')]:
                    robot.pop(key)   # FROZEN_MAP_AMCL only
                robot.pop('task.saved_map.slam_covariance_max_age_ns', None)
        with self.assertRaises(FieldLaunchError):
            self.values('robot_a', field=self.field(saved))

    def test_slam_launch_only_for_the_slam_robot(self):
        from rescue_bringup.field_launch import FieldLaunchError, slam_launch_values
        self.assertEqual('/robot6', slam_launch_values(self.deployment, field=self.field(), robot_id='robot_a')['namespace'])
        with self.assertRaises(FieldLaunchError):
            slam_launch_values(self.deployment, field=self.field(), robot_id='robot_b')

    def test_amcl_parameters_keep_the_turtlebot4_values_and_set_the_start(self):
        from rescue_bringup.field_launch import amcl_parameters
        base = {'amcl': {'ros__parameters': {'alpha1': 0.2, 'scan_topic': 'scan', 'global_frame_id': 'map'}}}
        p = amcl_parameters(base, initial_pose_xy_yaw=[0.4, -0.3, 0.1], use_sim_time=False)
        self.assertEqual(0.2, p['alpha1'])
        self.assertEqual('scan', p['scan_topic'])
        self.assertIs(True, p['set_initial_pose'])
        self.assertEqual({'x': 0.4, 'y': -0.3, 'z': 0.0, 'yaw': 0.1}, p['initial_pose'])
        self.assertIs(False, p['use_sim_time'])
        with self.assertRaises(ValueError):
            amcl_parameters({}, initial_pose_xy_yaw=[0.0, 0.0, 0.0], use_sim_time=False)
        with self.assertRaises(ValueError):
            amcl_parameters(base, initial_pose_xy_yaw=[0.0, math.nan, 0.0], use_sim_time=False)


class FreezeLaunch(FieldCase):
    def params(self, **changes):
        from rescue_bringup.field_launch import map_freeze_parameters
        values = dict(output_dir='/srv/freeze') | changes
        return map_freeze_parameters(self.deployment, field=self.field(), **values)

    def test_freeze_reads_the_slam_robot_map_and_pose_on_the_profile_basis(self):
        p = self.params()
        basis = self.deployment.map_basis
        self.assertEqual({'map_topic': '/robot6/map', 'pose_topic': '/robot6/pose', 'map_frame': 'map',
                          'map_session_id': basis.map_session_id, 'map_revision': basis.map_revision,
                          'pose_max_age_ns': 2_000_000_000, 'output_dir': '/srv/freeze',
                          # TB-6 pose from TF map -> base_link like the executor (OPS-179)
                          'tf_topic': '/robot6/tf', 'odom_frame': 'odom', 'base_frame': 'base_link',
                          'covariance_max_age_ns': 30_000_000_000}, p)

    @unittest.expectedFailure
    def test_the_tb5_start_goes_to_the_freeze_to_clear_its_body(self):
        """C25: TB-5's body is cleared from each frozen map at robots.<TB-5>.initial_pose_xy_yaw (TB-6 start frame)."""
        p = self.params()
        self.assertEqual(('robot_b', [0.0, -1.86, 0.0]), (p['support_robot_id'], p['support_start_xy_yaw']))

    @unittest.expectedFailure
    def test_no_measured_tb5_start_gives_no_place_to_clear(self):
        from rescue_bringup.field_launch import map_freeze_parameters
        p = map_freeze_parameters(self.deployment, field=self.field(unmeasured), output_dir='/srv/freeze')
        self.assertEqual(('robot_b', []), (p['support_robot_id'], p['support_start_xy_yaw']))

    def test_output_dir_is_absolute_and_a_restart_reads_it(self):
        # two freezes per run (OPS-154): a restart reloads every record of the run from output_dir
        from rescue_bringup.field_launch import FieldLaunchError
        self.assertNotIn('record_path', self.params())
        for bad in (dict(output_dir=''), dict(output_dir='freeze')):
            with self.subTest(bad=bad), self.assertRaises(FieldLaunchError):
                self.params(**bad)


class OnePcTwoRoles(FieldCase):
    def test_both_robots_on_one_pc_use_separate_names(self):
        """Trial: compute_a and compute_b may be one PC (OPS-124 ③): every robot node is in its namespace."""
        a = self.robot(field=self.field(), occupancy_sha256='')
        b = self.robot(field=self.field(tb5_depth), robot_id='robot_b', occupancy_sha256='')
        self.assertNotEqual(a['namespace'], b['namespace'])
        for key in ('executor_heartbeat_topic', 'watchdog_heartbeat_topic', 'task.saved_map.map_topic',
                    'task.saved_map.localization_topic'):
            self.assertNotEqual(a['task_input'][key], b['task_input'][key])

    @unittest.skipUnless(LAUNCH.exists(), 'installed-wheel checkout has no src/ launch files')
    def test_frozen_localization_nodes_are_in_the_robot_namespace(self):
        self.assertIn('namespace=ns', LAUNCH.read_text())


@unittest.skipUnless(CAPTURE.exists(), 'installed-wheel checkout has no scripts/')
class CaptureInitialPose(unittest.TestCase):
    def tool(self):
        sys.path.insert(0, str(REPO / 'scripts' / 'field'))
        self.addCleanup(sys.path.remove, str(REPO / 'scripts' / 'field'))
        return runpy.run_path(str(CAPTURE))

    def origin(self):
        """TB-6 start pose at the map origin: relative values equal map values."""
        from types import SimpleNamespace as NS
        return NS(header=NS(frame_id='map'), pose=NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                  orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=[0.0] * 36))

    def message(self, frame='map', yaw=0.1, covariance=None):
        from types import SimpleNamespace as NS
        cov = covariance or [0.0] * 36
        return NS(header=NS(frame_id=frame), pose=NS(pose=NS(position=NS(x=0.4, y=-0.3, z=0.0),
                  orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))), covariance=cov))

    def test_prints_the_field_file_line_for_the_robot(self):
        snippet = self.tool()['snippet']
        # TB-6 start at the map origin (0, 0, 0): the relative value is the map value (OPS-141)
        text = snippet(self.message(), start=self.origin(), map_frame='map')
        self.assertEqual({'initial_pose_xy_yaw': [0.4, -0.3, 0.1]},
                         {k: [round(v, 6) for v in vals] for k, vals in json.loads('{' + text + '}').items()})

    def test_refuses_another_frame_or_a_non_planar_pose(self):
        tool = self.tool()
        with self.assertRaises(tool['CaptureError']):
            tool['snippet'](self.message(frame='odom'), start=self.origin(), map_frame='map')
        bad = self.message()
        bad.pose.pose.orientation.x = 0.3
        with self.assertRaises(tool['CaptureError']):
            tool['snippet'](bad, start=self.origin(), map_frame='map')


if __name__ == '__main__':
    unittest.main()


class SlamPoseFromTf(FieldCase):
    """TB-6 SLAM-phase pose from TF map -> base_link (OPS-177): what the launch hands the executor."""

    def test_tb6_task_input_names_its_tf_and_frames_and_the_covariance_age(self):
        task = self.robot(field=self.field(), occupancy_sha256='')['task_input']
        self.assertEqual(task['tf_topic'], task['task.saved_map.slam_tf_topic'])
        self.assertEqual(task['odom_frame'], task['task.saved_map.slam_odom_frame'])
        self.assertEqual('base_link', task['task.saved_map.slam_base_frame'])
        self.assertEqual(30_000_000_000, task['task.saved_map.slam_covariance_max_age_ns'])   # [proposal] OPS-177

    def test_only_the_slam_robot_and_a_positive_age(self):
        from rescue_bringup.field_launch import FieldLaunchError
        b = self.robot(field=self.field(), robot_id='robot_b', occupancy_sha256='')['task_input']
        self.assertFalse([k for k in b if k.startswith('task.saved_map.slam_')])
        key = 'task.saved_map.slam_covariance_max_age_ns'
        for change in (lambda raw: raw['robots']['robot_a'].pop(key), lambda raw: raw['robots']['robot_a'].update({key: 0}),
                       lambda raw: raw['robots']['robot_b'].update({key: 1})):
            with self.assertRaises(FieldLaunchError):
                self.field(change)


class TwoFreezes(FieldCase):
    """Map frozen twice (OPS-154/159): what the launches hand the executor and the relay."""

    def test_tb5_executor_gets_its_amcl_initialpose_topic_for_re_aligning(self):
        b = self.robot(field=self.field(), robot_id='robot_b', occupancy_sha256='')['task_input']
        self.assertEqual('/robot5/initialpose', b['task.saved_map.initial_pose_topic'])
        a = self.robot(field=self.field(), occupancy_sha256='')['task_input']
        self.assertNotIn('task.saved_map.initial_pose_topic', a)   # TB-6 starts AMCL from the frozen pose

    def test_relay_follows_the_revision_pair_and_its_own_map_state(self):
        from rescue_bringup.field_launch import frozen_localization_values
        for robot_id in ('robot_a', 'robot_b'):
            with self.subTest(robot_id=robot_id):
                v = frozen_localization_values(self.deployment, field=self.field(frozen_field), robot_id=robot_id)
                self.assertEqual('/rescue/map/frozen_revision', v['frozen_revision_topic'])
                self.assertEqual(f'/rescue/robots/{robot_id}/map_state', v['map_state_topic'])
