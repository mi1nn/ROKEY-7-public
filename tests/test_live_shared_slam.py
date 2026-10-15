"""Live AMR1 map -> AMR2 localization/navigation, synthetic E2 inputs (2026-10-14; 방식 B start_align seed 2026-10-15)."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from rescue_bringup import field_launch as launch
from rescue_bringup.profile import load_deployment
from rescue_contracts import RescueError
from rescue_robot.live_parameters import frozen_revision
from rescue_robot.live_sources import Grid, SavedMapSources
from test_bringup_field_launch import FIELD, NOW_NS, REPO, PLACEHOLDERS, field_profile
from test_robot_live_map_lifecycle import Runtime, grid_message, pose_message, settings
from test_coordination_approach import pose_msg
from test_coordination_map_freeze import FakeGrid, map_state
from test_coordination_ros_core import SCOUT, SUPPORT, location, make_core, receipt, site_msg, status_msg
from test_coordination_search import NOW

S = 1_000_000_000


def live_field():
    raw = json.loads(FIELD.read_text())
    raw['coordinator']['map_mode'] = 'SLAM_LIVE'
    raw['coordinator']['live_map_max_age_s'] = 10.0
    raw['robots']['robot_a']['task.saved_map.mode'] = 'SLAM_LIVE'
    raw['coordinator'].update(map_correction_m=0.15, map_correction_rad=0.087)
    raw['robots']['robot_b']['task.saved_map.mode'] = 'LIVE_MAP_AMCL'
    for c in raw['robots'].values():
        c['task.saved_map.live_max_age_ns'] = 10 * S
    return raw


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.profile = load_deployment(field_profile(self.folder))

    def field(self, raw=None):
        path = self.folder / 'live.json'
        path.write_text(json.dumps(raw or live_field()))
        return launch.load_field(path)

    def test_support_task_and_localizer_read_the_scout_map_directly(self):
        raw = live_field()
        raw['robots']['robot_b']['initial_pose_xy_yaw'] = [1.0, 2.0, 0.0]  # synthetic fixture
        field = self.field(raw)
        values = launch.robot_launch_values(self.profile, field=field, robot_id='robot_b',
                                            occupancy_sha256='', now_ns=NOW_NS)
        # TB-6's live map through TB-5's own-body relay (self_clear_map), same coordinates
        self.assertEqual('/robot5/rescue/shared_map', values['task_input']['task.saved_map.map_topic'])
        self.assertEqual('/robot5/rescue/shared_map', values['map_topic'])
        self.assertEqual('/robot5/tf', values['task_input']['tf_topic'])
        loc = launch.live_localization_values(self.profile, field=field, robot_id='robot_b')
        self.assertEqual(('/robot5', '/robot5/rescue/shared_map'), (loc['namespace'], loc['map_topic']))
        self.assertEqual(('/robot6/map', '/robot5/rescue/shared_map'),
                         (loc['self_clear']['input_topic'], loc['self_clear']['output_topic']))
        self.assertEqual([1.0, 2.0, 0.0], loc['initial_pose_xy_yaw'])
        self.assertEqual('/robot5/rescue/shared_map', loc['start_align']['map_topic'])
        # start_align's gate output is exactly the executor's localization input
        self.assertEqual(values['task_input']['task.saved_map.localization_topic'],
                         loc['start_align']['output_pose_topic'])
        self.assertEqual('/robot5/localized_pose', values['task_input']['task.saved_map.localization_topic'])

    def test_live_amcl_accepts_repeated_maps(self):
        params = launch.amcl_parameters({'amcl': {'ros__parameters': {}}},
                                       initial_pose_xy_yaw=[1., 2., 0.], use_sim_time=False,
                                       map_topic='/robot6/map', live=True)
        self.assertEqual('/robot6/map', params['map_topic'])
        self.assertFalse(params['first_map_only'])

    def test_no_invented_support_initial_pose(self):
        with self.assertRaisesRegex(launch.FieldLaunchError, 'never made up'):
            launch.live_localization_values(self.profile, field=self.field(), robot_id='robot_b')
        loc = launch.live_localization_values(self.profile, field=self.field(), robot_id='robot_b',
                                              start_xy_yaw=[0.1, 1.9, 0.05])
        self.assertEqual(([0.1, 1.9, 0.05], 'launch argument'), (loc['initial_pose_xy_yaw'], loc['start_source']))
        with self.assertRaises(launch.FieldLaunchError):
            launch.live_localization_values(self.profile, field=self.field(), robot_id='robot_b',
                                            start_xy_yaw=[0.1, float('nan'), 0.0])

    def test_modes_cannot_mix_live_and_frozen_maps(self):
        raw = live_field()
        raw['robots']['robot_b']['task.saved_map.mode'] = 'FROZEN_MAP_AMCL'
        with self.assertRaises(launch.FieldLaunchError):
            self.field(raw)

    def test_frozen_field_stays_and_the_live_overlay_switches_both_robots(self):
        frozen = launch.load_field(REPO / 'config/field/search_area.field.json')   # 방식 A unchanged
        self.assertEqual('SLAM_FREEZE', frozen.coordinator['map_mode'])
        live = launch.load_field(REPO / 'config/field/live_map.overlay.json')
        self.assertEqual(('SLAM_LIVE', 'EXIT_WAIT_SCOUT_DOCK'),
                         (live.coordinator['map_mode'], live.coordinator['completion_mode']))
        self.assertEqual('SLAM_LIVE', live.robots['robot_a']['task.saved_map.mode'])
        self.assertEqual('LIVE_MAP_AMCL', live.robots['robot_b']['task.saved_map.mode'])
        self.assertNotIn('initial_pose_xy_yaw', live.robots['robot_b'])   # measured at the site, not invented

    def test_live_amcl_with_seed_broadcasts_tf_and_takes_every_map(self):
        raw = {'amcl': {'ros__parameters': {'scan_topic': 'scan'}}}
        params = launch.amcl_parameters(raw, initial_pose_xy_yaw=[1., 2., 0.5], use_sim_time=False,
                                        map_topic='/robot6/map', live=True)
        self.assertTrue(params['set_initial_pose'] and params['tf_broadcast'])
        self.assertFalse(params['first_map_only'])
        self.assertEqual({'x': 1., 'y': 2., 'z': 0., 'yaw': 0.5}, params['initial_pose'])
        self.assertEqual('scan', params['scan_topic'])

    def test_nav2_static_layer_reads_live_map_and_keeps_other_layers(self):
        raw = {'global_costmap': {'global_costmap': {'ros__parameters': {
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer'},
            'obstacle_layer': {'enabled': True}}}}}
        result = launch.nav2_shared_map(raw, '/robot6/map')
        params = result['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual('/robot6/map', params['static_layer']['map_topic'])
        self.assertTrue(params['static_layer']['map_subscribe_transient_local'])
        self.assertFalse(params['static_layer']['subscribe_to_updates'])
        self.assertTrue(params['obstacle_layer']['enabled'])
        self.assertNotIn('map_topic', raw['global_costmap']['global_costmap']['ros__parameters']['static_layer'])

    def test_coordinator_map_topic_follows_the_profile_namespace(self):
        params = launch.coordinator_parameters(self.profile, profile_path='profile.json', field=self.field(),
                                                **PLACEHOLDERS)
        self.assertEqual('/robot6/map', params['live_map_topic'])

    def test_task_input_reads_live_launch_values_without_ros(self):
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        import sys
        from test_robot_live_parameters_launch import FakeNode, fake_ros
        from rescue_robot.live_parameters import read_config
        for robot in ('robot_a', 'robot_b'):
            values = launch.robot_launch_values(self.profile, field=self.field(), robot_id=robot,
                                                occupancy_sha256='', now_ns=NOW_NS)['task_input']
            adapter = NS(robot_id=robot, executor=NS(authority_epoch='66666666-6666-4666-8666-666666666666'))
            with patch.dict(sys.modules, fake_ros()):
                settings = read_config(FakeNode(values), adapter)['saved_maps'][robot]
            self.assertEqual(10*S, settings['live_max_age_ns'])
            self.assertNotIn('frozen_topic', settings)

    def test_live_wiring_has_no_frozen_map_service_or_support_slam(self):
        import importlib.util
        path = REPO / 'scripts/field/wiring_check.py'
        if not path.exists():
            self.skipTest('repository wiring script absent in installed checkout')
        spec = importlib.util.spec_from_file_location('co_map_wiring', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        links = module.expected_links(self.profile, self.field(), now_ns=NOW_NS)
        names = {row.name for row in links}
        self.assertNotIn('/rescue/map/freeze', names)
        self.assertNotIn('/rescue/map/frozen', names)
        self.assertIn('/robot5/localized_pose', names)
        row = next(row for row in links if row.name == '/robot6/map')
        self.assertEqual(('/robot6/slam_toolbox',), row.writers)
        self.assertIn('/robot5/rescue_self_clear_map', row.readers)
        row = next(row for row in links if row.name == '/robot5/rescue/shared_map')
        self.assertEqual(('/robot5/rescue_self_clear_map',), row.writers)
        self.assertIn('/robot5/amcl', row.readers)

    def test_scout_can_keep_slam_and_has_a_current_revision_home(self):
        field = self.field()
        self.assertEqual('/robot6', launch.slam_launch_values(self.profile, field=field,
                                                            robot_id='robot_a')['namespace'])
        self.assertEqual(0, frozen_revision({'mode': 'SLAM_LIVE', 'map_revision': 0}))


class SourceTests(unittest.TestCase):
    def source(self, mode='LIVE_MAP_AMCL'):
        self.r = Runtime()
        c = settings('SLAM_THEN_FROZEN' if mode == 'SLAM_LIVE' else mode, 'robot_b')
        c['robot_b'].update(mode=mode, map_topic='/robot6/map', live_max_age_ns=10*S)
        self.s = SavedMapSources(self.r, c)
        return self.s

    def message(self, cells=None):
        msg = grid_message(cells)
        msg.header.stamp = pose_message(self.r.now).header.stamp
        return msg

    def test_map_updates_while_moving_without_freeze_or_revision_adoption(self):
        s = self.source()
        s.receive_map('robot_b', self.message())
        self.r.now += S
        s.receive_pose('robot_b', pose_message(self.r.now))
        self.r.adapter.executor.active = object()
        self.r.adapter.executor.motion = 'MOVING'
        before = s.maps['robot_b'].sha256
        s.receive_map('robot_b', self.message({0: 100}))
        self.assertNotEqual(before, s.maps['robot_b'].sha256)
        self.assertEqual((0, 2), (s.basis('robot_b')['map_revision'], s.basis('robot_b')['map_update_sequence']))
        self.assertFalse(s.pending('robot_b'))
        s.check_new_goal('robot_b')
        self.assertEqual({}, s.adopted)

    def test_stale_or_missing_map_blocks_localization_and_new_goals(self):
        s = self.source()
        with self.assertRaises(RescueError):
            s.check_new_goal('robot_b')
        s.receive_map('robot_b', self.message())
        self.r.now += 11*S
        s.receive_pose('robot_b', pose_message(self.r.now))
        self.assertFalse(s.loaded('robot_b'))
        self.assertEqual('UNKNOWN', s.records()['robot_b']['localization_status'])
        with self.assertRaises(RescueError):
            s.check_new_goal('robot_b')

    def test_replayed_old_map_does_not_extend_freshness(self):
        s = self.source()
        msg = self.message()
        s.receive_map('robot_b', msg)
        self.r.now += 11*S
        s.receive_map('robot_b', msg)
        self.assertFalse(s.loaded('robot_b'))

    def test_scout_grid_is_available_for_return_home_while_slam_continues(self):
        s = self.source('SLAM_LIVE')
        s.receive_map('robot_b', self.message())
        self.r.now += S
        s.receive_pose('robot_b', pose_message(self.r.now), source='slam')
        self.assertEqual('SLAM', s.phase('robot_b'))
        self.assertIn('robot_b', s.maps)
        self.assertEqual('VALID', s.records()['robot_b']['localization_status'])

    def test_invalid_map_removes_last_good_grid(self):
        s = self.source()
        s.receive_map('robot_b', self.message())
        wrong = self.message()
        wrong.header.frame_id = 'odom'
        s.receive_map('robot_b', wrong)
        self.assertFalse(s.loaded('robot_b'))

    def test_map_expansion_and_origin_change_do_not_make_a_freeze_revision(self):
        s = self.source()
        s.receive_map('robot_b', self.message())
        expanded = grid_message(width=30, height=20)
        expanded.info.origin.position.x = -1.0
        expanded.header.stamp = pose_message(self.r.now).header.stamp
        s.receive_map('robot_b', expanded)
        self.assertEqual((30, 20), (s.maps['robot_b'].width, s.maps['robot_b'].height))
        self.assertEqual(-1.0, s.maps['robot_b'].origin[0])
        self.assertEqual((0, 2), (s.basis('robot_b')['map_revision'], s.basis('robot_b')['map_update_sequence']))


class CoordinatorTests(unittest.TestCase):
    def core(self):
        c = make_core(map_mode='SLAM_LIVE', live_map_max_age_ns=10*S, support_dispatch='AUTO')
        c.on_site_event(site_msg())
        c.on_status(status_msg(SCOUT, observed_at_ns=NOW-10**8, motion_state='MOVING'))
        c.on_status(status_msg(SUPPORT, observed_at_ns=NOW-10**8))
        c.on_map_state(map_state(SUPPORT, revision=3))
        c.on_pose(pose_msg(map_revision=3))
        c.on_target_location(location(1), now_ns=NOW)
        for report in c.reports_due(NOW):
            c.on_receipt(receipt(report))
        return c

    def test_dispatch_on_live_map_with_scout_moving_and_no_freeze(self):
        c = self.core()
        c.on_scout_map(FakeGrid(), now_ns=NOW)
        sent = c.tick(NOW)
        self.assertEqual(['APPROACH_TARGET'], [t['kind'] for t in sent])
        self.assertFalse(c.take_freeze_request())
        self.assertEqual(3, sent[0]['route']['approach_goal'].map_revision)

    def test_missing_stale_and_blocked_live_map_prevent_dispatch(self):
        for case in ('missing', 'stale', 'blocked'):
            with self.subTest(case=case):
                c = self.core()
                if case != 'missing':
                    c.on_scout_map(FakeGrid(blocked=[(3.0, 4.0)] if case == 'blocked' else ()),
                                   now_ns=NOW-11*S if case == 'stale' else NOW)
                self.assertEqual([], c.tick(NOW))

    def test_exploration_complete_returns_scout_without_either_freeze(self):
        from types import SimpleNamespace as NS
        from test_coordination_two_freezes import core_
        from test_coordination_frontier import scout_pose
        from test_coordination_mission_end import RealGrid
        from test_coordination_ros_core import admission_msg, request
        from test_coordination_search import uid
        c = core_(map_mode='SLAM_LIVE', live_map_max_age_ns=10*S, completion_mode='EXIT_ARRIVAL')
        c.on_map_state(map_state(SCOUT, revision=3))
        c.on_start_pose((0., 0., 0.))
        c.on_site_event(site_msg())
        c.on_status(status_msg(SCOUT))
        c.on_status(status_msg(SUPPORT))
        c.on_command(request('START_SEARCH', command_id=uid(690), robot_id=SCOUT), now_ns=NOW)
        sent = []
        for k in range(2):
            c.on_scout_map(RealGrid(), now_ns=NOW+k)
            c.on_pose(scout_pose(at=NOW+k))
            c.on_status(status_msg(SCOUT, observed_at_ns=NOW+k))
            sent += c.tick(NOW+k)
            self.assertFalse(c.take_freeze_request())
        self.assertTrue(c.exploration_complete)
        self.assertEqual(0, c.frozen_stage)
        sent += c.tick(NOW+2)  # return decision follows the exploration-complete transition
        self.assertEqual([(SCOUT, 'RETURN_HOME')], [(t['robot_id'], t['kind']) for t in sent])
        c.on_admission(admission_msg(sent[0]))
        c.on_result(sent[0]['task_id'], NS(outcome='COMPLETED', stop_confirmed=True, terminal_reason=''))
        c.system_state_change(NOW+2)
        self.assertEqual(('FINISHED', 'NO_SURVIVOR_FOUND'), (c.last_state['phase'], c.last_state['reason']))

    def test_support_waiting_spot_is_the_first_localized_pose(self):
        from types import SimpleNamespace as NS
        c = self.core()
        c.last_tick_ns = NOW
        first = pose_msg((0., 4.), map_revision=3)
        first.mission_pose.pose.pose.orientation = NS(x=0., y=0., z=0., w=1.)
        c.on_pose(first)
        c.on_pose(pose_msg((3., 4.), map_revision=3))
        self.assertEqual((0., 4., 0.), c.support_start_map_pose)


if __name__ == '__main__':
    unittest.main()
