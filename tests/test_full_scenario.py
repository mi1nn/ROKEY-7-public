"""One-PC full scenario (방식 B, 2026-10-15): runner plan/verdict, overlay, dock, correction, alert, gate. No ROS."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from rescue_bringup import field_launch as launch
from rescue_bringup import full_scenario as fs
from rescue_bringup.profile import load_deployment
from rescue_operations.found_alert import APPROACH_COMPLETE, FoundAlert, chime_wav, play
from rescue_robot.dock_node import DockCore
from rescue_robot.map_correction import CorrectionWatch
from rescue_robot.slam_pose_latest import Latest
from rescue_robot.start_align import gate_pose, keepalive_due
from test_bringup_field_launch import REPO, field_profile

OVERLAY = REPO / 'config' / 'field' / 'live_map.overlay.json'
S = 1_000_000_000


class OverlayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.dir = Path(self.temp.name)

    def write(self, name, raw):
        path = self.dir / name
        path.write_text(json.dumps(raw))
        return path

    def test_overlay_changes_only_its_keys_and_the_base_stays_frozen(self):
        base = json.loads((REPO / 'config/field/search_area.field.json').read_text())
        live = launch.load_field(OVERLAY)
        self.assertEqual('SLAM_FREEZE', base['coordinator']['map_mode'])
        for key, value in base['coordinator'].items():
            if key not in ('map_mode', 'completion_mode'):
                self.assertEqual(value, live.coordinator[key], key)
        self.assertIn('overlay live_map.overlay.json', live.raw['source'])

    def test_null_removes_and_unknown_robot_or_section_is_refused(self):
        base = json.loads((REPO / 'config/field/search_area.field.json').read_text())
        self.write('base.json', base)
        overlay = json.loads(OVERLAY.read_text())
        overlay['base'] = 'base.json'
        overlay['set']['robots']['robot_b']['start_align.max_updates'] = None   # removed: then refused by the check
        with self.assertRaisesRegex(launch.FieldLaunchError, 'start_align.max_updates'):
            launch.load_field(self.write('o1.json', overlay))
        overlay = json.loads(OVERLAY.read_text())
        overlay['base'] = 'base.json'
        overlay['set']['robots']['robot_z'] = {'x': 1}
        with self.assertRaisesRegex(launch.FieldLaunchError, 'robot_z'):
            launch.load_field(self.write('o2.json', overlay))
        overlay['set'] = {'nonsense': {}}
        with self.assertRaisesRegex(launch.FieldLaunchError, 'may change only'):
            launch.load_field(self.write('o3.json', overlay))
        overlay = json.loads(OVERLAY.read_text())
        overlay['base'] = '../search_area.field.json'
        with self.assertRaisesRegex(launch.FieldLaunchError, 'next to the overlay'):
            launch.load_field(self.write('o4.json', overlay))

    def test_dock_completion_mode_needs_the_live_frontier_run(self):
        raw = launch.load_field(OVERLAY).raw
        raw['coordinator']['search_mode'] = 'AREAS'
        raw['coordinator']['traffic_control'] = 'DISABLED'
        raw['coordinator']['map_mode'] = 'SAVED_MAP'
        for section in raw['robots'].values():
            section['task.saved_map.mode'] = 'SAVED_MAP_AMCL'
            section.pop('task.saved_map.live_max_age_ns', None)
        with self.assertRaises(launch.FieldLaunchError):
            launch.load_field(self.write('f.json', raw))


class LaunchValueTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.profile = load_deployment(field_profile(Path(self.temp.name)))
        self.field = launch.load_field(OVERLAY)

    def test_dock_node_only_for_the_live_scout(self):
        dock = launch.dock_values(self.profile, field=self.field, robot_id='robot_a')
        self.assertEqual(('/robot6/dock', '/rescue/robots/robot_a/dock', '/rescue/robots/robot_a/dock_state'),
                         (dock['dock_action'], dock['service'], dock['state_topic']))
        self.assertLess(dock['result_timeout_s'] + dock['action_wait_s'] + dock['confirm_timeout_s'],
                        self.field.coordinator['scout_dock_timeout_s'] + 1e-9)
        self.assertIsNone(launch.dock_values(self.profile, field=self.field, robot_id='robot_b'))
        frozen = launch.load_field(REPO / 'config/field/search_area.field.json')
        self.assertIsNone(launch.dock_values(self.profile, field=frozen, robot_id='robot_a'))

    def test_single_host_only_when_asked(self):
        def values(**kw):
            return launch.robot_launch_values(self.profile, field=self.field, robot_id='robot_b', occupancy_sha256='',
                                              now_ns=10 ** 18, **kw)['task_input']
        self.assertNotIn('task.live.allow_same_host', values())
        self.assertIs(True, values(single_host=True)['task.live.allow_same_host'])
        with self.assertRaises(launch.FieldLaunchError):
            values(single_host='true')

    def test_scout_covariance_comes_from_the_repeated_slam_pose(self):
        task = launch.robot_launch_values(self.profile, field=self.field, robot_id='robot_a', occupancy_sha256='',
                                          now_ns=10 ** 18)['task_input']
        latest = launch.slam_pose_latest_values(self.profile, field=self.field)
        self.assertEqual(latest['output_topic'], task['task.saved_map.slam_localization_topic'])
        self.assertEqual('/robot6/pose', latest['input_topic'])
        frozen = launch.load_field(REPO / 'config/field/search_area.field.json')
        task = launch.robot_launch_values(self.profile, field=frozen, robot_id='robot_a', occupancy_sha256='',
                                          now_ns=10 ** 18)['task_input']
        self.assertEqual('/robot6/pose', task['task.saved_map.slam_localization_topic'])   # 방식 A unchanged

    def test_live_slam_processes_scans_while_standing(self):
        params = launch.live_slam_params({'slam_toolbox': {'ros__parameters': {'minimum_travel_distance': 0.1}}})
        p = params['slam_toolbox']['ros__parameters']
        self.assertEqual((0.0, 0.0, 1.0), (p['minimum_travel_distance'], p['minimum_travel_heading'],
                                           p['minimum_time_interval']))
        self.assertTrue(launch.slam_launch_values(self.profile, field=self.field, robot_id='robot_a')['live'])

    def test_correction_watch_reads_the_scout_tf(self):
        values = launch.map_correction_values(self.profile, field=self.field)
        self.assertEqual(('/robot6/tf', 0.15, 0.087), (values['tf_topic'], values['correction_m'],
                                                       values['correction_rad']))


class RunnerPlanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.dir = Path(self.temp.name)
        self.profile_path = field_profile(self.dir)
        self.deployment = load_deployment(self.profile_path)

    def options(self, **kw):
        return dict(dict(rviz=True, alert=True, overhead=True, rehearsal=False, sim_world=None, world=None,
                         rehearsal_perception=None, perception_args={'image_topic': '/robot6/oakd/rgb'},
                         coordinator_args={}, webcam_params=None, web=None, api_port=18765, python='python3',
                         envelope_schema_version='rescue-envelope/1', report_publisher_id='r'), **kw)

    def plan(self, **kw):
        return fs.stage_plan(profile=self.profile_path, field=self.dir / 'field.json', run_dir=self.dir, scout='robot_a',
                             support='robot_b', ns={'robot_a': '/robot6', 'robot_b': '/robot5'},
                             options=self.options(**kw))

    def test_dependency_order(self):
        order = [stage for stage, _ in self.plan()]
        self.assertEqual(['live_map', 'tb6_slam', 'passage', 'tb5_localization', 'robots', 'perception', 'operations',
                          'coordinator', 'alert', 'rviz'], order)
        # the recorder before SLAM, TB-5 AMCL before its Nav2, the coordinator after the robots it talks to
        self.assertLess(order.index('live_map'), order.index('tb6_slam'))
        self.assertLess(order.index('tb5_localization'), order.index('robots'))

    def test_both_robot_launches_are_single_host_and_options_drop_heavy_parts(self):
        plan = dict(self.plan(rviz=False, overhead=False, alert=False))
        self.assertNotIn('rviz', plan)
        self.assertEqual(['passage_robot_a', 'passage_robot_b'], [label for label, _ in plan['passage']])
        self.assertTrue(all('single_host:=true' in argv for _, argv in plan['robots']))
        self.assertIn('enabled:=false', plan['alert'][0][1])

    def test_rehearsal_replaces_only_robots_and_detector(self):
        plan = dict(self.plan(rehearsal=True, sim_world=Path('/x/sim_world.py'), world=Path('/x/world.json'),
                              rehearsal_perception=Path('/x/perception.launch.py')))
        self.assertIn('sim_world', plan)
        self.assertNotIn('passage', plan)
        self.assertIn('/x/perception.launch.py', plan['perception'][0][1])
        self.assertTrue(any('robot.launch.py' in argv for _, argv in plan['robots']))

    def test_seed_is_never_made_up(self):
        self.assertIsNone(fs.tb5_seed(None, {}))
        self.assertEqual([0.1, 1.9, 0.05], fs.tb5_seed(None, {'TB5_START_X': '0.1', 'TB5_START_Y': '1.9',
                                                              'TB5_START_YAW': '0.05'}))
        with self.assertRaises(fs.ScenarioError):
            fs.tb5_seed(None, {'TB5_START_X': '0.1'})
        with self.assertRaises(fs.ScenarioError):
            fs.run_field(OVERLAY, None)   # no seed anywhere
        raw, support = fs.run_field(OVERLAY, [0.1, 1.9, 0.05])
        self.assertEqual(('robot_b', [0.1, 1.9, 0.05]), (support, raw['robots']['robot_b']['initial_pose_xy_yaw']))
        with self.assertRaises(fs.ScenarioError):
            fs.run_field(REPO / 'config/field/search_area.field.json', [0.0, 1.0, 0.0])   # 방식 A is not this runner's

    def test_environment_checks(self):
        ok = {'ROS_DISTRO': 'jazzy', 'ROS_DOMAIN_ID': '7', 'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}
        self.assertTrue(all(c[1] for c in fs.env_checks(ok, rehearsal=False, domains={7})))
        self.assertFalse(all(c[1] for c in fs.env_checks(dict(ok, ROS_DOMAIN_ID='8'), rehearsal=False, domains={7})))
        self.assertFalse(all(c[1] for c in fs.env_checks(dict(ok, ROS_DOMAIN_ID=''), rehearsal=False, domains={7})))
        reh = dict(ok, ROS_DOMAIN_ID='195', ROS_DISCOVERY_SERVER='127.0.0.1:11889')
        self.assertTrue(all(c[1] for c in fs.env_checks(reh, rehearsal=True, domains={7})))
        # a rehearsal never joins a field Discovery Server
        self.assertFalse(all(c[1] for c in fs.env_checks(dict(reh, ROS_DISCOVERY_SERVER='192.168.0.5:11811'),
                                                         rehearsal=True, domains={7})))
        self.assertEqual({7}, fs.profile_domains(self.deployment))


class VerdictTests(unittest.TestCase):
    def finished(self, **kw):
        return dict(dict(mode='MISSION_FINISHED', exploration_reason='EXPLORATION_COMPLETE', scout_dock='DOCKED',
                         support_exit='AT_EXIT', no_survivor=False), **kw)

    def test_success_needs_dock_exit_and_a_complete_exploration(self):
        self.assertEqual('SUCCESS', fs.judge(self.finished())[0])
        self.assertEqual('SUCCESS', fs.judge(self.finished(support_exit='', no_survivor=True))[0])
        self.assertEqual('FINISHED_INCOMPLETE', fs.judge(self.finished(exploration_reason='EXPLORATION_TIMEOUT'))[0])
        self.assertEqual('FAILED', fs.judge(self.finished(scout_dock=''))[0])
        self.assertEqual('FAILED', fs.judge(self.finished(support_exit='WAITING'))[0])

    def test_failures_and_running(self):
        self.assertEqual(('RUNNING', ''), fs.judge({'mode': 'DISASTER'}))
        self.assertEqual('FAILED', fs.judge({'scout_dock': 'FAILED', 'scout_dock_reason': 'DOCK_TIMEOUT'})[0])
        self.assertEqual('FAILED', fs.judge({'approach': 'FAILED'})[0])
        self.assertEqual('FAILED', fs.judge({}, child_exits=[('robot_robot_a', 1)])[0])
        self.assertEqual('FAILED', fs.judge({}, timed_out=True)[0])
        self.assertIn('SUPPORT_TASK_AFTER_EXIT', fs.judge({}, support_task_after_exit='t1')[1])
        self.assertIn('NAV2_DOWN', fs.judge({'mode': 'DISASTER'}, nav2_down='robot_a')[1])

    def test_align_supervisor_restarts_only_retryable_failures(self):
        sup = fs.AlignSupervisor(restarts=2, gap_s=5.0)
        self.assertEqual('wait', sup.decide(None, 0))
        self.assertEqual('wait', sup.decide({'status': 'ALIGNING'}, 0))
        low = {'status': 'FAILED', 'reason': 'START_ALIGN_LOW_MATCH'}
        self.assertEqual('restart', sup.decide(low, 0))
        self.assertEqual('wait', sup.decide(low, 1))
        self.assertEqual('restart', sup.decide(low, 6))
        self.assertEqual('fail', sup.decide(low, 20)[0])
        self.assertEqual('fail', fs.AlignSupervisor().decide({'status': 'FAILED',
                                                               'reason': 'START_ALIGN_JUMP_TOO_LARGE'}, 0)[0])
        self.assertEqual('done', sup.decide({'status': 'ALIGNED'}, 30))


class DockTests(unittest.TestCase):
    def core(self):
        c = DockCore(status_max_age_ns=3 * S, dock_status_max_age_ns=3 * S, action_wait_ns=5 * S,
                     result_timeout_ns=100 * S, confirm_timeout_ns=5 * S)
        c.on_status('READY', 'STOPPED', '', 10 * S)
        c.on_dock_status(False, 10 * S)
        return c

    def test_request_gate(self):
        c = self.core()
        self.assertIsNone(c.refusal(11 * S))
        self.assertEqual('ROBOT_STATUS_STALE', c.refusal(20 * S))
        c.on_status('READY', 'MOVING', '', 11 * S)
        self.assertEqual('ROBOT_NOT_STOPPED_MOVING', c.refusal(11 * S))
        c.on_status('READY', 'STOPPED', 'task', 11 * S)
        self.assertEqual('ROBOT_TASK_ACTIVE', c.refusal(11 * S))
        c.on_status('READY', 'STOPPED', '', 11 * S)
        c.on_dock_status(True, 11 * S)
        self.assertEqual('ALREADY_DOCKED', c.refusal(11 * S))

    def test_docked_needs_the_result_and_a_later_dock_status(self):
        c = self.core()
        self.assertEqual(1, c.start(11 * S))
        self.assertEqual('DOCK_BUSY', c.refusal(11 * S))
        self.assertEqual('send', c.tick(11 * S, server_ready=True))
        c.on_goal_sent()
        c.on_goal_response(True)
        c.on_result(True, 20 * S)
        self.assertIsNone(c.tick(20 * S, server_ready=True))
        self.assertEqual('DOCKING', c.state)            # the result alone is not DOCKED
        c.on_dock_status(True, 19 * S)                    # older than the result: does not count
        c.tick(21 * S, server_ready=True)
        self.assertEqual('DOCKING', c.state)
        c.on_dock_status(True, 22 * S)
        c.tick(22 * S, server_ready=True)
        self.assertEqual(('DOCKED', True), (c.state, c.record(22 * S)['is_docked']))

    def test_failures_are_named_never_pretended(self):
        c = self.core()
        c.start(11 * S)
        c.tick(17 * S, server_ready=False)
        self.assertEqual(('FAILED', 'DOCK_ACTION_UNAVAILABLE'), (c.state, c.reason))
        c = self.core()
        c.start(11 * S)
        c.on_goal_sent()
        c.on_result(False, 20 * S)
        self.assertEqual('DOCK_RESULT_NOT_DOCKED', c.reason)
        c = self.core()
        c.start(11 * S)
        c.on_goal_sent()
        self.assertEqual('cancel', c.tick(112 * S, server_ready=True))
        self.assertEqual('DOCK_TIMEOUT', c.reason)
        c = self.core()
        c.start(11 * S)
        c.on_goal_sent()
        c.on_result(True, 20 * S)
        c.tick(26 * S, server_ready=True)
        self.assertEqual('DOCK_STATUS_NOT_CONFIRMED', c.reason)
        c = self.core()
        c.start(11 * S)
        self.assertTrue(c.cancel('DOCK_CANCELED'))
        self.assertFalse(c.cancel('DOCK_CANCELED'))


class CorrectionTests(unittest.TestCase):
    def test_small_drift_is_no_event_and_a_jump_is(self):
        w = CorrectionWatch(correction_m=0.15, correction_rad=0.087)
        w.on_odom_base((2.0, 0.0, 0.0))
        self.assertIsNone(w.on_map_odom((0.0, 0.0, 0.0), 1))
        self.assertIsNone(w.on_map_odom((0.01, 0.0, 0.005), 2))
        event = w.on_map_odom((0.01, 0.0, 0.105), 3)   # 0.1 rad about the odom origin moves TB-6 2 m away ~0.2 m
        self.assertIsNotNone(event)
        self.assertAlmostEqual(0.2, event['shift_m'], places=2)
        self.assertEqual(1, event['count'])
        with self.assertRaises(ValueError):
            CorrectionWatch(correction_m=0.0, correction_rad=0.1)


class AlertTests(unittest.TestCase):
    def test_once_per_found_target_per_mission(self):
        a = FoundAlert()
        self.assertEqual([], a.on_targets([('t1', APPROACH_COMPLETE)]))   # no mission yet
        a.on_trigger('m1')
        self.assertEqual([], a.on_targets([('t1', 'ASSIGNED'), ('t1', 'IN_PROGRESS')]))   # goal / start: no sound
        self.assertEqual([('m1', 't1')], a.on_targets([('t1', APPROACH_COMPLETE)]))
        self.assertEqual([], a.on_targets([('t1', APPROACH_COMPLETE)]))   # repeated snapshot
        a.on_trigger('m2')
        self.assertEqual([('m2', 't1')], a.on_targets([('t1', APPROACH_COMPLETE)]))   # next mission
        self.assertEqual([], FoundAlert(played=[('m2', 't1')]).on_targets([]))

    def test_player_absence_or_failure_is_a_reason(self):
        with tempfile.TemporaryDirectory() as folder:
            wav = chime_wav(Path(folder) / 'c.wav')
            self.assertGreater(wav.stat().st_size, 20000)
            self.assertEqual('NOT_PLAYED', play(wav, which=lambda _: None, run=None)[0])
            failing = lambda *a, **k: NS(returncode=1, stderr='no device')
            result = play(wav, which=lambda name: '/bin/' + name, run=failing)
            self.assertEqual('NOT_PLAYED', result[0])
            self.assertIn('no device', result[1])

            def boom(*a, **k):
                raise OSError('broken')
            self.assertEqual('NOT_PLAYED', play(wav, which=lambda name: '/bin/' + name, run=boom)[0])
            ok = lambda argv, **k: NS(returncode=0, stderr='')
            self.assertEqual(('PLAYED', 'paplay'), play(wav, which=lambda name: '/bin/' + name, run=ok))


class GateTests(unittest.TestCase):
    def test_pose_relayed_only_while_aligned_and_keepalive_only_standing(self):
        core = NS(status='ALIGNING', _stopped=lambda: True)
        self.assertFalse(gate_pose(core, 'map', 'map'))
        self.assertFalse(keepalive_due(core, 10, None))
        core.status = 'ALIGNED'
        self.assertTrue(gate_pose(core, 'map', 'map'))
        self.assertFalse(gate_pose(core, 'odom', 'map'))
        self.assertTrue(keepalive_due(core, 10, None))
        self.assertFalse(keepalive_due(core, 10, 11))
        core._stopped = lambda: False
        self.assertFalse(keepalive_due(core, 20, 11))

    def test_latest_slam_pose_keeps_its_own_stamp(self):
        latest = Latest()
        new = NS(header=NS(stamp=NS(sec=5, nanosec=0)))
        old = NS(header=NS(stamp=NS(sec=4, nanosec=0)))
        self.assertTrue(latest.on_pose(new))
        self.assertFalse(latest.on_pose(old))
        self.assertIs(new, latest.message)



class ChildrenCleanupTests(unittest.TestCase):
    """Rehearsal 2026-10-15 #22: the launch leader exited on SIGINT, a node it started (ignoring SIGINT) lived on in
    the group and was still on the graph in the next run. The whole group is stopped, not only the leader."""

    def test_a_grandchild_that_outlives_the_leader_is_stopped(self):
        with tempfile.TemporaryDirectory() as tmp:
            children = fs.Children(Path(tmp) / 'logs', lambda line: None)
            proc = children.start('launch', ['sh', '-c', '(trap "" INT; exec sleep 30) & trap "exit 0" INT; wait'])
            import time
            time.sleep(0.3)
            self.assertEqual([], children.stop_all(grace_s=0.5))
            self.assertFalse(fs.Children.group_alive(proc))


if __name__ == '__main__':
    unittest.main()
