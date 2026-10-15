"""scripts/field/wiring_check.py without ROS: the expected connections from the code, and the verdicts.

The graph reading (rclpy graph API only) is exercised by tests/ros/e3_wiring_check_probe.py.
"""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import re
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'wiring_check.py'
if not SCRIPT.is_file():
    # as tests/test_field_link_check.py: a repo script, not part of the installed-wheel checkout
    raise unittest.SkipTest('scripts/field/wiring_check.py absent (installed-wheel checkout)')
SPEC = importlib.util.spec_from_file_location('wiring_check', SCRIPT)
wc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wc)

import test_bringup_field_launch as fixtures  # noqa: E402  (field profile: robot_a=/robot6, robot_b=/robot5)
from rescue_bringup.field_launch import load_field  # noqa: E402
from rescue_bringup.profile import load_deployment  # noqa: E402

LAUNCH = REPO / 'src' / 'rescue_bringup' / 'launch'
TB4 = REPO / 'src' / 'vendor' / 'turtlebot4' / 'turtlebot4_navigation' / 'config'
OK_QOS = {'reliability': 'RELIABLE', 'durability': 'VOLATILE'}
PI = '/robot_pi_driver'   # an external writer (robot Pi); its name is not judged


class Case(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.profile = fixtures.field_profile(self.dir)
        self.deployment = load_deployment(self.profile)
        self.field = load_field(fixtures.FIELD)

    def links(self, **kw):
        return wc.expected_links(self.deployment, self.field, now_ns=fixtures.NOW_NS, **kw)

    def by_id(self, **kw):
        return {lk.id: lk for lk in self.links(**kw)}


def healthy(links):
    """The graph every expectation asks for: each writer once, every reader, one type, compatible QoS."""
    graph = {'nodes': [], 'topics': {}, 'services': {}, 'actions': {}}
    nodes = set()
    for lk in links:
        writers = [PI] if lk.external_writer else list(lk.writers)
        readers = list(lk.readers) or (['/create3_driver'] if lk.external_reader else [])
        nodes.update(writers + readers)
        if lk.kind == 'topic':
            entry = graph['topics'].setdefault(lk.name, {'publishers': [], 'subscribers': []})
            entry['publishers'] += [{'node': n, 'type': 'pkg/msg/T', 'qos': dict(OK_QOS)} for n in writers]
            entry['subscribers'] += [{'node': n, 'type': 'pkg/msg/T', 'qos': dict(OK_QOS)} for n in readers]
        else:
            entry = graph[lk.kind + 's'].setdefault(lk.name, {'servers': [], 'clients': []})
            entry['servers'] += writers
            entry['clients'] += readers
    graph['nodes'] = sorted(nodes)
    return graph


class Expectations(Case):
    """Names come from the launch value functions and code constants, never from the old CSV."""

    def test_node_names_are_the_launch_file_names(self):
        ours = {'adapter': 'robot.launch.py', 'watchdog': 'robot.launch.py', 'passage_points': 'passage_points.launch.py',
                'overhead': 'overhead_passage.launch.py', 'relay': 'frozen_localization.launch.py',
                'amcl': 'frozen_localization.launch.py', 'start_align': 'frozen_localization.launch.py',
                'map_freeze': 'map_freeze.launch.py',
                'db_writer': 'operations.launch.py', 'report_node': 'operations.launch.py',
                'webcam': 'operations.launch.py', 'target_locator': 'amr_perception.launch.py',
                'exit_locator': 'amr_perception.launch.py'}
        for key, launch in ours.items():
            with self.subTest(node=key):
                self.assertIn(f"name='{wc.NODES[key]}'", (LAUNCH / launch).read_text())
        for key, config in (('collision_monitor', 'nav2.yaml'), ('docking_server', 'nav2.yaml'),
                            ('bt_navigator', 'nav2.yaml'), ('slam_toolbox', 'slam.yaml')):
            with self.subTest(node=key):
                self.assertRegex((TB4 / config).read_text(), rf'(?m)^{wc.NODES[key]}:')
        nav2 = (TB4 / 'nav2.yaml').read_text()
        self.assertIn(f'state_topic: "{wc.COLLISION_STATE}"', nav2)
        runtime = (REPO / 'src' / 'rescue_robot' / 'rescue_robot' / 'task_site_runtime.py').read_text()
        self.assertIn(f"attach_passage(self,'{wc.COLLISION_STATE}')", runtime)

    def test_the_watchdog_alone_writes_cmd_vel_and_nav2_writes_its_input(self):
        links = self.by_id()
        self.assertEqual(('/robot6/rescue_command_watchdog',), links['robot_a.cmd_vel'].writers)
        self.assertTrue(links['robot_a.cmd_vel'].external_reader)
        self.assertEqual(('/robot6/collision_monitor', '/robot6/docking_server'),
                         links['robot_a.cmd_vel_watchdog_in'].writers)
        self.assertEqual('/robot5/cmd_vel_watchdog_in', links['robot_b.cmd_vel_watchdog_in'].name)

    def test_robot_rows_follow_the_launch_values(self):
        from rescue_bringup.field_launch import robot_launch_values
        links = self.by_id()
        for robot in ('robot_a', 'robot_b'):
            t = robot_launch_values(self.deployment, field=self.field, robot_id=robot, occupancy_sha256='',
                                    now_ns=fixtures.NOW_NS)['task_input']
            with self.subTest(robot=robot):
                self.assertEqual(t['executor_heartbeat_topic'], links[f'{robot}.executor_heartbeat'].name)
                self.assertEqual(t['odom_topic'], links[f'{robot}.odom'].name)
                self.assertTrue(links[f'{robot}.odom'].external_writer)
                self.assertEqual(t['nav_action'], links[f'{robot}.navigate_to_pose'].name)
                for camera in t['camera_topics']:
                    self.assertIn(f'{robot}.camera:{camera}', links)

    def test_coordinator_rows_use_the_coordination_constants(self):
        from rescue_coordination import ros_node as co
        links = self.by_id()
        self.assertEqual(co.SYSTEM_STATE_TOPIC, links['system_state'].name)
        self.assertEqual(('/mission_coordinator',), links['system_state'].writers)
        self.assertEqual(co.ROBOT_PREFIX + 'robot_b/execute_task', links['robot_b.execute_task'].name)
        self.assertEqual('action', links['robot_b.execute_task'].kind)
        self.assertEqual(co.MAP_FREEZE_SERVICE, links['map_freeze'].name)
        self.assertEqual(('/rescue_map_freeze',), links['frozen_map'].writers)

    def test_both_robots_share_one_row_for_the_coordinator_service(self):
        links = self.by_id()
        self.assertEqual({'/robot6/rescue_command_adapter', '/robot5/rescue_command_adapter', '/rescue_db_writer'},
                         set(links['system_state'].readers))
        self.assertEqual({'/robot6/rescue_command_adapter', '/robot5/rescue_command_adapter'},
                         set(links['task_inputs'].readers))
        self.assertEqual(1, sum(lk.name == '/mission_coordinator/task_inputs' for lk in links.values()))

    def test_map_rows_follow_the_tb6_phase(self):
        searching = self.by_id()
        self.assertEqual(('/robot6/slam_toolbox',), searching['robot_a.map'].writers)
        self.assertNotIn('robot_a.amcl_pose', searching)
        self.assertEqual(('/robot5/rescue_frozen_map_relay',), searching['robot_b.map'].writers)
        frozen = self.by_id(slam_phase='FROZEN')
        self.assertEqual(('/robot6/rescue_frozen_map_relay',), frozen['robot_a.map'].writers)
        self.assertEqual(('/robot6/amcl',), frozen['robot_a.amcl_pose'].writers)
        self.assertIn('/robot6/rescue_frozen_map_relay', frozen['frozen_map'].readers)

    def test_tb5_executor_reads_its_start_alignment_verdict(self):
        # VALID only after start_align says ALIGNED (#317); TB-6 has no start alignment
        for phase in ('SEARCHING', 'FROZEN'):
            links = self.by_id(slam_phase=phase)
            with self.subTest(phase=phase):
                row = links['robot_b.start_align']
                self.assertEqual(('/robot5/rescue_start_align', ('/robot5/rescue_start_align',),
                                  ('/robot5/rescue_command_adapter',)), (row.name, row.writers, row.readers))
                self.assertNotIn('robot_a.start_align', links)

    def test_each_executor_asks_its_nav2_lifecycle_manager(self):
        # nav2 READY needs is_active success (field run1015b: inactive bt_navigator was READY)
        links = self.by_id()
        for robot, ns in (('robot_a', '/robot6'), ('robot_b', '/robot5')):
            with self.subTest(robot=robot):
                row = links[f'{robot}.nav2_lifecycle']
                self.assertEqual(('service', f'{ns}/lifecycle_manager_navigation/is_active',
                                  (f'{ns}/lifecycle_manager_navigation',), (f'{ns}/rescue_command_adapter',)),
                                 (row.kind, row.name, row.writers, row.readers))

    def test_one_robot_keeps_the_shared_rows_without_the_other_robot(self):
        links = self.links(robots=['robot_b'])
        text = json.dumps([lk.name for lk in links] + [n for lk in links for n in lk.writers + lk.readers])
        self.assertNotIn('/robot6/', text)
        self.assertIn('/rescue/system_state', text)
        with self.assertRaisesRegex(wc.WiringError, 'robot_x'):
            self.links(robots=['robot_x'])

    def test_no_old_names(self):
        text = json.dumps([[lk.name, lk.writers, lk.readers] for lk in self.links(slam_phase='FROZEN')])
        self.assertIsNone(re.search(r'/amr[12]\b|PC[1-4]|/tb[12]\b', text))

    def test_frontier_coordinator_reads_the_scout_live_map(self):
        from rescue_coordination.ros_node import SCOUT_MAP_TOPIC
        self.assertEqual('FRONTIER', self.field.coordinator['search_mode'])
        for phase, writer in (('SEARCHING', '/robot6/slam_toolbox'), ('FROZEN', '/robot6/rescue_frozen_map_relay')):
            row = next(lk for lk in self.links(slam_phase=phase) if lk.name == SCOUT_MAP_TOPIC)
            with self.subTest(phase=phase):
                self.assertIn('/mission_coordinator', row.readers)
                self.assertEqual((writer,), row.writers)
        self.assertFalse(any(lk.name == SCOUT_MAP_TOPIC for lk in self.links(robots=['robot_b'])))

    def test_a_row_without_a_writer_is_refused(self):
        with self.assertRaisesRegex(wc.WiringError, 'no writer'):
            wc.link('x', 'topic', '/x', [], ['/reader'], source='t')

    def test_two_writer_expectations_for_one_name_are_refused(self):
        a = wc.link('a', 'topic', '/x', ['/n1'], source='t')
        b = wc.link('b', 'topic', '/x', ['/n2'], source='t')
        with self.assertRaises(wc.WiringError):
            wc.merge([a, b])


class Verdicts(Case):
    def judge(self, graph, **kw):
        rows, nodes, code = wc.check(self.links(**kw), graph)
        return {r['id']: r for r in rows}, nodes, code

    def test_the_expected_graph_passes(self):
        rows, nodes, code = self.judge(healthy(self.links()))
        self.assertEqual(0, code, [r for r in rows.values() if r['status'] != wc.PASSED])
        self.assertEqual(wc.PASSED, nodes['status'])

    def test_no_writer_fails(self):
        graph = healthy(self.links())
        graph['topics']['/robot6/cmd_vel']['publishers'] = []
        rows, _, code = self.judge(graph)
        self.assertEqual((wc.FAILED, 2), (rows['robot_a.cmd_vel']['status'], code))
        self.assertIn('발행자 없음: /robot6/rescue_command_watchdog', rows['robot_a.cmd_vel']['problems'])

    def test_a_second_writer_fails(self):
        graph = healthy(self.links())
        pubs = graph['topics']['/robot6/map']['publishers']
        pubs.append(deepcopy(pubs[0]))   # the same node twice (two slam_toolbox)
        graph['topics']['/robot5/odom']['publishers'].append({'node': '/other', 'type': 'pkg/msg/T',
                                                               'qos': dict(OK_QOS)})
        graph['services']['/rescue/map/freeze']['servers'].append('/rescue_map_freeze')
        rows, _, _ = self.judge(graph)
        self.assertIn('발행자 중복: /robot6/slam_toolbox', rows['robot_a.map']['problems'])
        self.assertIn('발행자 2개 (1개여야 함)', rows['robot_b.odom']['problems'])
        self.assertIn('서버 중복: /rescue_map_freeze', rows['map_freeze']['problems'])
        for row in ('robot_a.map', 'robot_b.odom', 'map_freeze'):
            self.assertEqual(wc.FAILED, rows[row]['status'])

    def test_a_writer_from_another_node_fails(self):
        graph = healthy(self.links())
        graph['topics']['/rescue/system_state']['publishers'][0]['node'] = '/robot6/mission_coordinator'
        rows, _, _ = self.judge(graph)
        self.assertEqual(wc.FAILED, rows['system_state']['status'])
        self.assertIn('다른 발행자: /robot6/mission_coordinator', rows['system_state']['problems'])

    def test_a_missing_reader_fails(self):
        graph = healthy(self.links())
        subs = graph['topics']['/robot5/rescue/executor_heartbeat']['subscribers']
        subs[:] = [s for s in subs if s['node'] != '/robot5/rescue_command_watchdog']
        graph['actions']['/rescue/robots/robot_b/execute_task']['clients'] = []
        graph['topics']['/robot6/cmd_vel']['subscribers'] = []   # Create3 not seen
        rows, _, _ = self.judge(graph)
        self.assertIn('구독 없음: /robot5/rescue_command_watchdog', rows['robot_b.executor_heartbeat']['problems'])
        self.assertIn('클라이언트 없음: /mission_coordinator', rows['robot_b.execute_task']['problems'])
        self.assertIn('구독 0개', rows['robot_a.cmd_vel']['problems'])

    def test_qos_and_type_mismatches_fail(self):
        graph = healthy(self.links())
        graph['topics']['/rescue/robots/robot_a/status']['publishers'][0]['qos']['reliability'] = 'BEST_EFFORT'
        graph['topics']['/rescue/map/frozen']['publishers'][0]['qos']['durability'] = 'VOLATILE'
        for sub in graph['topics']['/rescue/map/frozen']['subscribers']:
            sub['qos']['durability'] = 'TRANSIENT_LOCAL'
        graph['topics']['/robot5/scan']['subscribers'][0]['type'] = 'other/msg/T'
        rows, _, code = self.judge(graph)
        self.assertEqual(2, code)
        self.assertTrue(any('reliability BEST_EFFORT -> RELIABLE' in p for p in rows['robot_a.status']['problems']))
        self.assertTrue(any('durability VOLATILE -> TRANSIENT_LOCAL' in p for p in rows['frozen_map']['problems']))
        self.assertTrue(any(p.startswith('타입 불일치') for p in rows['robot_b.scan']['problems']))

    def test_reliable_to_best_effort_and_latched_to_volatile_are_compatible(self):
        self.assertEqual('', wc.qos_problem({'reliability': 'RELIABLE', 'durability': 'TRANSIENT_LOCAL'},
                                            {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE'}))

    def test_unknown_qos_is_not_run_never_passed(self):
        graph = healthy(self.links())
        graph['topics']['/robot6/odom']['publishers'][0]['qos']['reliability'] = 'SYSTEM_DEFAULT'
        rows, _, code = self.judge(graph)
        self.assertEqual(wc.NOT_RUN, rows['robot_a.odom']['status'])
        self.assertEqual(3, code)

    def test_duplicate_node_names_fail(self):
        graph = healthy(self.links())
        graph['nodes'].append('/robot6/slam_toolbox')   # mu-06 2026-10-14: two answers to one lifecycle request
        _, nodes, code = self.judge(graph)
        self.assertEqual((wc.FAILED, ['/robot6/slam_toolbox'], 2), (nodes['status'], nodes['names'], code))

    def test_no_graph_is_not_run(self):
        rows, nodes, code = self.judge(None)
        self.assertEqual({wc.NOT_RUN}, {r['status'] for r in rows.values()} | {nodes['status']})
        self.assertEqual(3, code)


class Main(Case):
    def run_main(self, reader, *extra):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = wc.main(['--profile', str(self.profile), '--field', str(fixtures.FIELD), *extra],
                           graph_reader=reader)
        return code, json.loads(out.getvalue()), err.getvalue()

    def test_json_on_stdout_table_on_stderr(self):
        code, result, table = self.run_main(lambda links, spin: healthy(links), '--robot', 'robot_b')
        self.assertEqual((0, 0), (code, result['exit_code']))
        self.assertEqual(['robot_b'], result['robots'])
        self.assertIn('PASSED', table)
        self.assertIn('wiring_check exit=0', table)
        self.assertFalse(any('/robot6/' in r['name'] for r in result['connections']))

    def test_without_ros_everything_is_not_run(self):
        def no_ros(links, spin):
            raise ImportError("No module named 'rclpy'")
        code, result, table = self.run_main(no_ros)
        self.assertEqual(3, code)
        self.assertIn('rclpy', result['graph_error'])
        self.assertEqual({wc.NOT_RUN}, {r['status'] for r in result['connections']})

    def test_a_graph_failure_is_not_run(self):
        def broken(links, spin):
            raise RuntimeError('rcl_init failed: discovery server unreachable')
        code, result, table = self.run_main(broken)
        self.assertEqual((3, 3), (code, result['exit_code']))
        self.assertIn('RuntimeError: rcl_init failed', result['graph_error'])
        self.assertIn('그래프를 읽지 못함', table)
        self.assertEqual({wc.NOT_RUN}, {r['status'] for r in result['connections']})

    def test_an_unreadable_expectation_is_not_run(self):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = wc.main(['--profile', str(self.dir / 'missing.json'), '--field', str(fixtures.FIELD)],
                           graph_reader=lambda links, spin: self.fail('no graph without expectations'))
        self.assertEqual(3, code)
        self.assertIn('기대 연결표를 만들지 못함', err.getvalue())


if __name__ == '__main__':
    unittest.main()
