"""Exercise the #85 requested order through the installed runtime, no robot."""
import unittest
import os
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch
from uuid import UUID
from rescue_robot.task_runtime_demo import run_e2
from rescue_robot import task_runtime_demo as demo
from rescue_robot import task_server_probe as probe


class DelayedRosTransport:
    """Scripted asynchronous transport; never claims to run ROS or stop a robot.

    Lease delivery precedes action result delivery. This ordering reproduces
    the PC's HOLD result race while running the actual run_ros control flow.
    """
    def __init__(self, *, finish_hold=True):
        self.now = 0.
        self.pending = []
        self.goal_count = 0
        self.finish_hold = finish_hold
        self.hold_result = None
        self.cleaned = []
        self.on_admission = None

    def schedule(self, callback, ticks=2):
        self.pending.append([ticks, callback])

    def spin(self, node, **kwargs):
        self.now += .05
        for event in list(self.pending):
            event[0] -= 1
            if event[0] == 0:
                self.pending.remove(event)
                event[1]()

    def future(self, value=None):
        state = NS(value=value)
        return NS(done=lambda: state.value is not None, result=lambda: state.value,
                  complete=lambda value: setattr(state, 'value', value))

    def node(self, name):
        def subscribe(kind, topic, callback, qos):
            if topic.endswith('/task_admission'):
                self.on_admission = callback
            else:
                callback(NS())  # initial status observation
        def publisher(*args):
            def publish(lease):
                self.schedule(lambda: setattr(self.fixture.adapter.executor,
                                              'lease_sequence', lease.renewal_sequence), ticks=1)
            return NS(publish=publish, get_subscription_count=lambda: 1)
        return NS(create_subscription=subscribe, create_publisher=publisher,
                  create_timer=lambda *args: None,
                  destroy_node=lambda: self.cleaned.append('node'))

    def make_fixture(self, node):
        executor = NS(motion='STOPPED', lease_sequence=-1, state='RUNNING')
        def arm():
            executor.lease_sequence = -1
            return True
        def goal_values(kind):
            self.goal_count += 1
            command = str(self.goal_count)
            adapter.sent.add(command)
            return dict(kind=kind, command_id=command)
        adapter = NS(executor=executor, sent=set(), on_terminal=lambda *args: None)
        self.fixture = NS(prefix='/synthetic-probe', adapter=adapter, observe=lambda: None,
            runtime=NS(arm=arm, server=NS(publisher=NS(get_subscription_count=lambda: 1)),
                       destroy=lambda: self.cleaned.append('runtime')),
            goal_values=goal_values,
            lease_values=lambda goal, sequence: dict(renewal_sequence=sequence))
        return self.fixture

    def client(self, *args):
        def send(goal):
            self.on_admission(NS())
            result = self.future()
            scope = {'SEARCH_AREA': 'SEARCH', 'REOBSERVE_TARGET': 'RECHECK', 'HOLD': 'HOLD'}[goal.kind]
            def complete(outcome):
                result.complete(NS(result=NS(result=NS(outcome=outcome, stop_confirmed=True,
                                                       completion_scope=scope))))
            if self.goal_count <= 3:
                if goal.kind == 'HOLD':
                    self.hold_result = result
                if goal.kind != 'HOLD' or self.finish_hold:
                    self.schedule(lambda: complete('COMPLETED'))
            elif self.goal_count == 4:
                self.schedule(lambda: complete('EXPIRED'))
            # The renewal scenario stays pending until an explicit cancel.
            def cancel():
                self.schedule(lambda: complete('CANCELED'))
                return self.future(NS(goals_canceling=[NS()]))
            return self.future(NS(accepted=True, get_result_async=lambda: result,
                                  cancel_goal_async=cancel))
        return NS(server_is_ready=lambda: True, send_goal_async=send,
                  destroy=lambda: self.cleaned.append('client'))

    def run(self):
        modules = {
            'rclpy': NS(init=lambda **kwargs: None, spin_once=self.spin,
                        try_shutdown=lambda: self.cleaned.append('rclpy')),
            'rclpy.action': NS(ActionClient=self.client),
            'rclpy.node': NS(Node=self.node),
            'rescue_interfaces.action': NS(ExecuteTask=NS(Goal=NS)),
            'rescue_interfaces.msg': NS(CommandLease=NS, TaskAdmission=NS, RobotStatus=NS),
        }
        env = {'ROS_DOMAIN_ID': '173', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
        with patch.dict(sys.modules, modules), patch.dict(os.environ, env, clear=True), \
                patch.object(demo, 'Fixture', self.make_fixture), \
                patch.object(demo.time, 'monotonic', lambda: self.now):
            return demo.run_ros(173, 5., stage='common')


class OrderedRuntimeTests(unittest.TestCase):
    def test_probe_namespace_is_valid_for_every_uuid_hex_prefix(self):
        for first in '0123456789abcdef':
            with self.subTest(first=first):
                value = UUID(first + '1234567-89ab-4cde-8123-456789abcdef')
                with patch.object(demo, 'uuid4', return_value=value), \
                        patch.object(probe, 'uuid4', return_value=value):
                    fixture = demo.Fixture(demo.DemoNode(), bindings=demo.fake_bindings())
                try:
                    self.assertRegex(fixture.prefix, r'^(/[A-Za-z_][A-Za-z0-9_]*)+$')
                finally:
                    fixture.runtime.destroy()

    def test_ros_hold_waits_for_result_after_lease_delivery(self):
        transport = DelayedRosTransport()
        report = transport.run()
        self.assertEqual([s['kind'] for s in report['scenarios']],
                         ['SEARCH_AREA', 'REOBSERVE_TARGET', 'HOLD'])
        self.assertTrue(transport.hold_result.done())
        self.assertEqual(report['scenarios'][-1]['outcome'], 'COMPLETED')
        self.assertEqual(report['cancel'], 'CANCELED_STOP_CONFIRMED')
        self.assertEqual(transport.cleaned, ['runtime', 'client', 'node', 'rclpy'])

    def test_ros_hold_missing_result_times_out_and_cleans_up(self):
        transport = DelayedRosTransport(finish_hold=False)
        with self.assertRaisesRegex(RuntimeError, 'E3 local transport timeout'):
            transport.run()
        self.assertFalse(transport.hold_result.done())
        self.assertEqual(transport.cleaned, ['runtime', 'client', 'node', 'rclpy'])

    def test_common_stage_accepts_search_reobserve_hold_and_renews_active_search(self):
        report = run_e2(stage='common')
        self.assertEqual([s['kind'] for s in report['scenarios']],
                         ['SEARCH_AREA', 'REOBSERVE_TARGET', 'HOLD'])
        renewal = report['renewal']
        self.assertEqual(renewal['status'], 'PASSED')
        self.assertGreater(renewal['active_for_s'], renewal['initial_duration_s'])
        self.assertGreater(renewal['last_sequence'], 0)
        self.assertTrue(renewal['duplicate_rejected'])
        self.assertFalse(report['watchdog_process_created'])

    def test_common_cancel_waits_for_stop_and_no_odom_is_not_success(self):
        report = run_e2(stage='common')
        self.assertEqual(report['cancel'], 'CANCELED_STOP_CONFIRMED')
        self.assertEqual(report['no_stop_evidence'], 'STOP_UNCONFIRMED')
        self.assertEqual(report['late_renewal'], 'NO_REVIVAL')

    def test_fallback_stage_only_runs_segment_and_preserves_scope(self):
        report = run_e2(stage='fallback')
        self.assertEqual([s['kind'] for s in report['scenarios']], ['MOVE_SEGMENT'])
        self.assertEqual(report['scenarios'][0]['scope'], 'SEGMENT')
        self.assertEqual(report['lease_expiry'], 'EXPIRED_STOP_CONFIRMED')

    def test_unknown_stage_rejected_before_runtime_creation(self):
        with self.assertRaises(ValueError):
            run_e2(stage='follow')


if __name__ == '__main__':
    unittest.main()
