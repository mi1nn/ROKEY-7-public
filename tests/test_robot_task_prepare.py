"""Preparing files never invents a policy or erases live measurements."""
import json
from pathlib import Path
import tempfile
import unittest

from rescue_robot.task_prepare import prepare
from rescue_robot.task_runtime_demo import run_e2

class TaskPrepareTests(unittest.TestCase):
    def test_no_watchdog_needed_to_prepare_and_missing_values_are_listed(self):
        with tempfile.TemporaryDirectory() as d:
            report=prepare(d,robot_id='logical_tb2',namespace='/physical_tb2')
            self.assertEqual(report['status'],'NOT_CONFIGURED')
            self.assertIn('lease.duration_s',report['missing'])
            c=json.loads((Path(d)/'runtime.json').read_text())
            self.assertIsNone(c['lease']['duration_s'])
            self.assertEqual(c['endpoints']['action_name'],'/rescue/robots/logical_tb2/execute_task')
            overlay=json.loads((Path(d)/'task.params.yaml').read_text())
            self.assertEqual(overlay['/**']['ros__parameters']['task.runtime_factory'],
                             'rescue_robot.task_site_runtime:create_runtime')

    def test_regenerating_task_overlay_does_not_refresh_or_replace_input(self):
        with tempfile.TemporaryDirectory() as d:
            prepare(d,robot_id='logical_tb2',namespace='/physical_tb2')
            p=Path(d)/'inputs.json'; p.write_text('{"live":"original-time"}')
            prepare(d,robot_id='logical_tb2',namespace='/physical_tb2')
            self.assertEqual(p.read_text(),'{"live":"original-time"}')

    def test_watchdog_independent_rehearsal_runs_real_runtime_six_scenarios(self):
        result=run_e2()
        self.assertEqual(result['status'],'PASSED')
        self.assertEqual([s['scope'] for s in result['scenarios']],['SEARCH','RECHECK','SEGMENT','RETURN','HOLD'])
        self.assertEqual(result['lease_expiry'],'EXPIRED_STOP_CONFIRMED')
        self.assertFalse(result['watchdog_process_created'])
        self.assertEqual(result['ros_transport'],'NOT_RUN')
