"""Preparing files never invents a policy or erases live measurements."""
import json
from pathlib import Path
import tempfile
import unittest

from rescue_robot.task_prepare import prepare

class TaskPrepareTests(unittest.TestCase):
    def test_no_watchdog_needed_to_prepare_and_missing_values_are_listed(self):
        with tempfile.TemporaryDirectory() as d:
            report=prepare(d,robot_id='logical_tb2',namespace='/physical_tb2')
            self.assertEqual(report['status'],'NOT_CONFIGURED')
            self.assertIn('lease.duration_s',report['missing'])
            c=json.loads((Path(d)/'runtime.json').read_text())
            self.assertFalse((Path(d)/'inputs.json').exists())
            self.assertIsNone(c['lease']['duration_s'])
            self.assertEqual(c['endpoints']['action_name'],'/rescue/robots/logical_tb2/execute_task')
            overlay=json.loads((Path(d)/'task.params.yaml').read_text())
            self.assertEqual(overlay['/**']['ros__parameters']['task.runtime_factory'],
                             'rescue_robot.task_site_runtime:create_runtime')

    def test_regenerating_task_overlay_does_not_refresh_or_replace_input(self):
        with tempfile.TemporaryDirectory() as d:
            settings=dict(clock_basis_file=str(Path(d)/'clock-basis.json'),configured_destinations={},
                coordinator_targets_topic='/rescue/targets',coordinator_authority={})
            prepare(d,robot_id='logical_tb2',namespace='/physical_tb2',settings=settings)
            p=Path(d)/'input-sources.json'; p.write_text('{"live":"original-time"}')
            prepare(d,robot_id='logical_tb2',namespace='/physical_tb2',settings=settings)
            self.assertEqual(p.read_text(),'{"live":"original-time"}')
            c=json.loads((Path(d)/'runtime.json').read_text())
            self.assertEqual(c['clock_basis_file'],settings['clock_basis_file'])
            self.assertEqual(c['configured_destinations'],{})
            self.assertEqual(c['coordinator_targets_topic'],settings['coordinator_targets_topic'])
            self.assertEqual(c['coordinator_authority'],{})
