"""task_input reads what robot.launch.py gives it (no ROS): live_parameters.read_config on robot_launch_values.

Found in the one-PC wiring rehearsal (2026-10-14): with the field file's map modes (TB-6
SLAM_THEN_FROZEN, TB-5 FROZEN_MAP_AMCL) the launch passes task.saved_map.occupancy_sha256 '' (the
freeze decides the hash) and read_config refused it as a missing parameter, so task_input stopped
at start on both robots. rclpy is replaced by a small fake node here; the parameter values are the
launch's own.
"""
import sys
import tempfile
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import test_bringup_field_launch as fixtures
from rescue_bringup.field_launch import amcl_update_thresholds, load_field, robot_launch_values
from rescue_bringup.profile import load_deployment
from rescue_robot.executor import ContractError


class Uninitialized(Exception):
    pass


def fake_ros():
    """rcl_interfaces.msg / rclpy.parameter / rclpy.exceptions: only what read_config and declare_required use."""
    kinds = NS(PARAMETER_BOOL=1, PARAMETER_INTEGER=2, PARAMETER_DOUBLE=3, PARAMETER_STRING=4,
               PARAMETER_BOOL_ARRAY=6, PARAMETER_INTEGER_ARRAY=7, PARAMETER_DOUBLE_ARRAY=8,
               PARAMETER_STRING_ARRAY=9)

    class Type(int):
        BOOL, INTEGER, DOUBLE, STRING = 1, 2, 3, 4
        BOOL_ARRAY, INTEGER_ARRAY, DOUBLE_ARRAY, STRING_ARRAY = 6, 7, 8, 9
    msg = ModuleType('rcl_interfaces.msg')
    msg.ParameterType, msg.ParameterDescriptor = kinds, (lambda **kw: NS(**kw))
    parameter = ModuleType('rclpy.parameter')
    parameter.Parameter = NS(Type=Type)
    exceptions = ModuleType('rclpy.exceptions')
    exceptions.ParameterUninitializedException = Uninitialized
    rcl, rclpy = ModuleType('rcl_interfaces'), ModuleType('rclpy')
    return {'rcl_interfaces': rcl, 'rcl_interfaces.msg': msg, 'rclpy': rclpy, 'rclpy.parameter': parameter,
            'rclpy.exceptions': exceptions}


class FakeNode:
    def __init__(self, values):
        self.values, self.declared = dict(values), set()

    def declare_parameter(self, name, *_args, **_kwargs):
        self.declared.add(name)

    def get_parameter(self, name):
        if name not in self.values:
            raise Uninitialized(name)
        return NS(value=self.values[name])


TB4_AMCL = {'amcl': {'ros__parameters': {'update_min_d': 0.25, 'update_min_a': 0.2}}}  # turtlebot4 localization.yaml


class ReadsTheLaunchValues(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.deployment = load_deployment(fixtures.field_profile(Path(self.tmp.name)))
        self.field = load_field(fixtures.FIELD)

    def read(self, robot, *, field=None, sha='', change=None, drop=()):
        task = robot_launch_values(self.deployment, field=field or self.field, robot_id=robot,
                                   occupancy_sha256=sha, now_ns=fixtures.NOW_NS)['task_input']
        if task['task.saved_map.mode'] != 'SAVED_MAP_AMCL':   # robot.launch.py adds AMCL's update thresholds
            task.update(amcl_update_thresholds(TB4_AMCL))
        task.update(change or {})
        for name in drop:
            del task[name]
        adapter = NS(robot_id=robot, executor=NS(authority_epoch=task['authority_epoch']
                                                  if 'authority_epoch' in task else
                                                  '66666666-6666-4666-8666-666666666666'))
        with patch.dict(sys.modules, fake_ros()):
            from rescue_robot.live_parameters import read_config
            return read_config(FakeNode(task), adapter)

    def test_frozen_map_modes_start_with_an_empty_hash(self):
        for robot, mode in (('robot_a', 'SLAM_THEN_FROZEN'), ('robot_b', 'FROZEN_MAP_AMCL')):
            with self.subTest(robot=robot):
                config = self.read(robot)
                saved = config['saved_maps'][robot]
                self.assertEqual((mode, ''), (saved['mode'], saved['occupancy_sha256']))
                self.assertIs(True, config['auto_rearm'])

    def test_frozen_map_modes_read_the_tf_pose_values(self):
        """The frozen-phase TF pose (a standing robot has no amcl_pose): topic, frames, AMCL thresholds."""
        saved = self.read('robot_b')['saved_maps']['robot_b']
        self.assertEqual(('/robot5/tf', 'odom', 'base_link', 0.25, 0.2),
                         tuple(saved[k] for k in ('amcl_tf_topic', 'amcl_odom_frame', 'amcl_base_frame',
                                                  'amcl_update_min_d', 'amcl_update_min_a')))
        with self.assertRaisesRegex(ContractError, 'amcl_update_min_d'):
            self.read('robot_b', drop=('task.saved_map.amcl_update_min_d',))

    def test_a_frozen_map_mode_refuses_a_hash(self):
        with self.assertRaisesRegex(ContractError, 'occupancy_sha256'):
            self.read('robot_b', change={'task.saved_map.occupancy_sha256': 'ab' * 32})

    def test_saved_map_needs_the_measured_hash(self):
        saved = load_field(self._saved_field())
        self.assertEqual('ab' * 32, self.read('robot_a', field=saved, sha='ab' * 32)
                         ['saved_maps']['robot_a']['occupancy_sha256'])
        with self.assertRaisesRegex(ContractError, 'occupancy_sha256'):
            self.read('robot_a', field=saved, sha='ab' * 32, change={'task.saved_map.occupancy_sha256': ''})

    def test_the_hash_must_still_be_given(self):
        with self.assertRaisesRegex(ContractError, 'missing.*occupancy_sha256'):
            task_without = {'task.saved_map.occupancy_sha256': None}
            self.read('robot_a', change=task_without)

    def test_tb5_reads_its_start_alignment_verdict_topic(self):
        # VALID only after start_align says ALIGNED (#317, 재범 decision 2026-10-15 (나))
        config = self.read('robot_b')
        self.assertEqual('/robot5/rescue_start_align', config['saved_maps']['robot_b']['start_align_topic'])
        self.assertNotIn('start_align_topic', self.read('robot_a')['saved_maps']['robot_a'])
        task = robot_launch_values(self.deployment, field=self.field, robot_id='robot_b', occupancy_sha256='',
                                   now_ns=fixtures.NOW_NS)['task_input']
        task.pop('task.saved_map.start_align_topic')   # not given: never set, as ROS leaves it uninitialized
        adapter = NS(robot_id='robot_b', executor=NS(authority_epoch=task['authority_epoch']))
        with patch.dict(sys.modules, fake_ros()), self.assertRaisesRegex(ContractError, 'start_align_topic'):
            from rescue_robot.live_parameters import read_config
            read_config(FakeNode(task), adapter)

    def _saved_field(self):
        import json
        raw = json.loads(fixtures.FIELD.read_text())
        fixtures.saved_maps(raw)
        path = Path(self.tmp.name) / 'saved.json'
        path.write_text(json.dumps(raw))
        return path


if __name__ == '__main__':
    unittest.main()
