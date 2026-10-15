"""RViz uses the shared map and one native TF tree; never merges duplicate frame names."""
import json
import runpy
import sys
from pathlib import Path
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from rescue_bringup.field_launch import amcl_parameters, load_field
from rescue_bringup.profile import load_deployment
from rescue_bringup.rviz_view import rviz_config, view_values
from test_bringup_field_launch import REPO, field_profile


class ViewLaunchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.deployment = load_deployment(field_profile(Path(self.temp.name)))
        self.field = load_field(REPO / 'config/field/live_map.overlay.json')

    def test_default_view_shows_both_positions_and_the_support_model(self):
        values = view_values(self.deployment, self.field)
        self.assertEqual(['robot_a', 'robot_b'], values['markers']['robot_ids'])
        self.assertEqual('/robot6/map', values['map_topic'])
        self.assertEqual('/robot5/tf', values['tf_topic'])
        self.assertEqual('/robot5/tf_static', values['tf_static_topic'])
        self.assertEqual('/robot5/robot_description', values['description_topic'])

    def test_single_robot_view_only_draws_the_selected_robot(self):
        values = view_values(self.deployment, self.field, show_all=False)
        self.assertEqual(['robot_b'], values['markers']['robot_ids'])

    def test_robot_description_and_map_use_late_subscriber_qos(self):
        values = view_values(self.deployment, self.field)
        config = rviz_config(values)
        manager = config['Visualization Manager']
        self.assertEqual('map', manager['Global Options']['Fixed Frame'])
        displays = {d['Class']: d for d in manager['Displays']}
        grid = displays['rviz_default_plugins/Map']['Topic']
        description = displays['rviz_default_plugins/RobotModel']['Description Topic']
        for topic in (grid, description):
            self.assertEqual('Transient Local', topic['Durability Policy'])
        self.assertEqual('/robot6/map', grid['Value'])
        self.assertEqual('/robot5/robot_description', description['Value'])
        self.assertEqual('', displays['rviz_default_plugins/RobotModel']['TF Prefix'])
        self.assertEqual('/rescue/view/robots', displays['rviz_default_plugins/MarkerArray']['Topic']['Value'])

    def test_read_only_view_has_no_robot_command_tools(self):
        config = rviz_config(view_values(self.deployment, self.field))
        classes = {item['Class'] for item in config['Visualization Manager']['Tools']}
        self.assertNotIn('nav2_rviz_plugins/GoalTool', classes)
        self.assertNotIn('rviz_default_plugins/SetInitialPose', classes)
        self.assertEqual(config, json.loads(json.dumps(config)))

    def test_live_amcl_always_broadcasts_its_measured_map_to_odom(self):
        raw = {'amcl': {'ros__parameters': {'tf_broadcast': False}}}
        params = amcl_parameters(raw, initial_pose_xy_yaw=None, use_sim_time=False,
                                 map_topic='/robot6/map', live=True)
        self.assertTrue(params['tf_broadcast'])
        self.assertFalse(params['set_initial_pose'])

    @unittest.skipUnless((REPO / 'src/rescue_bringup/launch/shared_map_view.launch.py').exists(),
                         'source launch absent in installed checkout')
    def test_launch_connects_rviz_to_native_support_tf_and_cleans_generated_config(self):
        def factory(kind):
            return lambda *args, **kw: NS(kind=kind, args=args, kwargs=kw)
        modules = {}
        for module_name, symbols in {
            'launch': {'LaunchDescription': factory('description')},
            'launch.actions': {key: factory(key) for key in
                               ('DeclareLaunchArgument', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler')},
            'launch.event_handlers': {'OnShutdown': factory('OnShutdown')},
            'launch.substitutions': {'LaunchConfiguration': lambda name: NS(perform=lambda context: context[name])},
            'launch_ros': {}, 'launch_ros.actions': {'Node': factory('Node')},
            'rescue_bringup.ros_parameters': {'typed': lambda values: values},
        }.items():
            module = ModuleType(module_name)
            module.__dict__.update(symbols)
            modules[module_name] = module
        context = dict(profile_path=str(Path(self.temp.name) / 'profile.json'),
                       field_path=str(REPO / 'config/field/live_map.overlay.json'), robot_id='',
                       show_all='true', marker_topic='/rescue/view/robots', description_topic='', rviz='true')
        with patch.dict(sys.modules, modules):
            launch = runpy.run_path(str(REPO / 'src/rescue_bringup/launch/shared_map_view.launch.py'))
            actions = launch['_nodes'](context)
            rviz = next(a for a in actions if a.kind == 'Node' and a.kwargs['package'] == 'rviz2')
            self.assertEqual('/robot5', rviz.kwargs['namespace'])
            self.assertEqual([('/tf', '/robot5/tf'), ('/tf_static', '/robot5/tf_static')],
                             rviz.kwargs['remappings'])
            config_path = Path(rviz.kwargs['arguments'][1])
            event = next(a for a in actions if a.kind == 'RegisterEventHandler')
            cleanup = event.args[0].kwargs['on_shutdown'][0].kwargs['function']
            self.addCleanup(lambda: config_path.unlink(missing_ok=True))
            self.assertEqual('map', json.loads(config_path.read_text())['Visualization Manager']['Global Options']['Fixed Frame'])
            cleanup(context)
            self.assertFalse(config_path.exists())
            actions = launch['_nodes'](context | {'rviz': 'false'})
            self.assertFalse(any(a.kind == 'Node' and a.kwargs['package'] == 'rviz2' for a in actions))


if __name__ == '__main__':
    unittest.main()
