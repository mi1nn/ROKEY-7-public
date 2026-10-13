"""Passage stop on both robots from one place in the field run file (OPS-135/137), no ROS.

Owner decision 2026-10-13 evening: the 46 cm overhead passage rule (scope Q-25, TB-5) applies to
TB-6 too. The heights and the stop polygon are written once, in the field run file's shared
robot section (passage.*), so the two robots cannot differ; robot.launch.py adds the passage
stop polygon and the depth source to the Nav2 collision_monitor parameters it starts with
(no 0.46 in any yaml). Each robot names its depth image in robots.<id>.passage.*; a robot
without one does not start (decision A, OPS-137): no movement without the passage check.
The depth topic is also a readiness camera topic, so lost depth makes the robot NOT_READY.
"""
import json
import unittest

from test_bringup_field_launch import FIELD, SHA, FieldCase

BASE = {'collision_monitor': {'ros__parameters': {
    'state_topic': 'collision_monitor_state', 'source_timeout': 1.0,
    'polygons': ['FootprintApproach'],
    'FootprintApproach': {'type': 'polygon', 'action_type': 'approach', 'min_points': 6, 'enabled': True},
    'observation_sources': ['scan'],
    'scan': {'type': 'scan', 'topic': 'scan', 'min_height': 0.15, 'max_height': 2.0, 'enabled': True}}},
    'controller_server': {'ros__parameters': {'controller_frequency': 20.0}}}
TB5_DEPTH = {'passage.depth_topic': '/robot5/oakd/stereo/image_raw',
             'passage.camera_info_topic': '/robot5/oakd/stereo/camera_info',
             'passage.depth_transport': 'compressedDepth'}


def with_tb5_depth(raw):
    raw['robots']['robot_b'].update(TB5_DEPTH)


class FieldValues(FieldCase):
    def test_one_shared_passage_policy_in_the_repository_field_file(self):
        raw = json.loads(FIELD.read_text())
        r = raw['robot']
        self.assertEqual(0.46, r['passage.max_height_m'])   # owner decision 46 cm (scope Q-25)
        self.assertEqual(0.05, r['passage.min_height_m'])
        self.assertEqual('PassageStop', r['task.live.passage.polygon_name'])
        for robot in raw['robots'].values():
            self.assertFalse({k for k in robot if k.startswith('passage.') and 'height' in k},
                             'heights are only in the shared robot section')
        self.assertEqual('/robot6/oakd/stereo/image_raw', raw['robots']['robot_a']['passage.depth_topic'])
        # TB-5 measured 2026-10-13 20:19-20:20 (OPS-165/166): depth image is published; compressedDepth
        # 6.9-9.2 Hz (max gap 0.30 s) beats raw 5.1-5.5 Hz (max gap 0.45 s) over Wi-Fi
        self.assertEqual(TB5_DEPTH, {k: v for k, v in raw['robots']['robot_b'].items() if k.startswith('passage.')})

    def test_both_robots_start_from_the_repository_field_file(self):
        for robot_id in ('robot_a', 'robot_b'):
            with self.subTest(robot_id=robot_id):
                self.assertEqual('PassageStop', self.robot(field=self.field(), robot_id=robot_id,
                                                           occupancy_sha256='')['passage']['polygon_name'])

    def test_bad_passage_values_are_refused(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def robot(key, value):
            return lambda raw: raw['robot'].__setitem__(key, value)

        def robot_a(key, value):
            return lambda raw: raw['robots']['robot_a'].__setitem__(key, value)
        for name, change in {
                'band upside down': robot('passage.min_height_m', 0.5),
                'band below the floor': robot('passage.min_height_m', -0.1),
                'front range reversed': robot('passage.front_m', [1.0, 0.4]),
                'front range not two numbers': robot('passage.front_m', [0.4]),
                'no width': robot('passage.half_width_m', 0.0),
                'no points': robot('passage.min_points', 0),
                'unknown transport': robot_a('passage.depth_transport', 'jpeg'),
                'half the depth keys': lambda raw: raw['robots']['robot_a'].pop('passage.camera_info_topic'),
                'no shared heights': lambda raw: raw['robot'].pop('passage.max_height_m')}.items():
            with self.subTest(name), self.assertRaises(FieldLaunchError):
                self.field(change)


class RobotLaunch(FieldCase):
    def values(self, robot_id='robot_a', change=None):
        return self.robot(field=self.field(change), robot_id=robot_id, occupancy_sha256='')

    def test_tb6_task_input_reports_the_passage_and_watches_its_depth(self):
        v = self.values()
        task = v['task_input']
        self.assertEqual(('PassageStop', 5_000_000_000), (task['task.live.passage.polygon_name'],
                                                          task['task.live.passage.result_window_ns']))
        # readiness watches the depth over the transport the points use: a raw Image subscription would pull
        # the raw depth over Wi-Fi as well (REV-226-01)
        self.assertIn('/robot6/oakd/stereo/image_raw/compressedDepth', task['camera_topics'])
        self.assertNotIn('/robot6/oakd/stereo/image_raw', task['camera_topics'])
        self.assertFalse([k for k in task if k.startswith('passage.')])   # launch-only keys stay out
        p = v['passage']
        self.assertEqual((0.05, 0.46, [0.4, 1.0], 0.2, 5), (p['min_height_m'], p['max_height_m'], p['front_m'],
                                                             p['half_width_m'], p['min_points']))
        self.assertEqual(('/robot6/oakd/stereo/image_raw', '/robot6/oakd/stereo/camera_info', 'compressedDepth'),
                         (p['depth_topic'], p['camera_info_topic'], p['depth_transport']))

    def test_raw_depth_transport_is_watched_on_the_raw_topic(self):
        task = self.values(change=lambda raw: raw['robots']['robot_a'].__setitem__('passage.depth_transport', 'raw'))[
            'task_input']
        self.assertIn('/robot6/oakd/stereo/image_raw', task['camera_topics'])
        self.assertNotIn('/robot6/oakd/stereo/image_raw/compressedDepth', task['camera_topics'])

    def test_both_robots_get_the_same_passage_numbers(self):
        a = self.values()['passage']
        b = self.values('robot_b', with_tb5_depth)['passage']
        keys = ('min_height_m', 'max_height_m', 'front_m', 'half_width_m', 'min_points', 'polygon_name')
        self.assertEqual([a[k] for k in keys], [b[k] for k in keys])

    def test_a_robot_without_its_depth_image_does_not_start(self):
        from rescue_bringup.field_launch import FieldLaunchError

        def no_depth(raw):
            for key in TB5_DEPTH:
                raw['robots']['robot_b'].pop(key, None)
        with self.assertRaisesRegex(FieldLaunchError, 'passage'):
            self.values('robot_b', no_depth)

    def test_depth_topics_belong_to_this_robot(self):
        from rescue_bringup.field_launch import FieldLaunchError
        with self.assertRaises(FieldLaunchError):
            self.values('robot_b', lambda raw: raw['robots']['robot_b'].update(
                TB5_DEPTH, **{'passage.depth_topic': '/robot6/oakd/stereo/image_raw'}))


class CollisionMonitor(unittest.TestCase):
    PASSAGE = dict(min_height_m=0.05, max_height_m=0.46, front_m=[0.4, 1.0], half_width_m=0.2, min_points=5,
                   polygon_name='PassageStop')

    def apply(self, base=None):
        from rescue_bringup.field_launch import nav2_with_passage
        return nav2_with_passage(json.loads(json.dumps(base or BASE)), self.PASSAGE)

    def test_adds_the_passage_polygon_and_depth_source_only(self):
        out = self.apply()
        self.assertEqual(BASE['controller_server'], out['controller_server'])
        p = out['collision_monitor']['ros__parameters']
        self.assertEqual(['FootprintApproach', 'PassageStop'], p['polygons'])
        self.assertEqual(['scan', 'passage_depth'], p['observation_sources'])
        stop = p['PassageStop']
        self.assertEqual(('polygon', 'stop', 5, ['passage_depth'], True),
                         (stop['type'], stop['action_type'], stop['min_points'], stop['sources_names'], stop['enabled']))
        # one string, as nav2.yaml footprints: ROS parameters cannot hold nested arrays
        self.assertEqual([[1.0, 0.2], [1.0, -0.2], [0.4, -0.2], [0.4, 0.2]], json.loads(stop['points']))
        depth = p['passage_depth']
        self.assertEqual(('pointcloud', 'passage_points', 0.05, 0.46, True),
                         (depth['type'], depth['topic'], depth['min_height'], depth['max_height'], depth['enabled']))
        # existing polygons keep reading only what they read before (the lidar)
        self.assertEqual(['scan'], p['FootprintApproach']['sources_names'])
        self.assertEqual('collision_monitor_state', p['state_topic'])

    def test_refuses_parameters_it_cannot_extend_safely(self):
        broken = json.loads(json.dumps(BASE))
        broken['collision_monitor']['ros__parameters'].pop('state_topic')
        taken = json.loads(json.dumps(BASE))
        taken['collision_monitor']['ros__parameters']['polygons'].append('PassageStop')
        for base in ({'controller_server': {}}, broken, taken):
            with self.subTest(base=list(base)), self.assertRaises(ValueError):
                self.apply(base)


class PointsLaunch(FieldCase):
    def test_points_launch_values_come_from_the_field_file(self):
        from rescue_bringup.field_launch import passage_points_launch_values
        v = passage_points_launch_values(self.deployment, field=self.field(), robot_id='robot_a')
        self.assertEqual({'namespace': '/robot6', 'image_rect': '/robot6/oakd/stereo/image_raw',
                          'camera_info': '/robot6/oakd/stereo/camera_info', 'points': '/robot6/passage_points',
                          'depth_transport': 'compressedDepth'}, v)


if __name__ == '__main__':
    unittest.main()
