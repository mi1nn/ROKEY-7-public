"""scripts/field/noweb/tf_merge.py: both robots' TF on one /tf for one RViz (재범 「제일 중요」, 2026-10-16; no ROS).

Each robot publishes its TF on <ns>/tf and <ns>/tf_static with the same frame names (odom, base_link,
rplidar_link, ...), so one RViz could only read one robot (two_robots.rviz showed TB-5 as an AMCL arrow). The
merge prefixes every frame of a robot with its namespace (robot6/odom, robot5/base_link), keeps the shared frames
(map) as they are, and keeps a frame that already carries that prefix. Static transforms are collected per robot
and published together (/tf_static keeps only the last message per publisher). Each scan is republished on
/rviz/<ns>/scan with its frame prefixed the same way, so RViz can place it. The field (no-web track) and the
simulation use the same file.

Before tf_merge.py existed these tests were expectedFailure.
"""
import importlib.util
import math
import re
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'field' / 'noweb' / 'tf_merge.py'
if not (REPO / 'scripts' / 'field' / 'noweb' / 'operator_command.py').is_file():
    raise unittest.SkipTest('scripts/field/noweb/ absent (installed-wheel checkout)')


def load():
    spec = importlib.util.spec_from_file_location('tf_merge', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def transform(parent, child):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=1, nanosec=2)), child_frame_id=child,
              transform=NS(translation=NS(x=1.0, y=0.0, z=0.0), rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))


class Frames(unittest.TestCase):
    def test_prefix_rules(self):
        m = load()
        self.assertEqual('robot6/odom', m.prefixed('odom', 'robot6', shared=('map',)))
        self.assertEqual('robot6/base_link', m.prefixed('/base_link', 'robot6', shared=('map',)))
        self.assertEqual('map', m.prefixed('map', 'robot6', shared=('map',)))
        self.assertEqual('robot5/base_link', m.prefixed('robot5/base_link', 'robot5', shared=('map',)))

    def test_a_robots_transforms_are_renamed_without_touching_the_input(self):
        m = load()
        message = NS(transforms=[transform('map', 'odom'), transform('odom', 'base_link')])
        out = m.renamed(message.transforms, 'robot5', shared=('map',))
        self.assertEqual([('map', 'robot5/odom'), ('robot5/odom', 'robot5/base_link')],
                         [(t.header.frame_id, t.child_frame_id) for t in out])
        self.assertEqual('odom', message.transforms[0].child_frame_id)
        self.assertEqual(1.0, out[0].transform.translation.x)

    def test_statics_of_both_robots_are_kept_together(self):
        m = load()
        statics = m.StaticSet()
        statics.add(m.renamed([transform('base_link', 'rplidar_link')], 'robot6', shared=('map',)))
        statics.add(m.renamed([transform('base_link', 'rplidar_link')], 'robot5', shared=('map',)))
        statics.add(m.renamed([transform('base_link', 'rplidar_link')], 'robot6', shared=('map',)))   # again
        self.assertEqual(['robot5/rplidar_link', 'robot6/rplidar_link'],
                         sorted(t.child_frame_id for t in statics.all()))


def pose(parent, child, x, y, yaw):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=5, nanosec=0)), child_frame_id=child,
              transform=NS(translation=NS(x=x, y=y, z=0.0),
                           rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def close(test, expected, actual):
    for e, a in zip(expected, actual):
        test.assertAlmostEqual(e, a, places=9)


class Provisional(unittest.TestCase):
    """TB-5 has no map -> odom until AMCL runs on the frozen map, so RViz could not draw it while TB-6 explores
    (재범 2026-10-16 08:00 「내가 분명히 알비즈에 tb 두대 띄우게 하라고 했을텐데」). Until TB-5's own map -> odom
    comes, the merge puts a provisional one on /tf from the field file's start value (robots.<id>.initial_pose_xy_yaw,
    relative to the TB-6 start pose, the same value AMCL starts from)."""

    def test_the_start_value_and_the_start_pose_topic_come_from_the_field_file(self):
        m = load()
        field = REPO / 'config' / 'field' / 'search_area.field.json'
        offset, topic = m.provisional_source(field, 'robot_b')
        self.assertEqual((0.0, -1.86, 0.0), offset)
        self.assertEqual('/rescue/map/start_pose/robot_a', topic)

    def test_a_robot_without_a_start_value_is_refused(self):
        m = load()
        with self.assertRaises(ValueError):
            m.provisional_source(REPO / 'config' / 'field' / 'search_area.field.json', 'robot_a')

    def test_nothing_before_the_start_pose_and_the_robot_s_odometry(self):
        m = load()
        p = m.ProvisionalOdom('robot5', (0.0, -1.86, 0.0))
        self.assertIsNone(p.transform())
        p.on_start_pose((0.0, 0.0, 0.0))
        self.assertIsNone(p.transform())
        p.on_robot_tf([pose('robot5/odom', 'robot5/base_link', 0.0, 0.0, 0.0)])
        close(self, (0.0, -1.86, 0.0), p.transform())

    def test_the_robot_body_lands_on_its_start_spot_on_the_map(self):
        m = load()
        p = m.ProvisionalOdom('robot5', (0.0, -1.86, 0.0))
        p.on_start_pose((1.0, 0.0, math.pi / 2))
        p.on_robot_tf([pose('robot5/odom', 'robot5/base_link', 0.3, 0.1, 0.2)])
        mx, my, myaw = p.transform()
        # map -> odom -> base_link must be the start pose plus the start value: (1 + 1.86, 0, pi/2)
        c, s = math.cos(myaw), math.sin(myaw)
        close(self, (2.86, 0.0, math.pi / 2), (mx + c * 0.3 - s * 0.1, my + s * 0.3 + c * 0.1, myaw + 0.2))

    def test_it_stops_for_good_once_the_robot_s_own_map_to_odom_comes(self):
        m = load()
        p = m.ProvisionalOdom('robot5', (0.0, -1.86, 0.0))
        p.on_start_pose((0.0, 0.0, 0.0))
        self.assertFalse(p.on_robot_tf([pose('robot5/odom', 'robot5/base_link', 0.0, 0.0, 0.0)]))
        self.assertTrue(p.on_robot_tf([pose('map', 'robot5/odom', 0.0, -1.8, 0.0)]))   # once: log it
        self.assertIsNone(p.transform())
        self.assertFalse(p.on_robot_tf([pose('map', 'robot5/odom', 0.0, -1.8, 0.0)]))
        self.assertIsNone(p.transform())

    def test_the_provisional_edge_is_a_stamped_transform_map_to_odom(self):
        m = load()
        p = m.ProvisionalOdom('robot5', (0.0, -1.86, 0.0))
        p.on_start_pose((0.0, 0.0, 0.0))
        p.on_robot_tf([pose('robot5/odom', 'robot5/base_link', 0.0, 0.0, 0.0)])
        stamp = NS(sec=7, nanosec=8)
        t = p.stamped(stamp, NS(header=NS(frame_id=None, stamp=None), child_frame_id=None,
                                transform=NS(translation=NS(x=None, y=None, z=None),
                                             rotation=NS(x=None, y=None, z=None, w=None))))
        self.assertEqual(('map', 'robot5/odom', stamp), (t.header.frame_id, t.child_frame_id, t.header.stamp))
        close(self, (0.0, -1.86, 0.0, 0.0, 0.0, 0.0, 1.0),
              (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
               t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w))


class Launchable(unittest.TestCase):
    def test_the_tool_is_there_with_its_topics(self):
        text = TOOL.read_text(encoding='utf-8')
        for needle in ("'/tf'", "'/tf_static'", "'/rviz/' + ns + '/scan'", "'/' + ns + '/tf'", "'/' + ns + '/scan'"):
            self.assertIn(needle, text)

    def test_the_provisional_edge_goes_to_rviz_only_and_its_end_is_logged(self):
        """Nothing goes to the robots: the only TF publishers are /tf and /tf_static."""
        text = TOOL.read_text(encoding='utf-8')
        self.assertIn('TF_PROVISIONAL_OFF', text)
        self.assertEqual(["'/tf'", "'/tf_static'"],
                         re.findall(r"create_publisher\(TFMessage, ([^,]+),", text))


if __name__ == '__main__':
    unittest.main()
