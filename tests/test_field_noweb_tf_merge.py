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


class Launchable(unittest.TestCase):
    def test_the_tool_is_there_with_its_topics(self):
        text = TOOL.read_text(encoding='utf-8')
        for needle in ("'/tf'", "'/tf_static'", "'/rviz/' + ns + '/scan'", "'/' + ns + '/tf'", "'/' + ns + '/scan'"):
            self.assertIn(needle, text)


if __name__ == '__main__':
    unittest.main()
