"""scripts/field/noweb/two_robots.rviz: the RViz layout of the runs without the web console (웹 없는 트랙, 2026-10-15).

Syntax only (E1): the file parses, the fixed frame is map, and it shows exactly the topics agreed with 작전
(OPS-289 follow-up), the frozen map latched as the coordinator publishes it. It has no camera and no tool that
publishes (no goal, no initial pose). What RViz actually draws is a field check (NOT_RUN).
"""
from pathlib import Path
import re
import unittest

from rescue_coordination import ros_node as coordinator

REPO = Path(__file__).resolve().parents[1]
NOWEB = REPO / 'scripts' / 'field' / 'noweb'
if not (NOWEB / 'operator_command.py').is_file():
    # as tests/test_field_noweb_status_watch.py: repo scripts, not part of the installed-wheel checkout
    raise unittest.SkipTest('scripts/field/noweb/ absent (installed-wheel checkout)')
RVIZ = NOWEB / 'two_robots.rviz'
TOPICS = {'/robot6/map', '/robot6/map_updates', coordinator.FROZEN_MAP_TOPIC, '/robot6/robot_description',
          '/robot6/scan', '/robot6/plan', '/robot5/amcl_pose', '/robot5/plan',
          '/rescue/noweb/markers'}   # target_markers.py: targets and QR exits (OPS, 2026-10-15)


def displays(text):
    """(class, {topic key: (topic, durability)}) per display, read from the 2-space layout RViz writes."""
    found = []
    section = '\n' + text.split('\n  Displays:\n', 1)[1].split('\n  Tools:\n', 1)[0]
    for block in re.split(r'\n    - Class: ', section)[1:]:
        topics = {}
        for key, body in re.findall(r'(?m)^      ((?:Update |Description )?Topic):\n((?:        .*\n?)+)', block):
            value = re.search(r'Value: (\S+)', body).group(1)
            durability = re.search(r'Durability Policy: (.+)', body)
            topics[key] = (value, durability and durability.group(1).strip())
        found.append((block.split('\n', 1)[0].strip(), topics))
    return found


class RvizTests(unittest.TestCase):
    def setUp(self):
        self.text = RVIZ.read_text(encoding='utf-8')

    def test_layout_is_plain_two_space_yaml(self):
        self.assertNotIn('\t', self.text)
        for line in self.text.splitlines():
            self.assertEqual(0, (len(line) - len(line.lstrip(' '))) % 2, line)

    def test_it_loads_as_yaml_when_pyyaml_is_there(self):
        try:
            import yaml
        except ImportError:
            self.skipTest('PyYAML absent: the regex checks below still run')
        data = yaml.safe_load(self.text)
        self.assertEqual('map', data['Visualization Manager']['Global Options']['Fixed Frame'])
        self.assertEqual(len(displays(self.text)), len(data['Visualization Manager']['Displays']))

    def test_fixed_frame_is_map(self):
        self.assertRegex(self.text, r'\n    Fixed Frame: map\n')

    def test_exactly_the_agreed_topics(self):
        shown = {topic for _, topics in displays(self.text) for topic, _ in topics.values()}
        self.assertEqual(TOPICS, shown)

    def test_frozen_map_and_amcl_pose_are_latched(self):
        latched = {topic: durability for _, topics in displays(self.text) for topic, durability in topics.values()}
        self.assertEqual('Transient Local', latched[coordinator.FROZEN_MAP_TOPIC])
        self.assertEqual('Transient Local', latched['/robot5/amcl_pose'])
        self.assertEqual('Transient Local', latched['/robot6/map'])

    def test_no_camera_and_no_publishing_tool(self):
        classes = {cls for cls, _ in displays(self.text)}
        self.assertFalse({c for c in classes if 'Image' in c or 'Camera' in c}, classes)
        tools = self.text.split('\n  Tools:\n', 1)[1].split('\n  Views:\n', 1)[0]
        self.assertEqual(['rviz_default_plugins/MoveCamera'], re.findall(r'Class: (\S+)', tools))
        self.assertNotIn('/rescue/search_goal', self.text)

    def test_target_markers_are_shown(self):
        classes = {topics['Topic'][0]: cls for cls, topics in displays(self.text) if 'Topic' in topics}
        self.assertEqual('rviz_default_plugins/MarkerArray', classes.get('/rescue/noweb/markers'))


if __name__ == '__main__':
    unittest.main()
