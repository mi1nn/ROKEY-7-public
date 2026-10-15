"""scripts/sim/rviz_keepout.py: the two-robot RViz of the field plus the tunnel keep-out mask, for run_full.sh."""
from pathlib import Path
import sys
import unittest

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'rviz_keepout.py'
FIELD_RVIZ = REPO / 'scripts' / 'field' / 'noweb' / 'two_robots.rviz'
sys.path.insert(0, str(TOOL.parent))


@unittest.skipUnless(TOOL.exists() and FIELD_RVIZ.exists(), 'installed-wheel checkout has no scripts/')
class RvizKeepout(unittest.TestCase):
    def setUp(self):
        import rviz_keepout
        self.tool = rviz_keepout
        self.field = FIELD_RVIZ.read_text(encoding='utf-8')

    def test_the_mask_comes_once_after_the_frozen_map_and_nothing_else_changes(self):
        out = self.tool.with_keepout(self.field)
        self.assertEqual(1, out.count('/rescue/passage_keepout/mask'))
        self.assertLess(out.index('Name: Frozen map'), out.index('Name: Tunnel keep-out mask'))
        self.assertLess(out.index('Name: Tunnel keep-out mask'), out.index('Name: TB-6 model'))
        self.assertEqual(self.field, out.replace(self.tool.DISPLAY, '', 1))

    def test_the_result_is_still_yaml_with_one_more_display(self):
        try:
            import yaml
        except ImportError:
            self.skipTest('PyYAML not installed')
        before = yaml.safe_load(self.field)['Visualization Manager']['Displays']
        after = yaml.safe_load(self.tool.with_keepout(self.field))['Visualization Manager']['Displays']
        self.assertEqual(len(before) + 1, len(after))
        mask = [d for d in after if d.get('Name') == 'Tunnel keep-out mask'][0]
        self.assertEqual('/rescue/passage_keepout/mask', mask['Topic']['Value'])
        self.assertEqual('Transient Local', mask['Topic']['Durability Policy'])

    def test_a_file_it_does_not_recognise_is_refused(self):
        for text in ('Panels: []\n', self.tool.with_keepout(self.field)):
            with self.subTest(text=text[:20]), self.assertRaises(ValueError):
                self.tool.with_keepout(text)

    def test_run_full_shows_the_merged_tf_and_this_rviz(self):
        text = (REPO / 'scripts' / 'sim' / 'run_full.sh').read_text()
        self.assertIn('scripts/field/noweb/tf_merge.py', text)
        self.assertIn('rviz_keepout.py', text)
        self.assertIn('rviz2 -d', text)


if __name__ == '__main__':
    unittest.main()
