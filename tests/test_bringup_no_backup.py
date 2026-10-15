"""No backing up in Nav2 recovery, on both robots (OPS-207), no ROS.

Owner approval 2026-10-14 (재범): 「오케이 그럼 빼자」. Scenario ⑧ says no reversing: the robots
are round, so they turn in place and drive forward, and a person follows TB-5. TurtleBot4's
nav2.yaml (config/robots/turtlebot4_nav2.baseline.yaml) loads nav2_behaviors::BackUp, and
the Jazzy default behavior trees of bt_navigator (no default_*_bt_xml given) run
<BackUp backup_dist="0.30"> as the fourth recovery. Removing only the plugin would leave
the BT node waiting for a missing action server, and BtActionNode throws when it is
not there [Nav2 jazzy source, bt_action_node.hpp]. So both go: our launches remove the
backup plugin and point bt_navigator at our copies of the two default trees without BackUp.
DriveOnHeading accepts a negative speed (backwards), but no tree of ours calls it.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
BASELINE = REPO / 'config' / 'robots' / 'turtlebot4_nav2.baseline.yaml'
REVERSING = {'BackUp', 'DriveOnHeading', 'AssistedTeleop'}


def load_yaml(path):
    try:
        import yaml
    except ImportError:   # the installed-wheel check venv has no PyYAML (a ROS launch dependency)
        raise unittest.SkipTest('no PyYAML here') from None
    return yaml.safe_load(path.read_text())


def vendor_params():
    """TurtleBot4 Jazzy nav2.yaml as it was before OPS-207 (the vendor copy when the source tree is here)."""
    vendor = REPO / 'src' / 'vendor' / 'turtlebot4' / 'turtlebot4_navigation' / 'config' / 'nav2.yaml'
    if not vendor.exists():
        raise unittest.SkipTest('installed-wheel checkout: no vendor nav2.yaml')
    return load_yaml(vendor)


def tags(path):
    return {element.tag for element in ET.parse(path).iter()}


class Trees(unittest.TestCase):
    def test_our_trees_never_back_up_and_keep_the_other_recoveries(self):
        from rescue_bringup.nav2_recovery import BT_FILES, bt_dir
        for key, name in BT_FILES.items():
            with self.subTest(key=key):
                found = tags(bt_dir() / name)
                self.assertFalse(found & REVERSING, found & REVERSING)
                self.assertTrue({'Spin', 'Wait', 'ClearEntireCostmap', 'RoundRobin', 'FollowPath'} <= found, found)


class Parameters(unittest.TestCase):
    def strip(self, params):
        from rescue_bringup.nav2_recovery import nav2_without_backup
        return nav2_without_backup(params)

    def test_backup_plugin_is_removed_and_the_trees_are_ours(self):
        from rescue_bringup.nav2_recovery import BT_FILES, bt_dir
        params = self.strip(vendor_params())
        behaviors = params['behavior_server']['ros__parameters']
        self.assertNotIn('backup', behaviors['behavior_plugins'])
        self.assertNotIn('backup', behaviors)
        self.assertFalse([v for v in behaviors.values() if isinstance(v, dict) and v.get('plugin') == 'nav2_behaviors::BackUp'])
        self.assertEqual(['spin', 'drive_on_heading', 'assisted_teleop', 'wait'], behaviors['behavior_plugins'])
        navigator = params['bt_navigator']['ros__parameters']
        for key, name in BT_FILES.items():
            self.assertEqual(str(bt_dir() / name), navigator[key])

    def test_the_repository_baseline_has_no_backup_either(self):
        params = self.strip(load_yaml(BASELINE))
        self.assertNotIn('backup', params['behavior_server']['ros__parameters']['behavior_plugins'])
        self.assertNotIn('nav2_behaviors::BackUp', BASELINE.read_text())

    def test_unexpected_parameters_are_refused(self):
        from rescue_bringup.nav2_recovery import Nav2RecoveryError
        cases = {
            'no behavior_server': lambda p: p.pop('behavior_server'),
            'no bt_navigator': lambda p: p.pop('bt_navigator'),
            'another tree chosen': lambda p: p['bt_navigator']['ros__parameters'].update(
                default_nav_to_pose_bt_xml='/elsewhere/tree.xml'),
            'BackUp under another name': lambda p: p['behavior_server']['ros__parameters'].update(
                behavior_plugins=['spin', 'reverse'], reverse={'plugin': 'nav2_behaviors::BackUp'}),
        }
        for name, change in cases.items():
            params = vendor_params()
            change(params)
            with self.subTest(name), self.assertRaises(Nav2RecoveryError):
                self.strip(params)

    def test_a_tree_with_backup_is_refused(self):
        import shutil
        import tempfile
        from rescue_bringup.nav2_recovery import BT_FILES, Nav2RecoveryError, bt_dir, nav2_without_backup
        folder = Path(tempfile.mkdtemp())
        for name in BT_FILES.values():
            shutil.copy(bt_dir() / name, folder / name)
        bad = folder / BT_FILES['default_nav_through_poses_bt_xml']
        bad.write_text(bad.read_text().replace('<Wait ', '<BackUp backup_dist="0.30" backup_speed="0.15"/>\n<Wait '))
        with self.assertRaisesRegex(Nav2RecoveryError, 'BackUp'):
            nav2_without_backup(vendor_params(), trees=folder)


class Launches(unittest.TestCase):
    """Every rescue_bringup launch that starts Nav2 goes through nav2_without_backup (inside nav2_field.field_nav2)."""

    def test_nav2_launches_use_it(self):
        launch = REPO / 'src' / 'rescue_bringup' / 'launch'
        if not launch.exists():
            self.skipTest('installed-wheel checkout: no launch sources')
        starting = [p for p in sorted(launch.glob('*.launch.py')) if "'nav2.launch.py'" in p.read_text()]
        self.assertEqual(['localization_navigation.launch.py', 'mapping_navigation.launch.py', 'robot.launch.py'],
                         [p.name for p in starting])
        for path in starting:
            with self.subTest(path.name):
                self.assertIn('field_nav2', path.read_text())
        from rescue_bringup.nav2_field import field_nav2
        params = field_nav2(vendor_params())
        self.assertNotIn('backup', params['behavior_server']['ros__parameters']['behavior_plugins'])



RUNBOOK = REPO / 'docs' / 'process' / 'field-runbook.md'


@unittest.skipUnless(RUNBOOK.exists(), 'installed-wheel checkout has no docs/')
class RunbookCheck(unittest.TestCase):
    def test_the_bt_check_finds_the_trees_the_launch_uses_in_either_build(self):
        """A --symlink-install build has no <prefix>/lib/python3*/site-packages/rescue_bringup (field 2026-10-15):
        the check asks nav2_recovery.bt_dir(), the folder the launch hands to bt_navigator."""
        text = RUNBOOK.read_text()
        self.assertNotIn('site-packages/rescue_bringup/behavior_trees', text)
        self.assertIn('from rescue_bringup.nav2_recovery import bt_dir', text)

if __name__ == '__main__':
    unittest.main()
