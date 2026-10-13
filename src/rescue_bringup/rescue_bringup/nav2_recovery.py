"""Nav2 recovery without backing up, on both robots (OPS-207); no ROS.

Owner approval 2026-10-14 (재범, grade 4): 「오케이 그럼 빼자」. Scenario ⑧: no reversing (round
robots turn in place and drive forward), and a person follows TB-5. TurtleBot4's nav2.yaml loads
nav2_behaviors::BackUp and the Jazzy default trees of bt_navigator run BackUp as a recovery. A
tree node whose action server is missing makes the tree fail to load [Nav2 jazzy
bt_action_node.hpp], so the plugin and the tree node go together: the backup plugin is removed
and bt_navigator gets our copies of the two default trees without BackUp (behavior_trees/).
Anything unexpected is refused, never patched around.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

BT_FILES = {'default_nav_to_pose_bt_xml': 'navigate_to_pose_no_backup.xml',
            'default_nav_through_poses_bt_xml': 'navigate_through_poses_no_backup.xml'}
BACKUP_PLUGIN = 'nav2_behaviors::BackUp'
BACKUP_NODE = 'BackUp'


class Nav2RecoveryError(ValueError):
    pass


def bt_dir():
    return Path(__file__).resolve().parent / 'behavior_trees'


def nav2_without_backup(params, *, trees=None):
    """A copy of the Nav2 parameters (dicts) without the backup behavior, bt_navigator on our trees."""
    params = copy.deepcopy(params)
    trees = bt_dir() if trees is None else Path(trees)
    try:
        behaviors = params['behavior_server']['ros__parameters']
        plugins = list(behaviors['behavior_plugins'])
        navigator = params['bt_navigator']['ros__parameters']
    except (KeyError, TypeError):
        raise Nav2RecoveryError('Nav2 parameters need behavior_server behavior_plugins and bt_navigator') from None
    if 'backup' in plugins:
        plugins.remove('backup')
    behaviors.pop('backup', None)
    named = [name for name in plugins if isinstance(behaviors.get(name), dict)
             and behaviors[name].get('plugin') == BACKUP_PLUGIN]
    if named:
        raise Nav2RecoveryError(f'behavior_server still loads {BACKUP_PLUGIN} as {", ".join(named)} (OPS-207)')
    behaviors['behavior_plugins'] = plugins
    for key, name in BT_FILES.items():
        path = trees / name
        if navigator.get(key) not in (None, str(path)):
            raise Nav2RecoveryError(f'bt_navigator {key} is already {navigator[key]}; '
                                    f'OPS-207 needs {path} (no BackUp)')
        try:
            found = {element.tag for element in ET.parse(path).iter()}
        except (OSError, ET.ParseError) as error:
            raise Nav2RecoveryError(f'behavior tree {path} unreadable: {error}') from None
        if BACKUP_NODE in found:
            raise Nav2RecoveryError(f'behavior tree {path} has a {BACKUP_NODE} node (OPS-207)')
        navigator[key] = str(path)
    return params


def write_nav2_without_backup(base_path, *, prefix):
    """Read the Nav2 yaml at ``base_path``, strip the backup, write a temporary yaml; returns its path."""
    import tempfile

    import yaml

    params = nav2_without_backup(yaml.safe_load(Path(base_path).read_text()))
    with tempfile.NamedTemporaryFile('w', prefix=prefix, suffix='.yaml', delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name
