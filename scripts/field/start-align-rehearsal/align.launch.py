"""E3: AMCL + lifecycle + start_align exactly as frozen_localization.launch.py groups them (TB-5, /robot5).

Parameters come from rescue_bringup.field_launch (amcl_parameters, frozen_localization_values) with the
repository field file and the test profile; the measured start is MEASURED (x y yaw on the map).
"""
import json
import os
from pathlib import Path
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, SetRemap
import yaml

REPO = Path(os.environ['E3_REPO'])
sys.path.insert(0, str(REPO / 'tests'))
from test_bringup_field_launch import field_profile   # noqa: E402

from rescue_bringup.field_launch import amcl_parameters, frozen_localization_values, load_field   # noqa: E402
from rescue_bringup.profile import load_deployment   # noqa: E402
from rescue_bringup.ros_parameters import typed   # noqa: E402


def generate_launch_description():
    measured = [float(v) for v in os.environ['E3_MEASURED'].split()]
    folder = Path(tempfile.mkdtemp(prefix='e3align_'))
    deployment = load_deployment(field_profile(folder))
    raw = json.loads((REPO / 'config' / 'field' / 'search_area.field.json').read_text())
    raw['robots']['robot_b']['initial_pose_xy_yaw'] = [0.0, 0.0, 0.0]
    (folder / 'field.json').write_text(json.dumps(raw))
    values = frozen_localization_values(deployment, field=load_field(folder / 'field.json'), robot_id='robot_b')
    ns = values['namespace']
    loc = yaml.safe_load((REPO / 'src/vendor/turtlebot4/turtlebot4_navigation/config/localization.yaml').read_text())
    amcl = amcl_parameters(loc, initial_pose_xy_yaw=measured, use_sim_time=False)
    print('E3 start_align', json.dumps(values['start_align']), 'amcl start', measured, flush=True)
    return LaunchDescription([GroupAction([
        SetRemap('/tf', ns + '/tf'), SetRemap('/tf_static', ns + '/tf_static'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=ns, output='screen',
             parameters=[typed(amcl)]),
        Node(package='rescue_robot', executable='start_align', name='rescue_start_align', namespace=ns,
             output='screen', parameters=[typed(values['start_align'])]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             namespace=ns, output='screen', parameters=[{'use_sim_time': False, 'autostart': True,
                                                          'node_names': ['amcl']}]),
    ])])
