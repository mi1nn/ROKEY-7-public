"""Supervised single-robot mapping setup; exploration is started separately."""
from pathlib import Path
import hashlib
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    sha = subprocess.check_output(
        ['git', '-C', str(folder), 'rev-parse', 'HEAD'], text=True
    ).strip()
    namespace = LaunchConfiguration('namespace')
    actions = [DeclareLaunchArgument('namespace', description='Assigned robot namespace')]
    for filename in ('slam.yaml', 'nav2.yaml'):
        digest = hashlib.sha256((folder / filename).read_bytes()).hexdigest()
        actions.append(LogInfo(msg=f'git={sha} config={filename} sha256={digest}'))
    for package, launch_file, extra in (
        ('turtlebot4_navigation', 'slam.launch.py', {'params': str(folder / 'slam.yaml')}),
        ('turtlebot4_navigation', 'nav2.launch.py', {'params_file': str(folder / 'nav2.yaml')}),
        ('turtlebot4_viz', 'view_navigation.launch.py', {}),
    ):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(package)) / 'launch' / launch_file
            )),
            launch_arguments={
                'namespace': namespace, 'use_sim_time': 'false', **extra
            }.items(),
        ))
    return LaunchDescription(actions)
