"""Start the standalone upstream explorer after the operator readiness checks."""
from pathlib import Path
import hashlib
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_explorer(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    if not namespace or not namespace.replace('_', '').isalnum():
        raise ValueError('namespace must be one nonempty robot name')
    folder = Path(__file__).resolve().parent
    params = folder / 'explore.yaml'
    sha = subprocess.check_output(
        ['git', '-C', str(folder), 'rev-parse', 'HEAD'], text=True
    ).strip()
    digest = hashlib.sha256(params.read_bytes()).hexdigest()
    return [
        LogInfo(msg=f'git={sha} config=explore.yaml sha256={digest} namespace={namespace}'),
        Node(package='explore_lite', executable='explore', namespace=namespace,
             parameters=[str(params), {'use_sim_time': False}],
             remappings=[('/tf', f'/{namespace}/tf'),
                         ('/tf_static', f'/{namespace}/tf_static')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Assigned robot namespace'),
        OpaqueFunction(function=start_explorer),
    ])
