"""Run existing TurtleBot4 Nav2 with node-specific final velocity remaps.

The watchdog runs outside the remap group. Controller/smoother remaps stay
untouched. Field deployment must remove any independent final cmd_vel writer.
No task input, arm, reset or movement command is emitted by this launch file.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from pathlib import Path


def setup(context):
    ns = LaunchConfiguration('namespace').perform(context).rstrip('/')
    if not ns.startswith('/') or ns == '':
        raise ValueError('explicit absolute physical namespace required')
    nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    upstream = ns + '/cmd_vel_watchdog_in'
    nav2 = GroupAction([
        SetRemap(src='collision_monitor:' + ns + '/cmd_vel', dst=upstream),
        SetRemap(src='docking_server:' + ns + '/cmd_vel', dst=upstream),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / 'launch' / 'nav2.launch.py')),
            launch_arguments={'namespace': ns, 'use_sim_time': 'false',
                'params_file': LaunchConfiguration('params_file')}.items()),
    ])
    watchdog = Node(package='rescue_robot', executable='command_watchdog',
        name='rescue_command_watchdog', namespace=ns, output='screen',
        parameters=[{
            'namespace': ns,
            'cmd_vel_in_topic': upstream, 'cmd_vel_out_topic': ns + '/cmd_vel',
            'cmd_vel_stamped': True,
            'executor_heartbeat_topic': ns + '/rescue/executor_heartbeat',
            'watchdog_heartbeat_topic': ns + '/rescue/watchdog_heartbeat',
            'step_period': float(LaunchConfiguration('step_period').perform(context)),
            'watchdog.version': LaunchConfiguration('watchdog_version').perform(context),
            **{'watchdog.' + key: float(LaunchConfiguration(key).perform(context)) for key in
                ('heartbeat_max_age_s', 'command_max_age_s', 'max_linear_mps', 'max_angular_rps')},
        }])
    return [nav2, watchdog]


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4_navigation'))
    names = ('namespace', 'watchdog_version', 'step_period', 'heartbeat_max_age_s',
             'command_max_age_s', 'max_linear_mps', 'max_angular_rps')
    return LaunchDescription([
        *(DeclareLaunchArgument(name) for name in names),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'nav2.yaml')),
        OpaqueFunction(function=setup),
    ])
