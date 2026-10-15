"""turtlebot4_gz.launch.py without the Gazebo window: gz sim runs server-only (-s), robot and bridges as usual.

Used by run_sim.sh when RUN_SIM_GUI=0 (the window and its renderer cost about one core on a PC that runs two
simulations; nobody watches it during an unattended run). Same arguments as turtlebot4_gz.launch.py:
world, namespace, x, y, z, yaw. Written for turtlebot4_simulator Jazzy (turtlebot4_gz_bringup sim.launch.py,
2026-10-15): the resource path, the clock bridge and the spawn launch are copied from it.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('turtlebot4_gz_bringup')
    arguments = [DeclareLaunchArgument('world', default_value='warehouse'),
                 DeclareLaunchArgument('namespace', default_value='')]
    arguments += [DeclareLaunchArgument(k, default_value='0.0') for k in ('x', 'y', 'z', 'yaw')]
    resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
        str(Path(bringup) / 'worlds'),
        str(Path(get_package_share_directory('irobot_create_gz_bringup')) / 'worlds'),
        str(Path(get_package_share_directory('turtlebot4_description')).parent.resolve()),
        str(Path(get_package_share_directory('irobot_create_description')).parent.resolve())]))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world'), '.sdf -r -s -v 4'])])
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([bringup, 'launch', 'turtlebot4_spawn.launch.py'])]),
        launch_arguments=[('namespace', LaunchConfiguration('namespace')), ('rviz', 'false')]
        + [(k, LaunchConfiguration(k)) for k in ('x', 'y', 'z', 'yaw')])
    return LaunchDescription(arguments + [resource_path, gazebo, clock, spawn])
