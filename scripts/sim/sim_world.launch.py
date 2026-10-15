"""Gazebo with a world and the first TurtleBot 4 (run_sim.sh, sim_robots.sh; 2026-10-15, -16).

turtlebot4_gz_bringup turtlebot4_gz.launch.py (sim.launch.py + turtlebot4_spawn.launch.py, turtlebot4_simulator Jazzy)
with two changes:
- gui:=false runs the Gazebo server only (gz sim -s): the window costs about a core and nobody watches an unattended run.
- The robot and its dock come from spawn_extra.launch.py with strip_sensors:=false (this robot keeps the Sensors system
  that renders every sensor). There the dock is made without its IR emitter rays, which made other lidars pass through
  walls (config/sim/README.md S10). turtlebot4_spawn.launch.py has no argument for that.
Arguments: world, namespace, x, y, z, yaw, gui. The resource paths and the clock bridge are as in sim.launch.py.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

SPAWN = str(Path(__file__).resolve().parent / 'spawn_extra.launch.py')


def generate_launch_description():
    bringup = get_package_share_directory('turtlebot4_gz_bringup')
    arguments = [DeclareLaunchArgument('world', default_value='warehouse'),
                 DeclareLaunchArgument('namespace', default_value=''),
                 DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'])]
    arguments += [DeclareLaunchArgument(k, default_value='0.0') for k in ('x', 'y', 'z', 'yaw')]
    resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
        str(Path(bringup) / 'worlds'),
        str(Path(get_package_share_directory('irobot_create_gz_bringup')) / 'worlds'),
        str(Path(get_package_share_directory('turtlebot4_description')).parent.resolve()),
        str(Path(get_package_share_directory('irobot_create_description')).parent.resolve())]))
    gui_plugin_path = SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', ':'.join([
        str(Path(get_package_share_directory('turtlebot4_gz_gui_plugins')) / 'lib'),
        str(Path(get_package_share_directory('irobot_create_gz_plugins')) / 'lib')]))
    gui_config = str(Path(bringup) / 'gui' / 'standard' / 'gui.config')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world'), '.sdf -r -v 4 ',
                                       PythonExpression(["'--gui-config " + gui_config + "' if '",
                                                         LaunchConfiguration('gui'), "' == 'true' else '-s'"])])])
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([SPAWN]),
        launch_arguments=[('namespace', LaunchConfiguration('namespace')), ('world', LaunchConfiguration('world')),
                          ('strip_sensors', 'false')]
        + [(k, LaunchConfiguration(k)) for k in ('x', 'y', 'z', 'yaw')])
    return LaunchDescription(arguments + [resource_path, gui_plugin_path, gazebo, clock, spawn])
