# Extra TurtleBot 4 in a running simulation, for run_sim.sh --also (2026-10-15).
#
# Same as turtlebot4_gz_bringup turtlebot4_spawn.launch.py (turtlebot4_simulator 2.0.2, Copyright 2023
# Clearpath Robotics, Apache License 2.0) without its optional SLAM/Nav2/RViz, with one change: the robot is
# created from robot_description_sim, which strip_sensors.py republishes without the Gazebo Sensors system.
# A second Sensors system aborts the server (see strip_sensors.py); the first robot's one renders all sensors.
from pathlib import Path

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
    DeclareLaunchArgument('namespace', description='Robot namespace, no leading slash'),
    DeclareLaunchArgument('world', description='Gazebo world name'),
]
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0'))

STRIP = str(Path(__file__).resolve().parent / 'strip_sensors.py')


def generate_launch_description():
    gz_bringup = FindPackageShare('turtlebot4_gz_bringup')
    namespace = LaunchConfiguration('namespace')
    x, y, z, yaw = (LaunchConfiguration(k) for k in ('x', 'y', 'z', 'yaw'))
    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    group = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('turtlebot4_description'), 'launch', 'robot_description.launch.py'])),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('irobot_create_common_bringup'), 'launch', 'dock_description.launch.py'])),
            launch_arguments={'gazebo': 'ignition'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', robot_name, '-x', x, '-y', y, '-z', OffsetParser(z, -0.0025), '-Y', yaw,
                        '-topic', 'robot_description_sim']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', dock_name,
                        '-x', OffsetParser(x, RotationalOffsetX(0.157, yaw)),
                        '-y', OffsetParser(y, RotationalOffsetY(0.157, yaw)),
                        '-z', z, '-Y', OffsetParser(yaw, 3.1416), '-topic', 'standard_dock_description']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([gz_bringup, 'launch', 'ros_gz_bridge.launch.py'])),
            launch_arguments=[('model', LaunchConfiguration('model')), ('robot_name', robot_name),
                              ('dock_name', dock_name), ('namespace', namespace),
                              ('world', LaunchConfiguration('world'))]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [gz_bringup, 'launch', 'turtlebot4_nodes.launch.py'])),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('param_file', PathJoinSubstitution([gz_bringup, 'config', 'turtlebot4_node.yaml']))]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('irobot_create_common_bringup'), 'launch', 'create3_nodes.launch.py'])),
            launch_arguments=[('namespace', namespace)]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('irobot_create_gz_bringup'), 'launch', 'create3_gz_nodes.launch.py'])),
            launch_arguments=[('robot_name', robot_name), ('dock_name', dock_name)]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='rplidar_stf', output='screen',
             arguments=['0', '0', '0', '0', '0', '0.0', 'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_stf', output='screen',
             arguments=['0', '0', '0', '1.5707', '-1.5707', '0', 'oakd_rgb_camera_optical_frame',
                        [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ])
    strip = ExecuteProcess(cmd=['python3', STRIP, '--ros-args', '-r', ['__ns:=/', namespace],
                                '-p', 'use_sim_time:=true'], output='screen')
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(strip)
    ld.add_action(group)
    return ld
