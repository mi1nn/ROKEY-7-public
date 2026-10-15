"""Rehearsal only: the real target_locator and exit_locator (amr_perception.launch.py's two locators) without the
OAK-D detector, which sim_world.py replaces. TF is TB-6's own (SLAM map -> odom, simulated odom -> base_link).

    ros2 launch scripts/field/full-scenario-rehearsal/perception.launch.py profile_path:=<p> field_path:=<f>
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import load_field
from rescue_bringup.profile import load_deployment, require_map_basis
from rescue_perception.exit_qr import EXIT_LOCATION_TOPIC, EXIT_OBSERVATIONS_TOPIC
from rescue_perception.target_locator import profile_parameters


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    deployment, field = load_deployment(arg('profile_path')), load_field(arg('field_path'))
    now = time.time_ns()
    locator = profile_parameters(deployment, require_map_basis(deployment), tf_wait_s=2.0,
                                 clock_domain_id=field.coordinator['target_clock_domain_id'],
                                 alignment_authority_id='rehearsal-identity-alignment',
                                 alignment_valid_from_ns=now - 3_000_000_000,
                                 alignment_valid_until_ns=now + 3600 * 1_000_000_000)
    scout = locator['robot_id']
    ns = {r.id: r.parameters.namespace for r in deployment.robots}[scout]
    exit_locator = locator | dict(observations_topic=EXIT_OBSERVATIONS_TOPIC, target_locations_topic=EXIT_LOCATION_TOPIC,
                                  publisher_id='amr.exit_locator')
    tf = [('/tf', ns + '/tf'), ('/tf_static', ns + '/tf_static')]
    return [Node(package='rescue_perception', executable='target_locator', name='target_locator', output='screen',
                 parameters=[locator], remappings=tf),
            Node(package='rescue_perception', executable='target_locator', name='exit_locator', output='screen',
                 parameters=[exit_locator], remappings=tf)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('profile_path'), DeclareLaunchArgument('field_path'),
                              OpaqueFunction(function=_nodes)])
