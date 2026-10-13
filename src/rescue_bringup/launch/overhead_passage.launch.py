"""<ns>/passage_points -> overhead structures (tunnels) on the map (OPS-185/186).

    ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=robot_a

rescue_perception overhead_passage reads the passage depth points of that robot (run
passage_points.launch.py first) with the passage.* box and heights of the field run file, and
publishes /rescue/robots/<id>/overhead_passages (latched JSON, C-04b). The robot's TF is
namespaced, so /tf and /tf_static are remapped. No arm, task or movement command is sent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup.field_launch import load_field, overhead_passage_launch_values
from rescue_bringup.profile import load_deployment


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    v = overhead_passage_launch_values(load_deployment(arg('profile_path')), field=load_field(arg('field_path')),
                                       robot_id=arg('robot_id'))
    p = v['parameters']
    return [
        LogInfo(msg=f'overhead passages {p["points_topic"]} -> /rescue/robots/{p["robot_id"]}/overhead_passages '
                    f'(clearance {p["clearance_m"]} m, LiDAR plane {p["lidar_plane_m"]} m)'),
        Node(package='rescue_perception', executable='overhead_passage', name='overhead_passage',
             namespace=v['namespace'], output='screen', parameters=[p],
             remappings=[('/tf', v['tf_topic']), ('/tf_static', v['tf_static_topic'])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', description='Deployment profile v2.1 JSON (outside the repo)'),
        DeclareLaunchArgument('field_path', description='Field run file (config/field/*.field.json)'),
        DeclareLaunchArgument('robot_id', description='robot_a (TB-6) or robot_b (TB-5)'),
        OpaqueFunction(function=_nodes),
    ])
