"""Read-only shared-map RViz configuration from the deployment, no ROS imports.

User request 2026-10-15: native AMR2 TF and robot model plus both robots' valid
mission-pose markers. Duplicate native odom/base_link names are never merged.
Display colors, layout and camera scale are presentation choices, not field policy.
"""
from rescue_bringup.field_launch import FieldLaunchError
from rescue_bringup.profile import require_map_basis

MARKER_TOPIC = '/rescue/view/robots'


def view_values(deployment, field, *, robot_id='', show_all=True, marker_topic=MARKER_TOPIC,
                description_topic=''):
    basis = require_map_basis(deployment)
    if field.coordinator['map_mode'] != 'SLAM_LIVE' or deployment.mission is None:
        raise FieldLaunchError('shared view requires SLAM_LIVE with mission roles')
    robots = {r.id: r for r in deployment.robots}
    if set(robots) != set(field.robots):
        raise FieldLaunchError('field and profile robot IDs differ')
    roles = {role: robot for robot, role in deployment.mission.roles}
    focus = robots.get(robot_id or roles['SUPPORT'])
    if focus is None or focus.parameters.readiness is None:
        raise FieldLaunchError('selected view robot needs profile readiness and TF topics')
    use_sim_time = focus.parameters.use_sim_time
    if any(r.parameters.use_sim_time != use_sim_time for r in robots.values()):
        raise FieldLaunchError('shared map view requires one ROS clock mode')
    description_topic = description_topic or focus.namespace + '/robot_description'
    if any(not isinstance(t, str) or not t.startswith('/') or t.endswith('/')
           for t in (marker_topic, description_topic)):
        raise FieldLaunchError('absolute marker and robot description topics required')
    selected = list(robots) if show_all else [focus.id]
    return dict(frame=basis.frame_id, map_topic=robots[roles['SCOUT']].namespace + '/map',
                focus_robot_id=focus.id, namespace=focus.namespace, use_sim_time=use_sim_time,
                tf_topic=focus.parameters.readiness.tf_topic,
                tf_static_topic=focus.namespace + '/tf_static', description_topic=description_topic,
                marker_topic=marker_topic,
                markers=dict(robot_ids=selected, frame=basis.frame_id, map_session_id=basis.map_session_id,
                             map_revision=basis.map_revision, transform_revision=basis.transform_revision,
                             marker_topic=marker_topic))


def _topic(name, *, latched=False):
    return {'Value': name, 'Depth': 1, 'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local' if latched else 'Volatile'}


def rviz_config(values):
    """JSON-serializable YAML document for RViz, with no Nav2 goal/initial-pose tools."""
    return {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                   {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {
            'Class': '', 'Name': 'root', 'Enabled': True,
            'Global Options': {'Fixed Frame': values['frame'], 'Frame Rate': 30,
                               'Background Color': '38; 40; 44'},
            'Displays': [
                {'Class': 'rviz_default_plugins/Map', 'Name': 'AMR1 live SLAM map', 'Enabled': True,
                 'Value': True, 'Alpha': .8, 'Color Scheme': 'map',
                 'Topic': _topic(values['map_topic'], latched=True)},
                {'Class': 'rviz_default_plugins/RobotModel',
                 'Name': values['focus_robot_id'] + ' model (AMCL TF)', 'Enabled': True, 'Value': True,
                 'Description Source': 'Topic', 'Description Topic': _topic(values['description_topic'], latched=True),
                 'Description File': '', 'TF Prefix': '', 'Visual Enabled': True, 'Collision Enabled': False,
                 'Update Interval': 0., 'Alpha': .7},
                {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Validated robot positions',
                 'Enabled': True, 'Value': True, 'Topic': _topic(values['marker_topic'])},
                {'Class': 'rviz_default_plugins/TF', 'Name': 'Selected robot native TF',
                 'Enabled': False, 'Value': False, 'Frame Timeout': 2.,
                 'Show Arrows': True, 'Show Axes': True, 'Show Names': True},
            ],
            'Tools': [{'Class': 'rviz_default_plugins/' + tool}
                      for tool in ('MoveCamera', 'Select', 'FocusCamera', 'Measure')],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Map',
                                  'Target Frame': '<Fixed Frame>', 'Scale': 60., 'Angle': -1.5707963267948966,
                                  'X': 0., 'Y': 0.}},
        },
        'Window Geometry': {'Width': 1200, 'Height': 850},
    }
