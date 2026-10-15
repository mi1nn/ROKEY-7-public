"""The slam_toolbox parameters tb6_slam.launch.py runs with; no ROS.

TurtleBot4's slam.yaml is read as is and changed here only (실습 기록 #339 B3, 2026-10-15):
- minimum_travel_distance and minimum_travel_heading 0.1 -> 0.0. slam_toolbox only processes a scan after the
  robot moved that far, so a TB-6 standing on its start mark gave no <ns>/pose and the freeze had no start
  pose. With both 0.0, <ns>/pose came at 2 Hz in the field [측정 2026-10-15]. The map is then also updated while
  the robot stands still (map_update_interval 1.0 s stays as TurtleBot4 has it).
Anything unexpected is refused, never patched around.
"""
import copy

STANDING = {'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.0}


class SlamFieldError(ValueError):
    pass


def field_slam(params):
    """A copy of TurtleBot4 slam.yaml parameters with STANDING applied; the input is not changed."""
    node = params.get('slam_toolbox') if isinstance(params, dict) else None
    values = node.get('ros__parameters') if isinstance(node, dict) else None
    if not isinstance(values, dict):
        raise SlamFieldError('slam parameters must have slam_toolbox.ros__parameters')
    missing = sorted(set(STANDING) - set(values))
    if missing:
        raise SlamFieldError(f'slam_toolbox.ros__parameters has no {missing}: not the expected slam.yaml')
    out = copy.deepcopy(params)
    out['slam_toolbox']['ros__parameters'].update(STANDING)
    return out
