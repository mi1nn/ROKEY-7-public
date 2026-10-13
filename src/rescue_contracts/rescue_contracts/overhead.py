"""Overhead passages (tunnels) on the map: the JSON text each robot publishes (C-04b, OPS-185/186).

Each robot's overhead_passage node publishes its whole list on /rescue/robots/<id>/overhead_passages
(std_msgs/String, RELIABLE, TRANSIENT_LOCAL, depth 1; no new IDL). An item is a structure floating
above the LiDAR plane ahead of the robot, at its floor centre in the map frame, with the lowest
height seen: PASSABLE (>= clearance_m, the 46 cm owner decision), BLOCKED (below), UNKNOWN (the
floor under it was not seen, so no height is claimed). Each item keeps the map session/revision
it was measured on. db_writer, the web map and the coordinator read it only through
parse_overhead_passages; any other text, missing or extra field or inconsistent value is refused.
"""
import json
import math

from . import robot_id as _robot_id
from . import uuid_string

OVERHEAD_PASSAGES_FORMAT = 'rescue-overhead-passages/1'
STATES = ('PASSABLE', 'BLOCKED', 'UNKNOWN')
_TOP_KEYS = frozenset({'format', 'robot_id', 'map_frame', 'generated_at_ns', 'clearance_m', 'floor_noise_m',
                       'lidar_plane_m', 'items'})
_ITEM_KEYS = frozenset({'id', 'state', 'lowest_m', 'x', 'y', 'map_session_id', 'map_revision', 'frames',
                        'first_seen_ns', 'last_seen_ns', 'reason'})


def overhead_passages_topic(robot_id):
    return f'/rescue/robots/{_robot_id(robot_id)}/overhead_passages'


def _require(ok, detail):
    if not ok:
        raise ValueError(f'overhead passages: {detail}')


def _float(value, name):
    _require(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
             f'{name} must be a finite number')
    return float(value)


def _int(value, name, low):
    _require(type(value) is int and value >= low, f'{name} must be an integer >= {low}')
    return value


def _item(value, clearance_m):
    _require(isinstance(value, dict) and set(value) == _ITEM_KEYS, f'an item has exactly {sorted(_ITEM_KEYS)}')
    state, lowest = value['state'], value['lowest_m']
    _require(state in STATES, f'state must be one of {STATES}')
    if state == 'UNKNOWN':
        _require(lowest is None, 'UNKNOWN has no lowest_m')
    else:
        lowest = _float(lowest, 'lowest_m')
        _require((lowest >= clearance_m) == (state == 'PASSABLE'), f'{state} does not match lowest_m {lowest}')
    _require(isinstance(value['map_session_id'], str), 'map_session_id must be a UUID')
    uuid_string(value['map_session_id'])
    first, last = _int(value['first_seen_ns'], 'first_seen_ns', 0), _int(value['last_seen_ns'], 'last_seen_ns', 0)
    _require(first <= last, 'last_seen_ns before first_seen_ns')
    _require(isinstance(value['reason'], str), 'reason must be a string')
    return dict(id=uuid_string(value['id']), state=state, lowest_m=lowest, x=_float(value['x'], 'x'),
                y=_float(value['y'], 'y'), map_session_id=value['map_session_id'],
                map_revision=_int(value['map_revision'], 'map_revision', 0), frames=_int(value['frames'], 'frames', 1),
                first_seen_ns=first, last_seen_ns=last, reason=value['reason'])


def _fields(robot_id, map_frame, generated_at_ns, clearance_m, floor_noise_m, lidar_plane_m, items):
    _robot_id(robot_id)
    _require(isinstance(map_frame, str) and map_frame.strip(), 'map_frame must be a non-empty string')
    _int(generated_at_ns, 'generated_at_ns', 0)
    clearance, floor = _float(clearance_m, 'clearance_m'), _float(floor_noise_m, 'floor_noise_m')
    lidar = _float(lidar_plane_m, 'lidar_plane_m')
    _require(0 <= floor < lidar < clearance, 'need 0 <= floor_noise_m < lidar_plane_m < clearance_m')
    _require(isinstance(items, list), 'items must be a list')
    checked = [_item(i, clearance) for i in items]
    _require(len({i['id'] for i in checked}) == len(checked), 'item ids must be distinct')
    return dict(robot_id=robot_id, map_frame=map_frame, generated_at_ns=generated_at_ns, clearance_m=clearance,
                floor_noise_m=floor, lidar_plane_m=lidar, items=checked)


def overhead_passages_text(*, robot_id, map_frame, generated_at_ns, clearance_m, floor_noise_m, lidar_plane_m, items):
    """The canonical text of one robot's list (sorted keys, no spaces); ValueError for any bad value."""
    fields = _fields(robot_id, map_frame, generated_at_ns, clearance_m, floor_noise_m, lidar_plane_m, list(items))
    return json.dumps(dict(format=OVERHEAD_PASSAGES_FORMAT, **fields), separators=(',', ':'), sort_keys=True,
                      allow_nan=False)


def parse_overhead_passages(text):
    """The fields of one robot's list; ValueError for another format or any bad or missing field."""
    try:
        data = json.loads(text) if isinstance(text, str) else None
    except ValueError:
        data = None
    _require(isinstance(data, dict) and set(data) == _TOP_KEYS and data['format'] == OVERHEAD_PASSAGES_FORMAT,
             f'not a {OVERHEAD_PASSAGES_FORMAT} text')
    return _fields(data['robot_id'], data['map_frame'], data['generated_at_ns'], data['clearance_m'],
                   data['floor_noise_m'], data['lidar_plane_m'], data['items'])
