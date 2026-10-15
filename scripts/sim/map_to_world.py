#!/usr/bin/env python3
"""A Gazebo (gz Harmonic) world for the TurtleBot4 simulator from a SLAM map and a layout list.

Simulation track stage 1 (재범 decision 2026-10-15 evening): the practice field rebuilt so a virtual TB-6 can
map it with SLAM. The map is only the input of this tool. The simulation never gets it: no map topic, no
map_server, and run_sim.sh takes no map path. The robots know the world only through their own sensors.

    map_to_world.py --map <map.yaml> --layout <layout.json> --name <world> --out-dir config/sim/worlds

- Walls: the occupied cells of the map (trinary, occupied_thresh as map_server reads it), minus connected
  groups smaller than layout ``min_cells`` (SLAM noise), merged into rectangles and stood up as boxes of
  ``wall.height_m``. Box footprints are the cells themselves, so the world matches the map to one cell.
- World frame = map frame: the map yaml origin (x, y, yaw) is kept, so a SLAM map made in the simulation from
  the TB-6 start pose lies on the real map without a transform. The real field looks turned about 10 deg in
  this frame because the map frame is TB-6's start pose.
- Things the map does not show (white boxes, dolls, the exit QR board, the overhead passage, measured walls
  the robot never saw) come from the layout ``objects``. A ``wall`` object is a segment of the layout wall
  height and thickness (MDF board). Every value is required; nothing has a default. A value not measured yet carries
  ``"estimate": true`` and is listed as [추정] when the world is written, so a measurement replaces just it.
- Robot starts (``robots``) go to <name>.spawn.json for run_sim.sh; the TurtleBot4 spawn launch puts each
  robot's dock next to it.
With ``--layout-only`` there is no map at all: walls and things come only from the layout (a floor plan carried
into metres, e.g. arena.layout.json). Writes <out-dir>/<name>.sdf (world name = file name, as turtlebot4_gz_bringup world:=<name> needs) and
<out-dir>/<name>.spawn.json. Standard library only.
"""
import argparse
import re
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from xml.sax.saxutils import escape


class WorldError(ValueError):
    """The input is not what this tool expects; nothing is guessed."""


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple            # (x, y, yaw) of the lower-left cell corner, map frame
    occupied: frozenset      # (column, row), row 0 = bottom (map y grows up)


# ---- map ----------------------------------------------------------------------------------------------------

MAP_KEYS = {'image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'}


def _map_yaml(path):
    """The flat key: value map yaml map_server writes (no PyYAML needed)."""
    values = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        key, sep, value = line.partition(':')
        if not sep:
            raise WorldError(f'{path}: not a key: value line: {line!r}')
        values[key.strip()] = value.strip()
    missing = sorted(MAP_KEYS - set(values))
    if missing:
        raise WorldError(f'{path}: missing {missing}')
    if values['mode'] != 'trinary':
        raise WorldError(f'{path}: mode {values["mode"]!r}: only trinary maps are read')
    try:
        origin = tuple(float(v) for v in values['origin'].strip('[]').split(','))
        out = dict(image=values['image'], resolution=float(values['resolution']), origin=origin,
                   negate=int(values['negate']), occupied_thresh=float(values['occupied_thresh']))
    except ValueError as exc:
        raise WorldError(f'{path}: {exc}') from None
    if len(origin) != 3 or out['resolution'] <= 0 or out['negate'] not in (0, 1):
        raise WorldError(f'{path}: origin must be [x, y, yaw], resolution > 0, negate 0 or 1')
    return out


def _pgm(path):
    """(width, height, maxval, values top row first) of a P5 or P2 pgm."""
    data = path.read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    kind = tokens[0]
    try:
        width, height, maxval = (int(t) for t in tokens[1:])
    except ValueError:
        raise WorldError(f'{path}: bad pgm header') from None
    if kind not in (b'P5', b'P2') or not 0 < maxval <= 255:
        raise WorldError(f'{path}: only 8-bit P5/P2 pgm is read')
    if kind == b'P5':
        body = data[i + 1:i + 1 + width * height]
        values = list(body)
    else:
        values = [int(t) for t in data[i:].split()]
    if len(values) != width * height:
        raise WorldError(f'{path}: {len(values)} pixels for {width}x{height}')
    return width, height, maxval, values


def read_map(yaml_path):
    yaml_path = Path(yaml_path)
    meta = _map_yaml(yaml_path)
    width, height, maxval, values = _pgm(yaml_path.parent / meta['image'])
    occupied = set()
    for index, value in enumerate(values):
        # map_server: p = (maxval - value) / maxval, or value / maxval when negate
        p = value / maxval if meta['negate'] else (maxval - value) / maxval
        if p > meta['occupied_thresh']:
            row_top, column = divmod(index, width)
            occupied.add((column, height - 1 - row_top))
    return Grid(width, height, meta['resolution'], meta['origin'], frozenset(occupied))


def drop_small(cells, *, min_cells):
    """Cells of 8-connected groups with at least min_cells cells."""
    left, keep = set(cells), set()
    while left:
        group, todo = set(), [left.pop()]
        while todo:
            c, r = todo.pop()
            group.add((c, r))
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    n = (c + dc, r + dr)
                    if n in left:
                        left.remove(n)
                        todo.append(n)
        if len(group) >= min_cells:
            keep |= group
    return keep


def rectangles(cells):
    """(column, row, width, height) rectangles covering exactly the cells: runs per row, stacked while the
    same run continues in the next row."""
    runs_by_row = {}
    for c, r in sorted(cells, key=lambda cell: (cell[1], cell[0])):
        runs = runs_by_row.setdefault(r, [])
        if runs and runs[-1][1] == c:
            runs[-1][1] = c + 1
        else:
            runs.append([c, c + 1])
    open_rects, done = {}, []
    for r in sorted(runs_by_row):
        current = {}
        for c0, c1 in runs_by_row[r]:
            rect = open_rects.pop((c0, c1), None)
            if rect is not None and rect[1] + rect[3] == r:
                rect[3] += 1
            else:
                if rect is not None:
                    done.append(rect)
                rect = [c0, r, c1 - c0, 1]
            current[(c0, c1)] = rect
        done += open_rects.values()
        open_rects = current
    done += open_rects.values()
    return [tuple(rect) for rect in done]


def cell_box(rect, grid):
    """(x, y, yaw, size_x, size_y) of a rectangle in the map frame."""
    c, r, w, h = rect
    res = grid.resolution
    local_x, local_y = (c + w / 2) * res, (r + h / 2) * res
    ox, oy, yaw = grid.origin
    x = ox + local_x * math.cos(yaw) - local_y * math.sin(yaw)
    y = oy + local_x * math.sin(yaw) + local_y * math.cos(yaw)
    return x, y, yaw, w * res, h * res


# ---- layout -------------------------------------------------------------------------------------------------

LAYOUT_KEYS = {'wall', 'floor', 'min_cells', 'robots', 'objects', 'note'}
LAYOUT_OPTIONAL = {'wall_feet'}   # brackets the MDF panels stand on, at every panel joint of each layout wall
OBJECT_KEYS = {
    'box': {'name', 'kind', 'x', 'y', 'yaw', 'size', 'color'},
    'cylinder': {'name', 'kind', 'x', 'y', 'radius', 'length', 'color'},
    'overhead': {'name', 'kind', 'x', 'y', 'yaw', 'size', 'clearance_m', 'color'},
    'wall': {'name', 'kind', 'x1', 'y1', 'x2', 'y2'},   # a measured wall the map did not see; layout wall size
}
OPTIONAL = {'estimate', 'source', 'texture', 'note'}


def _number(value, what, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise WorldError(f'{what} must be a {"positive " if positive else ""}number')
    return float(value)


def _color(value, what):
    if not isinstance(value, list) or len(value) != 3:
        raise WorldError(f'{what} must be [r, g, b] in 0..1')
    out = [_number(v, what) for v in value]
    if not all(0 <= v <= 1 for v in out):
        raise WorldError(f'{what} must be [r, g, b] in 0..1')
    return out


def _exact(raw, need, what):
    if not isinstance(raw, dict):
        raise WorldError(f'{what} must be an object')
    missing, unknown = sorted(need - set(raw)), sorted(set(raw) - need - OPTIONAL)
    if missing or unknown:
        raise WorldError(f'{what}: missing {missing}, unknown {unknown}')


def load_layout(path):
    raw = json.loads(Path(path).read_text())
    _exact({k: v for k, v in raw.items() if k not in LAYOUT_OPTIONAL}, LAYOUT_KEYS - {'note'}, 'layout')
    if 'wall_feet' in raw:
        feet = raw['wall_feet']
        _exact(feet, {'spacing_m', 'size', 'color'}, 'wall_feet')
        _number(feet['spacing_m'], 'wall_feet.spacing_m', positive=True)
        if not isinstance(feet['size'], list) or len(feet['size']) != 3:
            raise WorldError('wall_feet.size must be [along the wall, out from it, height]')
        for v in feet['size']:
            _number(v, 'wall_feet.size', positive=True)
        _color(feet['color'], 'wall_feet.color')
    _exact(raw['wall'], {'height_m', 'thickness_m', 'color'}, 'wall')
    _number(raw['wall']['height_m'], 'wall.height_m', positive=True)
    _number(raw['wall']['thickness_m'], 'wall.thickness_m', positive=True)
    _color(raw['wall']['color'], 'wall.color')
    _exact(raw['floor'], {'color'}, 'floor')
    _color(raw['floor']['color'], 'floor.color')
    if type(raw['min_cells']) is not int or raw['min_cells'] < 1:
        raise WorldError('min_cells must be a positive integer')
    if not isinstance(raw['robots'], dict) or not raw['robots']:
        raise WorldError('robots must name at least one robot start')
    for name, start in raw['robots'].items():
        _exact(start, {'x', 'y', 'yaw'}, f'robots.{name}')
        for key in ('x', 'y', 'yaw'):
            _number(start[key], f'robots.{name}.{key}')
    names = set()
    for i, obj in enumerate(raw['objects']):
        kind = obj.get('kind') if isinstance(obj, dict) else None
        if kind not in OBJECT_KEYS:
            raise WorldError(f'objects[{i}].kind must be one of {sorted(OBJECT_KEYS)}')
        _exact(obj, OBJECT_KEYS[kind], f'objects[{i}]')
        if obj['name'] in names or not str(obj['name']).isidentifier():
            raise WorldError(f'objects[{i}].name {obj["name"]!r}: a new identifier')
        names.add(obj['name'])
        if 'color' in obj:
            _color(obj['color'], f'{obj["name"]}.color')
        if kind == 'wall' and math.hypot(_number(obj['x2'], 'x2') - _number(obj['x1'], 'x1'),
                                         _number(obj['y2'], 'y2') - _number(obj['y1'], 'y1')) <= 0:
            raise WorldError(f'{obj["name"]}: a wall needs two different ends')
        for key in ('x', 'y', 'yaw', 'clearance_m'):
            if key in obj:
                _number(obj[key], f'{obj["name"]}.{key}')
        for key in ('radius', 'length'):
            if key in obj:
                _number(obj[key], f'{obj["name"]}.{key}', positive=True)
        if 'size' in obj and (not isinstance(obj['size'], list) or len(obj['size']) != 3):
            raise WorldError(f'{obj["name"]}.size must be [x, y, z]')
        for v in obj.get('size', []):
            _number(v, f'{obj["name"]}.size', positive=True)
    return raw


# ---- world --------------------------------------------------------------------------------------------------

HEADER = """<?xml version="1.0"?>
<!-- Written by scripts/sim/map_to_world.py from {source}. Do not edit by hand: change the layout and run it again.
     Not measured yet [추정]: {estimates} -->
<sdf version="1.8">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
"""
# Plugins, light and scene as turtlebot4_gz_bringup worlds/maze.sdf (turtlebot4_simulator jazzy b7d0f3b).


def _material(color, texture=None):
    rgb = ' '.join(f'{v:g}' for v in color)
    out = f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
    if texture:
        out += f'<pbr><metal><albedo_map>{escape(texture)}</albedo_map></metal></pbr>'
    return out + '</material>'


def _model(name, pose, geometry, color, texture=None):
    x, y, z, yaw = pose
    return (f'    <model name="{name}">\n      <static>true</static>\n'
            f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>\n'
            f'      <link name="link">\n'
            f'        <collision name="collision"><geometry>{geometry}</geometry></collision>\n'
            f'        <visual name="visual"><geometry>{geometry}</geometry>{_material(color, texture)}</visual>\n'
            f'      </link>\n    </model>\n')


def _box(size):
    return '<box><size>' + ' '.join(f'{v:.4f}' for v in size) + '</size></box>'


def build(grid, layout, *, name, source='a SLAM map'):
    """(sdf text, spawn dict, names of the estimated values)."""
    if not name.isidentifier():
        raise WorldError(f'world name {name!r} must be an identifier (it is also the file name)')
    estimates = (['wall'] if layout['wall'].get('estimate') else []) + \
        (['wall_feet'] if layout.get('wall_feet', {}).get('estimate') else []) + \
        [f'robots.{robot}' for robot, start in layout['robots'].items() if start.get('estimate')] + \
        [obj['name'] for obj in layout['objects'] if obj.get('estimate')]
    parts = [HEADER.format(source=escape(source), estimates=', '.join(estimates) or '-', name=name)]
    floor = '<plane><normal>0 0 1</normal><size>100 100</size></plane>'
    parts.append(_model('ground_plane', (0, 0, 0, 0), floor, layout['floor']['color']))
    wall = layout['wall']
    for i, rect in enumerate(sorted(rectangles(drop_small(grid.occupied, min_cells=layout['min_cells'])))):
        x, y, yaw, sx, sy = cell_box(rect, grid)
        parts.append(_model(f'wall_{i:04d}', (x, y, wall['height_m'] / 2, yaw),
                            _box((sx, sy, wall['height_m'])), wall['color']))
    for obj in layout['objects']:
        if obj['kind'] == 'wall':
            length = math.hypot(obj['x2'] - obj['x1'], obj['y2'] - obj['y1'])
            pose = ((obj['x1'] + obj['x2']) / 2, (obj['y1'] + obj['y2']) / 2, wall['height_m'] / 2,
                    math.atan2(obj['y2'] - obj['y1'], obj['x2'] - obj['x1']))
            # Both ends reach half a wall thickness past the end point, so two walls that meet at a corner (or a wall
            # that ends on another wall's centre line) overlap there: no notch the lidar could see through.
            parts.append(_model(obj['name'], pose, _box((length + wall['thickness_m'], wall['thickness_m'],
                                                         wall['height_m'])), wall['color']))
            parts.extend(_feet(obj, layout))
            continue
        if obj['kind'] == 'cylinder':
            geometry = f'<cylinder><radius>{obj["radius"]:.4f}</radius><length>{obj["length"]:.4f}</length></cylinder>'
            pose = (obj['x'], obj['y'], obj['length'] / 2, 0.0)
        else:
            sx, sy, sz = obj['size']
            z = obj['clearance_m'] + sz / 2 if obj['kind'] == 'overhead' else sz / 2
            geometry, pose = _box((sx, sy, sz)), (obj['x'], obj['y'], z, obj['yaw'])
        parts.append(_model(obj['name'], pose, geometry, obj['color'], obj.get('texture')))
    parts.append('  </world>\n</sdf>\n')
    return ''.join(parts), spawn_of(layout, name), estimates


def _feet(wall_obj, layout):
    """Models of the brackets one layout wall stands on: a pair (one each side) at every joint of panels of
    wall_feet.spacing_m along it (a wall shorter than one panel: one pair in the middle). None without wall_feet."""
    feet = layout.get('wall_feet')
    if not feet:
        return []
    (x1, y1, x2, y2), half = (wall_obj[k] for k in ('x1', 'y1', 'x2', 'y2')), layout['wall']['thickness_m'] / 2
    length, yaw = math.hypot(x2 - x1, y2 - y1), math.atan2(y2 - y1, x2 - x1)
    along, out, height = feet['size']
    panels = max(1, round(length / feet['spacing_m']))
    joints = [i / panels for i in range(1, panels)] or [0.5]
    nx, ny = -math.sin(yaw), math.cos(yaw)   # left of the wall direction
    models = []
    for i, f in enumerate(joints, 1):
        cx, cy = x1 + f * (x2 - x1), y1 + f * (y2 - y1)
        for side, sign in (('l', 1), ('r', -1)):
            off = sign * (half + out / 2)
            models.append(_model(f'{wall_obj["name"]}_foot{i}_{side}', (cx + off * nx, cy + off * ny, height / 2, yaw),
                                 _box((along, out, height)), feet['color']))
    return models


def spawn_of(layout, name):
    """What run_sim.sh and run_full.sh read: robot starts, and the doll's place when the layout has one."""
    spawn = {'world': name, 'robots': {robot: {k: float(v) for k, v in start.items() if k in ('x', 'y', 'yaw')}
                                       for robot, start in layout['robots'].items()}}
    doll = [obj for obj in layout['objects'] if obj['name'] == 'doll']
    if doll:
        spawn['doll'] = [float(doll[0]['x']), float(doll[0]['y'])]
    return spawn


def build_on_base(base_text, layout, *, name, source):
    """(sdf text, spawn dict, estimates): an existing world (e.g. the TurtleBot4 maze) renamed to name, with the
    layout objects added before </world>. The base keeps its own floor, walls and plugins; the layout wall
    settings are used only for wall-kind objects."""
    if not name.isidentifier():
        raise WorldError(f'world name {name!r} must be an identifier (it is also the file name)')
    match = re.search(r"<world name=(['\"])[^'\"]*\1>", base_text)
    if not match or base_text.count('</world>') != 1:
        raise WorldError('the base is not a single-world SDF')
    objects = build(Grid(0, 0, 0.05, (0.0, 0.0, 0.0), frozenset()), layout, name=name, source=source)[0]
    models = objects[objects.index('<model name="ground_plane">'):objects.rindex('  </world>')]
    models = models[models.index('</model>\n') + len('</model>\n'):]   # the base has its own floor
    estimates = [obj['name'] for obj in layout['objects'] if obj.get('estimate')]
    head = (f'<!-- Objects added by scripts/sim/map_to_world.py (base mode) from {escape(source)}. Do not edit by hand.\n'
            f'     Not measured yet [추정]: {", ".join(estimates) or "-"} -->\n')
    text = base_text.replace(match.group(0), f"<world name='{name}'>", 1)
    text = text.replace('</world>', head + models + '  </world>', 1)
    return text, spawn_of(layout, name), estimates


def main(argv=None):
    parser = argparse.ArgumentParser(description='SLAM 지도와 배치 목록으로 Gazebo world 를 만든다')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--map', type=Path, help='지도 yaml (map_server 형식, trinary)')
    source.add_argument('--base', type=Path, help='기존 world SDF(예: TurtleBot4 미로)에 배치 목록의 물체만 더한다')
    source.add_argument('--layout-only', action='store_true',
                        help='지도 없이 배치 목록(벽 선분·물체)만으로 만든다(평면도에서 옮긴 배치용)')
    parser.add_argument('--layout', type=Path, required=True, help='배치 목록 JSON (벽 높이, 물체, 로봇 출발 자리)')
    parser.add_argument('--wall-thickness', type=float, metavar='M',
                        help='벽 두께(m)를 배치 목록의 wall.thickness_m 대신 쓴다 [추정]. 실측(MDF 두께)이 오면 이 값 하나만 바꾼다')
    parser.add_argument('--name', required=True, help='world 이름 = 파일 이름 (turtlebot4 world:=<name>)')
    parser.add_argument('--out-dir', type=Path, required=True, help='쓸 폴더 (config/sim/worlds)')
    args = parser.parse_args(argv)
    try:
        layout = load_layout(args.layout)
        if args.wall_thickness is not None:
            layout['wall']['thickness_m'] = _number(args.wall_thickness, '--wall-thickness', positive=True)
            layout['wall']['estimate'] = True   # a value given on the command line is not a measurement unless the layout says so
        grid = read_map(args.map) if args.map else Grid(0, 0, 0.05, (0.0, 0.0, 0.0), frozenset())
        if args.base:
            world, spawn, estimates = build_on_base(args.base.read_text(), layout, name=args.name,
                                                    source=f'{args.base.name} + {args.layout.name}')
        else:
            world, spawn, estimates = build(grid, layout, name=args.name,
                                            source=args.map.name if args.map else f'{args.layout.name} (no map)')
    except (WorldError, OSError, json.JSONDecodeError) as exc:
        print(f'map_to_world: {exc}', file=sys.stderr)
        return 2
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / f'{args.name}.sdf').write_text(world)
    (args.out_dir / f'{args.name}.spawn.json').write_text(json.dumps(spawn, indent=2) + '\n')
    walls = world.count('<model name="wall_')
    print(f'{args.name}: walls {walls} boxes from {len(grid.occupied)} occupied cells, '
          f'objects {len(layout["objects"])}, robots {sorted(spawn["robots"])}')
    print('[추정] ' + (', '.join(estimates) or '없음'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
