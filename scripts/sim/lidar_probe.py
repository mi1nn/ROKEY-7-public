#!/usr/bin/env python3
"""Does the simulated lidar see the walls the layout says are there? Offline: Gazebo server only, no robot, no ROS node.

    lidar_probe.py --world config/sim/worlds/arena.sdf --layout config/sim/worlds/arena.layout.json \\
        --at 1.0,0.9 --at 3.0,1.2 --at 1.0,0.25

For every --at (x, y metres, world frame) a static GPU lidar like the TurtleBot4 one (640 rays, 0.164 .. 12 m,
0.193 m above the floor) is put into a copy of the world, the server runs a moment, and each ray is compared with
the distance computed from the layout's wall segments and boxes (plain ray casting). A ray whose measured range is
more than 0.25 m longer than that distance has *passed through* geometry: a leak. Leaks are counted by the true
distance of the ray, so a near-field effect and a thin-wall effect can be told apart. Things the layout puts
overhead (tunnel lintels) or round (the doll) are not in the computed distances.

Measured 2026-10-15 on arena.sdf (walls 0.01 .. 0.10 m thick, 12 probes): leaks are almost all rays whose true
distance is under about 0.25 m. A 0.10 m wall removed little (71 -> 40 leaking rays of 7680). Wall thickness is not
the cause; keeping the robot 0.3 m or more from a wall is what avoids it.

Needs the ROS 2 Jazzy install (it brings the gz tools). Uses the NVIDIA renderer if the driver is there, as run_sim.sh
does. Stops only the server it started.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

LIDAR_Z = 0.193         # lidar height above the floor: base_link -> rplidar_link, TurtleBot4 URDF [측정 2026-10-15, tf2_echo]
RAYS, R_MIN, R_MAX = 640, 0.164, 12.0    # turtlebot4 rplidar xacro: h_samples, r_min, r_max
LEAK_M = 0.25           # a measurement this much longer than the computed distance passed through something
BUCKETS = ((0.0, 0.25), (0.25, 0.35), (0.35, 0.5), (0.5, math.inf))   # by the true distance of the ray, m


def wall_segments(layout):
    """Line segments of the layout objects a lidar at 0.19 m can hit: walls and boxes (not overhead, not round)."""
    segments = []
    for obj in layout['objects']:
        if obj['kind'] == 'wall':
            segments.append(((obj['x1'], obj['y1']), (obj['x2'], obj['y2'])))
        elif obj['kind'] == 'box':
            sx, sy = obj['size'][0], obj['size'][1]
            c, s = math.cos(obj['yaw']), math.sin(obj['yaw'])
            corners = [(obj['x'] + c * dx - s * dy, obj['y'] + s * dx + c * dy)
                       for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
            segments += [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    return segments


def cast(segments, x, y, angle):
    """Distance from (x, y) along angle to the nearest segment; inf if none."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def ray_angle(k):
    return -math.pi + k * (2 * math.pi / RAYS)


def count_leaks(segments, x, y, ranges):
    """[(true_distance, leaked, hit_x, hit_y, measured)] for every ray with a computed distance inside the range."""
    out = []
    for k, measured in enumerate(ranges):
        angle = ray_angle(k)
        true = cast(segments, x, y, angle)
        if R_MIN <= true < R_MAX - 0.5:
            out.append((true, measured > true + LEAK_M, x + true * math.cos(angle), y + true * math.sin(angle), measured))
    return out


def probe_model(i, x, y, sensor_min, visualize=False, name='probe'):
    return (f'    <model name="{name}{i}"><static>true</static><pose>{x} {y} {LIDAR_Z} 0 0 0</pose><link name="link">'
            f'<sensor name="lidar{i}" type="gpu_lidar"><topic>{name}{i}_scan</topic><update_rate>10</update_rate>'
            f'<always_on>true</always_on><visualize>{str(visualize).lower()}</visualize><lidar><scan><horizontal><samples>{RAYS}</samples><resolution>1</resolution>'
            f'<min_angle>{-math.pi}</min_angle><max_angle>{math.pi}</max_angle></horizontal>'
            f'<vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle>'
            f'</vertical></scan><range><min>{sensor_min}</min><max>{R_MAX}</max><resolution>0.01</resolution></range>'
            f'</lidar></sensor></link></model>\n')


DROP_PLUGINS = re.compile(r"<plugin[^>]*(?:gz-sim-sensors-system|gz_ros2_control)[^>]*?(?:/>|>.*?</plugin>)", re.DOTALL)


def extra_model(sdf_text, name, x, y, yaw):
    """The <model> of an SDF file (from `gz sdf -p`), renamed, placed and made static. Its own Sensors system and
    ros2_control plugin are dropped: the probe world has one Sensors system, as a second robot made from a
    stripped description has; its sensors (lidar, cameras) are still in the scene and still rendered."""
    match = re.search(r"<model name=['\"][^'\"]*['\"]>(.*)</model>", sdf_text, re.DOTALL)
    if not match:
        raise ValueError('no <model> in the extra SDF')
    body = DROP_PLUGINS.sub('', match.group(1))
    body = re.sub(r"^\s*<pose[^>]*>[^<]*</pose>", '', body, count=1)
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>{body}</model>\n')


def probe_world(sdf_text, points, sensor_min=R_MIN, shown=(), extras=()):
    """The world with a sensors system (as the Create3 description loads it) and one static lidar per point."""
    match = re.search(r'<world name="([^"]+)">', sdf_text)
    if not match or '</world>' not in sdf_text:
        raise ValueError('not a world sdf')
    sensors = ('\n    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
               '<render_engine>ogre</render_engine></plugin>')
    text = sdf_text.replace(match.group(0), match.group(0) + sensors, 1)
    models = ''.join(probe_model(i, x, y, sensor_min) for i, (x, y) in enumerate(points))
    # lidars that only draw their rays (<visualize>), like the TurtleBot4 rplidar of another robot; not counted
    models += ''.join(probe_model(i, x, y, sensor_min, visualize=True, name='shown') for i, (x, y) in enumerate(shown))
    models += ''.join(extras)
    return text.replace('</world>', models + '  </world>')


def read_ranges(text):
    values = [float(v) if v not in ('inf', '-inf', 'nan') else math.inf for v in re.findall(r'ranges: ([-\w.+]+)', text)]
    return values if len(values) == RAYS else None


def gz_env(partition):
    env = {'PATH': '/usr/bin:/bin', 'HOME': os.environ.get('HOME', '/tmp'), 'GZ_PARTITION': partition,
           'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share'}   # model:// meshes of the TurtleBot4 and dock packages
    for key in ('DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'WAYLAND_DISPLAY'):
        env[key] = os.environ.get(key, '')
    if Path('/dev/nvidiactl').exists() and Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json').exists():
        env.update({'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                    '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'})
    return env


def run_probes(world_text, points, settle_s):
    partition = f'probe_{os.getpid()}'
    env = gz_env(partition)
    shell = lambda cmd: ['bash', '-c', 'source /opt/ros/jazzy/setup.bash; ' + cmd]   # noqa: E731
    with tempfile.TemporaryDirectory() as tmp:
        (Path(tmp) / 'probe_world.sdf').write_text(world_text)
        server = subprocess.Popen(shell(f'cd {tmp} && exec gz sim -s -r -v 1 probe_world.sdf'), env=env,
                                  start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(settle_s)
            scans = []
            for i in range(len(points)):
                ranges = None
                for _ in range(3):   # a busy PC sometimes lets one read time out
                    done = subprocess.run(shell(f'timeout 20 gz topic -e -t /probe{i}_scan -n 1'), env=env,
                                          capture_output=True, text=True)
                    ranges = read_ranges(done.stdout)
                    if ranges is not None:
                        break
                scans.append(ranges)
        finally:
            os.killpg(server.pid, signal.SIGINT)   # the server this script started, nothing else
            time.sleep(3)
            try:
                os.killpg(server.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return scans


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', type=Path, required=True, help='world .sdf')
    parser.add_argument('--layout', type=Path, required=True, help='layout .json the world was made from')
    parser.add_argument('--at', action='append', required=True, metavar='X,Y', help='probe place in the world frame, m')
    parser.add_argument('--sensor-min', type=float, default=R_MIN, metavar='M',
                        help='the Gazebo lidar <range><min> to simulate (default 0.164, the TurtleBot4 description); '
                             'leaks are still counted for rays from 0.164 m on')
    parser.add_argument('--shown-at', action='append', default=[], metavar='X,Y',
                        help='another lidar there that draws its rays (<visualize>true</visualize>, as the TurtleBot4 '
                             'rplidar does); it is not measured, it is only in the scene')
    parser.add_argument('--model', action='append', default=[], metavar='FILE.sdf@X,Y,YAW',
                        help='put this SDF model (e.g. the TurtleBot4 or its dock from `gz sdf -p`) into the scene, '
                             'static, without its own Sensors system')
    parser.add_argument('--detail', action='store_true', help='list every leaking ray with the place it should have hit')
    parser.add_argument('--settle', type=float, default=25.0, help='seconds to let the server start')
    args = parser.parse_args(argv)
    try:
        points = [tuple(float(v) for v in p.split(',')) for p in args.at]
        shown = [tuple(float(v) for v in p.split(',')) for p in args.shown_at]
        if any(len(p) != 2 for p in points + shown):
            raise ValueError('--at and --shown-at want X,Y')
        extras = []
        for i, spec in enumerate(args.model):
            path, _, pose = spec.rpartition('@')
            mx, my, myaw = (float(v) for v in pose.split(','))
            extras.append(extra_model(Path(path).read_text(), f'extra{i}', mx, my, myaw))
        segments = wall_segments(json.loads(args.layout.read_text()))
        world_text = probe_world(args.world.read_text(), points, args.sensor_min, shown, extras)
    except (ValueError, OSError, KeyError, json.JSONDecodeError) as exc:
        print(f'lidar_probe: {exc}', file=sys.stderr)
        return 2
    scans = run_probes(world_text, points, args.settle)
    totals = [[0, 0] for _ in BUCKETS]
    missing = 0
    for (x, y), ranges in zip(points, scans):
        if ranges is None:
            print(f'({x:g}, {y:g}): no scan received', file=sys.stderr)
            missing += 1
            continue
        rays = count_leaks(segments, x, y, ranges)
        print(f'({x:g}, {y:g}): {sum(r[1] for r in rays)} leaking of {len(rays)} rays with a wall in range')
        if args.detail:
            for true, leak, hx, hy, measured in rays:
                if leak:
                    print(f'    should hit ({hx:.2f}, {hy:.2f}) at {true:.2f} m, measured {measured:.2f} m')
        for true, leak, _, _, _ in rays:
            index = next(i for i, (lo, hi) in enumerate(BUCKETS) if lo <= true < hi)
            totals[index][0] += 1
            totals[index][1] += leak
    print('leaks by the true distance of the ray:')
    for (lo, hi), (rays, leaks) in zip(BUCKETS, totals):
        label = f'{lo:.2f} - {hi:.2f} m' if math.isfinite(hi) else f'over {lo:.2f} m'
        print(f'  {label:>14}: {leaks} of {rays}' + (f' ({leaks / rays:.1%})' if rays else ''))
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
