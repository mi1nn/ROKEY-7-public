#!/usr/bin/env python3
"""How much a simulated SLAM map overlaps the real SLAM map (simulation track stage 1, 2026-10-15). No ROS.

    compare_maps.py --sim <sim_map.yaml> --real <real_map.yaml> [--tol 0.10] [--png overlay.png]

Both maps are map_server files (trinary pgm + yaml). Only the occupied cells (walls) are compared; unknown cells
carry nothing. Every SLAM map lives in the frame of the pose where its SLAM started, and the real and the
simulated starts are different poses, so the simulated walls are first moved by the rigid transform
(rotation + shift) that matches the real walls best, then counted. The transform is printed: a turn or shift
far from what the start poses allow is a warning, not a score.

Counts, with a tolerance --tol (m) around the other map's occupied cells:
  precision = share of simulated wall cells that lie near a real wall cell
  recall    = share of real wall cells that lie near a simulated wall cell
  overlap   = (matched simulated + matched real cells) / (all simulated + all real wall cells)   [계산]
A low recall only means the simulation did not map (drive past) that part of the field; a low precision
means the simulated field has walls the real map does not show. Both are read together with the picture.
Standard library only. The picture is a PNG: blue = real only, orange = simulated only, green = both.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import sys
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_to_world import _pgm   # noqa: E402  (the same pgm reader the world builder uses)


class CompareError(ValueError):
    """The input is not a map this tool reads; nothing is guessed."""


KEYS = {'image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh'}


def read_occupied(yaml_path):
    """(resolution, [(x, y)] centres of the occupied cells in the map frame, known cell count)."""
    yaml_path = Path(yaml_path)
    values = {}
    for line in yaml_path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            key, sep, value = line.partition(':')
            if not sep:
                raise CompareError(f'{yaml_path}: not a key: value line: {line!r}')
            values[key.strip()] = value.strip()
    missing = sorted(KEYS - set(values))
    if missing:
        raise CompareError(f'{yaml_path}: missing {missing}')
    if values['mode'] != 'trinary':
        raise CompareError(f'{yaml_path}: only trinary maps are read')
    try:
        res = float(values['resolution'])
        ox, oy, oyaw = (float(v) for v in values['origin'].strip('[]').split(','))
        negate, occupied_thresh = int(values['negate']), float(values['occupied_thresh'])
    except ValueError as exc:
        raise CompareError(f'{yaml_path}: {exc}') from None
    width, height, maxval, pixels = _pgm(yaml_path.parent / values['image'])
    free_thresh = float(values.get('free_thresh', 0.25))
    points, known = [], 0
    for index, value in enumerate(pixels):
        p = value / maxval if negate else (maxval - value) / maxval
        if p > occupied_thresh:
            known += 1
            row_top, column = divmod(index, width)
            lx, ly = (column + 0.5) * res, (height - 1 - row_top + 0.5) * res
            points.append((ox + lx * math.cos(oyaw) - ly * math.sin(oyaw),
                           oy + lx * math.sin(oyaw) + ly * math.cos(oyaw)))
        elif p < free_thresh:
            known += 1
    return res, points, known


def transform(points, theta, dx, dy):
    c, s = math.cos(theta), math.sin(theta)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in points]


def occupancy_set(points, cell):
    return {(math.floor(x / cell), math.floor(y / cell)) for x, y in points}


def near(points, target_cells, cell, tol):
    """How many points lie within about tol of a cell in target_cells (a square window, as the cells are square)."""
    reach = max(0, math.ceil(tol / cell - 0.5))
    count = 0
    for x, y in points:
        cx, cy = math.floor(x / cell), math.floor(y / cell)
        if any((cx + i, cy + j) in target_cells for i in range(-reach, reach + 1) for j in range(-reach, reach + 1)):
            count += 1
    return count


def score(sim_points, real_points, real_cells, sim_cells_fn, cell, tol):
    moved_cells = sim_cells_fn
    return near(sim_points, real_cells, cell, tol) + near(real_points, moved_cells, cell, tol)


def downsample(points, step):
    return sorted(occupancy_set(points, step)), step


def align(sim_points, real_points, cell, tol):
    """(theta, dx, dy) of the rigid move of the simulated points that matches the real ones best.
    Coarse: for each turn, every simulated/real point pair votes for the shift that would put them together.
    Fine: the best few are polished by a small search that counts matched cells, then by icp()."""
    coarse_step = 0.10
    s_cells, _ = downsample(sim_points, coarse_step)
    r_cells, _ = downsample(real_points, coarse_step)
    s_pts = [((i + 0.5) * coarse_step, (j + 0.5) * coarse_step) for i, j in s_cells]
    r_pts = [((i + 0.5) * coarse_step, (j + 0.5) * coarse_step) for i, j in r_cells]
    candidates = []
    for deg in range(0, 360, 3):
        theta = math.radians(deg)
        moved = transform(s_pts, theta, 0.0, 0.0)
        votes = Counter()
        for sx, sy in moved:
            for rx, ry in r_pts:
                votes[(round((rx - sx) / coarse_step), round((ry - sy) / coarse_step))] += 1
        # a shift and its neighbours count together: the turn step is coarse, so the walls smear a little
        best = max(votes, key=lambda k: sum(votes.get((k[0] + i, k[1] + j), 0) for i in (-1, 0, 1) for j in (-1, 0, 1)))
        total = sum(votes.get((best[0] + i, best[1] + j), 0) for i in (-1, 0, 1) for j in (-1, 0, 1))
        candidates.append((total, theta, best[0] * coarse_step, best[1] * coarse_step))
    candidates.sort(reverse=True)
    real_cells = occupancy_set(real_points, cell)
    best = None
    for _, theta0, dx0, dy0 in candidates[:6]:
        cur = (theta0, dx0, dy0)
        cur_score = -1
        for span, tstep, sstep in ((math.radians(3), math.radians(1), 0.10), (math.radians(1), math.radians(0.25), 0.05)):
            top = cur
            for ti in range(-3, 4):
                for xi in range(-3, 4):
                    for yi in range(-3, 4):
                        trial = (cur[0] + ti * span / 3, cur[1] + xi * sstep, cur[2] + yi * sstep)
                        moved = transform(sim_points, *trial)
                        sc = near(moved, real_cells, cell, tol) + near(real_points, occupancy_set(moved, cell), cell, tol)
                        if sc > cur_score:
                            cur_score, top = sc, trial
            cur = top
        if best is None or cur_score > best[0]:
            best = (cur_score, cur)
    return icp(sim_points, real_points, best[1], tol)


def icp(sim_points, real_points, start, tol, rounds=100):
    """Polish a rigid move: pair each moved simulated point with the nearest real point within 2 x tol, move by the
    best rigid fit of those pairs (2D closed form), repeat. The search above only counts matches inside the
    tolerance, so many moves score the same; this settles on the one that fits the walls themselves."""
    reach = 2 * tol
    buckets = {}
    for x, y in real_points:
        buckets.setdefault((math.floor(x / reach), math.floor(y / reach)), []).append((x, y))
    theta, dx, dy = start
    for _ in range(rounds):
        pairs = []
        for px, py in transform(sim_points, theta, dx, dy):
            cx, cy = math.floor(px / reach), math.floor(py / reach)
            best, best_d = None, reach
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    for qx, qy in buckets.get((cx + i, cy + j), ()):
                        d = math.hypot(qx - px, qy - py)
                        if d < best_d:
                            best, best_d = (qx, qy), d
            if best:
                pairs.append(((px, py), best))
        if len(pairs) < 3:
            break
        mpx = sum(p[0][0] for p in pairs) / len(pairs)
        mpy = sum(p[0][1] for p in pairs) / len(pairs)
        mqx = sum(p[1][0] for p in pairs) / len(pairs)
        mqy = sum(p[1][1] for p in pairs) / len(pairs)
        sin = sum((p[0][0] - mpx) * (p[1][1] - mqy) - (p[0][1] - mpy) * (p[1][0] - mqx) for p in pairs)
        cos = sum((p[0][0] - mpx) * (p[1][0] - mqx) + (p[0][1] - mpy) * (p[1][1] - mqy) for p in pairs)
        dtheta = math.atan2(sin, cos)
        c, s_ = math.cos(dtheta), math.sin(dtheta)
        tx, ty = mqx - (c * mpx - s_ * mpy), mqy - (s_ * mpx + c * mpy)
        theta, dx, dy = theta + dtheta, c * dx - s_ * dy + tx, s_ * dx + c * dy + ty
        if abs(dtheta) < 1e-5 and math.hypot(tx, ty) < 1e-4:
            break
    return theta, dx, dy


def measure(sim_points, real_points, cell, tol):
    real_cells, sim_cells = occupancy_set(real_points, cell), occupancy_set(sim_points, cell)
    matched_sim, matched_real = near(sim_points, real_cells, cell, tol), near(real_points, sim_cells, cell, tol)
    total = len(sim_points) + len(real_points)
    return {'sim_cells': len(sim_points), 'real_cells': len(real_points),
            'precision': matched_sim / len(sim_points) if sim_points else 0.0,
            'recall': matched_real / len(real_points) if real_points else 0.0,
            'overlap': (matched_sim + matched_real) / total if total else 0.0}


def write_png(path, width, height, rows):
    """rows: height lists of (r, g, b). Plain PNG, no libraries."""
    raw = b''.join(b'\x00' + bytes(v for px in row for v in px) for row in rows)

    def chunk(kind, data):
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xffffffff)

    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def overlay(path, sim_points, real_points, cell, tol, scale=6):
    cells_s, cells_r = occupancy_set(sim_points, cell), occupancy_set(real_points, cell)
    allc = cells_s | cells_r
    if not allc:
        raise CompareError('both maps have no occupied cell')
    x0, x1 = min(c[0] for c in allc) - 2, max(c[0] for c in allc) + 2
    y0, y1 = min(c[1] for c in allc) - 2, max(c[1] for c in allc) + 2
    rows = []
    for j in range(y1, y0 - 1, -1):
        row = []
        for i in range(x0, x1 + 1):
            s, r = (i, j) in cells_s, (i, j) in cells_r
            row += [(40, 160, 80) if s and r else (240, 140, 30) if s else (40, 90, 220) if r else (255, 255, 255)] * scale
        rows += [row] * scale
    write_png(path, (x1 - x0 + 1) * scale, (y1 - y0 + 1) * scale, rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sim', type=Path, required=True, help='simulated SLAM map yaml')
    parser.add_argument('--real', type=Path, required=True, help='real SLAM map yaml')
    parser.add_argument('--tol', type=float, default=0.10, help='match distance in m (default 0.10 = two cells)')
    parser.add_argument('--no-align', action='store_true', help='compare in the given frames, no rigid move')
    parser.add_argument('--png', type=Path, help='write the overlay picture here')
    parser.add_argument('--json', action='store_true', help='print the result as JSON')
    args = parser.parse_args(argv)
    try:
        sim_res, sim_points, sim_known = read_occupied(args.sim)
        real_res, real_points, real_known = read_occupied(args.real)
        if not sim_points or not real_points:
            raise CompareError('a map has no occupied cell')
        cell = max(sim_res, real_res)
        raw = measure(sim_points, real_points, cell, args.tol)
        theta, dx, dy = (0.0, 0.0, 0.0) if args.no_align else align(sim_points, real_points, cell, args.tol)
        moved = transform(sim_points, theta, dx, dy)
        result = measure(moved, real_points, cell, args.tol)
        if args.png:
            overlay(args.png, moved, real_points, cell, args.tol)
    except (CompareError, OSError) as exc:
        print(f'compare_maps: {exc}', file=sys.stderr)
        return 2
    out = {'tolerance_m': args.tol, 'cell_m': cell, 'known_cells': {'sim': sim_known, 'real': real_known},
           'same_frame': raw, 'aligned': result,
           'transform': {'turn_deg': math.degrees(theta), 'dx_m': dx, 'dy_m': dy}}
    if args.json:
        print(json.dumps(out, indent=2))
    else:
        for label, m in (('같은 좌표(변환 없음)', raw), ('맞춰 본 뒤', result)):
            print(f'{label}: 겹침 {m["overlap"]:.1%}  (정밀도 {m["precision"]:.1%}, 재현율 {m["recall"]:.1%};'
                  f' 벽 칸 시뮬레이션 {m["sim_cells"]}, 실제 {m["real_cells"]}, 허용 {args.tol:.2f} m)')
        print(f'맞춤 변환: 회전 {math.degrees(theta):.1f}°, 이동 ({dx:.2f}, {dy:.2f}) m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
