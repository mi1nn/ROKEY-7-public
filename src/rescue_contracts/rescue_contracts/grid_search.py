"""Frontiers, grid paths and step-aside cells on the OccupancyGrid fields (OPS-154/157, 2026-10-13).

TB-6 searches the whole map (the coordinator sends it to frontiers) and TB-5 leaves by the exit
nearest to the doll along free cells. Pure functions on the same grid fields as
occupancy.footprint_clear (frame, size, resolution, planar origin x/y/yaw, cell values); no ROS.
Coordinates are in the grid frame; the same grid gives the same answer.

A cell is clear when footprint_clear passes at its centre: every cell touching the robot disc is
inside the map and free (0). ``blocked`` circles (x, y, r), e.g. the other robot standing still,
make the cells they touch occupied before that rule. ``mask`` lets a caller reuse one clear_mask
for several calls; it already holds its blocked circles, so giving both is refused.

Paths go 8-way through clear cells, never cutting a corner (a diagonal step needs both side cells
clear), between cell centres. A start that is not clear moves to the nearest clear cell within
the radius (the robot is really there, e.g. beside unknown cells); the move is added to the length
and reported. A goal is never moved. Invalid input raises ValueError.
"""
from collections import namedtuple
import heapq
import math

from .occupancy import _checked

Frontier = namedtuple('Frontier', 'cells centroid goal')   # goal None: no clear cell within the distance
GridPath = namedtuple('GridPath', 'length_m points start_moved_m')

_SIDES = ((1, 0), (-1, 0), (0, 1), (0, -1))
_STEPS = tuple((dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy)


class _Grid:
    def __init__(self, frame, width, height, resolution, origin, data):
        self.res, (self.ox, self.oy, yaw), self.data = _checked(frame, width, height, resolution, origin, data)
        self.w, self.h = width, height
        self.c, self.s = math.cos(yaw), math.sin(yaw)

    def centre(self, i):
        lx, ly = (i % self.w + .5) * self.res, (i // self.w + .5) * self.res
        return self.ox + self.c * lx - self.s * ly, self.oy + self.s * lx + self.c * ly

    def cell(self, x, y):
        """(ix, iy) of the cell holding (x, y), possibly outside the map."""
        return (math.floor((self.c * (x - self.ox) + self.s * (y - self.oy)) / self.res),
                math.floor((-self.s * (x - self.ox) + self.c * (y - self.oy)) / self.res))

    def index(self, ix, iy):
        return iy * self.w + ix if 0 <= ix < self.w and 0 <= iy < self.h else None


def _number(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value < 0:
        raise ValueError(f'{name} must be a finite number >= 0')
    return float(value)


def _point(value, name):
    if len(value) != 2 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                                  for v in value):
        raise ValueError(f'{name} must be two finite numbers (x, y)')
    return float(value[0]), float(value[1])


def _mask(g, radius, blocked):
    radius = _number(radius, 'radius')
    circles = []
    for b in blocked:
        if len(b) != 3:
            raise ValueError('blocked entries must be (x, y, r)')
        circles.append((*_point(b[:2], 'blocked centre'), _number(b[2], 'blocked radius')))
    touch = g.res / math.sqrt(2)   # a cell touches a disc when its centre is this much closer (footprint_clear)
    free = bytearray(1 if v == 0 else 0 for v in g.data)
    for i in range(len(free)):
        if free[i] and circles:
            x, y = g.centre(i)
            if any(math.hypot(x - bx, y - by) <= r + touch for bx, by, r in circles):
                free[i] = 0
    extent = math.ceil(radius / g.res) + 1
    stencil = [(dx, dy) for dy in range(-extent, extent + 1) for dx in range(-extent, extent + 1)
               if math.hypot(dx * g.res, dy * g.res) <= radius + touch]
    mask = bytearray(len(free))
    for iy in range(g.h):
        for ix in range(g.w):
            if free[iy * g.w + ix]:
                mask[iy * g.w + ix] = all(0 <= ix + dx < g.w and 0 <= iy + dy < g.h and free[(iy + dy) * g.w + ix + dx]
                                          for dx, dy in stencil)
    return bytes(mask)


def clear_mask(*, frame, width, height, resolution, origin, data, radius, blocked=()):
    """One byte per cell (row-major): 1 where footprint_clear passes at the cell centre, blocked circles occupied."""
    return _mask(_Grid(frame, width, height, resolution, origin, data), radius, blocked)


def _prepare(grid, radius, mask, blocked):
    g = _Grid(**grid)
    if mask is None:
        return g, _mask(g, radius, blocked)
    if blocked:
        raise ValueError('give blocked circles to clear_mask, not together with mask')
    if not isinstance(mask, bytes) or len(mask) != g.w * g.h:
        raise ValueError('mask must be the bytes of clear_mask for this grid')
    _number(radius, 'radius')
    return g, mask


def _start(g, mask, point, radius):
    """(cell index, distance moved) of the start, or None: its own clear cell, else the nearest within radius
    (1e-9 m float tolerance: a cell centre exactly one radius away counts)."""
    x, y = _point(point, 'start')
    ix, iy = g.cell(x, y)
    i = g.index(ix, iy)
    if i is not None and mask[i]:
        return i, 0.0
    extent = math.ceil(radius / g.res) + 1
    best = None
    for cy in range(iy - extent, iy + extent + 1):
        for cx in range(ix - extent, ix + extent + 1):
            j = g.index(cx, cy)
            if j is not None and mask[j]:
                d = round(math.dist((x, y), g.centre(j)), 9)   # 1 nm: mirror cells tie exactly
                if d <= radius + 1e-9 and (best is None or (d, cy, cx) < best[0]):
                    best = ((d, cy, cx), j)
    return None if best is None else (best[1], best[0][0])


def _goal(g, mask, point):
    i = g.index(*g.cell(*_point(point, 'goal')))
    return i if i is not None and mask[i] else None


def _neighbours(g, mask, i):
    ix, iy = i % g.w, i // g.w
    for dx, dy in _STEPS:
        j = g.index(ix + dx, iy + dy)
        if j is None or not mask[j]:
            continue
        if dx and dy:
            if not (mask[iy * g.w + ix + dx] and mask[(iy + dy) * g.w + ix]):
                continue
            yield j, g.res * math.sqrt(2)
        else:
            yield j, g.res


def _search(g, mask, start, stop=None, goal=None):
    """Dijkstra (A* towards ``goal``) from cell ``start``; pops in (distance, iy, ix) order.

    Returns (distances, previous, popped cell) where popped is the first cell for which ``stop``
    is true (or ``goal``), else None after the whole reachable part."""
    if goal is not None:
        gx, gy = goal % g.w, goal // g.w

        def heuristic(i):
            dx, dy = abs(i % g.w - gx), abs(i // g.w - gy)
            return g.res * (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy))
        stop = goal.__eq__
    else:
        heuristic = lambda i: 0.0   # noqa: E731
    distance, previous, done = {start: 0.0}, {}, set()
    queue = [(round(heuristic(start), 9), start // g.w, start % g.w, start)]
    while queue:
        _, _, _, i = heapq.heappop(queue)
        if i in done:
            continue
        done.add(i)
        if stop is not None and stop(i):
            return distance, previous, i
        for j, step in _neighbours(g, mask, i):
            d = distance[i] + step
            if j not in done and d < distance.get(j, math.inf) - 1e-12:
                distance[j], previous[j] = d, i
                heapq.heappush(queue, (round(d + heuristic(j), 9), j // g.w, j % g.w, j))
    return distance, previous, None


def grid_path(*, frame, width, height, resolution, origin, data, radius, start, goal, mask=None, blocked=()):
    """Shortest clear 8-way path start -> goal as GridPath(length_m, points, start_moved_m), or None."""
    g, mask = _prepare(dict(frame=frame, width=width, height=height, resolution=resolution, origin=origin, data=data),
                       radius, mask, blocked)
    found, target = _start(g, mask, start, radius), _goal(g, mask, goal)
    if found is None or target is None:
        return None
    first, moved = found
    distance, previous, reached = _search(g, mask, first, goal=target)
    if reached is None:
        return None
    cells = [target]
    while cells[-1] != first:
        cells.append(previous[cells[-1]])
    points = tuple(g.centre(i) for i in reversed(cells))
    if moved:
        points = (_point(start, 'start'),) + points
    return GridPath(moved + distance[target], points, moved)


def path_lengths(*, frame, width, height, resolution, origin, data, radius, start, goals, mask=None, blocked=()):
    """Length (m) of the shortest clear path from start to each goal, None where there is none; one search."""
    g, mask = _prepare(dict(frame=frame, width=width, height=height, resolution=resolution, origin=origin, data=data),
                       radius, mask, blocked)
    targets = [_goal(g, mask, goal) for goal in goals]
    found = _start(g, mask, start, radius)
    if found is None:
        return tuple(None for _ in targets)
    first, moved = found
    distance, _, _ = _search(g, mask, first)
    return tuple(None if t is None or t not in distance else moved + distance[t] for t in targets)


def find_frontiers(*, frame, width, height, resolution, origin, data, radius, min_cells, max_goal_distance_m,
                   mask=None, blocked=()):
    """Frontier clusters, largest first: free (0) cells with an unknown (-1) side neighbour, 8-connected.

    ``goal`` is the clear cell nearest the cluster centroid (ties: lower row, then column) within
    ``max_goal_distance_m``, else None; frontier cells themselves touch unknown cells and are never clear.
    Whether the goal can be reached is grid_path's question."""
    g, mask = _prepare(dict(frame=frame, width=width, height=height, resolution=resolution, origin=origin, data=data),
                       radius, mask, blocked)
    if type(min_cells) is not int or min_cells < 1:
        raise ValueError('min_cells must be an integer >= 1')
    limit = _number(max_goal_distance_m, 'max_goal_distance_m')
    edge = set()
    for i, v in enumerate(g.data):
        if v == 0:
            ix, iy = i % g.w, i // g.w
            if any((j := g.index(ix + dx, iy + dy)) is not None and g.data[j] == -1 for dx, dy in _SIDES):
                edge.add(i)
    clear = [i for i in range(len(mask)) if mask[i]]
    found, seen = [], set()
    for i in sorted(edge):
        if i in seen:
            continue
        cluster, todo = [], [i]
        seen.add(i)
        while todo:
            c = todo.pop()
            cluster.append(c)
            for dx, dy in _STEPS:
                j = g.index(c % g.w + dx, c // g.w + dy)
                if j in edge and j not in seen:
                    seen.add(j)
                    todo.append(j)
        if len(cluster) < min_cells:
            continue
        centres = [g.centre(c) for c in cluster]
        centroid = (sum(x for x, _ in centres) / len(centres), sum(y for _, y in centres) / len(centres))
        # distances rounded to 1 nm so a geometric tie stays a tie whatever float noise the sum carries
        # (Python 3.12+ sums floats with compensation, 3.10/3.11 do not)
        best = min(((round(math.dist(centroid, g.centre(c)), 9), c // g.w, c % g.w, c) for c in clear),
                   default=None)
        goal = g.centre(best[3]) if best is not None and best[0] <= limit else None
        found.append(Frontier(len(cluster), centroid, goal))
    return tuple(sorted(found, key=lambda f: (-f.cells, f.centroid)))


def nearest_in_mask(*, frame, width, height, resolution, origin, data, mask, point, within_m):
    """``point`` itself when its cell is in ``mask``, else the centre of the nearest mask cell within ``within_m``
    (ties: lower row, then column), or None. Moves a goal off walls onto a roomier clear_mask (OPS 2026-10-15)."""
    g = _Grid(frame, width, height, resolution, origin, data)
    if not isinstance(mask, bytes) or len(mask) != g.w * g.h:
        raise ValueError('mask must be the bytes of clear_mask for this grid')
    x, y = _point(point, 'point')
    limit = _number(within_m, 'within_m')
    ix, iy = g.cell(x, y)
    i = g.index(ix, iy)
    if i is not None and mask[i]:
        return x, y
    extent = math.ceil(limit / g.res) + 1
    best = None
    for cy in range(iy - extent, iy + extent + 1):
        for cx in range(ix - extent, ix + extent + 1):
            j = g.index(cx, cy)
            if j is not None and mask[j]:
                d = round(math.dist((x, y), g.centre(j)), 9)
                if d <= limit + 1e-9 and (best is None or (d, cy, cx) < best[0]):
                    best = ((d, cy, cx), j)
    return None if best is None else g.centre(best[1])


def nearest_reachable(*, frame, width, height, resolution, origin, data, radius, start, mask, among, point, within_m):
    """Centre of the ``among`` cell nearest ``point`` (straight line, within ``within_m``; ties: lower row, then
    column) that has a clear path from ``start`` (moved like a grid_path start), or None. ``among`` is a clear_mask
    (a roomier one: a frontier goal TB-6 can stop and turn at, C12); the path keeps ``mask``. Nearest the frontier,
    not the shortest path: a goal in line with a gap sees past it (9회차 2026-10-16, C14)."""
    g, mask = _prepare(dict(frame=frame, width=width, height=height, resolution=resolution, origin=origin, data=data),
                       radius, mask, ())
    if not isinstance(among, bytes) or len(among) != g.w * g.h:
        raise ValueError('among must be the bytes of clear_mask for this grid')
    x, y = _point(point, 'point')
    limit = _number(within_m, 'within_m')
    found = _start(g, mask, start, radius)
    if found is None:
        return None
    distance, _, _ = _search(g, mask, found[0])
    best = min(((round(math.dist(g.centre(i), (x, y)), 9), i // g.w, i % g.w, i) for i in distance
                if among[i] and math.dist(g.centre(i), (x, y)) <= limit + 1e-9), default=None)
    return None if best is None else g.centre(best[3])


def nearest_clear_outside(*, frame, width, height, resolution, origin, data, radius, from_xy, path_points, distance_m,
                          mask=None, blocked=()):
    """Centre of the clear cell farther than ``distance_m`` from every path point with the shortest clear path
    from ``from_xy`` (moved like a grid_path start; ties: lower row, then column), or None. A step-aside spot."""
    g, mask = _prepare(dict(frame=frame, width=width, height=height, resolution=resolution, origin=origin, data=data),
                       radius, mask, blocked)
    limit = _number(distance_m, 'distance_m')
    points = [_point(p, 'path point') for p in path_points]
    found = _start(g, mask, from_xy, radius)
    if found is None:
        return None
    _, _, cell = _search(g, mask, found[0],
                         stop=lambda i: all(math.dist(g.centre(i), p) > limit for p in points))
    return None if cell is None else g.centre(cell)
