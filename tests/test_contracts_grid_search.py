"""Frontier and grid path on the OccupancyGrid fields, shared with the coordinator (OPS-154).

TB-6 searches the whole map: the coordinator picks the next frontier, and after the search
TB-5 leaves by the exit nearest to the doll along free cells. Both use these pure functions on
the same grid fields as occupancy.footprint_clear, so the same grid gives the same answer.
"""
import math
import unittest

from rescue_contracts.grid_search import clear_mask, find_frontiers, grid_path, nearest_clear_outside, path_lengths
from rescue_contracts.occupancy import footprint_clear

R = 0.1   # radius 0.1 m at resolution 0.1 m: a cell is clear when its 3x3 block is in the map and free


def grid(width, height, cells=(), *, origin=(0.0, 0.0, 0.0), resolution=0.1):
    data = [0] * (width * height)
    for (ix, iy), value in cells:
        data[iy * width + ix] = value
    return dict(frame='map', width=width, height=height, resolution=resolution, origin=origin, data=data)


def centre(g, ix, iy):
    ox, oy, yaw = g['origin']
    lx, ly = (ix + .5) * g['resolution'], (iy + .5) * g['resolution']
    return ox + math.cos(yaw) * lx - math.sin(yaw) * ly, oy + math.sin(yaw) * lx + math.cos(yaw) * ly


class ClearMaskTests(unittest.TestCase):
    def test_same_as_footprint_clear_at_every_cell_centre(self):
        cells = [((5, 4), 100), ((6, 4), 100), ((11, 2), -1), ((0, 7), 50)]
        g = grid(14, 9, cells, origin=(0.3, -1.2, 0.4), resolution=0.05)
        for radius in (0.0, 0.07, 0.2):
            mask = clear_mask(**g, radius=radius)
            self.assertEqual(14 * 9, len(mask))
            for iy in range(9):
                for ix in range(14):
                    x, y = centre(g, ix, iy)
                    with self.subTest(radius=radius, cell=(ix, iy)):
                        self.assertEqual(footprint_clear(**g, x=x, y=y, radius=radius), bool(mask[iy * 14 + ix]))

    def test_blocked_circles_count_as_occupied_cells(self):
        g = grid(14, 9, [((5, 4), 100)], origin=(0.3, -1.2, 0.4), resolution=0.05)
        blocked = ((0.6, -0.6, 0.08), (0.9, -0.2, 0.0))
        data = list(g['data'])   # what the function must equal: cells touching a circle set occupied
        for iy in range(9):
            for ix in range(14):
                x, y = centre(g, ix, iy)
                if any(math.hypot(x - bx, y - by) <= r + 0.05 / math.sqrt(2) for bx, by, r in blocked):
                    data[iy * 14 + ix] = 100
        self.assertNotEqual(data, g['data'])
        self.assertEqual(clear_mask(**dict(g, data=data), radius=0.07), clear_mask(**g, radius=0.07, blocked=blocked))

    def test_mask_with_blocked_or_of_the_wrong_size_is_refused(self):
        g = grid(6, 6)
        mask = clear_mask(**g, radius=R)
        for kwargs in (dict(mask=mask, blocked=((0.3, 0.3, 0.1),)), dict(mask=mask[:-1]), dict(mask=list(mask))):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                grid_path(**g, radius=R, start=(0.25, 0.25), goal=(0.35, 0.35), **kwargs)

    def test_invalid_grid_is_refused(self):
        with self.assertRaises(ValueError):
            clear_mask(**dict(grid(3, 3), data=[0] * 8), radius=R)


class FrontierTests(unittest.TestCase):
    # columns 15..19 unknown: frontier = column 14; one unknown cell inside the free part
    G = grid(20, 10, [((ix, iy), -1) for ix in range(15, 20) for iy in range(10)] + [((3, 3), -1)])

    def test_clusters_largest_first_with_a_clear_goal_nearest_the_centroid(self):
        found = find_frontiers(**self.G, radius=R, min_cells=1, max_goal_distance_m=1.0)
        self.assertEqual([10, 4], [f.cells for f in found])
        big, small = found
        self.assertEqual((1.45, 0.5), tuple(round(v, 9) for v in big.centroid))
        self.assertEqual((1.35, 0.45), tuple(round(v, 9) for v in big.goal))   # tie (0.45 / 0.55) -> lower row
        self.assertEqual((0.35, 0.35), tuple(round(v, 9) for v in small.centroid))
        self.assertEqual((0.35, 0.15), tuple(round(v, 9) for v in small.goal))
        for f in found:
            self.assertTrue(footprint_clear(**self.G, x=f.goal[0], y=f.goal[1], radius=R))

    def test_small_clusters_are_left_out(self):
        self.assertEqual([10], [f.cells for f in find_frontiers(**self.G, radius=R, min_cells=5,
                                                                max_goal_distance_m=1.0)])

    def test_no_clear_cell_near_enough_is_a_frontier_without_goal(self):
        found = find_frontiers(**self.G, radius=R, min_cells=5, max_goal_distance_m=0.05)
        self.assertEqual([(10, None)], [(f.cells, f.goal) for f in found])

    def test_fully_known_map_has_no_frontier(self):
        self.assertEqual((), find_frontiers(**grid(8, 8, [((4, 4), 100)]), radius=R, min_cells=1,
                                            max_goal_distance_m=1.0))

    def test_goal_avoids_a_blocked_circle(self):
        blocked = ((1.35, 0.5, 0.05),)
        goal = find_frontiers(**self.G, radius=R, min_cells=5, max_goal_distance_m=1.0, blocked=blocked)[0].goal
        mask = clear_mask(**self.G, radius=R, blocked=blocked)
        self.assertTrue(mask[math.floor(goal[1] / 0.1) * 20 + math.floor(goal[0] / 0.1)])
        self.assertGreater(math.dist(goal, (1.35, 0.5)), 0.05 + R)

    def test_same_grid_same_result(self):
        args = dict(self.G, radius=R, min_cells=1, max_goal_distance_m=1.0)
        self.assertEqual(find_frontiers(**args), find_frontiers(**args))


class PathTests(unittest.TestCase):
    # wall in column 10, rows 0..6: the way round is row 8 (row 9 touches the map edge)
    G = grid(20, 10, [((10, iy), 100) for iy in range(7)])

    def test_shortest_8_way_path_round_the_wall_without_cutting_corners(self):
        found = grid_path(**self.G, radius=R, start=(0.25, 0.25), goal=(1.75, 0.25))
        self.assertAlmostEqual(0.1 * (11 * math.sqrt(2) + 5), found.length_m)
        self.assertEqual(((0.25, 0.25), (1.75, 0.25)), (found.points[0], found.points[-1]))
        mask = clear_mask(**self.G, radius=R)
        cells = [(math.floor(x / 0.1), math.floor(y / 0.1)) for x, y in found.points]
        self.assertIn((10, 8), cells)
        for (ax, ay), (bx, by) in zip(cells, cells[1:]):
            self.assertTrue(max(abs(bx - ax), abs(by - ay)) == 1 and mask[by * 20 + bx])
            if ax != bx and ay != by:   # a diagonal step needs both side cells clear
                self.assertTrue(mask[ay * 20 + bx] and mask[by * 20 + ax])
        self.assertAlmostEqual(found.length_m, sum(math.dist(a, b) for a, b in zip(found.points, found.points[1:])))

    def test_start_and_goal_snap_to_their_cell_centres_on_a_rotated_grid(self):
        g = grid(12, 6, origin=(1.0, 2.0, math.pi / 2))
        start, goal = centre(g, 2, 2), centre(g, 9, 3)
        found = grid_path(**g, radius=R, start=(start[0] + 0.02, start[1] - 0.03), goal=goal)
        self.assertAlmostEqual(0.1 * (math.sqrt(2) + 6), found.length_m)
        self.assertEqual((round(start[0], 9), round(start[1], 9)), tuple(round(v, 9) for v in found.points[0]))

    def test_no_way_through_is_none(self):
        closed = grid(20, 10, [((10, iy), 100) for iy in range(10)])
        self.assertIsNone(grid_path(**closed, radius=R, start=(0.25, 0.25), goal=(1.75, 0.25)))

    def test_a_start_that_is_not_clear_moves_to_the_nearest_clear_cell_within_the_radius(self):
        found = grid_path(**self.G, radius=R, start=(0.05, 0.25), goal=(1.75, 0.25))   # touches the map edge
        self.assertAlmostEqual(0.1, found.start_moved_m)
        self.assertEqual((0.05, 0.25), found.points[0])
        self.assertEqual((0.15, 0.25), tuple(round(v, 9) for v in found.points[1]))
        self.assertAlmostEqual(0.1 + 0.1 * (11 * math.sqrt(2) + 6), found.length_m)
        self.assertEqual(0.0, grid_path(**self.G, radius=R, start=(0.25, 0.25), goal=(1.75, 0.25)).start_moved_m)

    def test_start_without_a_clear_cell_in_the_radius_or_a_goal_not_clear_is_none(self):
        for start, goal in (((1.05, 0.25), (1.75, 0.25)),    # in the wall
                            ((0.25, 0.25), (1.95, 0.25)),    # goal touches the map edge: never moved
                            ((0.25, 0.25), (5.0, 0.25))):    # outside the map
            with self.subTest(start=start, goal=goal):
                self.assertIsNone(grid_path(**self.G, radius=R, start=start, goal=goal))

    def test_same_cell_is_a_zero_length_path(self):
        found = grid_path(**self.G, radius=R, start=(0.25, 0.25), goal=(0.26, 0.27))
        self.assertEqual((0.0, ((0.25, 0.25),), 0.0), (found.length_m, tuple((round(x, 9), round(y, 9))
                                                                             for x, y in found.points), found.start_moved_m))

    def test_a_blocked_circle_in_the_gap_closes_the_way(self):
        self.assertIsNone(grid_path(**self.G, radius=R, start=(0.25, 0.25), goal=(1.75, 0.25),
                                    blocked=((1.05, 0.85, 0.05),)))

    def test_lengths_to_many_goals_in_one_search_equal_grid_path(self):
        goals = ((1.75, 0.25), (0.85, 0.85), (1.95, 0.25), (1.05, 0.25), (0.25, 0.25))
        for start in ((0.25, 0.25), (0.05, 0.25)):
            for blocked in ((), ((1.5, 0.5, 0.1),)):
                lengths = path_lengths(**self.G, radius=R, start=start, goals=goals, blocked=blocked)
                expected = [grid_path(**self.G, radius=R, start=start, goal=g, blocked=blocked) for g in goals]
                with self.subTest(start=start, blocked=blocked):
                    self.assertEqual([e is None for e in expected], [v is None for v in lengths])
                    for value, e in zip(lengths, expected):
                        if e is not None:
                            self.assertAlmostEqual(e.length_m, value)
        mask = clear_mask(**self.G, radius=R)
        self.assertEqual(path_lengths(**self.G, radius=R, start=(0.25, 0.25), goals=goals),
                         path_lengths(**self.G, radius=R, start=(0.25, 0.25), goals=goals, mask=mask))

    def test_same_grid_same_path(self):
        args = dict(self.G, radius=R, start=(0.25, 0.25), goal=(1.75, 0.25))
        self.assertEqual(grid_path(**args), grid_path(**args))


class StepAsideTests(unittest.TestCase):
    G = grid(20, 10)
    PATH = tuple((0.25 + 0.1 * i, 0.45) for i in range(16))

    def test_nearest_clear_cell_by_grid_path_farther_than_the_distance_from_every_path_point(self):
        found = nearest_clear_outside(**self.G, radius=R, from_xy=(1.0, 0.45), path_points=self.PATH, distance_m=0.25)
        self.assertEqual((1.05, 0.15), tuple(round(v, 9) for v in found))   # rows 1 and 7 tie: lower row first

    def test_blocked_cells_are_not_chosen(self):
        found = nearest_clear_outside(**self.G, radius=R, from_xy=(1.0, 0.45), path_points=self.PATH, distance_m=0.25,
                                      blocked=((1.05, 0.15, 0.0),))
        self.assertEqual((1.05, 0.75), tuple(round(v, 9) for v in found))

    def test_nothing_far_enough_is_none(self):
        self.assertIsNone(nearest_clear_outside(**self.G, radius=R, from_xy=(1.0, 0.45), path_points=self.PATH,
                                                distance_m=5.0))


if __name__ == '__main__':
    unittest.main()
