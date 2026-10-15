"""TB-5's body is cleared from the frozen map (C25, run 15 #461; no ROS).

TB-5 stands at its start while TB-6 builds the map with SLAM, so TB-6's lidar marks TB-5's body as occupied cells.
In run 15 the frozen map had an arc of occupied cells 0.10-0.15 m round TB-5 (-0.402, 1.765): no cell 0.2 m clear of
them, so the coordinator found no path start for TB-5, and TB-5's Nav2 global costmap (the same map) had its start
cell LETHAL. The field is the same: TB-5 waits at its start while TB-6 maps.

At each freeze (R1 and R2) map_freeze turns the occupied cells within SUPPORT_BODY_CLEAR_M [제안] of TB-5's known start
into free cells, before the hash: TB-6's start pose (start_pose.json) plus robots.<TB-5>.initial_pose_xy_yaw, the
same place frozen_localization gives AMCL. Unknown cells stay unknown; walls farther away stay. Without a measured
TB-5 start nothing is cleared, and the record says so (no place is made up).
"""
import math
from pathlib import Path
import tempfile
import unittest

from rescue_robot.map_freeze import MapFreeze

from test_robot_map_freeze import NOW, SESSION, pose, sha

RES = 0.05


def room(cells):
    """A 2 x 2 m grid at (-1, -1), free, with ``cells`` {(x, y): value} set."""
    data = [0] * (40 * 40)
    for (x, y), value in cells.items():
        data[y * 40 + x] = value
    return dict(frame='map', width=40, height=40, resolution=RES, position=(-1.0, -1.0, 0.0),
                orientation=(0., 0., 0., 1.), data=data)


def cell(x, y):
    return math.floor((x + 1.0) / RES), math.floor((y + 1.0) / RES)


def centre(c):
    return (c[0] + 0.5) * RES - 1.0, (c[1] + 0.5) * RES - 1.0


def ring(x, y, r):
    """Occupied cells whose centres lie r +- 0.025 m from (x, y): a body outline seen by a lidar."""
    out = {}
    for cx in range(40):
        for cy in range(40):
            px, py = centre((cx, cy))
            if abs(math.hypot(px - x, py - y) - r) <= 0.025:
                out[(cx, cy)] = 100
    return out


class ClearSupportBody(unittest.TestCase):
    BODY = (0.3, 0.2)   # TB-6 start (0, 0, 0) + TB-5 start [0.3, 0.2, 0]

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def freeze(self, grid, *, support_start=(0.3, 0.2, 0.0), start=(0.0, 0.0, 0.0)):
        f = MapFreeze(map_frame='map', map_session_id=SESSION, map_revision=0, pose_max_age_ns=2_000_000_000,
                      output_dir=Path(self.dir.name), wall_clock_ns=lambda: NOW, git_sha='abc123',
                      support_start=None if support_start is None else ('robot_b', support_start))
        x, y, yaw = start
        f.on_tf_pose(pose(position=(x, y, 0.0), orientation=(0., 0., math.sin(yaw / 2), math.cos(yaw / 2))),
                     covariance_fresh=True)
        f.on_map(grid)
        return f.freeze()

    def at(self, record, xy):
        x, y = cell(*xy)
        return record['grid']['data'][y * 40 + x]

    @unittest.expectedFailure
    def test_the_body_outline_is_free_and_a_wall_farther_out_stays(self):
        body = ring(*self.BODY, 0.125)
        wall = {(cell(0.65, 0.2)[0], y): 100 for y in range(40)}   # x = 0.65: 0.35 m from TB-5
        record = self.freeze(room({**body, **wall}))
        data = record['grid']['data']
        self.assertEqual([], [c for c in body if data[c[1] * 40 + c[0]] != 0])
        self.assertEqual([], [c for c in wall if data[c[1] * 40 + c[0]] != 100])
        self.assertEqual(dict(robot_id='robot_b', xy=[0.3, 0.2], radius_m=0.25, cells=len(body)),
                         record['support_body_cleared'])
        self.assertEqual(sha(record['grid']), record['occupancy_sha256'])   # the robots get the cleared grid

    @unittest.expectedFailure
    def test_the_place_is_the_tb6_start_pose_plus_the_tb5_start(self):
        # TB-6 started at (0.1, -0.1) facing +y: TB-5 [0.3, 0.2, 0] relative is (-0.1, 0.2) on the map
        body = ring(-0.1, 0.2, 0.125)
        record = self.freeze(room(body), start=(0.1, -0.1, math.pi / 2))
        self.assertEqual(len(body), record['support_body_cleared']['cells'])
        self.assertEqual([-0.1, 0.2], [round(v, 9) for v in record['support_body_cleared']['xy']])

    @unittest.expectedFailure
    def test_only_occupied_cells_within_the_radius_change(self):
        inside, outside = cell(0.3, 0.2 + 0.2), cell(0.3, 0.2 + 0.3)
        record = self.freeze(room({inside: 100, outside: 100, cell(0.3, 0.1): -1, cell(0.3, 0.25): 70}))
        self.assertEqual((0, 100, -1, 0),
                         (self.at(record, (0.3, 0.4)), self.at(record, (0.3, 0.5)), self.at(record, (0.3, 0.1)),
                          self.at(record, (0.3, 0.25))))
        self.assertEqual(2, record['support_body_cleared']['cells'])

    @unittest.expectedFailure
    def test_no_measured_tb5_start_clears_nothing(self):
        body = ring(*self.BODY, 0.125)
        g = room(body)
        record = self.freeze(g, support_start=None)
        self.assertEqual(g['data'], record['grid']['data'])
        self.assertIsNone(record['support_body_cleared'])


if __name__ == '__main__':
    unittest.main()
