"""One occupancy hash, MapState snapshot id and free-footprint rule for robot and coordinator (OPS-117).

After the map freeze the coordinator checks /rescue/map/frozen against TB-6 MapState itself;
it must compute exactly what the robot computes without depending on rescue_robot. The
values pinned here were computed with the robot code before the move (main fee1e72), so the
move changes no hash and no snapshot id.
"""
from types import SimpleNamespace as NS
import unittest

from rescue_contracts.occupancy import footprint_clear, map_snapshot_id, occupancy_sha256
from rescue_robot.executor import ContractError
from rescue_robot.live_pose import map_state_record
from rescue_robot.live_sources import Grid

DATA = [0] * 12
DATA[5], DATA[6] = 100, -1   # cell (1, 1) occupied, cell (2, 1) unknown
GRID = dict(frame='map', width=4, height=3, resolution=0.05, origin=(-1.0, -0.5, 0.0), data=DATA)
SHA = '8b83ee9d699d7a6f5c41919b9744fbdc004ffa1f9febc6b96d48c4b6b654077a'
SESSION = '22222222-2222-4222-8222-222222222222'
SNAPSHOT = '4bca2f81-b621-5369-ba04-691225027b13'   # session, revision 1, transform revision 1


def message(**changes):
    g = dict(GRID, **changes)
    x, y, _ = g['origin']
    return NS(header=NS(frame_id=g['frame']), info=NS(width=g['width'], height=g['height'],
              resolution=g['resolution'], origin=NS(position=NS(x=x, y=y, z=0.),
                                                    orientation=NS(x=0., y=0., z=0., w=1.))), data=g['data'])


class OccupancyContract(unittest.TestCase):
    def test_hash_is_the_robots_hash_before_the_move(self):
        self.assertEqual(SHA, occupancy_sha256(**GRID))
        self.assertEqual(SHA, Grid(message()).sha256)

    def test_any_content_change_changes_the_hash(self):
        for change in (dict(frame='odom'), dict(resolution=0.1), dict(origin=(-1.0, -0.4, 0.0)),
                       dict(data=[0] * 12)):
            with self.subTest(change=change):
                self.assertNotEqual(SHA, occupancy_sha256(**dict(GRID, **change)))

    def test_snapshot_id_is_the_robots_map_state_id_before_the_move(self):
        self.assertEqual(SNAPSHOT, map_snapshot_id(SHA, SESSION, 1, 1))
        basis = dict(map_session_id=SESSION, map_revision=1, transform_revision=1, native_frame='map')
        self.assertEqual(SNAPSHOT, map_state_record(robot_id='robot_a', occupancy_sha256=SHA, basis=basis,
                                                    map_loaded=True, localization_valid=True,
                                                    generated_at_ns=1)['snapshot_id'])
        self.assertNotEqual(SNAPSHOT, map_snapshot_id(SHA, SESSION, 2, 1))
        self.assertNotEqual(SNAPSHOT, map_snapshot_id(SHA, SESSION, 1, 2))

    def test_only_free_cells_inside_the_map_are_clear(self):
        """Same rule as the robot's approach check: cell value 0 only; occupied, unknown or outside is not clear."""
        cases = (((-0.875, -0.475), 0.01, True),    # free cell (2, 0)
                 ((-0.925, -0.425), 0.01, False),   # occupied cell (1, 1)
                 ((-0.875, -0.425), 0.01, False),   # unknown cell (2, 1)
                 ((-0.975, -0.475), 0.03, False))   # free cell (0, 0), but the radius reaches outside the map
        for (x, y), radius, expected in cases:
            with self.subTest(point=(x, y)):
                self.assertIs(expected, footprint_clear(**GRID, x=x, y=y, radius=radius))
                self.assertIs(expected, Grid(message()).clear(x, y, radius))

    def test_invalid_grids_are_refused(self):
        for change in (dict(frame=''), dict(width=0), dict(resolution=float('nan')), dict(data=DATA[:11]),
                       dict(data=DATA[:11] + [101]), dict(data=DATA[:11] + [True])):
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    occupancy_sha256(**dict(GRID, **change))
                with self.assertRaises(ContractError):
                    Grid(message(**change))


if __name__ == '__main__':
    unittest.main()
