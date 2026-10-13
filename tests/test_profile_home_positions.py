"""Profile v2 home_positions: where each robot returns (user decision 5A, 2026-10-13).

TB-6 (SCOUT) returns to its docking approach point, TB-5 (SUPPORT) to the exit. One place for
both sides: the coordinator launch passes them as home_poses, the robot launch as this robot's
RETURN_HOME destination. They are on the profile map_basis, so the block needs it. Values are
measured at the site; the block is optional (absent = no home, the coordinator says
NO_HOME_POSITION).
"""
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rescue_bringup.profile import ProfileError, load_deployment

EXAMPLE = Path(__file__).resolve().parents[1] / "config"


class HomePositionsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / "config"
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / "deployment.v2.example.json"
        self.data = json.loads(self.profile.read_text())

    def load(self, homes):
        self.data["home_positions"] = homes
        self.profile.write_text(json.dumps(self.data))
        return load_deployment(self.profile)

    def test_absent_block_means_no_homes(self):
        self.assertEqual((), load_deployment(self.profile).home_positions)

    def test_homes_per_robot_on_the_map_basis(self):
        deployment = self.load({"robot_a": [1.0, 2.0, 0.5], "robot_b": [3.0, -4.0, 0.0]})
        self.assertEqual((("robot_a", (1.0, 2.0, 0.5)), ("robot_b", (3.0, -4.0, 0.0))), deployment.home_positions)
        self.assertEqual((3.0, -4.0, 0.0), deployment.home_of("robot_b"))
        self.assertIsNone(self.load({"robot_a": [1.0, 2.0, 0.5]}).home_of("robot_b"))

    def test_bad_blocks_are_refused(self):
        for homes in (None, [], {"robot_c": [0.0, 0.0, 0.0]}, {"robot_a": [0.0, 0.0]},
                      {"robot_a": [0.0, 0.0, float("nan")]}, {"robot_a": [0.0, 0.0, True]}, {"robot_a": "dock"}):
            with self.subTest(homes=homes), self.assertRaises(ProfileError):
                self.load(homes)

    def test_homes_need_the_map_basis(self):
        del self.data["map_basis"]
        with self.assertRaisesRegex(ProfileError, "map_basis"):
            self.load({"robot_a": [1.0, 2.0, 0.5]})


if __name__ == "__main__":
    unittest.main()
