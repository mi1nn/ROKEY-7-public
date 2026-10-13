"""Profile v2 map_basis: the one map basis the coordinator goals and target locations share.

The coordinator refuses a TargetLocation or goal on another basis (ROUTE_BASIS_MISMATCH), so
the coordinator launch and the AMR target locator launch read these four values from the
profile instead of each holding a copy. The block is optional so existing v2 profiles keep
loading; a launch that needs it calls ``require_map_basis`` and fails without it (no default).
"""
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rescue_bringup.profile import MapBasis, ProfileError, load_deployment, require_map_basis

EXAMPLE = Path(__file__).resolve().parents[1] / "config"


class MapBasisTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / "config"
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / "deployment.v2.example.json"
        self.data = json.loads(self.profile.read_text())

    def save(self):
        self.profile.write_text(json.dumps(self.data))

    def assertRejected(self, fragment):
        self.save()
        with self.assertRaises(ProfileError) as caught:
            load_deployment(self.profile)
        self.assertIn(fragment, str(caught.exception))

    def test_example_carries_the_map_basis(self):
        basis = require_map_basis(load_deployment(self.profile))
        self.assertEqual(MapBasis(frame_id="mission", map_session_id="22222222-2222-4222-8222-222222222222",
                                  map_revision=0, transform_revision=1), basis)

    def test_absent_block_loads_but_cannot_be_required(self):
        del self.data["map_basis"]
        self.save()
        deployment = load_deployment(self.profile)
        self.assertIsNone(deployment.map_basis)
        with self.assertRaises(ProfileError) as caught:
            require_map_basis(deployment)
        self.assertIn("map_basis", str(caught.exception))

    def test_v1_profile_has_no_map_basis(self):
        with self.assertRaises(ProfileError):
            require_map_basis(load_deployment(self.config / "deployment.example.json"))

    def test_each_value_is_checked(self):
        for key, bad in (("frame_id", ""), ("frame_id", "/map"), ("map_session_id", "not-a-uuid"),
                         ("map_revision", -1), ("map_revision", 1.0), ("map_revision", True),
                         ("transform_revision", "1")):
            with self.subTest(key=key, bad=bad):
                self.data = json.loads((EXAMPLE / "deployment.v2.example.json").read_text())
                self.data["map_basis"][key] = bad
                self.assertRejected(f"map_basis.{key}")

    def test_missing_and_extra_keys_are_refused(self):
        for change in (lambda b: b.pop("transform_revision"), lambda b: b.update(extra=1)):
            with self.subTest(change=change):
                self.data = json.loads((EXAMPLE / "deployment.v2.example.json").read_text())
                change(self.data["map_basis"])
                self.assertRejected("map_basis")
        self.data["map_basis"] = None
        self.assertRejected("map_basis")

    def test_prior_map_search_areas_must_be_on_the_map_basis(self):
        area = self.data["mission"]["search_areas"][1]["basis"]
        for key, other in (("map_session_id", "33333333-3333-4333-8333-333333333333"), ("map_revision", 1),
                           ("transform_revision", 2)):
            with self.subTest(key=key):
                saved = area[key]
                area[key] = other
                self.assertRejected("search_areas[1]")
                area[key] = saved

    def test_hold_positions_must_be_on_the_map_basis(self):
        """The map only: a hold pose may name a robot-scoped TF frame (test_profile_v2)."""
        hold = self.data["mission"]["hold_positions"]["robot_b"]
        for key, other in (("map_session_id", "33333333-3333-4333-8333-333333333333"), ("map_revision", 1)):
            with self.subTest(key=key):
                saved = hold[key]
                hold[key] = other
                self.assertRejected("hold_positions[robot_b]")
                hold[key] = saved


if __name__ == "__main__":
    unittest.main()
