"""Schema v2 (mission inputs) — roles, search areas, hold positions, per-robot ROS names."""

from copy import deepcopy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from uuid import UUID

from rescue_bringup.cli import main
from rescue_bringup.profile import ProfileError, load_deployment, ROBOT_ROLES, AREA_BASIS_KINDS

EXAMPLE = Path(__file__).resolve().parents[1] / "config"


def uid(n):
    return str(UUID(int=n))


class MissionProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / "config"
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / "deployment.v2.example.json"
        self.data = json.loads(self.profile.read_text())

    def save(self, data=None):
        self.profile.write_text(json.dumps(self.data if data is None else data))

    def parameters(self, robot="robot_a", **changes):
        path = self.config / "robots" / f"{robot}.v2.json"
        value = json.loads(path.read_text())
        value.update(changes)
        path.write_text(json.dumps(value))

    def assertRejected(self, fragment):
        with self.assertRaises(ProfileError) as caught:
            load_deployment(self.profile)
        self.assertIn(fragment, str(caught.exception))

    def test_v2_example_loads_with_roles_areas_holds_and_names(self):
        deployment = load_deployment(self.profile)
        self.assertEqual(2, deployment.schema_version)
        mission = deployment.mission
        self.assertEqual({"SCOUT", "SUPPORT"}, {role for _, role in mission.roles})
        self.assertEqual(ROBOT_ROLES, ("SCOUT", "SUPPORT"))
        support = next(r for r, role in mission.roles if role == "SUPPORT")
        self.assertIsNotNone(mission.hold_of(support))
        self.assertGreaterEqual(len(mission.search_areas), 1)
        self.assertIn(mission.search_areas[0].basis.kind, AREA_BASIS_KINDS)
        for robot in deployment.robots:
            params = robot.parameters
            for name in (params.nav_action, params.odom_topic, params.dock_status_topic):
                self.assertTrue(name.startswith(robot.namespace + "/"), name)

    def test_v1_example_still_loads_without_mission(self):
        deployment = load_deployment(self.config / "deployment.example.json")
        self.assertEqual(1, deployment.schema_version)
        self.assertIsNone(deployment.mission)
        self.assertIsNone(deployment.robots[0].parameters.nav_action)

    def test_roles_must_be_exactly_one_scout_and_one_support(self):
        robots = [r["id"] for r in self.data["robots"]]
        for roles in ({robots[0]: "SCOUT", robots[1]: "SCOUT"},
                      {robots[0]: "SUPPORT", robots[1]: "SUPPORT"},
                      {robots[0]: "SCOUT"},
                      {robots[0]: "SCOUT", robots[1]: "SUPPORT", "robot_x": "SCOUT"},
                      {robots[0]: "scout", robots[1]: "SUPPORT"}):
            with self.subTest(roles=roles):
                data = deepcopy(self.data)
                data["mission"]["roles"] = roles
                self.save(data)
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_roles_are_data_so_swapping_them_swaps_scout_and_support(self):
        robots = [r["id"] for r in self.data["robots"]]
        before = load_deployment(self.profile).mission
        data = deepcopy(self.data)
        data["mission"]["roles"] = {robots[0]: before.role_of(robots[1]), robots[1]: before.role_of(robots[0])}
        support = next(r for r, role in data["mission"]["roles"].items() if role == "SUPPORT")
        data["mission"]["hold_positions"] = {support: next(iter(self.data["mission"]["hold_positions"].values()))}
        self.save(data)
        after = load_deployment(self.profile).mission
        self.assertEqual(after.role_of(robots[0]), before.role_of(robots[1]))
        self.assertEqual(after.role_of(robots[1]), before.role_of(robots[0]))

    def test_support_robot_requires_a_hold_pose(self):
        self.data["mission"]["hold_positions"] = {}
        self.save()
        self.assertRejected("hold_positions must include the SUPPORT robot")

    def test_hold_pose_rejects_unknown_robot_nonfinite_and_boolean_revision(self):
        hold = next(iter(self.data["mission"]["hold_positions"].values()))
        support = next(iter(self.data["mission"]["hold_positions"]))
        for key, bad in (("x", "1.0"), ("yaw", True), ("map_revision", True), ("map_revision", -1),
                         ("map_session_id", uid(0)), ("frame_id", "/mission")):
            with self.subTest(key=key):
                data = deepcopy(self.data)
                data["mission"]["hold_positions"][support][key] = bad
                self.save(data)
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)
        data = deepcopy(self.data)
        data["mission"]["hold_positions"]["robot_x"] = hold
        self.save(data)
        self.assertRejected("hold_positions keys")

    def test_search_areas_need_unique_canonical_ids_finite_xy_and_known_basis(self):
        area = self.data["mission"]["search_areas"][0]
        for mutate, fragment in ((lambda a: a.update(id=uid(0)), "canonical nonzero UUID"),
                                 (lambda a: a.update(id=uid(0xABCDEF).upper()), "canonical nonzero UUID"),
                                 (lambda a: a.update(mission_xy=[1.0]), "mission_xy"),
                                 (lambda a: a.update(mission_xy=[True, 0.0]), "finite number"),
                                 (lambda a: a["basis"].update(kind="GUESS"), "basis.kind"),
                                 (lambda a: a["basis"].update(map_revision=2**64), "unsigned 64-bit"),
                                 (lambda a: a["basis"].update(transform_revision=False), "unsigned 64-bit")):
            with self.subTest(fragment=fragment):
                data = deepcopy(self.data)
                mutate(data["mission"]["search_areas"][0])
                self.save(data)
                self.assertRejected(fragment)
        data = deepcopy(self.data)
        data["mission"]["search_areas"].append(deepcopy(area))
        self.save(data)
        self.assertRejected("duplicate search_areas id")
        data = deepcopy(self.data)
        data["mission"]["search_areas"] = []
        self.save(data)
        self.assertRejected("1..")

    def test_ros_names_must_live_under_the_robot_namespace_and_be_distinct(self):
        for key, bad in (("nav_action", "/robot_b/navigate_to_pose"), ("odom_topic", "odom"),
                         ("dock_status_topic", "/robot_a/dock status"), ("odom_topic", "/robot_a/navigate_to_pose")):
            with self.subTest(key=key, bad=bad):
                shutil.copytree(EXAMPLE, self.config, dirs_exist_ok=True)
                self.parameters("robot_a", **{key: bad})
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_v2_deployment_rejects_v1_parameter_files_and_vice_versa(self):
        self.parameters("robot_a", schema_version=1)
        self.assertRejected("schema_version")
        data = json.loads((self.config / "deployment.example.json").read_text())
        data["robots"][0]["parameters_file"] = "robots/robot_a.v2.json"
        (self.config / "deployment.example.json").write_text(json.dumps(data))
        with self.assertRaises(ProfileError):
            load_deployment(self.config / "deployment.example.json")

    def test_v1_profile_cannot_carry_mission_and_v2_must(self):
        data = json.loads((self.config / "deployment.example.json").read_text())
        data["mission"] = self.data["mission"]
        (self.config / "deployment.example.json").write_text(json.dumps(data))
        with self.assertRaises(ProfileError):
            load_deployment(self.config / "deployment.example.json")
        del self.data["mission"]
        self.save()
        self.assertRejected("missing=['mission']")

    def test_cli_reports_roles_and_mission_summary(self):
        import io
        from contextlib import redirect_stdout
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(0, main([str(self.profile)]))
        text = out.getvalue()
        self.assertIn("role=SCOUT", text)
        self.assertIn("role=SUPPORT", text)
        self.assertIn("mission: schema v2", text)
        self.assertIn("Runtime is disabled", text)


if __name__ == "__main__":
    unittest.main()
