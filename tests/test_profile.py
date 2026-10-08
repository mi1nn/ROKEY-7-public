"""Behavioural checks for deployment identity and inert configuration loading."""

from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from rescue_bringup.cli import main
from rescue_bringup.profile import MAX_CONFIG_BYTES, ProfileError, load_deployment


EXAMPLE = Path(__file__).resolve().parents[1] / "config"


class DeploymentProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "config"
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / "deployment.example.json"
        self.data = json.loads(self.profile.read_text())

    def save(self, data=None):
        self.profile.write_text(json.dumps(self.data if data is None else data))

    def parameters(self, robot="robot_a", **changes):
        path = self.config / "robots" / f"{robot}.json"
        value = json.loads(path.read_text())
        value.update(changes)
        path.write_text(json.dumps(value))

    def test_example_loads_and_retains_robot_ownership(self):
        deployment = load_deployment(self.profile)
        self.assertEqual(deployment.robots[0].namespace, "/robot_a")
        self.assertEqual(deployment.robots[1].command_host, "compute_b")
        self.assertEqual(deployment.coordination_host, "coordination")
        self.assertEqual(len(deployment.hosts), 4)

    def test_hardware_roles_are_not_frozen_to_example_placement(self):
        self.data["robots"][0]["slam_host"] = "coordination"
        self.data["robots"][1]["perception_host"] = "operations"
        self.save()
        deployment = load_deployment(self.profile)
        self.assertEqual(deployment.robots[0].slam_host, "coordination")

    def test_two_robot_command_owners_can_share_a_host(self):
        self.data["robots"][1]["command_host"] = "compute_a"
        self.save()
        deployment = load_deployment(self.profile)
        self.assertEqual({robot.command_host for robot in deployment.robots}, {"compute_a"})
        self.assertEqual(deployment.robots[1].slam_host, "compute_b")

    def test_coordination_and_operations_can_share_if_all_hosts_have_roles(self):
        self.data["services"]["operations_host"] = "coordination"
        self.data["robots"][1]["perception_host"] = "operations"
        self.save()
        deployment = load_deployment(self.profile)
        self.assertEqual(deployment.coordination_host, deployment.operations_host)
        self.assertEqual(deployment.robots[1].perception_host, "operations")

    def test_policy_baseline_change_updates_counts_and_cli_without_literal_edits(self):
        self.data["hosts"] = [host for host in self.data["hosts"] if host["id"] != "compute_b"]
        for field in ["command_host", "slam_host", "perception_host"]:
            self.data["robots"][1][field] = "compute_a"
        self.save()
        stdout = io.StringIO()
        with patch("rescue_bringup.profile.HOST_TIERS", {"high": 1, "mid": 2}):
            with redirect_stdout(stdout):
                result = main([str(self.profile)])
        self.assertEqual(result, 0)
        self.assertIn("2 robots; 3 hosts (1 high, 2 mid)", stdout.getvalue())

    def test_profile_symlink_uses_target_directory_for_parameter_paths(self):
        link_dir = self.root / "profile_links"
        link_dir.mkdir()
        link = link_dir / "deployment.json"
        link.symlink_to(self.profile)
        deployment = load_deployment(link)
        self.assertEqual(deployment.robots[0].parameters.robot_id, "robot_a")
        self.assertFalse((link_dir / "robots").exists())

    def test_wrong_robot_host_counts_and_tier_mix_are_rejected(self):
        variants = []
        one_robot = deepcopy(self.data)
        one_robot["robots"].pop()
        variants.append(one_robot)
        three_hosts = deepcopy(self.data)
        three_hosts["hosts"].pop()
        variants.append(three_hosts)
        three_high = deepcopy(self.data)
        three_high["hosts"][2]["tier"] = "high"
        variants.append(three_high)
        for variant in variants:
            with self.subTest(variant=variant):
                self.save(variant)
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_duplicate_host_is_rejected(self):
        self.data["hosts"][1]["id"] = self.data["hosts"][0]["id"]
        self.save()
        with self.assertRaisesRegex(ProfileError, "duplicate host"):
            load_deployment(self.profile)

    def test_duplicate_robot_namespace_is_rejected(self):
        self.data["robots"][1]["namespace"] = "/robot_a"
        self.parameters("robot_b", namespace="/robot_a")
        self.save()
        with self.assertRaisesRegex(ProfileError, "duplicate robot namespace"):
            load_deployment(self.profile)

    def test_robot_identity_collision_is_rejected(self):
        self.data["robots"][1]["id"] = "robot_a"
        self.parameters("robot_b", robot_id="robot_a")
        self.save()
        with self.assertRaisesRegex(ProfileError, "duplicate robot id"):
            load_deployment(self.profile)

    def test_unknown_command_host_is_rejected(self):
        self.data["robots"][0]["command_host"] = "wrong_pc"
        self.save()
        with self.assertRaisesRegex(ProfileError, "unknown host"):
            load_deployment(self.profile)

    def test_fourth_host_cannot_be_listed_but_unused(self):
        self.data["services"]["operations_host"] = "coordination"
        self.save()
        with self.assertRaisesRegex(ProfileError, "without an assigned role"):
            load_deployment(self.profile)

    def test_swapped_robot_parameter_files_are_rejected(self):
        self.data["robots"][0]["parameters_file"] = "robots/robot_b.json"
        self.save()
        with self.assertRaisesRegex(ProfileError, "do not belong"):
            load_deployment(self.profile)

    def test_namespace_must_be_explicit_single_robot_prefix(self):
        for namespace in ["/", "robot_a", "/robot_a/nested", "/robot-a", "$(hostname)"]:
            with self.subTest(namespace=namespace):
                self.data["robots"][0]["namespace"] = namespace
                self.save()
                with self.assertRaisesRegex(ProfileError, "invalid robot.namespace"):
                    load_deployment(self.profile)

    def test_unknown_configuration_cannot_silently_enable_actions(self):
        self.data["robots"][0]["launch_command"] = ["ros2", "launch", "anything"]
        self.save()
        with self.assertRaisesRegex(ProfileError, "unknown=.*launch_command"):
            load_deployment(self.profile)

    def test_duplicate_json_keys_and_nonfinite_numbers_are_rejected(self):
        for raw in ['{"schema_version": 1, "schema_version": 2}', '{"value": NaN}',
                    '{"value": Infinity}']:
            with self.subTest(raw=raw):
                self.profile.write_text(raw)
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_schema_version_is_not_boolean_or_unrecognized_version(self):
        for version in [True, 3, "1", 1.0]:
            with self.subTest(version=version):
                self.data["schema_version"] = version
                self.save()
                with self.assertRaisesRegex(ProfileError, "schema_version"):
                    load_deployment(self.profile)
        # v2 is supported but a v1-shaped file cannot just bump the number: mission is required.
        self.data["schema_version"] = 2
        self.save()
        with self.assertRaisesRegex(ProfileError, "missing=\\['mission'\\]"):
            load_deployment(self.profile)

    def test_runtime_activation_is_rejected(self):
        for enabled in [True, 1, "false", None]:
            with self.subTest(enabled=enabled):
                self.parameters(runtime_enabled=enabled)
                with self.assertRaisesRegex(ProfileError, "runtime_enabled must be false"):
                    load_deployment(self.profile)

    def test_mixed_clock_modes_are_rejected(self):
        self.parameters("robot_b", use_sim_time=True)
        with self.assertRaisesRegex(ProfileError, "same use_sim_time"):
            load_deployment(self.profile)

    def test_clock_mode_does_not_accept_truthy_strings(self):
        self.parameters(use_sim_time="false")
        with self.assertRaisesRegex(ProfileError, "must be boolean"):
            load_deployment(self.profile)

    def test_parameter_path_cannot_escape_by_parent_absolute_or_symlink(self):
        outside = self.root / "outside.json"
        shutil.copyfile(self.config / "robots/robot_a.json", outside)
        (self.config / "escape.json").symlink_to(outside)
        for path in ["../outside.json", str(outside), "escape.json"]:
            with self.subTest(path=path):
                self.data["robots"][0]["parameters_file"] = path
                self.save()
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_missing_file_and_oversized_parameter_file_are_rejected(self):
        parameter_file = self.config / "robots/robot_a.json"
        parameter_file.unlink()
        with self.assertRaises(ProfileError):
            load_deployment(self.profile)
        parameter_file.write_bytes(b" " * (MAX_CONFIG_BYTES + 1))
        with self.assertRaisesRegex(ProfileError, "exceeds"):
            load_deployment(self.profile)

    def test_oversized_profile_invalid_utf8_and_deep_json_fail_cleanly(self):
        for content in [b" " * (MAX_CONFIG_BYTES + 1), b"\xff", b"[" * 2000 + b"]" * 2000]:
            with self.subTest(content_length=len(content)):
                self.profile.write_bytes(content)
                with self.assertRaises(ProfileError):
                    load_deployment(self.profile)

    def test_nonregular_profile_is_rejected_before_read(self):
        with self.assertRaisesRegex(ProfileError, "regular file"):
            load_deployment(self.config)
        if hasattr(os, "mkfifo"):
            pipe = self.root / "unreadable.json"
            os.mkfifo(pipe)
            with self.assertRaisesRegex(ProfileError, "regular file"):
                load_deployment(pipe)

    def test_cli_distinguishes_static_validation_from_runtime(self):
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            result = main([str(self.profile)])
        self.assertEqual(result, 0)
        self.assertIn("STATIC CONFIG VALID", stdout.getvalue())
        self.assertIn("Runtime is disabled", stdout.getvalue())
        self.assertIn("No hardware, ROS graph", stdout.getvalue())

    def test_invalid_cli_has_nonzero_exit_and_no_success_output(self):
        self.parameters(runtime_enabled=True)
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            result = main([str(self.profile)])
        self.assertEqual(result, 2)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn("INVALID:", stderr.getvalue())

    def assert_cli_rejects_without_traceback(self, profile):
        # The string is constructed inside Python because OS argv cannot carry NUL.
        code = ("from rescue_bringup.cli import main; "
                f"raise SystemExit(main([{str(profile)!r}]))")
        result = subprocess.run([sys.executable, "-c", code],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(result.stdout, "")
        self.assertIn("INVALID:", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_cli_rejects_overlong_json_integer_without_traceback(self):
        self.profile.write_text('{"schema_version": ' + "1" * 5000 + "}")
        self.assert_cli_rejects_without_traceback(self.profile)

    def test_cli_rejects_nul_parameter_path_without_traceback(self):
        self.data["robots"][0]["parameters_file"] = "bad\x00.json"
        self.save()
        self.assert_cli_rejects_without_traceback(self.profile)

    def test_cli_rejects_nul_input_path_without_traceback(self):
        self.assert_cli_rejects_without_traceback("bad\x00.json")


if __name__ == "__main__":
    unittest.main()
