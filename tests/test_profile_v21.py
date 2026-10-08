"""Schema v2 additive blocks (v2.1, 2026-10-08): readiness, host network, site cameras, entry_yaw.

All blocks are optional so existing v2 files keep loading; when present they are
validated strictly and never defaulted.
"""

import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rescue_bringup.cli import main
from rescue_bringup.profile import (ProfileError, load_deployment, READINESS_SOURCES,
                                    SITE_CAMERA_ROLES)

EXAMPLE = Path(__file__).resolve().parents[1] / "config"


class V21ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / "config"
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / "deployment.v2.example.json"
        self.data = json.loads(self.profile.read_text())

    def save(self, data=None):
        self.profile.write_text(json.dumps(self.data if data is None else data))

    def robot_file(self, robot="robot_a"):
        return self.config / "robots" / f"{robot}.v2.json"

    def set_readiness(self, robot="robot_a", **changes):
        path = self.robot_file(robot)
        value = json.loads(path.read_text())
        value["readiness"].update(changes)
        path.write_text(json.dumps(value))

    def assertRejected(self, fragment):
        with self.assertRaises(ProfileError) as caught:
            load_deployment(self.profile)
        self.assertIn(fragment, str(caught.exception))

    def test_example_loads_readiness_network_cameras_and_entry_yaw(self):
        profile = load_deployment(self.profile)
        a = next(r for r in profile.robots if r.id == "robot_a").parameters.readiness
        self.assertEqual("/robot_a/scan", a.scan_topic)
        self.assertEqual(set(READINESS_SOURCES), {s for s, _ in a.max_age_s})
        self.assertIsNone(a.watchdog_heartbeat_topic)
        self.assertEqual(("/robot_a/oakd/rgb/preview/image_raw/compressed",), a.camera_topics)
        central = next(h for h in profile.hosts if h.id == "operations")
        self.assertEqual(2, len(central.network.discovery_servers))
        self.assertEqual((0, 1), tuple(s.server_id for s in central.network.discovery_servers))
        self.assertFalse(central.network.super_client)
        self.assertEqual({"DISASTER_SIGN", "EXIT_PASS"}, {c.role for c in profile.site_cameras})
        self.assertTrue(all(c.id.startswith("cam_") for c in profile.site_cameras))
        self.assertEqual([0.0, 1.5708], [area.entry_yaw for area in profile.mission.search_areas])

    def test_blocks_are_optional_and_absent_means_none(self):
        for host in self.data["hosts"]:
            host.pop("network")
        self.data.pop("site_cameras")
        for area in self.data["mission"]["search_areas"]:
            area.pop("entry_yaw")
        self.save()
        for robot in ("robot_a", "robot_b"):
            path = self.robot_file(robot)
            value = json.loads(path.read_text())
            value.pop("readiness")
            path.write_text(json.dumps(value))
        profile = load_deployment(self.profile)
        self.assertEqual((), profile.site_cameras)
        self.assertTrue(all(h.network is None for h in profile.hosts))
        self.assertTrue(all(r.parameters.readiness is None for r in profile.robots))
        self.assertTrue(all(a.entry_yaw is None for a in profile.mission.search_areas))

    def test_readiness_rejections(self):
        self.set_readiness(max_age_s={"odom": 0.5})
        self.assertRejected("max_age_s")
        self.setUp()
        self.set_readiness(max_age_s=dict(odom=0.5, scan=0, camera=1, wheel_status=1, map_to_odom_tf=1))
        self.assertRejected("max_age_s.scan")
        self.setUp()
        self.set_readiness(scan_topic="/robot_b/scan")
        self.assertRejected("scan_topic")
        self.setUp()
        self.set_readiness(watchdog_heartbeat_topic="/robot_a/watchdog/heartbeat")
        self.assertRejected("watchdog_heartbeat_topic")
        self.setUp()
        self.set_readiness(camera_topics=["/robot_a/cam", "/robot_a/cam"])
        self.assertRejected("camera_topics")
        self.setUp()
        self.set_readiness(map_frame="/map")
        self.assertRejected("map_frame")

    def test_network_rejections(self):
        host = self.data["hosts"][0]
        host["network"]["domain_id"] = 233
        self.save(); self.assertRejected("domain_id")
        self.setUp(); host = self.data["hosts"][0]
        host["network"]["discovery_servers"].append(dict(host["network"]["discovery_servers"][0]))
        self.save(); self.assertRejected("server_id")
        self.setUp(); host = self.data["hosts"][0]
        host["network"]["super_client"] = True
        self.save(); self.assertRejected("super_client")
        self.setUp(); host = self.data["hosts"][0]
        host["network"]["discovery_servers"][0]["port"] = 70000
        self.save(); self.assertRejected("port")
        self.setUp(); host = self.data["hosts"][0]
        host["network"]["domain_id"] = True
        self.save(); self.assertRejected("domain_id")

    def test_site_camera_rejections(self):
        cam = self.data["site_cameras"][0]
        cam["id"] = "robot_a"
        self.save(); self.assertRejected("site_cameras[0].id")
        self.setUp(); self.data["site_cameras"][0]["host"] = "nowhere"
        self.save(); self.assertRejected("unknown host")
        self.setUp(); self.data["site_cameras"][0]["role"] = "FIRE"
        self.save(); self.assertRejected("role")
        self.setUp(); self.data["site_cameras"][1]["id"] = self.data["site_cameras"][0]["id"]
        self.save(); self.assertRejected("site_cameras id")
        self.setUp(); self.data["site_cameras"][0]["source"] = ""
        self.save(); self.assertRejected("source")
        self.assertEqual(("DISASTER_SIGN", "EXIT_PASS"), SITE_CAMERA_ROLES)

    def test_null_is_not_absent_and_server_id_is_global(self):
        self.data["hosts"][0]["network"] = None
        self.save(); self.assertRejected("network")
        self.setUp(); self.data["site_cameras"] = None
        self.save(); self.assertRejected("site_cameras")
        self.setUp(); self.data["hosts"][1]["network"]["discovery_servers"][0]["server_id"] = 5
        self.save(); self.assertRejected("different server_id across hosts")

    def test_entry_yaw_must_be_finite_number(self):
        self.data["mission"]["search_areas"][0]["entry_yaw"] = "north"
        self.save(); self.assertRejected("entry_yaw")
        self.setUp(); self.data["mission"]["search_areas"][0]["entry_yaw"] = True
        self.save(); self.assertRejected("entry_yaw")

    def test_cli_reports_cameras_and_networks(self):
        import contextlib, io
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(0, main([str(self.profile)]))
        self.assertIn("2 site cameras", out.getvalue())
        self.assertIn("4 hosts with network", out.getvalue())


if __name__ == "__main__":
    unittest.main()
