"""target_locator parameters come from the deployment profile's map_basis (2026-10-13).

The coordinator and the AMR perception launch read the same map_basis block, so a target
location cannot be made on another basis than the coordinator's goals. The profile has no
map -> mission transform: identity is used only when the SCOUT's map frame is the mission frame.
OPS-103 (C-03·C-04): the locator also needs the measured clock domain and the alignment authority with
its validity window; they come from launch arguments (no defaults), never from code.
"""
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rescue_bringup.profile import ProfileError, load_deployment, require_map_basis
from rescue_perception.target_locator import PARAMS, TARGET_LOCATIONS_TOPIC, profile_parameters

EXAMPLE = Path(__file__).resolve().parents[1] / 'config'
FIELD = dict(tf_wait_s=1.0, clock_domain_id='rokey7-utc-field', alignment_authority_id='tb6-saved-map-identity',
             alignment_valid_from_ns=1_760_000_000_000_000_000, alignment_valid_until_ns=1_760_003_600_000_000_000)


class LocatorProfileTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.config = Path(temp.name) / 'config'
        shutil.copytree(EXAMPLE, self.config)
        self.profile = self.config / 'deployment.v2.example.json'
        self.data = json.loads(self.profile.read_text())

    def load(self):
        self.profile.write_text(json.dumps(self.data))
        return load_deployment(self.profile)

    def test_same_map_and_mission_frame_gives_every_locator_parameter(self):
        self.data['map_basis']['frame_id'] = 'map'  # field (#132): mission frame = robot map frame
        deployment = self.load()
        values = profile_parameters(deployment, require_map_basis(deployment), **FIELD)
        self.assertEqual(set(PARAMS), set(values))
        self.assertEqual(dict(robot_id='robot_a', map_frame='map', mission_frame='map',
                              map_session_id='22222222-2222-4222-8222-222222222222', map_revision=0,
                              transform_revision=1, alignment_xy_yaw=[0.0, 0.0, 0.0],
                              target_locations_topic='/rescue/target_locations', tf_wait_s=1.0,
                              clock_domain_id='rokey7-utc-field', alignment_authority_id='tb6-saved-map-identity'),
                         {key: values[key] for key in ('robot_id', 'map_frame', 'mission_frame', 'map_session_id',
                                                       'map_revision', 'transform_revision', 'alignment_xy_yaw',
                                                       'target_locations_topic', 'tf_wait_s', 'clock_domain_id',
                                                       'alignment_authority_id')})

    def test_other_mission_frame_is_refused_without_an_alignment(self):
        deployment = self.load()  # example: map_basis.frame_id 'mission', robot map frame 'map'
        with self.assertRaisesRegex(ValueError, 'alignment'):
            profile_parameters(deployment, require_map_basis(deployment), **FIELD)

    def test_missing_map_basis_stops_the_launch(self):
        del self.data['map_basis']
        with self.assertRaises(ProfileError):
            require_map_basis(self.load())

    def test_bad_wait_is_refused(self):
        self.data['map_basis']['frame_id'] = 'map'
        deployment = self.load()
        for bad in (dict(tf_wait_s=0.0), dict(tf_wait_s=-1.0), dict(tf_wait_s=float('nan')),
                    dict(alignment_valid_until_ns=FIELD['alignment_valid_from_ns']), dict(clock_domain_id=''),
                    dict(alignment_authority_id='')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                profile_parameters(deployment, require_map_basis(deployment), **(FIELD | bad))

    def test_topic_is_the_coordinator_subscription(self):
        try:
            from rescue_coordination.ros_node import TARGET_LOCATION_TOPIC
        except ImportError as exc:
            self.skipTest(f'coordinator module not importable: {exc}')
        self.assertEqual(TARGET_LOCATION_TOPIC, TARGET_LOCATIONS_TOPIC)


if __name__ == '__main__':
    unittest.main()
