"""scripts/sim/fake_target.py 순수 부분(ROS 없음): 인형 지도 좌표, 보임 판정, amr_target 형식 관측 (2026-10-15).

관측은 실제 target_location.locate(target_locator 가 부르는 함수)에 넣어 거절되지 않고 인형 자리로 돌아오는지 본다.
시뮬레이션 카메라 값은 turtlebot4_description oakd.urdf.xacro(horizontal_fov 1.25, 320x240, 광학 joint rpy
(-pi/2, 0, -pi/2))에서 왔다.
"""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from rescue_bringup.field_launch import load_field
from rescue_bringup.profile import load_deployment
from rescue_contracts.occupancy import compose_planar
from rescue_mapping import RigidTransform
from rescue_perception.target_location import LocationBasis, LocationRefused, locate
from rescue_perception.target_utils import SETTINGS


REPO = Path(__file__).resolve().parents[1]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fake = _load('fake_target', REPO / 'scripts' / 'sim' / 'fake_target.py')

DOLL_ARENA = (1.20, 2.87)            # 평면도 인형
SCOUT_START = (0.88, 0.37, -1.5708)  # 평면도 TB-6, 남쪽 도크를 봄
OPTICAL = 'oakd_rgb_camera_optical_frame'
FX = 160 / math.tan(1.25 / 2)        # 320 px, 화각 1.25 rad
CAMERA = fake.Camera(320, 240, FX, FX, 160.0, 120.0)
DOMAIN = 'rokey7-utc-field'
OBSERVED = 1_792_070_000_123_456_789
SESSION = '22222222-2222-4222-8222-222222222222'
CONFIG = dict(robot_id='robot_a', camera_id=SETTINGS['camera_id'], class_label=SETTINGS['model_label'],
              validity_ns=SETTINGS['observation_validity_ns'], clock_domain_id=DOMAIN, rgb_optical_frame=OPTICAL)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rpy(roll, pitch, yaw):
    qx = (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))
    qy = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return qmul(qz, qmul(qy, qx))


OPTICAL_IN_BASE = rpy(-math.pi / 2, 0.0, -math.pi / 2)   # oakd_rgb_camera_optical_joint


def camera_pose(x, y, yaw, height=0.25):
    """map <- 광학 frame: 로봇이 (x, y, yaw) 에 서 있고 카메라가 height m 에 있을 때."""
    return RigidTransform((x, y, height), qmul(rpy(0.0, 0.0, yaw), OPTICAL_IN_BASE))


def free_grid(fill=None):
    """map frame 6 x 4 m, 0.05 m, 원점 (-4, -2). fill(x, y) -> 칸 값."""
    width, height, res, ox, oy = 120, 80, 0.05, -4.0, -2.0
    data = [0] * (width * height)
    if fill is not None:
        for iy in range(height):
            for ix in range(width):
                data[iy * width + ix] = fill(ox + (ix + .5) * res, oy + (iy + .5) * res)
    return fake.grid_from_fields('map', width, height, res, (ox, oy, 0.0), (0.0, 0.0, 0.0, 1.0), data)


def doll_xyz():
    return (*fake.doll_in_map(DOLL_ARENA, SCOUT_START, (0.0, 0.0, 0.0)), fake.DOLL_HEIGHT_M / 2)


class DollInMap(unittest.TestCase):
    def test_floor_plan_doll_is_behind_the_docked_scout(self):
        # TB-6 이 남쪽(도크)을 보므로 지도 +x 는 남쪽, +y 는 동쪽. 인형은 북쪽 2.50 m, 동쪽 0.32 m.
        x, y = fake.doll_in_map(DOLL_ARENA, SCOUT_START, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(-2.50, x, places=3)
        self.assertAlmostEqual(0.32, y, places=3)

    def test_start_pose_carries_the_relative_doll_like_the_coordinator(self):
        start = (0.10, -0.20, 0.30)
        relative = fake.doll_in_map(DOLL_ARENA, SCOUT_START, (0.0, 0.0, 0.0))
        expected = compose_planar(start, (*relative, 0.0))[:2]
        got = fake.doll_in_map(DOLL_ARENA, SCOUT_START, start)
        self.assertAlmostEqual(expected[0], got[0], places=9)
        self.assertAlmostEqual(expected[1], got[1], places=9)

    def test_start_pose_message_rules(self):
        half = math.sqrt(0.5)
        self.assertEqual((1.0, 2.0, math.pi / 2), fake.start_pose('map', (1.0, 2.0, 0.0), (0, 0, half, half), 'map'))
        with self.assertRaises(ValueError):
            fake.start_pose('odom', (0.0, 0.0, 0.0), (0, 0, 0, 1), 'map')
        with self.assertRaises(ValueError):   # 기운 자세는 조정기처럼 거절(투영하지 않음)
            fake.start_pose('map', (0.0, 0.0, 0.0), (0.05, 0, 0, math.sqrt(1 - 0.0025)), 'map')


class CameraModel(unittest.TestCase):
    def test_camera_info_gives_the_simulated_fov(self):
        camera = fake.camera_model(320, 240, [FX, 0, 160, 0, FX, 120, 0, 0, 1], OPTICAL, OPTICAL)
        self.assertAlmostEqual(1.25, fake.horizontal_fov(camera), places=6)

    def test_other_frame_or_bad_k_is_refused(self):
        with self.assertRaises(ValueError):
            fake.camera_model(320, 240, [FX, 0, 160, 0, FX, 120, 0, 0, 1], 'oakd_rgb_camera_frame', OPTICAL)
        with self.assertRaises(ValueError):
            fake.camera_model(320, 240, [0, 0, 160, 0, FX, 120, 0, 0, 1], OPTICAL, OPTICAL)

    def test_optical_frame_looks_along_base_x(self):
        pose = camera_pose(0.0, 0.0, 0.0)
        ahead = pose.apply((0.0, 0.0, 1.0))
        self.assertAlmostEqual(1.0, ahead[0], places=9)
        self.assertAlmostEqual(0.0, ahead[1], places=9)


class Visibility(unittest.TestCase):
    def test_visible_in_front_within_range(self):
        sight = fake.assess(doll_xyz(), camera_pose(-1.5, 0.32, math.pi), CAMERA, free_grid())
        self.assertIsNone(sight.reason, sight.detail)
        self.assertAlmostEqual(1.0, sight.sensor[2], places=4)      # 앞으로 1 m (yaw -1.5708 은 -pi/2 와 1e-6 차이)
        self.assertAlmostEqual(0.0, sight.sensor[0], places=4)      # 가운데
        self.assertAlmostEqual(0.15, sight.sensor[1], places=4)     # 카메라 0.25 m, 인형 중심 0.10 m: 아래로 0.15
        x0, y0, x1, y1 = sight.bbox
        self.assertTrue(0 < x0 < x1 < 320 and 0 < y0 < y1 < 240)

    def test_behind_far_and_outside_the_view(self):
        cases = {'NOT_IN_FRONT': camera_pose(-1.5, 0.32, 0.0),
                 'OUT_OF_RANGE': camera_pose(0.0, 0.32, math.pi),
                 'OUT_OF_VIEW': camera_pose(-1.5, 0.32, math.pi - 1.0)}   # 57도 옆: 반화각 35.8도 밖
        for reason, pose in cases.items():
            with self.subTest(reason):
                self.assertEqual(reason, fake.assess(doll_xyz(), pose, CAMERA, free_grid()).reason)

    def test_occupied_cell_between_blocks(self):
        wall = free_grid(lambda x, y: 100 if -2.10 < x < -2.00 else 0)
        sight = fake.assess(doll_xyz(), camera_pose(-1.5, 0.32, math.pi), CAMERA, wall)
        self.assertEqual('OCCLUDED', sight.reason)

    def test_unknown_cells_and_the_dolls_own_cells_do_not_block(self):
        doll = doll_xyz()
        grid = free_grid(lambda x, y: 100 if math.hypot(x - doll[0], y - doll[1]) < 0.09 else -1)
        self.assertIsNone(fake.assess(doll, camera_pose(-1.5, 0.32, math.pi), CAMERA, grid).reason)

    def test_rotated_grid_origin_uses_the_planar_rule(self):
        # 원점 yaw 90도: 칸 (1, 0) 은 x -1..0, y 1..2 에 있다(돌리지 않았으면 x 1..2, y 0..1).
        half = math.sqrt(0.5)
        grid = fake.grid_from_fields('map', 2, 1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, half, half), [0, 100])
        self.assertEqual((1, 0, 100), fake.blocking_cell(grid, (-0.5, 0.2), (-0.5, 1.9), skip_end_m=0.0))
        self.assertIsNone(fake.blocking_cell(grid, (1.5, 0.2), (1.5, 0.9), skip_end_m=0.0))


class Clock(unittest.TestCase):
    RECORD = dict(state='SYNCED', error_bound_ns=40_000_000, measured_at_ns=OBSERVED - 10**9,
                  sampled_at_ns=OBSERVED - 10**9, valid_until_ns=OBSERVED + 10**12, drift_ppm=100.0)

    def test_synced_adds_the_sensor_host_bound_like_clock_binding(self):
        state, bound = fake.observation_clock_fields(self.RECORD, OBSERVED, clock_domain_id=DOMAIN,
                                                     sensor_host_bound_ns=150_000_000, sensor_host_domain_id=DOMAIN)
        self.assertEqual('SYNCED', state)
        self.assertEqual(40_000_000 + 100_000 + 150_000_000, bound)   # 상한 + 1 s x 100 ppm drift + 센서 호스트

    def test_unknown_when_not_measured_or_other_domain(self):
        for record, domain in ((dict(state='UNKNOWN', reason='waiting'), DOMAIN), (self.RECORD, 'other')):
            with self.subTest(domain):
                self.assertEqual(('UNKNOWN', None), fake.observation_clock_fields(
                    record, OBSERVED, clock_domain_id=DOMAIN, sensor_host_bound_ns=150_000_000,
                    sensor_host_domain_id=domain))


def observation(clock_state='SYNCED', bound=190_100_000, pose=None):
    pose = pose or camera_pose(-1.5, 0.32, math.pi)
    sight = fake.assess(doll_xyz(), pose, CAMERA, free_grid())
    record = fake.observation_record(config=CONFIG, boot_id='5f0c1a2b-3c4d-4e5f-8a6b-7c8d9e0f1a2b',
                                     observed_at_ns=OBSERVED, clock_state=clock_state, clock_error_bound_ns=bound,
                                     sensor=sight.sensor, bbox=sight.bbox, model_sha256='a' * 64,
                                     evidence_sha256=fake.evidence_digest({'observed_at_ns': OBSERVED}))
    message = fake.fill(NS(time=NS(), sensor_position=NS(), evidence=NS()), record)
    return message, pose


def tf_at(pose, stamp_ns, frame='map', child=OPTICAL):
    (tx, ty, tz), (qx, qy, qz, qw) = pose.translation_xyz, pose.rotation_xyzw
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)),
              child_frame_id=child, transform=NS(translation=NS(x=tx, y=ty, z=tz),
                                                 rotation=NS(x=qx, y=qy, z=qz, w=qw)))


def basis():
    return LocationBasis.from_values(dict(
        robot_id='robot_a', map_frame='map', mission_frame='map', map_session_id=SESSION, map_revision=1,
        transform_revision=1, alignment_xy_yaw=[0.0, 0.0, 0.0], alignment_position_error_m=0.0,
        alignment_rotation_error_rad=0.0, clock_domain_id=DOMAIN, alignment_authority_id='tb6-saved-map-identity',
        alignment_valid_from_ns=OBSERVED - 3 * 10**9, alignment_valid_until_ns=OBSERVED + 3600 * 10**9))


class AmrTargetFormat(unittest.TestCase):
    def test_target_locator_places_it_on_the_doll(self):
        message, pose = observation()
        located = locate(message, tf_at(pose, OBSERVED), basis(), now_ns=OBSERVED + 100_000_000)
        for want, got in zip(doll_xyz(), located.position):
            self.assertAlmostEqual(want, got, places=6)

    def test_fields_are_amr_target_values_or_marked_simulation_values(self):
        message, _ = observation()
        self.assertEqual(SETTINGS['model_label'], message.class_label)
        self.assertEqual('sim-oracle', message.model_id)
        self.assertGreaterEqual(message.confidence, SETTINGS['confirm_confidence_min'])
        self.assertEqual(('DETECTION_ONLY', False, ''), (message.tracking_mode, message.has_track_id, message.track_id))
        self.assertEqual(('VALID', 'LOCATED'), (message.localization_status, message.depth_status))
        self.assertEqual(OPTICAL, message.sensor_frame)
        self.assertEqual((0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04), tuple(message.position_covariance))
        self.assertEqual(OBSERVED + SETTINGS['observation_validity_ns'], message.time.valid_until_ns)
        self.assertEqual('DRIVER_RECEIVE', message.time.stamp_semantics)
        self.assertEqual(('robot_a', SETTINGS['camera_id']), (message.source_robot, message.source_camera))
        self.assertEqual((SETTINGS['camera_id'], OBSERVED), (message.evidence.source_id, message.evidence.observed_at_ns))
        self.assertEqual(4, len(message.bbox_xyxy_pixels))

    def test_same_capture_same_id_other_capture_other_id(self):
        first, _ = observation()
        again, _ = observation()
        self.assertEqual(first.observation_id, again.observation_id)
        self.assertNotEqual(first.observation_id, first.evidence.evidence_id)

    def test_unknown_clock_would_be_refused_so_the_node_waits_for_synced(self):
        message, pose = observation(clock_state='UNKNOWN', bound=None)
        self.assertFalse(message.time.has_clock_error_bound)
        with self.assertRaises(LocationRefused) as caught:
            locate(message, tf_at(pose, OBSERVED), basis(), now_ns=OBSERVED + 100_000_000)
        self.assertEqual('CLOCK_NOT_SYNCED', caught.exception.reason)


class Settings(unittest.TestCase):
    def test_names_and_clock_from_the_field_profile(self):
        maker = _load('make_field_profile', REPO / 'scripts' / 'field' / 'make_field_profile.py')
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'profile'
            maker.write(out, maker.build(discovery='127.0.0.1:11811', domain=78))
            config = fake.settings(load_deployment(out / 'profile.json'),
                                   load_field(REPO / 'config' / 'field' / 'search_area.field.json'),
                                   camera_info_topic='/robot6/oakd/rgb/camera_info', rgb_optical_frame=OPTICAL)
        self.assertEqual(('robot_a', '/robot6', 'map'), (config['robot_id'], config['namespace'], config['map_frame']))
        self.assertEqual(('/robot6/tf', '/robot6/tf_static', '/robot6/map'),
                         (config['tf_topic'], config['tf_static_topic'], config['map_topic']))
        self.assertEqual('/rescue/map/start_pose/robot_a', config['start_pose_topic'])
        self.assertEqual('/rescue/observations', config['observations_topic'])
        self.assertEqual(DOMAIN, config['clock_domain_id'])
        self.assertEqual((DOMAIN, 150_000_000), (config['sensor_host_clock_domain_id'],
                                                 config['sensor_host_clock_error_bound_ns']))

    def test_cli_needs_every_value(self):
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            fake.parse_args(['--profile', 'p.json', '--field', 'f.json', '--doll', '1.2', '2.87'])
        args = fake.parse_args(['--profile', 'p.json', '--field', 'f.json', '--doll', '1.2', '2.87',
                                '--scout-start', '0.88', '0.37', '-1.5708', '--camera-info-topic', '/c',
                                '--rgb-optical-frame', OPTICAL])
        self.assertEqual([0.88, 0.37, -1.5708], args.scout_start)


if __name__ == '__main__':
    unittest.main()
