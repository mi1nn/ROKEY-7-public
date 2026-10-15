#!/usr/bin/env python3
"""시뮬레이션 전용 인형 관측 대역(가제보_실습, 2026-10-15): amr_target 이 확정 인형에 내는 것과 같은 TargetObservation.

Gazebo 의 인형은 원기둥이라 실기 검출기(rescue_perception amr_target: TB-6 RGB → YOLO → 확정 관측)가 알아보지 못한다.
이 노드가 그 자리에 서서 /rescue/observations 에 amr_target 과 같은 형식의 확정 관측을 낸다. 그 뒤
target_locator → /rescue/target_locations → 조정기 → 지도 고정 → TB-5 출동은 실기 코드 그대로다.

아는 것은 둘뿐이다(Gazebo 정답 위치·gz 토픽은 쓰지 않는다):
- 인형 위치(경기장 좌표, --doll): 원점 = 로봇 구역 남서쪽 모서리, x 동쪽, y 북쪽, m. 평면도 (1.20, 2.87).
- TB-6 출발 자세(경기장 좌표, --scout-start): 평면도 (0.88, 0.37, -1.5708), 도크를 보고 도킹한 자세.
사전에 잰 좌표는 TB-6 출발 자세 기준이다(rescue_contracts.occupancy.compose_planar, OPS-141). 출발 자세의 지도 좌표는
map_freeze 가 낸 START_POSE(<start_pose_topic>, TB-6 SLAM 의 첫 자세, 조정기와 같은 planar 규칙)다. 그래서
인형(지도) = compose_planar(START_POSE, relative_planar(출발 자세(경기장), 인형(경기장))). 2026-10-15 시뮬레이션 회차의
START_POSE 는 (0, 0, 0) 이었다 [측정, map_freeze 로그 `START_POSE captured robot_a map=(-0.000, -0.000, 0.000)`].

보이는지는 로봇 자신의 것으로만 정한다: TB-6 TF(map → 광학 frame, <ns>/tf·<ns>/tf_static), 카메라 CameraInfo,
TB-6 SLAM 지도(<ns>/map). 거리·화각·가림 판정 값은 아래 상수다. 보이고 이 PC 시계가 SYNCED 이면 관측 하나를 낸다.

amr_target 과 같은 점: 토픽(target_utils.SETTINGS observations_topic), QoS(RELIABLE·TRANSIENT_LOCAL·depth 1),
class_label·source_camera·유효 시간·공분산(SETTINGS), 시계 칸(이 PC 측정 ClockSampler + 센서 호스트 상한,
target_utils.clock_binding 과 같은 규칙), envelope(fill_envelope), 관측 한 번만(OneShotTarget: 한 노드 실행에 확정 관측
하나, 사용자 정책 2026-10-12. 다시 내려면 노드를 다시 띄운다).
다른 점(시뮬레이션 값이라고 드러냄): model_id 'sim-oracle', model_sha256 = 이 파일의 sha256, publisher_id
'sim.fake_target', confidence 1.0(확률 아님), bbox = 인형 크기를 CameraInfo 로 투영한 상자, stamp_semantics
DRIVER_RECEIVE(관측 시각 = 중계가 다시 찍은 TF 시각), evidence sha256 = 판정 입력의 sha256. base_point 는 내지 않는다
(구독하는 곳 없음). amr_target 과 달리 시계가 SYNCED 가 아니면 내지 않고 기다린다(UNKNOWN 관측은 target_locator 가
CLOCK_NOT_SYNCED 로 거절하므로 한 번뿐인 관측을 버리게 된다).

    fake_target.py --profile <profile.json> --field <field.json> --doll 1.20 2.87 --scout-start 0.88 0.37 -1.5708 \
        --camera-info-topic /robot6/oakd/rgb/camera_info --rgb-optical-frame oakd_rgb_camera_optical_frame

현장 도메인(중계의 --field-domain)에서 띄운다. 판정 함수는 ROS 없이 시험한다(tests/test_sim_fake_target.py).
"""
import argparse
from collections import namedtuple
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from uuid import NAMESPACE_URL, uuid4, uuid5

from rescue_bringup.field_launch import START_POSE_TOPIC, load_field
from rescue_bringup.profile import load_deployment, require_map_basis
from rescue_contracts import STAMP_SEMANTICS
from rescue_contracts.occupancy import compose_planar, planar_origin, relative_planar
from rescue_mapping import RigidTransform
from rescue_perception.target_utils import SETTINGS, clock_values, covariance_matrix
from rescue_perception.webcam_clock import observation_clock

# 시뮬레이션 값 표시(시뮬레이션 담당, 2026-10-15). 검출기 값처럼 보이지 않게 이름에 드러낸다.
MODEL_ID = 'sim-oracle'
PUBLISHER_ID = 'sim.fake_target'
CONFIDENCE = 1.0                   # 확률이 아니라 「판정기가 보인다고 함」. amr_target 확정 하한 0.9 이상
STAMP = 'DRIVER_RECEIVE'           # 관측 시각 = 중계(wall_clock_relay)가 받아서 다시 찍은 TF 시각
NAMESPACE = uuid5(NAMESPACE_URL, 'ROKEY-7/sim/fake_target/v1')

# 보이는 조건 [제안] 시뮬레이션 담당 2026-10-15. 실기 값이 아니고 측정한 값도 아니다.
MAX_RANGE_M = 2.0      # 광학 원점 → 인형 중심 거리 상한(이 안에서 YOLO 가 인형을 잡는다고 본다)
MIN_DEPTH_M = 0.3      # 앞쪽 깊이 하한: 시뮬레이션 OAK-D near clip 0.3 m(turtlebot4_description oakd.urdf.xacro),
                       # amr_target min_depth_m 0.2 보다 큰 쪽
DOLL_RADIUS_M = 0.07   # 평면도 원 r 7 cm(arena.layout.json doll), bbox 투영에만 쓴다
DOLL_HEIGHT_M = 0.20   # [추정] layout 원기둥 길이. 인형 중심 높이 = 절반(지도 z 0 = 바닥, 출발 때 base_link)
OCCUPIED_MIN = 65      # 이 값 이상인 지도 칸이 시선을 막는다(map_server occupied_thresh 0.65; slam_toolbox 는 100)
TARGET_SKIP_M = 0.15   # 인형 앞 이 거리는 가림 검사에서 뺀다(인형 자신의 칸, 반지름 0.07 + 한 칸 0.05 + 여유)
CHECK_PERIOD_S = 0.5   # 판정 주기(2 Hz). 내는 것은 한 번뿐이다(amr_target OneShotTarget)
TF_MAX_AGE_NS = 500_000_000            # 가장 새 TF 가 이보다 낡으면 판정하지 않는다
CAMERA_INFO_MAX_AGE_NS = 1_500_000_000  # amr_target README 현장 값 camera_info_max_age_ns 와 같음
CLOCK_POLL_PERIOD_S = 5.0              # amr_target README 현장 값 clock_poll_period_s 와 같음

Camera = namedtuple('Camera', 'width height fx fy cx cy')
Grid = namedtuple('Grid', 'frame width height resolution origin data')   # origin (x, y, yaw), data row-major
Sight = namedtuple('Sight', 'reason detail sensor bbox distance_m camera_xy')   # reason None = 보임


# ---- 순수 부분 (tests/test_sim_fake_target.py, ROS 없음) ---------------------------------------------------

def settings(deployment, field, *, camera_info_topic, rgb_optical_frame):
    """프로파일 + 현장 파일 → 이 노드가 쓰는 이름과 시계 값. target_locator.profile_parameters 와 같은 SCOUT·frame 규칙."""
    basis = require_map_basis(deployment)
    scouts = [robot_id for robot_id, role in deployment.mission.roles if role == 'SCOUT']
    if len(scouts) != 1:
        raise ValueError(f'exactly one SCOUT robot required, got {scouts}')
    robot_id = scouts[0]
    robot = {r.id: r for r in deployment.robots}[robot_id].parameters
    if robot.readiness is None or robot.readiness.map_frame != basis.frame_id:
        raise ValueError(f'profile robot {robot_id} readiness.map_frame must be the map_basis frame {basis.frame_id!r}')
    if robot_id not in field.robots:
        raise ValueError(f'field run file has no robots.{robot_id}')
    for name, value in (('camera_info_topic', camera_info_topic), ('rgb_optical_frame', rgb_optical_frame)):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{name} must be a non-empty string')
    c, r = field.coordinator, field.robots[robot_id]
    clock = clock_values({'clock_domain_id': c['target_clock_domain_id'],   # 조정기가 받는 관측 시계 영역
                          'clock_backend': c['live_inputs.clock_backend'],
                          'clock_max_age_ns': c['live_inputs.clock_max_age_ns'],
                          'clock_drift_ppm': c['live_inputs.clock_drift_ppm'],
                          'clock_poll_period_s': CLOCK_POLL_PERIOD_S,
                          'sensor_host_clock_error_bound_ns': r['task.live.sensor_host_clock_error_bound_ns'],
                          'sensor_host_clock_domain_id': r['task.live.sensor_host_clock_domain_id']})
    ns = robot.namespace
    return dict(robot_id=robot_id, namespace=ns, map_frame=basis.frame_id, map_topic=ns + '/map',
                tf_topic=ns + '/tf', tf_static_topic=ns + '/tf_static', start_pose_topic=START_POSE_TOPIC + robot_id,
                camera_info_topic=camera_info_topic, rgb_optical_frame=rgb_optical_frame,
                observations_topic=SETTINGS['observations_topic'], camera_id=SETTINGS['camera_id'],
                class_label=SETTINGS['model_label'], validity_ns=SETTINGS['observation_validity_ns'],
                envelope_version=SETTINGS['envelope_version'], **clock)


def start_pose(frame_id, position, orientation, map_frame):
    """START_POSE 메시지 값 → 지도 (x, y, yaw). 조정기 start_pose_from_msg 와 같은 planar 규칙, 아니면 ValueError."""
    if frame_id != map_frame:
        raise ValueError(f'start pose frame {frame_id!r} is not {map_frame!r}')
    return planar_origin(*position, *orientation)


def doll_in_map(doll_arena, scout_start_arena, start_pose_map):
    """경기장 좌표 인형 (x, y) → TB-6 지도 (x, y). 출발 자세 기준 상대 좌표를 START_POSE 에 얹는다."""
    relative = relative_planar(tuple(scout_start_arena), (doll_arena[0], doll_arena[1], 0.0))
    x, y, _ = compose_planar(tuple(start_pose_map), relative)
    return x, y


def camera_model(width, height, k, frame_id, expected_frame):
    """CameraInfo 값 → Camera(왜곡 없는 핀홀; 시뮬레이션 OAK-D 는 왜곡 0). 틀리면 ValueError."""
    if frame_id != expected_frame:
        raise ValueError(f'CameraInfo frame {frame_id!r} is not {expected_frame!r}')
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if (type(width) is not int or type(height) is not int or width <= 0 or height <= 0
            or not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0):
        raise ValueError('CameraInfo needs a positive size and K')
    return Camera(width, height, fx, fy, cx, cy)


def horizontal_fov(camera):
    return 2 * math.atan(camera.width / (2 * camera.fx))


def grid_from_fields(frame, width, height, resolution, position, orientation, data):
    """OccupancyGrid 값 → Grid. 원점은 조정기·로봇과 같은 planar_origin 규칙."""
    data = tuple(int(v) for v in data)
    if (not frame or type(width) is not int or type(height) is not int or width <= 0 or height <= 0
            or not math.isfinite(resolution) or resolution <= 0 or len(data) != width * height):
        raise ValueError('invalid occupancy grid')
    return Grid(frame, width, height, float(resolution), planar_origin(*position, *orientation), data)


def blocking_cell(grid, start_xy, end_xy, *, skip_end_m=TARGET_SKIP_M, occupied_min=OCCUPIED_MIN):
    """start → end 선분(끝 skip_end_m 제외)에 걸린 첫 occupied 칸 (ix, iy, value), 없으면 None.

    resolution/4 간격으로 표본을 찍는다. 지도 밖·미지(-1) 칸은 막지 않는다(막는 근거가 없음)."""
    length = math.dist(start_xy, end_xy)
    usable = length - skip_end_m
    if usable <= 0:
        return None
    ox, oy, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    ux, uy = (end_xy[0] - start_xy[0]) / length, (end_xy[1] - start_xy[1]) / length
    step = grid.resolution / 4
    for k in range(math.ceil(usable / step) + 1):
        d = min(k * step, usable)
        x, y = start_xy[0] + ux * d, start_xy[1] + uy * d
        ix = math.floor((c * (x - ox) + s * (y - oy)) / grid.resolution)
        iy = math.floor((-s * (x - ox) + c * (y - oy)) / grid.resolution)
        if 0 <= ix < grid.width and 0 <= iy < grid.height:
            value = grid.data[iy * grid.width + ix]
            if value >= occupied_min:
                return ix, iy, value
    return None


def doll_bbox(sensor, camera):
    """광학 frame 인형 중심 → 화면 상자 (x0, y0, x1, y1) px. 인형 반지름·높이를 깊이로 나눠 투영한다(시뮬레이션 값)."""
    x, y, z = sensor
    u, v = camera.fx * x / z + camera.cx, camera.fy * y / z + camera.cy
    half_w, half_h = camera.fx * DOLL_RADIUS_M / z, camera.fy * (DOLL_HEIGHT_M / 2) / z
    return u - half_w, v - half_h, u + half_w, v + half_h


def assess(doll_map_xyz, map_from_optical, camera, grid):
    """보이면 Sight(reason=None, sensor=광학 frame 점, bbox, 거리), 아니면 이유. 순서: 앞·거리 → 화면 → 가림."""
    sensor = map_from_optical.inverse().apply(doll_map_xyz)
    cam = map_from_optical.apply((0.0, 0.0, 0.0))
    distance = math.dist(sensor, (0.0, 0.0, 0.0))
    camera_xy = (cam[0], cam[1])

    def no(reason, detail):
        return Sight(reason, detail, sensor, None, distance, camera_xy)
    if sensor[2] < MIN_DEPTH_M:
        return no('NOT_IN_FRONT', f'depth={sensor[2]:.2f} m < {MIN_DEPTH_M}')
    if distance > MAX_RANGE_M:
        return no('OUT_OF_RANGE', f'dist={distance:.2f} m > {MAX_RANGE_M}')
    bbox = doll_bbox(sensor, camera)
    x0, y0, x1, y1 = bbox
    if not (0 < x0 < x1 < camera.width and 0 < y0 < y1 < camera.height):   # amr_target BBOX_AT_IMAGE_EDGE 와 같은 조건
        bearing = math.degrees(math.atan2(sensor[0], sensor[2]))
        return no('OUT_OF_VIEW', f'bearing={bearing:.1f} deg bbox=({x0:.0f}, {y0:.0f}, {x1:.0f}, {y1:.0f}) '
                                 f'image={camera.width}x{camera.height}')
    cell = blocking_cell(grid, camera_xy, doll_map_xyz[:2])
    if cell is not None:
        return no('OCCLUDED', f'cell=({cell[0]}, {cell[1]}) value={cell[2]}')
    return Sight(None, '', sensor, bbox, distance, camera_xy)


def observation_clock_fields(record, observed_ns, *, clock_domain_id, sensor_host_bound_ns, sensor_host_domain_id):
    """(clock_state, 오차 상한 ns 또는 None): target_utils.clock_binding 과 같은 규칙(이 PC 측정 + 센서 호스트 상한)."""
    state, bound = observation_clock(record, observed_ns)
    if state != 'SYNCED' or sensor_host_domain_id != clock_domain_id:
        return 'UNKNOWN', None
    return 'SYNCED', bound + sensor_host_bound_ns


def evidence_digest(decision):
    """판정 입력(인형·카메라 위치, 광학 점, 상자, 시각)의 sha256: 이 관측의 근거가 영상이 아니라 계산이라는 표시."""
    return hashlib.sha256(json.dumps(decision, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def observation_record(*, config, boot_id, observed_at_ns, clock_state, clock_error_bound_ns, sensor, bbox,
                       model_sha256, evidence_sha256):
    """TargetObservation 칸(envelope 제외)을 dict 로. 칸 이름과 채우는 방식은 target_utils.fill_observation 과 같다."""
    if clock_state not in ('SYNCED', 'UNKNOWN') or STAMP not in STAMP_SEMANTICS:
        raise ValueError('clock_state SYNCED/UNKNOWN and a known stamp meaning required')
    return {
        'observation_id': str(uuid5(NAMESPACE, f'{boot_id}/{observed_at_ns}')),
        'source_robot': config['robot_id'], 'source_camera': config['camera_id'], 'source_boot_id': boot_id,
        'time': {'observed_at_ns': observed_at_ns, 'valid_until_ns': observed_at_ns + config['validity_ns'],
                 'stamp_semantics': STAMP, 'clock_domain_id': config['clock_domain_id'], 'clock_state': clock_state,
                 'has_clock_error_bound': clock_error_bound_ns is not None,
                 'clock_error_bound_ns': clock_error_bound_ns if clock_error_bound_ns is not None else 0},
        'class_label': config['class_label'], 'confidence': CONFIDENCE,
        'model_id': MODEL_ID, 'model_sha256': model_sha256,
        'tracking_mode': 'DETECTION_ONLY', 'has_track_id': False, 'track_id': '',
        'bbox_xyxy_pixels': [float(v) for v in bbox],
        'sensor_frame': config['rgb_optical_frame'],
        'localization_status': 'VALID',
        'sensor_position': {'x': float(sensor[0]), 'y': float(sensor[1]), 'z': float(sensor[2])},
        'position_covariance': list(covariance_matrix(SETTINGS['sensor_variance_xyz'])),
        'depth_status': 'LOCATED',
        'evidence': {'evidence_id': str(uuid5(NAMESPACE, f'{boot_id}/{observed_at_ns}/oracle')),
                     'sha256': evidence_sha256, 'mime_type': 'application/octet-stream',
                     'source_id': config['camera_id'], 'source_boot_id': boot_id, 'observed_at_ns': observed_at_ns},
    }


def fill(message, record):
    """dict → 메시지 칸(중첩 메시지는 그 칸에 다시 채운다)."""
    for key, value in record.items():
        if isinstance(value, dict):
            fill(getattr(message, key), value)
        else:
            setattr(message, key, value)
    return message


def git_sha():
    here = str(Path(__file__).resolve().parent)
    try:
        sha = subprocess.run(['git', '-C', here, 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True,
                             timeout=5).stdout.strip()
        dirty = subprocess.run(['git', '-C', here, 'status', '--porcelain'], capture_output=True, text=True,
                               check=True, timeout=5).stdout.strip()
        return sha + ('+dirty' if dirty else '')
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='시뮬레이션 전용: amr_target 확정 인형 관측 대역 (Gazebo 정답 위치 안 씀)')
    p.add_argument('--profile', required=True, help='배포 프로파일 profile.json (조정기·target_locator 와 같은 것)')
    p.add_argument('--field', required=True, help='현장 파일 config/field/search_area.field.json')
    p.add_argument('--doll', nargs=2, type=float, required=True, metavar=('X', 'Y'), help='인형 위치, 경기장 좌표 m')
    p.add_argument('--scout-start', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'),
                   help='TB-6 출발 자세, 경기장 좌표 m·rad')
    p.add_argument('--camera-info-topic', required=True, help='TB-6 RGB CameraInfo (amr_target camera_info_topic)')
    p.add_argument('--rgb-optical-frame', required=True, help='RGB 광학 frame (amr_target rgb_optical_frame)')
    args = p.parse_args(argv)
    if not all(math.isfinite(v) for v in (*args.doll, *args.scout_start)):
        p.error('--doll and --scout-start must be finite numbers')
    return args


# ---- ROS 경계 ---------------------------------------------------------------------------------------------

def main(argv=None):
    args = parse_args(argv)
    try:   # ProfileError·FieldLaunchError 도 ValueError 다
        config = settings(load_deployment(args.profile), load_field(args.field),
                          camera_info_topic=args.camera_info_topic, rgb_optical_frame=args.rgb_optical_frame)
    except (ValueError, KeyError, OSError) as exc:
        print(f'FAKE_TARGET not started: {type(exc).__name__}: {exc}', file=sys.stderr)
        return 2

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rescue_contracts import ros_time_to_ns
    from rescue_contracts.clock_measurement import ClockSampler
    from rescue_interfaces.msg import TargetObservation
    from rescue_perception.target_utils import fill_envelope
    from sensor_msgs.msg import CameraInfo
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('fake_target', cli_args=['--ros-args', '-r', f'/tf:={config["tf_topic"]}',
                                                      '-r', f'/tf_static:={config["tf_static_topic"]}'])
    log = node.get_logger()
    sampler = None
    try:
        boot_id = str(uuid4())
        model_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        relative = relative_planar(tuple(args.scout_start), (args.doll[0], args.doll[1], 0.0))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        publisher = node.create_publisher(TargetObservation, config['observations_topic'], latched)  # amr_target 와 같은 QoS
        buffer = Buffer()
        node.tf_listener = TransformListener(buffer, node)
        state = {'start': None, 'doll': None, 'grid': None, 'camera': None, 'camera_stamp': None,
                 'reason': None, 'sent': None, 'sequence': 0, 'clock': None}
        if config['clock_backend'] != 'NONE':   # NONE: 측정 안 함 → 늘 UNKNOWN → 내지 않음(amr_target 과 같은 값 해석)
            sampler = ClockSampler(config['clock_backend'], config['clock_max_age_ns'], config['clock_drift_ppm'])
        clock = {'record': dict(state='UNKNOWN', reason='waiting' if sampler else 'clock_backend NONE')}

        def poll_clock():
            if sampler is None:
                return
            clock['record'] = sampler.poll()
            shown = (clock['record'].get('state'), clock['record'].get('reason'))
            if shown != state['clock']:
                state['clock'] = shown
                log.info(f'FAKE_TARGET clock {shown[0]} backend={config["clock_backend"]} '
                         f'error_bound_ns={clock["record"].get("error_bound_ns")} reason={shown[1]}')

        def on_start(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            try:
                pose = start_pose(msg.header.frame_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), config['map_frame'])
            except ValueError as exc:
                log.error(f'FAKE_TARGET START_POSE refused: {exc}')
                return
            if state['start'] is not None and state['start'] != pose:
                log.error(f'FAKE_TARGET START_POSE changed {state["start"]} -> {pose}; keeping the first (map_freeze '
                          'publishes one per run)')
                return
            state['start'], state['doll'] = pose, doll_in_map(args.doll, args.scout_start, pose)
            log.info(f'FAKE_TARGET start_pose map=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}) '
                     f'doll map=({state["doll"][0]:.3f}, {state["doll"][1]:.3f})')

        def on_map(msg):
            if msg.header.frame_id != config['map_frame']:
                log.error(f'FAKE_TARGET map frame {msg.header.frame_id!r} is not {config["map_frame"]!r}; ignored',
                          throttle_duration_sec=10.0)
                return
            o, i = msg.info.origin, msg.info
            try:
                state['grid'] = grid_from_fields(msg.header.frame_id, i.width, i.height, i.resolution,
                                                 (o.position.x, o.position.y, o.position.z),
                                                 (o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                                                 msg.data)
            except ValueError as exc:
                log.error(f'FAKE_TARGET map ignored: {exc}', throttle_duration_sec=10.0)

        def on_camera_info(msg):
            try:
                state['camera'] = camera_model(msg.width, msg.height, msg.k, msg.header.frame_id,
                                               config['rgb_optical_frame'])
                state['camera_stamp'] = ros_time_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
            except ValueError as exc:
                log.error(f'FAKE_TARGET CameraInfo ignored: {exc}', throttle_duration_sec=10.0)

        def waiting(reason, detail=''):
            if reason != state['reason']:
                state['reason'] = reason
                log.info(f'FAKE_TARGET not sent: {reason} {detail}'.rstrip())

        def check():
            if state['sent'] is not None:
                return
            if state['doll'] is None:
                return waiting('NO_START_POSE', config['start_pose_topic'])
            if state['grid'] is None:
                return waiting('NO_MAP', config['map_topic'])
            if state['camera'] is None:
                return waiting('NO_CAMERA_INFO', config['camera_info_topic'])
            try:   # 가장 새 TF(그 시각이 관측 시각이 된다. target_locator 가 같은 시각으로 다시 찾는다)
                tf = buffer.lookup_transform(config['map_frame'], config['rgb_optical_frame'], Time())
            except Exception as exc:
                return waiting('NO_TF', f'{config["map_frame"]}<-{config["rgb_optical_frame"]}: {type(exc).__name__}')
            observed = ros_time_to_ns(tf.header.stamp.sec, tf.header.stamp.nanosec)
            now = time.time_ns()
            if abs(now - observed) > TF_MAX_AGE_NS:
                return waiting('TF_STALE', f'age_ns={now - observed}')
            if abs(observed - state['camera_stamp']) > CAMERA_INFO_MAX_AGE_NS:
                return waiting('CAMERA_INFO_STALE', f'delta_ns={observed - state["camera_stamp"]}')
            t, q = tf.transform.translation, tf.transform.rotation
            map_from_optical = RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            doll = (*state['doll'], DOLL_HEIGHT_M / 2)
            sight = assess(doll, map_from_optical, state['camera'], state['grid'])
            if sight.reason is not None:
                return waiting(sight.reason, sight.detail)
            clock_state, bound = observation_clock_fields(
                clock['record'], observed, clock_domain_id=config['clock_domain_id'],
                sensor_host_bound_ns=config['sensor_host_clock_error_bound_ns'],
                sensor_host_domain_id=config['sensor_host_clock_domain_id'])
            if clock_state != 'SYNCED':
                return waiting('CLOCK_UNKNOWN', str(clock['record'].get('reason')))
            decision = dict(model_id=MODEL_ID, observed_at_ns=observed, doll_map=list(doll),
                            camera_map=list(sight.camera_xy), sensor=list(sight.sensor), bbox=list(sight.bbox))
            record = observation_record(config=config, boot_id=boot_id, observed_at_ns=observed,
                                        clock_state=clock_state, clock_error_bound_ns=bound, sensor=sight.sensor,
                                        bbox=sight.bbox, model_sha256=model_sha256,
                                        evidence_sha256=evidence_digest(decision))
            message = fill(TargetObservation(), record)
            state['sequence'] += 1
            fill_envelope(message.envelope, version=config['envelope_version'], publisher_id=PUBLISHER_ID,
                          boot_id=boot_id, sequence=state['sequence'], now_ns=time.time_ns())
            publisher.publish(message)
            state['sent'] = message.observation_id
            s, b = sight.sensor, sight.bbox
            log.info(f'FAKE_TARGET published observation={message.observation_id} map=({doll[0]:.3f}, {doll[1]:.3f}) '
                     f'sensor=({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f}) dist={sight.distance_m:.2f} '
                     f'camera_map=({sight.camera_xy[0]:.3f}, {sight.camera_xy[1]:.3f}) '
                     f'bbox=({b[0]:.0f}, {b[1]:.0f}, {b[2]:.0f}, {b[3]:.0f}) observed_at_ns={observed} '
                     f'clock={clock_state} bound_ns={bound} model_id={MODEL_ID} (simulation stand-in, not a detection)')
            log.info('FAKE_TARGET state=SENT: one confirmed observation per run like amr_target (OneShotTarget); '
                     'restart this node to send again')

        node.create_subscription(PoseWithCovarianceStamped, config['start_pose_topic'], on_start, latched)
        node.create_subscription(OccupancyGrid, config['map_topic'], on_map, latched)
        node.create_subscription(CameraInfo, config['camera_info_topic'], on_camera_info, sensor_qos)
        poll_clock()
        node.create_timer(CLOCK_POLL_PERIOD_S, poll_clock)
        node.create_timer(CHECK_PERIOD_S, check)
        thresholds = dict(max_range_m=MAX_RANGE_M, min_depth_m=MIN_DEPTH_M, doll_radius_m=DOLL_RADIUS_M,
                          doll_height_m=DOLL_HEIGHT_M, occupied_min=OCCUPIED_MIN, target_skip_m=TARGET_SKIP_M,
                          check_period_s=CHECK_PERIOD_S, tf_max_age_ns=TF_MAX_AGE_NS,
                          camera_info_max_age_ns=CAMERA_INFO_MAX_AGE_NS)
        log.info(f'FAKE_TARGET start git={git_sha()} boot={boot_id} model_id={MODEL_ID} model_sha256={model_sha256} '
                 f'publisher_id={PUBLISHER_ID} doll_arena=({args.doll[0]:.3f}, {args.doll[1]:.3f}) '
                 f'scout_start_arena=({args.scout_start[0]:.3f}, {args.scout_start[1]:.3f}, {args.scout_start[2]:.4f}) '
                 f'doll_in_start_frame=({relative[0]:.3f}, {relative[1]:.3f}) parameters={json.dumps(config, sort_keys=True)} '
                 f'thresholds={json.dumps(thresholds, sort_keys=True)} '
                 '(SIMULATION ONLY: no Gazebo ground truth; publishes like amr_target)')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if sampler is not None:
            sampler.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
