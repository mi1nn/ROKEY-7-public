"""SLAM 지도(OccupancyGrid) -> 관제 웹이 읽는 map_server 파일(--live-map).

TB-6 SLAM 지도를 관제 웹 큰 지도 칸에 띄운다(재범 결정 2026-10-13). 받은 지도마다
``map_<해시12>.pgm`` 과 ``live_map.yaml`` 을 쓴다. 그림을 먼저 쓰고 YAML 을 마지막에 바꾸며,
둘 다 임시 파일 + rename 이라 읽는 쪽은 반쯤 쓴 지도를 보지 않는다. YAML 의 수정 시각이
「받은 시각」이다. 지금 그림과 직전 그림만 남긴다.

고정 뒤 TB-6 은 고정 지도를 같은 토픽에 한 번 다시 내고(TRANSIENT_LOCAL, OPS-117/118) 그 뒤로는
새 메시지가 없다. 그래서 토픽에 발행자가 있는 동안 ``live_map.alive`` 를 주기적으로 갱신하고,
관제 웹은 받은 시각과 이 파일 중 늦은 쪽으로 낡음을 판단한다(발행자가 사라지면 낡음).

값은 nav2 map_saver trinary 와 같다(점유 >= 65 -> 0, 빈칸 <= 25 -> 254, 나머지·미지 -> 205).
파라미터는 모두 필수이고 기본값이 없다(AGENTS 8절). ROS 패키지는 main() 안에서 import 한다.
QoS: SLAM 지도 발행에 맞춰 RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1 (실물 값은 현장 확인).
"""
import hashlib
import math
import os
from pathlib import Path
import re

LIVE_YAML = 'live_map.yaml'
ALIVE_FILE = 'live_map.alive'                    # refreshed while the map topic has a publisher
ALIVE_PERIOD_S = 2.0                             # well under any console --live-map-stale-s
OCCUPIED_PERCENT, FREE_PERCENT = 65, 25          # nav2 map_saver defaults (0.65 / 0.25)
OCCUPIED, FREE, UNKNOWN = 0, 254, 205             # trinary PGM values
YAW_TOLERANCE = 1e-6                              # the console draws only yaw-0 maps (mapimage.load_map)
PARAMS = {'map_topic': 'STRING', 'out_dir': 'STRING'}


def grid_to_pgm(width, height, data):
    """OccupancyGrid cells (row 0 = bottom) -> binary PGM bytes (row 0 = top)."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError(f'grid {width}x{height} has {len(data)} cells')

    def value(cell):
        if cell < 0:
            return UNKNOWN
        return OCCUPIED if cell >= OCCUPIED_PERCENT else FREE if cell <= FREE_PERCENT else UNKNOWN

    rows = (bytes(value(cell) for cell in data[row * width:(row + 1) * width]) for row in reversed(range(height)))
    return f'P5\n{width} {height}\n255\n'.encode() + b''.join(rows)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _replace(path, data):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


def _image_of(yaml_path):
    try:
        match = re.search(r'^image:\s*(\S+)', yaml_path.read_text(encoding='utf-8'), re.M)
    except OSError:
        return None
    return match.group(1) if match else None


def touch_alive(out_dir):
    path = Path(out_dir) / ALIVE_FILE
    path.touch()
    os.utime(path)


def write_snapshot(out_dir, *, width, height, data, resolution, origin_x, origin_y, orientation):
    """Write one received map; returns the PGM sha256. Raises ValueError for a map the console cannot draw."""
    yaw = yaw_from_quaternion(*orientation)
    if abs(yaw) > YAW_TOLERANCE:
        raise ValueError(f'map origin yaw {yaw:.6f} is not 0: the console draws only unrotated maps')
    if not (math.isfinite(resolution) and resolution > 0):
        raise ValueError(f'map resolution {resolution} must be positive')
    out_dir = Path(out_dir)
    pgm = grid_to_pgm(width, height, data)
    sha = hashlib.sha256(pgm).hexdigest()
    image = f'map_{sha[:12]}.pgm'
    yaml_path = out_dir / LIVE_YAML
    previous = _image_of(yaml_path)
    if not (out_dir / image).exists():
        _replace(out_dir / image, pgm)
    _replace(yaml_path, (f'image: {image}\nmode: trinary\nresolution: {float(resolution)!r}\n'
                         f'origin: [{float(origin_x)!r}, {float(origin_y)!r}, 0.0]\nnegate: 0\n'
                         f'occupied_thresh: {OCCUPIED_PERCENT / 100}\nfree_thresh: {FREE_PERCENT / 100}\n').encode())
    for old in out_dir.glob('map_*.pgm'):
        if old.name not in (image, previous):
            old.unlink(missing_ok=True)
    return sha


def main(args=None):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from .ros_node import git_sha

    rclpy.init(args=args)
    node = Node('rescue_map_snapshot')
    log = node.get_logger()
    values = {}
    for name, kind in PARAMS.items():
        node.declare_parameter(name, getattr(Parameter.Type, kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, value in values.items() if not value)
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    out_dir = Path(values['out_dir'])
    out_dir.mkdir(parents=True, exist_ok=True)
    log.info(f'rescue_map_snapshot git={git_sha()} map_topic={values["map_topic"]} '
             f'out={out_dir / LIVE_YAML} console=--live-map {out_dir / LIVE_YAML}')
    last = [None]

    def received(msg):
        o = msg.info.origin
        try:
            sha = write_snapshot(out_dir, width=msg.info.width, height=msg.info.height, data=msg.data,
                                 resolution=msg.info.resolution, origin_x=o.position.x, origin_y=o.position.y,
                                 orientation=(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w))
        except (OSError, ValueError) as exc:
            log.warning(f'map not written: {exc}')
            return
        if sha != last[0]:
            log.info(f'map {msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f} '
                     f'frame={msg.header.frame_id} sha256={sha[:12]}')
            last[0] = sha

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, values['map_topic'], received, qos)

    def alive():
        if node.count_publishers(values['map_topic']) > 0:
            try:
                touch_alive(out_dir)
            except OSError as exc:
                log.warning(f'alive file not written: {exc}')

    node.create_timer(ALIVE_PERIOD_S, alive)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
