"""로봇 카메라 칸(OPS-191, 재범 결정): 고른 로봇 하나의 판정 그린 영상 -> 관제 웹 MJPEG.

통합-인식이 로봇 PC 에서 내는 ``<ns>/rescue/passage_view/compressed``(CompressedImage jpeg, BEST_EFFORT depth 1)를
고른 로봇 것만 구독한다. 고르지 않은 로봇은 구독을 풀어 Wi-Fi 에 영상이 한 줄만 흐른다. 기본은 launch 의 default_source(SUPPORT = TB-5)이고,
관제 웹이 ``POST /select {"robot": id}`` 로 바꾼다(누구나 — 운영 명령이 아니라 보기 설정).

바꾸면 이전 로봇의 마지막 화면을 버린다(FrameHub.reset). 낡음은 mjpeg_relay 와 같다(stale_s 동안 새 장면이 없으면
스트림을 닫고 새 요청은 503). HTTP 스레드는 고르기만 하고, 구독은 노드 타이머가 바꾼다(rclpy 를 한 스레드에서).
파라미터는 모두 필수이고 기본값이 없다.
"""
import json
import re
import threading

from .mjpeg_relay import FrameHub, is_jpeg_format, make_server

PARAMS = {'sources': 'STRING_ARRAY', 'default_source': 'STRING', 'bind_host': 'STRING', 'port': 'INTEGER',
          'path': 'STRING', 'stale_s': 'DOUBLE'}
APPLY_PERIOD_S = 0.2
TOPIC = re.compile(r'(/[A-Za-z][A-Za-z0-9_]*)+')


def parse_sources(items):
    """['robot_a=/robot6/rescue/passage_view/compressed', ...] -> {robot_id: topic}, in order."""
    sources = {}
    for item in items:
        robot, sep, topic = item.partition('=')
        if not sep or not robot or robot in sources or not TOPIC.fullmatch(topic):
            raise ValueError(f'sources item {item!r}: want <robot id>=/<absolute topic>, once per robot')
        sources[robot] = topic
    if not sources:
        raise ValueError('sources must name at least one robot')
    return sources


class Selector:
    """The chosen robot. ``select`` (HTTP thread) records it; ``take_pending`` (node timer) applies it once."""

    def __init__(self, sources, default):
        if default not in sources:
            raise ValueError(f'default_source {default!r} is not one of {list(sources)}')
        self.sources, self._lock = sources, threading.Lock()
        self._robot, self._pending = default, default

    def select(self, robot):
        if robot not in self.sources:
            return False, f'unknown robot {robot!r}; one of {list(self.sources)}'
        with self._lock:
            if robot != self._robot:
                self._robot = self._pending = robot
        return True, ''

    def state(self):
        with self._lock:
            return {'robot': self._robot, 'robots': list(self.sources)}

    def take_pending(self):
        with self._lock:
            robot, self._pending = self._pending, None
        return None if robot is None else (robot, self.sources[robot])


def main(args=None):
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    from .camera_utils import git_sha
    from .shutdown import spin_until_shutdown

    rclpy.init(args=args)
    node = Node('robot_view_relay')
    log = node.get_logger()
    server = hub = None
    serving = False
    try:
        values = {}
        for name, kind in PARAMS.items():
            node.declare_parameter(name, Parameter.Type[kind])
            try:
                values[name] = node.get_parameter(name).value
            except ParameterUninitializedException:
                values[name] = None
        missing = sorted(name for name, value in values.items() if value is None or value == '' or value == [])
        if missing:
            log.fatal(f'missing parameters: {missing}')
            return 2
        try:
            selector = Selector(parse_sources(values['sources']), values['default_source'])
            hub = FrameHub(values['stale_s'])
            if not values['path'].startswith('/') or not 0 < values['port'] < 65536:
                raise ValueError('path must start with / and port must be 1..65535')
            server = make_server(hub, values['bind_host'], values['port'], values['path'],
                                 log=lambda text: log.info(text), select=selector)
        except (ValueError, OSError) as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        warned, current = set(), {'sub': None}

        def on_image(msg):
            if not is_jpeg_format(msg.format):
                if msg.format not in warned:
                    warned.add(msg.format)
                    log.warning(f'frame format {msg.format!r} is not jpeg; not relayed')
                return
            hub.put(msg.data)

        def apply():
            pending = selector.take_pending()
            if pending is None:
                return
            robot, topic = pending
            if current['sub'] is not None:
                node.destroy_subscription(current['sub'])   # 고르지 않은 로봇은 구독하지 않는다
            hub.reset()
            current['sub'] = node.create_subscription(CompressedImage, topic, on_image,
                                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            log.info(f'robot view -> {robot} ({topic})')

        apply()
        node.create_timer(APPLY_PERIOD_S, apply)
        threading.Thread(target=server.serve_forever, name='robot-view-http', daemon=True).start()
        serving = True
        log.info(f'robot_view_relay git={git_sha()} parameters={json.dumps(values, sort_keys=True)} '
                 f'url=http://{values["bind_host"]}:{server.server_address[1]}{values["path"]} (subscribe only)')
        spin_until_shutdown(lambda: rclpy.spin(node), node.context.ok, RCLError)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if hub is not None:
            hub.close()
        if server is not None:
            if serving:
                server.shutdown()
            server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
