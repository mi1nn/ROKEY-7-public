"""로봇 기본 토픽 직접 관측 -> 관제 웹이 읽는 파일(--robot-watch).

로봇 PC 의 robot.launch.py(/rescue/robots/...)가 없어도 관제 웹 로봇 칸에 로봇이 보이게 한다(재범 2026-10-13).
구독만 한다: 발행자·서비스·액션이 없어 로봇에 아무것도 보내지 않는다(재범 허용 범위). 받는 토픽은 가벼운 것만:
``<ns>/battery_state``, ``<ns>/dock_status``, ``<ns>/tf``, ``<ns>/tf_static``. 카메라·라이다는 받지 않는다.

1초마다 로봇별 마지막 수신 시각(이 호스트 시계), 배터리, 도킹, ``map_frame -> base_frame`` 위치(tf 에 있을 때만)를
``out_path`` 에 임시 파일 + rename 으로 쓴다. 위치가 없으면 None 이고 지어내지 않는다(TB-5 지도 전 「위치 미확정」).
파라미터는 모두 필수이고 기본값이 없다(AGENTS 8절). ROS 패키지는 main() 안에서 import 한다.
"""
import json
import math
import os
from pathlib import Path

from .map_snapshot import yaw_from_quaternion

WATCH_SCHEMA = 'robot_watch/1'
WRITE_PERIOD_S = 1.0
PARAMS = {'robot_ids': 'STRING_ARRAY', 'namespaces': 'STRING_ARRAY', 'out_path': 'STRING', 'map_frame': 'STRING',
          'base_frame': 'STRING'}


def battery_percent(fraction):
    """BatteryState.percentage(0..1) -> %. NaN(모름)·범위 밖은 None."""
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        return None
    return round(fraction * 100.0, 1)


def pose_from_transform(x, y, rotation, frame_id):
    return {'x': float(x), 'y': float(y), 'yaw': yaw_from_quaternion(*rotation), 'frame_id': frame_id}


def write_watch(path, *, written_at_ns, robots):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps({'schema': WATCH_SCHEMA, 'written_at_ns': written_at_ns, 'robots': robots},
                              ensure_ascii=False), encoding='utf-8')
    os.replace(tmp, path)


def main(args=None):
    import rclpy
    from irobot_create_msgs.msg import DockStatus
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import BatteryState
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException

    from .ros_node import git_sha

    rclpy.init(args=args)
    node = Node('rescue_robot_watch')
    log = node.get_logger()
    values = {}
    for name, kind in PARAMS.items():
        node.declare_parameter(name, getattr(Parameter.Type, kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, value in values.items() if not value)
    if missing or len(values['robot_ids']) != len(values['namespaces']):
        log.fatal(f'missing parameters: {missing} (robot_ids and namespaces must pair up)')
        raise SystemExit(2)
    out_path = Path(values['out_path'])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    map_frame, base_frame = values['map_frame'], values['base_frame']
    log.info(f'rescue_robot_watch git={git_sha()} robots={dict(zip(values["robot_ids"], values["namespaces"]))} '
             f'tf={map_frame}->{base_frame} out={out_path} (subscribe only)')

    tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state, buffers = {}, {}
    for robot_id, ns in zip(values['robot_ids'], values['namespaces']):
        row = state[robot_id] = {'namespace': ns, 'last_seen_ns': None, 'battery_percent': None, 'docked': None,
                                 'pose': None, 'pose_seen_ns': None, '_stamp': None}
        buffer = buffers[robot_id] = Buffer(node=node)

        def seen(row=row):
            row['last_seen_ns'] = node.get_clock().now().nanoseconds

        def battery(msg, row=row):
            row['battery_percent'] = battery_percent(msg.percentage)
            seen(row)

        def dock(msg, row=row):
            row['docked'] = bool(msg.is_docked)
            seen(row)

        def tf(msg, row=row, buffer=buffer, static=False):
            for transform in msg.transforms:
                (buffer.set_transform_static if static else buffer.set_transform)(transform, 'robot_watch')
            seen(row)

        node.create_subscription(BatteryState, f'{ns}/battery_state', battery, qos_profile_sensor_data)
        node.create_subscription(DockStatus, f'{ns}/dock_status', dock, qos_profile_sensor_data)
        node.create_subscription(TFMessage, f'{ns}/tf', tf, tf_qos)
        node.create_subscription(TFMessage, f'{ns}/tf_static', lambda m, f=tf: f(m, static=True), static_qos)

    def write():
        now = node.get_clock().now().nanoseconds
        for robot_id, row in state.items():
            try:
                t = buffers[robot_id].lookup_transform(map_frame, base_frame, Time())
            except TransformException:
                continue   # 지도 위치 없음: 마지막 위치는 pose_seen_ns 로 낡음 판정
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            if stamp != row['_stamp']:
                r, p = t.transform.rotation, t.transform.translation
                row['pose'] = pose_from_transform(p.x, p.y, (r.x, r.y, r.z, r.w), map_frame)
                row['pose_seen_ns'], row['_stamp'] = now, stamp
        try:
            write_watch(out_path, written_at_ns=now,
                        robots={rid: {k: v for k, v in row.items() if not k.startswith('_')}
                                for rid, row in state.items()})
        except OSError as exc:
            log.warning(f'robot watch not written: {exc}')

    node.create_timer(WRITE_PERIOD_S, write)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
