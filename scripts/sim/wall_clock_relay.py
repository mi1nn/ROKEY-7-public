#!/usr/bin/env python3
"""Robot-side relay for the full simulation run (가제보_실습, 2026-10-15): the Wi-Fi between "Pi" and "PC".

The simulator (sim_robots.sh, its own ROS domain, simulation time) and our field stack (operations, SLAM, robot
launch, coordinator: field domain, wall clock) do not talk directly. This node joins the two domains, one rclpy
context each, and changes only what the real robot does differently [측정 2026-10-15, turtlebot4_simulator
2.0.2 vs the field robot files config/robots/*.v2.json]:

- time: every header stamp (and every TF stamp) leaving the simulator is replaced by the wall clock at relay time,
  because our nodes check message age against the wall clock. Messages that had the same simulation stamp get the
  same wall stamp (StampMap): depth_image_proc pairs depth and camera_info by exact stamp, and per-message wall
  stamps gave 0 pairs, no passage points and a collision_monitor stop [측정 2026-10-15]. Commands going to the simulator (cmd_vel) get the
  simulation time, because the simulated diff-drive controller judges command age on its own clock.
- frames: the simulator's odometry is <ns>/odom -> <ns>/base_link and its lidar frame is the Gazebo sensor frame;
  the field robot has odom -> base_link and rplidar_link. Those names are mapped (FRAME_MAP); a TF edge whose two
  ends become the same frame (the simulator's identity links odom -> <ns>/odom and <ns>/base_link -> base_link) is
  dropped.
- topics: the simulated OAK-D publishes only oakd/rgb/preview/* (raw RGB and float depth in metres); the field
  robot publishes oakd/rgb/camera_info (#342 readiness), oakd/rgb/image_raw/compressed (JPEG) and
  oakd/stereo/image_raw/compressedDepth (16UC1 millimetres in PNG after a 12-byte header, as
  rescue_perception.target_utils.decode_depth_png reads it; robot readiness and passage points wait for it).
  TOPICS lists every relayed topic with its field name and the conversion, if any.
- depth noise: the relayed depth gets gaussian noise sigma = k z^2 (--depth-noise k, default DEPTH_NOISE_PER_M2), as a
  stereo camera's error grows with the distance squared. Gazebo's own <camera><noise> did not reach the rgbd depth
  image (measured std 0.0000 m, 2026-10-16). Without noise the 1 cm margin of tunnel 1 (0.47 m against the 0.46 m
  clearance) always passed in the simulation (작전 2026-10-16).

Nothing else is added or computed (apart from the depth noise): no Gazebo ground truth, no map. Undock/dock are not relayed; the robot
supervisor calls them in the simulation domain as on the real robot (runbook step 4).

    wall_clock_relay.py --sim-domain 79 --field-domain 78 --namespace robot6 [--namespace robot5]
"""
import argparse
from dataclasses import dataclass
import struct
import sys
import threading
import time

# Field name -> what the simulator calls it. Frames not listed pass unchanged.
FRAME_MAP = {
    '{ns}/odom': 'odom',
    '{ns}/base_link': 'base_link',
    '{ns}/turtlebot4/rplidar_link/rplidar': 'rplidar_link',
}


@dataclass(frozen=True)
class Topic:
    sim: str          # under the robot namespace, as the simulator publishes it
    field: str        # under the robot namespace, as the field robot publishes it
    type: str         # module.Class
    qos: str          # 'sensor' | 'reliable' | 'latched'
    to_sim: bool = False   # True: field -> simulator (commands)
    convert: str = ''      # '' (same type) | 'jpeg' (Image rgb8 -> CompressedImage) | 'depth_png' (32FC1 m -> 16UC1 mm)
    max_hz: float = 0.0    # 0 = every message; else at most this rate (image conversion is the relay's CPU cost)


TOPICS = (
    Topic('tf', 'tf', 'tf2_msgs.msg.TFMessage', 'reliable'),
    Topic('tf_static', 'tf_static', 'tf2_msgs.msg.TFMessage', 'latched'),
    Topic('odom', 'odom', 'nav_msgs.msg.Odometry', 'sensor'),
    Topic('scan', 'scan', 'sensor_msgs.msg.LaserScan', 'sensor'),
    Topic('imu', 'imu', 'sensor_msgs.msg.Imu', 'sensor'),
    Topic('wheel_status', 'wheel_status', 'irobot_create_msgs.msg.WheelStatus', 'sensor'),
    Topic('dock_status', 'dock_status', 'irobot_create_msgs.msg.DockStatus', 'sensor'),
    Topic('battery_state', 'battery_state', 'sensor_msgs.msg.BatteryState', 'sensor'),
    Topic('hazard_detection', 'hazard_detection', 'irobot_create_msgs.msg.HazardDetectionVector', 'sensor'),
    Topic('joint_states', 'joint_states', 'sensor_msgs.msg.JointState', 'sensor'),
    Topic('robot_description', 'robot_description', 'std_msgs.msg.String', 'latched'),
    Topic('oakd/rgb/preview/camera_info', 'oakd/rgb/camera_info', 'sensor_msgs.msg.CameraInfo', 'sensor'),
    Topic('oakd/rgb/preview/image_raw', 'oakd/rgb/image_raw/compressed', 'sensor_msgs.msg.Image', 'sensor',
          convert='jpeg', max_hz=5.0),
    Topic('oakd/rgb/preview/camera_info', 'oakd/stereo/camera_info', 'sensor_msgs.msg.CameraInfo', 'sensor'),
    Topic('oakd/rgb/preview/depth', 'oakd/stereo/image_raw/compressedDepth', 'sensor_msgs.msg.Image', 'sensor',
          convert='depth_png', max_hz=10.0),
    Topic('cmd_vel', 'cmd_vel', 'geometry_msgs.msg.TwistStamped', 'reliable', to_sim=True),
)


# ---- pure parts (tests/test_sim_wall_clock_relay.py, no ROS) ------------------------------------------------

def frame_map(ns):
    """{simulator frame: field frame} for this robot namespace (no leading slash)."""
    if not ns or ns.startswith('/') or '/' in ns:
        raise ValueError(f'namespace {ns!r}: a plain name such as robot6')
    return {sim.format(ns=ns): field for sim, field in FRAME_MAP.items()}


def field_frame(frames, frame):
    return frames.get(frame.lstrip('/'), frame)


def stamp_of(ns_time):
    """(sec, nanosec) of an integer nanosecond time."""
    if type(ns_time) is not int or ns_time < 0:
        raise ValueError('time must be a non-negative integer in nanoseconds')
    return ns_time // 1_000_000_000, ns_time % 1_000_000_000


class StampMap:
    """Simulation stamp -> wall stamp, the same wall stamp for the same simulation stamp (last `size` kept)."""

    def __init__(self, now_ns, size=4096):
        self._now, self._size, self._map = now_ns, size, {}

    def __call__(self, sec, nanosec):
        key = (sec, nanosec)
        wall = self._map.get(key)
        if wall is None:
            wall = stamp_of(self._now())
            self._map[key] = wall
            if len(self._map) > self._size:
                del self._map[next(iter(self._map))]
        return wall


def restamp(header, stamps, frames=None):
    header.stamp.sec, header.stamp.nanosec = stamps(header.stamp.sec, header.stamp.nanosec)
    if frames is not None:
        header.frame_id = field_frame(frames, header.frame_id)


def relay_transforms(transforms, frames, stamps):
    """Field copies of TF edges: renamed, restamped; an edge whose ends become one frame is dropped."""
    out = []
    for t in transforms:
        restamp(t.header, stamps, frames)
        t.child_frame_id = field_frame(frames, t.child_frame_id)
        if t.header.frame_id != t.child_frame_id:
            out.append(t)
    return out


def relay_message(msg, frames, stamps):
    """Restamp/rename one message in place; returns it (or None when nothing is left to send).
    `stamps(sec, nanosec) -> (sec, nanosec)` gives the wall stamp for a simulation stamp (StampMap)."""
    if hasattr(msg, 'transforms'):
        msg.transforms = relay_transforms(msg.transforms, frames, stamps)
        return msg if msg.transforms else None
    if hasattr(msg, 'header'):
        restamp(msg.header, stamps, frames)
    if hasattr(msg, 'child_frame_id'):
        msg.child_frame_id = field_frame(frames, msg.child_frame_id)
    return msg


DEPTH_HEADER = struct.Struct('<iff')   # mode 0, two unused floats (decode_depth_png reads mode 0 only)
DEPTH_FORMAT = '16UC1; compressedDepth png'
JPEG_QUALITY = 80   # [제안] 2026-10-15, preview 크기라 화질보다 크기
DEPTH_NOISE_PER_M2 = 0.01   # sigma = k z^2: 1 cm at 1 m, 4 cm at 2 m [추정 2026-10-16; OAK-D Pro, 7.5 cm baseline, 96 disparities]


def depth_mm(metres):
    """float32 metres (NaN/inf/<=0 = no return) -> uint16 millimetres, 0 = no return, capped at 65535."""
    import numpy as np
    depth = np.asarray(metres, dtype=np.float32)
    mm = np.where(np.isfinite(depth) & (depth > 0), np.rint(depth * 1000.0), 0)
    return np.clip(mm, 0, 65535).astype(np.uint16)


def noisy_depth(metres, k, rng):
    """float32 metres with gaussian noise of sigma k * z^2 on the returns; no return (NaN/inf/<=0) stays as it is."""
    import numpy as np
    depth = np.asarray(metres, dtype=np.float32)
    if k <= 0:
        return depth
    valid = np.isfinite(depth) & (depth > 0)
    out = depth.copy()
    z = depth[valid].astype(np.float64)
    out[valid] = (z + rng.normal(0.0, 1.0, z.shape) * k * z * z).astype(np.float32)
    return out


def depth_payload(metres):
    """compressedDepth bytes for a float depth image in metres."""
    import cv2
    ok, png = cv2.imencode('.png', depth_mm(metres))
    if not ok:
        raise ValueError('PNG encoding failed')
    return DEPTH_HEADER.pack(0, 0.0, 0.0) + png.tobytes()


def _image_array(msg):
    import numpy as np
    if msg.encoding == '32FC1':
        dtype, channels = np.float32, 1
    elif msg.encoding in ('rgb8', 'bgr8'):
        dtype, channels = np.uint8, 3
    else:
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    row = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    width_bytes = msg.width * channels * np.dtype(dtype).itemsize
    return np.ascontiguousarray(row[:, :width_bytes]).view(dtype).reshape(msg.height, msg.width, channels).squeeze()


def converted(msg, kind, compressed_type, depth_noise=0.0, rng=None):
    """CompressedImage for an Image (header kept, already restamped/renamed); depth gets noise sigma depth_noise z^2."""
    import cv2
    out = compressed_type()
    out.header = msg.header
    image = _image_array(msg)
    if kind == 'depth_png':
        if depth_noise > 0:
            image = noisy_depth(image, depth_noise, rng)
        out.format, out.data = DEPTH_FORMAT, depth_payload(image)
    elif kind == 'jpeg':
        if msg.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise ValueError('JPEG encoding failed')
        out.format, out.data = 'bgr8; jpeg compressed bgr8', jpg.tobytes()
    else:
        raise ValueError(f'unknown conversion {kind!r}')
    return out


# ---- ROS boundary -------------------------------------------------------------------------------------------

def _qos(kind):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    if kind == 'sensor':
        return qos_profile_sensor_data
    if kind == 'latched':
        return QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    return QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


def _type(path):
    import importlib
    module, _, name = path.rpartition('.')
    return getattr(importlib.import_module(module), name)


def main(argv=None):
    parser = argparse.ArgumentParser(description='시뮬레이션 도메인 <-> 현장 도메인 중계(시각·프레임·토픽 이름)')
    parser.add_argument('--sim-domain', type=int, required=True)
    parser.add_argument('--field-domain', type=int, required=True)
    parser.add_argument('--namespace', action='append', required=True, help='로봇 namespace, 슬래시 없이 (여러 번)')
    parser.add_argument('--depth-noise', type=float, default=DEPTH_NOISE_PER_M2, metavar='K',
                        help=f'깊이 잡음 sigma = K z^2 (m), 0 이면 없음 (기본 {DEPTH_NOISE_PER_M2} [추정])')
    args = parser.parse_args(argv)
    if not args.depth_noise >= 0:
        parser.error('--depth-noise must be >= 0')
    if args.sim_domain == args.field_domain:
        parser.error('sim and field domains must differ')

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.serialization import deserialize_message

    sim_ctx, field_ctx = rclpy.Context(), rclpy.Context()
    rclpy.init(context=sim_ctx, domain_id=args.sim_domain)
    rclpy.init(context=field_ctx, domain_id=args.field_domain)
    sim_node = rclpy.create_node('wall_clock_relay_sim', context=sim_ctx,
                                 parameter_overrides=[Parameter('use_sim_time', value=True)])
    field_node = rclpy.create_node('wall_clock_relay_field', context=field_ctx)
    log = field_node.get_logger()
    counts = {}
    keep = []
    stamps = StampMap(time.time_ns)   # one map for every robot and topic: same simulation stamp = same wall stamp
    import numpy as np
    noise_rng = np.random.default_rng()
    log.info(f'RELAY depth noise sigma = {args.depth_noise} z^2 m')

    for ns in args.namespace:
        frames = frame_map(ns)
        for topic in TOPICS:
            msg_type, qos = _type(topic.type), _qos(topic.qos)
            sim_name, field_name = f'/{ns}/{topic.sim}', f'/{ns}/{topic.field}'
            if topic.to_sim:
                pub = sim_node.create_publisher(msg_type, sim_name, qos)

                def to_sim(msg, pub=pub, key=field_name):
                    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp_of(sim_node.get_clock().now().nanoseconds)
                    pub.publish(msg)
                    counts[key] = counts.get(key, 0) + 1
                keep.append(field_node.create_subscription(msg_type, field_name, to_sim, qos))
            else:
                out_type = _type('sensor_msgs.msg.CompressedImage') if topic.convert else msg_type
                pub = field_node.create_publisher(out_type, field_name, qos)

                gap_ns = int(1e9 / topic.max_hz) if topic.max_hz else 0
                last = {'t': 0}

                def to_field(msg, pub=pub, frames=frames, key=sim_name, kind=topic.convert, out_type=out_type,
                             gap_ns=gap_ns, last=last, msg_type=msg_type):
                    now_ns = time.time_ns()
                    if gap_ns and now_ns - last['t'] < gap_ns:
                        return
                    last['t'] = now_ns
                    if isinstance(msg, bytes):   # raw subscription: decode only what is sent
                        msg = deserialize_message(msg, msg_type)
                    out = relay_message(msg, frames, stamps)
                    if out is not None and kind:
                        try:
                            out = converted(out, kind, out_type, args.depth_noise, noise_rng)
                        except ValueError as exc:
                            log.error(f'RELAY_CONVERT_FAILED {key}: {exc}')
                            return
                    if out is not None:
                        pub.publish(out)
                        counts[key] = counts.get(key, 0) + 1
                keep.append(sim_node.create_subscription(msg_type, sim_name, to_field, qos, raw=bool(gap_ns)))
        log.info(f'RELAY {ns}: sim domain {args.sim_domain} -> field domain {args.field_domain}, '
                 f'{len(TOPICS)} topics, frames {frames}')

    def report():
        log.info('RELAY_COUNTS ' + ' '.join(f'{k}={v}' for k, v in sorted(counts.items())))
    field_node.create_timer(30.0, report)

    executors = []
    for ctx, node in ((sim_ctx, sim_node), (field_ctx, field_node)):
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        executors.append(ex)
    threads = [threading.Thread(target=ex.spin, daemon=True) for ex in executors]
    for t in threads:
        t.start()
    try:
        while all(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for ex in executors:
            ex.shutdown()
        for ctx in (sim_ctx, field_ctx):
            if ctx.ok():
                rclpy.shutdown(context=ctx)
    return 0


if __name__ == '__main__':
    sys.exit(main())
