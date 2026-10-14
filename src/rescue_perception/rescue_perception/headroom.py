"""TB-5 overhead clearance: repeat-judgement record for the field (OPS-115, 2026-10-13).

The stop is Nav2 collision_monitor with a depth point-cloud source (통합-로봇). This tool measures
what that source will see: from each depth image, points in a box ahead of TB-5 between the floor
noise and the clearance height are counted the way collision_monitor counts them (min_points), and
the lowest point height is kept. Run at one spot (e.g. 10 times) it shows whether the crossbar stays
above the clearance under depth noise. Missing, stale or unusable depth is UNKNOWN, never PASS, and so
is a box with fewer than min_points points above the floor (the crossbar was not seen).

    ros2 run rescue_perception headroom_record --ros-args -p depth_topic:=... -p camera_info_topic:=... \
        -p base_frame:=base_link -p samples:=10 -p max_depth_age_s:=0.5 -p output_path:=<file.json> \
        -p clearance_m:=0.46 -p floor_noise_m:=0.05 -p box_x_min_m:=0.3 -p box_x_max_m:=1.2 \
        -p box_half_width_m:=0.2 -p min_points:=3 -p stride:=4 -r /tf:=/robot5/tf -r /tf_static:=/robot5/tf_static
"""
from dataclasses import dataclass
import json
import math
import sys
import time


@dataclass(frozen=True)
class HeadroomPolicy:
    clearance_m: float        # owner decision 2026-10-13: 46 cm
    floor_noise_m: float      # points at or below this are floor (proposal 0.05 m)
    box_x_min_m: float        # box ahead of base_link (the camera cannot see closer than about 0.4 m up)
    box_x_max_m: float
    box_half_width_m: float
    min_points: int           # as collision_monitor min_points

    def __post_init__(self):
        for name in ('clearance_m', 'floor_noise_m', 'box_x_min_m', 'box_x_max_m', 'box_half_width_m'):
            value = getattr(self, name)
            if type(value) is not float or not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be a finite float >= 0')
        if not 0 < self.clearance_m or self.floor_noise_m >= self.clearance_m:
            raise ValueError('floor_noise_m must be below clearance_m (> 0)')
        if self.box_x_max_m <= self.box_x_min_m or self.box_half_width_m <= 0:
            raise ValueError('box must have positive size')
        if type(self.min_points) is not int or self.min_points < 1:
            raise ValueError('min_points must be an integer >= 1')


def judge(points, policy):
    """{decision PASS|STOP|UNKNOWN, points_below, points_seen, lowest_m} for base_link points (x, y, z) or None.

    PASS needs at least min_points points above the floor in the box: an empty box, floor points only or a few
    stray points did not see the crossbar, so they are UNKNOWN (robot6 2026-10-13 gave PASS with lowest None)."""
    if points is None:
        return {'decision': 'UNKNOWN', 'points_below': None, 'points_seen': None, 'lowest_m': None}
    inside = [z for x, y, z in points if policy.box_x_min_m <= x <= policy.box_x_max_m
              and abs(y) <= policy.box_half_width_m and z > policy.floor_noise_m]
    below = sum(1 for z in inside if z < policy.clearance_m)
    decision = 'STOP' if below >= policy.min_points else 'PASS' if len(inside) >= policy.min_points else 'UNKNOWN'
    return {'decision': decision, 'points_below': below, 'points_seen': len(inside),
            'lowest_m': round(min(inside), 4) if inside else None}


def _rotation(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def depth_points(rows, encoding, *, fx, fy, cx, cy, optical_to_base, stride):
    """Depth image (16UC1 mm or 32FC1 m) -> base_link points, every ``stride`` pixel; invalid pixels dropped."""
    import numpy as np
    depth = np.asarray(rows, dtype=np.float64)[::stride, ::stride] * (0.001 if encoding == '16UC1' else 1.0)
    v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]] * stride
    valid = np.isfinite(depth) & (depth > 0)
    z = depth[valid]
    optical = np.stack([(u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z], axis=1)
    translation, quaternion = optical_to_base
    base = optical @ np.array(_rotation(quaternion)).T + np.array(translation)
    return [tuple(float(c) for c in p) for p in base]


def summary(runs):
    counts = {}
    for run in runs:
        counts[run['decision']] = counts.get(run['decision'], 0) + 1
    lowest = [r['lowest_m'] for r in runs if r['lowest_m'] is not None]
    return {'counts': counts, 'lowest_min_m': min(lowest) if lowest else None,
            'lowest_max_m': max(lowest) if lowest else None}


CLEARANCE_SOURCE = 'owner decision 2026-10-13 (OPS-113): 46 cm; floor_noise 0.05 m is a proposal'

PARAMS = {  # name -> rcl_interfaces ParameterType name; every one is required, none has a default
    'depth_topic': 'STRING', 'camera_info_topic': 'STRING', 'base_frame': 'STRING', 'output_path': 'STRING',
    'samples': 'INTEGER', 'stride': 'INTEGER', 'max_depth_age_s': 'DOUBLE',
    'clearance_m': 'DOUBLE', 'floor_noise_m': 'DOUBLE', 'box_x_min_m': 'DOUBLE', 'box_x_max_m': 'DOUBLE',
    'box_half_width_m': 'DOUBLE', 'min_points': 'INTEGER',
}


def main(args=None):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener
    from .camera_utils import git_sha, read_field_parameters

    rclpy.init(args=args)
    node = rclpy.create_node('headroom_record')
    log = node.get_logger()
    try:
        values = read_field_parameters(node, PARAMS)
        missing = sorted(name for name, value in values.items() if value is None or value == '')
        if missing:
            log.fatal(f'missing parameters: {missing}')
            return 2
        try:
            policy = HeadroomPolicy(**{k: values[k] for k in ('clearance_m', 'floor_noise_m', 'box_x_min_m',
                                                              'box_x_max_m', 'box_half_width_m', 'min_points')})
            if values['samples'] < 1 or values['stride'] < 1 or not values['max_depth_age_s'] > 0:
                raise ValueError('samples, stride >= 1 and max_depth_age_s > 0 required')
        except ValueError as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        log.info(f'headroom_record git={git_sha()} parameters={json.dumps(values, sort_keys=True)}')
        bridge, buffer = CvBridge(), Buffer()
        node.tf_listener = TransformListener(buffer, node)
        state = {'info': None, 'depth': None}
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(CameraInfo, values['camera_info_topic'], lambda m: state.update(info=m), sensor)
        node.create_subscription(Image, values['depth_topic'], lambda m: state.update(depth=m), sensor)
        runs, last_stamp, skipped = [], None, 'NO_DEPTH'
        deadline = time.monotonic() + 10 * values['samples'] + 10   # never hangs: a missing topic ends as UNKNOWN
        while len(runs) < values['samples'] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            depth, info = state['depth'], state['info']
            if depth is None:
                continue
            stamp = depth.header.stamp.sec * 10**9 + depth.header.stamp.nanosec
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            age = (node.get_clock().now().nanoseconds - stamp) / 1e9
            reason = ''
            if info is None:
                result, reason = judge(None, policy), 'NO_CAMERA_INFO'
            elif age > values['max_depth_age_s']:
                result, reason = judge(None, policy), f'STALE_DEPTH age={age:.3f}s'
            else:
                try:
                    tf = buffer.lookup_transform(values['base_frame'], depth.header.frame_id, Time(nanoseconds=stamp))
                except Exception as exc:   # TF not (yet) received: not a sample; kept as the reason if it never comes
                    skipped = f'NO_TF {type(exc).__name__}: {exc}'
                    log.warning(skipped, throttle_duration_sec=2.0)
                    continue
                try:
                    t, q = tf.transform.translation, tf.transform.rotation
                    rows = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
                    points = depth_points(rows, depth.encoding, fx=info.k[0], fy=info.k[4], cx=info.k[2], cy=info.k[5],
                                          optical_to_base=((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)),
                                          stride=values['stride'])
                    result = judge(points, policy)
                    if result['decision'] == 'UNKNOWN':
                        reason = f'FEW_POINTS_ABOVE_FLOOR seen={result["points_seen"]} min_points={policy.min_points}'
                except Exception as exc:   # unusable image: says nothing about the passage
                    result, reason = judge(None, policy), f'{type(exc).__name__}: {exc}'
            run = dict(result, stamp_ns=stamp, age_s=round(age, 3), reason=reason)
            runs.append(run)
            log.info(f'run {len(runs)}/{values["samples"]}: {run}')
            time.sleep(0.2)
        while len(runs) < values['samples']:   # no usable depth within the deadline
            runs.append(dict(judge(None, policy), stamp_ns=None, age_s=None, reason=skipped))
        record = {'git': git_sha(), 'clearance_source': CLEARANCE_SOURCE, 'parameters': values, 'runs': runs, 'summary': summary(runs)}
        with open(values['output_path'], 'w') as handle:
            json.dump(record, handle, indent=1)
        log.info(f'summary {json.dumps(record["summary"])} -> {values["output_path"]}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
