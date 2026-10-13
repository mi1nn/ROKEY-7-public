"""Overhead structures (tunnels) ahead of a robot, marked on the map (OPS-185/186, 2026-10-13).

The 46 cm passage stop (Nav2 collision_monitor on <ns>/passage_points) only stops the robot; where
a structure is and how high it is was kept nowhere. This node reads the same depth points and
reports each overhead structure on the map:

* Points in the stop box ahead of base_link (passage.front_m, passage.half_width_m) between the
  floor noise and ``top_m``. The lowest height is the ``min_points``-th lowest point, the count
  collision_monitor needs to stop, so a few noisy points neither stop nor lower it.
* It floats only when that height is above the LiDAR plane (OPS-186 a): lower structures the LiDAR
  already sees (a wall, a box) are no tunnel.
* PASSABLE at or above the clearance (passage.max_height_m, owner decision 46 cm), BLOCKED below.
  UNKNOWN when the floor under it (same x span) was not seen: no height is claimed (OPS-186 b).
* Reported after ``confirm_frames`` agreeing processed frames at one place; a frame with another
  answer, nothing, or no usable input restarts the count. Position = the structure's floor centre
  in the map frame (TF at the cloud stamp); the map revision from the SCOUT's MapState.
* One item per place and map revision (``merge_radius_m``); UNKNOWN never replaces a known height.
  The whole list goes out as one latched JSON text (rescue_contracts.overhead, C-04b).
* Camera view (owner decision OPS-189): the RGB preview frame nearest the cloud (``view_slop_ns``),
  projected with the preview CameraInfo (the depth is aligned to the RGB camera, same optical frame;
  only the size differs; an image of another size than its CameraInfo is refused). The preview is
  the small stream each Pi already publishes, so the full RGB, the doll detector input, gets no
  second subscriber over Wi-Fi (OPS-198). The free space under the structure (its x span,
  the box width, floor to its lowest point) painted: green 「통과 가능 ↑h」 / red 「통과 불가 ↑h」 once
  that place is confirmed, grey 「판정 불가」 when the floor is not seen (one wording with the map, OPS-196),
  grey 「판정 중」 while the place still counts its agreeing frames; nothing painted
  without a structure. A small JPEG on <ns>/rescue/passage_view/compressed for the web camera panel.
  ``view_image_topic`` and ``view_camera_info_topic`` both '' = no view.

    ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> \
        field_path:=<repo>/config/field/search_area.field.json robot_id:=<robot>
"""
from collections import namedtuple
from dataclasses import dataclass
import math
import sys
import time
from statistics import median
from uuid import uuid4

# x, y in the frame of the points it came from; x0, x1 (its x span) and seen_m (lowest point seen, also for
# UNKNOWN) only for drawing the camera view
Reading = namedtuple('Reading', 'state lowest_m x y reason x0 x1 seen_m', defaults=(None, None, None))
VIEW_TOPIC = 'rescue/passage_view/compressed'   # relative: <ns>/rescue/passage_view/compressed
GREEN, RED, GREY = (0, 200, 0), (0, 0, 220), (160, 160, 160)   # BGR, the same as the web map (OPS-189)

POLICY_PARAMS = {
    'clearance_m': 'DOUBLE', 'floor_noise_m': 'DOUBLE', 'front_near_m': 'DOUBLE', 'front_far_m': 'DOUBLE',
    'half_width_m': 'DOUBLE', 'lidar_plane_m': 'DOUBLE', 'top_m': 'DOUBLE', 'min_points': 'INTEGER',
    'floor_min_points': 'INTEGER', 'confirm_frames': 'INTEGER', 'position_tolerance_m': 'DOUBLE',
    'merge_radius_m': 'DOUBLE'}
NODE_PARAMS = {
    'points_topic': 'STRING', 'base_frame': 'STRING', 'map_frame': 'STRING', 'robot_id': 'STRING',
    'map_robot_id': 'STRING', 'map_state_topic': 'STRING', 'clock_error_bound_ns': 'INTEGER',
    'process_period_s': 'DOUBLE', 'max_cloud_age_s': 'DOUBLE', 'stride': 'INTEGER',
    'view_camera_info_topic': 'STRING', 'view_image_topic': 'STRING', 'view_width': 'INTEGER', 'view_quality': 'INTEGER',
    'view_slop_ns': 'INTEGER', 'view_font_path': 'STRING'}
OVERHEAD_PARAMS = {**NODE_PARAMS, **POLICY_PARAMS}   # every one required, none has a default


def _positive(value, name, *, zero=False):
    if type(value) is not float or not math.isfinite(value) or value < 0 or (value == 0 and not zero):
        raise ValueError(f'{name} must be a finite float {">=" if zero else ">"} 0')


@dataclass(frozen=True)
class OverheadPolicy:
    clearance_m: float            # passage.max_height_m: owner decision 46 cm
    floor_noise_m: float          # passage.min_height_m: at or below is floor
    front_near_m: float           # passage.front_m: the stop box ahead of base_link
    front_far_m: float
    half_width_m: float           # passage.half_width_m
    lidar_plane_m: float          # LiDAR scan height above the floor [estimate until measured]
    top_m: float                  # higher points are not part of a passage
    min_points: int               # passage.min_points, as collision_monitor
    floor_min_points: int         # floor points under the structure before "empty below" is claimed
    confirm_frames: int           # agreeing processed frames before reporting
    position_tolerance_m: float   # spread allowed within one run (map frame)
    merge_radius_m: float         # same place as an item already reported (same map revision)

    def __post_init__(self):
        _positive(self.floor_noise_m, 'floor_noise_m', zero=True)
        for name in ('clearance_m', 'front_near_m', 'front_far_m', 'half_width_m', 'lidar_plane_m', 'top_m',
                     'position_tolerance_m', 'merge_radius_m'):
            _positive(getattr(self, name), name, zero=name == 'front_near_m')
        if not self.floor_noise_m < self.lidar_plane_m < self.clearance_m < self.top_m:
            raise ValueError('need floor_noise_m < lidar_plane_m < clearance_m < top_m')
        if self.front_far_m <= self.front_near_m:
            raise ValueError('front_far_m must be beyond front_near_m')
        for name in ('min_points', 'floor_min_points', 'confirm_frames'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f'{name} must be an integer >= 1')


def overhead_values(values):
    """Field values (no defaults) -> (OverheadPolicy, node settings) or ValueError."""
    missing = sorted(name for name in OVERHEAD_PARAMS if values.get(name) is None)
    if missing:
        raise ValueError(f'missing parameters: {missing}')
    node = {name: values[name] for name in NODE_PARAMS}
    for name, kind in NODE_PARAMS.items():
        if kind == 'STRING' and not name.startswith('view_') and (type(node[name]) is not str or not node[name].strip()):
            raise ValueError(f'{name} must be a non-empty STRING')
    view_topics = (node['view_image_topic'], node['view_camera_info_topic'])
    if any(type(t) is not str for t in view_topics) or bool(view_topics[0].strip()) != bool(view_topics[1].strip()):
        raise ValueError("view_image_topic and view_camera_info_topic: both topics, or both '' (no camera view)")
    if type(node['view_font_path']) is not str or not node['view_font_path'].strip():
        raise ValueError('view_font_path must be a non-empty STRING')
    if type(node['view_width']) is not int or node['view_width'] < 16:
        raise ValueError('view_width must be an integer >= 16')
    if type(node['view_quality']) is not int or not 1 <= node['view_quality'] <= 100:
        raise ValueError('view_quality must be 1..100')
    if type(node['view_slop_ns']) is not int or node['view_slop_ns'] < 0:
        raise ValueError('view_slop_ns must be an integer >= 0')
    if type(node['clock_error_bound_ns']) is not int or node['clock_error_bound_ns'] < 0:
        raise ValueError('clock_error_bound_ns must be an integer >= 0')
    if type(node['stride']) is not int or node['stride'] < 1:
        raise ValueError('stride must be an integer >= 1')
    _positive(node['process_period_s'], 'process_period_s')
    _positive(node['max_cloud_age_s'], 'max_cloud_age_s')
    return OverheadPolicy(**{name: values[name] for name in POLICY_PARAMS}), node


def cloud_xyz(msg, stride):
    """(N, 3) finite x, y, z of a PointCloud2 (little-endian float32 x/y/z), every ``stride`` row and column."""
    import numpy as np
    fields = {f.name: f for f in msg.fields}
    if msg.is_bigendian or any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in 'xyz'):
        raise ValueError('PointCloud2 needs little-endian FLOAT32 x, y, z')
    if msg.row_step != msg.width * msg.point_step or len(msg.data) != msg.height * msg.row_step:
        raise ValueError('PointCloud2 rows must be packed (row_step = width * point_step)')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [fields[n].offset for n in 'xyz'], 'itemsize': msg.point_step})
    # no copy: rclpy gives the data as array.array, which shares its buffer
    grid = np.frombuffer(msg.data, dtype=dtype, count=msg.height * msg.width).reshape(
        msg.height, msg.width)[::stride, ::stride]
    points = np.stack([grid['x'].ravel(), grid['y'].ravel(), grid['z'].ravel()], axis=1).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def to_frame(points, translation, quaternion):
    """Points expressed in the parent frame of a transform (translation, quaternion x/y/z/w)."""
    import numpy as np
    x, y, z, w = quaternion
    rotation = np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def judge(points, policy):
    """Reading for the overhead structure in the box ahead (points in base_link), or None when there is none."""
    import numpy as np
    if len(points) == 0:
        return None
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    box = (x >= policy.front_near_m) & (x <= policy.front_far_m) & (np.abs(y) <= policy.half_width_m)
    above = box & (z > policy.floor_noise_m) & (z <= policy.top_m)
    if np.count_nonzero(above) < policy.min_points:
        return None
    lowest = round(float(np.partition(z[above], policy.min_points - 1)[policy.min_points - 1]), 3)
    if lowest <= policy.lidar_plane_m:
        return None   # the LiDAR sees it: an ordinary obstacle
    xs = x[above]
    x0, x1 = float(xs.min()), float(xs.max())
    floor = box & (z <= policy.floor_noise_m) & (x >= x0) & (x <= x1)
    cx, cy = float(xs.mean()), float(y[above].mean())
    if np.count_nonzero(floor) < policy.floor_min_points:
        return Reading('UNKNOWN', None, cx, cy, 'NO_FLOOR_SEEN', x0, x1, lowest)
    return Reading('PASSABLE' if lowest >= policy.clearance_m else 'BLOCKED', lowest, cx, cy, '', x0, x1, lowest)


class OverheadConfirmer:
    """Counts consecutive processed frames with the same answer at one place (map frame)."""

    def __init__(self, policy):
        self.policy = policy
        self.run = []
        self.frames_of_last = 0

    def add(self, reading):
        """One processed frame (Reading in the map frame, or None); the confirmed Reading on the Nth agreeing one."""
        if reading is None:
            self.run = []
            return None
        if self.run and (reading.state != self.run[0].state or math.dist((reading.x, reading.y), (
                self.run[0].x, self.run[0].y)) > self.policy.position_tolerance_m):
            self.run = []
        self.run.append(reading)
        if len(self.run) < self.policy.confirm_frames:
            return None
        run, self.run = self.run, []
        self.frames_of_last = len(run)
        lowest = None if run[0].state == 'UNKNOWN' else round(median(r.lowest_m for r in run), 3)
        return Reading(run[0].state, lowest, sum(r.x for r in run) / len(run), sum(r.y for r in run) / len(run),
                       run[-1].reason)


class PassageRegistry:
    """Reported places per map revision; ``update`` says whether the published list changes."""

    def __init__(self, policy, new_id=lambda: str(uuid4())):
        self.policy, self.new_id = policy, new_id
        self._items = []

    def update(self, reading, *, map_session_id, map_revision, observed_at_ns, frames):
        same = [i for i in self._items if (i['map_session_id'], i['map_revision']) == (map_session_id, map_revision)
                and math.dist((i['x'], i['y']), (reading.x, reading.y)) <= self.policy.merge_radius_m]
        if not same:
            self._items.append(dict(id=self.new_id(), state=reading.state, lowest_m=reading.lowest_m, x=reading.x,
                                    y=reading.y, map_session_id=map_session_id, map_revision=map_revision,
                                    frames=frames, first_seen_ns=observed_at_ns, last_seen_ns=observed_at_ns,
                                    reason=reading.reason))
            return True
        item = min(same, key=lambda i: math.dist((i['x'], i['y']), (reading.x, reading.y)))
        if reading.state == 'UNKNOWN' and item['state'] != 'UNKNOWN':
            return False   # another distance did not see the floor: the height measured before stands
        changed = (item['state'], item['lowest_m']) != (reading.state, reading.lowest_m)
        item.update(state=reading.state, lowest_m=reading.lowest_m, x=reading.x, y=reading.y, reason=reading.reason,
                    frames=item['frames'] + frames, last_seen_ns=max(item['last_seen_ns'], observed_at_ns))
        return changed

    def items(self):
        return [dict(i) for i in sorted(self._items, key=lambda i: (i['first_seen_ns'], i['id']))]

    def confirmed_at(self, reading, *, map_session_id, map_revision):
        """The reported item at this place with the same answer (the camera view turns green/red), or None."""
        same = [i for i in self._items if (i['map_session_id'], i['map_revision']) == (map_session_id, map_revision)
                and i['state'] == reading.state
                and math.dist((i['x'], i['y']), (reading.x, reading.y)) <= self.policy.merge_radius_m]
        return min(same, key=lambda i: math.dist((i['x'], i['y']), (reading.x, reading.y)), default=None)


def nearest_image(images, stamp_ns, slop_ns):
    """The image of [(stamp_ns, image), ...] nearest stamp_ns within slop_ns, or None (never another frame)."""
    best = min(images, key=lambda item: abs(item[0] - stamp_ns), default=None)
    return best[1] if best is not None and abs(best[0] - stamp_ns) <= slop_ns else None


def free_space_polygon(reading, policy, height_m, base_to_optical, k, *, scale):
    """Image outline (int pixels) of the box under the structure: its x span, the passage width, floor to height_m.

    ``base_to_optical`` = (translation, quaternion) taking base_link points to the camera optical frame; ``k`` the
    row-major 3x3 CameraInfo K of the image the depth is aligned to; ``scale`` image size / CameraInfo size.
    None when a corner is not in front of the camera."""
    import numpy as np
    corners = np.array([(x, y, z) for x in (reading.x0, reading.x1) for y in (-policy.half_width_m, policy.half_width_m)
                        for z in (0.0, height_m)])
    optical = to_frame(corners, *base_to_optical)
    if (optical[:, 2] <= 0.05).any():
        return None
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    pixels = [(int(round((fx * px / pz + cx) * scale)), int(round((fy * py / pz + cy) * scale))) for px, py, pz in optical]
    hull, points = [], sorted(set(pixels))   # convex hull (monotone chain) so any box view is one outline
    for chain in (points, points[::-1]):
        part = []
        for p in chain:
            while len(part) >= 2 and ((part[-1][0] - part[-2][0]) * (p[1] - part[-2][1])
                                      - (part[-1][1] - part[-2][1]) * (p[0] - part[-2][0])) <= 0:
                part.pop()
            part.append(p)
        hull += part[:-1]
    return hull


def view_size_checked(shape, info):
    """None when the image (numpy shape) is the size of its CameraInfo; ValueError otherwise (never rescaled K)."""
    if (shape[1], shape[0]) != (info.width, info.height):
        raise ValueError(f'view image {shape[1]}x{shape[0]} is not its CameraInfo {info.width}x{info.height}')


def view_label(state, lowest_m, *, confirmed):
    """(text, BGR colour) for the camera view; the same words and colours as the web map (OPS-189)."""
    if confirmed and state == 'PASSABLE':
        return f'통과 가능 ↑{lowest_m:.2f} m', GREEN
    if confirmed and state == 'BLOCKED':
        return f'통과 불가 ↑{lowest_m:.2f} m', RED
    if confirmed:
        return '판정 불가', GREY   # UNKNOWN: the floor under it was not seen
    return '판정 중', GREY         # camera only: the place is still counting its agreeing frames


_FONTS = {}


def render_view(bgr, polygon, text, colour, *, width, quality, font_path):
    """JPEG bytes: the polygon filled half-transparent and outlined, the text on a dark band, scaled to ``width``."""
    import cv2
    import numpy as np
    image = bgr.copy()
    if polygon:
        fill = image.copy()
        outline = np.array(polygon, dtype=np.int32)
        cv2.fillPoly(fill, [outline], colour)
        image = cv2.addWeighted(fill, 0.45, image, 0.55, 0)
        cv2.polylines(image, [outline], True, colour, 2)
    height = max(1, round(image.shape[0] * width / image.shape[1]))
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    if text:
        from PIL import Image, ImageDraw, ImageFont
        size = max(10, width // 16)
        if (font_path, size) not in _FONTS:
            _FONTS[font_path, size] = ImageFont.truetype(font_path, size)   # Hangul needs a CJK font, not Hershey
        font = _FONTS[font_path, size]
        canvas = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(canvas)
        box = draw.textbbox((6, 4), text, font=font)
        draw.rectangle((0, 0, box[2] + 6, box[3] + 4), fill=(0, 0, 0))
        draw.text((6, 4), text, font=font, fill=tuple(reversed(colour)))
        image = cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()


def main(args=None):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rclpy._rclpy_pybind11 import RCLError
    from rescue_contracts.overhead import overhead_passages_text, overhead_passages_topic
    from rescue_interfaces.msg import MapState
    from collections import deque
    from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from .camera_utils import git_sha, read_field_parameters
    from .shutdown import spin_until_shutdown
    from .target_location import MapRevisionTracker

    rclpy.init(args=args)
    node = rclpy.create_node('overhead_passage')
    log = node.get_logger()
    try:
        values = read_field_parameters(node, OVERHEAD_PARAMS)
        try:
            policy, v = overhead_values(values)
            topic = overhead_passages_topic(v['robot_id'])
        except ValueError as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        tracker = MapRevisionTracker(robot_id=v['map_robot_id'], map_frame=v['map_frame'])
        confirmer, registry = OverheadConfirmer(policy), PassageRegistry(policy)
        buffer = Buffer()
        node.tf_listener = TransformListener(buffer, node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(String, topic, latched)
        state = {'cloud': None, 'done': None, 'frames': 0, 'busy_s': 0.0, 'info': None}
        msg_frame = ['']   # optical frame of the cloud last processed: the view CameraInfo must name the same one
        images = deque(maxlen=30)   # (stamp_ns, CompressedImage): about 1 s of 30 Hz RGB
        view_pub = node.create_publisher(CompressedImage, VIEW_TOPIC, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)) if v['view_image_topic'] else None

        def publish_view(stamp, view, base_to_optical):
            """The RGB frame of this cloud with the judgement drawn; skipped (logged) without a matching frame."""
            image, info = nearest_image(list(images), stamp, v['view_slop_ns']), state['info']
            if image is None or info is None:
                log.warning(f'camera view {stamp}: no RGB frame within {v["view_slop_ns"]} ns or no CameraInfo',
                            throttle_duration_sec=5.0)
                return
            import cv2
            import numpy as np
            bgr = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f'RGB format {image.format!r} not decodable')
            view_size_checked(bgr.shape, info)
            if info.header.frame_id != msg_frame[0]:
                raise ValueError(f'view CameraInfo frame {info.header.frame_id!r} is not the depth frame {msg_frame[0]!r}')
            polygon = None
            if view['reading'] is not None:
                polygon = free_space_polygon(view['reading'], policy, view['height'], base_to_optical, info.k,
                                             scale=1.0)
            out = CompressedImage(header=image.header, format='jpeg')
            out.data = render_view(bgr, polygon, view['text'], view['colour'], width=v['view_width'],
                                   quality=v['view_quality'], font_path=v['view_font_path'])
            view_pub.publish(out)

        def publish():
            publisher.publish(String(data=overhead_passages_text(
                robot_id=v['robot_id'], map_frame=v['map_frame'], generated_at_ns=node.get_clock().now().nanoseconds,
                clearance_m=policy.clearance_m, floor_noise_m=policy.floor_noise_m,
                lidar_plane_m=policy.lidar_plane_m, items=registry.items())))

        def on_map_state(msg):
            if msg.robot_id == v['map_robot_id']:
                tracker.update(msg)

        def lookup(target, source, stamp):
            tf = buffer.lookup_transform(target, source, Time(nanoseconds=stamp))
            t, q = tf.transform.translation, tf.transform.rotation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

        def process():
            msg = state['cloud']
            if msg is None:
                return
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if stamp == state['done']:
                return
            state['done'] = stamp
            started = time.monotonic()
            view, base_to_optical = {'reading': None, 'height': None, 'text': None, 'colour': None}, None
            try:
                age = (node.get_clock().now().nanoseconds - stamp) / 1e9
                if age > v['max_cloud_age_s']:
                    raise ValueError(f'STALE_CLOUD age={age:.3f}s')
                session, revision = tracker.basis_for(stamp, v['clock_error_bound_ns'])
                import numpy as np
                points = to_frame(cloud_xyz(msg, v['stride']), *lookup(v['base_frame'], msg.header.frame_id, stamp))
                found = judge(points, policy)
                if view_pub is not None:
                    msg_frame[0] = msg.header.frame_id
                    base_to_optical = lookup(msg.header.frame_id, v['base_frame'], stamp)
                if found is not None:
                    (mx, my, _), = to_frame(np.array([[found.x, found.y, 0.0]]),
                                            *lookup(v['map_frame'], v['base_frame'], stamp))
                    found = found._replace(x=float(mx), y=float(my))
                confirmed = confirmer.add(found)
                if confirmed is not None:
                    changed = registry.update(confirmed, map_session_id=session, map_revision=revision,
                                              observed_at_ns=stamp, frames=confirmer.frames_of_last)
                    if changed:   # the same answer again only refreshes the item, no log line
                        log.info(f'overhead {confirmed.state} lowest={confirmed.lowest_m} map=({confirmed.x:.2f}, '
                                 f'{confirmed.y:.2f}) revision={session}/{revision} {confirmed.reason} published')
                        publish()
                if found is not None:
                    item = registry.confirmed_at(found, map_session_id=session, map_revision=revision)
                    text, colour = view_label(found.state, item and item['lowest_m'], confirmed=item is not None)
                    view = {'reading': found, 'height': item['lowest_m'] if item and item['lowest_m'] else found.seen_m,
                            'text': text, 'colour': colour}
            except Exception as exc:   # stale, no map revision (LocationRefused), no TF, bad cloud: says nothing
                confirmer.add(None)
                view = {'reading': None, 'height': None, 'text': '판정 불가', 'colour': GREY}
                log.warning(f'cloud {stamp} not judged: {type(exc).__name__}: {exc}', throttle_duration_sec=5.0)
            if view_pub is not None:
                try:
                    publish_view(stamp, view, base_to_optical)
                except Exception as exc:   # the view never changes the judgement
                    log.warning(f'camera view {stamp} skipped: {type(exc).__name__}: {exc}', throttle_duration_sec=5.0)
            state['frames'] += 1
            state['busy_s'] += time.monotonic() - started
            if state['frames'] % 20 == 0:
                log.info(f'processed={state["frames"]} mean_ms={1000 * state["busy_s"] / state["frames"]:.1f}')

        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(PointCloud2, v['points_topic'], lambda m: state.update(cloud=m), sensor)
        node.create_subscription(MapState, v['map_state_topic'], on_map_state, latched)
        if view_pub is not None:
            node.create_subscription(CameraInfo, v['view_camera_info_topic'], lambda m: state.update(info=m), sensor)
            node.create_subscription(CompressedImage, v['view_image_topic'], lambda m: images.append(
                (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m)), sensor)
        else:
            log.warning('camera view OFF: no view_image_topic / view_camera_info_topic in the field file for this robot')
        node.create_timer(v['process_period_s'], process)
        publish()   # an empty list: no structure reported yet (latched for late listeners)
        log.info(f'overhead_passage git={git_sha()} topic={topic} parameters={dict(sorted(values.items()))}')
        spin_until_shutdown(lambda: rclpy.spin(node), node.context.ok, RCLError)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
