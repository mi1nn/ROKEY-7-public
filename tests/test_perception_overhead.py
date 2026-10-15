"""Overhead structure ahead of a robot: tunnel PASSABLE / BLOCKED / UNKNOWN from depth points (OPS-185/186).

Owner decisions 2026-10-13: 46 cm is the clearance (passage.max_height_m). Operations decisions
OPS-186: a structure floats when its lowest point is above the LiDAR plane (lower ones the LiDAR
already sees: a wall is not a tunnel); without floor seen under it the answer is UNKNOWN, never
PASSABLE; N agreeing frames at one place before it is reported.
"""
import math
import struct
from types import SimpleNamespace as NS
import unittest

try:
    import numpy as np
except ImportError:   # installed-wheel runs without NumPy skip the point tests
    np = None

from rescue_perception.overhead import (OVERHEAD_PARAMS, OverheadConfirmer, OverheadPolicy, PassageRegistry, Reading,
                                        cloud_xyz, free_space_polygon, judge, nearest_image, overhead_values,
                                        render_view, to_frame, view_label, view_size_checked)

POLICY = OverheadPolicy(clearance_m=0.46, floor_noise_m=0.05, front_near_m=0.4, front_far_m=1.0, half_width_m=0.2,
                        lidar_plane_m=0.19, top_m=1.0, min_points=5, floor_min_points=20, confirm_frames=3,
                        position_tolerance_m=0.3, merge_radius_m=0.5, near_end_m=0.05)
SESSION = '22222222-2222-4222-8222-222222222222'


def grid(x0, x1, z, *, y0=-0.2, y1=0.2, step=0.02):
    xs, ys = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step)
    return np.array([(x, y, z) for x in xs for y in ys])


def scene(*parts):
    return np.concatenate(parts)


class PolicyTests(unittest.TestCase):
    def test_rejects_unusable_values(self):
        base = dict(POLICY.__dict__)
        for bad in (dict(clearance_m=0.05), dict(floor_noise_m=-0.1), dict(lidar_plane_m=0.04),
                    dict(lidar_plane_m=0.5), dict(top_m=0.4), dict(front_far_m=0.4), dict(half_width_m=0.0),
                    dict(min_points=0), dict(floor_min_points=0), dict(confirm_frames=0), dict(confirm_frames=2.0),
                    dict(position_tolerance_m=math.nan), dict(merge_radius_m=0.0), dict(clearance_m=1)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                OverheadPolicy(**dict(base, **bad))

    def test_every_value_is_required(self):
        values = {name: None for name in OVERHEAD_PARAMS}
        with self.assertRaises(ValueError) as caught:
            overhead_values(values)
        self.assertIn('clearance_m', str(caught.exception))


@unittest.skipIf(np is None, 'NumPy not installed: point judgement NOT_RUN')
class JudgeTests(unittest.TestCase):
    FLOOR = None

    def setUp(self):
        self.floor = grid(0.4, 1.0, 0.0)

    def test_tunnel_at_or_above_the_clearance_is_passable(self):
        r = judge(scene(self.floor, grid(0.6, 0.8, 0.47)), POLICY)
        self.assertEqual(('PASSABLE', 0.47), (r.state, round(r.lowest_m, 3)))
        # the free space under it for the camera view (OPS-189): x span of the structure, lowest seen
        self.assertEqual((0.6, 0.8, 0.47), (round(r.x0, 6), round(r.x1, 6), r.seen_m))
        self.assertAlmostEqual(0.7, r.x, places=6)
        self.assertAlmostEqual(0.0, r.y, places=6)

    def test_lower_structure_above_the_lidar_plane_is_blocked(self):
        r = judge(scene(self.floor, grid(0.6, 0.8, 0.40)), POLICY)
        self.assertEqual(('BLOCKED', 0.40), (r.state, round(r.lowest_m, 3)))

    def test_a_wall_from_the_floor_is_not_a_tunnel(self):
        wall = np.array([(0.7, y, z) for y in np.arange(-0.2, 0.21, 0.05) for z in np.arange(0.06, 0.9, 0.05)])
        self.assertIsNone(judge(scene(self.floor, wall), POLICY))   # the LiDAR sees it

    def test_no_floor_seen_under_it_is_unknown_never_passable(self):
        r = judge(grid(0.6, 0.8, 0.47), POLICY)
        self.assertEqual(('UNKNOWN', None, 'NO_FLOOR_SEEN'), (r.state, r.lowest_m, r.reason))
        self.assertEqual(0.47, r.seen_m)   # drawn grey up to what was seen; no height is claimed
        far_floor = grid(0.4, 0.5, 0.0)   # floor seen only nearer than the structure
        self.assertEqual('UNKNOWN', judge(scene(far_floor, grid(0.7, 0.9, 0.47)), POLICY).state)

    def test_lowest_is_the_min_points_th_lowest_point(self):
        noise = np.array([(0.7, 0.0, 0.30)] * 4)   # fewer than min_points: as collision_monitor, not a stop
        r = judge(scene(self.floor, grid(0.6, 0.8, 0.47), noise), POLICY)
        self.assertEqual(('PASSABLE', 0.47), (r.state, round(r.lowest_m, 3)))
        r = judge(scene(self.floor, grid(0.6, 0.8, 0.47), np.array([(0.7, 0.0, 0.30)] * 5)), POLICY)
        self.assertEqual(('BLOCKED', 0.30), (r.state, round(r.lowest_m, 3)))

    def test_nothing_or_too_little_or_outside_is_none(self):
        self.assertIsNone(judge(self.floor, POLICY))
        self.assertIsNone(judge(scene(self.floor, np.array([(0.7, 0.0, 0.47)] * 4)), POLICY))
        self.assertIsNone(judge(scene(self.floor, grid(0.6, 0.8, 1.2)), POLICY))           # above top_m
        self.assertIsNone(judge(scene(self.floor, grid(0.6, 0.8, 0.47, y0=0.3, y1=0.5)), POLICY))   # beside the path
        self.assertIsNone(judge(scene(self.floor, grid(1.2, 1.4, 0.47)), POLICY))          # farther than the box
        self.assertIsNone(judge(np.empty((0, 3)), POLICY))


@unittest.skipIf(np is None, 'NumPy not installed: point cloud reading NOT_RUN')
class CloudTests(unittest.TestCase):
    def cloud(self, rows):
        h, w = len(rows), len(rows[0])
        data = b''.join(struct.pack('<fffI', *p, 0) for row in rows for p in row)
        fields = [NS(name=n, offset=o, datatype=7, count=1) for n, o in (('x', 0), ('y', 4), ('z', 8))]
        return NS(height=h, width=w, fields=fields, point_step=16, row_step=16 * w, is_bigendian=False, data=data)

    def test_organized_cloud_with_stride_and_no_nan(self):
        nan = float('nan')
        rows = [[(float(c), float(r), 1.0) if (r, c) != (0, 2) else (nan, nan, nan) for c in range(4)] for r in range(4)]
        points = cloud_xyz(self.cloud(rows), stride=2)
        self.assertEqual([(0.0, 0.0, 1.0), (0.0, 2.0, 1.0), (2.0, 2.0, 1.0)], [tuple(p) for p in points.tolist()])
        self.assertEqual(15, len(cloud_xyz(self.cloud(rows), stride=1)))

    def test_unsupported_layout_is_refused(self):
        msg = self.cloud([[(0.0, 0.0, 1.0)]])
        for change in (dict(is_bigendian=True), dict(fields=msg.fields[:2]),
                       dict(fields=[NS(name='x', offset=0, datatype=8, count=1)] + msg.fields[1:])):
            with self.subTest(change=change), self.assertRaises(ValueError):
                cloud_xyz(NS(**dict(msg.__dict__, **change)), stride=1)

    def test_optical_to_base(self):
        # camera 0.244 m up looking along base x: optical z forward, x right, y down
        q = (-0.5, 0.5, -0.5, 0.5)
        base = to_frame(np.array([[0.0, 0.0, 2.0], [0.1, 0.2, 1.0]]), (0.0, 0.0, 0.244), q)
        np.testing.assert_allclose([[2.0, 0.0, 0.244], [1.0, -0.1, 0.044]], base, atol=1e-9)


def reading(state='PASSABLE', lowest=0.47, x=2.0, y=1.0):
    return Reading(state, lowest, x, y, '' if state != 'UNKNOWN' else 'NO_FLOOR_SEEN')


class ConfirmerTests(unittest.TestCase):
    def test_reported_on_the_nth_agreeing_frame_with_the_median(self):
        c = OverheadConfirmer(POLICY)
        out = [c.add(reading(lowest=v, x=2.0 + d)) for v, d in ((0.47, 0.0), (0.49, 0.1), (0.48, 0.2))]
        self.assertEqual([None, None], out[:2])
        self.assertEqual(('PASSABLE', 0.48, 3), (out[2].state, out[2].lowest_m, c.frames_of_last))
        self.assertAlmostEqual(2.1, out[2].x)
        self.assertIsNone(c.add(reading()))   # counts again from the start

    def test_another_state_place_or_frame_without_it_restarts(self):
        for gap in (reading(state='BLOCKED', lowest=0.40), reading(x=2.5), None, reading(state='UNKNOWN', lowest=None)):
            c = OverheadConfirmer(POLICY)
            out = [c.add(reading()), c.add(reading()), c.add(gap), c.add(reading())]
            with self.subTest(gap=gap):
                self.assertEqual([None] * 4, out)


class RegistryTests(unittest.TestCase):
    def update(self, registry, r, *, revision=1, at=1_000, frames=3):
        return registry.update(r, map_session_id=SESSION, map_revision=revision, observed_at_ns=at, frames=frames)

    def test_new_place_then_same_place_updates_without_a_new_item(self):
        reg = PassageRegistry(POLICY, new_id=iter(['33333333-3333-4333-8333-%012d' % i for i in range(1, 9)]).__next__)
        self.assertTrue(self.update(reg, reading()))
        self.assertFalse(self.update(reg, reading(x=2.2), at=2_000))   # same state and height: nothing to resend
        self.assertTrue(self.update(reg, reading(state='BLOCKED', lowest=0.40, x=2.1), at=3_000))
        items = reg.items()
        self.assertEqual(1, len(items))
        self.assertEqual(dict(id='33333333-3333-4333-8333-000000000001', state='BLOCKED', lowest_m=0.40, x=2.1, y=1.0,
                              map_session_id=SESSION, map_revision=1, frames=9, first_seen_ns=1_000,
                              last_seen_ns=3_000, reason=''), items[0])

    def test_unknown_never_replaces_a_known_height_but_a_height_replaces_unknown(self):
        reg = PassageRegistry(POLICY)
        self.update(reg, reading())
        self.assertFalse(self.update(reg, reading(state='UNKNOWN', lowest=None), at=2_000))
        self.assertEqual('PASSABLE', reg.items()[0]['state'])
        other = PassageRegistry(POLICY)
        self.update(other, reading(state='UNKNOWN', lowest=None))
        self.assertTrue(self.update(other, reading(), at=2_000))
        self.assertEqual(('PASSABLE', 0.47, ''), tuple(other.items()[0][k] for k in ('state', 'lowest_m', 'reason')))

    def test_farther_than_the_merge_radius_or_another_revision_is_another_item(self):
        reg = PassageRegistry(POLICY)
        self.update(reg, reading())
        self.assertTrue(self.update(reg, reading(x=2.6)))
        self.assertTrue(self.update(reg, reading(), revision=2))   # kept with its own revision (OPS-186 c)
        self.assertEqual([(1, 2.0), (1, 2.6), (2, 2.0)],
                         sorted((i['map_revision'], i['x']) for i in reg.items()))


class ViewTests(unittest.TestCase):
    """Judgement drawn on the robot camera (owner decision OPS-189): the free space under the structure."""
    K = (100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0)
    BASE_TO_OPTICAL = ((0.0, 0.244, 0.0), (0.5, -0.5, 0.5, 0.5))   # inverse of camera 0.244 m up looking ahead

    def test_nearest_image_within_the_slop(self):
        images = [(1_000, 'a'), (1_030, 'b'), (1_070, 'c')]
        self.assertEqual('b', nearest_image(images, 1_040, 20))
        self.assertIsNone(nearest_image(images, 1_100, 20))
        self.assertIsNone(nearest_image([], 1_000, 20))

    @unittest.skipIf(np is None, 'NumPy not installed')
    def test_free_space_polygon_projects_the_box_under_the_structure(self):
        r = Reading('PASSABLE', 0.47, 0.7, 0.0, '', x0=1.0, x1=1.0, seen_m=0.47)
        poly = free_space_polygon(r, POLICY, 0.244, self.BASE_TO_OPTICAL, self.K, scale=1.0)
        # at 1 m ahead: floor 0.244 m below the camera -> v = 50 + 100*0.244 = 74.4; 0.47 m -> 0.226 up -> 27.4
        us, vs = [u for u, _ in poly], [v for _, v in poly]
        self.assertEqual((30, 70), (min(us), max(us)))   # half width 0.2 m -> 20 px
        self.assertEqual((50 - 0, 74), (min(vs), max(vs)))   # drawn up to the given height (camera level here)
        half = free_space_polygon(r, POLICY, 0.244, self.BASE_TO_OPTICAL, self.K, scale=0.5)
        self.assertEqual((15, 35), (min(u for u, _ in half), max(u for u, _ in half)))

    def test_the_view_image_must_be_its_camera_info_size(self):
        # the small preview stream is projected with its own CameraInfo (OPS-198): another size is refused
        info = NS(width=250, height=250)
        self.assertIsNone(view_size_checked((250, 250, 3), info))
        for shape in ((704, 704, 3), (250, 300, 3)):
            with self.subTest(shape=shape), self.assertRaises(ValueError):
                view_size_checked(shape, info)

    def test_labels_match_the_map_words(self):
        self.assertEqual(('통과 가능 ↑0.47 m', (0, 200, 0)), view_label('PASSABLE', 0.47, confirmed=True))
        self.assertEqual(('통과 불가 ↑0.40 m', (0, 0, 220)), view_label('BLOCKED', 0.4, confirmed=True))
        # one wording for the map and the camera (OPS-196): UNKNOWN 「판정 불가」; only the camera has
        # 「판정 중」, a place still counting its agreeing frames
        self.assertEqual(('판정 불가', (160, 160, 160)), view_label('UNKNOWN', None, confirmed=True))
        for state, lowest in (('PASSABLE', 0.47), ('UNKNOWN', None), ('BLOCKED', 0.4)):
            self.assertEqual(('판정 중', (160, 160, 160)), view_label(state, lowest, confirmed=False))

    @unittest.skipIf(np is None, 'NumPy not installed')
    def test_render_view_paints_the_polygon_and_writes_a_jpeg(self):
        try:
            import cv2
        except ImportError:
            self.skipTest('OpenCV not installed: view NOT_RUN')
        from pathlib import Path
        fonts = sorted(Path('/usr/share/fonts').glob('**/NotoSansCJK-*.ttc'))
        if not fonts:
            self.skipTest('no CJK font: view text NOT_RUN')
        bgr = np.full((100, 100, 3), 255, dtype=np.uint8)
        jpeg = render_view(bgr, [(30, 50), (70, 50), (70, 74), (30, 74)], '통과 가능 ↑0.47 m', (0, 200, 0),
                           width=50, quality=90, font_path=str(fonts[0]))
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual((50, 50, 3), image.shape)
        b, g, r = (int(c) for c in image[31, 25])   # inside the polygon (scaled by half)
        self.assertTrue(g > 200 and r < 200 and b < 200, (b, g, r))
        self.assertTrue(all(int(c) > 240 for c in image[45, 5]))   # outside: untouched
        plain = cv2.imdecode(np.frombuffer(render_view(bgr, None, None, None, width=50, quality=90,
                                                       font_path=str(fonts[0])), np.uint8), cv2.IMREAD_COLOR)
        self.assertTrue((plain > 240).all())   # no structure: nothing painted


class ParamsTests(unittest.TestCase):
    def test_values_give_the_policy_and_the_node_settings(self):
        values = dict(points_topic='/robot6/passage_points', base_frame='base_link', map_frame='map',
                      map_robot_id='robot_a', map_state_topic='/rescue/robots/robot_a/map_state', robot_id='robot_a',
                      clock_error_bound_ns=150_000_000, process_period_s=0.5, max_cloud_age_s=0.5, stride=2,
                      view_camera_info_topic='/robot6/oakd/rgb/preview/camera_info',
                      view_image_topic='/robot6/oakd/rgb/preview/image_raw/compressed', view_width=352, view_quality=70,
                      view_slop_ns=20_000_000, view_font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
                      clearance_m=0.46, floor_noise_m=0.05, front_near_m=0.4, front_far_m=1.0, half_width_m=0.2,
                      lidar_plane_m=0.19, top_m=1.0, min_points=5, floor_min_points=20, confirm_frames=3,
                      position_tolerance_m=0.3, merge_radius_m=0.5, near_end_m=0.05)
        self.assertEqual(set(values), set(OVERHEAD_PARAMS))
        policy, node = overhead_values(values)
        self.assertEqual(POLICY, policy)
        self.assertEqual(('/robot6/passage_points', 2, 0.5), (node['points_topic'], node['stride'], node['process_period_s']))
        self.assertEqual(('/robot6/oakd/rgb/preview/image_raw/compressed', 352), (node['view_image_topic'], node['view_width']))
        off = overhead_values(dict(values, view_image_topic='', view_camera_info_topic=''))[1]   # view off
        self.assertEqual(('', ''), (off['view_image_topic'], off['view_camera_info_topic']))
        for half in (dict(view_image_topic=''), dict(view_camera_info_topic='')):   # both or neither
            with self.subTest(half=half), self.assertRaises(ValueError):
                overhead_values(dict(values, **half))
        for name, bad in (('stride', 0), ('process_period_s', 0.0), ('max_cloud_age_s', -1.0),
                          ('clock_error_bound_ns', -1), ('points_topic', ' '), ('view_width', 0),
                          ('view_quality', 101), ('view_slop_ns', -1), ('view_font_path', '')):
            with self.subTest(name=name), self.assertRaises(ValueError):
                overhead_values(dict(values, **{name: bad}))


if __name__ == '__main__':
    unittest.main()
