"""scripts/sim/wall_clock_relay.py pure parts (no ROS): frame names, restamping, TF edges (2026-10-15).

The simulator publishes <ns>/odom -> <ns>/base_link and a Gazebo lidar frame, stamped with simulation time; the
field robot publishes odom -> base_link and rplidar_link on the wall clock [측정 2026-10-15]. The relay
renames those frames, drops the simulator's identity links that become self loops, and restamps.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / 'scripts' / 'sim' / 'wall_clock_relay.py'
HAS_TOOL = TOOL.exists()   # the installed-wheel check copies tests/ and config/ only
if HAS_TOOL:
    spec = importlib.util.spec_from_file_location('wall_clock_relay', TOOL)
    relay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(relay)
else:
    raise unittest.SkipTest('installed-wheel checkout has no scripts/')


def header(frame, sec=14, nsec=150_000_000):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nsec))


def edge(parent, child):
    return NS(header=header(parent), child_frame_id=child)


class FrameNames(unittest.TestCase):
    def test_simulator_frames_become_field_frames(self):
        frames = relay.frame_map('robot6')
        self.assertEqual('odom', relay.field_frame(frames, 'robot6/odom'))
        self.assertEqual('base_link', relay.field_frame(frames, 'robot6/base_link'))
        self.assertEqual('rplidar_link', relay.field_frame(frames, 'robot6/turtlebot4/rplidar_link/rplidar'))
        self.assertEqual('oakd_rgb_camera_optical_frame',
                         relay.field_frame(frames, 'oakd_rgb_camera_optical_frame'))   # already the field name
        self.assertEqual('robot5/odom', relay.field_frame(frames, 'robot5/odom'))     # other robot untouched

    def test_namespace_is_a_plain_name(self):
        for bad in ('', '/robot6', 'a/b'):
            with self.subTest(bad), self.assertRaises(ValueError):
                relay.frame_map(bad)


class Restamp(unittest.TestCase):
    def test_stamp_of_nanoseconds(self):
        self.assertEqual((1792068725, 142700400), relay.stamp_of(1792068725142700400))
        for bad in (-1, 1.5, None):
            with self.subTest(bad), self.assertRaises(ValueError):
                relay.stamp_of(bad)

    def test_message_header_and_child_frame(self):
        odom = NS(header=header('robot6/odom'), child_frame_id='robot6/base_link')
        out = relay.relay_message(odom, relay.frame_map('robot6'), lambda sec, nsec: (1792068725, 5))
        self.assertEqual(('odom', 'base_link'), (out.header.frame_id, out.child_frame_id))
        self.assertEqual((1792068725, 5), (out.header.stamp.sec, out.header.stamp.nanosec))

    def test_message_without_header_passes(self):
        text = NS(data='<robot/>')
        self.assertIs(text, relay.relay_message(text, relay.frame_map('robot6'), lambda sec, nsec: (1, 2)))

    def test_same_simulation_stamp_gets_the_same_wall_stamp(self):
        # depth_image_proc pairs depth and camera_info by exact stamp (0 pairs before, 2026-10-15)
        clock = iter([1_000_000_000_000_000_001, 1_000_000_000_000_000_500, 1_000_000_000_000_000_900])
        stamps = relay.StampMap(lambda: next(clock))
        depth, info, later = stamps(14, 120_000_000), stamps(14, 120_000_000), stamps(14, 170_000_000)
        self.assertEqual(depth, info)
        self.assertEqual((1_000_000_000, 1), depth)
        self.assertNotEqual(depth, later)

    def test_stamp_map_keeps_a_bounded_history(self):
        clock = iter(range(1, 100))
        stamps = relay.StampMap(lambda: next(clock), size=2)
        first = stamps(1, 0)
        stamps(2, 0)
        stamps(3, 0)                       # pushes (1, 0) out
        self.assertNotEqual(first, stamps(1, 0))


class TfEdges(unittest.TestCase):
    def test_identity_links_are_dropped_and_the_rest_renamed(self):
        msg = NS(transforms=[edge('odom', 'robot6/odom'), edge('robot6/base_link', 'base_link'),
                             edge('robot6/odom', 'robot6/base_link'),
                             edge('rplidar_link', 'robot6/turtlebot4/rplidar_link/rplidar'),
                             edge('shell_link', 'rplidar_link')])
        out = relay.relay_message(msg, relay.frame_map('robot6'), lambda sec, nsec: (7, 8))
        self.assertEqual([('odom', 'base_link'), ('shell_link', 'rplidar_link')],
                         [(t.header.frame_id, t.child_frame_id) for t in out.transforms])
        self.assertTrue(all((t.header.stamp.sec, t.header.stamp.nanosec) == (7, 8) for t in out.transforms))

    def test_nothing_left_sends_nothing(self):
        msg = NS(transforms=[edge('odom', 'robot6/odom')])
        self.assertIsNone(relay.relay_message(msg, relay.frame_map('robot6'), lambda sec, nsec: (7, 8)))


class Depth(unittest.TestCase):
    def setUp(self):
        try:
            import cv2  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            raise unittest.SkipTest('OpenCV/numpy not installed')

    def test_metres_to_millimetres_with_no_return_as_zero(self):
        import numpy as np
        mm = relay.depth_mm(np.array([[0.4047, float('nan')], [float('inf'), -1.0]], dtype=np.float32))
        self.assertEqual([[405, 0], [0, 0]], mm.tolist())
        self.assertEqual(65535, int(relay.depth_mm(np.array([100.0], dtype=np.float32))[0]))

    def test_depth_noise_grows_with_distance_squared_and_keeps_no_return(self):
        # sigma = k z^2 [추정 2026-10-16]: stereo depth error grows with the distance squared; 1 cm at 1 m
        import numpy as np
        rng = np.random.default_rng(7)
        near, far = np.full((200, 200), 0.5, np.float32), np.full((200, 200), 2.0, np.float32)
        s_near = float(np.std(relay.noisy_depth(near, 0.01, rng) - near))
        s_far = float(np.std(relay.noisy_depth(far, 0.01, rng) - far))
        self.assertAlmostEqual(0.01 * 0.25, s_near, delta=0.0003)
        self.assertAlmostEqual(0.01 * 4.0, s_far, delta=0.002)
        holes = np.array([[float('nan'), float('-inf'), 0.0, 1.0]], np.float32)
        out = relay.noisy_depth(holes, 0.01, rng)
        self.assertEqual([0, 0, 0, 1], relay.depth_mm(out).astype(bool).astype(int).tolist()[0])   # no return stays
        self.assertTrue(np.array_equal(near, relay.noisy_depth(near, 0.0, rng)))                   # k 0: unchanged

    def test_the_default_noise_is_on_and_documented(self):
        self.assertEqual(0.01, relay.DEPTH_NOISE_PER_M2)

    def test_payload_is_what_the_field_decoder_reads(self):
        # rescue_perception.target_utils.decode_depth_png: '<iff' header with mode 0, then a uint16 PNG
        import cv2
        import numpy as np
        depth = np.array([[0.5, 1.25], [2.0, float('nan')]], dtype=np.float32)
        payload = relay.depth_payload(depth)
        mode, _, _ = relay.DEPTH_HEADER.unpack_from(payload)
        self.assertEqual(0, mode)
        self.assertEqual(b'\x89PNG\r\n\x1a\n', payload[12:20])
        back = cv2.imdecode(np.frombuffer(payload[12:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        self.assertEqual((np.uint16, [[500, 1250], [2000, 0]]), (back.dtype.type, back.tolist()))
        self.assertRegex(relay.DEPTH_FORMAT, r'^16UC1;\s*compressedDepth png$')


class TopicNames(unittest.TestCase):
    def test_camera_info_reaches_the_field_readiness_name(self):
        by_field = {t.field: t for t in relay.TOPICS}
        self.assertEqual('oakd/rgb/preview/camera_info', by_field['oakd/rgb/camera_info'].sim)   # #342 readiness
        self.assertEqual('depth_png', by_field['oakd/stereo/image_raw/compressedDepth'].convert)  # passage, readiness
        self.assertEqual('jpeg', by_field['oakd/rgb/image_raw/compressed'].convert)

    def test_only_commands_go_to_the_simulator(self):
        self.assertEqual(['cmd_vel'], [t.field for t in relay.TOPICS if t.to_sim])


if __name__ == '__main__':
    unittest.main()
