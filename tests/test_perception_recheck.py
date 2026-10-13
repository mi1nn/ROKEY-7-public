"""AMR1 recheck of a watched target point -> RecheckObservation states (2026-10-13). Pure, no ROS.

Rescue completion = fixed-webcam disappearance candidate + this recheck (owner decision 2026-10-13,
Q-10; contracts C-11 rows 7·9, SDD ObserveTarget Rules). An empty detection list alone is never
ABSENT: the point must be in view, in depth range and not occluded; the LiDAR must actually see
the disc around it. Anything not shown is UNKNOWN.
"""
import math
import unittest

from rescue_mapping import RigidTransform
from rescue_perception.recheck import (CameraModel, RecheckPolicy, RecheckWindow, frame_coverage, scan_state,
                                       window_states)

POLICY = RecheckPolicy(presence_radius_m=0.5, lidar_radius_m=0.3, min_frames=3, min_scans=3,
                       min_lidar_rays=3, min_lidar_hits=2, image_margin_px=10, min_depth_m=0.2, max_depth_m=5.0,
                       occlusion_tolerance_m=0.3, depth_window_px=2)
CAMERA = CameraModel(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
# Optical frame: z forward, x right, y down. Camera at map (0, 0, 0.3) looking along map +x.
# Rotation map<-optical: optical z -> map x, optical x -> map -y, optical y -> map -z.
MAP_FROM_OPTICAL = RigidTransform((0.0, 0.0, 0.3), (-0.5, 0.5, -0.5, 0.5))
POINT = (2.0, 0.0, 0.3)   # 2 m straight ahead


def depth(value):
    return lambda u, v, k: value


class FrameCoverageTests(unittest.TestCase):
    def test_point_in_view_with_matching_depth_is_valid(self):
        self.assertEqual(('VALID', 'IN_VIEW'), frame_coverage(POINT, MAP_FROM_OPTICAL, CAMERA, depth(2.0), POLICY))

    def test_free_space_beyond_the_point_is_valid(self):   # empty floor: depth reads the wall behind
        self.assertEqual('VALID', frame_coverage(POINT, MAP_FROM_OPTICAL, CAMERA, depth(4.0), POLICY)[0])

    def test_occluded_point_is_invalid(self):
        self.assertEqual(('INVALID', 'OCCLUDED'), frame_coverage(POINT, MAP_FROM_OPTICAL, CAMERA, depth(1.0), POLICY))

    def test_point_behind_or_outside_the_image_is_invalid(self):
        self.assertEqual(('INVALID', 'BEHIND_CAMERA'),
                         frame_coverage((-2.0, 0.0, 0.3), MAP_FROM_OPTICAL, CAMERA, depth(2.0), POLICY))
        self.assertEqual(('INVALID', 'OUT_OF_IMAGE'),
                         frame_coverage((2.0, 3.0, 0.3), MAP_FROM_OPTICAL, CAMERA, depth(2.0), POLICY))

    def test_out_of_depth_range_is_invalid_and_no_depth_is_unknown(self):
        self.assertEqual(('INVALID', 'OUT_OF_DEPTH_RANGE'),
                         frame_coverage((9.0, 0.0, 0.3), MAP_FROM_OPTICAL, CAMERA, depth(9.0), POLICY))
        self.assertEqual(('UNKNOWN', 'NO_DEPTH'), frame_coverage(POINT, MAP_FROM_OPTICAL, CAMERA, depth(None), POLICY))


def scan(ranges, angle_min=-0.5, increment=0.01, range_max=8.0):
    return dict(ranges=ranges, angle_min=angle_min, angle_increment=increment, range_min=0.1, range_max=range_max)


MAP_FROM_LASER = RigidTransform((0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))   # laser at origin, facing map +x


class ScanStateTests(unittest.TestCase):
    def test_returns_inside_the_disc_are_blocked(self):
        ranges = [2.0] * 101   # wall of returns at 2 m: the point (2, 0) is hit
        self.assertEqual('BLOCKED', scan_state(scan(ranges), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)[0])

    def test_rays_through_the_disc_without_returns_are_clear(self):
        self.assertEqual('CLEAR', scan_state(scan([6.0] * 101), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)[0])
        self.assertEqual('CLEAR', scan_state(scan([math.inf] * 101), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)[0])

    def test_an_obstacle_in_front_hides_the_disc(self):
        self.assertEqual(('UNKNOWN', 'OCCLUDED'),
                         scan_state(scan([1.0] * 101), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY))

    def test_disc_beyond_range_or_outside_the_scan_is_unknown(self):
        self.assertEqual(('UNKNOWN', 'OUT_OF_RANGE'), scan_state(scan([math.inf] * 101, range_max=2.0), MAP_FROM_LASER,
                                                                 (2.0, 0.0, 0.0), POLICY))
        self.assertEqual(('UNKNOWN', 'NOT_IN_SCAN'),
                         scan_state(scan([6.0] * 101), MAP_FROM_LASER, (-2.0, 0.0, 0.0), POLICY))


class ScanEdgeTests(unittest.TestCase):
    """#165 review (CDX-165-01·02, 2026-10-13): CLEAR is evidence for rescue completion, so a ray only
    counts as passing when it truly went beyond the disc."""

    def test_negative_infinity_is_never_clear(self):   # REP-117: -inf = something closer than range_min
        self.assertNotEqual('CLEAR', scan_state(scan([-math.inf] * 101), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)[0])

    def test_nan_or_below_range_min_is_never_clear(self):
        for bad in (math.nan, 0.05):
            with self.subTest(bad=bad):
                ranges = [6.0] * 101
                ranges[50] = bad   # one unreadable ray toward the point
                self.assertNotEqual('CLEAR', scan_state(scan(ranges), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)[0])

    def test_return_in_front_of_the_disc_entry_is_occluded(self):   # CDX-165-02 reproduction
        # Rays at 0.100-0.102 rad meet the disc (centre (2, 0), r 0.3) at about 1.766 m; 1.72 m is in front.
        state = scan_state(dict(ranges=[1.72] * 3, angle_min=0.1, angle_increment=0.001, range_min=0.1,
                                range_max=8.0), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY)
        self.assertEqual(('UNKNOWN', 'OCCLUDED'), state)

    def test_any_hidden_part_of_the_disc_prevents_clear(self):
        ranges = [6.0] * 101
        ranges[50] = 1.0   # one ray stopped well before the disc
        self.assertEqual(('UNKNOWN', 'OCCLUDED'), scan_state(scan(ranges), MAP_FROM_LASER, (2.0, 0.0, 0.0), POLICY))


class WindowTests(unittest.TestCase):
    def window(self):
        return RecheckWindow(POINT, POLICY, start_ns=1_000)

    def feed(self, w, *, frames=3, coverage='VALID', near=False, unconfirmed_at_point=False, scans=3, lidar='CLEAR'):
        for i in range(frames):
            w.add_frame(stamp_ns=2_000 + i, coverage=coverage,
                        confirmed_points=[(2.1, 0.0, 0.3)] if near else [(5.0, 4.0, 0.3)],
                        unconfirmed_at_point=unconfirmed_at_point)
        for i in range(scans):
            w.add_scan(stamp_ns=2_000 + i, state=lidar)
        return w

    def test_rescued_spot_is_absent_clear_valid(self):
        self.assertEqual(('ABSENT', 'CLEAR', 'VALID'), window_states(self.feed(self.window())))

    def test_confirmed_detection_near_the_point_is_present(self):
        self.assertEqual('PRESENT', window_states(self.feed(self.window(), near=True))[0])

    def test_no_view_means_unknown_not_absent(self):
        states = window_states(self.feed(self.window(), coverage='INVALID'))
        self.assertEqual(('UNKNOWN', 'INVALID'), (states[0], states[2]))
        self.assertEqual(('UNKNOWN', 'UNKNOWN'), (window_states(self.feed(self.window(), frames=2))[0],
                                                  window_states(self.feed(self.window(), frames=2))[2]))

    def test_unconfirmed_detection_at_the_point_blocks_absent(self):
        self.assertEqual('UNKNOWN', window_states(self.feed(self.window(), unconfirmed_at_point=True))[0])

    def test_lidar_needs_enough_scans_and_any_block_wins(self):
        self.assertEqual('UNKNOWN', window_states(self.feed(self.window(), scans=2))[1])
        w = self.feed(self.window())
        w.add_scan(stamp_ns=3_000, state='BLOCKED')
        self.assertEqual('BLOCKED', window_states(w)[1])
        self.assertEqual('UNKNOWN', window_states(self.feed(self.window(), lidar='UNKNOWN'))[2])

    def test_samples_received_before_the_window_are_ignored(self):
        w = self.window()
        for i in range(5):
            w.add_frame(stamp_ns=500 + i, coverage='VALID', confirmed_points=[], unconfirmed_at_point=False)
            w.add_scan(stamp_ns=500 + i, state='CLEAR')
        self.assertEqual(('UNKNOWN', 'UNKNOWN', 'UNKNOWN'), window_states(w))


class PolicyTests(unittest.TestCase):
    def test_values_are_checked(self):
        for bad in (dict(presence_radius_m=0.0), dict(min_frames=0), dict(max_depth_m=0.1), dict(depth_window_px=-1),
                    dict(occlusion_tolerance_m=math.nan)):
            values = dict(POLICY.__dict__) | bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                RecheckPolicy(**values)


if __name__ == '__main__':
    unittest.main()
