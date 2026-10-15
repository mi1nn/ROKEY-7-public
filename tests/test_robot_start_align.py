"""TB-5 start alignment on the frozen TB-6 map (재범 decision 2026-10-15, method B); no ROS.

The field launch starts AMCL with the measured start (set_initial_pose parameter). Nav2 Jazzy builds
that initial pose with an all-zero covariance [Nav2 jazzy amcl_node.cpp:283-295, 621-631], so every
particle sits on the measured pose: AMCL cannot correct a tape-measure error while TB-5 stands still,
and its small covariance already passes the pose validity check. start_align, once TB-5 is undocked
and stopped, re-spreads the particles around AMCL's pose (set_initial_pose service, with covariance),
asks for no-motion updates (request_nomotion_update) and accepts the result only when the spread is
small, the pose has settled, the correction is bounded and the scan lies on the frozen map's walls.
Anything else is a reason code; nothing is replaced by another method.
"""
import math
import unittest

from rescue_robot.start_align import (AlignConfig, AlignCore, CODES, match_ratio, sigmas)

S = 1_000_000_000
CONFIG = dict(initial_sigma_xy_m=0.3, initial_sigma_yaw_rad=0.26, update_period_s=0.5, max_updates=20,
              accept_sigma_xy_m=0.1, accept_sigma_yaw_rad=0.087, settle_count=3, settle_xy_m=0.02,
              settle_yaw_rad=0.0175, max_jump_xy_m=0.5, max_jump_yaw_rad=0.35, match_hit_m=0.1,
              min_match_ratio=0.7, min_match_points=30, scan_max_age_s=1.0, stopped_linear_mps=0.01,
              stopped_angular_rps=0.02, service_wait_s=5.0)


def cov(sxy, syaw):
    c = [0.0] * 36
    c[0] = c[7] = sxy * sxy
    c[35] = syaw * syaw
    return c


def room(width=80, height=80, resolution=0.05):
    """Occupied walls on the border of a 4 x 4 m room, origin (-2, -2)."""
    data = [100 if x in (0, width - 1) or y in (0, height - 1) else 0 for y in range(height) for x in range(width)]
    return dict(width=width, height=height, resolution=resolution, origin=(-2.0, -2.0, 0.0), data=data)


def room_scan(x, y, yaw, beams=180):
    """Ranges a laser at (x, y, yaw) in room() sees: walls at +-1.975 m (cell centres)."""
    wall = 2.0 - 0.025
    angles, ranges = [], []
    for i in range(beams):
        a = -math.pi + 2 * math.pi * i / beams
        dx, dy = math.cos(yaw + a), math.sin(yaw + a)
        hits = [t for t in ((wall - x) / dx if dx > 1e-9 else None, (-wall - x) / dx if dx < -1e-9 else None,
                            (wall - y) / dy if dy > 1e-9 else None, (-wall - y) / dy if dy < -1e-9 else None)
                if t is not None and t > 0]
        angles.append(a)
        ranges.append(min(hits))
    return dict(angle_min=-math.pi, angle_increment=2 * math.pi / beams, range_min=0.1, range_max=12.0,
                ranges=ranges)


class Fixture:
    def __init__(self, **changes):
        self.core = AlignCore(AlignConfig(**{**CONFIG, **changes}))
        self.now = 100 * S

    def ready(self, pose=(0.5, 0.2, 0.1), scan_pose=None):
        c = self.core
        c.on_map(room(), sha256='a' * 64)
        c.on_dock(False)
        c.on_odom(0.0, 0.0)
        c.on_scan(room_scan(*(scan_pose or pose)), laser_in_base=(0.0, 0.0, 0.0), received_ns=self.now)
        c.on_amcl(pose, cov(0.0, 0.0))
        c.on_services(True, self.now)

    def tick(self, seconds=0.5):
        self.now += int(seconds * S)
        self.core.scan_received_ns = self.now   # scans keep coming while TB-5 stands
        return self.core.tick(self.now)


class Config(unittest.TestCase):
    def test_every_value_is_required_and_positive(self):
        for key in CONFIG:
            values = dict(CONFIG)
            values.pop(key)
            with self.subTest(missing=key), self.assertRaises(TypeError):
                AlignConfig(**values)
            values[key] = 0
            with self.subTest(zero=key), self.assertRaises(ValueError):
                AlignConfig(**values)

    def test_counts_are_integers_and_the_ratio_a_fraction(self):
        for key, bad in (('max_updates', 2.5), ('settle_count', 1), ('min_match_points', 3.0),
                         ('min_match_ratio', 1.5)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                AlignConfig(**{**CONFIG, key: bad})

    def test_reason_codes(self):
        self.assertEqual({'START_ALIGN_WAIT_MAP', 'START_ALIGN_DOCKED', 'START_ALIGN_NOT_STOPPED',
                          'START_ALIGN_NO_SCAN', 'START_ALIGN_WAIT_AMCL', 'START_ALIGN_NO_SERVICE',
                          'START_ALIGN_NOT_CONVERGED', 'START_ALIGN_JUMP_TOO_LARGE', 'START_ALIGN_LOW_MATCH'},
                         set(CODES))


class Waiting(unittest.TestCase):
    def test_reasons_in_order_until_every_input_is_there(self):
        f = Fixture()
        c = f.core
        self.assertEqual(('WAITING', 'START_ALIGN_WAIT_MAP'), c.state())
        c.on_map(room(), sha256='a' * 64)
        self.assertEqual(('WAITING', 'START_ALIGN_DOCKED'), c.state())   # dock state unknown counts as docked
        c.on_dock(True)
        self.assertEqual(('WAITING', 'START_ALIGN_DOCKED'), c.state())
        c.on_dock(False)
        self.assertEqual(('WAITING', 'START_ALIGN_NOT_STOPPED'), c.state())   # no odom yet
        c.on_odom(0.05, 0.0)
        self.assertEqual(('WAITING', 'START_ALIGN_NOT_STOPPED'), c.state())
        c.on_odom(0.0, 0.0)
        f.tick()
        self.assertEqual(('WAITING', 'START_ALIGN_NO_SCAN'), c.state())
        c.on_scan(room_scan(0, 0, 0), laser_in_base=(0, 0, 0), received_ns=f.now)
        f.tick()
        self.assertEqual(('WAITING', 'START_ALIGN_WAIT_AMCL'), c.state())
        c.on_amcl((0.0, 0.0, 0.0), cov(0.0, 0.0))
        self.assertEqual([], f.tick())
        self.assertEqual(('WAITING', 'START_ALIGN_NO_SERVICE'), c.state())

    def test_no_service_fails_after_the_wait(self):
        f = Fixture()
        f.ready()
        f.core.on_services(False, f.now)
        for _ in range(9):
            f.tick()
        self.assertEqual(('WAITING', 'START_ALIGN_NO_SERVICE'), f.core.state())
        f.tick(1.0)
        self.assertEqual(('FAILED', 'START_ALIGN_NO_SERVICE'), f.core.state())


class Aligning(unittest.TestCase):
    def converge(self, f, path, sigma_path):
        """AMCL answers: one pose per no-motion update."""
        actions = []
        for (pose, (sxy, syaw)) in zip(path, sigma_path):
            actions += f.tick()
            f.core.on_amcl(pose, cov(sxy, syaw))
        actions += f.tick()
        return actions

    def test_first_respreads_around_the_amcl_pose_then_asks_updates(self):
        f = Fixture()
        f.ready(pose=(0.5, 0.2, 0.1))
        actions = f.tick()
        self.assertEqual('set_initial_pose', actions[0][0])
        _, (x, y, yaw), covariance = actions[0]
        self.assertEqual((0.5, 0.2, 0.1), (x, y, yaw))
        self.assertAlmostEqual(0.09, covariance[0])
        self.assertAlmostEqual(0.09, covariance[7])
        self.assertAlmostEqual(0.26 ** 2, covariance[35])
        self.assertEqual(('ALIGNING', ''), f.core.state())
        self.assertEqual([], f.tick())            # waits for the set_initial_pose answer
        f.core.on_initial_set()
        self.assertEqual([('nomotion_update',)], f.tick())

    def test_a_wrong_start_is_corrected_on_the_frozen_map(self):
        f = Fixture()
        truth = (0.3, 0.0, 0.0)
        f.ready(pose=(0.5, 0.15, 0.17), scan_pose=truth)      # measured 0.2 m and about 10 deg off
        f.tick()
        f.core.on_initial_set()
        path = [(0.42, 0.08, 0.09), (0.35, 0.03, 0.03), (0.305, 0.004, 0.003), (0.301, 0.001, 0.001),
                (0.300, 0.000, 0.000), (0.300, 0.000, 0.000)]
        sig = [(0.2, 0.2), (0.12, 0.1), (0.06, 0.05), (0.04, 0.03), (0.03, 0.02), (0.03, 0.02)]
        self.converge(f, path, sig)
        self.assertEqual(('ALIGNED', ''), f.core.state())
        result = f.core.result()
        self.assertEqual([0.3, 0.0, 0.0], [round(v, 3) for v in result['pose']])
        self.assertGreaterEqual(result['match_ratio'], 0.9)
        self.assertAlmostEqual(math.hypot(0.2, 0.15), result['jump_xy_m'], places=3)
        self.assertLessEqual(result['updates'], 20)

    def test_a_spread_that_stays_large_is_not_converged(self):
        f = Fixture(max_updates=5)
        f.ready()
        f.tick()
        f.core.on_initial_set()
        self.converge(f, [(0.5, 0.2, 0.1)] * 6, [(0.3, 0.2)] * 6)
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_CONVERGED'), f.core.state())

    def test_a_pose_still_moving_is_not_settled(self):
        f = Fixture(max_updates=5)
        f.ready()
        f.tick()
        f.core.on_initial_set()
        path = [(0.5 + 0.05 * i, 0.2, 0.1) for i in range(6)]
        self.converge(f, path, [(0.05, 0.03)] * 6)
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_CONVERGED'), f.core.state())

    def test_a_large_correction_is_refused(self):
        f = Fixture()
        f.ready(pose=(1.3, 0.2, 0.1), scan_pose=(0.3, 0.2, 0.1))   # measured 1 m off
        f.tick()
        f.core.on_initial_set()
        self.converge(f, [(0.3, 0.2, 0.1)] * 4, [(0.03, 0.02)] * 4)
        self.assertEqual(('FAILED', 'START_ALIGN_JUMP_TOO_LARGE'), f.core.state())

    def test_a_tight_pose_off_the_walls_is_low_match(self):
        f = Fixture()
        f.ready(pose=(0.5, 0.2, 0.1), scan_pose=(0.0, -0.6, 0.6))   # scan from elsewhere
        f.tick()
        f.core.on_initial_set()
        self.converge(f, [(0.5, 0.2, 0.1)] * 4, [(0.03, 0.02)] * 4)
        self.assertEqual(('FAILED', 'START_ALIGN_LOW_MATCH'), f.core.state())
        self.assertLess(f.core.result()['match_ratio'], 0.7)

    def test_motion_during_alignment_fails(self):
        f = Fixture()
        f.ready()
        f.tick()
        f.core.on_initial_set()
        f.tick()
        f.core.on_odom(0.0, 0.3)
        f.tick()
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_STOPPED'), f.core.state())

    def test_scans_stopping_during_alignment_fail(self):
        f = Fixture()
        f.ready()
        f.tick()
        f.core.on_initial_set()
        f.tick()
        f.now += 2 * S
        self.assertEqual([], f.core.tick(f.now))
        self.assertEqual(('FAILED', 'START_ALIGN_NO_SCAN'), f.core.state())

    def test_amcl_poses_before_the_respread_do_not_count(self):
        f = Fixture()
        f.ready()
        f.tick()
        for _ in range(4):                      # old particle cloud, before the set_initial_pose answer
            f.core.on_amcl((0.5, 0.2, 0.1), cov(0.0, 0.0))
        f.tick()
        self.assertEqual(('ALIGNING', ''), f.core.state())

    def test_a_final_state_stays_until_restart(self):
        f = Fixture(max_updates=1)
        f.ready()
        f.tick()
        f.core.on_initial_set()
        self.converge(f, [(0.5, 0.2, 0.1)] * 2, [(0.3, 0.2)] * 2)
        self.assertEqual('FAILED', f.core.state()[0])
        self.assertEqual([], f.tick())
        f.core.restart()
        self.assertEqual('WAITING', f.core.state()[0])
        self.assertEqual('set_initial_pose', f.tick()[0][0])


class FailedDetail(unittest.TestCase):
    """A failed alignment says where AMCL ended and why it did not settle (simulation 2026-10-16 run 3: the
    NOT_CONVERGED JSON had only first_guess and updates, so AMCL's last pose had to be fetched by hand).
    Only the result grows; the verdict and every value stay as they were."""

    def run_updates(self, path, sigma_path, **changes):
        f = Fixture(**changes)
        f.ready()
        f.tick()
        f.core.on_initial_set()
        Aligning.converge(None, f, path, sigma_path)
        return f

    def test_a_large_spread_gives_the_last_pose_spread_and_reason(self):
        f = self.run_updates([(0.5, 0.2, 0.1)] * 6, [(0.3, 0.2)] * 6, max_updates=5)
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_CONVERGED'), f.core.state())
        r = f.core.result()
        self.assertEqual([0.5, 0.2, 0.1], r['last_pose'])
        self.assertAlmostEqual(0.3, r['last_sigma_xy_m'])
        self.assertAlmostEqual(0.2, r['last_sigma_yaw_rad'])
        self.assertEqual(5, r['poses_received'])                 # max_updates 5: the sixth comes after FAILED
        self.assertEqual('SPREAD_ABOVE_ACCEPT', r['not_settled'])

    def test_a_moving_pose_gives_how_far_the_last_poses_moved(self):
        path = [(0.5 + 0.05 * i, 0.2, 0.1) for i in range(6)]
        f = self.run_updates(path, [(0.05, 0.03)] * 6, max_updates=5)
        r = f.core.result()
        self.assertEqual('POSE_STILL_MOVING', r['not_settled'])
        self.assertAlmostEqual(0.1, r['settle_spread_xy_m'])      # the last three poses
        self.assertAlmostEqual(0.0, r['settle_spread_yaw_rad'])
        self.assertEqual([0.7, 0.2, 0.1], [round(v, 6) for v in r['last_pose']])

    def test_no_amcl_pose_after_the_respread(self):
        f = Fixture(max_updates=2)
        f.ready()
        f.tick()
        f.core.on_initial_set()
        for _ in range(4):
            f.tick()
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_CONVERGED'), f.core.state())
        r = f.core.result()
        self.assertEqual(('NO_AMCL_POSE', None, 0), (r['not_settled'], r['last_pose'], r['poses_received']))

    def test_too_few_poses(self):
        f = self.run_updates([(0.5, 0.2, 0.1)], [(0.03, 0.02)], max_updates=1)
        self.assertEqual(('FAILED', 'START_ALIGN_NOT_CONVERGED'), f.core.state())
        self.assertEqual('TOO_FEW_POSES', f.core.result()['not_settled'])

    def test_aligned_result_is_unchanged(self):
        f = Fixture()
        f.ready(pose=(0.3, 0.0, 0.0))
        f.tick()
        f.core.on_initial_set()
        Aligning.converge(None, f, [(0.3, 0.0, 0.0)] * 3, [(0.03, 0.02)] * 3)
        self.assertEqual('ALIGNED', f.core.state()[0])
        self.assertFalse({'last_pose', 'not_settled', 'poses_received'} & set(f.core.result()))


class Scores(unittest.TestCase):
    def test_sigmas_use_the_largest_xy_axis(self):
        c = [0.0] * 36
        c[0], c[7], c[1], c[6], c[35] = 0.04, 0.01, 0.0, 0.0, 0.0025
        self.assertEqual((0.2, 0.05), tuple(round(v, 6) for v in sigmas(c)))

    def test_sigmas_take_rounding_below_zero_as_zero(self):
        # [E3 2026-10-15] Nav2 Jazzy AMCL with zero spread publishes yaw variance -5.7e-14
        c = [0.0] * 36
        c[35] = -5.728750807065726e-14
        self.assertEqual((0.0, 0.0), sigmas(c))

    def test_sigmas_refuse_a_broken_covariance(self):
        for bad in ([0.0] * 35, [float('nan')] + [0.0] * 35, [-1.0] + [0.0] * 35):
            with self.subTest(bad=bad[:1]), self.assertRaises(ValueError):
                sigmas(bad)

    def test_match_on_the_true_pose_and_off_it(self):
        grid, scan = room(), room_scan(0.3, 0.0, 0.0)
        hits, points = match_ratio(grid, scan, pose=(0.3, 0.0, 0.0), laser_in_base=(0, 0, 0), hit_m=0.1)
        self.assertEqual(180, points)
        self.assertGreaterEqual(hits / points, 0.95)
        hits, points = match_ratio(grid, scan, pose=(0.3, 0.4, 0.5), laser_in_base=(0, 0, 0), hit_m=0.1)
        self.assertLess(hits / points, 0.5)

    def test_the_laser_mount_is_applied(self):
        grid = room()
        scan = room_scan(0.3, 0.0, math.pi / 2)           # laser turned 90 deg on the robot
        hits, points = match_ratio(grid, scan, pose=(0.3, 0.0, 0.0), laser_in_base=(0, 0, math.pi / 2),
                                   hit_m=0.1)
        self.assertGreaterEqual(hits / points, 0.95)

    def test_invalid_ranges_are_not_points(self):
        scan = room_scan(0.0, 0.0, 0.0, beams=10)
        scan['ranges'][:4] = [float('inf'), float('nan'), 0.05, 20.0]
        self.assertEqual(6, match_ratio(room(), scan, pose=(0, 0, 0), laser_in_base=(0, 0, 0), hit_m=0.1)[1])


if __name__ == '__main__':
    unittest.main()
