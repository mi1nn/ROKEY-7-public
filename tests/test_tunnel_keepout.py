"""Tunnels are handled by the vision height judgement, not by a depth stop (재범 decision 2026-10-16 01:4x / 01:5x,
option A, grade 4; no ROS).

C1 (#363) and run 4: PassageStop (depth points 0.05-0.46 m, 0.4-1.0 m ahead, action stop) held TB-6 in front
of walls and the dock, turn included. 재범: 「터널 지나갈 때 비전으로 터널 높이 측정해서 지나가는 로직 짰잖아」 then
「A로 진행」. So:
- PassageStop is removed from collision_monitor. PassageSlow stays (it buys the judgement time; slowing in
  front of a wall is accepted).
- overhead_passage's judgement moves the robots: rescue_robot.passage_keepout paints each BLOCKED place (the
  newest item per place over both robots and every map revision) into a Nav2 keepout mask, read by the global
  and local costmaps of TB-6 and TB-5. PASSABLE is not painted. UNKNOWN is not painted (the robot keeps
  approaching slowly and measures again) unless the structure reached the box's near end without a floor seen
  (reason NO_FLOOR_NEAR).
- A person can approve a painted place (/rescue/passage_keepout/approve); it leaves the mask for the run.

Before this change (b34972e) these tests were expectedFailure.
"""
import json
import math
import unittest
from collections import deque


from test_bringup_field_launch import FieldCase
from test_bringup_passage import BASE
from test_bringup_passage_deadlock import FORWARD, TURN, safe_velocity

SESSION = '22222222-2222-4222-8222-222222222222'
TUNNEL_2 = (-6.0, 1.0)   # cover 0.30 m: BLOCKED
TUNNEL_1 = (-4.0, 1.0)   # cover 0.47 m: PASSABLE
# Run 4: the west wall cuts the stop box's front left corner while TB-6 turns (x = 1.2 - 1.25 y), 0.06-0.22 m
WALL_POINTS = [(1.2 - 1.25 * y, y, z) for y in (0.16 + 0.005 * k for k in range(9)) for z in (0.06, 0.10, 0.14, 0.18, 0.22)]


def item(item_id, state, xy, *, revision=0, seen=1, reason='', lowest=None):
    lowest = {'BLOCKED': 0.30, 'PASSABLE': 0.47}.get(state) if lowest is None else lowest
    return dict(id=item_id, state=state, lowest_m=lowest, x=xy[0], y=xy[1], map_session_id=SESSION,
                map_revision=revision, frames=3, first_seen_ns=seen, last_seen_ns=seen, reason=reason)


def ids(n):
    return f'00000000-0000-4000-8000-{n:012d}'


def keepout(**changes):
    from rescue_robot.passage_keepout import KeepoutCore
    return KeepoutCore(**dict(dict(radius_m=0.3, resolution_m=0.05, merge_m=0.5, map_frame='map'), **changes))


def blocked_cells(mask):
    """World centres of the mask cells marked keepout (100)."""
    out = []
    for j in range(mask['height']):
        for i in range(mask['width']):
            if mask['data'][j * mask['width'] + i] == 100:
                out.append((mask['origin'][0] + (i + 0.5) * mask['resolution'],
                            mask['origin'][1] + (j + 0.5) * mask['resolution']))
    return out


def path_through(mask, start, goal, step=0.05):
    """A 4-connected grid path (as Nav2's planner on lethal cells) from start to goal along a corridor y = 1.0
    +- 0.2 m between x = -7 and -3: the only way passes both tunnels. None when the keepout closes it."""
    def cell(w):   # the grid cell holding w (mask cell centres sit in the middle of these cells)
        return math.floor(w / step + 1e-9)
    blocked = {(cell(x), cell(y)) for x, y in blocked_cells(mask)}
    lo, hi = (cell(-7.0), cell(0.8)), (cell(-3.0), cell(1.2))
    s, g = (cell(start[0]), cell(start[1])), (cell(goal[0]), cell(goal[1]))
    queue, seen = deque([s]), {s: None}
    while queue:
        c = queue.popleft()
        if c == g:
            path = []
            while c is not None:
                path.append((c[0] * step, c[1] * step))
                c = seen[c]
            return path[::-1]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if lo[0] <= n[0] <= hi[0] and lo[1] <= n[1] <= hi[1] and n not in seen and n not in blocked:
                seen[n] = c
                queue.append(n)
    return None


class DepthStopRemoved(FieldCase):
    def setUp(self):
        super().setUp()
        from rescue_bringup.field_launch import nav2_with_passage
        passage = self.robot(field=self.field(), robot_id='robot_a', occupancy_sha256='')['passage']
        self.cm = nav2_with_passage(json.loads(json.dumps(BASE)), passage)['collision_monitor']['ros__parameters']

    def test_run4_west_wall_in_the_box_corner_does_not_stop(self):
        self.assertGreaterEqual(len(WALL_POINTS), 38)
        self.assertGreater(safe_velocity(self.cm, WALL_POINTS, FORWARD)[0], 0.0)   # PassageSlow may slow it
        self.assertGreater(safe_velocity(self.cm, WALL_POINTS, TURN)[2], 0.0)

    def test_no_passage_stop_polygon_and_passage_slow_stays(self):
        self.assertNotIn('PassageStop', self.cm['polygons'])
        self.assertIn('PassageSlow', self.cm['polygons'])
        self.assertEqual('slowdown', self.cm['PassageSlow']['action_type'])


class KeepoutMask(unittest.TestCase):
    def test_tunnel_2_blocked_closes_the_path_and_tunnel_1_passable_leaves_it(self):
        k = keepout()
        start, goal = (-6.8, 1.0), (-3.2, 1.0)
        self.assertIsNotNone(path_through(k.mask(), start, goal))                  # nothing judged yet
        k.on_items('robot_a', [item(ids(1), 'PASSABLE', TUNNEL_1)])
        self.assertIsNotNone(path_through(k.mask(), start, goal))                  # tunnel 1 is passed
        k.on_items('robot_a', [item(ids(1), 'PASSABLE', TUNNEL_1), item(ids(2), 'BLOCKED', TUNNEL_2, seen=2)])
        self.assertIsNone(path_through(k.mask(), start, goal))                     # tunnel 2 is not
        self.assertIsNotNone(path_through(k.mask(), (-5.6, 1.0), goal))            # past tunnel 2, tunnel 1 is open

    def test_unknown_is_painted_only_when_it_reached_the_near_end(self):
        k = keepout()
        k.on_items('robot_a', [item(ids(3), 'UNKNOWN', TUNNEL_2, reason='NO_FLOOR_SEEN')])
        self.assertEqual([], blocked_cells(k.mask()))
        k.on_items('robot_a', [item(ids(3), 'UNKNOWN', TUNNEL_2, reason='NO_FLOOR_NEAR')])
        self.assertTrue(blocked_cells(k.mask()))

    def test_the_newest_item_of_a_place_wins_over_robots_and_revisions(self):
        k = keepout()
        k.on_items('robot_a', [item(ids(4), 'BLOCKED', TUNNEL_1, revision=0, seen=10, lowest=0.455)])   # noise, R0
        k.on_items('robot_b', [item(ids(5), 'PASSABLE', (TUNNEL_1[0] + 0.1, TUNNEL_1[1]), revision=1, seen=20)])
        self.assertEqual([], blocked_cells(k.mask()))
        k.on_items('robot_b', [item(ids(5), 'BLOCKED', (TUNNEL_1[0] + 0.1, TUNNEL_1[1]), revision=1, seen=30,
                                    lowest=0.45)])
        self.assertTrue(blocked_cells(k.mask()))

    def test_an_approved_place_leaves_the_mask_and_the_approval_is_logged(self):
        k = keepout()
        k.on_items('robot_a', [item(ids(6), 'BLOCKED', TUNNEL_1, lowest=0.455)])
        self.assertTrue(blocked_cells(k.mask()))
        ok, line = k.approve(ids(6), by='재범', now_ns=99)
        self.assertTrue(ok, line)
        self.assertIn('재범', line)
        self.assertIn(ids(6), line)
        self.assertEqual([], blocked_cells(k.mask()))
        k.on_items('robot_b', [item(ids(7), 'BLOCKED', (TUNNEL_1[0], TUNNEL_1[1] + 0.2), revision=1, seen=50)])
        self.assertEqual([], blocked_cells(k.mask()))                              # the place stays approved
        for bad_id, by in ((ids(99), '재범'), (ids(6), '')):
            with self.subTest(bad_id=bad_id, by=by):
                self.assertFalse(k.approve(bad_id, by=by, now_ns=100)[0])

    def test_disc_radius_and_mask_frame(self):
        k = keepout()
        k.on_items('robot_a', [item(ids(8), 'BLOCKED', TUNNEL_2)])
        mask = k.mask()
        self.assertEqual(('map', 0.05), (mask['frame_id'], mask['resolution']))
        cells = blocked_cells(mask)
        far = max(((x - TUNNEL_2[0]) ** 2 + (y - TUNNEL_2[1]) ** 2) ** 0.5 for x, y in cells)
        self.assertLessEqual(far, 0.3)
        self.assertGreater(far, 0.25)


class ApproveService(unittest.TestCase):
    """/rescue/passage_keepout/approve (rescue_interfaces/srv/ApproveOverheadPassage, SDD JSON first, #400).
    The console button calls it with approved_by = the logged-in account name."""

    @unittest.expectedFailure
    def test_answers_with_the_srv_reason_codes(self):
        from rescue_robot.passage_keepout import answer
        k = keepout()
        k.on_items('robot_a', [item(ids(6), 'BLOCKED', TUNNEL_1, lowest=0.455)])
        self.assertEqual((False, 'NO_APPROVER'), answer(k, ids(6), '  ', now_ns=1)[:2])
        self.assertEqual((False, 'NO_SUCH_ITEM'), answer(k, ids(99), '재범', now_ns=2)[:2])
        accepted, code, message = answer(k, ids(6), '재범', now_ns=3)
        self.assertEqual((True, 'ACCEPTED'), (accepted, code))
        self.assertTrue(message.startswith(f'PASSAGE_APPROVED {ids(6)} by 재범 at 3 ns'), message)
        self.assertEqual([], blocked_cells(k.mask()))

    @unittest.expectedFailure
    def test_the_node_offers_the_service_with_the_srv_type(self):
        from test_bringup_no_backup import REPO
        path = REPO / 'src' / 'rescue_robot' / 'rescue_robot' / 'passage_keepout.py'
        if not path.exists():
            self.skipTest('installed-wheel checkout has no src/')
        text = path.read_text()
        self.assertIn("APPROVE_SERVICE = '/rescue/passage_keepout/approve'", text)
        self.assertIn('node.create_service(ApproveOverheadPassage, APPROVE_SERVICE, on_approve)', text)


class Nav2Keepout(unittest.TestCase):
    def test_both_costmaps_get_the_keepout_filter(self):
        from rescue_bringup.nav2_field import KEEPOUT_INFO_TOPIC, nav2_with_tunnel_keepout
        from test_bringup_no_backup import vendor_params
        params = nav2_with_tunnel_keepout(vendor_params())
        for costmap in ('global_costmap', 'local_costmap'):
            with self.subTest(costmap=costmap):
                p = params[costmap][costmap]['ros__parameters']
                self.assertEqual(['tunnel_keepout'], p['filters'])
                self.assertEqual({'plugin': 'nav2_costmap_2d::KeepoutFilter', 'enabled': True,
                                  'filter_info_topic': KEEPOUT_INFO_TOPIC}, p['tunnel_keepout'])


class NearEndUnknown(unittest.TestCase):
    def test_a_structure_at_the_near_end_without_a_floor_is_no_floor_near(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('numpy not installed (installed-wheel check)')
        import dataclasses
        from rescue_perception.overhead import judge
        from test_perception_overhead import POLICY
        policy = dataclasses.replace(POLICY, near_end_m=0.05)
        cover = [(x, y, 0.35) for x in np.arange(0.40, 0.70, 0.02) for y in np.arange(-0.15, 0.16, 0.05)]
        far_cover = [(x, y, 0.35) for x in np.arange(0.70, 0.95, 0.02) for y in np.arange(-0.15, 0.16, 0.05)]
        near = judge(np.array(cover), policy)
        self.assertEqual(('UNKNOWN', 'NO_FLOOR_NEAR'), (near.state, near.reason))
        self.assertEqual('NO_FLOOR_SEEN', judge(np.array(far_cover), policy).reason)

if __name__ == '__main__':
    unittest.main()
