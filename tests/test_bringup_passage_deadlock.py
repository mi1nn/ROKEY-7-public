"""PassageStop deadlock (simulation 2026-10-15 22:2x; no ROS).

TB-6 stood 120 s with the TB-5 dock (about 0.1 m high) 0.91-1.0 m ahead: robot.launch logged
PASSAGE_BLOCKED polygon='PassageStop' action=1 and never moved again. The passage band in the field
file (0.05-0.46 m high, 0.4-1.0 m ahead) is there for the overhead passage the lidar cannot see, but
the dock, the white boxes and the 0.41 m practice walls fall in it too. The polygon action is stop,
and Jazzy collision_monitor stop zeroes the turn as well, so the robot cannot turn away.

safe_velocity() below follows Jazzy collision_monitor for polygons (nav2_collision_monitor
collision_monitor_node.cpp, jazzy branch):
- stop: x = y = tw = 0.
- slowdown: the whole command times slowdown_ratio.
- limit: one ratio for the whole command, min(1, linear_limit / |v|, angular_limit / |tw|), each term
  only when that part of the command is not zero. With linear_limit 0 a pure turn passes and any
  command with forward speed becomes 0.
- several polygons: the slowest result wins.
A polygon acts when at least min_points points of its sources are inside it, with z between the
source min_height and max_height (base_link). The scan source is not modelled (no scan points here).

재범 chose another way (2026-10-16, option A, grade 4): PassageStop is removed and tunnels are kept out by the
vision judgement (overhead_passage -> rescue_robot.passage_keepout -> Nav2 keepout mask, test_tunnel_keepout).
The tests below now describe that: nothing in the depth band stops the robot; above the lidar plane it slows.
"""
import json
import unittest

from test_bringup_field_launch import FieldCase
from test_bringup_passage import BASE

TURN = (0.0, 0.0, 1.0)        # in place, 1.0 rad/s (Rotation Shim rotate speed)
FORWARD = (0.2, 0.0, 0.0)     # RPP desired_linear_vel 0.2 m/s


def inside(points, x, y):
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)   # PassageStop/PassageSlow are rectangles


def safe_velocity(cm, cloud, command):
    """Jazzy collision_monitor polygon actions on one depth cloud (points in base_link)."""
    vx, vy, tw = command
    best = command
    for name in cm['polygons']:
        polygon = cm[name]
        if polygon.get('type') != 'polygon' or polygon['action_type'] not in ('stop', 'slowdown', 'limit'):
            continue
        sources = [cm[s] for s in polygon['sources_names'] if cm[s]['type'] == 'pointcloud']
        corners = json.loads(polygon['points'])
        hits = sum(1 for s in sources for x, y, z in cloud
                   if s['min_height'] <= z <= s['max_height'] and inside(corners, x, y))
        if hits < polygon['min_points']:
            continue
        if polygon['action_type'] == 'stop':
            result = (0.0, 0.0, 0.0)
        else:
            if polygon['action_type'] == 'slowdown':
                ratio = polygon['slowdown_ratio']
            else:
                ratio, linear = 1.0, (vx * vx + vy * vy) ** 0.5
                if linear != 0.0:
                    ratio = min(ratio, polygon['linear_limit'] / linear)
                if tw != 0.0:
                    ratio = min(ratio, polygon['angular_limit'] / abs(tw))
            result = (vx * ratio, vy * ratio, tw * ratio)
        if abs(result[0]) + abs(result[1]) + abs(result[2]) < abs(best[0]) + abs(best[1]) + abs(best[2]):
            best = result
    return best


def plate(x_near, x_far, half_width, z, step=0.05):
    """Depth points on a vertical face or a bar: a grid x_near..x_far, -half_width..half_width at height z."""
    n_x, n_y = int(round((x_far - x_near) / step)) + 1, int(round(2 * half_width / step)) + 1
    return [(x_near + i * step, -half_width + j * step, z) for i in range(n_x) for j in range(n_y)]


DOCK = plate(0.91, 1.0, 0.15, 0.10)                                     # simulation: TB-5 dock, top 0.1 m
WALL = [p for z in (0.10, 0.20, 0.30, 0.40) for p in plate(0.8, 0.8, 0.3, z)]   # 0.41 m MDF wall 0.8 m ahead
TUNNEL = plate(0.6, 0.7, 0.3, 0.35)    # an overhead bar 0.35 m up: above the lidar (0.23), below 0.46


class FieldPassage(FieldCase):
    """The repository field file's passage values put into collision_monitor as robot.launch.py does."""

    def setUp(self):
        super().setUp()
        from rescue_bringup.field_launch import nav2_with_passage
        passage = self.robot(field=self.field(), robot_id='robot_a', occupancy_sha256='')['passage']
        self.cm = nav2_with_passage(json.loads(json.dumps(BASE)), passage)['collision_monitor']['ros__parameters']

    def test_the_dock_no_longer_stops_the_robot(self):
        """The simulation case: the dock (below the lidar plane and the slow band) changes nothing now."""
        self.assertEqual(TURN, safe_velocity(self.cm, DOCK, TURN))
        self.assertEqual(FORWARD, safe_velocity(self.cm, DOCK, FORWARD))

    def test_an_overhead_bar_slows_the_robot_and_the_keepout_keeps_it_out(self):
        """No depth stop: PassageSlow slows it so overhead_passage can judge; a BLOCKED judgement goes into the
        Nav2 keepout mask (test_tunnel_keepout), which keeps the path out of it."""
        self.assertAlmostEqual(0.2 * 0.3, safe_velocity(self.cm, TUNNEL, FORWARD)[0])

    def test_facing_the_dock_the_robot_can_turn_in_place(self):
        self.assertGreater(safe_velocity(self.cm, DOCK, TURN)[2], 0.0)

    def test_facing_a_wall_the_robot_can_turn_in_place(self):
        self.assertGreater(safe_velocity(self.cm, WALL, TURN)[2], 0.0)

    def test_under_an_overhead_bar_the_robot_can_turn_in_place(self):
        """Turning away from a real low passage lets Nav2 try another way or give up with PASSAGE_BLOCKED."""
        self.assertGreater(safe_velocity(self.cm, TUNNEL, TURN)[2], 0.0)


class Model(unittest.TestCase):
    """safe_velocity() itself, on hand-made polygons (Jazzy rules in the module docstring)."""

    def cm(self, **polygon):
        box = json.dumps([[1.0, 0.2], [1.0, -0.2], [0.4, -0.2], [0.4, 0.2]])
        return {'polygons': ['P'], 'P': dict({'type': 'polygon', 'points': box, 'min_points': 5,
                                              'sources_names': ['d']}, **polygon),
                'd': {'type': 'pointcloud', 'min_height': 0.05, 'max_height': 0.46}}

    def test_limit_with_no_forward_speed_lets_a_pure_turn_through(self):
        cm = self.cm(action_type='limit', linear_limit=0.0, angular_limit=1.0)
        self.assertEqual(TURN, safe_velocity(cm, DOCK, TURN))
        self.assertEqual((0.0, 0.0, 0.0), safe_velocity(cm, DOCK, FORWARD))
        self.assertEqual((0.0, 0.0, 0.0), safe_velocity(cm, DOCK, (0.2, 0.0, 0.5)))   # an arc stops too

    def test_too_few_points_or_out_of_band_points_do_nothing(self):
        cm = self.cm(action_type='stop')
        self.assertEqual(FORWARD, safe_velocity(cm, DOCK[:4], FORWARD))
        self.assertEqual(FORWARD, safe_velocity(cm, [(x, y, 0.5) for x, y, _ in DOCK], FORWARD))
        self.assertEqual(FORWARD, safe_velocity(cm, [(x + 1.0, y, z) for x, y, z in DOCK], FORWARD))

    def test_slowdown_scales_the_whole_command(self):
        cm = self.cm(action_type='slowdown', slowdown_ratio=0.3)
        self.assertEqual((0.06, 0.0, 0.0), tuple(round(v, 6) for v in safe_velocity(cm, DOCK, FORWARD)))


if __name__ == '__main__':
    unittest.main()
