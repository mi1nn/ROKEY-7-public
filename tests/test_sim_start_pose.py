"""scripts/sim/start_pose.py: TB-5's start value for the simulation copy of the field file (run_full.sh).

Field: after both robots undock, TB-5's position seen from TB-6's body is measured (tape, 1.86 m) and goes into
robots.robot_b.initial_pose_xy_yaw. Simulation: the same measurement from the ground-truth poses, never from
"start place + odom" (3rd run 2026-10-16: the undock turn left the simulated odom 11.5 deg off, the value came out
0.4 m wrong and TB-5's start alignment did not converge, #386).
"""
import math
from pathlib import Path
import sys
import unittest

SIM = Path(__file__).resolve().parents[1] / 'scripts' / 'sim'
sys.path.insert(0, str(SIM))
HAS_SIM = (SIM / 'run_full.sh').exists()   # the installed-wheel check copies tests/ and config/ only

# 3rd run, after undocking [측정 2026-10-16, #386]: ground truth (arena) and TB-6 odom
GT6 = (0.880, 0.722, math.radians(89.5))
GT5 = (2.700, 0.722, math.radians(89.5))
ODOM6 = (0.0, 0.35, math.radians(101.0 - 89.5 + 90.0) - math.pi / 2)   # odom turned 11.5 deg more than the robot


@unittest.skipUnless(HAS_SIM, 'installed-wheel checkout has no scripts/')
class StartPose(unittest.TestCase):
    def test_tb5_seen_from_tb6_body_after_undocking(self):
        import start_pose
        x, y, yaw = start_pose.relative(GT6, GT5)
        self.assertAlmostEqual(0.016, x, places=3)    # as #386 computed from the true heading
        self.assertAlmostEqual(-1.82, y, places=3)    # 1.82 m to TB-6's right (field tape 1.86 m)
        self.assertAlmostEqual(0.0, yaw, places=4)

    def test_a_pose_moves_into_the_tb6_odom_frame_with_the_odom_turn(self):
        import start_pose
        # TB-6 itself lands exactly on its odom pose
        x, y, yaw = start_pose.in_odom_frame(GT6, ODOM6, GT6)
        self.assertAlmostEqual(ODOM6[0], x, places=6)
        self.assertAlmostEqual(ODOM6[1], y, places=6)
        self.assertAlmostEqual(ODOM6[2], yaw, places=6)
        # and TB-5 keeps its true distance from TB-6
        x5, y5, _ = start_pose.in_odom_frame(GT6, ODOM6, GT5)
        self.assertAlmostEqual(1.82, math.hypot(x5 - ODOM6[0], y5 - ODOM6[1]), places=6)

    def test_run_full_takes_the_value_from_the_ground_truth_not_from_odom(self):
        text = (SIM / 'run_full.sh').read_text()
        self.assertIn('/robot5/sim_ground_truth_pose', text)
        self.assertIn('start_pose.py', text)


if __name__ == '__main__':
    unittest.main()
