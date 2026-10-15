#!/usr/bin/env python3
"""Start values for the simulation copy of the field file (run_full.sh, 2026-10-16). Poses are (x, y, yaw radians).

    start_pose.py relative  <tb6 x y yaw> <tb5 x y yaw>                   # TB-5 seen from TB-6's body
    start_pose.py in-frame  <tb6 truth x y yaw> <tb6 odom x y yaw> <x y yaw>   # an arena pose in TB-6's odom frame

The field measures TB-5 from TB-6's body after both undock (tape). The simulation does that measurement with the
ground-truth poses: "start place + odom" went 0.4 m wrong in the 3rd run because the simulated odom turned 11.5 deg
more than the robot while undocking (#386). Standard library only.
"""
import math
import sys


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(tb6, tb5):
    """TB-5's pose in TB-6's body frame (both arena poses)."""
    dx, dy = tb5[0] - tb6[0], tb5[1] - tb6[1]
    c, s = math.cos(-tb6[2]), math.sin(-tb6[2])
    return c * dx - s * dy, s * dx + c * dy, _wrap(tb5[2] - tb6[2])


def in_odom_frame(tb6_truth, tb6_odom, pose):
    """An arena pose in TB-6's odom frame. The frame is fixed by TB-6's truth and odom poses at the same moment,
    so the odom's own turn is taken in (TB-6's map frame starts as its odom frame)."""
    fw = tb6_truth[2] - tb6_odom[2]
    fx = tb6_truth[0] - (math.cos(fw) * tb6_odom[0] - math.sin(fw) * tb6_odom[1])
    fy = tb6_truth[1] - (math.sin(fw) * tb6_odom[0] + math.cos(fw) * tb6_odom[1])
    dx, dy = pose[0] - fx, pose[1] - fy
    c, s = math.cos(-fw), math.sin(-fw)
    return c * dx - s * dy, s * dx + c * dy, _wrap(pose[2] - fw)


def main(argv):
    try:
        values = [float(v) for v in argv[1:]]
        if argv[0] == 'relative' and len(values) == 6:
            out = relative(values[0:3], values[3:6])
        elif argv[0] == 'in-frame' and len(values) == 9:
            out = in_odom_frame(values[0:3], values[3:6], values[6:9])
        else:
            raise ValueError('usage: relative <6 numbers> | in-frame <9 numbers>')
    except (ValueError, IndexError) as exc:
        print(f'start_pose: {exc}', file=sys.stderr)
        return 2
    print(round(out[0], 3), round(out[1], 3), round(out[2], 4))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
