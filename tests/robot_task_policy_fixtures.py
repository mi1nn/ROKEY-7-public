"""Synthetic clock/lease policy for E2; these are not deployment defaults."""
from rescue_robot.task_policy import ClockBasis, LeasePolicy

POLICY = LeasePolicy(2., 1_000_000_000, 100_000_000, 'test-clock')


def clock_basis(adapter):
    return ClockBasis('test-clock', 'SYNCED', 10_000_000, 'test-clock-session',
                      adapter.wall_clock_ns() + 60_000_000_000)
