"""ObserveTarget accepts one window at a time, reserved at goal acceptance (#165 review CDX-165-03, 2026-10-13).

The window used to be created only when the goal executed, so two goals accepted before that could both
run and overwrite one window. The slot is now taken in the accept callback and released when the goal
ends however it ends.
"""
import unittest

from rescue_perception.recheck_server import WindowSlot


class WindowSlotTests(unittest.TestCase):
    def test_second_goal_is_refused_until_the_first_ends(self):
        slot = WindowSlot()
        self.assertTrue(slot.reserve())
        self.assertFalse(slot.reserve())
        slot.release()
        self.assertTrue(slot.reserve())

    def test_release_is_safe_to_repeat(self):
        slot = WindowSlot()
        slot.release()
        self.assertTrue(slot.reserve())
        slot.release()
        slot.release()
        self.assertTrue(slot.reserve())


if __name__ == '__main__':
    unittest.main()
