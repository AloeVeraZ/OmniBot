import math
import unittest

from omni_kinematics import axis_deadzone, mix_three_omni, radial_deadzone


class KinematicsTests(unittest.TestCase):
    def assertPowers(self, actual, expected):
        for left, right in zip(actual, expected):
            self.assertAlmostEqual(left, right, places=6)

    def test_neutral_is_stopped(self):
        self.assertPowers(mix_three_omni(0, 0, 0), (0, 0, 0))

    def test_forward_uses_rear_pair(self):
        self.assertPowers(mix_three_omni(0, 1, 0), (0, -1, 1))

    def test_strafe_uses_three_wheels(self):
        self.assertPowers(mix_three_omni(1, 0, 0), (-1, 0.5, 0.5))

    def test_turn_uses_all_wheels_equally(self):
        self.assertPowers(mix_three_omni(0, 0, 1), (1, 1, 1))

    def test_combined_command_is_bounded(self):
        values = mix_three_omni(1, 1, 1)
        self.assertLessEqual(max(map(abs, values)), 1.0)
        self.assertAlmostEqual(max(map(abs, values)), 1.0)

    def test_radial_deadzone_preserves_direction(self):
        x, y = radial_deadzone(0.3, 0.4, 0.1)
        self.assertAlmostEqual(x / y, 0.75)
        self.assertAlmostEqual(math.hypot(x, y), (0.5 - 0.1) / 0.9)

    def test_axis_deadzone_remaps_continuously(self):
        self.assertEqual(axis_deadzone(0.1, 0.15), 0)
        self.assertAlmostEqual(axis_deadzone(1, 0.15), 1)
        self.assertAlmostEqual(axis_deadzone(-1, 0.15), -1)


if __name__ == "__main__":
    unittest.main()

