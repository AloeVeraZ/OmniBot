import math
import unittest

from omni_kinematics import (
    THREE_OMNI_MOTOR_SIGNS,
    axis_deadzone,
    cardinal_lock,
    controller_drive_axes,
    mix_three_omni,
    next_servo_angle,
    radial_deadzone,
    shape_motor_power,
    trigger_activation,
)


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

    def test_physical_forward_stops_front_and_drives_both_rears(self):
        mixed = mix_three_omni(0, 1, 0)
        physical = tuple(
            power * sign for power, sign in zip(mixed, THREE_OMNI_MOTOR_SIGNS)
        )
        self.assertPowers(physical, (0, -1, -1))

    def test_forward_axis_lock_keeps_front_motor_off(self):
        strafe, forward = cardinal_lock(1.0, 0.21)
        self.assertPowers((strafe, forward), (0, 0.21))
        mixed = mix_three_omni(strafe, forward, 0)
        self.assertEqual(mixed[0], 0)

        strafe, forward = cardinal_lock(0.20, 1.0)
        self.assertPowers((strafe, forward), (0, 1.0))
        mixed = mix_three_omni(strafe, forward, 0)
        self.assertEqual(mixed[0], 0)

        strafe, forward = cardinal_lock(-0.20, -1.0)
        self.assertPowers((strafe, forward), (0, -1.0))
        mixed = mix_three_omni(strafe, forward, 0)
        self.assertEqual(mixed[0], 0)

    def test_horizontal_axis_lock_uses_all_three_motors(self):
        strafe, forward = cardinal_lock(1.0, 0.20)
        self.assertPowers((strafe, forward), (1.0, 0))
        mixed = mix_three_omni(strafe, forward, 0)
        self.assertTrue(all(abs(power) > 0 for power in mixed))

    def test_controller_horizontal_axes_are_inverted(self):
        self.assertPowers(controller_drive_axes(1, 0, 0, 0), (-1, 0, 0))
        self.assertPowers(controller_drive_axes(0, -1, 0, 0), (0, 1, 0))
        self.assertPowers(controller_drive_axes(0, 0, 1, 0), (0, 0, 1))

    def test_right_stick_sideways_and_diagonal_input_cannot_turn(self):
        self.assertPowers(controller_drive_axes(0, 0, 0, -1), (0, 0, 0))
        self.assertPowers(controller_drive_axes(0, 0, 0, 1), (0, 0, 0))
        self.assertPowers(controller_drive_axes(0, 0, 0.25, 1), (0, 0, 0))
        self.assertPowers(controller_drive_axes(0, 0, -0.25, -1), (0, 0, 0))
        self.assertPowers(controller_drive_axes(0, 0, 1, 0.2), (0, 0, 1))

    def test_full_forward_and_reverse_hold_true_full_power(self):
        for forward in (-1.0, 1.0):
            mixed = mix_three_omni(0, forward, 0)
            physical = tuple(
                power * sign
                for power, sign in zip(mixed, THREE_OMNI_MOTOR_SIGNS)
            )
            duties = tuple(shape_motor_power(power) for power in physical)
            self.assertEqual(duties[0], 0)
            self.assertEqual(abs(duties[1]), 1.0)
            self.assertEqual(abs(duties[2]), 1.0)

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

    def test_motor_power_jumps_to_usable_range(self):
        self.assertEqual(shape_motor_power(0), 0)
        self.assertGreaterEqual(shape_motor_power(0.0001, 0.75, 1.0), 0.75)
        self.assertAlmostEqual(
            shape_motor_power(0.325, 0.75, 1.0, 0.65), 0.875
        )
        self.assertEqual(shape_motor_power(0.65, 0.75, 1.0, 0.65), 1.0)
        self.assertEqual(shape_motor_power(0.90, 0.75, 1.0, 0.65), 1.0)
        self.assertAlmostEqual(shape_motor_power(1, 0.75, 1.0), 1.0)
        self.assertAlmostEqual(shape_motor_power(-1, 0.75, 1.0), -1.0)

    def test_trigger_activation_supports_common_axis_ranges(self):
        self.assertEqual(trigger_activation(-1, -1), 0)
        self.assertAlmostEqual(trigger_activation(0, -1), 0.5)
        self.assertEqual(trigger_activation(1, -1), 1)
        self.assertEqual(trigger_activation(0, 0), 0)
        self.assertEqual(trigger_activation(1, 0), 1)

    def test_servo_trigger_rate_and_hard_limits(self):
        self.assertEqual(next_servo_angle(0, 1, 0, 0.5, 120), -60)
        self.assertEqual(next_servo_angle(0, 0, 1, 0.5, 120), 60)
        self.assertEqual(next_servo_angle(-149, 1, 0, 1, 120), -150)
        self.assertEqual(next_servo_angle(149, 0, 1, 1, 120), 150)
        self.assertEqual(next_servo_angle(20, 1, 1, 1, 120), 20)

    def test_full_trigger_commands_endpoint_quickly(self):
        self.assertEqual(next_servo_angle(0, 1, 0, 1, 1000), -150)
        self.assertEqual(next_servo_angle(0, 0, 1, 1, 1000), 150)


if __name__ == "__main__":
    unittest.main()
