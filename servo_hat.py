"""Waveshare PCA9685 Servo Driver HAT support."""

from __future__ import annotations

import math
import time

from smbus import SMBus

from omni_kinematics import clamp

PCA9685_ADDRESS = 0x40
PCA9685_BUS = 1
SERVO_CHANNEL = 0
SERVO_FREQUENCY_HZ = 50.0

# Calibrate these for the exact goBILDA positional servo before attaching a load.
# If the servo buzzes or pushes against a stop, reduce the range immediately.
SERVO_MIN_PULSE_US = 500.0   # -180 degrees
SERVO_CENTER_PULSE_US = 1500.0  # 0 degrees
SERVO_MAX_PULSE_US = 2500.0  # +180 degrees

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06


class PositionalServo:
    def __init__(self) -> None:
        self.bus = SMBus(PCA9685_BUS)
        self.angle = 0.0
        self._last_pulse_us: float | None = None
        self._initialize_pca9685()
        self.set_angle(0.0, force=True)

    def _initialize_pca9685(self) -> None:
        self.bus.write_byte_data(PCA9685_ADDRESS, MODE1, 0x00)
        prescale = int(round(25_000_000.0 / (4096.0 * SERVO_FREQUENCY_HZ)) - 1)
        old_mode = self.bus.read_byte_data(PCA9685_ADDRESS, MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(PCA9685_ADDRESS, MODE1, sleep_mode)
        self.bus.write_byte_data(PCA9685_ADDRESS, PRESCALE, prescale)
        self.bus.write_byte_data(PCA9685_ADDRESS, MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(PCA9685_ADDRESS, MODE1, old_mode | 0xA1)

    @staticmethod
    def angle_to_pulse_us(angle: float) -> float:
        angle = clamp(angle, -180.0, 180.0)
        if angle <= 0.0:
            fraction = (angle + 180.0) / 180.0
            return SERVO_MIN_PULSE_US + fraction * (
                SERVO_CENTER_PULSE_US - SERVO_MIN_PULSE_US
            )
        fraction = angle / 180.0
        return SERVO_CENTER_PULSE_US + fraction * (
            SERVO_MAX_PULSE_US - SERVO_CENTER_PULSE_US
        )

    def _write_pulse(self, pulse_us: float) -> None:
        ticks = int(round(pulse_us * SERVO_FREQUENCY_HZ * 4096.0 / 1_000_000.0))
        ticks = int(clamp(ticks, 0, 4095))
        register = LED0_ON_L + 4 * SERVO_CHANNEL
        payload = [0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F]
        self.bus.write_i2c_block_data(PCA9685_ADDRESS, register, payload)

    def set_angle(self, angle: float, force: bool = False) -> None:
        self.angle = clamp(angle, -180.0, 180.0)
        pulse_us = self.angle_to_pulse_us(self.angle)
        if force or self._last_pulse_us is None or not math.isclose(
            pulse_us, self._last_pulse_us, abs_tol=0.5
        ):
            self._write_pulse(pulse_us)
            self._last_pulse_us = pulse_us

    def center(self) -> None:
        self.set_angle(0.0)

    def close(self) -> None:
        self.bus.close()
