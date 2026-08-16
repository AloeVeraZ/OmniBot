#!/usr/bin/env python3
"""Three-motor Raspberry Pi omni robot with a pygame controller UI."""

from __future__ import annotations

import sys
import time

import pygame
import RPi.GPIO as GPIO

from omni_kinematics import axis_deadzone, clamp, mix_three_omni, radial_deadzone

# Generic Bluetooth controller mapping (kept from the original program).
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 3
BUTTON_A_INDEX = 0
BUTTON_Y_INDEX = 1

STICK_DEADZONE = 0.15
TURN_DEADZONE = 0.15
ARM_NEUTRAL_LIMIT = 0.18
ARM_NEUTRAL_SECONDS = 0.25

# BOARD pin numbering, matching the supplied wiring.
MOTOR_PINS = ((40, 38), (15, 35), (12, 16))
MOTOR_NAMES = ("Motor 0 (Front)", "Motor 1 (L-Rear)", "Motor 2 (R-Rear)")
# Set an entry to -1 only when that physical motor runs backward.
MOTOR_SIGNS = (1, 1, 1)

PWM_FREQUENCY_HZ = 1000
MAX_DUTY_PERCENT = 100.0
SLEW_PER_SECOND = 2.5  # zero to full power in 0.4 seconds
REVERSAL_DEADTIME_SECONDS = 0.08
ZERO_EPSILON = 0.005

SCREEN_WIDTH, SCREEN_HEIGHT = 800, 480
UI_FPS = 60


class Motor:
    """Bidirectional motor driven by PWM on two H-bridge input pins."""

    def __init__(self, name: str, in1: int, in2: int) -> None:
        self.name = name
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(in2, GPIO.OUT, initial=GPIO.LOW)
        self.pwm1 = GPIO.PWM(in1, PWM_FREQUENCY_HZ)
        self.pwm2 = GPIO.PWM(in2, PWM_FREQUENCY_HZ)
        self.pwm1.start(0.0)
        self.pwm2.start(0.0)
        self.current_power = 0.0
        self.blocked_until = 0.0

    def _coast(self) -> None:
        self.pwm1.ChangeDutyCycle(0.0)
        self.pwm2.ChangeDutyCycle(0.0)
        self.current_power = 0.0

    def _write(self, power: float) -> None:
        duty = abs(power) * MAX_DUTY_PERCENT
        if power > 0.0:
            self.pwm2.ChangeDutyCycle(0.0)
            self.pwm1.ChangeDutyCycle(duty)
        elif power < 0.0:
            self.pwm1.ChangeDutyCycle(0.0)
            self.pwm2.ChangeDutyCycle(duty)
        else:
            self._coast()

    def command(self, target: float, now: float, dt: float) -> tuple[str, float]:
        target = clamp(target, -1.0, 1.0)
        if abs(target) < ZERO_EPSILON:
            target = 0.0

        reversing = self.current_power * target < 0.0
        if reversing:
            self._coast()
            self.blocked_until = now + REVERSAL_DEADTIME_SECONDS
            return "COAST", 0.0

        if now < self.blocked_until:
            self._coast()
            return "WAIT", 0.0

        max_change = SLEW_PER_SECOND * max(dt, 0.0)
        self.current_power += clamp(
            target - self.current_power, -max_change, max_change
        )
        if abs(self.current_power) < ZERO_EPSILON and target == 0.0:
            self.current_power = 0.0
        self._write(self.current_power)

        if self.current_power > 0.0:
            state = "FWD"
        elif self.current_power < 0.0:
            state = "REV"
        else:
            state = "OFF"
        return state, abs(self.current_power) * MAX_DUTY_PERCENT

    def stop(self) -> None:
        self._coast()
        self.pwm1.stop()
        self.pwm2.stop()


def get_joystick() -> pygame.joystick.Joystick | None:
    if pygame.joystick.get_count() == 0:
        return None
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("3-Motor Omni Wheel Drivetrain - Generic Controller")
    clock = pygame.time.Clock()
    font_small = pygame.font.SysFont(None, 18)
    font_medium = pygame.font.SysFont(None, 22)
    font_bold = pygame.font.SysFont(None, 26, bold=True)

    def render(text: str, font=font_medium, color=(25, 25, 28)):
        return font.render(text, True, color)

    def draw_ui(joy_name, lx, ly, rows, enabled, armed):
        screen.fill((245, 246, 248))
        center_x, center_y, radius = 140, SCREEN_HEIGHT // 2, 110
        pygame.draw.circle(screen, (255, 255, 255), (center_x, center_y), radius)
        pygame.draw.circle(
            screen, (200, 205, 210), (center_x, center_y), radius, 3
        )
        inner_radius = radius - 16
        dot_x = center_x + int(clamp(lx, -1, 1) * (inner_radius - 6))
        dot_y = center_y + int(clamp(ly, -1, 1) * (inner_radius - 6))
        pygame.draw.circle(screen, (220, 224, 229), (dot_x, dot_y), 12)

        status = "ENABLED" if enabled else "DISABLED (Press A)"
        if enabled and not armed:
            status += " - UNARMED (Center sticks briefly)"
        screen.blit(
            render(
                f"Status: {status} - Deadzone: {int(STICK_DEADZONE * 100)}%",
                font_bold,
            ),
            (14, 16),
        )
        screen.blit(
            render(f"Controller Profile: {joy_name}", font_small, (90, 96, 105)),
            (14, 42),
        )

        panel_x = 280
        panel = (panel_x, 12, SCREEN_WIDTH - 296, SCREEN_HEIGHT - 24)
        pygame.draw.rect(screen, (255, 255, 255), panel, border_radius=16)
        pygame.draw.rect(screen, (210, 215, 220), panel, 2, border_radius=16)
        y = 20
        screen.blit(render("Omni Drivetrain Telemetry", font_bold), (panel_x + 12, y))
        y += 28
        for row in rows:
            screen.blit(render(row, font_medium), (panel_x + 12, y))
            y += 28
        pygame.display.flip()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    motors = [
        Motor(name, pins[0], pins[1])
        for name, pins in zip(MOTOR_NAMES, MOTOR_PINS)
    ]
    joystick = get_joystick()
    enabled = False
    armed = False
    neutral_since: float | None = None
    previous_a = False
    previous_y = False
    running = True
    last_time = time.monotonic()

    def all_stop() -> None:
        stop_time = time.monotonic()
        for motor in motors:
            motor.command(0.0, stop_time, 1.0)

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                    joystick = get_joystick()
                    if joystick is None:
                        enabled = armed = False
                        all_stop()

            now = time.monotonic()
            dt = min(now - last_time, 0.1)
            last_time = now
            joy_name = joystick.get_name() if joystick else "No controller found"

            def axis(index: int) -> float:
                if joystick is None or index >= joystick.get_numaxes():
                    return 0.0
                return joystick.get_axis(index)

            def button(index: int) -> bool:
                return bool(
                    joystick is not None
                    and index < joystick.get_numbuttons()
                    and joystick.get_button(index)
                )

            lx_raw = axis(LEFT_X_AXIS)
            ly_raw = axis(LEFT_Y_AXIS)
            rx_raw = axis(RIGHT_X_AXIS)
            a_pressed = button(BUTTON_A_INDEX)
            y_pressed = button(BUTTON_Y_INDEX)

            # Rising edges prevent a held A button from resetting arming each frame.
            if y_pressed and not previous_y:
                enabled = armed = False
                neutral_since = None
                all_stop()
            elif a_pressed and not previous_a:
                enabled = True
                armed = False
                neutral_since = None
                all_stop()
            previous_a, previous_y = a_pressed, y_pressed

            strafe, forward = radial_deadzone(
                lx_raw, -ly_raw, STICK_DEADZONE
            )
            turn = axis_deadzone(rx_raw, TURN_DEADZONE)
            neutral = max((strafe * strafe + forward * forward) ** 0.5, abs(turn))

            if enabled and not armed:
                if neutral <= ARM_NEUTRAL_LIMIT:
                    if neutral_since is None:
                        neutral_since = now
                    elif now - neutral_since >= ARM_NEUTRAL_SECONDS:
                        armed = True
                else:
                    neutral_since = None

            telemetry = []
            if not enabled or not armed or joystick is None:
                all_stop()
                if joystick is None:
                    reason = "STOPPED (Controller disconnected)"
                elif not enabled:
                    reason = "STOPPED (System disabled)"
                else:
                    reason = "SAFE (Return sticks to center)"
                telemetry = [f"{name}: {reason}" for name in MOTOR_NAMES]
            else:
                powers = mix_three_omni(strafe, forward, turn)
                for motor, sign, target in zip(motors, MOTOR_SIGNS, powers):
                    signed_target = target * sign
                    state, duty = motor.command(signed_target, now, dt)
                    telemetry.append(
                        f"{motor.name}: {state:5} Target {abs(signed_target)*100:5.1f}% "
                        f"Current {duty:5.1f}%"
                    )

            draw_ui(joy_name, lx_raw, ly_raw, telemetry, enabled, armed)
            clock.tick(UI_FPS)
    finally:
        for motor in motors:
            motor.stop()
        GPIO.cleanup()
        pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
