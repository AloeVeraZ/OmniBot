"""Pure three-wheel omni-drive math (no Raspberry Pi dependencies)."""

from __future__ import annotations

import math
from typing import Iterable, Tuple


THREE_OMNI_MOTOR_SIGNS = (1, -1, -1)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def radial_deadzone(x: float, y: float, deadzone: float) -> Tuple[float, float]:
    """Remove a circular deadzone and remap the remaining stick to 0..1."""
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0

    magnitude = min(magnitude, 1.0)
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return x / max(math.hypot(x, y), 1e-9) * scaled, y / max(
        math.hypot(x, y), 1e-9
    ) * scaled


def axis_deadzone(value: float, deadzone: float) -> float:
    """Remove a one-dimensional deadzone without leaving a power jump."""
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((min(abs(value), 1.0) - deadzone) / (1.0 - deadzone), value)


def controller_drive_axes(
    left_x: float,
    left_y: float,
    right_x: float,
    right_y: float,
    right_vertical_gate: float = 0.20,
) -> Tuple[float, float, float]:
    """Map reversed axes and accept turning only near right-stick horizontal."""
    turn = -right_x if abs(right_y) <= right_vertical_gate else 0.0
    return -left_x, -left_y, turn


def trigger_activation(raw_value: float, rest_value: float) -> float:
    """Return 0..1 trigger travel for controllers using either common axis range."""
    travel = abs(raw_value - rest_value)
    if abs(rest_value) >= 0.5:
        travel *= 0.5  # Common SDL trigger range is -1 (rest) to +1 (pressed).
    return clamp(travel, 0.0, 1.0)


def next_servo_angle(
    angle: float,
    left_trigger: float,
    right_trigger: float,
    dt: float,
    speed_degrees_per_second: float,
) -> float:
    """Rate-control the 300-degree goBILDA servo within -150..+150 degrees."""
    direction = clamp(right_trigger, 0.0, 1.0) - clamp(
        left_trigger, 0.0, 1.0
    )
    return clamp(
        angle + direction * speed_degrees_per_second * max(dt, 0.0),
        -150.0,
        150.0,
    )


def normalize(values: Iterable[float], limit: float = 1.0) -> Tuple[float, ...]:
    values = tuple(values)
    peak = max((abs(value) for value in values), default=0.0)
    if peak <= limit or peak == 0.0:
        return values
    scale = limit / peak
    return tuple(value * scale for value in values)


def shape_motor_power(
    power: float, start_power: float = 0.75, maximum_power: float = 1.00
) -> float:
    """Map every nonzero command into the motor's usable duty-cycle range."""
    power = clamp(power, -1.0, 1.0)
    if power == 0.0:
        return 0.0
    duty = start_power + (maximum_power - start_power) * abs(power)
    return math.copysign(duty, power)


def mix_three_omni(strafe: float, forward: float, turn: float) -> Tuple[float, float, float]:
    """Return front, left-rear, and right-rear motor powers in [-1, 1].

    The wheels are tangent to a circle and spaced 120 degrees apart. The
    translation portion is direction-normalized so a full stick uses the
    available motor range in every direction. Rotation is then added and the
    combined command is normalized while preserving all wheel ratios.
    """
    strafe = clamp(strafe, -1.0, 1.0)
    forward = clamp(forward, -1.0, 1.0)
    turn = clamp(turn, -1.0, 1.0)
    translation_magnitude = min(math.hypot(strafe, forward), 1.0)

    translation = (
        -strafe,
        0.5 * strafe - (math.sqrt(3.0) / 2.0) * forward,
        0.5 * strafe + (math.sqrt(3.0) / 2.0) * forward,
    )

    peak = max(abs(value) for value in translation)
    if peak > 0.0:
        translation = tuple(
            value * translation_magnitude / peak for value in translation
        )

    return normalize(tuple(value + turn for value in translation))  # type: ignore[return-value]
