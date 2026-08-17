"""Pure three-wheel omni-drive math (no Raspberry Pi dependencies)."""

from __future__ import annotations

import math
from typing import Iterable, Tuple


THREE_OMNI_MOTOR_SIGNS = (1, 1, -1)


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


def cardinal_lock(
    strafe: float, forward: float, horizontal_band: float = 0.20
) -> Tuple[float, float]:
    """Allow strafe only while the stick is close to exactly horizontal."""
    if abs(forward) <= horizontal_band:
        return strafe, 0.0
    return 0.0, forward


def axis_deadzone(value: float, deadzone: float) -> float:
    """Remove a one-dimensional deadzone without leaving a power jump."""
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((min(abs(value), 1.0) - deadzone) / (1.0 - deadzone), value)


def controller_drive_axes(
    left_x: float,
    left_y: float,
    turn_axis: float,
    turn_orthogonal_axis: float,
    turn_orthogonal_gate: float = 0.20,
) -> Tuple[float, float, float]:
    """Map drive axes and reject turn-stick diagonal/sideways input."""
    turn = (
        turn_axis
        if abs(turn_orthogonal_axis) <= turn_orthogonal_gate
        else 0.0
    )
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
    power: float,
    start_power: float = 0.75,
    maximum_power: float = 1.00,
    full_power_threshold: float = 0.65,
) -> float:
    """Map nonzero commands to usable duty and saturate strong input at 100%."""
    power = clamp(power, -1.0, 1.0)
    if power == 0.0:
        return 0.0
    magnitude = abs(power)
    if magnitude >= full_power_threshold:
        duty = maximum_power
    else:
        fraction = magnitude / max(full_power_threshold, 1e-9)
        duty = start_power + (maximum_power - start_power) * fraction
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
