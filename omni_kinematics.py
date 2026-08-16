"""Pure three-wheel omni-drive math (no Raspberry Pi dependencies)."""

from __future__ import annotations

import math
from typing import Iterable, Tuple


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


def normalize(values: Iterable[float], limit: float = 1.0) -> Tuple[float, ...]:
    values = tuple(values)
    peak = max((abs(value) for value in values), default=0.0)
    if peak <= limit or peak == 0.0:
        return values
    scale = limit / peak
    return tuple(value * scale for value in values)


def shape_motor_power(
    power: float, start_power: float = 0.50, maximum_power: float = 0.75
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
