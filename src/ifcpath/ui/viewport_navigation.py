from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, TypeVar


T = TypeVar("T")


@dataclass(frozen=True, slots=True)
class ViewportNavigationConfig:
    """Interaction tuning for the desktop BIM viewport.

    The interaction limits intentionally keep camera motion cheap. Full-quality
    geometry is restored after the camera settles, similar to culling/LOD update
    strategies used by high-performance BIM viewers.
    """

    orbit_yaw_degrees_per_pixel: float = 0.30
    orbit_pitch_degrees_per_pixel: float = 0.24
    min_pitch_degrees: float = 8.0
    max_pitch_degrees: float = 88.0
    wheel_zoom_per_notch: float = 1.16
    trackpad_pixels_per_notch: float = 80.0
    min_zoom_to_fit_ratio: float = 0.12
    max_zoom_to_fit_ratio: float = 120.0
    interaction_frame_interval_ms: int = 22
    settle_delay_ms: int = 90
    interactive_bim_triangle_limit: int = 1600
    interactive_nav_cell_limit: int = 1200
    interactive_portal_limit: int = 500


def orbit_angles(
    yaw_degrees: float,
    pitch_degrees: float,
    delta_x_pixels: float,
    delta_y_pixels: float,
    config: ViewportNavigationConfig,
) -> tuple[float, float]:
    """Apply one pointer orbit delta with a stable pitch clamp."""

    yaw = (yaw_degrees + delta_x_pixels * config.orbit_yaw_degrees_per_pixel) % 360.0
    pitch = pitch_degrees - delta_y_pixels * config.orbit_pitch_degrees_per_pixel
    pitch = min(config.max_pitch_degrees, max(config.min_pitch_degrees, pitch))
    return yaw, pitch


def wheel_zoom_factor(
    *,
    angle_delta_y: float,
    pixel_delta_y: float,
    config: ViewportNavigationConfig,
    precision_scale: float = 1.0,
) -> float:
    """Return a continuous exponential zoom factor for mouse wheels/trackpads."""

    if abs(pixel_delta_y) > 1e-9:
        notches = pixel_delta_y / max(1e-9, config.trackpad_pixels_per_notch)
    else:
        notches = angle_delta_y / 120.0
    if abs(notches) <= 1e-12:
        return 1.0
    exponent = notches * max(0.05, float(precision_scale))
    return math.exp(math.log(config.wheel_zoom_per_notch) * exponent)


def clamp_zoom_scale(
    current_scale: float,
    requested_factor: float,
    fit_scale: float,
    config: ViewportNavigationConfig,
) -> tuple[float, float]:
    """Return ``(applied_factor, target_scale)`` with fit-relative limits."""

    current = max(1e-12, abs(float(current_scale)))
    fit = max(1e-12, abs(float(fit_scale)))
    minimum = fit * config.min_zoom_to_fit_ratio
    maximum = fit * config.max_zoom_to_fit_ratio
    target = min(maximum, max(minimum, current * max(1e-6, requested_factor)))
    return target / current, target


def evenly_sample(sequence: Sequence[T], limit: int) -> Sequence[T]:
    """Deterministically retain a representative subset without random jitter."""

    count = len(sequence)
    if limit <= 0 or count == 0:
        return sequence[:0]
    if count <= limit:
        return sequence
    if limit == 1:
        return sequence[:1]

    last = count - 1
    indices = [round(index * last / (limit - 1)) for index in range(limit)]
    return [sequence[index] for index in indices]
