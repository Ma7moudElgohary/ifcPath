from __future__ import annotations

import math

from ifcpath.ui.viewport_navigation import (
    ViewportNavigationConfig,
    clamp_zoom_scale,
    evenly_sample,
    orbit_angles,
    wheel_zoom_factor,
)


def test_orbit_angles_wrap_yaw_and_clamp_pitch() -> None:
    config = ViewportNavigationConfig()

    yaw, pitch = orbit_angles(359.0, 85.0, 20.0, -100.0, config)

    assert 0.0 <= yaw < 360.0
    assert yaw == (359.0 + 20.0 * config.orbit_yaw_degrees_per_pixel) % 360.0
    assert pitch == config.max_pitch_degrees


def test_wheel_zoom_uses_continuous_trackpad_delta() -> None:
    config = ViewportNavigationConfig()

    wheel = wheel_zoom_factor(
        angle_delta_y=120.0,
        pixel_delta_y=0.0,
        config=config,
    )
    trackpad = wheel_zoom_factor(
        angle_delta_y=0.0,
        pixel_delta_y=config.trackpad_pixels_per_notch,
        config=config,
    )

    assert math.isclose(wheel, config.wheel_zoom_per_notch, rel_tol=1e-9)
    assert math.isclose(trackpad, wheel, rel_tol=1e-9)


def test_zoom_is_clamped_relative_to_fit_scale() -> None:
    config = ViewportNavigationConfig()

    factor, target = clamp_zoom_scale(1.0, 1e9, 2.0, config)
    assert math.isclose(target, 2.0 * config.max_zoom_to_fit_ratio)
    assert math.isclose(factor, target)

    factor, target = clamp_zoom_scale(1.0, 1e-9, 2.0, config)
    assert math.isclose(target, 2.0 * config.min_zoom_to_fit_ratio)
    assert math.isclose(factor, target)


def test_evenly_sample_keeps_ends_and_requested_count() -> None:
    values = list(range(100))

    sampled = list(evenly_sample(values, 7))

    assert len(sampled) == 7
    assert sampled[0] == 0
    assert sampled[-1] == 99
    assert sampled == sorted(sampled)
