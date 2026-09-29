from __future__ import annotations

import math

import pytest

from ifcpath.ui.gpu_camera import GpuCameraState, ray_triangle_distance


def test_gpu_camera_orbit_clamps_pitch_and_wraps_yaw() -> None:
    camera = GpuCameraState(yaw_deg=359.0, pitch_deg=80.0)
    camera.orbit(20.0, -100.0, sensitivity=1.0)
    assert camera.yaw_deg == pytest.approx(19.0)
    assert camera.pitch_deg == pytest.approx(88.0)


def test_gpu_camera_fit_bounds_and_dolly_are_scale_relative() -> None:
    camera = GpuCameraState()
    camera.fit_bounds((-10.0, -5.0, -2.0), (10.0, 5.0, 2.0))
    fitted = camera.distance
    assert fitted > 10.0
    camera.dolly(1.0)
    assert camera.distance < fitted
    camera.dolly(-1.0)
    assert camera.distance == pytest.approx(fitted)


def test_gpu_camera_pan_moves_target_in_view_plane() -> None:
    camera = GpuCameraState(target=(0.0, 0.0, 0.0), yaw_deg=0.0, pitch_deg=0.0, distance=10.0)
    before = camera.target
    camera.pan(100.0, 0.0, 1000.0)
    assert camera.target != before
    assert camera.target[2] == pytest.approx(0.0, abs=1e-9)


def test_screen_center_ray_points_at_camera_target() -> None:
    camera = GpuCameraState(target=(0.0, 0.0, 0.0), yaw_deg=30.0, pitch_deg=25.0, distance=15.0)
    origin, direction = camera.screen_ray(500.0, 400.0, 1000.0, 800.0)
    to_target = tuple(-value for value in origin)
    length = math.sqrt(sum(value * value for value in to_target))
    expected = tuple(value / length for value in to_target)
    assert direction == pytest.approx(expected, abs=1e-7)


def test_ray_triangle_intersection_returns_metric_distance() -> None:
    triangle = ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0))
    distance = ray_triangle_distance((0.5, 0.5, 5.0), (0.0, 0.0, -1.0), triangle)
    assert distance == pytest.approx(5.0)
    assert ray_triangle_distance((3.0, 3.0, 5.0), (0.0, 0.0, -1.0), triangle) is None
