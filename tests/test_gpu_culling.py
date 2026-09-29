from __future__ import annotations

from ifcpath.ui.gpu_camera import GpuCameraState
from ifcpath.ui.gpu_culling import aabb_visible_in_clip, world_bounds_to_local


def test_frustum_accepts_bounds_around_camera_target() -> None:
    camera = GpuCameraState(target=(0.0, 0.0, 0.0), yaw_deg=0.0, pitch_deg=0.0, distance=10.0)
    matrix = camera.view_projection(16.0 / 9.0)
    assert aabb_visible_in_clip(matrix, (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0))


def test_frustum_rejects_bounds_behind_camera() -> None:
    camera = GpuCameraState(target=(0.0, 0.0, 0.0), yaw_deg=0.0, pitch_deg=0.0, distance=10.0)
    matrix = camera.view_projection(16.0 / 9.0)
    # Camera is at x=-10 looking toward +X. This box is farther behind it.
    assert not aabb_visible_in_clip(matrix, (-30.0, -1.0, -1.0), (-20.0, 1.0, 1.0))


def test_frustum_rejects_far_side_bounds() -> None:
    camera = GpuCameraState(target=(0.0, 0.0, 0.0), yaw_deg=0.0, pitch_deg=0.0, distance=10.0)
    matrix = camera.view_projection(16.0 / 9.0)
    assert not aabb_visible_in_clip(matrix, (-1.0, 100.0, -1.0), (1.0, 110.0, 1.0))


def test_world_bounds_are_rebased_without_changing_extent() -> None:
    local_min, local_max = world_bounds_to_local(
        (1_000_000.0, 2_000_000.0, 50.0),
        (1_000_020.0, 2_000_040.0, 60.0),
        (1_000_010.0, 2_000_020.0, 55.0),
    )
    assert local_min == (-10.0, -20.0, -5.0)
    assert local_max == (10.0, 20.0, 5.0)
