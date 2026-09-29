from __future__ import annotations

from ..model import Vec3


def aabb_visible_in_clip(
    matrix: tuple[float, ...],
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> bool:
    """Return whether an AABB intersects the WebGPU homogeneous clip volume.

    The test is conservative: a batch is rejected only when all eight corners
    lie outside the same clip plane. WebGPU clip depth is ``0 <= z <= w``.
    """

    corners = [
        (x, y, z)
        for x in (bounds_min[0], bounds_max[0])
        for y in (bounds_min[1], bounds_max[1])
        for z in (bounds_min[2], bounds_max[2])
    ]
    clip = [_transform(matrix, point) for point in corners]

    if all(x < -w for x, _y, _z, w in clip):
        return False
    if all(x > w for x, _y, _z, w in clip):
        return False
    if all(y < -w for _x, y, _z, w in clip):
        return False
    if all(y > w for _x, y, _z, w in clip):
        return False
    if all(z < 0.0 for _x, _y, z, _w in clip):
        return False
    if all(z > w for _x, _y, z, w in clip):
        return False
    return True


def world_bounds_to_local(bounds_min: Vec3, bounds_max: Vec3, origin: Vec3) -> tuple[Vec3, Vec3]:
    return (
        (
            bounds_min[0] - origin[0],
            bounds_min[1] - origin[1],
            bounds_min[2] - origin[2],
        ),
        (
            bounds_max[0] - origin[0],
            bounds_max[1] - origin[1],
            bounds_max[2] - origin[2],
        ),
    )


def _transform(matrix: tuple[float, ...], point: Vec3) -> tuple[float, float, float, float]:
    x, y, z = point
    v = (x, y, z, 1.0)
    return tuple(
        sum(matrix[col * 4 + row] * v[col] for col in range(4))
        for row in range(4)
    )  # type: ignore[return-value]
