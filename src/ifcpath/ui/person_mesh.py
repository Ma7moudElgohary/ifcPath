from __future__ import annotations

import math

from ..model import Vec3

Triangle3D = tuple[Vec3, Vec3, Vec3]


def build_person_mesh(
    base: Vec3,
    forward: tuple[float, float] = (0.0, 1.0),
    *,
    height_m: float = 1.72,
) -> list[Triangle3D]:
    """Build a small license-free triangular person mesh rooted at *base*.

    The mesh is intentionally low-poly because it is a navigation-agent marker,
    not a character renderer. Local +Y is the facing direction and Z is up.
    """
    height_m = max(0.8, float(height_m))
    scale = height_m / 1.72
    fx, fy = forward
    length = math.hypot(fx, fy)
    if length <= 1e-9:
        fx, fy = 0.0, 1.0
    else:
        fx, fy = fx / length, fy / length

    # Local +Y rotated onto the requested XY direction.
    yaw = math.atan2(fx, fy)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    def world(local: Vec3) -> Vec3:
        x, y, z = (local[0] * scale, local[1] * scale, local[2] * scale)
        rx = cy * x + sy * y
        ry = -sy * x + cy * y
        return base[0] + rx, base[1] + ry, base[2] + z

    triangles: list[Triangle3D] = []

    def add_box(center: Vec3, size: Vec3) -> None:
        cx, cy0, cz = center
        sx, sy0, sz = (size[0] * 0.5, size[1] * 0.5, size[2] * 0.5)
        vertices = [
            (cx - sx, cy0 - sy0, cz - sz),
            (cx + sx, cy0 - sy0, cz - sz),
            (cx + sx, cy0 + sy0, cz - sz),
            (cx - sx, cy0 + sy0, cz - sz),
            (cx - sx, cy0 - sy0, cz + sz),
            (cx + sx, cy0 - sy0, cz + sz),
            (cx + sx, cy0 + sy0, cz + sz),
            (cx - sx, cy0 + sy0, cz + sz),
        ]
        faces = (
            (0, 2, 1), (0, 3, 2),
            (4, 5, 6), (4, 6, 7),
            (0, 1, 5), (0, 5, 4),
            (1, 2, 6), (1, 6, 5),
            (2, 3, 7), (2, 7, 6),
            (3, 0, 4), (3, 4, 7),
        )
        transformed = [world(vertex) for vertex in vertices]
        triangles.extend(
            (transformed[a], transformed[b], transformed[c])
            for a, b, c in faces
        )

    def add_octahedron(center: Vec3, radius: float) -> None:
        cx, cy0, cz = center
        points = [
            (cx, cy0, cz + radius),
            (cx + radius, cy0, cz),
            (cx, cy0 + radius, cz),
            (cx - radius, cy0, cz),
            (cx, cy0 - radius, cz),
            (cx, cy0, cz - radius),
        ]
        faces = (
            (0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1),
            (5, 2, 1), (5, 3, 2), (5, 4, 3), (5, 1, 4),
        )
        transformed = [world(point) for point in points]
        triangles.extend(
            (transformed[a], transformed[b], transformed[c])
            for a, b, c in faces
        )

    # Feet and legs.
    add_box((-0.095, 0.035, 0.055), (0.14, 0.28, 0.11))
    add_box((0.095, 0.035, 0.055), (0.14, 0.28, 0.11))
    add_box((-0.095, 0.0, 0.39), (0.14, 0.15, 0.67))
    add_box((0.095, 0.0, 0.39), (0.14, 0.15, 0.67))

    # Pelvis, torso and shoulders.
    add_box((0.0, 0.0, 0.78), (0.32, 0.20, 0.19))
    add_box((0.0, 0.0, 1.12), (0.38, 0.22, 0.53))

    # Arms hang slightly forward so facing direction remains visually obvious.
    add_box((-0.245, 0.025, 1.10), (0.11, 0.13, 0.57))
    add_box((0.245, 0.025, 1.10), (0.11, 0.13, 0.57))

    # Neck and faceted head.
    add_box((0.0, 0.0, 1.425), (0.12, 0.12, 0.12))
    add_octahedron((0.0, 0.0, 1.59), 0.145)

    return triangles
