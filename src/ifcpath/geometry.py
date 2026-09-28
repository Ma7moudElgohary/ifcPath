from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Callable

Vec3 = tuple[float, float, float]


def distance(a: Vec3, b: Vec3) -> float:
    return math.dist(a, b)


def triangle_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    nx, ny, nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
    mag = math.sqrt(nx*nx + ny*ny + nz*nz)
    return (0.0, 0.0, 0.0) if mag <= 1e-12 else (nx/mag, ny/mag, nz/mag)


def sample_walkable_triangles(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    spacing_m: float = 0.75,
    max_slope_deg: float = 45.0,
    max_points: int = 50000,
) -> list[Vec3]:
    """Sample upward-facing walkable surfaces."""
    min_up = math.cos(math.radians(max_slope_deg))
    accepted: list[tuple[Vec3, Vec3, Vec3]] = []
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        if triangle_normal(a, b, c)[2] >= min_up:
            accepted.append((a, b, c))
    return _sample_triangles(accepted, spacing_m, max_points)


def sample_space_floor_triangles(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    spacing_m: float = 0.75,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    max_points: int = 50000,
) -> list[Vec3]:
    """Sample the actual bottom floor surface of an IFC space volume.

    IfcSpace solids normally contain a horizontal bottom face whose triangle
    winding can point either up or down. We therefore use ``abs(normal.z)`` and
    restrict samples to triangles close to the minimum space elevation. This is
    materially more reliable than sampling a whole building slab and later
    guessing room membership from bounding boxes.
    """
    if not vertices:
        return []
    floor_z = min(v[2] for v in vertices)
    min_vertical = math.cos(math.radians(max_slope_deg))
    accepted: list[tuple[Vec3, Vec3, Vec3]] = []
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        if max(a[2], b[2], c[2]) > floor_z + floor_tolerance_m:
            continue
        if abs(triangle_normal(a, b, c)[2]) < min_vertical:
            continue
        accepted.append((a, b, c))
    return _sample_triangles(accepted, spacing_m, max_points)


def _sample_triangles(
    triangles: list[tuple[Vec3, Vec3, Vec3]],
    spacing_m: float,
    max_points: int,
) -> list[Vec3]:
    """Sample triangles with a maximum grid step controlled by edge length.

    Using ``sqrt(area)`` to choose subdivisions undersamples long, skinny IFC
    triangles: a ten-metre triangle only a few centimetres wide can have a tiny
    area and therefore receive just its corner samples. Basing subdivisions on
    the longest edge keeps neighbouring samples within the requested spacing
    regardless of triangle aspect ratio.

    Shared triangle boundaries generate the same barycentric samples repeatedly,
    so exact/near-exact duplicates are collapsed to keep the navigation graph
    compact and to prevent duplicate samples from consuming nearest-neighbour
    slots.
    """
    points: list[Vec3] = []
    seen: set[tuple[int, int, int]] = set()
    spacing = max(spacing_m, 1e-3)
    quantization = 1_000_000_000.0

    for a, b, c in triangles:
        area2 = math.dist((0, 0, 0), (
            (b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),
            (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),
            (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]),
        ))
        area = area2 * 0.5
        if area <= 1e-10:
            continue

        longest_edge = max(distance(a, b), distance(b, c), distance(c, a))
        divisions = max(1, int(math.ceil(longest_edge / spacing)))
        for i in range(divisions + 1):
            for j in range(divisions + 1 - i):
                u, v = i / divisions, j / divisions
                w = 1.0 - u - v
                point = (
                    u*a[0] + v*b[0] + w*c[0],
                    u*a[1] + v*b[1] + w*c[1],
                    u*a[2] + v*b[2] + w*c[2],
                )
                key = (
                    round(point[0] * quantization),
                    round(point[1] * quantization),
                    round(point[2] * quantization),
                )
                if key in seen:
                    continue
                seen.add(key)
                points.append(point)
                if len(points) >= max_points:
                    return points
    return points


def build_radius_edges(
    points: list[Vec3],
    max_distance_m: float,
    max_neighbors: int = 8,
    candidate_filter: Callable[[int, int], bool] | None = None,
) -> list[tuple[int, int, float]]:
    """Build a sparse local graph using spatial hashing and bounded neighbours.

    ``candidate_filter`` is applied *before* nearest-neighbour truncation. This
    is important for semantic navigation: invalid candidates across a wall or
    into another IFC space must not consume the limited neighbour slots and
    accidentally isolate a node from valid neighbours in its own space.
    """
    if not points or max_distance_m <= 0 or max_neighbors <= 0:
        return []

    inv = 1.0 / max_distance_m
    buckets: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for i, p in enumerate(points):
        buckets[(math.floor(p[0]*inv), math.floor(p[1]*inv), math.floor(p[2]*inv))].append(i)

    selected: dict[tuple[int, int], float] = {}
    for i, p in enumerate(points):
        key = (math.floor(p[0]*inv), math.floor(p[1]*inv), math.floor(p[2]*inv))
        candidates: list[tuple[float, int]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in buckets.get((key[0]+dx, key[1]+dy, key[2]+dz), []):
                        if j == i:
                            continue
                        d = distance(p, points[j])
                        if not 1e-9 < d <= max_distance_m:
                            continue
                        if candidate_filter is not None and not candidate_filter(i, j):
                            continue
                        candidates.append((d, j))

        candidates.sort(key=lambda item: item[0])
        for d, j in candidates[:max_neighbors]:
            a, b = (i, j) if i < j else (j, i)
            current = selected.get((a, b))
            if current is None or d < current:
                selected[(a, b)] = d

    return [(a, b, d) for (a, b), d in sorted(selected.items())]
