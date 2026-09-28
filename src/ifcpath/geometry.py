from __future__ import annotations

import math
from collections import defaultdict

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
    """Sample upward-facing walkable surfaces.

    This keeps the fast surface-sampling idea used by Topologic Studio while
    explicitly rejecting downward slab/landing faces, which otherwise create
    duplicate navigation layers beneath floors.
    """
    points: list[Vec3] = []
    min_up = math.cos(math.radians(max_slope_deg))
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        n = triangle_normal(a, b, c)
        if n[2] < min_up:
            continue
        area2 = math.dist((0, 0, 0), (
            (b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),
            (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),
            (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]),
        ))
        area = area2 * 0.5
        divisions = max(1, int(math.ceil(math.sqrt(max(area, 1e-9)) / max(spacing_m, 1e-3))))
        for i in range(divisions + 1):
            for j in range(divisions + 1 - i):
                u, v = i / divisions, j / divisions
                w = 1.0 - u - v
                points.append((
                    u*a[0] + v*b[0] + w*c[0],
                    u*a[1] + v*b[1] + w*c[1],
                    u*a[2] + v*b[2] + w*c[2],
                ))
                if len(points) >= max_points:
                    return points
    return points


def build_radius_edges(
    points: list[Vec3],
    max_distance_m: float,
    max_neighbors: int = 8,
) -> list[tuple[int, int, float]]:
    """Build a sparse local graph using spatial hashing and bounded neighbours.

    The original MVP connected every pair inside the radius. Real IFC
    qualification showed that this creates hundreds of thousands of redundant
    edges for only a few thousand nodes. Here each node nominates only its
    nearest ``max_neighbors`` candidates inside the radius; the undirected union
    is returned. Complexity stays local while preserving alternate routes.
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
                        if 1e-9 < d <= max_distance_m:
                            candidates.append((d, j))

        candidates.sort(key=lambda item: item[0])
        for d, j in candidates[:max_neighbors]:
            a, b = (i, j) if i < j else (j, i)
            current = selected.get((a, b))
            if current is None or d < current:
                selected[(a, b)] = d

    return [(a, b, d) for (a, b), d in sorted(selected.items())]
