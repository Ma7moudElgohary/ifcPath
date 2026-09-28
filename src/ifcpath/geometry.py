from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class TriangleSoup:
    vertices: list[float]
    indices: list[int]


def triangle_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    ab = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
    ac = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
    n = (
        ab[1]*ac[2]-ab[2]*ac[1],
        ab[2]*ac[0]-ab[0]*ac[2],
        ab[0]*ac[1]-ab[1]*ac[0],
    )
    mag = math.sqrt(sum(x*x for x in n))
    if mag <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (n[0]/mag, n[1]/mag, n[2]/mag)


def sample_walkable_triangles(
    soup: TriangleSoup,
    spacing_m: float = 0.75,
    max_slope_deg: float = 45.0,
    max_points: int = 50000,
) -> list[Vec3]:
    """Sample points over upward-facing triangles."""
    verts, inds = soup.vertices, soup.indices
    result: list[Vec3] = []
    min_up = math.cos(math.radians(max_slope_deg))

    for i in range(0, len(inds), 3):
        ia, ib, ic = inds[i]*3, inds[i+1]*3, inds[i+2]*3
        if ic + 2 >= len(verts):
            continue
        a = (verts[ia], verts[ia+1], verts[ia+2])
        b = (verts[ib], verts[ib+1], verts[ib+2])
        c = (verts[ic], verts[ic+1], verts[ic+2])
        n = triangle_normal(a, b, c)
        if n[2] < min_up:
            continue

        ab = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        ac = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
        cross = (
            ab[1]*ac[2]-ab[2]*ac[1],
            ab[2]*ac[0]-ab[0]*ac[2],
            ab[0]*ac[1]-ab[1]*ac[0],
        )
        area = 0.5 * math.sqrt(sum(x*x for x in cross))
        if area <= 1e-8:
            continue

        divisions = max(1, math.ceil(math.sqrt(area) / max(spacing_m, 1e-3)))
        for u in range(divisions + 1):
            for v in range(divisions + 1 - u):
                alpha = u / divisions
                beta = v / divisions
                gamma = 1.0 - alpha - beta
                p = (
                    alpha*a[0] + beta*b[0] + gamma*c[0],
                    alpha*a[1] + beta*b[1] + gamma*c[1],
                    alpha*a[2] + beta*b[2] + gamma*c[2],
                )
                result.append(p)
                if len(result) >= max_points:
                    return result
    return result


def build_radius_edges(points: list[Vec3], max_dist_m: float) -> list[tuple[int, int, float]]:
    """Spatial-hash radius graph; avoids O(n^2) pair testing."""
    if not points or max_dist_m <= 0:
        return []
    inv = 1.0 / max_dist_m
    buckets: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for i, p in enumerate(points):
        buckets[(math.floor(p[0]*inv), math.floor(p[1]*inv), math.floor(p[2]*inv))].append(i)

    edges: list[tuple[int, int, float]] = []
    for i, p in enumerate(points):
        key = (math.floor(p[0]*inv), math.floor(p[1]*inv), math.floor(p[2]*inv))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in buckets.get((key[0]+dx, key[1]+dy, key[2]+dz), []):
                        if j <= i:
                            continue
                        q = points[j]
                        d = math.dist(p, q)
                        if 1e-8 < d <= max_dist_m:
                            edges.append((i, j, d))
    return edges
