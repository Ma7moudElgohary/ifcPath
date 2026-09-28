from __future__ import annotations

from collections.abc import Iterable

from shapely.geometry import LineString, MultiPoint
from shapely.geometry.base import BaseGeometry

from .geometry import Vec3


def wall_obstacle_from_vertices(vertices: Iterable[Vec3]) -> BaseGeometry | None:
    """Project wall geometry to XY and return a conservative footprint.

    Convex hulls handle rotated walls substantially better than the axis-aligned
    bounding-box obstacle approximation used by the reference prototype. Very
    thin/degenerate projections are buffered slightly so they still block a
    crossing navigation edge.
    """
    xy = {(float(v[0]), float(v[1])) for v in vertices}
    if len(xy) < 2:
        return None
    hull = MultiPoint(list(xy)).convex_hull
    if hull.is_empty:
        return None
    if hull.area <= 1e-8:
        hull = hull.buffer(0.04, cap_style="flat")
    return hull


def edge_crosses_obstacle(
    a: Vec3,
    b: Vec3,
    obstacles: Iterable[BaseGeometry],
    min_intersection_m: float = 0.02,
) -> bool:
    """Return True when a horizontal navigation edge passes through a wall.

    Portal edges are created separately and deliberately bypass this test; this
    makes doors/openings the sanctioned transitions through wall obstacles.
    """
    line = LineString(((a[0], a[1]), (b[0], b[1])))
    if line.length <= 1e-9:
        return False
    for obstacle in obstacles:
        if obstacle.is_empty or not line.intersects(obstacle):
            continue
        intersection = line.intersection(obstacle)
        if getattr(intersection, "length", 0.0) > min_intersection_m:
            return True
    return False
