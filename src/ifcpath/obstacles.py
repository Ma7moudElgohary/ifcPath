from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

from shapely.geometry import LineString, MultiPoint
from shapely.geometry.base import BaseGeometry

from .geometry import Vec3


@dataclass(frozen=True, slots=True)
class WallObstacle:
    footprint: BaseGeometry
    z_min: float
    z_max: float


def wall_obstacle_from_vertices(vertices: Iterable[Vec3]) -> WallObstacle | None:
    """Project wall geometry to XY while retaining its vertical extent."""
    verts = list(vertices)
    xy = {(float(v[0]), float(v[1])) for v in verts}
    if len(xy) < 2 or not verts:
        return None
    hull = MultiPoint(list(xy)).convex_hull
    if hull.is_empty:
        return None
    if hull.area <= 1e-8:
        hull = hull.buffer(0.04, cap_style="flat")
    zs = [float(v[2]) for v in verts]
    return WallObstacle(hull, min(zs), max(zs))


def edge_crosses_obstacle(
    a: Vec3,
    b: Vec3,
    obstacles: Iterable[WallObstacle],
    min_intersection_m: float = 0.02,
    vertical_tolerance_m: float = 0.10,
) -> bool:
    """Return True when a navigation edge passes through a wall at its height.

    Portal edges are created separately and deliberately bypass this test; this
    makes doors/openings the sanctioned transitions through wall obstacles.
    """
    line = LineString(((a[0], a[1]), (b[0], b[1])))
    if line.length <= 1e-9:
        return False

    edge_z_min = min(a[2], b[2])
    edge_z_max = max(a[2], b[2])
    for obstacle in obstacles:
        if edge_z_max < obstacle.z_min - vertical_tolerance_m:
            continue
        if edge_z_min > obstacle.z_max + vertical_tolerance_m:
            continue
        footprint = obstacle.footprint
        if footprint.is_empty or not line.intersects(footprint):
            continue
        intersection = line.intersection(footprint)
        if getattr(intersection, "length", 0.0) > min_intersection_m:
            return True
    return False
