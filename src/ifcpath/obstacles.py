from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import shapely
from shapely.geometry import LineString, MultiPoint, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from .geometry import Vec3


@dataclass(frozen=True, slots=True)
class WallObstacle:
    """A projected obstacle footprint plus the vertical range it occupies."""

    footprint: BaseGeometry
    z_min: float
    z_max: float


def wall_obstacle_from_vertices(vertices: Iterable[Vec3]) -> WallObstacle | None:
    """Project wall geometry to XY while retaining its vertical extent.

    The wall fallback intentionally uses a convex hull because it is only used
    to reject uncertain sampled-graph edges. Precise free-space subtraction uses
    ``mesh_obstacle_from_triangles`` below.
    """
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


def mesh_obstacle_from_triangles(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
) -> WallObstacle | None:
    """Return the exact XY silhouette of a triangulated IFC object.

    Non-degenerate triangle projections are unioned instead of taking a convex
    hull, so concave columns/equipment do not unnecessarily remove free space.
    The Z range is retained so an overhead object can be ignored when it does not
    intersect the pedestrian clearance volume above a floor.
    """
    if not vertices or not triangles:
        return None

    projected: list[Polygon] = []
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        polygon = Polygon(((a[0], a[1]), (b[0], b[1]), (c[0], c[1])))
        if polygon.area > 1e-10:
            projected.append(polygon)

    if not projected:
        return None

    footprint = shapely.make_valid(unary_union(projected))
    if footprint.is_empty or getattr(footprint, "area", 0.0) <= 1e-10:
        return None

    zs = [float(v[2]) for v in vertices]
    return WallObstacle(footprint, min(zs), max(zs))


def obstacle_intersects_pedestrian_volume(
    obstacle: WallObstacle,
    floor_z: float,
    agent_height_m: float,
    vertical_tolerance_m: float = 0.05,
) -> bool:
    """Return whether an obstacle occupies the pedestrian volume above a floor."""
    top = floor_z + max(agent_height_m, 0.0)
    return not (
        obstacle.z_max < floor_z - vertical_tolerance_m
        or obstacle.z_min > top + vertical_tolerance_m
    )


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
