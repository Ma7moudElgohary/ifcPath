from __future__ import annotations

import math
from collections import defaultdict

import shapely
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from .geometry import Vec3, distance, triangle_normal


Edge = tuple[int, int, float]


def prepare_space_floor(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
):
    floor = space_floor_polygon(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
    )
    if floor is None or floor.is_empty:
        return None
    if clearance_m > 0.0:
        floor = shapely.buffer(floor, -clearance_m, join_style="mitre")
        floor = shapely.make_valid(floor)
        if floor.is_empty:
            return None
    return floor


def build_floor_cdt_graph(floor, z: float) -> tuple[list[Vec3], list[Edge]]:
    """Build the triangle-dual metric graph for a prepared walkable polygon."""
    if floor is None or floor.is_empty:
        return [], []

    triangulation = shapely.constrained_delaunay_triangles(floor)
    triangle_polygons = [
        geometry
        for geometry in getattr(triangulation, "geoms", ())
        if geometry.geom_type == "Polygon" and geometry.area > 1e-10
    ]
    if not triangle_polygons:
        return [], []

    points: list[Vec3] = []
    edge_owners: dict[tuple[tuple[int, int], tuple[int, int]], list[int]] = defaultdict(list)
    quantization = 1_000_000_000.0

    for triangle_index, triangle in enumerate(triangle_polygons):
        centroid = triangle.centroid
        points.append((float(centroid.x), float(centroid.y), z))

        coordinates = list(triangle.exterior.coords)
        for start, end in zip(coordinates, coordinates[1:]):
            a = (round(start[0] * quantization), round(start[1] * quantization))
            b = (round(end[0] * quantization), round(end[1] * quantization))
            key = (a, b) if a <= b else (b, a)
            edge_owners[key].append(triangle_index)

    edges: list[Edge] = []
    for owners in edge_owners.values():
        if len(owners) != 2:
            continue
        a, b = owners
        segment = LineString([(points[a][0], points[a][1]), (points[b][0], points[b][1])])
        if not floor.covers(segment):
            continue
        edges.append((a, b, distance(points[a], points[b])))

    return points, edges


def build_space_cdt_graph(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
) -> tuple[list[Vec3], list[Edge]]:
    """Build a sparse metric graph from an IFC space floor using CDT."""
    floor = prepare_space_floor(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
        clearance_m=clearance_m,
    )
    return build_floor_cdt_graph(floor, _floor_elevation(vertices))


def space_floor_polygon(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
):
    """Return the valid 2D polygonal footprint of the bottom of an IFC space."""
    if not vertices:
        return None

    floor_z = _floor_elevation(vertices)
    min_vertical = math.cos(math.radians(max_slope_deg))
    polygons: list[Polygon] = []

    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        if max(a[2], b[2], c[2]) > floor_z + floor_tolerance_m:
            continue
        if abs(triangle_normal(a, b, c)[2]) < min_vertical:
            continue
        polygon = Polygon([(a[0], a[1]), (b[0], b[1]), (c[0], c[1])])
        if polygon.area > 1e-10:
            polygons.append(polygon)

    if not polygons:
        return None

    merged = unary_union(polygons)
    return shapely.make_valid(merged)


def floor_elevation(vertices: list[Vec3]) -> float:
    return _floor_elevation(vertices)


def _floor_elevation(vertices: list[Vec3]) -> float:
    return min(vertex[2] for vertex in vertices)
