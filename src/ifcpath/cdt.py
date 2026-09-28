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


def build_floor_cdt_graph(
    floor,
    z: float,
    *,
    boundary_spacing_m: float = 1.0,
) -> tuple[list[Vec3], list[Edge]]:
    """Build a sparse CDT dual graph with explicit boundary anchor samples.

    Triangle centroids form the internal metric network. Internal shared edges
    connect adjacent triangle centroids. Each polygon-boundary edge additionally
    receives one or more interior edge samples (never the vertices themselves),
    connected to its owning triangle centroid. Those boundary anchors give
    semantic doors/transfers a stable geometric attachment point without relying
    on an arbitrary centroid-distance threshold.
    """
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
    edge_geometry: dict[tuple[tuple[int, int], tuple[int, int]], tuple[tuple[float, float], tuple[float, float]]] = {}
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
            edge_geometry.setdefault(key, ((float(start[0]), float(start[1])), (float(end[0]), float(end[1]))))

    edges: list[Edge] = []
    spacing = max(boundary_spacing_m, 0.1)
    for key, owners in edge_owners.items():
        if len(owners) == 2:
            a, b = owners
            segment = LineString([(points[a][0], points[a][1]), (points[b][0], points[b][1])])
            if floor.covers(segment):
                edges.append((a, b, distance(points[a], points[b])))
            continue

        if len(owners) != 1:
            continue

        owner = owners[0]
        start, end = edge_geometry[key]
        length = math.dist(start, end)
        divisions = max(1, int(math.ceil(length / spacing)))
        for k in range(divisions):
            # Segment midpoints avoid creating zero-width shortcuts through a
            # polygon vertex shared only by diagonally touching cells.
            t = (k + 0.5) / divisions
            boundary_point = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                z,
            )
            boundary_index = len(points)
            points.append(boundary_point)
            edges.append((owner, boundary_index, distance(points[owner], boundary_point)))

    return points, edges


def build_space_cdt_graph(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
    boundary_spacing_m: float = 1.0,
) -> tuple[list[Vec3], list[Edge]]:
    """Build a sparse metric graph from an IFC space floor using CDT."""
    floor = prepare_space_floor(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
        clearance_m=clearance_m,
    )
    return build_floor_cdt_graph(
        floor,
        _floor_elevation(vertices),
        boundary_spacing_m=boundary_spacing_m,
    )


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
