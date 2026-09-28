from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field

import shapely
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from .geometry import Vec3, distance, triangle_normal


Edge = tuple[int, int, float]


@dataclass(slots=True)
class CdtCellData:
    vertices: tuple[Vec3, Vec3, Vec3]
    neighbor_indices: list[int] = field(default_factory=list)


@dataclass(slots=True)
class CdtNavMeshResult:
    points: list[Vec3] = field(default_factory=list)
    edges: list[Edge] = field(default_factory=list)
    point_cell_indices: list[int | None] = field(default_factory=list)
    cells: list[CdtCellData] = field(default_factory=list)


def prepare_space_floor(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
    obstacle_footprints: Iterable[BaseGeometry] = (),
):
    floor = space_floor_polygon(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
    )
    if floor is None or floor.is_empty:
        return None

    obstacles = [
        obstacle
        for obstacle in obstacle_footprints
        if obstacle is not None and not obstacle.is_empty and floor.intersects(obstacle)
    ]
    if obstacles:
        floor = shapely.difference(floor, unary_union(obstacles))
        floor = shapely.make_valid(floor)
        if floor.is_empty:
            return None

    if clearance_m > 0.0:
        floor = shapely.buffer(floor, -clearance_m, join_style="mitre")
        floor = shapely.make_valid(floor)
        if floor.is_empty:
            return None
    return floor


def build_floor_cdt_navmesh(
    floor,
    z: float,
    *,
    boundary_spacing_m: float = 1.0,
) -> CdtNavMeshResult:
    """Build a constrained triangular navmesh plus its sparse dual graph.

    The returned triangle cells are the authoritative local walkable geometry.
    Centroid/boundary nodes remain for backward-compatible graph routing and for
    attaching semantic portals. Keeping both representations lets consumers use
    a triangle corridor + funnel algorithm without breaking existing graph-only
    consumers.
    """
    result = CdtNavMeshResult()
    if floor is None or floor.is_empty:
        return result

    triangulation = shapely.constrained_delaunay_triangles(floor)
    triangle_polygons = [
        geometry
        for geometry in getattr(triangulation, "geoms", ())
        if geometry.geom_type == "Polygon"
        and geometry.area > 1e-10
        and floor.covers(geometry.representative_point())
    ]
    if not triangle_polygons:
        return result

    edge_owners: dict[tuple[tuple[int, int], tuple[int, int]], list[int]] = defaultdict(list)
    edge_geometry: dict[
        tuple[tuple[int, int], tuple[int, int]],
        tuple[tuple[float, float], tuple[float, float]],
    ] = {}
    quantization = 1_000_000_000.0

    # Triangle centroids occupy the first N point slots, one per cell.
    for triangle_index, triangle in enumerate(triangle_polygons):
        coords = list(triangle.exterior.coords)[:-1]
        if len(coords) != 3:
            continue
        vertices = (
            (float(coords[0][0]), float(coords[0][1]), z),
            (float(coords[1][0]), float(coords[1][1]), z),
            (float(coords[2][0]), float(coords[2][1]), z),
        )
        result.cells.append(CdtCellData(vertices=vertices))

        centroid = triangle.centroid
        result.points.append((float(centroid.x), float(centroid.y), z))
        result.point_cell_indices.append(triangle_index)

        ring = list(triangle.exterior.coords)
        for start, end in zip(ring, ring[1:]):
            a = (round(start[0] * quantization), round(start[1] * quantization))
            b = (round(end[0] * quantization), round(end[1] * quantization))
            key = (a, b) if a <= b else (b, a)
            edge_owners[key].append(triangle_index)
            edge_geometry.setdefault(
                key,
                ((float(start[0]), float(start[1])), (float(end[0]), float(end[1]))),
            )

    spacing = max(boundary_spacing_m, 0.1)
    for key, owners in edge_owners.items():
        if len(owners) == 2:
            a, b = owners
            if a >= len(result.cells) or b >= len(result.cells):
                continue
            segment = LineString([
                (result.points[a][0], result.points[a][1]),
                (result.points[b][0], result.points[b][1]),
            ])
            if floor.covers(segment):
                result.edges.append((a, b, distance(result.points[a], result.points[b])))
                result.cells[a].neighbor_indices.append(b)
                result.cells[b].neighbor_indices.append(a)
            continue

        if len(owners) != 1:
            continue

        owner = owners[0]
        if owner >= len(result.cells):
            continue
        start, end = edge_geometry[key]
        length = math.dist(start, end)
        divisions = max(1, int(math.ceil(length / spacing)))
        for k in range(divisions):
            t = (k + 0.5) / divisions
            boundary_point = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                z,
            )
            boundary_index = len(result.points)
            result.points.append(boundary_point)
            result.point_cell_indices.append(owner)
            result.edges.append((
                owner,
                boundary_index,
                distance(result.points[owner], boundary_point),
            ))

    return result


def build_floor_cdt_graph(
    floor,
    z: float,
    *,
    boundary_spacing_m: float = 1.0,
) -> tuple[list[Vec3], list[Edge]]:
    result = build_floor_cdt_navmesh(
        floor,
        z,
        boundary_spacing_m=boundary_spacing_m,
    )
    return result.points, result.edges


def build_space_cdt_navmesh(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
    boundary_spacing_m: float = 1.0,
    obstacle_footprints: Iterable[BaseGeometry] = (),
) -> CdtNavMeshResult:
    floor = prepare_space_floor(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
        clearance_m=clearance_m,
        obstacle_footprints=obstacle_footprints,
    )
    return build_floor_cdt_navmesh(
        floor,
        _floor_elevation(vertices),
        boundary_spacing_m=boundary_spacing_m,
    )


def build_space_cdt_graph(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
    clearance_m: float = 0.0,
    boundary_spacing_m: float = 1.0,
    obstacle_footprints: Iterable[BaseGeometry] = (),
) -> tuple[list[Vec3], list[Edge]]:
    result = build_space_cdt_navmesh(
        vertices,
        triangles,
        floor_tolerance_m=floor_tolerance_m,
        max_slope_deg=max_slope_deg,
        clearance_m=clearance_m,
        boundary_spacing_m=boundary_spacing_m,
        obstacle_footprints=obstacle_footprints,
    )
    return result.points, result.edges


def space_floor_polygon(
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
    *,
    floor_tolerance_m: float = 0.12,
    max_slope_deg: float = 12.0,
):
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
