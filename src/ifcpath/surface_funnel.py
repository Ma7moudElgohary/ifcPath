from __future__ import annotations

import math
from dataclasses import dataclass

from .model import NavCell, Vec3
from .navmesh_routing import _string_pull
from .surface_nav import _weighted_cell_corridor, cell_centroid, closest_cell


_EPSILON = 1e-9
_COPLANAR_TOLERANCE_M = 1e-3


@dataclass(slots=True)
class SurfaceFunnelRoute:
    """A polygon-cell corridor and its surface-constrained XYZ centreline."""

    points: list[Vec3]
    cell_ids: list[str]

    @property
    def length_m(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))


def find_surface_funnel_route(
    cells: list[NavCell],
    start: Vec3,
    goal: Vec3,
    *,
    terrain_costs: dict[str, float] | None = None,
    blocked_portal_ids: set[str] | None = None,
) -> SurfaceFunnelRoute | None:
    """Project XYZ endpoints, A* a cell corridor, then shorten it on the surface.

    A classic funnel is a 2D algorithm. Applying it directly in IFC XY is safe
    only while the corridor lies on one geometric plane. Stairs and stepped
    landings are folded 3D surfaces: different treads can overlap in XY while
    having different Z values, so an XY funnel can incorrectly collapse several
    steps into one chord through empty space.

    The router therefore splits the corridor at every non-coplanar cell fold.
    Each coplanar run is projected into its own metric 2D basis and funnelled
    there. At a fold, the stored crossing portal is projected back onto both
    adjacent triangles, producing an explicit exit/entry pair. The final route
    is consequently constrained to the authoritative NavCell surface while flat
    floors and planar ramps keep true funnel shortening.
    """
    terrain_costs = terrain_costs or {}
    blocked = set(blocked_portal_ids or ())
    start_match = closest_cell(cells, start)
    goal_match = closest_cell(cells, goal)
    if start_match is None or goal_match is None:
        return None

    start_cell, start_point, _ = start_match
    goal_cell, goal_point, _ = goal_match
    corridor = _weighted_cell_corridor(
        cells,
        start_cell.id,
        goal_cell.id,
        terrain_costs,
        blocked,
    )
    if not corridor:
        return None
    if len(corridor) == 1:
        return SurfaceFunnelRoute([start_point, goal_point], corridor)

    by_id = {cell.id: cell for cell in cells}
    points = _fold_aware_corridor_route(by_id, corridor, start_point, goal_point)
    if not points:
        return None
    return SurfaceFunnelRoute(points, corridor)


def _fold_aware_corridor_route(
    by_id: dict[str, NavCell],
    corridor: list[str],
    start: Vec3,
    goal: Vec3,
) -> list[Vec3]:
    result: list[Vec3] = []
    run_start_index = 0
    run_start_point = start

    for index in range(len(corridor) - 1):
        current = by_id[corridor[index]]
        nxt = by_id[corridor[index + 1]]
        if _cells_coplanar(current, nxt):
            continue

        portal = _portal_segment(current, nxt)
        midpoint = _segment_midpoint(portal)
        exit_point = _closest_point_on_triangle(midpoint, *current.vertices_m)
        entry_point = _closest_point_on_triangle(midpoint, *nxt.vertices_m)

        run = corridor[run_start_index : index + 1]
        _append_points(
            result,
            _planar_funnel_run(by_id, run, run_start_point, exit_point),
        )
        _append_points(result, [entry_point])

        run_start_index = index + 1
        run_start_point = entry_point

    _append_points(
        result,
        _planar_funnel_run(
            by_id,
            corridor[run_start_index:],
            run_start_point,
            goal,
        ),
    )
    return result


def _planar_funnel_run(
    by_id: dict[str, NavCell],
    corridor: list[str],
    start: Vec3,
    goal: Vec3,
) -> list[Vec3]:
    if not corridor:
        return []
    if len(corridor) == 1:
        return _dedupe_3d([start, goal])

    basis = _plane_basis(by_id[corridor[0]])
    if basis is None:
        return _safe_portal_midpoint_route(by_id, corridor, start, goal)
    origin, axis_u, axis_v, _ = basis

    def project(point: Vec3) -> Vec3:
        delta = _sub(point, origin)
        return (_dot(delta, axis_u), _dot(delta, axis_v), 0.0)

    def unproject(point: Vec3) -> Vec3:
        return _add(origin, _add(_scale(axis_u, point[0]), _scale(axis_v, point[1])))

    projected_portals: list[tuple[Vec3, Vec3]] = [(project(start), project(start))]
    for current_id, next_id in zip(corridor, corridor[1:]):
        current = by_id[current_id]
        nxt = by_id[next_id]
        a, b = _portal_segment(current, nxt)
        pa, pb = project(a), project(b)
        projected_portals.append(
            _oriented_projected_portal(
                project(cell_centroid(current)),
                project(cell_centroid(nxt)),
                (pa, pb),
            )
        )
    projected_portals.append((project(goal), project(goal)))

    pulled = _string_pull(projected_portals)
    route = [unproject(point) for point in pulled]
    if route:
        route[0] = start
        route[-1] = goal
    return _dedupe_3d(route)


def _safe_portal_midpoint_route(
    by_id: dict[str, NavCell],
    corridor: list[str],
    start: Vec3,
    goal: Vec3,
) -> list[Vec3]:
    points = [start]
    for current_id, next_id in zip(corridor, corridor[1:]):
        points.append(_segment_midpoint(_portal_segment(by_id[current_id], by_id[next_id])))
    points.append(goal)
    return _dedupe_3d(points)


def _oriented_projected_portal(
    current_center: Vec3,
    next_center: Vec3,
    segment: tuple[Vec3, Vec3],
) -> tuple[Vec3, Vec3]:
    a, b = segment
    dx = next_center[0] - current_center[0]
    dy = next_center[1] - current_center[1]
    midpoint = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
    cross_a = dx * (a[1] - midpoint[1]) - dy * (a[0] - midpoint[0])
    cross_b = dx * (b[1] - midpoint[1]) - dy * (b[0] - midpoint[0])
    return (b, a) if cross_a >= cross_b else (a, b)


def _cells_coplanar(a: NavCell, b: NavCell, tolerance_m: float = _COPLANAR_TOLERANCE_M) -> bool:
    basis = _plane_basis(a)
    if basis is None:
        return False
    origin, _, _, normal = basis
    return all(abs(_dot(_sub(vertex, origin), normal)) <= tolerance_m for vertex in b.vertices_m)


def _plane_basis(cell: NavCell) -> tuple[Vec3, Vec3, Vec3, Vec3] | None:
    a, b, c = cell.vertices_m
    ab = _sub(b, a)
    ac = _sub(c, a)
    normal = _cross(ab, ac)
    normal_length = _length(normal)
    if normal_length <= _EPSILON:
        return None
    normal = _scale(normal, 1.0 / normal_length)

    axis_u = ab if _length(ab) > _EPSILON else ac
    axis_u_length = _length(axis_u)
    if axis_u_length <= _EPSILON:
        return None
    axis_u = _scale(axis_u, 1.0 / axis_u_length)
    axis_v = _cross(normal, axis_u)
    axis_v_length = _length(axis_v)
    if axis_v_length <= _EPSILON:
        return None
    axis_v = _scale(axis_v, 1.0 / axis_v_length)
    return a, axis_u, axis_v, normal


def _portal_segment(a: NavCell, b: NavCell, tolerance_m: float = 1e-4) -> tuple[Vec3, Vec3]:
    stored = a.portals.get(b.id) or b.portals.get(a.id)
    if stored is not None:
        return stored

    common: list[Vec3] = []
    for pa in a.vertices_m:
        if any(_distance_sq(pa, pb) <= tolerance_m * tolerance_m for pb in b.vertices_m):
            common.append(pa)
    if len(common) >= 2:
        return common[0], common[1]

    # Geometrically conservative fallback: preserve graph reachability without
    # inventing a wide crossing where no exact shared edge is available.
    ca, cb = cell_centroid(a), cell_centroid(b)
    midpoint = (
        (ca[0] + cb[0]) * 0.5,
        (ca[1] + cb[1]) * 0.5,
        (ca[2] + cb[2]) * 0.5,
    )
    return midpoint, midpoint


def _segment_midpoint(segment: tuple[Vec3, Vec3]) -> Vec3:
    a, b = segment
    return (
        (a[0] + b[0]) * 0.5,
        (a[1] + b[1]) * 0.5,
        (a[2] + b[2]) * 0.5,
    )


def _append_points(target: list[Vec3], points: list[Vec3]) -> None:
    for point in points:
        if not target or math.dist(target[-1], point) > _EPSILON:
            target.append(point)


def _dedupe_3d(points: list[Vec3]) -> list[Vec3]:
    result: list[Vec3] = []
    _append_points(result, points)
    return result


def _closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    # Christer Ericson, Real-Time Collision Detection, closest-point regions.
    ab = _sub(b, a)
    ac = _sub(c, a)
    ap = _sub(p, a)
    d1 = _dot(ab, ap)
    d2 = _dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = _sub(p, b)
    d3 = _dot(ab, bp)
    d4 = _dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return _add(a, _scale(ab, v))

    cp = _sub(p, c)
    d5 = _dot(ab, cp)
    d6 = _dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return _add(a, _scale(ac, w))

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return _add(b, _scale(_sub(c, b), w))

    denominator = 1.0 / (va + vb + vc)
    v = vb * denominator
    w = vc * denominator
    return _add(a, _add(_scale(ab, v), _scale(ac, w)))


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _add(a: Vec3, b: Vec3) -> Vec3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _scale(a: Vec3, value: float) -> Vec3:
    return a[0] * value, a[1] * value, a[2] * value


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _distance_sq(a: Vec3, b: Vec3) -> float:
    return sum((a[index] - b[index]) ** 2 for index in range(3))
