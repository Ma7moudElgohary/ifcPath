from __future__ import annotations

from dataclasses import dataclass

from .model import NavCell, Vec3
from .navmesh_routing import _oriented_portal, _string_pull
from .surface_nav import _weighted_cell_corridor, cell_centroid, closest_cell


@dataclass(slots=True)
class SurfaceFunnelRoute:
    """A polygon-cell corridor and its funnel-shortened XYZ centreline."""

    points: list[Vec3]
    cell_ids: list[str]

    @property
    def length_m(self) -> float:
        import math

        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))


def find_surface_funnel_route(
    cells: list[NavCell],
    start: Vec3,
    goal: Vec3,
    *,
    terrain_costs: dict[str, float] | None = None,
    blocked_portal_ids: set[str] | None = None,
) -> SurfaceFunnelRoute | None:
    """Project arbitrary XYZ endpoints, A* a cell corridor, then funnel it.

    This is the global-surface equivalent of ``find_navmesh_path``. Unlike the
    old midpoint route, it consumes the full geometric crossing segment stored
    on every cell adjacency. Semantic doors, floor/stair seams and exact shared
    edges therefore all participate in the same corridor/string-pulling model.

    The classic funnel is evaluated in IFC XY, matching the qualified CDT
    implementation. Returned corner vertices retain the portal Z value, so
    sloped/vertical-circulation corridors remain 3D even though left/right
    ordering is determined in plan.
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
    portals: list[tuple[Vec3, Vec3]] = [(start_point, start_point)]
    for current_id, next_id in zip(corridor, corridor[1:]):
        current = by_id[current_id]
        nxt = by_id[next_id]
        segment = _portal_segment(current, nxt)
        portals.append(_oriented_portal(current, nxt, segment))
    portals.append((goal_point, goal_point))

    return SurfaceFunnelRoute(_string_pull(portals), corridor)


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

    # Fail geometrically conservative: a degenerate point portal at the midpoint
    # preserves graph reachability but cannot invent corridor width.
    ca, cb = cell_centroid(a), cell_centroid(b)
    midpoint = (
        (ca[0] + cb[0]) * 0.5,
        (ca[1] + cb[1]) * 0.5,
        (ca[2] + cb[2]) * 0.5,
    )
    return midpoint, midpoint


def _distance_sq(a: Vec3, b: Vec3) -> float:
    return sum((a[index] - b[index]) ** 2 for index in range(3))
