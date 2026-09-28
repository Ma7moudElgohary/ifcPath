from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .geometry import Vec3, distance
from .model import InavModel, NavCell


_EPSILON = 1e-9


@dataclass(slots=True)
class NavMeshRoute:
    points: list[Vec3]
    cell_ids: list[str]

    @property
    def length_m(self) -> float:
        return sum(distance(a, b) for a, b in zip(self.points, self.points[1:]))


def find_navmesh_path(
    model: InavModel,
    start: Vec3,
    goal: Vec3,
    *,
    space_id: str | None = None,
) -> NavMeshRoute | None:
    """Find a locally short route through the portable CDT navmesh.

    The dual graph is used only to choose a sequence of triangles. The returned
    geometric route is then string-pulled through the shared triangle edges with
    the funnel algorithm, avoiding the centroid-to-centroid zig-zag that a raw
    dual-graph route would produce.

    This routine is intentionally local/planar. Building-wide routing first uses
    semantic transitions to choose spaces/transfers and then applies this metric
    path inside each CDT-backed space.
    """
    candidate_cells = [
        cell for cell in model.cells
        if space_id is None or cell.space_id == space_id
    ]
    if not candidate_cells:
        return None

    start_cell = _find_cell(candidate_cells, start)
    goal_cell = _find_cell(candidate_cells, goal)
    if start_cell is None or goal_cell is None:
        return None

    if start_cell.id == goal_cell.id:
        return NavMeshRoute(points=[start, goal], cell_ids=[start_cell.id])

    cells = {cell.id: cell for cell in candidate_cells}
    corridor = _cell_corridor(cells, start_cell.id, goal_cell.id, goal)
    if not corridor:
        return None

    portals: list[tuple[Vec3, Vec3]] = [(start, start)]
    for current_id, next_id in zip(corridor, corridor[1:]):
        current = cells[current_id]
        nxt = cells[next_id]
        shared = _shared_edge(current, nxt)
        if shared is None:
            return None
        portals.append(_oriented_portal(current, nxt, shared))
    portals.append((goal, goal))

    points = _string_pull(portals)
    return NavMeshRoute(points=points, cell_ids=corridor)


def _find_cell(cells: list[NavCell], point: Vec3) -> NavCell | None:
    for cell in cells:
        if _point_in_triangle_xy(point, cell.vertices_m):
            return cell
    return None


def _cell_corridor(
    cells: dict[str, NavCell],
    start_id: str,
    goal_id: str,
    goal: Vec3,
) -> list[str]:
    queue: list[tuple[float, float, str]] = []
    start_center = _centroid(cells[start_id])
    heapq.heappush(queue, (_distance_xy(start_center, goal), 0.0, start_id))
    cost = {start_id: 0.0}
    previous: dict[str, str] = {}

    while queue:
        _, current_cost, current_id = heapq.heappop(queue)
        if current_id == goal_id:
            break
        if current_cost != cost.get(current_id):
            continue
        current = cells[current_id]
        current_center = _centroid(current)
        for neighbour_id in current.neighbor_ids:
            neighbour = cells.get(neighbour_id)
            if neighbour is None:
                continue
            neighbour_center = _centroid(neighbour)
            step = _distance_xy(current_center, neighbour_center)
            candidate = current_cost + step
            if candidate >= cost.get(neighbour_id, math.inf):
                continue
            cost[neighbour_id] = candidate
            previous[neighbour_id] = current_id
            heuristic = _distance_xy(neighbour_center, goal)
            heapq.heappush(queue, (candidate + heuristic, candidate, neighbour_id))

    if goal_id not in cost:
        return []

    corridor = [goal_id]
    while corridor[-1] != start_id:
        corridor.append(previous[corridor[-1]])
    corridor.reverse()
    return corridor


def _shared_edge(a: NavCell, b: NavCell) -> tuple[Vec3, Vec3] | None:
    a_by_key = {_xy_key(vertex): vertex for vertex in a.vertices_m}
    common = [a_by_key[key] for key in a_by_key if key in {_xy_key(v) for v in b.vertices_m}]
    if len(common) != 2:
        return None
    return common[0], common[1]


def _oriented_portal(
    current: NavCell,
    nxt: NavCell,
    shared: tuple[Vec3, Vec3],
) -> tuple[Vec3, Vec3]:
    """Return shared edge endpoints ordered as (left, right) along travel."""
    a, b = shared
    current_center = _centroid(current)
    next_center = _centroid(nxt)
    dx = next_center[0] - current_center[0]
    dy = next_center[1] - current_center[1]
    midpoint = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
    cross_a = dx * (a[1] - midpoint[1]) - dy * (a[0] - midpoint[0])
    cross_b = dx * (b[1] - midpoint[1]) - dy * (b[0] - midpoint[0])
    return (a, b) if cross_a >= cross_b else (b, a)


def _string_pull(portals: list[tuple[Vec3, Vec3]]) -> list[Vec3]:
    """Classic simple-stupid funnel algorithm over ordered left/right portals."""
    if not portals:
        return []

    path: list[Vec3] = [portals[0][0]]
    apex = portals[0][0]
    left = portals[0][0]
    right = portals[0][1]
    apex_index = 0
    left_index = 0
    right_index = 0
    i = 1

    while i < len(portals):
        new_left, new_right = portals[i]

        # Tighten the right side.
        if _triarea2(apex, right, new_right) <= _EPSILON:
            if _vequal(apex, right) or _triarea2(apex, left, new_right) > _EPSILON:
                right = new_right
                right_index = i
            else:
                path.append(left)
                apex = left
                apex_index = left_index
                left = apex
                right = apex
                left_index = apex_index
                right_index = apex_index
                i = apex_index + 1
                continue

        # Tighten the left side.
        if _triarea2(apex, left, new_left) >= -_EPSILON:
            if _vequal(apex, left) or _triarea2(apex, right, new_left) < -_EPSILON:
                left = new_left
                left_index = i
            else:
                path.append(right)
                apex = right
                apex_index = right_index
                left = apex
                right = apex
                left_index = apex_index
                right_index = apex_index
                i = apex_index + 1
                continue

        i += 1

    goal = portals[-1][0]
    if not _vequal(path[-1], goal):
        path.append(goal)
    return _remove_consecutive_duplicates(path)


def _point_in_triangle_xy(point: Vec3, triangle: tuple[Vec3, Vec3, Vec3]) -> bool:
    a, b, c = triangle
    area = _triarea2(a, b, c)
    if abs(area) <= _EPSILON:
        return False
    s1 = _triarea2(a, b, point)
    s2 = _triarea2(b, c, point)
    s3 = _triarea2(c, a, point)
    if area > 0:
        return s1 >= -_EPSILON and s2 >= -_EPSILON and s3 >= -_EPSILON
    return s1 <= _EPSILON and s2 <= _EPSILON and s3 <= _EPSILON


def _centroid(cell: NavCell) -> Vec3:
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )


def _triarea2(a: Vec3, b: Vec3, c: Vec3) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _distance_xy(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _xy_key(point: Vec3) -> tuple[int, int]:
    scale = 1_000_000_000.0
    return round(point[0] * scale), round(point[1] * scale)


def _vequal(a: Vec3, b: Vec3) -> bool:
    return _distance_xy(a, b) <= _EPSILON


def _remove_consecutive_duplicates(points: list[Vec3]) -> list[Vec3]:
    result: list[Vec3] = []
    for point in points:
        if not result or not _vequal(result[-1], point):
            result.append(point)
    return result
