from __future__ import annotations

import heapq
import math
from collections import defaultdict

from .geometry import triangle_normal
from .model import NavCell, Vec3


def walkable_surface_cells(
    vertices,
    triangles,
    *,
    id_prefix,
    terrain,
    max_slope_deg,
    level_id=None,
    space_id=None,
):
    min_up = math.cos(math.radians(max_slope_deg))
    cells: list[NavCell] = []
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        if triangle_normal(a, b, c)[2] < min_up:
            continue
        cells.append(
            NavCell(
                id=f"{id_prefix}:{len(cells)}",
                vertices_m=(a, b, c),
                space_id=space_id,
                level_id=level_id,
                terrain=terrain,
            )
        )
    connect_cells_by_shared_edges(cells)
    return cells


def connect_cells_by_shared_edges(cells, tolerance_m=1e-5):
    """Connect cells that share the same 3D edge.

    Room-to-room ``open`` adjacency is intentionally rejected here. A real door
    must authorize that crossing later through ``connect_cells_through_portal``.
    """
    scale = 1.0 / max(tolerance_m, 1e-9)
    owners: dict[tuple, list[int]] = defaultdict(list)

    def quantize(point):
        return (
            round(point[0] * scale),
            round(point[1] * scale),
            round(point[2] * scale),
        )

    for index, cell in enumerate(cells):
        v = cell.vertices_m
        for a, b in ((v[0], v[1]), (v[1], v[2]), (v[2], v[0])):
            qa, qb = quantize(a), quantize(b)
            owners[(qa, qb) if qa <= qb else (qb, qa)].append(index)

    for shared_key, indices in owners.items():
        if len(indices) != 2:
            continue
        ai, bi = indices
        a, b = cells[ai], cells[bi]
        if (
            a.terrain == b.terrain == "open"
            and a.space_id
            and b.space_id
            and a.space_id != b.space_id
        ):
            continue
        _connect_pair(a, b)
        pa = tuple(value / scale for value in shared_key[0])
        pb = tuple(value / scale for value in shared_key[1])
        a.portals[b.id] = (pa, pb)
        b.portals[a.id] = (pa, pb)


def connect_cells_through_portal(
    cells: list[NavCell],
    *,
    point: Vec3,
    from_space_id: str | None,
    to_space_id: str | None,
    portal_id: str,
    width_m: float | None = None,
    level_id: str | None = None,
    max_distance_m: float = 2.5,
) -> bool:
    """Authorize one semantic door crossing in the metric surface graph.

    Candidate cells on both room sides are ranked by geometric distance to the
    authored door position. The nearest *free* pair is selected so two nearby
    doors never overwrite the one semantic portal ID that a NavCell adjacency
    can store. Repeating the same portal is idempotent and may reuse its pair.
    """
    if not from_space_id or not to_space_id or from_space_id == to_space_id:
        return False

    def candidates(space_id: str) -> list[NavCell]:
        exact_level = [
            cell
            for cell in cells
            if cell.space_id == space_id
            and (level_id is None or cell.level_id is None or cell.level_id == level_id)
        ]
        return exact_level or [cell for cell in cells if cell.space_id == space_id]

    def ranked(space_id: str):
        matches = []
        for cell in candidates(space_id):
            projected = _closest_point_on_triangle(point, *cell.vertices_m)
            distance = math.dist(point, projected)
            if distance <= max_distance_m:
                matches.append((distance, cell.id, projected, cell))
        matches.sort(key=lambda item: (item[0], item[1]))
        return matches

    left = ranked(from_space_id)
    right = ranked(to_space_id)
    if not left or not right:
        return False

    pair_candidates = []
    for da, _, qa, a in left:
        for db, _, qb, b in right:
            if a.id == b.id:
                continue
            existing_left = a.portal_ids.get(b.id)
            existing_right = b.portal_ids.get(a.id)
            existing_ids = {
                existing
                for existing in (existing_left, existing_right)
                if existing is not None
            }
            if existing_ids and existing_ids != {portal_id}:
                continue
            pair_candidates.append(
                (
                    da + db,
                    max(da, db),
                    a.id,
                    b.id,
                    a,
                    qa,
                    b,
                    qb,
                )
            )

    if not pair_candidates:
        return False

    pair_candidates.sort(key=lambda item: item[:4])
    _, _, _, _, a, qa, b, qb = pair_candidates[0]
    _connect_pair(a, b)

    ac = cell_centroid(a)
    bc = cell_centroid(b)
    dx, dy = bc[0] - ac[0], bc[1] - ac[1]
    planar = math.hypot(dx, dy)
    if planar <= 1e-9:
        dx, dy, planar = 1.0, 0.0, 1.0
    # Door plane tangent is perpendicular to room-to-room traversal direction.
    tx, ty = -dy / planar, dx / planar
    width = max(0.20, float(width_m or 0.90))
    half = width * 0.5
    center = (
        (qa[0] + qb[0] + 2.0 * point[0]) * 0.25,
        (qa[1] + qb[1] + 2.0 * point[1]) * 0.25,
        (qa[2] + qb[2]) * 0.5,
    )
    portal = (
        (center[0] - tx * half, center[1] - ty * half, center[2]),
        (center[0] + tx * half, center[1] + ty * half, center[2]),
    )
    a.portals[b.id] = portal
    b.portals[a.id] = portal
    a.portal_ids[b.id] = portal_id
    b.portal_ids[a.id] = portal_id
    return True


def _connect_pair(a: NavCell, b: NavCell) -> None:
    if b.id not in a.neighbor_ids:
        a.neighbor_ids.append(b.id)
    if a.id not in b.neighbor_ids:
        b.neighbor_ids.append(a.id)


def cell_centroid(cell: NavCell) -> Vec3:
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3,
        (a[1] + b[1] + c[1]) / 3,
        (a[2] + b[2] + c[2]) / 3,
    )


def find_cell_corridor(cells, start_cell_id, goal_cell_id, blocked_portal_ids=None):
    return _weighted_cell_corridor(
        cells,
        start_cell_id,
        goal_cell_id,
        terrain_costs={},
        blocked_portal_ids=set(blocked_portal_ids or ()),
    )


def stitch_surface_seams(cells, max_gap_m=0.20, max_vertical_gap_m=0.30):
    """Join independently tessellated floor/landing/circulation boundary edges."""
    by_id = {cell.id: cell for cell in cells}
    connected = {
        tuple(sorted((cell.id, neighbor)))
        for cell in cells
        for neighbor in cell.neighbor_ids
    }
    counts = defaultdict(int)
    data = {}
    scale = 1e5

    def key(a, b):
        qa = tuple(round(v * scale) for v in a)
        qb = tuple(round(v * scale) for v in b)
        return (qa, qb) if qa <= qb else (qb, qa)

    for cell in cells:
        v = cell.vertices_m
        for a, b in ((v[0], v[1]), (v[1], v[2]), (v[2], v[0])):
            edge = key(a, b)
            counts[edge] += 1
            data.setdefault(edge, (cell.id, a, b))

    boundaries = [data[edge] for edge, count in counts.items() if count == 1]
    added = 0
    for index, (aid, a0, a1) in enumerate(boundaries):
        for bid, b0, b1 in boundaries[index + 1 :]:
            pair = tuple(sorted((aid, bid)))
            if aid == bid or pair in connected:
                continue
            a, b = by_id[aid], by_id[bid]
            if a.terrain == b.terrain == "open" and a.space_id != b.space_id:
                continue
            pa, pb = _closest_segment_points(a0, a1, b0, b1)
            if (
                abs(pa[2] - pb[2]) > max_vertical_gap_m
                or math.dist(pa, pb) > max_gap_m
            ):
                continue

            center = tuple((pa[k] + pb[k]) / 2 for k in range(3))
            direction = (
                a1[0] - a0[0],
                a1[1] - a0[1],
                a1[2] - a0[2],
            )
            length = math.sqrt(sum(v * v for v in direction))
            if length > 1e-9:
                unit = tuple(v / length for v in direction)
                half = min(0.25 * length, 0.10)
                portal = (
                    tuple(center[k] - unit[k] * half for k in range(3)),
                    tuple(center[k] + unit[k] * half for k in range(3)),
                )
            else:
                portal = (center, center)

            _connect_pair(a, b)
            a.portals[b.id] = portal
            b.portals[a.id] = portal
            connected.add(pair)
            added += 1
    return added


def _closest_segment_points(p1, q1, p2, q2):
    """Closest points on two 3D segments (Real-Time Collision Detection)."""
    d1 = tuple(q1[i] - p1[i] for i in range(3))
    d2 = tuple(q2[i] - p2[i] for i in range(3))
    r = tuple(p1[i] - p2[i] for i in range(3))
    a = sum(v * v for v in d1)
    e = sum(v * v for v in d2)
    f = sum(d2[i] * r[i] for i in range(3))
    eps = 1e-12
    if a <= eps and e <= eps:
        return p1, p2
    if a <= eps:
        ss = 0.0
        tt = max(0.0, min(1.0, f / e))
    else:
        c = sum(d1[i] * r[i] for i in range(3))
        if e <= eps:
            tt = 0.0
            ss = max(0.0, min(1.0, -c / a))
        else:
            b = sum(d1[i] * d2[i] for i in range(3))
            denom = a * e - b * b
            ss = 0.0 if abs(denom) <= eps else max(0.0, min(1.0, (b * f - c * e) / denom))
            tt = (b * ss + f) / e
            if tt < 0.0:
                tt = 0.0
                ss = max(0.0, min(1.0, -c / a))
            elif tt > 1.0:
                tt = 1.0
                ss = max(0.0, min(1.0, (b - c) / a))
    return (
        tuple(p1[i] + d1[i] * ss for i in range(3)),
        tuple(p2[i] + d2[i] * tt for i in range(3)),
    )


def surface_components(cells):
    by_id = {cell.id: cell for cell in cells}
    remaining = set(by_id)
    result = []
    while remaining:
        start = remaining.pop()
        component = {start}
        stack = [start]
        while stack:
            current = stack.pop()
            for neighbor in by_id[current].neighbor_ids:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        result.append(component)
    return result


def closest_cell(cells, point):
    """Return the cell whose triangle is closest to an arbitrary 3D point."""
    best = None
    for cell in cells:
        projected = _closest_point_on_triangle(point, *cell.vertices_m)
        distance = math.dist(point, projected)
        candidate = (distance, cell.id, projected, cell)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    return None if best is None else (best[3], best[2], best[0])


def find_surface_route(
    cells,
    start,
    goal,
    terrain_costs=None,
    blocked_portal_ids=None,
):
    """Project endpoints to the nav surface and return a smoothed XYZ route."""
    terrain_costs = terrain_costs or {}
    blocked = set(blocked_portal_ids or ())
    start_match = closest_cell(cells, start)
    goal_match = closest_cell(cells, goal)
    if start_match is None or goal_match is None:
        return []
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
        return []
    by_id = {cell.id: cell for cell in cells}
    route = [start_point]
    for a_id, b_id in zip(corridor, corridor[1:]):
        portal = _cell_portal(by_id[a_id], by_id[b_id])
        route.append(portal if portal is not None else cell_centroid(by_id[b_id]))
    route.append(goal_point)
    return _string_pull_polyline(route)


def _weighted_cell_corridor(
    cells,
    start_id,
    goal_id,
    terrain_costs,
    blocked_portal_ids=frozenset(),
):
    by_id = {cell.id: cell for cell in cells}
    if start_id not in by_id or goal_id not in by_id:
        return []
    goal = cell_centroid(by_id[goal_id])
    queue = [(0.0, start_id)]
    g = {start_id: 0.0}
    previous = {}
    while queue:
        _, current_id = heapq.heappop(queue)
        if current_id == goal_id:
            break
        current = by_id[current_id]
        current_point = cell_centroid(current)
        for neighbor_id in current.neighbor_ids:
            neighbor = by_id.get(neighbor_id)
            if neighbor is None:
                continue
            semantic_portal = current.portal_ids.get(neighbor_id) or neighbor.portal_ids.get(current_id)
            if semantic_portal in blocked_portal_ids:
                continue
            neighbor_point = cell_centroid(neighbor)
            multiplier = max(1.0, float(terrain_costs.get(neighbor.terrain, 1.0)))
            candidate = g[current_id] + math.dist(current_point, neighbor_point) * multiplier
            if candidate >= g.get(neighbor_id, math.inf):
                continue
            g[neighbor_id] = candidate
            previous[neighbor_id] = current_id
            heapq.heappush(
                queue,
                (candidate + math.dist(neighbor_point, goal), neighbor_id),
            )
    if goal_id not in g:
        return []
    result = [goal_id]
    while result[-1] != start_id:
        result.append(previous[result[-1]])
    return list(reversed(result))


def _cell_portal(a, b, tol=1e-4):
    if b.id in a.portals:
        p, q = a.portals[b.id]
        return (
            (p[0] + q[0]) / 2,
            (p[1] + q[1]) / 2,
            (p[2] + q[2]) / 2,
        )
    pairs = []
    for pa in a.vertices_m:
        for pb in b.vertices_m:
            distance = math.dist(pa, pb)
            if distance <= tol:
                pairs.append(
                    (
                        (pa[0] + pb[0]) / 2,
                        (pa[1] + pb[1]) / 2,
                        (pa[2] + pb[2]) / 2,
                    )
                )
    if len(pairs) >= 2:
        p, q = pairs[0], pairs[1]
        return (
            (p[0] + q[0]) / 2,
            (p[1] + q[1]) / 2,
            (p[2] + q[2]) / 2,
        )
    ca, cb = cell_centroid(a), cell_centroid(b)
    return (
        (ca[0] + cb[0]) / 2,
        (ca[1] + cb[1]) / 2,
        (ca[2] + cb[2]) / 2,
    )


def _string_pull_polyline(points, eps=1e-6):
    if len(points) <= 2:
        return points
    result = [points[0]]
    for index in range(1, len(points) - 1):
        a, b, c = result[-1], points[index], points[index + 1]
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        bc = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
        cross = (
            ab[1] * bc[2] - ab[2] * bc[1],
            ab[2] * bc[0] - ab[0] * bc[2],
            ab[0] * bc[1] - ab[1] * bc[0],
        )
        if math.sqrt(sum(v * v for v in cross)) > eps:
            result.append(b)
    result.append(points[-1])
    return result


def _closest_point_on_triangle(p, a, b, c):
    # Christer Ericson, Real-Time Collision Detection, closest-point regions.
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    ap = tuple(p[i] - a[i] for i in range(3))
    d1 = sum(ab[i] * ap[i] for i in range(3))
    d2 = sum(ac[i] * ap[i] for i in range(3))
    if d1 <= 0 and d2 <= 0:
        return a
    bp = tuple(p[i] - b[i] for i in range(3))
    d3 = sum(ab[i] * bp[i] for i in range(3))
    d4 = sum(ac[i] * bp[i] for i in range(3))
    if d3 >= 0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        v = d1 / (d1 - d3)
        return tuple(a[i] + v * ab[i] for i in range(3))
    cp = tuple(p[i] - c[i] for i in range(3))
    d5 = sum(ab[i] * cp[i] for i in range(3))
    d6 = sum(ac[i] * cp[i] for i in range(3))
    if d6 >= 0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        w = d2 / (d2 - d6)
        return tuple(a[i] + w * ac[i] for i in range(3))
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return tuple(b[i] + w * (c[i] - b[i]) for i in range(3))
    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return tuple(a[i] + ab[i] * v + ac[i] * w for i in range(3))
