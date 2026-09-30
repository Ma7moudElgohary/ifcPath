from __future__ import annotations

import math
from collections import defaultdict

from .model import NavCell, Vec3
from .surface_nav import _connect_pair, _closest_segment_points


_VERTICAL_TERRAINS = {"stair", "ramp", "escalator"}


def stitch_clearance_aware_seams(
    cells: list[NavCell],
    *,
    max_gap_m: float = 0.20,
    max_vertical_gap_m: float = 0.30,
    edge_clearance_m: float = 0.18,
    parallel_tolerance_deg: float = 12.0,
    vertical_only: bool = False,
) -> int:
    """Stitch independent support meshes using the widest valid edge crossing.

    For each unconnected cell pair every relevant boundary-edge combination is
    evaluated. This is important for stairs: a corner contact may be geometrically
    valid but a parallel tread/landing overlap is the useful crossing. The widest
    valid portal wins, and parallel overlaps are trimmed by body clearance so the
    funnel cannot be anchored at a railing endpoint.

    ``vertical_only`` is used by the physical IFC reconstruction pipeline. Once a
    unified multi-layer floor field exists, a missing flat/open sample is evidence
    of an obstacle or insufficient clearance and must *not* be healed by proximity.
    Only stair/ramp/escalator boundaries may bridge a sampling/model tolerance gap.

    In that mode candidate enumeration starts only from vertical boundary edges.
    This preserves the exact same geometric candidates while reducing the common
    case from all-boundary O(B²) comparisons to O(V*B), where V is the much smaller
    number of stair/ramp/escalator boundary edges.
    """
    if not cells:
        return 0

    by_id = {cell.id: cell for cell in cells}
    connected = {
        tuple(sorted((cell.id, neighbor)))
        for cell in cells
        for neighbor in cell.neighbor_ids
    }
    counts: dict[tuple, int] = defaultdict(int)
    data: dict[tuple, tuple[str, Vec3, Vec3]] = {}
    scale = 100_000.0

    def key(a: Vec3, b: Vec3):
        qa = tuple(round(value * scale) for value in a)
        qb = tuple(round(value * scale) for value in b)
        return (qa, qb) if qa <= qb else (qb, qa)

    for cell in cells:
        v = cell.vertices_m
        for start, end in ((v[0], v[1]), (v[1], v[2]), (v[2], v[0])):
            edge = key(start, end)
            counts[edge] += 1
            data.setdefault(edge, (cell.id, start, end))

    boundaries = [data[edge] for edge, count in counts.items() if count == 1]
    best: dict[
        tuple[str, str],
        tuple[float, float, NavCell, NavCell, tuple[Vec3, Vec3]],
    ] = {}

    if vertical_only:
        vertical_indices = [
            index
            for index, (cell_id, _, _) in enumerate(boundaries)
            if by_id[cell_id].terrain in _VERTICAL_TERRAINS
        ]
        vertical_index_set = set(vertical_indices)
        candidate_pairs = (
            (index, other_index)
            for index in vertical_indices
            for other_index in range(len(boundaries))
            if index != other_index
            # A vertical/vertical edge pair would otherwise be visited twice.
            and not (other_index in vertical_index_set and other_index < index)
        )
    else:
        candidate_pairs = (
            (index, other_index)
            for index in range(len(boundaries))
            for other_index in range(index + 1, len(boundaries))
        )

    for index, other_index in candidate_pairs:
        aid, a0, a1 = boundaries[index]
        bid, b0, b1 = boundaries[other_index]
        pair = tuple(sorted((aid, bid)))
        if aid == bid or pair in connected:
            continue
        a, b = by_id[aid], by_id[bid]
        if (
            a.terrain == b.terrain == "open"
            and a.space_id
            and b.space_id
            and a.space_id != b.space_id
        ):
            # Cross-space floor travel must still be authorised by a semantic
            # door/open boundary later; geometry alone cannot create it.
            continue

        candidate = _overlap_portal(
            a0,
            a1,
            b0,
            b1,
            max_gap_m=max_gap_m,
            max_vertical_gap_m=max_vertical_gap_m,
            edge_clearance_m=edge_clearance_m,
            parallel_tolerance_deg=parallel_tolerance_deg,
        )
        if candidate is None:
            continue
        portal, separation = candidate
        width = math.dist(portal[0], portal[1])
        ranking = (width, -separation)
        previous = best.get(pair)
        if previous is None or ranking > previous[:2]:
            best[pair] = (width, -separation, a, b, portal)

    added = 0
    for pair in sorted(best):
        _, _, a, b, portal = best[pair]
        _connect_pair(a, b)
        a.portals[b.id] = portal
        b.portals[a.id] = portal
        connected.add(pair)
        added += 1
    return added


def _overlap_portal(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
    *,
    max_gap_m: float,
    max_vertical_gap_m: float,
    edge_clearance_m: float,
    parallel_tolerance_deg: float,
) -> tuple[tuple[Vec3, Vec3], float] | None:
    da = _sub(a1, a0)
    db = _sub(b1, b0)
    la_xy = math.hypot(da[0], da[1])
    lb_xy = math.hypot(db[0], db[1])

    if la_xy > 1e-8 and lb_xy > 1e-8:
        ua = (da[0] / la_xy, da[1] / la_xy)
        ub = (db[0] / lb_xy, db[1] / lb_xy)
        parallel = abs(ua[0] * ub[0] + ua[1] * ub[1]) >= math.cos(
            math.radians(parallel_tolerance_deg)
        )
        if parallel:
            candidate = _parallel_overlap_portal(
                a0,
                a1,
                b0,
                b1,
                ua,
                max_gap_m=max_gap_m,
                max_vertical_gap_m=max_vertical_gap_m,
                edge_clearance_m=edge_clearance_m,
            )
            if candidate is not None:
                return candidate

    # Corners and genuinely non-parallel seams use a conservative point contact.
    pa, pb = _closest_segment_points(a0, a1, b0, b1)
    separation = math.dist(pa, pb)
    if abs(pa[2] - pb[2]) > max_vertical_gap_m or separation > max_gap_m:
        return None
    center = _midpoint(pa, pb)
    return (center, center), separation


def _parallel_overlap_portal(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
    axis_xy: tuple[float, float],
    *,
    max_gap_m: float,
    max_vertical_gap_m: float,
    edge_clearance_m: float,
) -> tuple[tuple[Vec3, Vec3], float] | None:
    origin_xy = (a0[0], a0[1])

    def parameter(point: Vec3) -> float:
        return (
            (point[0] - origin_xy[0]) * axis_xy[0]
            + (point[1] - origin_xy[1]) * axis_xy[1]
        )

    a_values = sorted((parameter(a0), parameter(a1)))
    b_values = sorted((parameter(b0), parameter(b1)))
    low = max(a_values[0], b_values[0])
    high = min(a_values[1], b_values[1])
    if high <= low + 1e-6:
        return None

    # Check true geometric separation at the overlap midpoint, not merely that
    # infinite projected lines are parallel.
    middle = (low + high) * 0.5
    pa_mid = _point_at_axis_parameter(a0, a1, axis_xy, origin_xy, middle)
    pb_mid = _point_at_axis_parameter(b0, b1, axis_xy, origin_xy, middle)
    separation = math.dist(pa_mid, pb_mid)
    if (
        abs(pa_mid[2] - pb_mid[2]) > max_vertical_gap_m
        or separation > max_gap_m
    ):
        return None

    overlap = high - low
    clearance = max(0.0, edge_clearance_m)
    if overlap > 2.0 * clearance + 0.05:
        low += clearance
        high -= clearance
    else:
        # A real narrow contact may still be traversable, but never fabricate a
        # wide portal. Collapse it to the centre instead.
        low = high = (low + high) * 0.5

    pa0 = _point_at_axis_parameter(a0, a1, axis_xy, origin_xy, low)
    pb0 = _point_at_axis_parameter(b0, b1, axis_xy, origin_xy, low)
    pa1 = _point_at_axis_parameter(a0, a1, axis_xy, origin_xy, high)
    pb1 = _point_at_axis_parameter(b0, b1, axis_xy, origin_xy, high)
    return (_midpoint(pa0, pb0), _midpoint(pa1, pb1)), separation


def _point_at_axis_parameter(
    start: Vec3,
    end: Vec3,
    axis_xy: tuple[float, float],
    origin_xy: tuple[float, float],
    target: float,
) -> Vec3:
    def parameter(point: Vec3) -> float:
        return (
            (point[0] - origin_xy[0]) * axis_xy[0]
            + (point[1] - origin_xy[1]) * axis_xy[1]
        )

    start_param = parameter(start)
    end_param = parameter(end)
    length_param = end_param - start_param
    if abs(length_param) <= 1e-10:
        return start
    t = (target - start_param) / length_param
    t = max(0.0, min(1.0, t))
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    )


def _midpoint(a: Vec3, b: Vec3) -> Vec3:
    return (
        (a[0] + b[0]) * 0.5,
        (a[1] + b[1]) * 0.5,
        (a[2] + b[2]) * 0.5,
    )


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]
