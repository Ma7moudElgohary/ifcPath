from __future__ import annotations

import hashlib
import math
from collections import Counter
from dataclasses import dataclass

from .model import InavModel, NavCell, Portal, Vec3
from .obstacles import WallObstacle, edge_crosses_obstacle
from .surface_nav import _closest_segment_points, cell_centroid


@dataclass(frozen=True, slots=True)
class OpenBoundaryStats:
    candidates: int = 0
    connected: int = 0
    wall_rejected: int = 0
    existing_portal_pairs: int = 0


@dataclass(frozen=True, slots=True)
class _Candidate:
    a: NavCell
    b: NavCell
    a0: Vec3
    a1: Vec3
    b0: Vec3
    b1: Vec3
    pa: Vec3
    pb: Vec3
    gap_m: float
    portal: tuple[Vec3, Vec3]

    @property
    def width_m(self) -> float:
        return math.dist(*self.portal)


def connect_open_space_boundaries(
    model: InavModel,
    wall_obstacles: list[WallObstacle],
    *,
    max_gap_m: float = 0.20,
    max_vertical_gap_m: float = 0.10,
    probe_depth_m: float = 0.20,
) -> OpenBoundaryStats:
    """Connect adjacent IFC spaces only across wall-clear floor boundaries.

    IFC models commonly split one open-plan walkable area into several
    ``IfcSpace`` objects without placing a door between them. The normal surface
    builder deliberately rejects every raw cross-space edge so a coincident wall
    can never become a route leak. This recovery pass adds back only boundaries
    that satisfy all of the following:

    * both cells are open floor cells on the same level;
    * their *space-boundary* edges are within a small pedestrian-scale gap;
    * no already-authored portal connects the semantic space pair;
    * a short probe from inside one triangle to inside the other does not cross a
      wall obstacle at walking height.

    A synthetic ``open_boundary`` portal is persisted so the crossing remains
    explicit, inspectable and dynamically distinguishable from a door.
    """
    open_cells = [
        cell for cell in model.cells
        if cell.terrain == "open" and cell.space_id
    ]
    if len(open_cells) < 2:
        return OpenBoundaryStats()

    existing_pairs = {
        frozenset((portal.from_space_id, portal.to_space_id))
        for portal in model.portals
        if portal.from_space_id and portal.to_space_id
    }
    boundaries = _space_boundary_edges(open_cells)
    grouped: dict[frozenset[str], list[_Candidate]] = {}
    candidate_count = 0
    wall_rejected = 0

    for index, (a, a0, a1) in enumerate(boundaries):
        for b, b0, b1 in boundaries[index + 1 :]:
            if a.space_id == b.space_id:
                continue
            if a.level_id and b.level_id and a.level_id != b.level_id:
                continue
            pair = frozenset((a.space_id or "", b.space_id or ""))
            if len(pair) != 2 or pair in existing_pairs:
                continue

            pa, pb = _closest_segment_points(a0, a1, b0, b1)
            gap = math.dist(pa, pb)
            if gap > max_gap_m or abs(pa[2] - pb[2]) > max_vertical_gap_m:
                continue
            candidate_count += 1

            probe_a = _toward_inside(pa, cell_centroid(a), probe_depth_m)
            probe_b = _toward_inside(pb, cell_centroid(b), probe_depth_m)
            if edge_crosses_obstacle(probe_a, probe_b, wall_obstacles):
                wall_rejected += 1
                continue

            portal_segment = _overlap_portal(a0, a1, b0, b1, pa, pb)
            candidate = _Candidate(a, b, a0, a1, b0, b1, pa, pb, gap, portal_segment)
            grouped.setdefault(pair, []).append(candidate)

    connected = 0
    for pair, candidates in sorted(grouped.items(), key=lambda item: sorted(item[0])):
        # Prefer the widest clear shared boundary; tie-break by smallest gap and
        # stable cell IDs for deterministic exports.
        chosen = min(
            candidates,
            key=lambda item: (-item.width_m, item.gap_m, item.a.id, item.b.id),
        )
        sides = sorted(pair)
        portal_id = _portal_id(chosen.a.level_id or chosen.b.level_id, sides[0], sides[1])
        if any(portal.id == portal_id for portal in model.portals):
            continue

        _connect(chosen.a, chosen.b)
        chosen.a.portals[chosen.b.id] = chosen.portal
        chosen.b.portals[chosen.a.id] = chosen.portal
        chosen.a.portal_ids[chosen.b.id] = portal_id
        chosen.b.portal_ids[chosen.a.id] = portal_id

        p, q = chosen.portal
        center = tuple((p[index] + q[index]) * 0.5 for index in range(3))
        model.portals.append(Portal(
            id=portal_id,
            kind="open_boundary",
            position_m=center,
            from_space_id=sides[0],
            to_space_id=sides[1],
            level_id=chosen.a.level_id or chosen.b.level_id,
            width_m=max(0.0, chosen.width_m),
            is_exit=False,
        ))
        existing_pairs.add(pair)
        connected += 1

    return OpenBoundaryStats(
        candidates=candidate_count,
        connected=connected,
        wall_rejected=wall_rejected,
        existing_portal_pairs=len(existing_pairs),
    )


def _space_boundary_edges(cells: list[NavCell]) -> list[tuple[NavCell, Vec3, Vec3]]:
    counts: Counter[tuple[str, tuple, tuple]] = Counter()
    records: dict[tuple[str, tuple, tuple], tuple[NavCell, Vec3, Vec3]] = {}
    scale = 1e5

    def point_key(point: Vec3) -> tuple[int, int, int]:
        return tuple(round(value * scale) for value in point)

    for cell in cells:
        vertices = cell.vertices_m
        for a, b in ((vertices[0], vertices[1]), (vertices[1], vertices[2]), (vertices[2], vertices[0])):
            qa, qb = point_key(a), point_key(b)
            if qb < qa:
                qa, qb = qb, qa
                a, b = b, a
            key = (cell.space_id or "", qa, qb)
            counts[key] += 1
            records.setdefault(key, (cell, a, b))

    return [records[key] for key, count in counts.items() if count == 1]


def _toward_inside(boundary: Vec3, centroid: Vec3, depth_m: float) -> Vec3:
    delta = tuple(centroid[index] - boundary[index] for index in range(3))
    length = math.sqrt(sum(value * value for value in delta))
    if length <= 1e-9:
        return boundary
    distance = min(max(depth_m, 0.0), length)
    return tuple(boundary[index] + delta[index] * distance / length for index in range(3))


def _overlap_portal(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
    pa: Vec3,
    pb: Vec3,
) -> tuple[Vec3, Vec3]:
    """Return the shared collinear span when possible, else a point crossing."""
    dx, dy = a1[0] - a0[0], a1[1] - a0[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        center = tuple((pa[index] + pb[index]) * 0.5 for index in range(3))
        return center, center
    ux, uy = dx / length, dy / length
    b_values = [
        (point[0] - a0[0]) * ux + (point[1] - a0[1]) * uy
        for point in (b0, b1)
    ]
    lo = max(0.0, min(b_values))
    hi = min(length, max(b_values))
    if hi - lo < 0.10:
        center = tuple((pa[index] + pb[index]) * 0.5 for index in range(3))
        return center, center
    z = (pa[2] + pb[2]) * 0.5
    return (
        (a0[0] + ux * lo, a0[1] + uy * lo, z),
        (a0[0] + ux * hi, a0[1] + uy * hi, z),
    )


def _connect(a: NavCell, b: NavCell) -> None:
    if b.id not in a.neighbor_ids:
        a.neighbor_ids.append(b.id)
    if a.id not in b.neighbor_ids:
        b.neighbor_ids.append(a.id)


def _portal_id(level_id: str | None, a: str, b: str) -> str:
    payload = f"{level_id or ''}|{a}|{b}".encode("utf-8")
    digest = hashlib.sha1(payload).hexdigest()[:16]
    return f"open:{digest}"
