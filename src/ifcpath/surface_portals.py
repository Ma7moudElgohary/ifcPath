from __future__ import annotations

import math
from collections import deque

from .model import InavModel, NavCell, Vec3
from .surface_nav import (
    _closest_point_on_triangle,
    connect_cells_through_portal,
    surface_components,
)


def bind_semantic_surface_portals(
    model: InavModel,
    *,
    max_distance_m: float = 2.5,
) -> int:
    """Bind authored door semantics into the authoritative surface graph.

    The IFC loader resolves each door to ``Portal`` metadata and semantic room
    sides. This idempotent post-pass authorizes exactly one metric crossing for
    each internal door and records its semantic portal ID on the cell adjacency.
    It also upgrades older INAV payloads when they are loaded.

    A sampled physical surface can contain a tiny isolated door-threshold island
    beside the substantial room floor. Pure nearest-cell selection would bind the
    door to that island and leave the usable room component disconnected. For each
    side we therefore consider disconnected same-space components within a small,
    door-width-scaled band of the nearest component and prefer the largest one.
    A genuinely separate landing remains selectable when the main component is
    farther away than that local band.
    """
    if not model.cells or not model.portals:
        model.metadata["surface_door_portals"] = 0
        return 0

    bound_ids: set[str] = {
        portal_id
        for cell in model.cells
        for portal_id in cell.portal_ids.values()
        if portal_id
    }
    added = 0
    for portal in model.portals:
        if (
            portal.kind != "door"
            or not portal.from_space_id
            or not portal.to_space_id
            or portal.id in bound_ids
        ):
            continue

        component_band_m = max(
            0.25,
            min(0.75, float(portal.width_m or 0.90) * 0.50),
        )
        left = _preferred_space_component_cells(
            model.cells,
            portal.from_space_id,
            portal.position_m,
            portal.level_id,
            component_band_m,
        )
        right = _preferred_space_component_cells(
            model.cells,
            portal.to_space_id,
            portal.position_m,
            portal.level_id,
            component_band_m,
        )
        candidate_cells = left + [cell for cell in right if cell.id not in {item.id for item in left}]
        if not candidate_cells:
            continue

        if connect_cells_through_portal(
            candidate_cells,
            point=portal.position_m,
            from_space_id=portal.from_space_id,
            to_space_id=portal.to_space_id,
            portal_id=portal.id,
            width_m=portal.width_m,
            level_id=portal.level_id,
            max_distance_m=max_distance_m,
        ):
            added += 1
            bound_ids.add(portal.id)

    model.metadata["surface_door_portals"] = len(bound_ids)
    model.metadata["surface_components"] = len(surface_components(model.cells))
    return added


def _preferred_space_component_cells(
    cells: list[NavCell],
    space_id: str,
    point: Vec3,
    level_id: str | None,
    preference_band_m: float,
) -> list[NavCell]:
    exact_level = [
        cell
        for cell in cells
        if cell.space_id == space_id
        and (level_id is None or cell.level_id is None or cell.level_id == level_id)
    ]
    candidates = exact_level or [cell for cell in cells if cell.space_id == space_id]
    if not candidates:
        return []

    # Doors should bind to floor/landing support where possible, not directly to
    # a stair tread that happens to share the same semantic space label.
    open_candidates = [cell for cell in candidates if cell.terrain == "open"]
    if open_candidates:
        candidates = open_candidates

    by_id = {cell.id: cell for cell in candidates}
    remaining = set(by_id)
    components: list[list[NavCell]] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        ids = {start}
        queue = deque([start])
        while queue:
            current_id = queue.popleft()
            for neighbor_id in by_id[current_id].neighbor_ids:
                if neighbor_id not in remaining:
                    continue
                remaining.remove(neighbor_id)
                ids.add(neighbor_id)
                queue.append(neighbor_id)
        components.append([by_id[cell_id] for cell_id in sorted(ids)])

    ranked = []
    for component in components:
        distance = min(
            math.dist(
                point,
                _closest_point_on_triangle(point, *cell.vertices_m),
            )
            for cell in component
        )
        area = sum(_triangle_area(*cell.vertices_m) for cell in component)
        ranked.append((distance, area, len(component), component))

    nearest_distance = min(item[0] for item in ranked)
    eligible = [
        item
        for item in ranked
        if item[0] <= nearest_distance + max(0.0, preference_band_m) + 1e-9
    ]
    # Within the local doorway neighbourhood, physical support area is a better
    # indicator of the usable room floor than a tiny distance advantage caused by
    # grid phase. Distance remains the deterministic tiebreaker.
    _, _, _, selected = max(
        eligible,
        key=lambda item: (item[1], item[2], -item[0], item[3][0].id),
    )
    return selected


def _triangle_area(a: Vec3, b: Vec3, c: Vec3) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))
