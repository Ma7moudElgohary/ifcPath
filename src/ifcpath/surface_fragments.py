from __future__ import annotations

import math
from collections import defaultdict, deque
from dataclasses import dataclass

from .model import InavModel, NavCell
from .surface_nav import cell_centroid


_VERTICAL_TERRAINS = {"stair", "ramp", "escalator"}


@dataclass(frozen=True, slots=True)
class FragmentPruneStats:
    removed_cells: int = 0
    removed_components: int = 0
    protected_vertical_components: int = 0
    protected_bound_portal_components: int = 0
    protected_portal_proximity_components: int = 0


def prune_tiny_space_fragments(
    cells: list[NavCell],
    model: InavModel,
    *,
    max_cells: int = 16,
    max_area_m2: float = 0.20,
    max_relative_area: float = 0.10,
    sampling_cell_size_m: float | None = None,
    portal_protection_m: float = 0.65,
    protect_portal_proximity: bool = True,
    protect_bound_portals: bool = True,
) -> FragmentPruneStats:
    """Remove tiny detached open-surface islands created by grid sampling.

    This is deliberately much more conservative than generic component pruning.
    For each semantic space the largest open-surface component is always retained.
    An additional component is removed only when it is simultaneously:

    * small in cell count;
    * small in physical triangle area;
    * small relative to the space's main component;
    * not attached to stair/ramp/escalator terrain;
    * not part of an already-bound semantic crossing; and
    * when requested, not close to an authored semantic portal serving the space.

    The absolute area limit is resolution-aware when ``sampling_cell_size_m`` is
    supplied. A regular square sampling cell contributes two triangles, each with
    nominal area ``0.5 * cell_size^2``. Therefore a component that already passes
    ``max_cells`` should not escape pruning merely because a coarser but still
    supported detector resolution makes those few triangles exceed a fixed 0.20
    m2 threshold. The legacy fixed threshold remains the lower bound, while the
    relative-area, portal and vertical protections still prevent real mezzanines,
    landings or semantic thresholds from being hidden.

    The two portal protections intentionally serve different phases. Before
    semantic finalisation, proximity is conservative evidence that a small patch
    might be a real door threshold. After door binding has run, ``portal_ids`` are
    stronger evidence: a nearby but unused sliver may be removed, while the cells
    that actually carry a door/open-boundary crossing remain protected.
    """
    if not cells:
        return FragmentPruneStats()

    effective_max_area_m2 = max(0.0, float(max_area_m2))
    if sampling_cell_size_m is not None and sampling_cell_size_m > 0.0:
        # Allow a small quantisation margin for sloped support triangles and BRep
        # hit variation while keeping the independent max_cells guard authoritative.
        nominal_max_area = (
            0.5
            * max(0, int(max_cells))
            * float(sampling_cell_size_m) ** 2
            * 1.05
        )
        effective_max_area_m2 = max(effective_max_area_m2, nominal_max_area)

    by_id = {cell.id: cell for cell in cells}
    open_ids_by_space: dict[str, set[str]] = defaultdict(set)
    for cell in cells:
        if cell.terrain == "open" and cell.space_id:
            open_ids_by_space[cell.space_id].add(cell.id)

    portal_points_by_space: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    if protect_portal_proximity:
        for portal in model.portals:
            for space_id in {portal.from_space_id, portal.to_space_id}:
                if space_id:
                    portal_points_by_space[space_id].append(portal.position_m)

    remove_ids: set[str] = set()
    removed_components = 0
    protected_vertical_components = 0
    protected_bound_portal_components = 0
    protected_portal_proximity_components = 0

    for space_id, owned_ids in open_ids_by_space.items():
        components = _local_components(owned_ids, by_id)
        if len(components) <= 1:
            continue

        ranked = sorted(
            ((_component_area(component, by_id), component) for component in components),
            key=lambda item: (item[0], len(item[1]), min(item[1])),
            reverse=True,
        )
        largest_area = ranked[0][0]
        if largest_area <= 1e-9:
            continue

        for area, component in ranked[1:]:
            if len(component) > max_cells:
                continue
            if area > effective_max_area_m2 + 1e-9:
                continue
            if area > largest_area * max_relative_area + 1e-9:
                continue
            if _touches_vertical(component, by_id):
                protected_vertical_components += 1
                continue
            if protect_bound_portals and _has_bound_semantic_crossing(component, by_id):
                protected_bound_portal_components += 1
                continue
            if (
                protect_portal_proximity
                and _near_semantic_portal(
                    component,
                    by_id,
                    portal_points_by_space.get(space_id, ()),
                    portal_protection_m,
                )
            ):
                protected_portal_proximity_components += 1
                continue
            remove_ids.update(component)
            removed_components += 1

    if remove_ids:
        cells[:] = [cell for cell in cells if cell.id not in remove_ids]
        # Keep the portable topology self-contained immediately; callers should
        # not have to remember to repair references after a geometry-pruning pass.
        for cell in cells:
            cell.neighbor_ids[:] = [
                neighbor_id
                for neighbor_id in cell.neighbor_ids
                if neighbor_id not in remove_ids
            ]
            for neighbor_id in list(cell.portals):
                if neighbor_id in remove_ids:
                    cell.portals.pop(neighbor_id, None)
            for neighbor_id in list(cell.portal_ids):
                if neighbor_id in remove_ids:
                    cell.portal_ids.pop(neighbor_id, None)

    return FragmentPruneStats(
        removed_cells=len(remove_ids),
        removed_components=removed_components,
        protected_vertical_components=protected_vertical_components,
        protected_bound_portal_components=protected_bound_portal_components,
        protected_portal_proximity_components=protected_portal_proximity_components,
    )


def _local_components(owned_ids: set[str], by_id: dict[str, NavCell]) -> list[set[str]]:
    remaining = set(owned_ids)
    result: list[set[str]] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        component = {start}
        queue = deque([start])
        while queue:
            current_id = queue.popleft()
            for neighbor_id in by_id[current_id].neighbor_ids:
                if neighbor_id not in remaining:
                    continue
                remaining.remove(neighbor_id)
                component.add(neighbor_id)
                queue.append(neighbor_id)
        result.append(component)
    return result


def _component_area(component: set[str], by_id: dict[str, NavCell]) -> float:
    return sum(_triangle_area(*by_id[cell_id].vertices_m) for cell_id in component)


def _touches_vertical(component: set[str], by_id: dict[str, NavCell]) -> bool:
    for cell_id in component:
        for neighbor_id in by_id[cell_id].neighbor_ids:
            if neighbor_id in component:
                continue
            neighbor = by_id.get(neighbor_id)
            if neighbor is not None and neighbor.terrain in _VERTICAL_TERRAINS:
                return True
    return False


def _has_bound_semantic_crossing(
    component: set[str],
    by_id: dict[str, NavCell],
) -> bool:
    """Return whether a component participates in an actual semantic crossing.

    Bindings are normally reciprocal, but checking the neighbouring cell as well
    keeps pruning robust when reading older/partially-normalised INAV payloads.
    """
    for cell_id in component:
        cell = by_id[cell_id]
        if any(cell.portal_ids.values()):
            return True
        for neighbor_id in cell.neighbor_ids:
            neighbor = by_id.get(neighbor_id)
            if neighbor is not None and neighbor.portal_ids.get(cell_id):
                return True
    return False


def _near_semantic_portal(
    component: set[str],
    by_id: dict[str, NavCell],
    portal_points,
    threshold_m: float,
) -> bool:
    if not portal_points or threshold_m <= 0.0:
        return False
    for cell_id in component:
        center = cell_centroid(by_id[cell_id])
        if any(math.dist(center, point) <= threshold_m for point in portal_points):
            return True
    return False


def _triangle_area(a, b, c) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))
