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


def prune_tiny_space_fragments(
    cells: list[NavCell],
    model: InavModel,
    *,
    max_cells: int = 16,
    max_area_m2: float = 0.20,
    max_relative_area: float = 0.10,
    portal_protection_m: float = 0.65,
) -> FragmentPruneStats:
    """Remove tiny detached open-surface islands created by grid sampling.

    This is deliberately much more conservative than generic component pruning.
    For each semantic space the largest open-surface component is always retained.
    An additional component is removed only when it is simultaneously:

    * small in cell count;
    * small in physical triangle area;
    * small relative to the space's main component;
    * not attached to stair/ramp/escalator terrain; and
    * not close to an authored semantic portal serving that space.

    Consequently a real split-level room, mezzanine or second substantial floor
    remains visible to readiness validation. The pass targets only tiny sampling
    islands such as the 4--12-cell fragments exposed by the public Duplex model.
    """
    if not cells:
        return FragmentPruneStats()

    by_id = {cell.id: cell for cell in cells}
    open_ids_by_space: dict[str, set[str]] = defaultdict(set)
    for cell in cells:
        if cell.terrain == "open" and cell.space_id:
            open_ids_by_space[cell.space_id].add(cell.id)

    portal_points_by_space: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    for portal in model.portals:
        for space_id in {portal.from_space_id, portal.to_space_id}:
            if space_id:
                portal_points_by_space[space_id].append(portal.position_m)

    remove_ids: set[str] = set()
    removed_components = 0

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
            if area > max_area_m2 + 1e-9:
                continue
            if area > largest_area * max_relative_area + 1e-9:
                continue
            if _touches_vertical(component, by_id):
                continue
            if _near_semantic_portal(
                component,
                by_id,
                portal_points_by_space.get(space_id, ()),
                portal_protection_m,
            ):
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
