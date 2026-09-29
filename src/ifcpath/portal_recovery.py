from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass

from .model import InavModel, NavCell, Portal, SemanticTransition, Space, Vec3
from .surface_nav import _closest_point_on_triangle


@dataclass(frozen=True, slots=True)
class PortalRecoveryStats:
    considered: int = 0
    repaired: int = 0
    unresolved: int = 0


def qualify_surface_portal_sides(
    model: InavModel,
    *,
    max_distance_m: float = 2.5,
) -> PortalRecoveryStats:
    """Repair door-space semantics using the authoritative walkable surface.

    IFC space-boundary relationships are preferred evidence, but real files can
    contain duplicate/stale boundary records around an opening. A door threshold
    physically belongs to the surfaced spaces whose floor triangles are closest
    to the door position. This pass therefore re-qualifies only ``door`` portals
    against actual open NavCells and repairs a portal when its authored sides do
    not match the geometrically closest qualified spaces.

    Exterior exits deliberately keep one internal side. Internal doors require
    two surfaced sides. Existing portal-backed semantic transitions are updated
    in-place so old INAV files cannot retain stale transition endpoints after a
    portal repair.
    """
    if not model.cells or not model.portals:
        return PortalRecoveryStats()

    spaces = {space.id: space for space in model.spaces}
    cells_by_space: dict[str, list[NavCell]] = defaultdict(list)
    for cell in model.cells:
        if cell.terrain == "open" and cell.space_id:
            cells_by_space[cell.space_id].append(cell)

    considered = repaired = unresolved = 0
    for portal in model.portals:
        if portal.kind != "door":
            continue
        considered += 1
        desired_count = 1 if portal.is_exit else 2
        candidates = _rank_spaces(portal, spaces, cells_by_space, max_distance_m)
        if len(candidates) < desired_count:
            unresolved += 1
            continue

        selected = [space for _, space in candidates[:desired_count]]
        selected_ids = [space.id for space in selected]
        current_ids = [
            value for value in (portal.from_space_id, portal.to_space_id) if value
        ]
        if set(current_ids) == set(selected_ids) and len(current_ids) == desired_count:
            continue

        # Preserve an already-correct from-side where possible to avoid needless
        # direction churn in serialized files. Door transitions are bidirectional.
        if portal.from_space_id in selected_ids:
            first = portal.from_space_id
            remainder = [item for item in selected_ids if item != first]
            selected_ids = [first, *remainder]
        else:
            selected_ids.sort()

        portal.from_space_id = selected_ids[0]
        portal.to_space_id = selected_ids[1] if len(selected_ids) > 1 else None
        if not portal.level_id:
            portal.level_id = spaces[selected_ids[0]].level_id
        _repair_portal_transitions(model, portal, spaces)
        repaired += 1

    if repaired:
        model.metadata["surface_portal_side_repairs"] = int(
            model.metadata.get("surface_portal_side_repairs", 0)
        ) + repaired
    return PortalRecoveryStats(considered, repaired, unresolved)


def _rank_spaces(
    portal: Portal,
    spaces: dict[str, Space],
    cells_by_space: dict[str, list[NavCell]],
    max_distance_m: float,
) -> list[tuple[float, Space]]:
    ranked: list[tuple[float, Space]] = []
    authored = {value for value in (portal.from_space_id, portal.to_space_id) if value}
    for space_id, cells in cells_by_space.items():
        space = spaces.get(space_id)
        if space is None:
            continue
        if portal.level_id and space.level_id and portal.level_id != space.level_id:
            continue
        distance_m = _space_surface_distance(portal.position_m, cells)
        if distance_m > max_distance_m:
            continue
        # Geometry is the primary key. IFC-authored membership only breaks very
        # small ties, so a badly bound but much farther space cannot win.
        tie_break = 0 if space_id in authored else 1
        ranked.append((distance_m + tie_break * 1e-7, space))
    ranked.sort(key=lambda item: (item[0], item[1].id))
    return ranked


def _space_surface_distance(point: Vec3, cells: list[NavCell]) -> float:
    best = math.inf
    for cell in cells:
        a, b, c = cell.vertices_m
        projected = _closest_point_on_triangle(point, a, b, c)
        best = min(best, math.dist(point, projected))
    return best


def _repair_portal_transitions(
    model: InavModel,
    portal: Portal,
    spaces: dict[str, Space],
) -> None:
    matching = [
        transition for transition in model.transitions if transition.portal_id == portal.id
    ]
    for transition in matching:
        transition.from_space_id = portal.from_space_id or transition.from_space_id
        transition.to_space_id = portal.to_space_id
        from_space = spaces.get(transition.from_space_id)
        to_space = spaces.get(transition.to_space_id) if transition.to_space_id else None
        transition.from_level_id = from_space.level_id if from_space else portal.level_id
        transition.to_level_id = to_space.level_id if to_space else None


def portal_surface_distances(model: InavModel, portal_id: str) -> list[tuple[float, str]]:
    """Return deterministic diagnostics for one portal's nearby surfaced spaces."""
    portal = next((item for item in model.portals if item.id == portal_id), None)
    if portal is None:
        return []
    spaces = {space.id: space for space in model.spaces}
    cells_by_space: dict[str, list[NavCell]] = defaultdict(list)
    for cell in model.cells:
        if cell.terrain == "open" and cell.space_id:
            cells_by_space[cell.space_id].append(cell)
    ranked = _rank_spaces(portal, spaces, cells_by_space, math.inf)
    return [(distance, space.id) for distance, space in ranked]
