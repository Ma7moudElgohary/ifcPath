from __future__ import annotations

from .model import InavModel
from .surface_nav import connect_cells_through_portal, surface_components


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
        if connect_cells_through_portal(
            model.cells,
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
