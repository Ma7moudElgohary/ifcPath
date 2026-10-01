from __future__ import annotations

from pathlib import Path

import ifcopenshell

from .ifc_loader import (
    BuildOptions,
    _bbox,
    _bbox_from_vertices,
    _boundary_space_info,
    _contained_levels,
    _distance_to_box,
    _door_is_exit,
    _levels,
    _mesh,
    _safe_ifc_attr,
    _space_is_external,
    _spatial_level_id,
    build_from_ifc,
)
from .model import InavModel, Portal, Space


def build_semantic_skeleton_from_ifc(
    path: str | Path,
    options: BuildOptions | None = None,
) -> InavModel:
    """Extract semantics needed by physical reconstruction without a legacy graph.

    The local physical builder replaces legacy floor/stair cells immediately, so
    constructing thousands of sampled/CDT nodes, walk edges, wall-obstacle checks
    and legacy vertical cells first is wasted work. This importer keeps only the
    IFC information required by the physical pipeline: levels, spaces and authored
    door semantics.

    Elevators are deliberately conservative for now. Their IFC connector builder
    still depends on the compatibility node graph, so an IFC containing an
    elevator falls back to the full importer rather than silently dropping vertical
    transport semantics.
    """
    options = options or BuildOptions()
    model = ifcopenshell.open(str(path))

    if _has_elevator_transport(model):
        out = build_from_ifc(path, options)
        out.metadata["semantic_import_mode"] = "full-legacy-elevator"
        return out

    out = InavModel(
        metadata={
            "source_ifc": str(path),
            "semantic_import_mode": "semantics-only",
            "generator": "ifcpath",
        }
    )

    levels = _levels(model)
    out.levels.extend(levels)
    level_by_entity = {entity_id: level_id for entity_id, level_id in _contained_levels(model, levels)}
    level_by_guid = {level.id.removeprefix("level:"): level.id for level in levels}

    space_boxes: list[tuple[Space, tuple[float, float, float, float, float, float]]] = []
    space_by_entity_id: dict[int, Space] = {}
    for entity in model.by_type("IfcSpace"):
        mesh = _mesh(entity)
        if mesh is None or not mesh[0]:
            continue
        bbox = _bbox_from_vertices(mesh[0])
        level_id = _spatial_level_id(entity, level_by_entity, level_by_guid)
        centroid = (
            (bbox[0] + bbox[3]) * 0.5,
            (bbox[1] + bbox[4]) * 0.5,
            (bbox[2] + bbox[5]) * 0.5,
        )
        guid = str(_safe_ifc_attr(entity, "GlobalId", entity.id()))
        space = Space(
            id=f"space:{guid}",
            name=_safe_ifc_attr(entity, "Name") or _safe_ifc_attr(entity, "LongName") or guid,
            level_id=level_id,
            centroid_m=centroid,
            ifc_guid=guid,
            is_external=_space_is_external(entity),
        )
        out.spaces.append(space)
        space_boxes.append((space, bbox))
        space_by_entity_id[entity.id()] = space

    boundary_spaces, external_boundary_elements = _boundary_space_info(model, space_by_entity_id)
    explicit_portal_count = 0
    inferred_portal_count = 0

    for door in model.by_type("IfcDoor"):
        bbox = _bbox(door)
        if bbox is None:
            continue
        position = (
            (bbox[0] + bbox[3]) * 0.5,
            (bbox[1] + bbox[4]) * 0.5,
            bbox[2],
        )

        related_element_ids = {door.id()}
        for fills_rel in _safe_ifc_attr(door, "FillsVoids", ()) or ():
            opening = _safe_ifc_attr(fills_rel, "RelatingOpeningElement")
            if opening is not None:
                related_element_ids.add(opening.id())

        explicit_spaces: list[Space] = []
        seen_space_ids: set[str] = set()
        for element_id in related_element_ids:
            for space in boundary_spaces.get(element_id, ()):
                if space.id in seen_space_ids:
                    continue
                explicit_spaces.append(space)
                seen_space_ids.add(space.id)

        if explicit_spaces:
            connected_spaces = explicit_spaces[:2]
            explicit_portal_count += 1
        else:
            candidates = sorted(
                ((_distance_to_box(position, space_box), space) for space, space_box in space_boxes),
                key=lambda item: item[0],
            )
            connected_spaces = [space for distance, space in candidates if distance <= 1.5][:2]
            inferred_portal_count += 1

        from_space = connected_spaces[0].id if connected_spaces else None
        to_space = connected_spaces[1].id if len(connected_spaces) > 1 else None
        door_guid = str(_safe_ifc_attr(door, "GlobalId", door.id()))
        level_id = level_by_entity.get(door.id()) or (
            connected_spaces[0].level_id if connected_spaces else None
        )

        out.portals.append(
            Portal(
                id=f"door:{door_guid}",
                kind="door",
                position_m=position,
                from_space_id=from_space,
                to_space_id=to_space,
                level_id=level_id,
                width_m=float(_safe_ifc_attr(door, "OverallWidth", 0.0) or 0.0) or None,
                ifc_guid=door_guid,
                is_exit=_door_is_exit(
                    door,
                    related_element_ids,
                    external_boundary_elements,
                    len(connected_spaces),
                ),
            )
        )

    out.metadata.update(
        {
            "node_count": 0,
            "edge_count": 0,
            "cell_count": 0,
            "space_count": len(out.spaces),
            "portal_count": len(out.portals),
            "explicit_space_boundary_portals": explicit_portal_count,
            "geometry_inferred_portals": inferred_portal_count,
            "external_space_count": sum(space.is_external for space in out.spaces),
        }
    )
    return out


def _has_elevator_transport(model) -> bool:
    try:
        transports = model.by_type("IfcTransportElement")
    except Exception:
        return False
    for entity in transports:
        predefined = str(_safe_ifc_attr(entity, "PredefinedType", "") or "").upper()
        if predefined == "ELEVATOR":
            return True
    return False
