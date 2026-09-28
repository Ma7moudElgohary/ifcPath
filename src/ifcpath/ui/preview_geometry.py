from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import ifcopenshell

from ..ifc_loader import _contained_levels, _levels, _mesh, _spatial_level_id
from ..model import Vec3


@dataclass(slots=True)
class PreviewTriangle:
    vertices_m: tuple[Vec3, Vec3, Vec3]
    category: str
    level_id: str | None = None
    ifc_guid: str | None = None


@dataclass(slots=True)
class PreviewGeometry:
    triangles: list[PreviewTriangle] = field(default_factory=list)
    truncated: bool = False


_PREVIEW_CLASSES: tuple[tuple[str, str], ...] = (
    ("IfcWall", "wall"),
    ("IfcSlab", "slab"),
    ("IfcColumn", "column"),
    ("IfcStair", "stair"),
    ("IfcStairFlight", "stair"),
    ("IfcRamp", "ramp"),
    ("IfcRampFlight", "ramp"),
    ("IfcDoor", "door"),
)


def extract_preview_geometry(
    path: str | Path,
    *,
    max_triangles: int = 18_000,
) -> PreviewGeometry:
    """Extract a bounded BIM triangle soup for desktop visualization only.

    Navigation still comes entirely from INAV. This preview intentionally keeps a
    hard triangle cap so a large IFC cannot turn the desktop Builder into a full
    BIM renderer or increase the portable navigation payload.
    """
    model = ifcopenshell.open(str(path))
    levels = _levels(model)
    level_by_entity = {entity_id: level_id for entity_id, level_id in _contained_levels(model, levels)}
    level_by_guid = {level.id.removeprefix("level:"): level.id for level in levels}

    result = PreviewGeometry()
    seen_entities: set[int] = set()

    for class_name, category in _PREVIEW_CLASSES:
        try:
            entities = model.by_type(class_name)
        except Exception:
            continue

        for entity in entities:
            entity_id = entity.id()
            if entity_id in seen_entities:
                continue
            seen_entities.add(entity_id)

            mesh = _mesh(entity)
            if mesh is None:
                continue
            vertices, triangles = mesh
            level_id = _spatial_level_id(entity, level_by_entity, level_by_guid)
            guid = getattr(entity, "GlobalId", None)

            for a, b, c in triangles:
                if len(result.triangles) >= max_triangles:
                    result.truncated = True
                    return result
                if not (
                    0 <= a < len(vertices)
                    and 0 <= b < len(vertices)
                    and 0 <= c < len(vertices)
                ):
                    continue
                result.triangles.append(
                    PreviewTriangle(
                        vertices_m=(vertices[a], vertices[b], vertices[c]),
                        category=category,
                        level_id=level_id,
                        ifc_guid=guid,
                    )
                )

    return result
