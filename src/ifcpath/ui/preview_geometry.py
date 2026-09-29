from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import ifcopenshell
import ifcopenshell.util.element

from ..ifc_loader import _contained_levels, _levels, _mesh, _spatial_level_id
from ..model import Vec3


@dataclass(slots=True)
class PreviewTriangle:
    vertices_m: tuple[Vec3, Vec3, Vec3]
    category: str
    level_id: str | None = None
    ifc_guid: str | None = None


@dataclass(slots=True)
class PreviewElement:
    """Selection/inspection metadata for one IFC product in the preview."""

    guid: str
    express_id: int
    ifc_class: str
    category: str
    level_id: str | None = None
    name: str | None = None
    description: str | None = None
    object_type: str | None = None
    predefined_type: str | None = None
    tag: str | None = None
    attributes: dict[str, Any] = field(default_factory=dict)
    property_sets: dict[str, dict[str, Any]] = field(default_factory=dict)


@dataclass(slots=True)
class PreviewGeometry:
    triangles: list[PreviewTriangle] = field(default_factory=list)
    elements: dict[str, PreviewElement] = field(default_factory=dict)
    truncated: bool = False

    def element_for_guid(self, guid: str | None) -> PreviewElement | None:
        if not guid:
            return None
        return self.elements.get(guid)


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

_INFO_SKIP = {
    "id",
    "type",
    "GlobalId",
    "Name",
    "Description",
    "ObjectType",
    "PredefinedType",
    "Tag",
}


def extract_preview_geometry(
    path: str | Path,
    *,
    max_triangles: int = 18_000,
) -> PreviewGeometry:
    """Extract bounded BIM geometry plus IFC identity/property metadata.

    Navigation still comes entirely from INAV. Geometry is capped by the caller,
    but every included IFC product keeps its GlobalId and lightweight metadata so
    the viewport can select/highlight/inspect objects without reopening the IFC.
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
            if guid:
                result.elements[str(guid)] = _element_metadata(
                    entity,
                    category=category,
                    level_id=level_id,
                )

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
                        ifc_guid=str(guid) if guid else None,
                    )
                )

    return result


def _element_metadata(entity, *, category: str, level_id: str | None) -> PreviewElement:
    guid = str(getattr(entity, "GlobalId", ""))
    try:
        predefined_type = ifcopenshell.util.element.get_predefined_type(entity)
    except Exception:
        predefined_type = getattr(entity, "PredefinedType", None)

    try:
        info = entity.get_info(scalar_only=True)
    except Exception:
        info = {}
    attributes = {
        str(key): _safe_value(value)
        for key, value in info.items()
        if key not in _INFO_SKIP and value not in (None, "")
    }

    try:
        raw_psets = ifcopenshell.util.element.get_psets(entity)
    except Exception:
        raw_psets = {}
    property_sets: dict[str, dict[str, Any]] = {}
    for pset_name, values in raw_psets.items():
        if not isinstance(values, dict):
            continue
        clean = {
            str(key): _safe_value(value)
            for key, value in values.items()
            if key != "id" and value not in (None, "")
        }
        if clean:
            property_sets[str(pset_name)] = clean

    return PreviewElement(
        guid=guid,
        express_id=int(entity.id()),
        ifc_class=str(entity.is_a()),
        category=category,
        level_id=level_id,
        name=_optional_text(getattr(entity, "Name", None)),
        description=_optional_text(getattr(entity, "Description", None)),
        object_type=_optional_text(getattr(entity, "ObjectType", None)),
        predefined_type=_optional_text(predefined_type),
        tag=_optional_text(getattr(entity, "Tag", None)),
        attributes=attributes,
        property_sets=property_sets,
    )


def _optional_text(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _safe_value(value: Any, *, depth: int = 0) -> Any:
    """Convert IFC property values to compact UI-safe Python values."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if depth >= 3:
        return str(value)
    if isinstance(value, dict):
        return {
            str(key): _safe_value(item, depth=depth + 1)
            for key, item in list(value.items())[:64]
        }
    if isinstance(value, (list, tuple, set)):
        return [_safe_value(item, depth=depth + 1) for item in list(value)[:64]]
    wrapped = getattr(value, "wrappedValue", None)
    if wrapped is not None:
        return _safe_value(wrapped, depth=depth + 1)
    return str(value)
