from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.element
import ifcopenshell.util.placement
import ifcopenshell.util.unit

from .geometry import TriangleSoup
from .models import Level, Portal, Space


@dataclass
class IfcExtraction:
    walkable_geometry: TriangleSoup
    stair_geometry: TriangleSoup
    levels: list[Level]
    spaces: list[Space]
    portals: list[Portal]


def _shape_soup(entity, settings) -> TriangleSoup | None:
    try:
        shape = ifcopenshell.geom.create_shape(settings, entity)
        g = shape.geometry
        return TriangleSoup(vertices=list(g.verts), indices=list(g.faces))
    except Exception:
        return None


def _points(soup: TriangleSoup) -> list[tuple[float, float, float]]:
    return list(zip(soup.vertices[0::3], soup.vertices[1::3], soup.vertices[2::3]))


def _centroid_of_soup(soup: TriangleSoup) -> tuple[float, float, float] | None:
    pts = _points(soup)
    if not pts:
        return None
    return (
        sum(p[0] for p in pts) / len(pts),
        sum(p[1] for p in pts) / len(pts),
        sum(p[2] for p in pts) / len(pts),
    )


def _bounds_of_soup(soup: TriangleSoup) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    pts = _points(soup)
    if not pts:
        return None
    return (
        (min(p[0] for p in pts), min(p[1] for p in pts), min(p[2] for p in pts)),
        (max(p[0] for p in pts), max(p[1] for p in pts), max(p[2] for p in pts)),
    )


def _pset_bool(entity, pset_name: str, prop_name: str) -> bool | None:
    try:
        psets = ifcopenshell.util.element.get_psets(entity)
        value = (psets.get(pset_name) or {}).get(prop_name)
        if value is None:
            return None
        return bool(value)
    except Exception:
        return None


def _nearest_level_id(levels: list[Level], z: float, tolerance_m: float = 2.5) -> str | None:
    if not levels:
        return None
    level = min(levels, key=lambda x: abs(x.elevation_m - z))
    return level.id if abs(level.elevation_m - z) <= tolerance_m else level.id


def _infer_portal_spaces(portal: Portal, spaces: list[Space], expand_m: float = 0.35) -> tuple[str | None, str | None]:
    px, py, pz = portal.position_m
    candidates: list[tuple[float, Space]] = []
    for space in spaces:
        if portal.level_id and space.level_id and portal.level_id != space.level_id:
            continue
        if not space.bounds_m or not space.centroid_m:
            continue
        lo, hi = space.bounds_m
        inside_xy = (
            lo[0] - expand_m <= px <= hi[0] + expand_m
            and lo[1] - expand_m <= py <= hi[1] + expand_m
        )
        inside_z = lo[2] - 0.5 <= pz <= hi[2] + 0.5
        if inside_xy and inside_z:
            cx, cy, cz = space.centroid_m
            candidates.append(((cx-px)**2 + (cy-py)**2 + 0.1*(cz-pz)**2, space))

    candidates.sort(key=lambda x: x[0])
    ids = [s.id for _, s in candidates[:2]]
    if len(ids) >= 2:
        return ids[0], ids[1]
    if len(ids) == 1:
        return ids[0], None

    nearest: list[tuple[float, Space]] = []
    for space in spaces:
        if portal.level_id and space.level_id and portal.level_id != space.level_id:
            continue
        if not space.centroid_m:
            continue
        cx, cy, cz = space.centroid_m
        d2 = (cx-px)**2 + (cy-py)**2 + 0.1*(cz-pz)**2
        nearest.append((d2, space))
    nearest.sort(key=lambda x: x[0])
    ids = [s.id for d2, s in nearest[:2] if d2 < 100.0]
    return (ids[0] if ids else None, ids[1] if len(ids) > 1 else None)


def extract_ifc(path: str | Path) -> IfcExtraction:
    model = ifcopenshell.open(str(path))
    unit_scale = ifcopenshell.util.unit.calculate_unit_scale(model)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    levels: list[Level] = []
    for storey in model.by_type("IfcBuildingStorey"):
        elev = float(storey.Elevation or 0.0) * unit_scale
        levels.append(Level(
            id=f"level:{storey.GlobalId}", name=storey.Name,
            elevation_m=elev, ifc_guid=storey.GlobalId,
        ))
    levels.sort(key=lambda x: x.elevation_m)

    spaces: list[Space] = []
    for space in model.by_type("IfcSpace"):
        container = ifcopenshell.util.element.get_container(space)
        level_id = f"level:{container.GlobalId}" if container and container.is_a("IfcBuildingStorey") else None
        soup = _shape_soup(space, settings)
        centroid = _centroid_of_soup(soup) if soup else None
        bounds = _bounds_of_soup(soup) if soup else None
        spaces.append(Space(
            id=f"space:{space.GlobalId}", name=space.Name, level_id=level_id,
            ifc_guid=space.GlobalId, centroid_m=centroid, bounds_m=bounds,
        ))

    floor_vertices: list[float] = []
    floor_indices: list[int] = []
    for slab in model.by_type("IfcSlab"):
        predefined = str(getattr(slab, "PredefinedType", "") or "").upper()
        if predefined == "ROOF":
            continue
        soup = _shape_soup(slab, settings)
        if not soup:
            continue
        base = len(floor_vertices) // 3
        floor_vertices.extend(soup.vertices)
        floor_indices.extend(base + i for i in soup.indices)

    stair_vertices: list[float] = []
    stair_indices: list[int] = []
    stair_entities = list(model.by_type("IfcStair")) + list(model.by_type("IfcStairFlight"))
    seen = set()
    for stair in stair_entities:
        if stair.id() in seen:
            continue
        seen.add(stair.id())
        soup = _shape_soup(stair, settings)
        if not soup:
            continue
        base = len(stair_vertices) // 3
        stair_vertices.extend(soup.vertices)
        stair_indices.extend(base + i for i in soup.indices)

    portals: list[Portal] = []
    for door in model.by_type("IfcDoor"):
        soup = _shape_soup(door, settings)
        pos = _centroid_of_soup(soup) if soup else None
        if pos is None:
            try:
                m = ifcopenshell.util.placement.get_local_placement(door.ObjectPlacement)
                pos = (float(m[0][3]) * unit_scale, float(m[1][3]) * unit_scale, float(m[2][3]) * unit_scale)
            except Exception:
                pos = (0.0, 0.0, 0.0)
        container = ifcopenshell.util.element.get_container(door)
        level_id = f"level:{container.GlobalId}" if container and container.is_a("IfcBuildingStorey") else None
        if not level_id:
            level_id = _nearest_level_id(levels, pos[2])
        width = float(door.OverallWidth) * unit_scale if getattr(door, "OverallWidth", None) else None
        is_external = _pset_bool(door, "Pset_DoorCommon", "IsExternal") is True
        name_text = f"{door.Name or ''} {getattr(door, 'ObjectType', '') or ''}".lower()
        if any(token in name_text for token in ("exit", "external", "egress", "emergency")):
            is_external = True
        portals.append(Portal(
            id=f"door:{door.GlobalId}", kind="exit" if is_external else "door", position_m=pos,
            ifc_guid=door.GlobalId, name=door.Name, level_id=level_id, width_m=width,
            is_external=is_external,
        ))

    inferred: list[Portal] = []
    for portal in portals:
        a, b = _infer_portal_spaces(portal, spaces)
        inferred.append(portal.model_copy(update={"from_space_id": a, "to_space_id": b}))

    return IfcExtraction(
        walkable_geometry=TriangleSoup(floor_vertices, floor_indices),
        stair_geometry=TriangleSoup(stair_vertices, stair_indices),
        levels=levels,
        spaces=spaces,
        portals=inferred,
    )
