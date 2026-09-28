from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom

from .geometry import build_radius_edges, sample_walkable_triangles
from .model import InavModel, Level, NavEdge, NavNode, Portal, Space


@dataclass(slots=True)
class BuildOptions:
    floor_spacing_m: float = 0.8
    stair_spacing_m: float = 0.25
    connect_distance_m: float = 1.25
    portal_connect_distance_m: float = 2.5
    max_slope_deg: float = 50.0


def build_from_ifc(path: str | Path, options: BuildOptions | None = None) -> InavModel:
    options = options or BuildOptions()
    model = ifcopenshell.open(str(path))
    out = InavModel(metadata={"source_ifc": str(path)})

    levels = _levels(model)
    out.levels.extend(levels)
    level_by_entity = {x[0]: x[1] for x in _contained_levels(model, levels)}

    space_boxes: list[tuple[Space, tuple[float, float, float, float, float, float]]] = []
    for entity in model.by_type("IfcSpace"):
        bbox = _bbox(entity)
        if bbox is None:
            continue
        level_id = level_by_entity.get(entity.id())
        centroid = ((bbox[0]+bbox[3])*0.5, (bbox[1]+bbox[4])*0.5, (bbox[2]+bbox[5])*0.5)
        space = Space(
            id=f"space:{entity.GlobalId}",
            name=entity.Name or entity.LongName or entity.GlobalId,
            level_id=level_id,
            centroid_m=centroid,
            ifc_guid=entity.GlobalId,
        )
        out.spaces.append(space)
        space_boxes.append((space, bbox))

    points: list[tuple[float, float, float]] = []
    point_kinds: list[str] = []
    for ifc_type, spacing, kind in (
        ("IfcSlab", options.floor_spacing_m, "walk"),
        ("IfcRamp", options.floor_spacing_m, "ramp"),
        ("IfcStair", options.stair_spacing_m, "stair"),
        ("IfcStairFlight", options.stair_spacing_m, "stair"),
    ):
        for entity in model.by_type(ifc_type):
            mesh = _mesh(entity)
            if mesh is None:
                continue
            verts, tris = mesh
            sampled = sample_walkable_triangles(verts, tris, spacing, options.max_slope_deg)
            points.extend(sampled)
            point_kinds.extend([kind] * len(sampled))

    out.nodes = [
        NavNode(id=f"n:{i}", position_m=p, kind=point_kinds[i] if i < len(point_kinds) else "walk")
        for i, p in enumerate(points)
    ]
    for i, j, d in build_radius_edges(points, options.connect_distance_m):
        out.edges.append(NavEdge(a=f"n:{i}", b=f"n:{j}", distance_m=d, kind="walk"))

    for door in model.by_type("IfcDoor"):
        bbox = _bbox(door)
        if bbox is None:
            continue
        p = ((bbox[0]+bbox[3])*0.5, (bbox[1]+bbox[4])*0.5, bbox[2])
        candidates = sorted(
            (( _distance_to_box(p, sb), s) for s, sb in space_boxes),
            key=lambda x: x[0],
        )
        close = [s for d, s in candidates if d <= 1.5][:2]
        from_space = close[0].id if close else None
        to_space = close[1].id if len(close) > 1 else None
        portal_id = f"door:{door.GlobalId}"
        portal = Portal(
            id=portal_id,
            kind="door",
            position_m=p,
            from_space_id=from_space,
            to_space_id=to_space,
            level_id=level_by_entity.get(door.id()),
            width_m=float(getattr(door, "OverallWidth", 0.0) or 0.0) or None,
            ifc_guid=door.GlobalId,
            is_exit=(len(close) == 1),
        )
        out.portals.append(portal)

        node_id = f"p:{door.GlobalId}"
        out.nodes.append(NavNode(id=node_id, position_m=p, kind="portal", portal_id=portal_id))
        nearest = sorted(
            (( _dist(p, n.position_m), n) for n in out.nodes if n.id.startswith("n:")),
            key=lambda x: x[0],
        )
        for d, n in nearest[:6]:
            if d <= options.portal_connect_distance_m:
                out.edges.append(NavEdge(a=node_id, b=n.id, distance_m=d, kind="portal", portal_id=portal_id))

    out.metadata.update({
        "node_count": len(out.nodes),
        "edge_count": len(out.edges),
        "space_count": len(out.spaces),
        "portal_count": len(out.portals),
        "generator": "ifcpath",
    })
    return out


def _levels(model) -> list[Level]:
    result: list[Level] = []
    for storey in model.by_type("IfcBuildingStorey"):
        result.append(Level(
            id=f"level:{storey.GlobalId}",
            name=storey.Name or storey.GlobalId,
            elevation_m=float(storey.Elevation or 0.0),
        ))
    return result


def _contained_levels(model, levels: list[Level]):
    by_guid = {x.id.removeprefix("level:"): x.id for x in levels}
    for rel in model.by_type("IfcRelContainedInSpatialStructure"):
        structure = rel.RelatingStructure
        if not structure or not structure.is_a("IfcBuildingStorey"):
            continue
        level_id = by_guid.get(structure.GlobalId)
        for element in rel.RelatedElements:
            yield element.id(), level_id


def _settings():
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    return settings


def _mesh(entity):
    try:
        shape = ifcopenshell.geom.create_shape(_settings(), entity)
    except Exception:
        return None
    verts = shape.geometry.verts
    faces = shape.geometry.faces
    vertices = [(float(verts[i]), float(verts[i+1]), float(verts[i+2])) for i in range(0, len(verts), 3)]
    triangles = [(int(faces[i]), int(faces[i+1]), int(faces[i+2])) for i in range(0, len(faces), 3)]
    return vertices, triangles


def _bbox(entity):
    mesh = _mesh(entity)
    if mesh is None or not mesh[0]:
        return None
    xs = [p[0] for p in mesh[0]]
    ys = [p[1] for p in mesh[0]]
    zs = [p[2] for p in mesh[0]]
    return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


def _dist(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) ** 0.5


def _distance_to_box(p, box):
    dx = max(box[0]-p[0], 0.0, p[0]-box[3])
    dy = max(box[1]-p[1], 0.0, p[1]-box[4])
    dz = max(box[2]-p[2], 0.0, p[2]-box[5])
    return (dx*dx + dy*dy + dz*dz) ** 0.5
