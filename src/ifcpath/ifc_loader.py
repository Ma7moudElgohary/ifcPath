from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom

from .geometry import build_radius_edges, sample_walkable_triangles
from .model import InavModel, Level, NavEdge, NavNode, Portal, Space
from .obstacles import edge_crosses_obstacle, wall_obstacle_from_vertices


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
    space_by_entity_id: dict[int, Space] = {}
    for entity in model.by_type("IfcSpace"):
        bbox = _bbox(entity)
        if bbox is None:
            continue
        level_id = level_by_entity.get(entity.id())
        centroid = ((bbox[0]+bbox[3])*0.5, (bbox[1]+bbox[4])*0.5, (bbox[2]+bbox[5])*0.5)
        space = Space(
            id=f"space:{entity.GlobalId}",
            name=entity.Name or getattr(entity, "LongName", None) or entity.GlobalId,
            level_id=level_id,
            centroid_m=centroid,
            ifc_guid=entity.GlobalId,
        )
        out.spaces.append(space)
        space_boxes.append((space, bbox))
        space_by_entity_id[entity.id()] = space

    boundary_spaces = _boundary_space_map(model, space_by_entity_id)

    points: list[tuple[float, float, float]] = []
    point_kinds: list[str] = []
    point_levels: list[str | None] = []

    floor_entities = [
        e for e in model.by_type("IfcSlab")
        if str(getattr(e, "PredefinedType", "")).upper() not in {"ROOF"}
    ]
    ramp_entities = list(model.by_type("IfcRamp")) + list(model.by_type("IfcRampFlight"))
    stair_flights = list(model.by_type("IfcStairFlight"))
    stair_entities = stair_flights if stair_flights else list(model.by_type("IfcStair"))

    for entities, spacing, kind in (
        (floor_entities, options.floor_spacing_m, "walk"),
        (ramp_entities, options.floor_spacing_m, "ramp"),
        (stair_entities, options.stair_spacing_m, "stair"),
    ):
        for entity in entities:
            mesh = _mesh(entity)
            if mesh is None:
                continue
            verts, tris = mesh
            sampled = sample_walkable_triangles(verts, tris, spacing, options.max_slope_deg)
            points.extend(sampled)
            point_kinds.extend([kind] * len(sampled))
            point_levels.extend([level_by_entity.get(entity.id())] * len(sampled))

    out.nodes = []
    for i, p in enumerate(points):
        out.nodes.append(NavNode(
            id=f"n:{i}",
            position_m=p,
            kind=point_kinds[i] if i < len(point_kinds) else "walk",
            level_id=point_levels[i] if i < len(point_levels) else None,
            space_id=_space_at_point(p, space_boxes),
        ))

    wall_obstacles = []
    for wall in model.by_type("IfcWall"):
        mesh = _mesh(wall)
        if mesh is None:
            continue
        obstacle = wall_obstacle_from_vertices(mesh[0])
        if obstacle is not None:
            wall_obstacles.append(obstacle)

    blocked_walk_edges = 0
    for i, j, d in build_radius_edges(points, options.connect_distance_m):
        if edge_crosses_obstacle(points[i], points[j], wall_obstacles):
            blocked_walk_edges += 1
            continue
        out.edges.append(NavEdge(a=f"n:{i}", b=f"n:{j}", distance_m=d, kind="walk"))

    explicit_portal_count = 0
    inferred_portal_count = 0

    for door in model.by_type("IfcDoor"):
        bbox = _bbox(door)
        if bbox is None:
            continue
        p = ((bbox[0]+bbox[3])*0.5, (bbox[1]+bbox[4])*0.5, bbox[2])

        related_element_ids = {door.id()}
        for fills_rel in getattr(door, "FillsVoids", ()) or ():
            opening = getattr(fills_rel, "RelatingOpeningElement", None)
            if opening is not None:
                related_element_ids.add(opening.id())

        explicit_spaces: list[Space] = []
        seen_space_ids: set[str] = set()
        for element_id in related_element_ids:
            for space in boundary_spaces.get(element_id, ()):
                if space.id not in seen_space_ids:
                    explicit_spaces.append(space)
                    seen_space_ids.add(space.id)

        if explicit_spaces:
            connected_spaces = explicit_spaces[:2]
            explicit_portal_count += 1
        else:
            candidates = sorted(
                ((_distance_to_box(p, sb), s) for s, sb in space_boxes),
                key=lambda x: x[0],
            )
            connected_spaces = [s for d, s in candidates if d <= 1.5][:2]
            inferred_portal_count += 1

        from_space = connected_spaces[0].id if connected_spaces else None
        to_space = connected_spaces[1].id if len(connected_spaces) > 1 else None
        portal_id = f"door:{door.GlobalId}"
        level_id = level_by_entity.get(door.id())

        portal = Portal(
            id=portal_id,
            kind="door",
            position_m=p,
            from_space_id=from_space,
            to_space_id=to_space,
            level_id=level_id,
            width_m=float(getattr(door, "OverallWidth", 0.0) or 0.0) or None,
            ifc_guid=door.GlobalId,
            is_exit=(len(connected_spaces) == 1),
        )
        out.portals.append(portal)

        node_id = f"p:{door.GlobalId}"
        out.nodes.append(NavNode(
            id=node_id,
            position_m=p,
            kind="portal",
            level_id=level_id,
            space_id=from_space,
            portal_id=portal_id,
        ))

        # Portal edges intentionally bypass wall-obstacle filtering: doors are
        # the sanctioned graph transitions through wall footprints.
        nearest = sorted(
            ((_dist(p, n.position_m), n) for n in out.nodes if n.id.startswith("n:")),
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
        "explicit_space_boundary_portals": explicit_portal_count,
        "geometry_inferred_portals": inferred_portal_count,
        "wall_obstacle_count": len(wall_obstacles),
        "blocked_walk_edges": blocked_walk_edges,
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


def _boundary_space_map(model, space_by_entity_id: dict[int, Space]) -> dict[int, list[Space]]:
    """Map boundary-related IFC elements/openings to their relating spaces.

    IFC models that carry IfcRelSpaceBoundary data get semantic connectivity first;
    geometry is only the fallback for less complete authoring exports.
    """
    result: dict[int, list[Space]] = {}
    for relation_type in ("IfcRelSpaceBoundary", "IfcRelSpaceBoundary1stLevel", "IfcRelSpaceBoundary2ndLevel"):
        try:
            relations = model.by_type(relation_type)
        except Exception:
            continue
        for rel in relations:
            space_entity = getattr(rel, "RelatingSpace", None)
            element = getattr(rel, "RelatedBuildingElement", None)
            if space_entity is None or element is None:
                continue
            space = space_by_entity_id.get(space_entity.id())
            if space is None:
                continue
            bucket = result.setdefault(element.id(), [])
            if all(existing.id != space.id for existing in bucket):
                bucket.append(space)
    return result


def _space_at_point(point, space_boxes, tolerance_m: float = 0.15) -> str | None:
    x, y, z = point
    for space, box in space_boxes:
        if (
            box[0]-tolerance_m <= x <= box[3]+tolerance_m
            and box[1]-tolerance_m <= y <= box[4]+tolerance_m
            and box[2]-tolerance_m <= z <= box[5]+tolerance_m
        ):
            return space.id
    return None


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
