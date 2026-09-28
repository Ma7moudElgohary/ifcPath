from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom

from .exits import classify_ifc_door_exit
from .geometry import build_radius_edges, sample_space_floor_triangles, sample_walkable_triangles
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
    out = InavModel(metadata={
        "source_ifc": str(path),
        "source_schema": str(getattr(model, "schema", "") or ""),
    })

    levels = _levels(model)
    out.levels.extend(levels)
    level_by_entity = {x[0]: x[1] for x in _contained_levels(model, levels)}
    level_by_guid = {level.id.removeprefix("level:"): level.id for level in levels}

    space_boxes: list[tuple[Space, tuple[float, float, float, float, float, float]]] = []
    space_sources: list[tuple[object, Space]] = []
    space_by_entity_id: dict[int, Space] = {}
    for entity in model.by_type("IfcSpace"):
        mesh = _mesh(entity)
        if mesh is None or not mesh[0]:
            continue
        bbox = _bbox_from_vertices(mesh[0])
        level_id = _spatial_level_id(entity, level_by_entity, level_by_guid)
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
        space_sources.append((entity, space))
        space_by_entity_id[entity.id()] = space

    boundary_spaces = _boundary_space_map(model, space_by_entity_id)

    points: list[tuple[float, float, float]] = []
    point_kinds: list[str] = []
    point_levels: list[str | None] = []
    point_spaces: list[str | None] = []
    sampled_space_ids: set[str] = set()

    # Primary source: actual IfcSpace floor geometry. Nodes are born with exact
    # semantic space ownership instead of inferring rooms from slab/bbox overlap.
    for entity, space in space_sources:
        mesh = _mesh(entity)
        if mesh is None:
            continue
        sampled = sample_space_floor_triangles(
            mesh[0],
            mesh[1],
            spacing_m=options.floor_spacing_m,
        )
        if not sampled:
            continue
        sampled_space_ids.add(space.id)
        points.extend(sampled)
        point_kinds.extend(["walk"] * len(sampled))
        point_levels.extend([space.level_id] * len(sampled))
        point_spaces.extend([space.id] * len(sampled))

    # IFCs without usable IfcSpace geometry still get the original slab fallback.
    if not sampled_space_ids:
        floor_entities = [
            e for e in model.by_type("IfcSlab")
            if str(getattr(e, "PredefinedType", "")).upper() not in {"ROOF"}
        ]
        for entity in floor_entities:
            mesh = _mesh(entity)
            if mesh is None:
                continue
            sampled = sample_walkable_triangles(
                mesh[0], mesh[1], options.floor_spacing_m, options.max_slope_deg
            )
            points.extend(sampled)
            point_kinds.extend(["walk"] * len(sampled))
            point_levels.extend([level_by_entity.get(entity.id())] * len(sampled))
            point_spaces.extend([_space_at_point(p, space_boxes) for p in sampled])

    # Vertical circulation remains geometry-driven and is allowed to bridge
    # storeys/spaces. It is sampled after space floors so local routing can join
    # landings to the nearest floor nodes.
    ramp_entities = list(model.by_type("IfcRamp")) + list(model.by_type("IfcRampFlight"))
    stair_flights = list(model.by_type("IfcStairFlight"))
    stair_entities = stair_flights if stair_flights else list(model.by_type("IfcStair"))
    for entities, spacing, kind in (
        (ramp_entities, options.floor_spacing_m, "ramp"),
        (stair_entities, options.stair_spacing_m, "stair"),
    ):
        for entity in entities:
            mesh = _mesh(entity)
            if mesh is None:
                continue
            sampled = sample_walkable_triangles(mesh[0], mesh[1], spacing, options.max_slope_deg)
            points.extend(sampled)
            point_kinds.extend([kind] * len(sampled))
            point_levels.extend([level_by_entity.get(entity.id())] * len(sampled))
            point_spaces.extend([_space_at_point(p, space_boxes) for p in sampled])

    out.nodes = []
    for i, p in enumerate(points):
        out.nodes.append(NavNode(
            id=f"n:{i}",
            position_m=p,
            kind=point_kinds[i] if i < len(point_kinds) else "walk",
            level_id=point_levels[i] if i < len(point_levels) else None,
            space_id=point_spaces[i] if i < len(point_spaces) else None,
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
    semantic_cross_space_edges = 0
    vertical_kinds = {"stair", "ramp"}
    for i, j, d in build_radius_edges(points, options.connect_distance_m):
        a_node = out.nodes[i]
        b_node = out.nodes[j]
        is_vertical_transition = a_node.kind in vertical_kinds or b_node.kind in vertical_kinds

        if (
            not is_vertical_transition
            and a_node.space_id
            and b_node.space_id
            and a_node.space_id != b_node.space_id
        ):
            semantic_cross_space_edges += 1
            continue

        needs_geometry_check = not (
            a_node.space_id
            and b_node.space_id
            and a_node.space_id == b_node.space_id
        )
        if (
            needs_geometry_check
            and not is_vertical_transition
            and edge_crosses_obstacle(points[i], points[j], wall_obstacles)
        ):
            blocked_walk_edges += 1
            continue

        out.edges.append(NavEdge(a=f"n:{i}", b=f"n:{j}", distance_m=d, kind="walk"))

    explicit_portal_count = 0
    inferred_portal_count = 0
    explicit_external_exit_count = 0
    explicit_internal_door_count = 0
    heuristic_exit_count = 0

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
        level_id = level_by_entity.get(door.id()) or (
            connected_spaces[0].level_id if connected_spaces else None
        )
        exit_classification = classify_ifc_door_exit(door, len(connected_spaces))
        if exit_classification.source == "Pset_DoorCommon.IsExternal":
            if exit_classification.is_exit:
                explicit_external_exit_count += 1
            else:
                explicit_internal_door_count += 1
        elif exit_classification.is_exit:
            heuristic_exit_count += 1

        portal = Portal(
            id=portal_id,
            kind="door",
            position_m=p,
            from_space_id=from_space,
            to_space_id=to_space,
            level_id=level_id,
            width_m=float(getattr(door, "OverallWidth", 0.0) or 0.0) or None,
            ifc_guid=door.GlobalId,
            is_exit=exit_classification.is_exit,
            is_external=exit_classification.is_external,
            exit_source=exit_classification.source,
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

        connected_node_ids: set[str] = set()
        for connected_space in connected_spaces:
            nearest_in_space = sorted(
                (
                    (_dist(p, n.position_m), n)
                    for n in out.nodes
                    if n.id.startswith("n:") and n.space_id == connected_space.id
                ),
                key=lambda x: x[0],
            )
            for d, n in nearest_in_space[:3]:
                if d <= options.portal_connect_distance_m and n.id not in connected_node_ids:
                    out.edges.append(NavEdge(
                        a=node_id,
                        b=n.id,
                        distance_m=d,
                        kind="portal",
                        portal_id=portal_id,
                    ))
                    connected_node_ids.add(n.id)

        if not connected_node_ids:
            nearest = sorted(
                ((_dist(p, n.position_m), n) for n in out.nodes if n.id.startswith("n:")),
                key=lambda x: x[0],
            )
            for d, n in nearest[:6]:
                if d <= options.portal_connect_distance_m:
                    out.edges.append(NavEdge(
                        a=node_id,
                        b=n.id,
                        distance_m=d,
                        kind="portal",
                        portal_id=portal_id,
                    ))

    out.metadata.update({
        "node_count": len(out.nodes),
        "edge_count": len(out.edges),
        "space_count": len(out.spaces),
        "portal_count": len(out.portals),
        "space_floor_source_count": len(sampled_space_ids),
        "used_slab_fallback": not bool(sampled_space_ids),
        "explicit_space_boundary_portals": explicit_portal_count,
        "geometry_inferred_portals": inferred_portal_count,
        "explicit_external_exits": explicit_external_exit_count,
        "explicit_internal_doors": explicit_internal_door_count,
        "heuristic_exits": heuristic_exit_count,
        "wall_obstacle_count": len(wall_obstacles),
        "blocked_walk_edges": blocked_walk_edges,
        "semantic_cross_space_edges_removed": semantic_cross_space_edges,
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


def _spatial_level_id(entity, level_by_entity: dict[int, str | None], level_by_guid: dict[str, str]) -> str | None:
    direct = level_by_entity.get(entity.id())
    if direct:
        return direct
    for rel in getattr(entity, "Decomposes", ()) or ():
        parent = getattr(rel, "RelatingObject", None)
        if parent is not None and parent.is_a("IfcBuildingStorey"):
            return level_by_guid.get(parent.GlobalId)
    for rel in getattr(entity, "ContainedInStructure", ()) or ():
        parent = getattr(rel, "RelatingStructure", None)
        if parent is not None and parent.is_a("IfcBuildingStorey"):
            return level_by_guid.get(parent.GlobalId)
    return None


def _boundary_space_map(model, space_by_entity_id: dict[int, Space]) -> dict[int, list[Space]]:
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
    return _bbox_from_vertices(mesh[0])


def _bbox_from_vertices(vertices):
    xs = [p[0] for p in vertices]
    ys = [p[1] for p in vertices]
    zs = [p[2] for p in vertices]
    return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


def _dist(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) ** 0.5


def _distance_to_box(p, box):
    dx = max(box[0]-p[0], 0.0, p[0]-box[3])
    dy = max(box[1]-p[1], 0.0, p[1]-box[4])
    dz = max(box[2]-p[2], 0.0, p[2]-box[5])
    return (dx*dx + dy*dy + dz*dz) ** 0.5
