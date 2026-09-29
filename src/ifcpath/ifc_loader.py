from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom

from .cdt import build_space_cdt_navmesh
from .elevator_ifc import add_ifc_elevator_connectors
from .geometry import build_radius_edges, sample_space_floor_triangles, sample_walkable_triangles
from .model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from .obstacles import (
    edge_crosses_obstacle,
    mesh_obstacle_from_triangles,
    obstacle_intersects_pedestrian_volume,
    wall_obstacle_from_vertices,
)


@dataclass(slots=True)
class BuildOptions:
    floor_spacing_m: float = 0.8
    stair_spacing_m: float = 0.25
    connect_distance_m: float = 1.25
    portal_connect_distance_m: float = 2.5
    max_slope_deg: float = 50.0
    floor_backend: str = "cdt"
    agent_clearance_m: float = 0.0
    agent_height_m: float = 1.8
    fixed_obstacle_classes: tuple[str, ...] = ("IfcColumn",)
    elevator_door_search_distance_m: float = 1.0
    elevator_landing_connect_distance_m: float = 2.5


def build_from_ifc(path: str | Path, options: BuildOptions | None = None) -> InavModel:
    options = options or BuildOptions()
    if options.floor_backend not in {"cdt", "sampled"}:
        raise ValueError(f"Unsupported floor backend: {options.floor_backend}")

    model = ifcopenshell.open(str(path))
    out = InavModel(metadata={"source_ifc": str(path)})

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
        centroid = ((bbox[0] + bbox[3]) * 0.5, (bbox[1] + bbox[4]) * 0.5, (bbox[2] + bbox[5]) * 0.5)
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

    boundary_spaces, external_boundary_elements = _boundary_space_info(model, space_by_entity_id)
    fixed_obstacles = _fixed_obstacles(model, options.fixed_obstacle_classes)

    points: list[tuple[float, float, float]] = []
    point_kinds: list[str] = []
    point_levels: list[str | None] = []
    point_spaces: list[str | None] = []
    point_cell_ids: list[str | None] = []
    floor_space_ids: set[str] = set()
    cdt_space_ids: set[str] = set()
    sampled_space_ids: set[str] = set()
    cdt_floor_indices: set[int] = set()
    prebuilt_floor_edges: list[tuple[int, int, float]] = []
    obstacle_space_applications = 0

    # Primary source: actual IfcSpace floor geometry. Fixed obstacles whose
    # vertical extents intersect pedestrian height are subtracted before CDT.
    # The actual constrained triangles are persisted into INAV as NavCell data,
    # while the existing centroid graph remains for backward compatibility.
    for entity, space in space_sources:
        mesh = _mesh(entity)
        if mesh is None:
            continue

        generated = False
        if options.floor_backend == "cdt":
            floor_z = min(vertex[2] for vertex in mesh[0])
            applicable_obstacles = [
                obstacle.footprint
                for obstacle in fixed_obstacles
                if obstacle_intersects_pedestrian_volume(
                    obstacle,
                    floor_z,
                    options.agent_height_m,
                )
            ]
            cdt = build_space_cdt_navmesh(
                mesh[0],
                mesh[1],
                clearance_m=options.agent_clearance_m,
                obstacle_footprints=applicable_obstacles,
            )
            if cdt.points:
                base = len(points)
                local_cell_ids = [
                    f"cell:{space.ifc_guid or space.id}:{index}"
                    for index in range(len(cdt.cells))
                ]
                for index, cell in enumerate(cdt.cells):
                    out.cells.append(NavCell(
                        id=local_cell_ids[index],
                        vertices_m=cell.vertices,
                        space_id=space.id,
                        level_id=space.level_id,
                        neighbor_ids=[local_cell_ids[n] for n in cell.neighbor_indices],
                    ))

                points.extend(cdt.points)
                point_kinds.extend(["walk"] * len(cdt.points))
                point_levels.extend([space.level_id] * len(cdt.points))
                point_spaces.extend([space.id] * len(cdt.points))
                point_cell_ids.extend([
                    local_cell_ids[cell_index] if cell_index is not None else None
                    for cell_index in cdt.point_cell_indices
                ])
                indices = range(base, base + len(cdt.points))
                cdt_floor_indices.update(indices)
                prebuilt_floor_edges.extend((base + a, base + b, d) for a, b, d in cdt.edges)
                floor_space_ids.add(space.id)
                cdt_space_ids.add(space.id)
                obstacle_space_applications += len(applicable_obstacles)
                generated = True

        if generated:
            continue

        sampled = sample_space_floor_triangles(
            mesh[0],
            mesh[1],
            spacing_m=options.floor_spacing_m,
        )
        if not sampled:
            continue
        floor_space_ids.add(space.id)
        sampled_space_ids.add(space.id)
        points.extend(sampled)
        point_kinds.extend(["walk"] * len(sampled))
        point_levels.extend([space.level_id] * len(sampled))
        point_spaces.extend([space.id] * len(sampled))
        point_cell_ids.extend([None] * len(sampled))

    if not floor_space_ids:
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
            slab_level_id = _spatial_level_id(entity, level_by_entity, level_by_guid)
            point_levels.extend([slab_level_id] * len(sampled))
            point_spaces.extend([_space_at_point(p, space_boxes) for p in sampled])
            point_cell_ids.extend([None] * len(sampled))

    # Prefer decomposed flight geometry per assembly, while preserving monolithic
    # vertical elements in mixed-authoring IFCs. Sampling both an assembly and
    # its flights duplicates geometry; using a global "any flight exists" switch
    # can conversely drop unrelated monolithic stairs.
    ramp_entities = _vertical_walk_entities(model, "IfcRamp", "IfcRampFlight")
    stair_entities = _vertical_walk_entities(model, "IfcStair", "IfcStairFlight")
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
            containing_level_id = _spatial_level_id(entity, level_by_entity, level_by_guid)
            point_levels.extend([
                _vertical_point_level_id(
                    p,
                    levels,
                    containing_level_id=containing_level_id,
                )
                for p in sampled
            ])
            point_spaces.extend([_space_at_point(p, space_boxes) for p in sampled])
            point_cell_ids.extend([None] * len(sampled))

    out.nodes = []
    for i, p in enumerate(points):
        out.nodes.append(NavNode(
            id=f"n:{i}",
            position_m=p,
            kind=point_kinds[i] if i < len(point_kinds) else "walk",
            level_id=point_levels[i] if i < len(point_levels) else None,
            space_id=point_spaces[i] if i < len(point_spaces) else None,
            cell_id=point_cell_ids[i] if i < len(point_cell_ids) else None,
        ))

    for i, j, d in prebuilt_floor_edges:
        out.edges.append(NavEdge(a=f"n:{i}", b=f"n:{j}", distance_m=d, kind="walk"))

    wall_obstacles = []
    for wall in model.by_type("IfcWall"):
        mesh = _mesh(wall)
        if mesh is None:
            continue
        obstacle = wall_obstacle_from_vertices(mesh[0])
        if obstacle is not None:
            wall_obstacles.append(obstacle)

    semantic_rejections: set[tuple[int, int]] = set()
    obstacle_rejections: set[tuple[int, int]] = set()
    vertical_kinds = {"stair", "ramp"}

    def candidate_allowed(i: int, j: int) -> bool:
        a_node = out.nodes[i]
        b_node = out.nodes[j]
        pair = (i, j) if i < j else (j, i)
        is_vertical_transition = a_node.kind in vertical_kinds or b_node.kind in vertical_kinds

        if i in cdt_floor_indices and j in cdt_floor_indices and not is_vertical_transition:
            return False

        if (
            not is_vertical_transition
            and a_node.space_id
            and b_node.space_id
            and a_node.space_id != b_node.space_id
        ):
            semantic_rejections.add(pair)
            return False

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
            obstacle_rejections.add(pair)
            return False

        return True

    for i, j, d in build_radius_edges(
        points,
        options.connect_distance_m,
        candidate_filter=candidate_allowed,
    ):
        out.edges.append(NavEdge(a=f"n:{i}", b=f"n:{j}", distance_m=d, kind="walk"))

    vertical_landing_edges = _attach_vertical_landings(
        out,
        max_distance_m=max(options.connect_distance_m * 2.0, 2.5),
    )

    blocked_walk_edges = len(obstacle_rejections)
    semantic_cross_space_edges = len(semantic_rejections)
    explicit_portal_count = 0
    inferred_portal_count = 0

    for door in model.by_type("IfcDoor"):
        bbox = _bbox(door)
        if bbox is None:
            continue
        p = ((bbox[0] + bbox[3]) * 0.5, (bbox[1] + bbox[4]) * 0.5, bbox[2])

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

        portal = Portal(
            id=portal_id,
            kind="door",
            position_m=p,
            from_space_id=from_space,
            to_space_id=to_space,
            level_id=level_id,
            width_m=float(getattr(door, "OverallWidth", 0.0) or 0.0) or None,
            ifc_guid=door.GlobalId,
            is_exit=_door_is_exit(door, related_element_ids, external_boundary_elements, len(connected_spaces)),
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

    elevator_stats = add_ifc_elevator_connectors(
        model,
        out,
        bbox_provider=_bbox,
        door_search_distance_m=options.elevator_door_search_distance_m,
        landing_connect_distance_m=options.elevator_landing_connect_distance_m,
    )

    out.metadata.update({
        "node_count": len(out.nodes),
        "edge_count": len(out.edges),
        "cell_count": len(out.cells),
        "space_count": len(out.spaces),
        "portal_count": len(out.portals),
        "floor_backend": options.floor_backend,
        "agent_clearance_m": options.agent_clearance_m,
        "agent_height_m": options.agent_height_m,
        "fixed_obstacle_classes": list(options.fixed_obstacle_classes),
        "fixed_obstacle_count": len(fixed_obstacles),
        "obstacle_space_applications": obstacle_space_applications,
        "space_floor_source_count": len(floor_space_ids),
        "cdt_floor_space_count": len(cdt_space_ids),
        "sampled_floor_space_count": len(sampled_space_ids),
        "used_slab_fallback": not bool(floor_space_ids),
        "explicit_space_boundary_portals": explicit_portal_count,
        "geometry_inferred_portals": inferred_portal_count,
        "wall_obstacle_count": len(wall_obstacles),
        "blocked_walk_edges": blocked_walk_edges,
        "vertical_landing_edges": vertical_landing_edges,
        "semantic_cross_space_edges_removed": semantic_cross_space_edges,
        "elevator_transport_count": elevator_stats.detected,
        "elevator_connector_count": elevator_stats.connected,
        "elevator_landing_count": elevator_stats.landings,
        "elevator_rejected_count": elevator_stats.rejected,
        "generator": "ifcpath",
    })
    return out


def _attach_vertical_landings(out: InavModel, max_distance_m: float) -> int:
    """Attach each stair/ramp endpoint to the nearest walk node on its level.

    Generic radius connectivity is intentionally local and can miss a landing
    when tessellation centroids sit farther from the flight endpoint. Vertical
    circulation needs an explicit semantic-quality attachment, constrained by
    level and distance, so a valid flight is not silently disconnected.
    """
    vertical_ids = {
        node.id for node in out.nodes if node.kind in {"stair", "ramp"}
    }
    if not vertical_ids:
        return 0
    node_by_id = {node.id: node for node in out.nodes}
    adjacency = {node_id: set() for node_id in vertical_ids}
    for edge in out.edges:
        if edge.a in vertical_ids and edge.b in vertical_ids:
            adjacency[edge.a].add(edge.b)
            adjacency[edge.b].add(edge.a)

    added = 0
    remaining = set(vertical_ids)
    while remaining:
        start = remaining.pop()
        component = {start}
        queue = [start]
        while queue:
            current = queue.pop()
            for neighbour in adjacency.get(current, ()):
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    component.add(neighbour)
                    queue.append(neighbour)

        vertical_nodes = [node_by_id[node_id] for node_id in component]
        levels = {node.level_id for node in vertical_nodes if node.level_id}
        for level_id in levels:
            level_vertical = [node for node in vertical_nodes if node.level_id == level_id]
            walk_nodes = [
                node for node in out.nodes
                if node.kind == "walk" and node.level_id == level_id and node.space_id
            ]
            if not level_vertical or not walk_nodes:
                continue
            best = min(
                (
                    (_dist(vertical.position_m, walk.position_m), vertical, walk)
                    for vertical in level_vertical for walk in walk_nodes
                ),
                key=lambda item: (item[0], item[1].id, item[2].id),
            )
            distance, vertical, walk = best
            if distance > max_distance_m:
                continue
            if any(
                {edge.a, edge.b} == {vertical.id, walk.id}
                for edge in out.edges
            ):
                continue
            out.edges.append(NavEdge(
                a=vertical.id,
                b=walk.id,
                distance_m=distance,
                kind=vertical.kind,
            ))
            added += 1
    return added


def _fixed_obstacles(model, class_names: tuple[str, ...]):
    result = []
    seen_entities: set[int] = set()
    for class_name in class_names:
        try:
            entities = model.by_type(class_name)
        except Exception:
            continue
        for entity in entities:
            if entity.id() in seen_entities:
                continue
            seen_entities.add(entity.id())
            mesh = _mesh(entity)
            if mesh is None:
                continue
            obstacle = mesh_obstacle_from_triangles(mesh[0], mesh[1])
            if obstacle is not None:
                result.append(obstacle)
    return result


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


def _vertical_point_level_id(
    point: tuple[float, float, float],
    levels: list[Level],
    *,
    containing_level_id: str | None,
) -> str | None:
    """Assign a stair/ramp sample to the served storey band containing its Z.

    A flight belongs spatially to one IfcBuildingStorey but physically spans
    toward the next one. Using the containing storey for every sample collapses
    a multi-level connector into one semantic level. Storey elevations define
    half-open vertical bands; samples at/above the next elevation therefore
    attach to that next level.
    """
    if not levels:
        return containing_level_id
    ordered = sorted(levels, key=lambda level: (level.elevation_m, level.id))
    z = float(point[2])
    selected = ordered[0].id
    for level in ordered:
        if z + 1e-6 >= float(level.elevation_m):
            selected = level.id
        else:
            break
    if containing_level_id and all(level.id != selected for level in ordered):
        return containing_level_id
    return selected


def _spatial_level_id(entity, level_by_entity: dict[int, str | None], level_by_guid: dict[str, str]) -> str | None:
    """Resolve storey through containment and decomposition ancestry."""
    queue = [entity]
    visited: set[int] = set()
    while queue:
        current = queue.pop(0)
        try:
            current_id = current.id()
        except Exception:
            current_id = id(current)
        if current_id in visited:
            continue
        visited.add(current_id)

        direct = level_by_entity.get(current_id)
        if direct:
            return direct

        for rel in getattr(current, "ContainedInStructure", ()) or ():
            parent = getattr(rel, "RelatingStructure", None)
            if parent is None:
                continue
            try:
                if parent.is_a("IfcBuildingStorey"):
                    return level_by_guid.get(parent.GlobalId)
            except Exception:
                pass
            queue.append(parent)

        for rel in getattr(current, "Decomposes", ()) or ():
            parent = getattr(rel, "RelatingObject", None)
            if parent is None:
                continue
            try:
                if parent.is_a("IfcBuildingStorey"):
                    return level_by_guid.get(parent.GlobalId)
            except Exception:
                pass
            queue.append(parent)
    return None


def _boundary_space_info(model, space_by_entity_id: dict[int, Space]) -> tuple[dict[int, list[Space]], set[int]]:
    result: dict[int, list[Space]] = {}
    external_elements: set[int] = set()
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
            boundary_kind = str(getattr(rel, "InternalOrExternalBoundary", "") or "").upper()
            if boundary_kind == "EXTERNAL":
                external_elements.add(element.id())
    return result, external_elements


def _door_is_exit(door, related_element_ids: set[int], external_boundary_elements: set[int], connected_space_count: int) -> bool:
    """Classify an exterior portal using IFC semantics before geometry fallback."""
    semantic = _door_external_property(door)
    if semantic is not None:
        return semantic
    if related_element_ids & external_boundary_elements:
        return True
    return connected_space_count == 1


def _door_external_property(door) -> bool | None:
    """Read Pset_DoorCommon.IsExternal when authored on occurrence or type."""
    relationships = list(getattr(door, "IsDefinedBy", ()) or ())
    for typed_by in getattr(door, "IsTypedBy", ()) or ():
        relating_type = getattr(typed_by, "RelatingType", None)
        relationships.extend(getattr(relating_type, "HasPropertySets", ()) or ())

    property_sets = []
    for relation in relationships:
        pset = getattr(relation, "RelatingPropertyDefinition", relation)
        if getattr(pset, "Name", None) == "Pset_DoorCommon":
            property_sets.append(pset)

    for pset in property_sets:
        for prop in getattr(pset, "HasProperties", ()) or ():
            if getattr(prop, "Name", None) != "IsExternal":
                continue
            nominal = getattr(prop, "NominalValue", None)
            value = getattr(nominal, "wrappedValue", nominal)
            if isinstance(value, bool):
                return value
    return None


def _space_at_point(point, space_boxes, tolerance_m: float = 0.15) -> str | None:
    x, y, z = point
    for space, box in space_boxes:
        if (
            box[0] - tolerance_m <= x <= box[3] + tolerance_m
            and box[1] - tolerance_m <= y <= box[4] + tolerance_m
            and box[2] - tolerance_m <= z <= box[5] + tolerance_m
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
    vertices = [(float(verts[i]), float(verts[i + 1]), float(verts[i + 2])) for i in range(0, len(verts), 3)]
    triangles = [(int(faces[i]), int(faces[i + 1]), int(faces[i + 2])) for i in range(0, len(faces), 3)]
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
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _distance_to_box(p, box):
    dx = max(box[0] - p[0], 0.0, p[0] - box[3])
    dy = max(box[1] - p[1], 0.0, p[1] - box[4])
    dz = max(box[2] - p[2], 0.0, p[2] - box[5])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _vertical_walk_entities(model, assembly_type: str, flight_type: str):
    """Select vertical walk geometry without duplicating decomposed assemblies.

    IFC permits stairs/ramps either as a monolithic element or as an assembly
    decomposed into flights/landings. Keep all flights, plus only those parent
    assemblies that do not actually aggregate a flight of the requested type.
    This is intentionally decided per assembly so mixed authoring styles in one
    building remain valid.
    """
    flights = list(model.by_type(flight_type))
    assemblies = list(model.by_type(assembly_type))
    result = list(flights)

    for assembly in assemblies:
        has_flight_child = False
        for relation in getattr(assembly, "IsDecomposedBy", ()) or ():
            for child in getattr(relation, "RelatedObjects", ()) or ():
                try:
                    if child.is_a(flight_type):
                        has_flight_child = True
                        break
                except Exception:
                    continue
            if has_flight_child:
                break
        if not has_flight_child:
            result.append(assembly)

    return result
