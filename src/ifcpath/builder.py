from __future__ import annotations

import math
from pathlib import Path

from .geometry import build_radius_edges, sample_walkable_triangles
from .ifc_extract import extract_ifc
from .models import NavEdge, NavigationModel, NavNode, NodeKind
from .topology import containing_space_id, nearest_level_id


def build_navigation(
    ifc_path: str | Path,
    floor_spacing_m: float = 0.75,
    stair_spacing_m: float = 0.25,
    floor_connect_m: float = 1.15,
    stair_connect_m: float = 0.55,
    door_connect_m: float = 2.0,
) -> NavigationModel:
    data = extract_ifc(ifc_path)
    floor_pts = sample_walkable_triangles(data.walkable_geometry, floor_spacing_m, max_slope_deg=10)
    stair_pts = sample_walkable_triangles(data.stair_geometry, stair_spacing_m, max_slope_deg=50)

    nodes: list[NavNode] = []
    for i, p in enumerate(floor_pts):
        nodes.append(NavNode(
            id=f"walk:{i}", position_m=p, kind=NodeKind.WALK,
            level_id=nearest_level_id(data.levels, p),
            space_id=containing_space_id(data.spaces, p),
        ))
    for i, p in enumerate(stair_pts):
        nodes.append(NavNode(
            id=f"stair:{i}", position_m=p, kind=NodeKind.STAIR,
            level_id=nearest_level_id(data.levels, p),
        ))

    edges: list[NavEdge] = []
    for a, b, d in build_radius_edges(floor_pts, floor_connect_m):
        edges.append(NavEdge(a=f"walk:{a}", b=f"walk:{b}", length_m=d))
    for a, b, d in build_radius_edges(stair_pts, stair_connect_m):
        edges.append(NavEdge(a=f"stair:{a}", b=f"stair:{b}", length_m=d, kind="stair"))

    if stair_pts and floor_pts:
        zmin, zmax = min(p[2] for p in stair_pts), max(p[2] for p in stair_pts)
        endpoint_stairs = [
            (i, p) for i, p in enumerate(stair_pts)
            if abs(p[2] - zmin) <= stair_connect_m or abs(p[2] - zmax) <= stair_connect_m
        ]
        for si, sp in endpoint_stairs:
            candidates = sorted(
                ((math.dist(sp, fp), fi) for fi, fp in enumerate(floor_pts)),
                key=lambda x: x[0],
            )[:4]
            for d, fi in candidates:
                if d <= max(door_connect_m, floor_connect_m * 2):
                    edges.append(NavEdge(
                        a=f"stair:{si}", b=f"walk:{fi}", length_m=d, kind="stair-landing",
                    ))

    node_by_id = {n.id: n for n in nodes}
    for portal in data.portals:
        kind = NodeKind.EXIT if portal.is_external else NodeKind.DOOR
        nodes.append(NavNode(
            id=portal.id, position_m=portal.position_m, kind=kind,
            level_id=portal.level_id, source_ifc_guid=portal.ifc_guid,
        ))

        target_spaces = {x for x in (portal.from_space_id, portal.to_space_id) if x}
        candidates = []
        for fi, fp in enumerate(floor_pts):
            n = node_by_id[f"walk:{fi}"]
            preferred = 0 if (target_spaces and n.space_id in target_spaces) else 1
            candidates.append((preferred, math.dist(portal.position_m, fp), fi))
        candidates.sort(key=lambda x: (x[0], x[1]))

        linked = 0
        for preferred, d, fi in candidates[:12]:
            if d > door_connect_m:
                continue
            if target_spaces and preferred == 1 and linked >= 2:
                continue
            edges.append(NavEdge(
                a=portal.id, b=f"walk:{fi}", length_m=d,
                kind="exit" if portal.is_external else "door", portal_id=portal.id,
            ))
            linked += 1
            if linked >= 6:
                break

    warnings: list[str] = []
    if not floor_pts:
        warnings.append("No walkable floor samples were generated from IfcSlab geometry.")
    if not data.spaces:
        warnings.append("IFC contains no IfcSpace objects; semantic room routing will be limited.")
    for portal in data.portals:
        if not any(e.portal_id == portal.id for e in edges):
            warnings.append(f"Portal {portal.id} could not be connected to the walk graph.")
        if not portal.from_space_id:
            warnings.append(f"Portal {portal.id} could not be associated with an adjacent IfcSpace.")
        if not portal.is_external and not portal.to_space_id:
            warnings.append(f"Door {portal.id} has only one inferred adjacent space; check IFC space boundaries.")

    return NavigationModel(
        levels=data.levels,
        spaces=data.spaces,
        portals=data.portals,
        nodes=nodes,
        edges=edges,
        warnings=warnings,
    )
