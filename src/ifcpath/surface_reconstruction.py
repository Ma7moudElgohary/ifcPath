from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom
from shapely.geometry import Point

from .cdt import space_floor_polygon
from .model import InavModel, NavCell
from .raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceDetectionStats,
    SurfaceSamplingDomain,
    detect_ifc_walkable_cells,
    ifc_walkable_support_role,
)
from .surface_nav import connect_cells_by_shared_edges, stitch_surface_seams, surface_components


@dataclass(slots=True)
class SurfaceReconstructionStats:
    requested: bool = True
    replaced_legacy_surface: bool = False
    support_elements: int = 0
    labelled_cells: int = 0
    unlabelled_cells: int = 0
    component_count: int = 0
    seam_count: int = 0
    detector: SurfaceDetectionStats | None = None
    fallback_reason: str | None = None

    def to_dict(self) -> dict:
        raw = asdict(self)
        return raw


def reconstruct_walkable_surface(
    ifc_path: str | Path,
    model: InavModel,
    *,
    cell_size_m: float = 0.15,
    agent_height_m: float = 1.8,
    agent_radius_m: float = 0.22,
    max_slope_deg: float = 50.0,
    max_climb_m: float = 0.24,
    minimum_cells: int = 8,
) -> SurfaceReconstructionStats:
    """Replace legacy cells with a geometry-derived physical support surface.

    The legacy importer is deliberately kept as a fallback while this detector is
    qualified against the public IFC corpus. Replacement occurs only when the
    physical detector produces a non-trivial connected surface. IFC spaces are
    consulted *after* geometry reconstruction to label cells, never to create Z.
    """
    stats = SurfaceReconstructionStats()
    try:
        ifc_file = ifcopenshell.open(str(ifc_path))
    except Exception as exc:
        stats.fallback_reason = f"cannot reopen IFC for physical surface detection: {exc}"
        return stats

    domains = _support_domains(ifc_file, model)
    stats.support_elements = len(domains)
    if not domains:
        stats.fallback_reason = "no physical walkable-support IFC elements found"
        return stats

    detection_options = SurfaceDetectionOptions(
        cell_size_m=max(0.05, float(cell_size_m)),
        agent_height_m=max(0.0, float(agent_height_m)),
        agent_radius_m=max(0.0, float(agent_radius_m)),
        max_slope_deg=max(0.0, min(89.0, float(max_slope_deg))),
        max_climb_m=max(0.0, float(max_climb_m)),
    )
    cells, detector_stats = detect_ifc_walkable_cells(
        ifc_file,
        domains,
        options=detection_options,
    )
    stats.detector = detector_stats
    if detector_stats.error:
        stats.fallback_reason = detector_stats.error
        return stats
    if len(cells) < minimum_cells:
        stats.fallback_reason = (
            f"physical detector produced only {len(cells)} cells; "
            f"minimum is {minimum_cells}"
        )
        return stats

    _label_cells_from_ifc_spaces(ifc_file, model, cells)
    stats.labelled_cells = sum(cell.space_id is not None for cell in cells)
    stats.unlabelled_cells = len(cells) - stats.labelled_cells

    # Shared grid edges are exact where the detector saw one support manifold.
    # The conservative seam pass handles modelling gaps between independent IFC
    # support objects (e.g. stair flight to landing) without proximity graph links.
    connect_cells_by_shared_edges(
        cells,
        tolerance_m=max(1e-5, detection_options.hit_merge_tolerance_m * 0.2),
    )
    stats.seam_count = stitch_surface_seams(
        cells,
        max_gap_m=max(0.05, detection_options.cell_size_m * 1.1),
        max_vertical_gap_m=max(
            detection_options.max_climb_m + detection_options.hit_merge_tolerance_m,
            0.10,
        ),
    )
    components = surface_components(cells)
    stats.component_count = len(components)

    model.cells = cells
    model.metadata.update(
        {
            "surface_source": "ifc-physical-raycast",
            "surface_reconstruction": stats.to_dict(),
            "surface_detector_cell_size_m": detection_options.cell_size_m,
            "surface_detector_agent_radius_m": detection_options.agent_radius_m,
            "surface_detector_agent_height_m": detection_options.agent_height_m,
            "surface_detector_max_climb_m": detection_options.max_climb_m,
            "surface_detector_max_slope_deg": detection_options.max_slope_deg,
            "legacy_cell_count_before_reconstruction": int(
                model.metadata.get("cell_count", 0) or 0
            ),
            "cell_count": len(cells),
        }
    )
    stats.replaced_legacy_surface = True
    # Refresh the serialized stats now that the replacement flag is true.
    model.metadata["surface_reconstruction"] = stats.to_dict()
    return stats


def _support_domains(ifc_file, model: InavModel) -> list[SurfaceSamplingDomain]:
    level_by_guid = {level.id.removeprefix("level:"): level.id for level in model.levels}
    domains: list[SurfaceSamplingDomain] = []
    try:
        elements = list(ifc_file.by_type("IfcElement"))
    except Exception:
        elements = []

    for entity in elements:
        terrain = ifc_walkable_support_role(entity)
        if terrain is None:
            continue
        bounds = _bbox(entity)
        if bounds is None:
            continue
        entity_id = int(entity.id())
        guid = str(_safe_attr(entity, "GlobalId", entity_id))
        level_id = _entity_level_id(entity, level_by_guid)
        domains.append(
            SurfaceSamplingDomain(
                id=f"support:{guid}",
                bounds=bounds,
                support_entity_ids=frozenset({entity_id}),
                level_id=level_id,
                terrain=terrain,
            )
        )
    return domains


def _label_cells_from_ifc_spaces(ifc_file, model: InavModel, cells: list[NavCell]) -> None:
    if not cells or not model.spaces:
        return

    space_by_guid = {
        space.ifc_guid or space.id.removeprefix("space:"): space
        for space in model.spaces
    }
    regions = []
    try:
        space_entities = list(ifc_file.by_type("IfcSpace"))
    except Exception:
        space_entities = []

    for entity in space_entities:
        mesh = _mesh(entity)
        if mesh is None or not mesh[0]:
            continue
        guid = str(_safe_attr(entity, "GlobalId", ""))
        space = space_by_guid.get(guid)
        if space is None:
            continue
        bounds = _bbox_from_vertices(mesh[0])
        polygon = space_floor_polygon(mesh[0], mesh[1])
        if polygon is None or polygon.is_empty:
            continue
        regions.append((float(polygon.area), bounds, polygon, space))

    # Smallest containing space wins if malformed/nested space geometry overlaps.
    regions.sort(key=lambda item: item[0])
    levels = sorted(model.levels, key=lambda level: level.elevation_m)

    for cell in cells:
        center = _cell_centroid(cell)
        for _, bounds, polygon, space in regions:
            if center[2] < bounds[2] - 0.35 or center[2] > bounds[5] + 0.35:
                continue
            if polygon.covers(Point(center[0], center[1])):
                cell.space_id = space.id
                cell.level_id = space.level_id or cell.level_id
                break
        if cell.level_id is None and levels:
            cell.level_id = min(
                levels,
                key=lambda level: abs(level.elevation_m - center[2]),
            ).id


def _entity_level_id(entity, level_by_guid: dict[str, str]) -> str | None:
    queue = [entity]
    visited: set[int] = set()
    while queue:
        current = queue.pop(0)
        try:
            current_id = int(current.id())
        except Exception:
            current_id = id(current)
        if current_id in visited:
            continue
        visited.add(current_id)

        for relation in _safe_attr(current, "ContainedInStructure", ()) or ():
            parent = _safe_attr(relation, "RelatingStructure")
            if parent is None:
                continue
            try:
                if parent.is_a("IfcBuildingStorey"):
                    return level_by_guid.get(str(parent.GlobalId))
            except Exception:
                pass
            queue.append(parent)
        for relation in _safe_attr(current, "Decomposes", ()) or ():
            parent = _safe_attr(relation, "RelatingObject")
            if parent is None:
                continue
            try:
                if parent.is_a("IfcBuildingStorey"):
                    return level_by_guid.get(str(parent.GlobalId))
            except Exception:
                pass
            queue.append(parent)
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
    vertices = [
        (float(verts[index]), float(verts[index + 1]), float(verts[index + 2]))
        for index in range(0, len(verts), 3)
    ]
    triangles = [
        (int(faces[index]), int(faces[index + 1]), int(faces[index + 2]))
        for index in range(0, len(faces), 3)
    ]
    return vertices, triangles


def _bbox(entity):
    mesh = _mesh(entity)
    if mesh is None or not mesh[0]:
        return None
    return _bbox_from_vertices(mesh[0])


def _bbox_from_vertices(vertices):
    xs = [point[0] for point in vertices]
    ys = [point[1] for point in vertices]
    zs = [point[2] for point in vertices]
    return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


def _cell_centroid(cell: NavCell):
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )


def _safe_attr(entity, name, default=None):
    try:
        return getattr(entity, name, default)
    except (RuntimeError, IndexError, AttributeError):
        return default
