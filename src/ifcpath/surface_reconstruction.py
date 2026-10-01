from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from time import perf_counter

import ifcopenshell
import ifcopenshell.geom
from shapely.geometry import Point, box
from shapely.ops import unary_union

from .cdt import space_floor_polygon
from .model import InavModel, NavCell
from .raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceDetectionStats,
    SurfaceSamplingDomain,
    detect_ifc_walkable_cells,
    ifc_walkable_support_role,
)
from .surface_nav import connect_cells_by_shared_edges, surface_components
from .surface_seams import stitch_clearance_aware_seams


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
        return asdict(self)


def _log_reconstruction_stage(name: str, seconds: float | None = None, **details: object) -> None:
    detail_text = " ".join(f"{key}={value}" for key, value in sorted(details.items()))
    seconds_text = "" if seconds is None else f" seconds={seconds:.3f}"
    suffix = f" {detail_text}" if detail_text else ""
    print(f"[ifcpath-surface] stage={name}{seconds_text}{suffix}", flush=True)


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

    The detector is intentionally geometry-first. Physical IFC support elements
    are scanned as one multi-layer field so coincident floor finishes/slabs and
    vertical circulation do not trigger duplicate ray passes. IFC spaces are
    consulted only after geometry reconstruction to label cells; they never
    manufacture walkable elevation.

    Adjacency is rebuilt *after* semantic labelling. This ordering is critical:
    connecting anonymous cells first and labelling them later leaves stale
    room-to-room edges that bypass doors. Rebuilding after labelling lets the
    normal shared-edge rule reject unauthorised open/open space crossings while
    retaining open/stair/ramp geometric continuity.
    """
    stats = SurfaceReconstructionStats()

    stage_started = perf_counter()
    try:
        ifc_file = ifcopenshell.open(str(ifc_path))
    except Exception as exc:
        stats.fallback_reason = f"cannot reopen IFC for physical surface detection: {exc}"
        return stats
    _log_reconstruction_stage("open_ifc", perf_counter() - stage_started)

    detection_options = SurfaceDetectionOptions(
        cell_size_m=max(0.05, float(cell_size_m)),
        agent_height_m=max(0.0, float(agent_height_m)),
        agent_radius_m=max(0.0, float(agent_radius_m)),
        max_slope_deg=max(0.0, min(89.0, float(max_slope_deg))),
        max_climb_m=max(0.0, float(max_climb_m)),
    )

    stage_started = perf_counter()
    domains = _support_domains(
        ifc_file,
        model,
        padding_m=detection_options.cell_size_m * 0.75,
    )
    stats.support_elements = sum(len(domain.support_entity_ids) for domain in domains)
    _log_reconstruction_stage(
        "support_domains",
        perf_counter() - stage_started,
        domains=len(domains),
        supports=stats.support_elements,
    )
    if not domains:
        stats.fallback_reason = "no physical walkable-support IFC elements found"
        return stats

    _log_reconstruction_stage(
        "detector_start",
        cell_size_m=detection_options.cell_size_m,
        radius_m=detection_options.agent_radius_m,
    )
    stage_started = perf_counter()
    cells, detector_stats = detect_ifc_walkable_cells(
        ifc_file,
        domains,
        options=detection_options,
    )
    _log_reconstruction_stage(
        "detector",
        perf_counter() - stage_started,
        cells=len(cells),
        rays=detector_stats.rays,
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

    stage_started = perf_counter()
    _label_cells_from_ifc_spaces(ifc_file, model, cells)
    stats.labelled_cells = sum(cell.space_id is not None for cell in cells)
    stats.unlabelled_cells = len(cells) - stats.labelled_cells
    _log_reconstruction_stage(
        "space_labelling",
        perf_counter() - stage_started,
        labelled=stats.labelled_cells,
        unlabelled=stats.unlabelled_cells,
    )

    # The ray detector connects anonymous geometry so it can prune tiny physical
    # patches. Those links are not semantically authoritative. Clear them now and
    # rebuild against the final space labels so only same-space open surfaces or
    # physical vertical terrain remain connected. Doors/open boundaries are
    # authorised later by the portable semantic finalisation pass.
    stage_started = perf_counter()
    _clear_surface_adjacency(cells)
    connect_cells_by_shared_edges(
        cells,
        tolerance_m=max(1e-5, detection_options.hit_merge_tolerance_m * 0.2),
    )

    # A regular sample grid can end up to one cell inside each physical support
    # boundary, so two genuinely touching independently reconstructed supports
    # can be separated by almost 2*cell_size in the triangulated result. The seam
    # allowance therefore follows that geometric sampling bound. Body-clearance
    # carving is larger around walls/railings, so this does not re-authorise a
    # normal obstacle gap. Crucially, the unified open-floor field is never seam-
    # healed: a missing flat sample can represent furniture or another obstacle.
    sampling_gap_m = (
        2.0 * detection_options.cell_size_m
        + detection_options.hit_merge_tolerance_m
    )
    stats.seam_count = stitch_clearance_aware_seams(
        cells,
        max_gap_m=max(0.05, sampling_gap_m),
        max_vertical_gap_m=max(
            detection_options.max_climb_m + detection_options.hit_merge_tolerance_m,
            0.10,
        ),
        # Collision sampling has already kept the centreline one body radius from
        # nearby geometry. A half-radius seam trim prevents a modelling gap from
        # reintroducing a railing-edge crossing without over-shrinking narrow stairs.
        edge_clearance_m=detection_options.agent_radius_m * 0.5,
        vertical_only=True,
    )
    components = surface_components(cells)
    stats.component_count = len(components)
    _log_reconstruction_stage(
        "topology_and_seams",
        perf_counter() - stage_started,
        components=stats.component_count,
        seams=stats.seam_count,
    )

    # If spaces exist but the geometry-derived surface cannot be semantically
    # associated at all, keep the known qualified surface rather than publishing
    # a geometry-only model that would break door binding. Space-free IFCs remain
    # valid geometry probes and may intentionally have no labels.
    if model.spaces and stats.labelled_cells == 0:
        stats.fallback_reason = "detected surface could not be associated with any IFC space"
        return stats

    legacy_cell_count = len(model.cells)
    model.cells = cells
    stats.replaced_legacy_surface = True
    model.metadata.update(
        {
            "surface_source": "ifc-physical-raycast",
            "surface_reconstruction": stats.to_dict(),
            "surface_detector_cell_size_m": detection_options.cell_size_m,
            "surface_detector_agent_radius_m": detection_options.agent_radius_m,
            "surface_detector_agent_height_m": detection_options.agent_height_m,
            "surface_detector_max_climb_m": detection_options.max_climb_m,
            "surface_detector_max_slope_deg": detection_options.max_slope_deg,
            "legacy_cell_count_before_reconstruction": legacy_cell_count,
            "cell_count": len(cells),
        }
    )
    return stats


def _support_domains(
    ifc_file,
    model: InavModel,
    *,
    padding_m: float = 0.0,
) -> list[SurfaceSamplingDomain]:
    """Build one global multi-layer support domain from all physical supports.

    A building can contain several coincident support representations (structural
    slab + finish) and many storeys at the same XY. Scanning per element repeats
    the same expensive geometry-tree ray. A single domain casts each XY ray once
    through the complete vertical extent and retains every compatible support hit,
    which is the multi-layer-heightfield behaviour needed for indoor navigation.

    ``clip_xy`` is the union of buffered support footprints, preventing the global
    bounding box from turning into a huge rectangular sampling workload.
    """
    del model  # semantic levels are assigned after physical reconstruction
    support_ids: set[int] = set()
    support_bounds: list[tuple[float, float, float, float, float, float]] = []
    footprints = []
    try:
        elements = list(ifc_file.by_type("IfcElement"))
    except Exception:
        elements = []

    for entity in elements:
        if ifc_walkable_support_role(entity) is None:
            continue
        bounds = _bbox(entity)
        if bounds is None:
            continue
        try:
            support_ids.add(int(entity.id()))
        except Exception:
            continue
        support_bounds.append(bounds)
        footprint = box(bounds[0], bounds[1], bounds[3], bounds[4])
        if padding_m > 0.0:
            footprint = footprint.buffer(padding_m, cap_style="square", join_style="mitre")
        footprints.append(footprint)

    if not support_bounds or not support_ids:
        return []

    bounds = (
        min(item[0] for item in support_bounds),
        min(item[1] for item in support_bounds),
        min(item[2] for item in support_bounds),
        max(item[3] for item in support_bounds),
        max(item[4] for item in support_bounds),
        max(item[5] for item in support_bounds),
    )
    clip_xy = unary_union(footprints) if footprints else None
    if clip_xy is not None and clip_xy.is_empty:
        clip_xy = None

    return [
        SurfaceSamplingDomain(
            id="support:multilayer",
            bounds=bounds,
            support_entity_ids=frozenset(support_ids),
            terrain="",  # classify every ray hit from its actual IFC support type
            clip_xy=clip_xy,
        )
    ]


def _clear_surface_adjacency(cells: list[NavCell]) -> None:
    for cell in cells:
        cell.neighbor_ids.clear()
        cell.portals.clear()
        cell.portal_ids.clear()


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
