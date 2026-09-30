from __future__ import annotations

import math
import multiprocessing
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable

import ifcopenshell.geom
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from .model import NavCell, Vec3
from .surface_nav import connect_cells_by_shared_edges, surface_components


@dataclass(frozen=True, slots=True)
class SurfaceSamplingDomain:
    """A bounded area in which walkable support surfaces should be discovered.

    ``clip_xy`` is an accelerator/semantic boundary only. The actual walkable Z
    values always come from ray intersections with physical IFC geometry.
    """

    id: str
    bounds: tuple[float, float, float, float, float, float]
    space_id: str | None = None
    level_id: str | None = None
    clip_xy: BaseGeometry | None = None


@dataclass(frozen=True, slots=True)
class SurfaceDetectionOptions:
    cell_size_m: float = 0.15
    agent_height_m: float = 1.8
    agent_radius_m: float = 0.22
    max_slope_deg: float = 50.0
    max_climb_m: float = 0.24
    vertical_padding_m: float = 0.35
    hit_merge_tolerance_m: float = 0.025
    minimum_component_cells: int = 2
    floor_seed_tolerance_m: float = 0.35


@dataclass(slots=True)
class SurfaceDetectionStats:
    domains: int = 0
    rays: int = 0
    candidate_hits: int = 0
    clearance_rejections: int = 0
    headroom_rejections: int = 0
    samples: int = 0
    cells_before_pruning: int = 0
    cells_after_pruning: int = 0
    used_native_tree: bool = False
    error: str | None = None


@dataclass(frozen=True, slots=True)
class _SupportSample:
    ix: int
    iy: int
    position: Vec3
    owner_id: int
    terrain: str


_NON_BLOCKING_CLASSES = {
    "IfcDoor",
    "IfcOpeningElement",
    "IfcWindow",
    "IfcSpace",
}


def detect_ifc_walkable_cells(
    ifc_file,
    domains: list[SurfaceSamplingDomain],
    *,
    options: SurfaceDetectionOptions | None = None,
    class_for_entity: Callable[[object], str] | None = None,
) -> tuple[list[NavCell], SurfaceDetectionStats]:
    """Detect a continuous human-walkable support surface from physical IFC geometry.

    The detector follows the same physical ideas used by Recast-style navmesh
    builders, but it queries IfcOpenShell's exact native geometry tree instead of
    first converting the building to a voxel solid:

    * cast vertical rays through the building and keep upward support faces;
    * reject faces that do not have enough head room;
    * use exact geometry/sphere selection as a body-clearance test;
    * connect neighbouring samples only when the vertical step is traversable;
    * discard isolated raised surfaces such as desks by retaining components that
      begin near the lowest support surface in each semantic sampling domain;
    * triangulate the surviving multi-layer sample field into NavCells.

    IFC semantics label and bound the result; they do not manufacture the surface.
    """
    opts = options or SurfaceDetectionOptions()
    stats = SurfaceDetectionStats(domains=len(domains))
    if not domains:
        return [], stats

    try:
        tree = _native_geometry_tree(ifc_file)
        stats.used_native_tree = True
    except Exception as exc:  # pragma: no cover - exercised by real-IFC qualification
        stats.error = f"native geometry tree unavailable: {exc}"
        return [], stats

    classify = class_for_entity or _entity_class
    all_cells: list[NavCell] = []
    seen_cell_geometry: set[tuple] = set()

    for domain in domains:
        samples = _sample_domain(ifc_file, tree, domain, opts, stats, classify)
        if not samples:
            continue
        domain_cells = _triangulate_samples(domain, samples, opts)
        stats.cells_before_pruning += len(domain_cells)
        domain_cells = _retain_floor_connected_components(domain_cells, opts)
        stats.cells_after_pruning += len(domain_cells)

        for cell in domain_cells:
            geometry_key = _cell_geometry_key(cell)
            if geometry_key in seen_cell_geometry:
                continue
            seen_cell_geometry.add(geometry_key)
            all_cells.append(cell)

    connect_cells_by_shared_edges(all_cells, tolerance_m=max(1e-5, opts.hit_merge_tolerance_m * 0.2))
    return all_cells, stats


def _native_geometry_tree(ifc_file):
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    iterator = ifcopenshell.geom.iterator(
        settings,
        ifc_file,
        max(1, min(multiprocessing.cpu_count(), 8)),
    )
    tree = ifcopenshell.geom.tree()
    if not iterator.initialize():
        return tree
    while True:
        tree.add_element(iterator.get_native())
        if not iterator.next():
            break
    return tree


def _sample_domain(ifc_file, tree, domain, opts, stats, classify):
    x0, y0, z0, x1, y1, z1 = domain.bounds
    cell = max(0.05, float(opts.cell_size_m))
    ix0 = math.floor(x0 / cell)
    iy0 = math.floor(y0 / cell)
    ix1 = math.ceil(x1 / cell)
    iy1 = math.ceil(y1 / cell)
    ray_top = z1 + opts.vertical_padding_m
    ray_bottom = z0 - opts.vertical_padding_m
    ray_length = max(cell, ray_top - ray_bottom)
    min_up = math.cos(math.radians(opts.max_slope_deg))

    samples_by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for ix in range(ix0, ix1 + 1):
        x = ix * cell
        for iy in range(iy0, iy1 + 1):
            y = iy * cell
            if domain.clip_xy is not None and not domain.clip_xy.covers(Point(x, y)):
                continue

            stats.rays += 1
            hits = list(tree.select_ray((x, y, ray_top), (0.0, 0.0, -1.0), length=ray_length))
            if not hits:
                continue
            hits.sort(key=lambda hit: float(hit.distance))
            positions = [tuple(float(value) for value in hit.position) for hit in hits]

            for index, hit in enumerate(hits):
                position = positions[index]
                if position[2] < z0 - opts.vertical_padding_m or position[2] > z1 + opts.vertical_padding_m:
                    continue
                normal = tuple(float(value) for value in hit.normal)
                if normal[2] < min_up:
                    continue

                entity = _hit_entity(ifc_file, hit)
                if entity is None or not _is_physical_element(entity):
                    continue
                stats.candidate_hits += 1

                if _headroom_blocked(index, positions, position[2], opts.agent_height_m):
                    stats.headroom_rejections += 1
                    continue
                if _body_clearance_blocked(tree, ifc_file, entity, position, opts, classify):
                    stats.clearance_rejections += 1
                    continue

                owner_id = int(entity.id())
                sample = _SupportSample(
                    ix=ix,
                    iy=iy,
                    position=(x, y, position[2]),
                    owner_id=owner_id,
                    terrain=_terrain_for_class(classify(entity)),
                )
                _append_unique_sample(samples_by_xy[(ix, iy)], sample, opts.hit_merge_tolerance_m)

    result = [sample for bucket in samples_by_xy.values() for sample in bucket]
    stats.samples += len(result)
    return result


def _headroom_blocked(index, positions, z, required_height):
    # Rays travel downwards, therefore earlier intersections are geometrically
    # above this support. The nearest distinct intersection bounds head room.
    for upper in reversed(positions[:index]):
        delta = upper[2] - z
        if delta <= 0.02:
            continue
        return delta < required_height
    return False


def _body_clearance_blocked(tree, ifc_file, support_entity, position, opts, classify):
    if opts.agent_radius_m <= 0.0:
        return False
    torso_z = position[2] + min(max(opts.agent_height_m * 0.5, 0.45), 1.0)
    try:
        selected = tree.select((position[0], position[1], torso_z), extend=opts.agent_radius_m)
    except Exception:
        return False

    support_id = int(support_entity.id())
    for raw in selected:
        entity = _selected_entity(ifc_file, raw)
        if entity is None:
            continue
        try:
            entity_id = int(entity.id())
        except Exception:
            continue
        if entity_id == support_id:
            continue
        if classify(entity) in _NON_BLOCKING_CLASSES:
            continue
        if not _is_physical_element(entity):
            continue
        return True
    return False


def _triangulate_samples(domain, samples, opts):
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)
    for bucket in by_xy.values():
        bucket.sort(key=lambda sample: sample.position[2])

    cells: list[NavCell] = []
    created: set[tuple] = set()
    for (ix, iy), origins in sorted(by_xy.items()):
        east = by_xy.get((ix + 1, iy), ())
        north = by_xy.get((ix, iy + 1), ())
        diagonal = by_xy.get((ix + 1, iy + 1), ())
        if not east or not north or not diagonal:
            continue

        for origin in origins:
            b = _nearest_compatible(origin, east, opts)
            c = _nearest_compatible(origin, north, opts)
            if b is None or c is None:
                continue
            d_candidates = [
                candidate
                for candidate in diagonal
                if _samples_compatible(b, candidate, opts)
                and _samples_compatible(c, candidate, opts)
                and _samples_compatible(origin, candidate, opts, diagonal=True)
            ]
            if not d_candidates:
                continue
            d = min(d_candidates, key=lambda sample: (abs(sample.position[2] - origin.position[2]), sample.owner_id))

            for vertices, terrain in (
                ((origin.position, b.position, d.position), _majority_terrain(origin, b, d)),
                ((origin.position, d.position, c.position), _majority_terrain(origin, d, c)),
            ):
                key = _triangle_key(vertices)
                if key in created or _triangle_area_3d(*vertices) <= 1e-8:
                    continue
                created.add(key)
                cells.append(
                    NavCell(
                        id=f"cell:detected:{domain.id}:{len(cells)}",
                        vertices_m=vertices,
                        space_id=domain.space_id,
                        level_id=domain.level_id,
                        terrain=terrain,
                    )
                )

    connect_cells_by_shared_edges(cells, tolerance_m=max(1e-5, opts.hit_merge_tolerance_m * 0.2))
    return cells


def _retain_floor_connected_components(cells, opts):
    if not cells:
        return []
    components = surface_components(cells)
    by_id = {cell.id: cell for cell in cells}
    component_info = []
    for component in components:
        members = [by_id[cell_id] for cell_id in component]
        min_z = min(vertex[2] for cell in members for vertex in cell.vertices_m)
        component_info.append((min_z, members))

    global_min = min(info[0] for info in component_info)
    kept = []
    for min_z, members in component_info:
        if len(members) < opts.minimum_component_cells:
            continue
        if min_z <= global_min + opts.floor_seed_tolerance_m:
            kept.extend(members)
    return kept


def _nearest_compatible(source, candidates, opts):
    valid = [candidate for candidate in candidates if _samples_compatible(source, candidate, opts)]
    if not valid:
        return None
    return min(valid, key=lambda sample: (abs(sample.position[2] - source.position[2]), sample.owner_id))


def _samples_compatible(a, b, opts, *, diagonal=False):
    horizontal = opts.cell_size_m * (math.sqrt(2.0) if diagonal else 1.0)
    dz = abs(a.position[2] - b.position[2])
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    return dz <= max(opts.max_climb_m, slope_limit) + opts.hit_merge_tolerance_m


def _append_unique_sample(bucket, sample, tolerance):
    for existing in bucket:
        if abs(existing.position[2] - sample.position[2]) <= tolerance:
            return
    bucket.append(sample)


def _hit_entity(ifc_file, hit):
    instance = getattr(hit, "instance", None)
    if instance is None:
        return None
    try:
        return ifc_file.by_id(int(instance.id()))
    except Exception:
        return None


def _selected_entity(ifc_file, raw):
    instance = getattr(raw, "instance", raw)
    try:
        return ifc_file.by_id(int(instance.id()))
    except Exception:
        return None


def _is_physical_element(entity):
    try:
        return bool(entity.is_a("IfcElement"))
    except Exception:
        return False


def _entity_class(entity):
    try:
        return str(entity.is_a())
    except Exception:
        return ""


def _terrain_for_class(class_name):
    value = class_name.upper()
    if "STAIR" in value or "ESCALATOR" in value:
        return "stair"
    if "RAMP" in value:
        return "ramp"
    return "open"


def _majority_terrain(*samples):
    counts: dict[str, int] = defaultdict(int)
    for sample in samples:
        counts[sample.terrain] += 1
    return max(counts, key=lambda terrain: (counts[terrain], terrain != "open", terrain))


def _triangle_key(vertices):
    quantized = sorted(
        tuple(round(value * 100_000.0) for value in vertex)
        for vertex in vertices
    )
    return tuple(quantized)


def _cell_geometry_key(cell):
    return _triangle_key(cell.vertices_m)


def _triangle_area_3d(a, b, c):
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))
