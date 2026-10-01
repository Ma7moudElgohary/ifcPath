from __future__ import annotations

import math
import multiprocessing
from collections import defaultdict
from dataclasses import dataclass
from time import perf_counter
from typing import Callable

import ifcopenshell.geom
from shapely.geometry import Point

from .model import NavCell, Vec3
from .raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceDetectionStats,
    SurfaceSamplingDomain,
    _SupportCandidate,
    _WalkableSpan,
    _append_unique_span,
    _cell_geometry_key,
    _column_body_blocked,
    _entity_class,
    _erode_walkable_spans,
    _group_support_candidates,
    _preferred_support_candidate,
    _prune_small_components,
    _triangulate_samples,
    ifc_walkable_support_role,
)
from .surface_nav import connect_cells_by_shared_edges


@dataclass(frozen=True, slots=True)
class _RasterHit:
    entity_id: int
    position: Vec3
    normal: Vec3


def detect_ifc_walkable_cells_heightfield(
    ifc_file,
    domains: list[SurfaceSamplingDomain],
    *,
    options: SurfaceDetectionOptions | None = None,
    class_for_entity: Callable[[object], str] | None = None,
) -> tuple[list[NavCell], SurfaceDetectionStats]:
    """Reconstruct walkable NavCells from a triangulated multilayer heightfield.

    This is the Recast-style reconstruction path. IFC geometry is interpreted in
    one cached/multicore iterator, rasterized into vertical XY columns, filtered
    for support/slope/headroom/body occupancy, eroded for agent radius per height
    layer, and only then triangulated into IfcPath's continuous NavCells.

    The field is reconstruction machinery, never the routing graph. IfcSpace is
    deliberately absent here and is applied later only as semantic labelling.
    """
    opts = options or SurfaceDetectionOptions()
    stats = SurfaceDetectionStats(domains=len(domains))
    if not domains:
        return [], stats

    classify = class_for_entity or _entity_class
    all_cells: list[NavCell] = []
    seen_cell_geometry: set[tuple] = set()

    ordered_domains = sorted(domains, key=lambda domain: (domain.terrain == "open", domain.id))
    for domain in ordered_domains:
        stage_started = perf_counter()
        sample_keys = _sample_keys(domain, opts)
        if not sample_keys:
            continue
        columns, entity_cache, triangle_count = _rasterize_columns(
            ifc_file,
            domain,
            sample_keys,
            opts,
        )
        print(
            "[ifcpath-heightfield] "
            f"stage=rasterize seconds={perf_counter() - stage_started:.3f} "
            f"sample_columns={len(sample_keys)} populated_columns={len(columns)} "
            f"triangles={triangle_count}",
            flush=True,
        )

        stage_started = perf_counter()
        samples = _spans_from_columns(
            columns,
            entity_cache,
            domain,
            sample_keys,
            opts,
            stats,
            classify,
        )
        print(
            "[ifcpath-heightfield] "
            f"stage=filter seconds={perf_counter() - stage_started:.3f} "
            f"samples={len(samples)} rejected={stats.clearance_rejections}",
            flush=True,
        )
        if not samples:
            continue

        domain_cells = _triangulate_samples(domain, samples, opts)
        stats.cells_before_pruning += len(domain_cells)
        domain_cells = _prune_small_components(domain_cells, opts)
        stats.cells_after_pruning += len(domain_cells)

        for cell in domain_cells:
            geometry_key = _cell_geometry_key(cell, opts.hit_merge_tolerance_m)
            if geometry_key in seen_cell_geometry:
                continue
            seen_cell_geometry.add(geometry_key)
            all_cells.append(cell)

    connect_cells_by_shared_edges(
        all_cells,
        tolerance_m=max(1e-5, opts.hit_merge_tolerance_m * 0.2),
    )
    return all_cells, stats


def _sample_keys(domain: SurfaceSamplingDomain, opts: SurfaceDetectionOptions) -> set[tuple[int, int]]:
    x0, y0, _, x1, y1, _ = domain.bounds
    cell = max(0.05, float(opts.cell_size_m))
    ix0 = math.floor(x0 / cell)
    iy0 = math.floor(y0 / cell)
    ix1 = math.ceil(x1 / cell)
    iy1 = math.ceil(y1 / cell)

    keys: set[tuple[int, int]] = set()
    for ix in range(ix0, ix1 + 1):
        x = ix * cell
        for iy in range(iy0, iy1 + 1):
            y = iy * cell
            if domain.clip_xy is not None and not domain.clip_xy.covers(Point(x, y)):
                continue
            keys.add((ix, iy))
    return keys


def _rasterize_columns(
    ifc_file,
    domain: SurfaceSamplingDomain,
    sample_keys: set[tuple[int, int]],
    opts: SurfaceDetectionOptions,
) -> tuple[dict[tuple[int, int], list[_RasterHit]], dict[int, object], int]:
    """Rasterize triangle surfaces at vertical grid-column centres.

    Horizontal and sloped triangle intersections provide exact interpolated Z
    values. A perfectly vertical triangle has zero projected XY area and is not
    intersected by a mathematical vertical ray except for a measure-zero edge
    coincidence, so it must not manufacture a broad column obstacle. Closed
    solids still contribute their horizontal/sloped cap intersections exactly as
    the previous geometry-tree vertical-ray detector did.
    """
    try:
        physical_elements = list(ifc_file.by_type("IfcElement"))
    except Exception:
        physical_elements = []

    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    iterator = ifcopenshell.geom.iterator(
        settings,
        ifc_file,
        max(1, min(multiprocessing.cpu_count(), 8)),
        include=physical_elements,
    )

    columns: dict[tuple[int, int], list[_RasterHit]] = defaultdict(list)
    entity_cache: dict[int, object] = {}
    triangle_count = 0
    cell = max(0.05, float(opts.cell_size_m))
    ix_values = [key[0] for key in sample_keys]
    iy_values = [key[1] for key in sample_keys]
    grid_bounds = (min(ix_values), min(iy_values), max(ix_values), max(iy_values))

    if not iterator.initialize():
        return columns, entity_cache, triangle_count

    while True:
        shape = iterator.get()
        entity_id = _shape_entity_id(shape)
        if entity_id is not None:
            try:
                entity = ifc_file.by_id(entity_id)
            except Exception:
                entity = None
            if entity is not None:
                entity_cache[entity_id] = entity
                vertices, faces = _shape_mesh(shape)
                for face in faces:
                    try:
                        a = vertices[face[0]]
                        b = vertices[face[1]]
                        c = vertices[face[2]]
                    except (IndexError, TypeError):
                        continue
                    triangle_count += 1
                    _rasterize_triangle(
                        columns,
                        sample_keys,
                        grid_bounds,
                        entity_id,
                        a,
                        b,
                        c,
                        cell,
                    )
        if not iterator.next():
            break

    return columns, entity_cache, triangle_count


def _shape_entity_id(shape) -> int | None:
    value = getattr(shape, "id", None)
    try:
        return int(value() if callable(value) else value)
    except (TypeError, ValueError):
        return None


def _shape_mesh(shape) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    try:
        verts = shape.geometry.verts
        faces = shape.geometry.faces
    except Exception:
        return [], []
    vertices = [
        (float(verts[index]), float(verts[index + 1]), float(verts[index + 2]))
        for index in range(0, len(verts), 3)
    ]
    triangles = [
        (int(faces[index]), int(faces[index + 1]), int(faces[index + 2]))
        for index in range(0, len(faces), 3)
    ]
    return vertices, triangles


def _rasterize_triangle(
    columns,
    sample_keys,
    grid_bounds,
    entity_id,
    a,
    b,
    c,
    cell,
):
    normal = _triangle_normal(a, b, c)
    if normal is None:
        return

    projected_area2 = _projected_area2(a, b, c)
    if abs(projected_area2) <= 1e-10:
        return

    min_x = min(a[0], b[0], c[0])
    max_x = max(a[0], b[0], c[0])
    min_y = min(a[1], b[1], c[1])
    max_y = max(a[1], b[1], c[1])
    ix0 = max(grid_bounds[0], math.floor(min_x / cell) - 1)
    iy0 = max(grid_bounds[1], math.floor(min_y / cell) - 1)
    ix1 = min(grid_bounds[2], math.ceil(max_x / cell) + 1)
    iy1 = min(grid_bounds[3], math.ceil(max_y / cell) + 1)

    for ix in range(ix0, ix1 + 1):
        x = ix * cell
        for iy in range(iy0, iy1 + 1):
            key = (ix, iy)
            if key not in sample_keys:
                continue
            y = iy * cell
            weights = _barycentric_xy(x, y, a, b, c, projected_area2)
            if weights is None:
                continue
            wa, wb, wc = weights
            if min(wa, wb, wc) < -1e-8 or max(wa, wb, wc) > 1.0 + 1e-8:
                continue
            z = wa * a[2] + wb * b[2] + wc * c[2]
            columns[key].append(_RasterHit(entity_id, (x, y, float(z)), normal))


def _projected_area2(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _barycentric_xy(x, y, a, b, c, area2):
    if abs(area2) <= 1e-12:
        return None
    wa = ((b[0] - x) * (c[1] - y) - (b[1] - y) * (c[0] - x)) / area2
    wb = ((c[0] - x) * (a[1] - y) - (c[1] - y) * (a[0] - x)) / area2
    wc = 1.0 - wa - wb
    return wa, wb, wc


def _triangle_normal(a, b, c) -> Vec3 | None:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length <= 1e-12:
        return None
    return tuple(value / length for value in cross)


def _spans_from_columns(
    columns,
    entity_cache,
    domain,
    sample_keys,
    opts,
    stats,
    classify,
):
    _, _, z0, _, _, z1 = domain.bounds
    min_up = math.cos(math.radians(opts.max_slope_deg))
    support_ids = set(domain.support_entity_ids)
    spans_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)

    for ix, iy in sorted(sample_keys):
        stats.rays += 1
        hits = columns.get((ix, iy), ())
        if not hits:
            continue
        ordered = sorted(hits, key=lambda hit: -hit.position[2])
        positions = [hit.position for hit in ordered]
        entities = [entity_cache.get(hit.entity_id) for hit in ordered]

        seen_supports: set[int] = set()
        candidates: list[_SupportCandidate] = []
        for index, hit in enumerate(ordered):
            z = hit.position[2]
            if z < z0 - opts.vertical_padding_m or z > z1 + opts.vertical_padding_m:
                continue
            entity = entity_cache.get(hit.entity_id)
            if entity is None:
                continue
            entity_id = hit.entity_id
            if support_ids and entity_id not in support_ids:
                continue
            role = ifc_walkable_support_role(entity)
            if not support_ids and role is None:
                continue
            if entity_id in seen_supports:
                continue
            if abs(hit.normal[2]) < min_up:
                continue
            seen_supports.add(entity_id)
            stats.candidate_hits += 1
            candidates.append(
                _SupportCandidate(
                    hit_index=index,
                    position=hit.position,
                    owner_id=entity_id,
                    terrain=domain.terrain or role or "open",
                )
            )

        for layer in _group_support_candidates(candidates, opts.hit_merge_tolerance_m):
            stats.candidate_layers += 1
            stats.deduplicated_candidate_hits += max(0, len(layer) - 1)
            candidate = _preferred_support_candidate(layer)
            ceiling_z = _nearest_distinct_surface_above(
                candidate.hit_index,
                positions,
                candidate.position[2],
                opts.hit_merge_tolerance_m,
            )
            free_height = math.inf if ceiling_z is None else ceiling_z - candidate.position[2]
            if free_height + opts.hit_merge_tolerance_m < opts.agent_height_m:
                stats.headroom_rejections += 1
                continue

            ignored_support_ids = support_ids or {item.owner_id for item in layer}
            direct_blocked = _column_body_blocked(
                entities,
                positions,
                ignored_entity_ids=ignored_support_ids,
                support_z=candidate.position[2],
                opts=opts,
                classify=classify,
            )
            if direct_blocked:
                stats.heightfield_direct_rejections += 1
                stats.clearance_rejections += 1

            _append_unique_span(
                spans_by_xy[(ix, iy)],
                _WalkableSpan(
                    ix=ix,
                    iy=iy,
                    position=candidate.position,
                    owner_id=candidate.owner_id,
                    terrain=candidate.terrain,
                    ceiling_z=ceiling_z,
                    free_height_m=free_height,
                    direct_blocked=direct_blocked,
                ),
                opts.hit_merge_tolerance_m,
            )

    stats.heightfield_spans += sum(len(bucket) for bucket in spans_by_xy.values())
    result = _erode_walkable_spans(
        spans_by_xy,
        opts,
        stats,
        sampled_xy=sample_keys,
    )
    stats.samples += len(result)
    return result


def _nearest_distinct_surface_above(index, positions, z, tolerance):
    for upper in reversed(positions[:index]):
        delta = upper[2] - z
        if delta <= tolerance:
            continue
        return float(upper[2])
    return None
