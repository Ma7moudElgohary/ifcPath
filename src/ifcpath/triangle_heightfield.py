from __future__ import annotations

import math
import multiprocessing
from collections import defaultdict, deque
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
    _SupportSample,
    _WalkableSpan,
    _append_unique_span,
    _body_clearance_blocked,
    _cell_geometry_key,
    _column_body_blocked,
    _entity_class,
    _erode_walkable_spans,
    _field_spans_compatible,
    _group_support_candidates,
    _native_geometry_tree,
    _preferred_support_candidate,
    _prune_small_components,
    _span_to_sample,
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
            ifc_file,
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
            f"samples={len(samples)} rejected={stats.clearance_rejections} "
            f"repair_candidates={stats.topology_repair_candidates} "
            f"repair_restored={stats.topology_repair_restored}",
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
    ifc_file,
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
    result = _repair_field_bottlenecks_exact(
        ifc_file,
        domain,
        spans_by_xy,
        result,
        opts,
        stats,
        classify,
    )
    stats.samples += len(result)
    return result


def _repair_field_bottlenecks_exact(
    ifc_file,
    domain: SurfaceSamplingDomain,
    spans_by_xy: dict[tuple[int, int], list[_WalkableSpan]],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    stats: SurfaceDetectionStats,
    classify,
) -> list[_SupportSample]:
    """Restore only field-rejected samples that are topological bottlenecks.

    Metric heightfield erosion is deliberately conservative, but a coarse field
    can reject a small number of otherwise exact-clear samples at a narrow neck.
    The old all-OCC path proved Duplex topology with ~15k precise queries. Rather
    than paying that cost everywhere, build components from the fast accepted
    field and exact-check only rejected spans adjacent to at least two different
    components. A restored span must therefore both matter to connectivity and
    pass the previous exact pedestrian-body clearance test.
    """
    if opts.agent_radius_m <= 0.0 or not accepted:
        return accepted

    viable_spans = [
        span
        for bucket in spans_by_xy.values()
        for span in bucket
        if not span.direct_blocked
    ]
    accepted_keys = {_sample_key(sample, opts.hit_merge_tolerance_m) for sample in accepted}
    rejected = [
        span
        for span in viable_spans
        if _span_key(span, opts.hit_merge_tolerance_m) not in accepted_keys
    ]
    if not rejected:
        return accepted

    current = list(accepted)
    tree = None
    support_ids = set(domain.support_entity_ids)

    # A two-cell raster gap needs two restoration rounds: after the first exact-
    # clear bottleneck is restored it may make the second one component-critical.
    # Bound the iterations by the number of rejected spans; normally Duplex needs
    # only a tiny handful.
    remaining = list(rejected)
    while remaining:
        components = _sample_component_ids(current, opts)
        candidates = [
            span
            for span in remaining
            if _touches_distinct_components(span, current, components, opts)
        ]
        if not candidates:
            break

        stats.topology_repair_candidates += len(candidates)
        if tree is None:
            try:
                tree = _native_geometry_tree(ifc_file)
                stats.used_native_tree = True
            except Exception:
                break

        restored_this_round = 0
        candidate_keys = {_span_key(span, opts.hit_merge_tolerance_m) for span in candidates}
        next_remaining: list[_WalkableSpan] = []
        for span in remaining:
            span_key = _span_key(span, opts.hit_merge_tolerance_m)
            if span_key not in candidate_keys:
                next_remaining.append(span)
                continue

            stats.topology_repair_exact_checks += 1
            ignored_support_ids = support_ids or {span.owner_id}
            blocked = _body_clearance_blocked(
                tree,
                ifc_file,
                ignored_entity_ids=ignored_support_ids,
                position=span.position,
                opts=opts,
                classify=classify,
                stats=stats,
            )
            if blocked:
                continue

            current.append(_span_to_sample(span))
            stats.topology_repair_restored += 1
            stats.heightfield_radius_rejections = max(0, stats.heightfield_radius_rejections - 1)
            stats.clearance_rejections = max(0, stats.clearance_rejections - 1)
            restored_this_round += 1

        remaining = next_remaining
        if restored_this_round == 0:
            break

    return current


def _sample_key(sample: _SupportSample, tolerance: float) -> tuple[int, int, int]:
    scale = 1.0 / max(1e-6, tolerance)
    return sample.ix, sample.iy, round(sample.position[2] * scale)


def _span_key(span: _WalkableSpan, tolerance: float) -> tuple[int, int, int]:
    scale = 1.0 / max(1e-6, tolerance)
    return span.ix, span.iy, round(span.position[2] * scale)


def _sample_component_ids(samples: list[_SupportSample], opts: SurfaceDetectionOptions) -> dict[tuple[int, int, int], int]:
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)

    component_ids: dict[tuple[int, int, int], int] = {}
    component = 0
    for sample in samples:
        key = _sample_key(sample, opts.hit_merge_tolerance_m)
        if key in component_ids:
            continue
        component_ids[key] = component
        queue = deque([sample])
        while queue:
            current = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                for neighbor in by_xy.get((current.ix + dx, current.iy + dy), ()):
                    neighbor_key = _sample_key(neighbor, opts.hit_merge_tolerance_m)
                    if neighbor_key in component_ids:
                        continue
                    if not _sample_layers_compatible(current, neighbor, opts, dx, dy):
                        continue
                    component_ids[neighbor_key] = component
                    queue.append(neighbor)
        component += 1
    return component_ids


def _touches_distinct_components(
    span: _WalkableSpan,
    samples: list[_SupportSample],
    component_ids: dict[tuple[int, int, int], int],
    opts: SurfaceDetectionOptions,
) -> bool:
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)

    touched: set[int] = set()
    span_sample = _span_to_sample(span)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        for neighbor in by_xy.get((span.ix + dx, span.iy + dy), ()):
            if not _sample_layers_compatible(span_sample, neighbor, opts, dx, dy):
                continue
            component_id = component_ids.get(_sample_key(neighbor, opts.hit_merge_tolerance_m))
            if component_id is not None:
                touched.add(component_id)
                if len(touched) >= 2:
                    return True
    return False


def _sample_layers_compatible(a: _SupportSample, b: _SupportSample, opts: SurfaceDetectionOptions, dx: int, dy: int) -> bool:
    a_span = _WalkableSpan(a.ix, a.iy, a.position, a.owner_id, a.terrain, None, math.inf, False)
    b_span = _WalkableSpan(b.ix, b.iy, b.position, b.owner_id, b.terrain, None, math.inf, False)
    return _field_spans_compatible(a_span, b_span, opts, dx, dy)


def _nearest_distinct_surface_above(index, positions, z, tolerance):
    for upper in reversed(positions[:index]):
        delta = upper[2] - z
        if delta <= tolerance:
            continue
        return float(upper[2])
    return None
