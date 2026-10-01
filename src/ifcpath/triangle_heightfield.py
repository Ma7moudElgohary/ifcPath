from __future__ import annotations

import math
import multiprocessing
from collections import defaultdict
from dataclasses import dataclass
from time import perf_counter
from typing import Callable

import ifcopenshell.geom
from shapely.geometry import Point

from .heightfield_repair import span_key
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
    _group_support_candidates,
    _native_geometry_tree,
    _preferred_support_candidate,
    _prune_small_components,
    _span_to_sample,
    _triangulate_samples,
    ifc_walkable_support_role,
)
from .semantic_heightfield_repair import (
    SemanticRepairRegion,
    build_semantic_repair_regions,
    semantic_bridge_paths,
    semantic_region_for_position,
)
from .surface_nav import connect_cells_by_shared_edges


_MAX_REPAIR_EXACT_CHECKS = 128
_MAX_REPAIR_ROUNDS = 3
_MAX_REPAIR_PATH_SPANS = 16


@dataclass(frozen=True, slots=True)
class _RasterHit:
    entity_id: int
    position: Vec3
    normal: Vec3


@dataclass(slots=True)
class _RepairTelemetry:
    candidates: int = 0
    exact_checks: int = 0
    restored: int = 0
    headroom_candidates: int = 0
    headroom_checks: int = 0
    headroom_restored: int = 0
    region_seconds: float = 0.0
    tree_seconds: float = 0.0
    headroom_seconds: float = 0.0
    exact_seconds: float = 0.0
    tree: object | None = None


def detect_ifc_walkable_cells_heightfield(
    ifc_file,
    domains: list[SurfaceSamplingDomain],
    *,
    options: SurfaceDetectionOptions | None = None,
    class_for_entity: Callable[[object], str] | None = None,
) -> tuple[list[NavCell], SurfaceDetectionStats]:
    """Reconstruct walkable NavCells from a triangulated multilayer heightfield.

    IFC geometry is rasterized once into a fast multilayer field. If triangle
    rasterization disputes headroom inside an authored room, a native IFC ray is
    used as the exact tie-breaker. Agent-radius erosion remains conservative; only
    short rejected paths splitting the same room receive exact OCC body-clearance
    verification. IfcSpace therefore prioritizes verification but never creates
    walkable geometry.
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

        repair = _RepairTelemetry()
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
            repair,
        )
        print(
            "[ifcpath-heightfield] "
            f"stage=filter seconds={perf_counter() - stage_started:.3f} "
            f"samples={len(samples)} rejected={stats.clearance_rejections} "
            f"headroom_candidates={repair.headroom_candidates} "
            f"headroom_checks={repair.headroom_checks} "
            f"headroom_restored={repair.headroom_restored} "
            f"repair_candidates={repair.candidates} "
            f"repair_checks={repair.exact_checks} "
            f"repair_restored={repair.restored} "
            f"repair_region_seconds={repair.region_seconds:.3f} "
            f"repair_tree_seconds={repair.tree_seconds:.3f} "
            f"repair_headroom_seconds={repair.headroom_seconds:.3f} "
            f"repair_exact_seconds={repair.exact_seconds:.3f}",
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
    repair: _RepairTelemetry,
):
    _, _, z0, _, _, z1 = domain.bounds
    min_up = math.cos(math.radians(opts.max_slope_deg))
    support_ids = set(domain.support_entity_ids)
    spans_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)

    region_started = perf_counter()
    regions = build_semantic_repair_regions(ifc_file)
    repair.region_seconds += perf_counter() - region_started

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
                if (
                    candidate.terrain == "open"
                    and semantic_region_for_position(regions, candidate.position) is not None
                ):
                    repair.headroom_candidates += 1
                    tree = _ensure_repair_tree(ifc_file, stats, repair)
                    if tree is not None:
                        repair.headroom_checks += 1
                        check_started = perf_counter()
                        exact_blocked = _exact_headroom_blocked(tree, candidate.position, opts)
                        repair.headroom_seconds += perf_counter() - check_started
                        if not exact_blocked:
                            repair.headroom_restored += 1
                        else:
                            stats.headroom_rejections += 1
                            continue
                    else:
                        stats.headroom_rejections += 1
                        continue
                else:
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
        regions,
        spans_by_xy,
        result,
        opts,
        stats,
        classify,
        repair,
    )
    stats.samples += len(result)
    return result


def _repair_field_bottlenecks_exact(
    ifc_file,
    domain: SurfaceSamplingDomain,
    regions: list[SemanticRepairRegion],
    spans_by_xy: dict[tuple[int, int], list[_WalkableSpan]],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    stats: SurfaceDetectionStats,
    classify,
    telemetry: _RepairTelemetry,
) -> list[_SupportSample]:
    if opts.agent_radius_m <= 0.0 or not accepted or not regions:
        return accepted

    accepted_keys = {
        (sample.ix, sample.iy, round(sample.position[2] / max(1e-6, opts.hit_merge_tolerance_m)))
        for sample in accepted
    }
    remaining = [
        span
        for bucket in spans_by_xy.values()
        for span in bucket
        if not span.direct_blocked
        and span_key(span, opts.hit_merge_tolerance_m) not in accepted_keys
    ]
    if not remaining:
        return accepted

    current = list(accepted)
    checked: set[tuple[int, int, int]] = set()
    support_ids = set(domain.support_entity_ids)

    for _ in range(_MAX_REPAIR_ROUNDS):
        unchecked = [
            span
            for span in remaining
            if span_key(span, opts.hit_merge_tolerance_m) not in checked
        ]
        paths = semantic_bridge_paths(
            regions,
            unchecked,
            current,
            opts,
            max_path_spans=_MAX_REPAIR_PATH_SPANS,
        )
        if not paths:
            break

        available_budget = _MAX_REPAIR_EXACT_CHECKS - telemetry.exact_checks
        candidates: list[_WalkableSpan] = []
        candidate_keys: set[tuple[int, int, int]] = set()
        for path in paths:
            for span in path:
                key = span_key(span, opts.hit_merge_tolerance_m)
                if key in checked or key in candidate_keys:
                    continue
                if len(candidates) >= available_budget:
                    break
                candidates.append(span)
                candidate_keys.add(key)
            if len(candidates) >= available_budget:
                break
        if not candidates:
            break
        telemetry.candidates += len(candidates)

        tree = _ensure_repair_tree(ifc_file, stats, telemetry)
        if tree is None:
            break

        exact_started = perf_counter()
        restored_this_round = 0
        for span in candidates:
            key = span_key(span, opts.hit_merge_tolerance_m)
            if key in checked or telemetry.exact_checks >= _MAX_REPAIR_EXACT_CHECKS:
                continue
            checked.add(key)
            telemetry.exact_checks += 1
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
            telemetry.restored += 1
            restored_this_round += 1
            stats.heightfield_radius_rejections = max(0, stats.heightfield_radius_rejections - 1)
            stats.clearance_rejections = max(0, stats.clearance_rejections - 1)
        telemetry.exact_seconds += perf_counter() - exact_started

        if restored_this_round == 0 or telemetry.exact_checks >= _MAX_REPAIR_EXACT_CHECKS:
            break

    return current


def _ensure_repair_tree(ifc_file, stats: SurfaceDetectionStats, telemetry: _RepairTelemetry):
    if telemetry.tree is not None:
        return telemetry.tree
    started = perf_counter()
    try:
        telemetry.tree = _native_geometry_tree(ifc_file)
        stats.used_native_tree = True
    except Exception:
        telemetry.tree = None
    telemetry.tree_seconds += perf_counter() - started
    return telemetry.tree


def _exact_headroom_blocked(tree, position, opts: SurfaceDetectionOptions) -> bool:
    tolerance = max(1e-4, opts.hit_merge_tolerance_m)
    required = max(0.0, opts.agent_height_m)
    if required <= tolerance:
        return False
    z = float(position[2])
    start_z = z + tolerance
    length = max(0.0, required - tolerance)
    try:
        hits = tree.select_ray(
            (float(position[0]), float(position[1]), start_z),
            (0.0, 0.0, 1.0),
            length=length,
        )
    except Exception:
        return True
    for hit in hits:
        try:
            hit_z = float(hit.position[2])
        except Exception:
            continue
        clearance = hit_z - z
        if clearance <= tolerance:
            continue
        if clearance + tolerance < required:
            return True
    return False


def _nearest_distinct_surface_above(index, positions, z, tolerance):
    for upper in reversed(positions[:index]):
        delta = upper[2] - z
        if delta <= tolerance:
            continue
        return float(upper[2])
    return None
