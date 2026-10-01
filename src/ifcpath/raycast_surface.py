from __future__ import annotations

import math
import multiprocessing
from collections import defaultdict
from dataclasses import dataclass
from typing import Callable

import ifcopenshell.geom
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from .model import NavCell, Vec3
from .surface_nav import connect_cells_by_shared_edges, surface_components


@dataclass(frozen=True, slots=True)
class SurfaceSamplingDomain:
    """A bounded physical support region to scan for walkable faces.

    The domain identifies IFC elements that are allowed to *support* a person.
    All other physical elements remain available to the geometry tree as
    obstructions. ``clip_xy`` is optional acceleration/semantic geometry only;
    the actual elevation always comes from ray intersections with IFC geometry.
    """

    id: str
    bounds: tuple[float, float, float, float, float, float]
    support_entity_ids: frozenset[int] = frozenset()
    space_id: str | None = None
    level_id: str | None = None
    terrain: str = "open"
    clip_xy: BaseGeometry | None = None


@dataclass(frozen=True, slots=True)
class SurfaceDetectionOptions:
    # 0.15 m is deliberately comparable to detailed BIM navigation literature
    # and captures ordinary stair treads without turning NavCells into a coarse
    # sampled graph. The output remains a continuous triangle surface.
    cell_size_m: float = 0.15
    agent_height_m: float = 1.8
    agent_radius_m: float = 0.22
    max_slope_deg: float = 50.0
    max_climb_m: float = 0.24
    vertical_padding_m: float = 0.35
    hit_merge_tolerance_m: float = 0.025
    minimum_component_cells: int = 2
    minimum_component_area_m2: float = 0.08
    # Diagnostic escape hatch only. The normal path uses the Recast-style span
    # field and does not perform thousands of expensive OCC sphere selections.
    exact_clearance_verification: bool = False


@dataclass(slots=True)
class SurfaceDetectionStats:
    domains: int = 0
    rays: int = 0
    candidate_hits: int = 0
    candidate_layers: int = 0
    deduplicated_candidate_hits: int = 0
    clearance_rejections: int = 0
    clearance_broadphase_queries: int = 0
    clearance_precise_queries: int = 0
    clearance_precise_skips: int = 0
    clearance_verification_mismatches: int = 0
    heightfield_spans: int = 0
    heightfield_direct_rejections: int = 0
    heightfield_radius_rejections: int = 0
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


@dataclass(frozen=True, slots=True)
class _SupportCandidate:
    """One top-face support hit before coincident representations are collapsed."""

    hit_index: int
    position: Vec3
    owner_id: int
    terrain: str


@dataclass(frozen=True, slots=True)
class _WalkableSpan:
    """One physical support layer in a sampled XY column.

    The span field is reconstruction machinery only. Final routing still uses
    continuous triangulated NavCells. ``direct_blocked`` means physical geometry
    occupies the pedestrian body interval at this XY location; neighbouring
    spans then apply metric agent-radius erosion without OCC sphere selections.
    """

    ix: int
    iy: int
    position: Vec3
    owner_id: int
    terrain: str
    ceiling_z: float | None
    free_height_m: float
    direct_blocked: bool = False


_NON_BLOCKING_CLASSES = {
    "IfcDoor",
    "IfcOpeningElement",
    "IfcSpace",
}


def ifc_walkable_support_role(entity) -> str | None:
    """Map IFC semantics to a physical walkable-support role."""
    class_name = _entity_class(entity)
    upper = class_name.upper()
    predefined = str(_safe_attr(entity, "PredefinedType", "") or "").upper()

    if upper in {"IFCSTAIR", "IFCSTAIRFLIGHT"}:
        return "stair"
    if upper in {"IFCRAMP", "IFCRAMPFLIGHT"}:
        return "ramp"
    if upper == "IFCTRANSPORTELEMENT" and predefined == "ESCALATOR":
        return "stair"
    if upper == "IFCTRANSPORTELEMENT" and predefined == "MOVINGWALKWAY":
        return "open"
    if upper == "IFCCOVERING" and predefined == "FLOORING":
        return "open"
    if upper == "IFCSLAB" and predefined != "ROOF":
        return "open"
    return None


def detect_ifc_walkable_cells(
    ifc_file,
    domains: list[SurfaceSamplingDomain],
    *,
    options: SurfaceDetectionOptions | None = None,
    class_for_entity: Callable[[object], str] | None = None,
) -> tuple[list[NavCell], SurfaceDetectionStats]:
    """Detect a human-walkable support manifold from physical IFC geometry.

    Vertical IFC rays build a multilayer span field. Support semantics and slope
    determine candidate walking layers; physical ray intervals determine body
    occupancy and headroom; metric erosion applies agent radius per compatible
    height layer. The surviving field is triangulated into continuous NavCells.
    IFC spaces can label/clip a domain, but they never manufacture walkable Z.
    """
    opts = options or SurfaceDetectionOptions()
    stats = SurfaceDetectionStats(domains=len(domains))
    if not domains:
        return [], stats

    try:
        tree = _native_geometry_tree(ifc_file)
        stats.used_native_tree = True
    except Exception as exc:  # pragma: no cover
        stats.error = f"native geometry tree unavailable: {exc}"
        return [], stats

    classify = class_for_entity or _entity_class
    all_cells: list[NavCell] = []
    seen_cell_geometry: set[tuple] = set()

    ordered_domains = sorted(domains, key=lambda domain: (domain.terrain == "open", domain.id))
    for domain in ordered_domains:
        samples = _sample_domain(ifc_file, tree, domain, opts, stats, classify)
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


def _native_geometry_tree(ifc_file):
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    try:
        physical_elements = list(ifc_file.by_type("IfcElement"))
    except Exception:
        physical_elements = []

    kwargs = {"include": physical_elements} if physical_elements else {}
    iterator = ifcopenshell.geom.iterator(
        settings,
        ifc_file,
        max(1, min(multiprocessing.cpu_count(), 8)),
        **kwargs,
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
    support_ids = set(domain.support_entity_ids)

    spans_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)
    sampled_xy: set[tuple[int, int]] = set()
    for ix in range(ix0, ix1 + 1):
        x = ix * cell
        for iy in range(iy0, iy1 + 1):
            y = iy * cell
            if domain.clip_xy is not None and not domain.clip_xy.covers(Point(x, y)):
                continue

            sampled_xy.add((ix, iy))
            stats.rays += 1
            hits = list(
                tree.select_ray(
                    (x, y, ray_top),
                    (0.0, 0.0, -1.0),
                    length=ray_length,
                )
            )
            if not hits:
                continue
            hits.sort(key=lambda hit: float(hit.distance))
            positions = [tuple(float(value) for value in hit.position) for hit in hits]
            entities = [_hit_entity(ifc_file, hit) for hit in hits]

            seen_supports: set[int] = set()
            candidates: list[_SupportCandidate] = []
            for index, hit in enumerate(hits):
                position = positions[index]
                if position[2] < z0 - opts.vertical_padding_m or position[2] > z1 + opts.vertical_padding_m:
                    continue
                entity = entities[index]
                if entity is None:
                    continue
                try:
                    entity_id = int(entity.id())
                except Exception:
                    continue
                if support_ids and entity_id not in support_ids:
                    continue
                role = ifc_walkable_support_role(entity)
                if not support_ids and role is None:
                    continue
                if entity_id in seen_supports:
                    continue

                normal = tuple(float(value) for value in hit.normal)
                if abs(normal[2]) < min_up:
                    continue
                seen_supports.add(entity_id)
                stats.candidate_hits += 1
                candidates.append(
                    _SupportCandidate(
                        hit_index=index,
                        position=(x, y, position[2]),
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

                if opts.exact_clearance_verification:
                    exact_blocked = _body_clearance_blocked(
                        tree,
                        ifc_file,
                        ignored_entity_ids=ignored_support_ids,
                        position=candidate.position,
                        opts=opts,
                        classify=classify,
                        stats=stats,
                    )
                    if exact_blocked != direct_blocked:
                        stats.clearance_verification_mismatches += 1
                    direct_blocked = direct_blocked or exact_blocked

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
        sampled_xy=sampled_xy,
    )
    stats.samples += len(result)
    return result


def _nearest_distinct_surface_above(index, positions, z, tolerance=0.025):
    """Return the closest ray intersection above one support layer."""
    for upper in reversed(positions[:index]):
        delta = upper[2] - z
        if delta <= tolerance:
            continue
        return float(upper[2])
    return None


def _headroom_blocked(index, positions, z, required_height):
    ceiling_z = _nearest_distinct_surface_above(index, positions, z)
    return ceiling_z is not None and ceiling_z - z < required_height


def _column_body_blocked(
    entities,
    positions,
    *,
    ignored_entity_ids,
    support_z,
    opts,
    classify,
):
    """Classify body occupancy from the already-computed vertical ray column."""
    body_low = float(support_z) + max(opts.hit_merge_tolerance_m, 0.01)
    body_high = float(support_z) + max(opts.agent_height_m, opts.agent_radius_m * 2.0)
    by_entity: dict[int, list[float]] = defaultdict(list)

    for entity, position in zip(entities, positions):
        if entity is None:
            continue
        try:
            entity_id = int(entity.id())
        except Exception:
            continue
        if entity_id in ignored_entity_ids:
            continue
        if classify(entity) in _NON_BLOCKING_CLASSES:
            continue
        if not _is_physical_element(entity):
            continue
        by_entity[entity_id].append(float(position[2]))

    tolerance = max(opts.hit_merge_tolerance_m, 1e-4)
    for z_values in by_entity.values():
        levels = _distinct_levels(z_values, tolerance)
        if not levels:
            continue

        pair_count = len(levels) // 2
        for pair_index in range(pair_count):
            high = levels[pair_index * 2]
            low = levels[pair_index * 2 + 1]
            if _intervals_overlap(low, high, body_low, body_high, tolerance):
                return True

        if len(levels) % 2:
            lone = levels[-1]
            if body_low - tolerance <= lone <= body_high + tolerance:
                return True
    return False


def _distinct_levels(values, tolerance):
    ordered = sorted((float(value) for value in values), reverse=True)
    result: list[float] = []
    for value in ordered:
        if result and abs(result[-1] - value) <= tolerance:
            continue
        result.append(value)
    return result


def _intervals_overlap(low_a, high_a, low_b, high_b, tolerance):
    return high_a > low_b + tolerance and low_a < high_b - tolerance


def _erode_walkable_spans(
    spans_by_xy,
    opts,
    stats=None,
    *,
    sampled_xy=None,
):
    """Apply metric, layer-aware agent-radius erosion to the span field.

    Only coordinates that were actually sampled may behave as missing/blocked
    neighbours. Coordinates outside the sampling clip are acceleration/domain
    boundaries, not physical obstacles, and must not carve false notches into a
    valid floor. Inside the sampled mask, a missing compatible span still acts as
    a ledge/obstruction and is eroded normally.
    """
    viable_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)
    for key, bucket in spans_by_xy.items():
        viable_by_xy[key].extend(span for span in bucket if not span.direct_blocked)

    if opts.agent_radius_m <= 0.0:
        return [
            _span_to_sample(span)
            for bucket in viable_by_xy.values()
            for span in bucket
        ]

    coverage = set(spans_by_xy) if sampled_xy is None else set(sampled_xy)
    cell = max(0.05, float(opts.cell_size_m))
    radius = float(opts.agent_radius_m)
    radius_cells = max(1, math.ceil(radius / cell))
    threshold = radius + cell * 0.5
    offsets = [
        (dx, dy)
        for dx in range(-radius_cells, radius_cells + 1)
        for dy in range(-radius_cells, radius_cells + 1)
        if (dx or dy) and math.hypot(dx * cell, dy * cell) < threshold - 1e-9
    ]

    result: list[_SupportSample] = []
    for key in sorted(viable_by_xy):
        for span in viable_by_xy[key]:
            rejected = False
            for dx, dy in offsets:
                neighbour_key = (span.ix + dx, span.iy + dy)
                if neighbour_key not in coverage:
                    continue
                neighbours = viable_by_xy.get(neighbour_key, ())
                if any(_field_spans_compatible(span, neighbour, opts) for neighbour in neighbours):
                    continue
                rejected = True
                break
            if rejected:
                if stats is not None:
                    stats.heightfield_radius_rejections += 1
                    stats.clearance_rejections += 1
                continue
            result.append(_span_to_sample(span))
    return result


def _field_spans_compatible(a, b, opts):
    dx = abs(a.ix - b.ix)
    dy = abs(a.iy - b.iy)
    if dx == 0 and dy == 0:
        return abs(a.position[2] - b.position[2]) <= opts.hit_merge_tolerance_m
    horizontal = opts.cell_size_m * math.hypot(dx, dy)
    dz = abs(a.position[2] - b.position[2])
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    return dz <= max(opts.max_climb_m, slope_limit) + opts.hit_merge_tolerance_m


def _span_to_sample(span):
    return _SupportSample(
        ix=span.ix,
        iy=span.iy,
        position=span.position,
        owner_id=span.owner_id,
        terrain=span.terrain,
    )


def _append_unique_span(bucket, span, tolerance):
    for existing in list(bucket):
        if abs(existing.position[2] - span.position[2]) > tolerance:
            continue
        if existing.direct_blocked and not span.direct_blocked:
            return
        if span.direct_blocked and not existing.direct_blocked:
            bucket.remove(existing)
            bucket.append(span)
            return
        if existing.terrain == "open" and span.terrain != "open":
            bucket.remove(existing)
            bucket.append(span)
        return
    bucket.append(span)


def _group_support_candidates(candidates, tolerance):
    """Group top-face hits that describe the same physical XY/Z support layer."""
    if not candidates:
        return []
    ordered = sorted(
        candidates,
        key=lambda item: (-item.position[2], item.hit_index, item.owner_id),
    )
    groups: list[list[_SupportCandidate]] = []
    for candidate in ordered:
        if groups and abs(groups[-1][0].position[2] - candidate.position[2]) <= tolerance:
            groups[-1].append(candidate)
        else:
            groups.append([candidate])
    return groups


def _preferred_support_candidate(layer):
    """Choose one representative while preserving vertical-circulation terrain."""
    return min(
        layer,
        key=lambda item: (
            item.terrain == "open",
            item.hit_index,
            -item.position[2],
            item.owner_id,
        ),
    )


def _body_clearance_blocked(
    tree,
    ifc_file,
    *,
    ignored_entity_ids,
    position,
    opts,
    classify,
    stats: SurfaceDetectionStats | None = None,
):
    """Legacy exact OCC clearance, retained only for diagnostic verification."""
    if opts.agent_radius_m <= 0.0:
        return False

    low = max(opts.agent_radius_m, 0.20)
    top = max(low, opts.agent_height_m - opts.agent_radius_m)
    heights = {low, min(top, max(0.70, opts.agent_height_m * 0.50)), top}
    radius = float(opts.agent_radius_m)

    for dz in sorted(heights):
        center = (
            float(position[0]),
            float(position[1]),
            float(position[2] + dz),
        )

        broadphase = None
        try:
            if stats is not None:
                stats.clearance_broadphase_queries += 1
            broadphase = tree.select_box(center, extend=radius)
        except Exception:
            broadphase = None

        if broadphase is not None:
            if not any(
                _clearance_candidate_blocks(
                    ifc_file,
                    raw,
                    ignored_entity_ids=ignored_entity_ids,
                    classify=classify,
                )
                for raw in broadphase
            ):
                if stats is not None:
                    stats.clearance_precise_skips += 1
                continue

        try:
            if stats is not None:
                stats.clearance_precise_queries += 1
            selected = tree.select(center, extend=radius)
        except Exception:
            return False

        if any(
            _clearance_candidate_blocks(
                ifc_file,
                raw,
                ignored_entity_ids=ignored_entity_ids,
                classify=classify,
            )
            for raw in selected
        ):
            return True
    return False


def _clearance_candidate_blocks(
    ifc_file,
    raw,
    *,
    ignored_entity_ids,
    classify,
):
    entity = _selected_entity(ifc_file, raw)
    if entity is None:
        return False
    try:
        entity_id = int(entity.id())
    except Exception:
        return False
    if entity_id in ignored_entity_ids:
        return False
    if classify(entity) in _NON_BLOCKING_CLASSES:
        return False
    return _is_physical_element(entity)


def _triangulate_samples(domain, samples, opts):
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)
    for bucket in by_xy.values():
        bucket.sort(key=lambda sample: (sample.position[2], sample.owner_id))

    quad_origins: set[tuple[int, int]] = set()
    for ix, iy in by_xy:
        for ox in (ix - 1, ix):
            for oy in (iy - 1, iy):
                quad_origins.add((ox, oy))

    cells: list[NavCell] = []
    created: set[tuple] = set()
    for ix, iy in sorted(quad_origins):
        corner_keys = (
            (ix, iy),
            (ix + 1, iy),
            (ix + 1, iy + 1),
            (ix, iy + 1),
        )
        buckets = [by_xy.get(key, ()) for key in corner_keys]
        populated = [index for index, bucket in enumerate(buckets) if bucket]
        if len(populated) < 3:
            continue

        anchor_index = populated[0]
        for anchor in buckets[anchor_index]:
            selected: dict[int, _SupportSample] = {anchor_index: anchor}
            for corner_index in populated[1:]:
                candidate = _nearest_grid_compatible(anchor, buckets[corner_index], opts)
                if candidate is not None:
                    selected[corner_index] = candidate

            triangles = _quad_layer_triangles(selected, opts)
            for triangle in triangles:
                vertices = tuple(sample.position for sample in triangle)
                key = _triangle_key(vertices, opts.hit_merge_tolerance_m)
                if key in created or _triangle_area_3d(*vertices) <= 1e-8:
                    continue
                created.add(key)
                cells.append(
                    NavCell(
                        id=f"cell:detected:{domain.id}:{len(cells)}",
                        vertices_m=vertices,
                        space_id=domain.space_id,
                        level_id=domain.level_id,
                        terrain=_majority_terrain(*triangle),
                    )
                )

    connect_cells_by_shared_edges(
        cells,
        tolerance_m=max(1e-5, opts.hit_merge_tolerance_m * 0.2),
    )
    return cells


def _quad_layer_triangles(
    selected: dict[int, _SupportSample],
    opts: SurfaceDetectionOptions,
) -> list[tuple[_SupportSample, _SupportSample, _SupportSample]]:
    if len(selected) < 3:
        return []

    if len(selected) == 3:
        indices = tuple(sorted(selected))
        triangle = tuple(selected[index] for index in indices)
        return [triangle] if _triangle_samples_compatible(triangle, opts) else []

    sw, se, ne, nw = (selected[index] for index in range(4))
    diagonal_sw_ne = (
        (sw, se, ne),
        (sw, ne, nw),
    )
    diagonal_se_nw = (
        (sw, se, nw),
        (se, ne, nw),
    )
    valid_a = all(_triangle_samples_compatible(triangle, opts) for triangle in diagonal_sw_ne)
    valid_b = all(_triangle_samples_compatible(triangle, opts) for triangle in diagonal_se_nw)

    if valid_a and valid_b:
        rise_a = abs(sw.position[2] - ne.position[2])
        rise_b = abs(se.position[2] - nw.position[2])
        return list(
            diagonal_sw_ne
            if (rise_a, sw.owner_id, ne.owner_id) <= (rise_b, se.owner_id, nw.owner_id)
            else diagonal_se_nw
        )
    if valid_a:
        return list(diagonal_sw_ne)
    if valid_b:
        return list(diagonal_se_nw)
    return []


def _triangle_samples_compatible(
    triangle: tuple[_SupportSample, _SupportSample, _SupportSample],
    opts: SurfaceDetectionOptions,
) -> bool:
    a, b, c = triangle
    return (
        _grid_samples_compatible(a, b, opts)
        and _grid_samples_compatible(b, c, opts)
        and _grid_samples_compatible(c, a, opts)
    )


def _nearest_grid_compatible(source, candidates, opts):
    valid = [candidate for candidate in candidates if _grid_samples_compatible(source, candidate, opts)]
    if not valid:
        return None
    return min(
        valid,
        key=lambda sample: (
            abs(sample.position[2] - source.position[2]),
            sample.owner_id,
        ),
    )


def _grid_samples_compatible(a, b, opts):
    dx = abs(a.ix - b.ix)
    dy = abs(a.iy - b.iy)
    if dx == 0 and dy == 0:
        return abs(a.position[2] - b.position[2]) <= opts.hit_merge_tolerance_m
    if dx > 1 or dy > 1:
        return False
    horizontal = opts.cell_size_m * math.hypot(dx, dy)
    dz = abs(a.position[2] - b.position[2])
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    return dz <= max(opts.max_climb_m, slope_limit) + opts.hit_merge_tolerance_m


def _prune_small_components(cells, opts):
    if not cells:
        return []
    components = surface_components(cells)
    by_id = {cell.id: cell for cell in cells}
    kept: list[NavCell] = []
    for component in components:
        members = [by_id[cell_id] for cell_id in component]
        area = sum(_triangle_area_3d(*cell.vertices_m) for cell in members)
        if len(members) < opts.minimum_component_cells:
            continue
        if area + 1e-9 < opts.minimum_component_area_m2:
            continue
        kept.extend(members)
    return kept


def _nearest_compatible(source, candidates, opts):
    valid = [candidate for candidate in candidates if _samples_compatible(source, candidate, opts)]
    if not valid:
        return None
    return min(
        valid,
        key=lambda sample: (
            abs(sample.position[2] - source.position[2]),
            sample.owner_id,
        ),
    )


def _samples_compatible(a, b, opts, *, diagonal=False):
    horizontal = opts.cell_size_m * (math.sqrt(2.0) if diagonal else 1.0)
    dz = abs(a.position[2] - b.position[2])
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    return dz <= max(opts.max_climb_m, slope_limit) + opts.hit_merge_tolerance_m


def _append_unique_sample(bucket, sample, tolerance):
    for existing in bucket:
        if abs(existing.position[2] - sample.position[2]) <= tolerance:
            if existing.terrain == "open" and sample.terrain != "open":
                bucket.remove(existing)
                bucket.append(sample)
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


def _safe_attr(entity, name, default=None):
    try:
        return getattr(entity, name, default)
    except (RuntimeError, IndexError, AttributeError):
        return default


def _majority_terrain(*samples):
    counts: dict[str, int] = defaultdict(int)
    for sample in samples:
        counts[sample.terrain] += 1
    return max(counts, key=lambda terrain: (counts[terrain], terrain != "open", terrain))


def _triangle_key(vertices, tolerance_m=1e-5):
    scale = 1.0 / max(tolerance_m, 1e-6)
    quantized = sorted(
        tuple(round(value * scale) for value in vertex)
        for vertex in vertices
    )
    return tuple(quantized)


def _cell_geometry_key(cell, tolerance_m):
    return _triangle_key(cell.vertices_m, tolerance_m)


def _triangle_area_3d(a, b, c):
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))
