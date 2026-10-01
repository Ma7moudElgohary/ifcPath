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
    # Selective topology repair uses the exact OCC body check only where a span
    # rejected by metric field erosion can reconnect otherwise separate regions.
    # These counters make sure the fast path stays far below the old ~15k exact
    # queries while remaining observable in real-IFC qualification.
    topology_repair_candidates: int = 0
    topology_repair_exact_checks: int = 0
    topology_repair_restored: int = 0
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

    samples_by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
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
                if direct_blocked:
                    stats.heightfield_direct_rejections += 1
                    stats.clearance_rejections += 1

                span = _WalkableSpan(
                    ix=ix,
                    iy=iy,
                    position=candidate.position,
                    owner_id=candidate.owner_id,
                    terrain=candidate.terrain,
                    ceiling_z=ceiling_z,
                    free_height_m=free_height,
                    direct_blocked=direct_blocked,
                )
                _append_unique_span(
                    spans_by_xy[(ix, iy)],
                    span,
                    opts.hit_merge_tolerance_m,
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

    stats.heightfield_spans += sum(len(bucket) for bucket in spans_by_xy.values())
    result = _erode_walkable_spans(
        spans_by_xy,
        opts,
        stats,
        sampled_xy=sampled_xy,
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


def _column_body_blocked(
    entities,
    positions,
    *,
    ignored_entity_ids,
    support_z,
    opts,
    classify,
):
    if opts.agent_height_m <= 0.0:
        return False

    body_bottom = support_z + opts.hit_merge_tolerance_m
    body_top = support_z + opts.agent_height_m
    levels_by_entity: dict[int, list[float]] = defaultdict(list)

    for entity, position in zip(entities, positions):
        if entity is None:
            continue
        try:
            entity_id = int(entity.id())
        except Exception:
            continue
        if entity_id in ignored_entity_ids:
            continue
        try:
            class_name = classify(entity)
        except Exception:
            class_name = _entity_class(entity)
        if class_name in _NON_BLOCKING_CLASSES:
            continue
        z = float(position[2])
        if z < body_bottom - opts.hit_merge_tolerance_m:
            continue
        if z > body_top + opts.hit_merge_tolerance_m:
            continue
        levels_by_entity[entity_id].append(z)

    for levels in levels_by_entity.values():
        distinct = _distinct_levels(levels, opts.hit_merge_tolerance_m)
        # Closed solids normally contribute paired top/bottom crossings. Pairing
        # those intervals makes a vertical ray sufficient for body occupancy. An
        # unmatched shell crossing inside the body is treated conservatively as a
        # blocker rather than allowing routes through thin/open obstacle geometry.
        for index in range(0, len(distinct) - 1, 2):
            high = distinct[index]
            low = distinct[index + 1]
            if _intervals_overlap(low, high, body_bottom, body_top):
                return True
        if len(distinct) % 2 == 1:
            unmatched = distinct[-1]
            if body_bottom - opts.hit_merge_tolerance_m <= unmatched <= body_top + opts.hit_merge_tolerance_m:
                return True
    return False


def _distinct_levels(levels, tolerance):
    result: list[float] = []
    for value in sorted((float(item) for item in levels), reverse=True):
        if result and abs(result[-1] - value) <= tolerance:
            continue
        result.append(value)
    return result


def _intervals_overlap(a0, a1, b0, b1):
    low_a, high_a = sorted((float(a0), float(a1)))
    low_b, high_b = sorted((float(b0), float(b1)))
    return high_a >= low_b and high_b >= low_a


def _erode_walkable_spans(
    spans_by_xy,
    opts,
    stats: SurfaceDetectionStats | None = None,
    sampled_xy=None,
):
    viable_by_xy = {
        key: [span for span in spans if not span.direct_blocked]
        for key, spans in spans_by_xy.items()
    }
    viable_by_xy = {key: spans for key, spans in viable_by_xy.items() if spans}
    sampled_xy = None if sampled_xy is None else set(sampled_xy)
    if opts.agent_radius_m <= 0.0:
        return [_span_to_sample(span) for spans in viable_by_xy.values() for span in spans]

    cell = max(0.05, float(opts.cell_size_m))
    radius = float(opts.agent_radius_m)
    radius_cells = max(1, int(math.ceil(radius / cell)))
    # A sample represents the centre of a square grid cell. Expanding by half a
    # cell approximates distance to the occupied-cell boundary instead of distance
    # only to neighbouring sample centres.
    threshold = radius + cell * 0.5
    offsets: list[tuple[int, int]] = []
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if dx == 0 and dy == 0:
                continue
            if math.hypot(dx * cell, dy * cell) < threshold - 1e-9:
                offsets.append((dx, dy))

    accepted: list[_SupportSample] = []
    for key in sorted(viable_by_xy):
        for span in viable_by_xy[key]:
            rejected = False
            for dx, dy in offsets:
                neighbor_key = (span.ix + dx, span.iy + dy)
                # A missing coordinate outside the support sampling mask is not a
                # physical obstacle. Without this distinction, the global domain
                # padding is eroded as if it were a wall and narrow edge regions
                # can be severed for purely numerical reasons.
                if sampled_xy is not None and neighbor_key not in sampled_xy:
                    continue
                neighbors = viable_by_xy.get(neighbor_key, ())
                if any(_field_spans_compatible(span, neighbor, opts, dx, dy) for neighbor in neighbors):
                    continue
                rejected = True
                break
            if rejected:
                if stats is not None:
                    stats.heightfield_radius_rejections += 1
                    stats.clearance_rejections += 1
                continue
            accepted.append(_span_to_sample(span))
    return accepted


def _field_spans_compatible(a, b, opts, dx=1, dy=0):
    dz = abs(a.position[2] - b.position[2])
    if dz <= opts.max_climb_m + opts.hit_merge_tolerance_m:
        return True
    horizontal = max(opts.cell_size_m, math.hypot(dx * opts.cell_size_m, dy * opts.cell_size_m))
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    if dz <= slope_limit + opts.hit_merge_tolerance_m and (a.terrain != "open" or b.terrain != "open"):
        return True
    return False


def _span_to_sample(span):
    return _SupportSample(
        ix=span.ix,
        iy=span.iy,
        position=span.position,
        owner_id=span.owner_id,
        terrain=span.terrain,
    )


def _append_unique_span(bucket, span, tolerance):
    for existing in bucket:
        if abs(existing.position[2] - span.position[2]) <= tolerance:
            return
    bucket.append(span)


def _group_support_candidates(candidates, tolerance):
    if not candidates:
        return []
    ordered = sorted(candidates, key=lambda item: (-item.position[2], item.hit_index, item.owner_id))
    groups: list[list[_SupportCandidate]] = []
    for candidate in ordered:
        if groups and abs(groups[-1][0].position[2] - candidate.position[2]) <= tolerance:
            groups[-1].append(candidate)
        else:
            groups.append([candidate])
    return groups


def _preferred_support_candidate(layer):
    return min(
        layer,
        key=lambda item: (
            item.terrain == "open",
            item.hit_index,
            -item.position[2],
            item.owner_id,
        ),
    )


def _append_unique_sample(bucket, sample, tolerance):
    for existing in bucket:
        if abs(existing.position[2] - sample.position[2]) <= tolerance:
            return
    bucket.append(sample)


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
    """Legacy exact OCC body-clearance path retained for diagnostics only."""
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
    try:
        class_name = classify(entity)
    except Exception:
        class_name = _entity_class(entity)
    return class_name not in _NON_BLOCKING_CLASSES


def _selected_entity(ifc_file, raw):
    candidate = getattr(raw, "instance", raw)
    entity_id = getattr(candidate, "id", None)
    if callable(entity_id):
        try:
            entity_id = entity_id()
        except Exception:
            entity_id = None
    if entity_id is None:
        entity_id = getattr(candidate, "id", None)
    try:
        if entity_id is not None:
            return ifc_file.by_id(int(entity_id))
    except Exception:
        return None
    return None


def _hit_entity(ifc_file, hit):
    return _selected_entity(ifc_file, getattr(hit, "instance", hit))


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


def _triangulate_samples(domain, samples, opts):
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)
    for bucket in by_xy.values():
        bucket.sort(key=lambda sample: sample.position[2])

    cells: list[NavCell] = []
    counter = 0
    keys = sorted(by_xy)
    for ix, iy in keys:
        corners = [
            by_xy.get((ix, iy), []),
            by_xy.get((ix + 1, iy), []),
            by_xy.get((ix + 1, iy + 1), []),
            by_xy.get((ix, iy + 1), []),
        ]
        for combo in _quad_layers(corners, opts):
            present = [index for index, sample in enumerate(combo) if sample is not None]
            if len(present) < 3:
                continue
            terrain = _preferred_terrain(sample.terrain for sample in combo if sample is not None)
            if len(present) == 4:
                triangles = ((0, 1, 2), (0, 2, 3))
            else:
                triangles = (tuple(present),)
            for tri in triangles:
                triangle_samples = [combo[index] for index in tri]
                if any(sample is None for sample in triangle_samples):
                    continue
                vertices = tuple(sample.position for sample in triangle_samples)
                cells.append(
                    NavCell(
                        id=f"cell:detected:{domain.id}:{counter}",
                        vertices_m=vertices,
                        space_id=domain.space_id,
                        level_id=domain.level_id,
                        terrain=terrain,
                    )
                )
                counter += 1
    return cells


def _quad_layers(corners, opts):
    seeds = [sample for bucket in corners for sample in bucket]
    groups: list[list[_SupportSample | None]] = []
    used: set[tuple[int, int, int]] = set()
    for seed in seeds:
        key = (seed.ix, seed.iy, seed.owner_id)
        if key in used:
            continue
        combo: list[_SupportSample | None] = []
        for bucket in corners:
            match = _nearest_compatible_sample(seed, bucket, opts)
            combo.append(match)
            if match is not None:
                used.add((match.ix, match.iy, match.owner_id))
        if sum(item is not None for item in combo) >= 3:
            groups.append(combo)
    return groups


def _nearest_compatible_sample(seed, bucket, opts):
    compatible = [
        item
        for item in bucket
        if _samples_compatible(seed, item, opts)
    ]
    if not compatible:
        return None
    return min(compatible, key=lambda item: abs(item.position[2] - seed.position[2]))


def _samples_compatible(a, b, opts):
    dz = abs(a.position[2] - b.position[2])
    horizontal = max(opts.cell_size_m, math.hypot(a.position[0] - b.position[0], a.position[1] - b.position[1]))
    if dz <= opts.max_climb_m + opts.hit_merge_tolerance_m:
        return True
    slope_limit = math.tan(math.radians(opts.max_slope_deg)) * horizontal
    return dz <= slope_limit + opts.hit_merge_tolerance_m and (a.terrain != "open" or b.terrain != "open")


def _preferred_terrain(terrains):
    priority = {"stair": 0, "ramp": 1, "escalator": 2, "open": 3}
    return min((str(item) for item in terrains), key=lambda item: priority.get(item, 99), default="open")


def _prune_small_components(cells, opts):
    if not cells:
        return []
    connect_cells_by_shared_edges(
        cells,
        tolerance_m=max(1e-5, opts.hit_merge_tolerance_m * 0.2),
    )
    components = surface_components(cells)
    if len(components) <= 1:
        return cells
    by_id = {cell.id: cell for cell in cells}
    keep: set[str] = set()
    for component in components:
        if len(component) >= opts.minimum_component_cells:
            keep.update(component)
            continue
        area = sum(_triangle_area(by_id[cell_id]) for cell_id in component)
        if area >= opts.minimum_component_area_m2:
            keep.update(component)
    return [cell for cell in cells if cell.id in keep]


def _triangle_area(cell):
    a, b, c = cell.vertices_m
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))


def _cell_geometry_key(cell, tolerance):
    scale = 1.0 / max(1e-6, tolerance)
    return tuple(
        sorted(
            tuple(round(value * scale) for value in vertex)
            for vertex in cell.vertices_m
        )
    )
