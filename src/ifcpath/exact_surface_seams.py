from __future__ import annotations

import math
from collections import Counter, defaultdict, deque
from dataclasses import dataclass

from .model import NavCell, Vec3
from .raycast_surface import (
    SurfaceDetectionOptions,
    _body_clearance_blocked,
    _entity_class,
    _hit_entity,
    _native_geometry_tree,
    ifc_walkable_support_role,
)
from .semantic_heightfield_repair import (
    SemanticRepairRegion,
    build_semantic_repair_regions,
)
from .surface_nav import _closest_segment_points, _connect_pair


@dataclass(slots=True)
class ExactSurfaceSeamStats:
    split_spaces: int = 0
    component_pairs: int = 0
    candidate_crossings: int = 0
    verified_crossings: int = 0
    rejected_support: int = 0
    rejected_clearance: int = 0
    rejected_semantic_path: int = 0
    support_rays: int = 0
    exact_body_checks: int = 0
    seams_added: int = 0


def stitch_exact_verified_open_seams(
    ifc_file,
    cells: list[NavCell],
    *,
    options: SurfaceDetectionOptions,
    max_gap_m: float = 1.25,
    support_z_tolerance_m: float = 0.08,
    sample_spacing_m: float | None = None,
    max_candidates_per_pair: int = 24,
) -> ExactSurfaceSeamStats:
    """Repair small raster holes only after exact IFC geometry verification.

    A regular heightfield can leave a short empty band through an otherwise clear
    room even though the physical floor and body-clearance corridor are continuous.
    This function never trusts semantic proximity alone. It considers only split
    components of the *same* authored space, finds short boundary-to-boundary
    crossings, then samples the complete crossing against the native IFC tree:

    * every sample must remain inside the same IfcSpace floor region;
    * a physical walkable support must exist at the expected elevation;
    * the exact OCC body-clearance check must be clear at every sample.

    Only then is the two surface components stitched through an internal portal.
    The portal represents an exactly verified piece of continuous support omitted
    by raster quantisation; IfcSpace never manufactures a floor or bypasses an
    obstacle on its own.
    """
    stats = ExactSurfaceSeamStats()
    if not cells or max_gap_m <= 0.0:
        return stats

    components_by_space = _open_components_by_space(cells)
    split = {
        space_id: components
        for space_id, components in components_by_space.items()
        if len(components) > 1
    }
    stats.split_spaces = len(split)
    if not split:
        return stats

    regions = {region.id: region for region in build_semantic_repair_regions(ifc_file)}
    tree = None
    spacing = sample_spacing_m or max(0.06, min(0.10, options.cell_size_m * 0.5))
    by_id = {cell.id: cell for cell in cells}

    for space_id, components in sorted(split.items()):
        region = regions.get(space_id)
        if region is None:
            continue

        # Union-find lets one verified crossing collapse a component pair before
        # evaluating later candidates. This is important for malformed spaces with
        # three or more fragments and prevents redundant OCC work.
        parent = list(range(len(components)))

        def find(index: int) -> int:
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        boundaries = [_component_boundary_edges(component, by_id) for component in components]
        pair_candidates: list[tuple[float, int, int, str, Vec3, Vec3, str, Vec3, Vec3]] = []
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                stats.component_pairs += 1
                candidates = _closest_edge_candidates(
                    boundaries[i],
                    boundaries[j],
                    max_gap_m=max_gap_m,
                    max_candidates=max_candidates_per_pair,
                )
                for separation, aid, a0, a1, bid, b0, b1 in candidates:
                    pair_candidates.append((separation, i, j, aid, a0, a1, bid, b0, b1))

        pair_candidates.sort(key=lambda item: (item[0], item[1], item[2], item[3], item[6]))
        attempted_pairs: Counter[tuple[int, int]] = Counter()
        for separation, i, j, aid, a0, a1, bid, b0, b1 in pair_candidates:
            if find(i) == find(j):
                continue
            pair_key = (i, j)
            if attempted_pairs[pair_key] >= max_candidates_per_pair:
                continue
            attempted_pairs[pair_key] += 1
            stats.candidate_crossings += 1

            pa, pb = _closest_segment_points(a0, a1, b0, b1)
            if abs(pa[2] - pb[2]) > max(
                options.hit_merge_tolerance_m * 2.0,
                support_z_tolerance_m,
            ):
                continue

            if tree is None:
                try:
                    tree = _native_geometry_tree(ifc_file)
                except Exception:
                    return stats

            verdict = _verify_crossing(
                tree,
                ifc_file,
                pa,
                pb,
                region,
                options,
                sample_spacing_m=spacing,
                support_z_tolerance_m=support_z_tolerance_m,
                stats=stats,
            )
            if not verdict:
                continue

            a = by_id[aid]
            b = by_id[bid]
            _connect_pair(a, b)
            center = _midpoint(pa, pb)
            portal = (center, center)
            a.portals[b.id] = portal
            b.portals[a.id] = portal
            union(i, j)
            stats.verified_crossings += 1
            stats.seams_added += 1

    return stats


def _open_components_by_space(cells: list[NavCell]) -> dict[str, list[set[str]]]:
    by_id = {cell.id: cell for cell in cells}
    ids_by_space: dict[str, set[str]] = defaultdict(set)
    for cell in cells:
        if cell.terrain == "open" and cell.space_id:
            ids_by_space[cell.space_id].add(cell.id)

    result: dict[str, list[set[str]]] = {}
    for space_id, owned in ids_by_space.items():
        remaining = set(owned)
        components: list[set[str]] = []
        while remaining:
            start = min(remaining)
            remaining.remove(start)
            component = {start}
            queue = deque([start])
            while queue:
                current = queue.popleft()
                for neighbor_id in by_id[current].neighbor_ids:
                    if neighbor_id not in remaining:
                        continue
                    remaining.remove(neighbor_id)
                    component.add(neighbor_id)
                    queue.append(neighbor_id)
            components.append(component)
        result[space_id] = sorted(components, key=lambda item: (-len(item), min(item)))
    return result


def _component_boundary_edges(component: set[str], by_id: dict[str, NavCell]):
    counts: Counter[tuple] = Counter()
    data: dict[tuple, tuple[str, Vec3, Vec3]] = {}
    scale = 100_000.0

    def key(a: Vec3, b: Vec3):
        qa = tuple(round(value * scale) for value in a)
        qb = tuple(round(value * scale) for value in b)
        return (qa, qb) if qa <= qb else (qb, qa)

    for cell_id in component:
        cell = by_id[cell_id]
        v = cell.vertices_m
        for start, end in ((v[0], v[1]), (v[1], v[2]), (v[2], v[0])):
            edge = key(start, end)
            counts[edge] += 1
            data.setdefault(edge, (cell_id, start, end))
    return [data[edge] for edge, count in counts.items() if count == 1]


def _closest_edge_candidates(boundaries_a, boundaries_b, *, max_gap_m, max_candidates):
    candidates = []
    for aid, a0, a1 in boundaries_a:
        amin_x, amax_x = sorted((a0[0], a1[0]))
        amin_y, amax_y = sorted((a0[1], a1[1]))
        for bid, b0, b1 in boundaries_b:
            bmin_x, bmax_x = sorted((b0[0], b1[0]))
            bmin_y, bmax_y = sorted((b0[1], b1[1]))
            if _interval_gap(amin_x, amax_x, bmin_x, bmax_x) > max_gap_m:
                continue
            if _interval_gap(amin_y, amax_y, bmin_y, bmax_y) > max_gap_m:
                continue
            pa, pb = _closest_segment_points(a0, a1, b0, b1)
            separation = math.dist(pa, pb)
            if separation > max_gap_m + 1e-9:
                continue
            candidates.append((separation, aid, a0, a1, bid, b0, b1))
    candidates.sort(key=lambda item: (item[0], item[1], item[4]))
    return candidates[:max_candidates]


def _verify_crossing(
    tree,
    ifc_file,
    start: Vec3,
    end: Vec3,
    region: SemanticRepairRegion,
    options: SurfaceDetectionOptions,
    *,
    sample_spacing_m: float,
    support_z_tolerance_m: float,
    stats: ExactSurfaceSeamStats,
) -> bool:
    distance = math.dist(start, end)
    count = max(2, int(math.ceil(distance / max(0.03, sample_spacing_m))))
    min_up = math.cos(math.radians(options.max_slope_deg))

    # Endpoints already belong to accepted triangles. Verify only the interior
    # interpolation samples, where the heightfield omitted geometry.
    for index in range(1, count):
        t = index / count
        expected = _lerp(start, end, t)
        if not region.contains(expected, vertical_tolerance_m=max(0.35, options.max_climb_m)):
            stats.rejected_semantic_path += 1
            return False

        stats.support_rays += 1
        support = _support_hit_near(
            tree,
            ifc_file,
            expected,
            options,
            z_tolerance_m=support_z_tolerance_m,
            min_up=min_up,
        )
        if support is None:
            stats.rejected_support += 1
            return False
        entity_id, position = support

        stats.exact_body_checks += 1
        if _body_clearance_blocked(
            tree,
            ifc_file,
            ignored_entity_ids={entity_id},
            position=position,
            opts=options,
            classify=_entity_class,
        ):
            stats.rejected_clearance += 1
            return False
    return True


def _support_hit_near(
    tree,
    ifc_file,
    expected: Vec3,
    options: SurfaceDetectionOptions,
    *,
    z_tolerance_m: float,
    min_up: float,
) -> tuple[int, Vec3] | None:
    vertical = max(0.30, options.max_climb_m + 0.12)
    origin = (expected[0], expected[1], expected[2] + vertical)
    try:
        hits = list(tree.select_ray(origin, (0.0, 0.0, -1.0), length=vertical * 2.0))
    except Exception:
        return None
    hits.sort(key=lambda hit: float(getattr(hit, "distance", 0.0)))
    for hit in hits:
        entity = _hit_entity(ifc_file, hit)
        if entity is None or ifc_walkable_support_role(entity) is None:
            continue
        try:
            normal = tuple(float(value) for value in hit.normal)
            position = tuple(float(value) for value in hit.position)
            entity_id = int(entity.id())
        except Exception:
            continue
        if abs(normal[2]) < min_up:
            continue
        if abs(position[2] - expected[2]) > z_tolerance_m:
            continue
        return entity_id, (expected[0], expected[1], position[2])
    return None


def _interval_gap(a0: float, a1: float, b0: float, b1: float) -> float:
    if a1 < b0:
        return b0 - a1
    if b1 < a0:
        return a0 - b1
    return 0.0


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _midpoint(a: Vec3, b: Vec3) -> Vec3:
    return (
        (a[0] + b[0]) * 0.5,
        (a[1] + b[1]) * 0.5,
        (a[2] + b[2]) * 0.5,
    )
