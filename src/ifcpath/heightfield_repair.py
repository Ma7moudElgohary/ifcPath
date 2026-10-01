from __future__ import annotations

import math
from collections import defaultdict, deque

from .raycast_surface import (
    SurfaceDetectionOptions,
    _SupportSample,
    _WalkableSpan,
    _field_spans_compatible,
    _span_to_sample,
)


def sample_key(sample: _SupportSample, tolerance: float) -> tuple[int, int, int]:
    scale = 1.0 / max(1e-6, tolerance)
    return sample.ix, sample.iy, round(sample.position[2] * scale)


def span_key(span: _WalkableSpan, tolerance: float) -> tuple[int, int, int]:
    scale = 1.0 / max(1e-6, tolerance)
    return span.ix, span.iy, round(span.position[2] * scale)


def sample_component_ids(
    samples: list[_SupportSample],
    opts: SurfaceDetectionOptions,
) -> dict[tuple[int, int, int], int]:
    """Label 4-neighbour connected components in the accepted surface field."""
    by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        by_xy[(sample.ix, sample.iy)].append(sample)

    component_ids: dict[tuple[int, int, int], int] = {}
    component = 0
    for sample in samples:
        key = sample_key(sample, opts.hit_merge_tolerance_m)
        if key in component_ids:
            continue
        component_ids[key] = component
        queue = deque([sample])
        while queue:
            current = queue.popleft()
            for dx, dy in _CARDINAL_OFFSETS:
                for neighbor in by_xy.get((current.ix + dx, current.iy + dy), ()):
                    neighbor_key = sample_key(neighbor, opts.hit_merge_tolerance_m)
                    if neighbor_key in component_ids:
                        continue
                    if not samples_compatible(current, neighbor, opts):
                        continue
                    component_ids[neighbor_key] = component
                    queue.append(neighbor)
        component += 1
    return component_ids


def rejected_bridge_groups(
    rejected: list[_WalkableSpan],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    *,
    max_group_spans: int = 96,
) -> list[list[_WalkableSpan]]:
    """Return rejected span chains that connect distinct accepted components.

    A coarse metric erosion can remove two or more consecutive grid samples from
    a physically clear neck. No individual rejected sample then touches both
    surviving sides, so a one-cell repair heuristic can never fire. This routine
    treats compatible rejected spans as a temporary graph, finds each connected
    rejected chain, and selects only chains that touch two or more *different*
    accepted components. Large rejected regions are intentionally ignored: repair
    is for narrow quantisation bottlenecks, not for bypassing real obstacles.
    """
    if not rejected or not accepted:
        return []

    accepted_by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in accepted:
        accepted_by_xy[(sample.ix, sample.iy)].append(sample)
    accepted_components = sample_component_ids(accepted, opts)

    rejected_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)
    rejected_by_key: dict[tuple[int, int, int], _WalkableSpan] = {}
    for span in rejected:
        key = span_key(span, opts.hit_merge_tolerance_m)
        rejected_by_key[key] = span
        rejected_by_xy[(span.ix, span.iy)].append(span)

    visited: set[tuple[int, int, int]] = set()
    result: list[list[_WalkableSpan]] = []

    for seed_key, seed in sorted(rejected_by_key.items()):
        if seed_key in visited:
            continue
        visited.add(seed_key)
        group: list[_WalkableSpan] = []
        touched_components: set[int] = set()
        queue = deque([seed])

        while queue:
            span = queue.popleft()
            group.append(span)
            span_sample = _span_to_sample(span)

            for dx, dy in _CARDINAL_OFFSETS:
                neighbor_xy = (span.ix + dx, span.iy + dy)

                for accepted_neighbor in accepted_by_xy.get(neighbor_xy, ()):
                    if not samples_compatible(span_sample, accepted_neighbor, opts):
                        continue
                    component = accepted_components.get(
                        sample_key(accepted_neighbor, opts.hit_merge_tolerance_m)
                    )
                    if component is not None:
                        touched_components.add(component)

                for rejected_neighbor in rejected_by_xy.get(neighbor_xy, ()):
                    neighbor_key = span_key(rejected_neighbor, opts.hit_merge_tolerance_m)
                    if neighbor_key in visited:
                        continue
                    if not spans_compatible(span, rejected_neighbor, opts):
                        continue
                    visited.add(neighbor_key)
                    queue.append(rejected_neighbor)

        if 1 <= len(group) <= max_group_spans and len(touched_components) >= 2:
            result.append(sorted(group, key=lambda item: (item.ix, item.iy, item.position[2])))

    return result


def samples_compatible(
    a: _SupportSample,
    b: _SupportSample,
    opts: SurfaceDetectionOptions,
) -> bool:
    return spans_compatible(_sample_as_span(a), _sample_as_span(b), opts)


def spans_compatible(
    a: _WalkableSpan,
    b: _WalkableSpan,
    opts: SurfaceDetectionOptions,
) -> bool:
    if abs(a.ix - b.ix) > 1 or abs(a.iy - b.iy) > 1:
        return False
    return _field_spans_compatible(a, b, opts)


def _sample_as_span(sample: _SupportSample) -> _WalkableSpan:
    return _WalkableSpan(
        ix=sample.ix,
        iy=sample.iy,
        position=sample.position,
        owner_id=sample.owner_id,
        terrain=sample.terrain,
        ceiling_z=None,
        free_height_m=math.inf,
        direct_blocked=False,
    )


_CARDINAL_OFFSETS = ((1, 0), (-1, 0), (0, 1), (0, -1))
