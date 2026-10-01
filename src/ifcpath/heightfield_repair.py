from __future__ import annotations

import math
from collections import defaultdict, deque

from .raycast_surface import (
    SurfaceDetectionOptions,
    _SupportSample,
    _WalkableSpan,
    _field_spans_compatible,
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
    """Label 4-neighbour connected components in an accepted surface field."""
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


def rejected_bridge_paths(
    rejected: list[_WalkableSpan],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    *,
    max_path_spans: int = 16,
) -> list[list[_WalkableSpan]]:
    """Find short rejected paths connecting distinct accepted components.

    This is intentionally path-based rather than rejected-component-based. A
    narrow missing strip can be connected to the much larger erosion band around
    a room boundary; selecting that entire rejected component would be unsafe and
    expensive. Instead, each accepted component launches a BFS through compatible
    rejected spans and only the shortest path reaching another accepted component
    is returned. Exact physical clearance still decides whether those spans may be
    restored.
    """
    if not rejected or not accepted:
        return []

    accepted_by_xy: dict[tuple[int, int], list[_SupportSample]] = defaultdict(list)
    for sample in accepted:
        accepted_by_xy[(sample.ix, sample.iy)].append(sample)
    component_ids = sample_component_ids(accepted, opts)
    component_count = len(set(component_ids.values()))
    if component_count < 2:
        return []

    rejected_by_xy: dict[tuple[int, int], list[_WalkableSpan]] = defaultdict(list)
    rejected_by_key: dict[tuple[int, int, int], _WalkableSpan] = {}
    for span in rejected:
        key = span_key(span, opts.hit_merge_tolerance_m)
        rejected_by_key[key] = span
        rejected_by_xy[(span.ix, span.iy)].append(span)

    touched_by_rejected: dict[tuple[int, int, int], set[int]] = {}
    for key, span in rejected_by_key.items():
        touched: set[int] = set()
        span_sample = _sample_as_sample(span)
        for dx, dy in _CARDINAL_OFFSETS:
            for neighbor in accepted_by_xy.get((span.ix + dx, span.iy + dy), ()):
                if not samples_compatible(span_sample, neighbor, opts):
                    continue
                component = component_ids.get(sample_key(neighbor, opts.hit_merge_tolerance_m))
                if component is not None:
                    touched.add(component)
        touched_by_rejected[key] = touched

    candidate_paths: list[list[_WalkableSpan]] = []
    components = sorted(set(component_ids.values()))
    for source_component in components:
        source_keys = sorted(
            key for key, touched in touched_by_rejected.items() if source_component in touched
        )
        if not source_keys:
            continue

        queue = deque(source_keys)
        parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {
            key: None for key in source_keys
        }
        depth: dict[tuple[int, int, int], int] = {key: 1 for key in source_keys}
        target_key = None

        while queue:
            key = queue.popleft()
            if depth[key] > max_path_spans:
                continue
            other_components = touched_by_rejected.get(key, set()) - {source_component}
            if other_components:
                target_key = key
                break

            span = rejected_by_key[key]
            for dx, dy in _CARDINAL_OFFSETS:
                for neighbor in rejected_by_xy.get((span.ix + dx, span.iy + dy), ()):
                    neighbor_key = span_key(neighbor, opts.hit_merge_tolerance_m)
                    if neighbor_key in parent:
                        continue
                    if not spans_compatible(span, neighbor, opts):
                        continue
                    parent[neighbor_key] = key
                    depth[neighbor_key] = depth[key] + 1
                    if depth[neighbor_key] <= max_path_spans:
                        queue.append(neighbor_key)

        if target_key is None:
            continue
        path_keys = []
        cursor = target_key
        while cursor is not None:
            path_keys.append(cursor)
            cursor = parent[cursor]
        path_keys.reverse()
        candidate_paths.append([rejected_by_key[key] for key in path_keys])

    # The same bridge is discovered from both sides. Deduplicate by its span set,
    # then prefer shorter paths so the exact-repair budget is spent conservatively.
    unique: dict[frozenset[tuple[int, int, int]], list[_WalkableSpan]] = {}
    for path in candidate_paths:
        signature = frozenset(span_key(span, opts.hit_merge_tolerance_m) for span in path)
        existing = unique.get(signature)
        if existing is None or len(path) < len(existing):
            unique[signature] = path
    return sorted(
        unique.values(),
        key=lambda path: (len(path), [(span.ix, span.iy, span.position[2]) for span in path]),
    )


def rejected_bridge_groups(
    rejected: list[_WalkableSpan],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    *,
    max_group_spans: int = 96,
) -> list[list[_WalkableSpan]]:
    """Backward-compatible wrapper for earlier qualification tests."""
    return [
        path
        for path in rejected_bridge_paths(
            rejected,
            accepted,
            opts,
            max_path_spans=max_group_spans,
        )
        if len(path) <= max_group_spans
    ]


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


def _sample_as_sample(span: _WalkableSpan) -> _SupportSample:
    return _SupportSample(
        ix=span.ix,
        iy=span.iy,
        position=span.position,
        owner_id=span.owner_id,
        terrain=span.terrain,
    )


_CARDINAL_OFFSETS = ((1, 0), (-1, 0), (0, 1), (0, -1))
