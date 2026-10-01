from __future__ import annotations

from collections import Counter, defaultdict, deque

from .heightfield_repair import sample_component_ids
from .model import NavCell
from .raycast_surface import SurfaceDetectionOptions, _SupportSample
from .semantic_heightfield_repair import SemanticRepairRegion, semantic_region_for_position


def semantic_connectivity_diagnostics(
    regions: list[SemanticRepairRegion],
    samples: list[_SupportSample],
    cells: list[NavCell],
    opts: SurfaceDetectionOptions,
) -> dict[str, dict[str, list[int]]]:
    """Compare room-local connectivity in the sample field and NavCell surface.

    This is qualification telemetry only. It does not change walkability. It tells
    us whether a semantic split already exists in the accepted physical samples or
    is introduced while those samples are converted into continuous triangles.
    """
    if not regions:
        return {}

    samples_by_region: dict[str, list[_SupportSample]] = defaultdict(list)
    for sample in samples:
        region = semantic_region_for_position(regions, sample.position)
        if region is not None:
            samples_by_region[region.id].append(sample)

    cells_by_region: dict[str, list[NavCell]] = defaultdict(list)
    for cell in cells:
        center = _cell_centroid(cell)
        region = semantic_region_for_position(regions, center)
        if region is not None:
            cells_by_region[region.id].append(cell)

    result: dict[str, dict[str, list[int]]] = {}
    for region in regions:
        room_samples = samples_by_region.get(region.id, [])
        room_cells = cells_by_region.get(region.id, [])
        sample_sizes = _sample_component_sizes(room_samples, opts)
        cell_sizes = _cell_component_sizes(room_cells)
        if len(sample_sizes) > 1 or len(cell_sizes) > 1:
            result[region.id] = {
                "sample_components": sample_sizes,
                "cell_components": cell_sizes,
            }
    return result


def _sample_component_sizes(
    samples: list[_SupportSample],
    opts: SurfaceDetectionOptions,
) -> list[int]:
    if not samples:
        return []
    ids = sample_component_ids(samples, opts)
    counts = Counter(ids.values())
    return sorted(counts.values(), reverse=True)


def _cell_component_sizes(cells: list[NavCell]) -> list[int]:
    if not cells:
        return []
    by_id = {cell.id: cell for cell in cells}
    remaining = set(by_id)
    sizes: list[int] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        queue = deque([start])
        size = 0
        while queue:
            cell_id = queue.popleft()
            size += 1
            for neighbor_id in by_id[cell_id].neighbor_ids:
                if neighbor_id not in remaining:
                    continue
                remaining.remove(neighbor_id)
                queue.append(neighbor_id)
        sizes.append(size)
    return sorted(sizes, reverse=True)


def _cell_centroid(cell: NavCell):
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )
