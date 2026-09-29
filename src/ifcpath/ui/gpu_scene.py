from __future__ import annotations

from array import array
from dataclasses import dataclass, field
from typing import Iterable

from ..model import InavModel, NavCell, Vec3
from .preview_geometry import PreviewGeometry

TriangleRecord = tuple[tuple[Vec3, Vec3, Vec3], str | None]

_CATEGORY_COLORS: dict[str, tuple[float, float, float, float]] = {
    "wall": (0.45, 0.51, 0.58, 1.0),
    "slab": (0.34, 0.40, 0.47, 1.0),
    "column": (0.58, 0.62, 0.68, 1.0),
    "stair": (0.50, 0.44, 0.65, 1.0),
    "ramp": (0.44, 0.50, 0.65, 1.0),
    "door": (0.72, 0.50, 0.28, 1.0),
    "navmesh": (0.18, 0.56, 0.90, 0.30),
}


@dataclass(slots=True)
class GpuMeshBatch:
    """CPU-side mesh payload ready for one GPU upload.

    Vertex positions are stored relative to ``GpuSceneData.origin`` to preserve
    float32 precision for IFC/geospatial coordinates. Vertices are interleaved
    as xyz + rgba (7 float32 values). Triangle records are spatially ordered
    before chunking so each batch has a useful local AABB for frustum culling.
    """

    key: str
    category: str
    level_id: str | None
    vertices: array = field(default_factory=lambda: array("f"))
    indices: array = field(default_factory=lambda: array("I"))
    triangle_guids: list[str | None] = field(default_factory=list)
    bounds_min: Vec3 = (0.0, 0.0, 0.0)
    bounds_max: Vec3 = (0.0, 0.0, 0.0)

    @property
    def vertex_count(self) -> int:
        return len(self.vertices) // 7

    @property
    def triangle_count(self) -> int:
        return len(self.indices) // 3

    @property
    def byte_size(self) -> int:
        return len(self.vertices) * self.vertices.itemsize + len(self.indices) * self.indices.itemsize


@dataclass(slots=True)
class GpuSceneData:
    batches: list[GpuMeshBatch]
    origin: Vec3
    bounds_min: Vec3
    bounds_max: Vec3

    @property
    def triangle_count(self) -> int:
        return sum(batch.triangle_count for batch in self.batches)

    @property
    def byte_size(self) -> int:
        return sum(batch.byte_size for batch in self.batches)

    def world_to_local(self, point: Vec3) -> Vec3:
        return (
            point[0] - self.origin[0],
            point[1] - self.origin[1],
            point[2] - self.origin[2],
        )

    def local_to_world(self, point: Vec3) -> Vec3:
        return (
            point[0] + self.origin[0],
            point[1] + self.origin[1],
            point[2] + self.origin[2],
        )


def build_gpu_scene(
    model: InavModel | None,
    bim: PreviewGeometry | None,
    *,
    level_id: str | None = None,
    show_bim: bool = True,
    show_navmesh: bool = True,
    max_triangles_per_batch: int = 24_000,
) -> GpuSceneData:
    groups: dict[tuple[str | None, str], list[TriangleRecord]] = {}
    source_points: list[Vec3] = []

    if show_bim and bim is not None:
        for tri in bim.triangles:
            if level_id and tri.level_id and tri.level_id != level_id:
                continue
            groups.setdefault((tri.level_id, tri.category), []).append((tri.vertices_m, tri.ifc_guid))
            source_points.extend(tri.vertices_m)

    if show_navmesh and model is not None:
        for cell in model.cells:
            if level_id and cell.level_id != level_id:
                continue
            groups.setdefault((cell.level_id, "navmesh"), []).append((cell.vertices_m, None))
            source_points.extend(cell.vertices_m)

    if not source_points:
        return GpuSceneData(
            batches=[],
            origin=(0.0, 0.0, 0.0),
            bounds_min=(0.0, 0.0, 0.0),
            bounds_max=(1.0, 1.0, 1.0),
        )

    bounds_min: Vec3 = tuple(min(p[i] for p in source_points) for i in range(3))  # type: ignore[assignment]
    bounds_max: Vec3 = tuple(max(p[i] for p in source_points) for i in range(3))  # type: ignore[assignment]
    origin: Vec3 = tuple((bounds_min[i] + bounds_max[i]) * 0.5 for i in range(3))  # type: ignore[assignment]

    batches: list[GpuMeshBatch] = []
    cap = max(1, int(max_triangles_per_batch))
    for (batch_level, category), triangles in sorted(groups.items(), key=lambda item: ((item[0][0] or ""), item[0][1])):
        ordered = _spatially_order(triangles)
        for chunk_index in range(0, len(ordered), cap):
            chunk = ordered[chunk_index : chunk_index + cap]
            batches.append(
                _make_batch(
                    key=f"{batch_level or 'all'}:{category}:{chunk_index // cap}",
                    category=category,
                    level_id=batch_level,
                    triangles=chunk,
                    origin=origin,
                )
            )

    return GpuSceneData(
        batches=batches,
        origin=origin,
        bounds_min=bounds_min,
        bounds_max=bounds_max,
    )


def visible_cells(model: InavModel | None, level_id: str | None) -> list[NavCell]:
    if model is None:
        return []
    if not level_id:
        return list(model.cells)
    return [cell for cell in model.cells if cell.level_id == level_id]


def _spatially_order(triangles: list[TriangleRecord]) -> list[TriangleRecord]:
    """Deterministically Morton-order triangles by centroid.

    Count chunking alone can produce batches whose AABBs span a whole building,
    making frustum culling ineffective. Morton ordering preserves locality with
    minimal preprocessing and no dependency on a renderer-specific spatial tree.
    """

    if len(triangles) <= 1:
        return list(triangles)
    centroids = [
        tuple(sum(vertex[i] for vertex in tri) / 3.0 for i in range(3))
        for tri, _guid in triangles
    ]
    mins = [min(point[i] for point in centroids) for i in range(3)]
    maxs = [max(point[i] for point in centroids) for i in range(3)]

    def quantize(value: float, axis: int) -> int:
        span = maxs[axis] - mins[axis]
        if span <= 1e-12:
            return 0
        return min(1023, max(0, int(round((value - mins[axis]) / span * 1023.0))))

    decorated = []
    for index, (record, centroid) in enumerate(zip(triangles, centroids)):
        qx = quantize(centroid[0], 0)
        qy = quantize(centroid[1], 1)
        qz = quantize(centroid[2], 2)
        decorated.append((_morton3(qx, qy, qz), index, record))
    decorated.sort(key=lambda item: (item[0], item[1]))
    return [record for _key, _index, record in decorated]


def _morton3(x: int, y: int, z: int) -> int:
    value = 0
    for bit in range(10):
        value |= ((x >> bit) & 1) << (3 * bit)
        value |= ((y >> bit) & 1) << (3 * bit + 1)
        value |= ((z >> bit) & 1) << (3 * bit + 2)
    return value


def _make_batch(
    *,
    key: str,
    category: str,
    level_id: str | None,
    triangles: Iterable[TriangleRecord],
    origin: Vec3,
) -> GpuMeshBatch:
    color = _CATEGORY_COLORS.get(category, (0.50, 0.55, 0.61, 1.0))
    vertices = array("f")
    indices = array("I")
    guids: list[str | None] = []
    points: list[Vec3] = []

    for tri, guid in triangles:
        base = len(vertices) // 7
        for point in tri:
            vertices.extend(
                (
                    float(point[0] - origin[0]),
                    float(point[1] - origin[1]),
                    float(point[2] - origin[2]),
                    *color,
                )
            )
            points.append(point)
        indices.extend((base, base + 1, base + 2))
        guids.append(guid)

    if points:
        bmin = tuple(min(point[i] for point in points) for i in range(3))
        bmax = tuple(max(point[i] for point in points) for i in range(3))
    else:
        bmin = (0.0, 0.0, 0.0)
        bmax = (0.0, 0.0, 0.0)

    return GpuMeshBatch(
        key=key,
        category=category,
        level_id=level_id,
        vertices=vertices,
        indices=indices,
        triangle_guids=guids,
        bounds_min=bmin,  # type: ignore[arg-type]
        bounds_max=bmax,  # type: ignore[arg-type]
    )
