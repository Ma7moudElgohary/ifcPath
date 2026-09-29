from __future__ import annotations

from array import array
from dataclasses import dataclass, field
from typing import Iterable

from ..model import InavModel, NavCell, Vec3
from .preview_geometry import PreviewGeometry, PreviewTriangle


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
    """CPU-side immutable-ish mesh payload ready for one GPU upload.

    Vertices are interleaved as xyz + rgba (7 float32 values). Batches are
    intentionally grouped by semantic category and level so later frustum/LOD
    and streaming decisions can operate without reparsing the IFC.
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
    bounds_min: Vec3
    bounds_max: Vec3

    @property
    def triangle_count(self) -> int:
        return sum(batch.triangle_count for batch in self.batches)

    @property
    def byte_size(self) -> int:
        return sum(batch.byte_size for batch in self.batches)


def build_gpu_scene(
    model: InavModel | None,
    bim: PreviewGeometry | None,
    *,
    level_id: str | None = None,
    show_bim: bool = True,
    show_navmesh: bool = True,
    max_triangles_per_batch: int = 24_000,
) -> GpuSceneData:
    groups: dict[tuple[str | None, str], list[tuple[tuple[Vec3, Vec3, Vec3], str | None]]] = {}

    if show_bim and bim is not None:
        for tri in bim.triangles:
            if level_id and tri.level_id and tri.level_id != level_id:
                continue
            groups.setdefault((tri.level_id, tri.category), []).append((tri.vertices_m, tri.ifc_guid))

    if show_navmesh and model is not None:
        for cell in model.cells:
            if level_id and cell.level_id != level_id:
                continue
            groups.setdefault((cell.level_id, "navmesh"), []).append((cell.vertices_m, None))

    batches: list[GpuMeshBatch] = []
    all_points: list[Vec3] = []
    cap = max(1, int(max_triangles_per_batch))
    for (batch_level, category), triangles in sorted(groups.items(), key=lambda item: ((item[0][0] or ""), item[0][1])):
        for chunk_index in range(0, len(triangles), cap):
            chunk = triangles[chunk_index : chunk_index + cap]
            batch = _make_batch(
                key=f"{batch_level or 'all'}:{category}:{chunk_index // cap}",
                category=category,
                level_id=batch_level,
                triangles=chunk,
            )
            batches.append(batch)
            all_points.extend((batch.bounds_min, batch.bounds_max))

    if not all_points:
        return GpuSceneData(batches=[], bounds_min=(0.0, 0.0, 0.0), bounds_max=(1.0, 1.0, 1.0))

    return GpuSceneData(
        batches=batches,
        bounds_min=tuple(min(p[i] for p in all_points) for i in range(3)),  # type: ignore[arg-type]
        bounds_max=tuple(max(p[i] for p in all_points) for i in range(3)),  # type: ignore[arg-type]
    )


def visible_cells(model: InavModel | None, level_id: str | None) -> list[NavCell]:
    if model is None:
        return []
    if not level_id:
        return list(model.cells)
    return [cell for cell in model.cells if cell.level_id == level_id]


def _make_batch(
    *,
    key: str,
    category: str,
    level_id: str | None,
    triangles: Iterable[tuple[tuple[Vec3, Vec3, Vec3], str | None]],
) -> GpuMeshBatch:
    color = _CATEGORY_COLORS.get(category, (0.50, 0.55, 0.61, 1.0))
    vertices = array("f")
    indices = array("I")
    guids: list[str | None] = []
    points: list[Vec3] = []

    for tri, guid in triangles:
        base = len(vertices) // 7
        for point in tri:
            vertices.extend((float(point[0]), float(point[1]), float(point[2]), *color))
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
