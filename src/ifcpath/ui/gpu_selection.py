from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from ..model import Vec3
from .gpu_camera import ray_triangle_distance
from .preview_geometry import PreviewElement, PreviewGeometry, PreviewTriangle


@dataclass(slots=True, frozen=True)
class ElementSelectionHit:
    guid: str
    point_m: Vec3
    distance_m: float
    category: str
    level_id: str | None
    element: PreviewElement | None


@dataclass(slots=True)
class _TriangleRecord:
    triangle: PreviewTriangle
    bounds_min: Vec3
    bounds_max: Vec3
    centroid: Vec3


@dataclass(slots=True)
class _BvhNode:
    bounds_min: Vec3
    bounds_max: Vec3
    left: _BvhNode | None = None
    right: _BvhNode | None = None
    triangle_indices: tuple[int, ...] = ()

    @property
    def is_leaf(self) -> bool:
        return self.left is None and self.right is None


class BimSelectionIndex:
    """CPU BVH for scalable click selection of IFC preview products."""

    def __init__(
        self,
        preview: PreviewGeometry | None,
        *,
        level_id: str | None = None,
        leaf_size: int = 12,
    ) -> None:
        self.preview = preview
        self.level_id = level_id
        self.leaf_size = max(2, int(leaf_size))
        self._records: list[_TriangleRecord] = []
        self._triangles_by_guid: dict[str, list[PreviewTriangle]] = {}

        if preview is not None:
            for triangle in preview.triangles:
                if not triangle.ifc_guid:
                    continue
                if level_id and triangle.level_id and triangle.level_id != level_id:
                    continue
                record = _record(triangle)
                self._records.append(record)
                self._triangles_by_guid.setdefault(triangle.ifc_guid, []).append(triangle)

        self._root = self._build(tuple(range(len(self._records)))) if self._records else None

    @property
    def triangle_count(self) -> int:
        return len(self._records)

    @property
    def element_count(self) -> int:
        return len(self._triangles_by_guid)

    def triangles_for_guid(self, guid: str | None) -> list[PreviewTriangle]:
        if not guid:
            return []
        return list(self._triangles_by_guid.get(guid, ()))

    def bounds_for_guid(self, guid: str | None) -> tuple[Vec3, Vec3] | None:
        triangles = self.triangles_for_guid(guid)
        if not triangles:
            return None
        points = [point for triangle in triangles for point in triangle.vertices_m]
        bounds_min: Vec3 = tuple(min(point[axis] for point in points) for axis in range(3))  # type: ignore[assignment]
        bounds_max: Vec3 = tuple(max(point[axis] for point in points) for axis in range(3))  # type: ignore[assignment]
        return bounds_min, bounds_max

    def pick(self, origin_m: Vec3, direction: Vec3) -> ElementSelectionHit | None:
        if self._root is None:
            return None
        direction = _normalized(direction)
        if direction is None:
            return None

        best_distance = math.inf
        best_record: _TriangleRecord | None = None
        stack = [self._root]
        while stack:
            node = stack.pop()
            if not _ray_intersects_aabb(origin_m, direction, node.bounds_min, node.bounds_max, best_distance):
                continue
            if node.is_leaf:
                for index in node.triangle_indices:
                    record = self._records[index]
                    distance = ray_triangle_distance(origin_m, direction, record.triangle.vertices_m)
                    if distance is not None and 0.0 <= distance < best_distance:
                        best_distance = distance
                        best_record = record
                continue
            if node.left is not None:
                stack.append(node.left)
            if node.right is not None:
                stack.append(node.right)

        if best_record is None or not best_record.triangle.ifc_guid:
            return None
        triangle = best_record.triangle
        hit = (
            origin_m[0] + direction[0] * best_distance,
            origin_m[1] + direction[1] * best_distance,
            origin_m[2] + direction[2] * best_distance,
        )
        element = self.preview.element_for_guid(triangle.ifc_guid) if self.preview else None
        return ElementSelectionHit(
            guid=triangle.ifc_guid,
            point_m=hit,
            distance_m=best_distance,
            category=triangle.category,
            level_id=triangle.level_id,
            element=element,
        )

    def _build(self, indices: tuple[int, ...]) -> _BvhNode:
        bounds_min, bounds_max = _combined_bounds(self._records[index] for index in indices)
        if len(indices) <= self.leaf_size:
            return _BvhNode(bounds_min=bounds_min, bounds_max=bounds_max, triangle_indices=indices)

        extents = tuple(bounds_max[axis] - bounds_min[axis] for axis in range(3))
        axis = max(range(3), key=lambda candidate: extents[candidate])
        ordered = tuple(sorted(indices, key=lambda index: self._records[index].centroid[axis]))
        midpoint = len(ordered) // 2
        if midpoint <= 0 or midpoint >= len(ordered):
            return _BvhNode(bounds_min=bounds_min, bounds_max=bounds_max, triangle_indices=indices)
        return _BvhNode(
            bounds_min=bounds_min,
            bounds_max=bounds_max,
            left=self._build(ordered[:midpoint]),
            right=self._build(ordered[midpoint:]),
        )


def _record(triangle: PreviewTriangle) -> _TriangleRecord:
    vertices = triangle.vertices_m
    bounds_min: Vec3 = tuple(min(point[axis] for point in vertices) for axis in range(3))  # type: ignore[assignment]
    bounds_max: Vec3 = tuple(max(point[axis] for point in vertices) for axis in range(3))  # type: ignore[assignment]
    centroid: Vec3 = tuple(sum(point[axis] for point in vertices) / 3.0 for axis in range(3))  # type: ignore[assignment]
    return _TriangleRecord(triangle, bounds_min, bounds_max, centroid)


def _combined_bounds(records: Iterable[_TriangleRecord]) -> tuple[Vec3, Vec3]:
    records = list(records)
    bounds_min: Vec3 = tuple(min(record.bounds_min[axis] for record in records) for axis in range(3))  # type: ignore[assignment]
    bounds_max: Vec3 = tuple(max(record.bounds_max[axis] for record in records) for axis in range(3))  # type: ignore[assignment]
    return bounds_min, bounds_max


def _normalized(direction: Vec3) -> Vec3 | None:
    length = math.sqrt(sum(value * value for value in direction))
    if length <= 1e-12:
        return None
    return direction[0] / length, direction[1] / length, direction[2] / length


def _ray_intersects_aabb(
    origin: Vec3,
    direction: Vec3,
    bounds_min: Vec3,
    bounds_max: Vec3,
    max_distance: float,
) -> bool:
    t_min = 0.0
    t_max = max_distance
    for axis in range(3):
        d = direction[axis]
        if abs(d) <= 1e-12:
            if origin[axis] < bounds_min[axis] or origin[axis] > bounds_max[axis]:
                return False
            continue
        inv = 1.0 / d
        t1 = (bounds_min[axis] - origin[axis]) * inv
        t2 = (bounds_max[axis] - origin[axis]) * inv
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return False
    return t_max >= 0.0
