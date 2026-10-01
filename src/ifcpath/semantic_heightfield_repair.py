from __future__ import annotations

from dataclasses import dataclass

import ifcopenshell.geom
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from .cdt import space_floor_polygon
from .heightfield_repair import rejected_bridge_paths
from .raycast_surface import SurfaceDetectionOptions, _SupportSample, _WalkableSpan


@dataclass(frozen=True, slots=True)
class SemanticRepairRegion:
    id: str
    area_m2: float
    min_z: float
    max_z: float
    polygon: BaseGeometry

    def contains(self, position, *, vertical_tolerance_m: float = 0.35) -> bool:
        z = float(position[2])
        if z < self.min_z - vertical_tolerance_m or z > self.max_z + vertical_tolerance_m:
            return False
        return bool(self.polygon.covers(Point(float(position[0]), float(position[1]))))


def build_semantic_repair_regions(ifc_file) -> list[SemanticRepairRegion]:
    """Build IfcSpace floor regions used only to target exact verification.

    These regions never create or elevate navigation geometry. They only answer:
    'which already-detected support samples belong to the same authored room?' A
    rejected sample can be restored only after passing exact physical OCC body
    clearance, so semantic geometry cannot punch through a wall or obstacle.
    """
    try:
        spaces = list(ifc_file.by_type("IfcSpace"))
    except Exception:
        return []
    if not spaces:
        return []

    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    regions: list[SemanticRepairRegion] = []

    try:
        iterator = ifcopenshell.geom.iterator(
            settings,
            ifc_file,
            1,
            include=spaces,
        )
        initialized = iterator.initialize()
    except Exception:
        iterator = None
        initialized = False

    yielded: set[int] = set()
    if initialized and iterator is not None:
        while True:
            shape = iterator.get()
            entity_id = _shape_entity_id(shape)
            if entity_id is not None:
                region = _region_from_shape(ifc_file, entity_id, shape)
                if region is not None:
                    yielded.add(entity_id)
                    regions.append(region)
            if not iterator.next():
                break

    # Preserve robust behaviour when the bulk iterator cannot tessellate a space.
    for entity in spaces:
        try:
            entity_id = int(entity.id())
        except Exception:
            continue
        if entity_id in yielded:
            continue
        try:
            shape = ifcopenshell.geom.create_shape(settings, entity)
        except Exception:
            continue
        region = _region_from_shape(ifc_file, entity_id, shape)
        if region is not None:
            regions.append(region)

    # Smallest containing authored space wins for malformed/nested spaces, matching
    # the final semantic labelling rule used by surface reconstruction.
    regions.sort(key=lambda item: (item.area_m2, item.id))
    return regions


def semantic_bridge_paths(
    regions: list[SemanticRepairRegion],
    rejected: list[_WalkableSpan],
    accepted: list[_SupportSample],
    opts: SurfaceDetectionOptions,
    *,
    max_path_spans: int = 16,
) -> list[list[_WalkableSpan]]:
    """Find short radius-rejected paths splitting one semantic room."""
    if not regions or not rejected or not accepted:
        return []

    accepted_by_region: dict[str, list[_SupportSample]] = {region.id: [] for region in regions}
    rejected_by_region: dict[str, list[_WalkableSpan]] = {region.id: [] for region in regions}

    for sample in accepted:
        region = _smallest_region(regions, sample.position)
        if region is not None:
            accepted_by_region[region.id].append(sample)

    for span in rejected:
        # Keep vertical terrain out of room-repair logic. Stair/ramp topology is
        # qualified independently and should not be altered by an IfcSpace floor.
        if span.terrain != "open":
            continue
        region = _smallest_region(regions, span.position)
        if region is not None:
            rejected_by_region[region.id].append(span)

    paths: list[list[_WalkableSpan]] = []
    for region in regions:
        room_paths = rejected_bridge_paths(
            rejected_by_region.get(region.id, []),
            accepted_by_region.get(region.id, []),
            opts,
            max_path_spans=max_path_spans,
        )
        paths.extend(room_paths)

    # A span on a shared/nested boundary can theoretically appear through malformed
    # space geometry. Deduplicate paths by their quantized coordinates.
    unique = {}
    tolerance = opts.hit_merge_tolerance_m
    scale = 1.0 / max(1e-6, tolerance)
    for path in paths:
        signature = frozenset(
            (span.ix, span.iy, round(span.position[2] * scale))
            for span in path
        )
        unique.setdefault(signature, path)
    return sorted(
        unique.values(),
        key=lambda path: (len(path), [(span.ix, span.iy, span.position[2]) for span in path]),
    )


def _smallest_region(regions, position):
    for region in regions:
        if region.contains(position):
            return region
    return None


def _region_from_shape(ifc_file, entity_id: int, shape) -> SemanticRepairRegion | None:
    vertices, triangles = _shape_mesh(shape)
    if not vertices:
        return None
    polygon = space_floor_polygon(vertices, triangles)
    if polygon is None or polygon.is_empty:
        return None
    try:
        entity = ifc_file.by_id(entity_id)
        guid = str(getattr(entity, "GlobalId", entity_id))
    except Exception:
        guid = str(entity_id)
    zs = [vertex[2] for vertex in vertices]
    return SemanticRepairRegion(
        id=f"space:{guid}",
        area_m2=float(polygon.area),
        min_z=min(zs),
        max_z=max(zs),
        polygon=polygon,
    )


def _shape_entity_id(shape) -> int | None:
    value = getattr(shape, "id", None)
    try:
        return int(value() if callable(value) else value)
    except (TypeError, ValueError):
        return None


def _shape_mesh(shape):
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
