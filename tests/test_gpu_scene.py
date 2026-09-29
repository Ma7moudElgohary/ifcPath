from __future__ import annotations

import pytest

from ifcpath.model import InavModel, Level, NavCell, Space
from ifcpath.ui.gpu_scene import build_gpu_scene
from ifcpath.ui.preview_geometry import PreviewGeometry, PreviewTriangle


def _model() -> InavModel:
    level = Level("L1", "Ground", 100.0)
    space = Space("space:A", "A", level.id)
    cell = NavCell(
        id="cell:A",
        vertices_m=((1_000_000.0, 2_000_000.0, 100.0), (1_000_010.0, 2_000_000.0, 100.0), (1_000_000.0, 2_000_010.0, 100.0)),
        space_id=space.id,
        level_id=level.id,
    )
    return InavModel(levels=[level], spaces=[space], cells=[cell])


def test_gpu_scene_rebases_large_ifc_coordinates_around_local_origin() -> None:
    model = _model()
    bim = PreviewGeometry(
        triangles=[
            PreviewTriangle(
                vertices_m=((1_000_000.0, 2_000_000.0, 100.0), (1_000_010.0, 2_000_000.0, 100.0), (1_000_000.0, 2_000_010.0, 103.0)),
                category="wall",
                level_id="L1",
                ifc_guid="WALL-1",
            )
        ]
    )

    scene = build_gpu_scene(model, bim, level_id="L1")

    assert scene.triangle_count == 2
    assert scene.origin[0] == pytest.approx(1_000_005.0)
    assert scene.origin[1] == pytest.approx(2_000_005.0)
    assert scene.origin[2] == pytest.approx(101.5)
    local_xyz = []
    for batch in scene.batches:
        for i in range(0, len(batch.vertices), 7):
            local_xyz.append(tuple(batch.vertices[i : i + 3]))
    assert max(abs(value) for point in local_xyz for value in point) <= 5.1


def test_gpu_scene_chunks_batches_deterministically() -> None:
    model = _model()
    triangles = [
        PreviewTriangle(
            vertices_m=((float(i), 0.0, 0.0), (float(i) + 0.5, 0.0, 0.0), (float(i), 0.5, 0.0)),
            category="wall",
            level_id="L1",
            ifc_guid=f"W{i}",
        )
        for i in range(5)
    ]
    scene = build_gpu_scene(model, PreviewGeometry(triangles=triangles), show_navmesh=False, max_triangles_per_batch=2)

    wall_batches = [batch for batch in scene.batches if batch.category == "wall"]
    assert [batch.triangle_count for batch in wall_batches] == [2, 2, 1]
    assert [guid for batch in wall_batches for guid in batch.triangle_guids] == ["W0", "W1", "W2", "W3", "W4"]
