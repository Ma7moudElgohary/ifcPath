from __future__ import annotations

import pytest

from ifcpath.ui.gpu_selection import BimSelectionIndex
from ifcpath.ui.preview_geometry import PreviewElement, PreviewGeometry, PreviewTriangle


def test_selection_bvh_returns_nearest_ifc_element_and_metadata() -> None:
    near = PreviewTriangle(
        ((0.0, 0.0, 5.0), (1.0, 0.0, 5.0), (0.0, 1.0, 5.0)),
        "wall",
        "L1",
        "NEAR",
    )
    far = PreviewTriangle(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        "slab",
        "L1",
        "FAR",
    )
    element = PreviewElement(
        guid="NEAR",
        express_id=42,
        ifc_class="IfcWall",
        category="wall",
        level_id="L1",
        name="Selected wall",
        property_sets={"Pset_WallCommon": {"FireRating": "2HR"}},
    )
    preview = PreviewGeometry(
        triangles=[far, near],
        elements={"NEAR": element},
    )
    index = BimSelectionIndex(preview)

    hit = index.pick((0.2, 0.2, 10.0), (0.0, 0.0, -1.0))

    assert hit is not None
    assert hit.guid == "NEAR"
    assert hit.distance_m == pytest.approx(5.0)
    assert hit.element is element
    assert hit.element.property_sets["Pset_WallCommon"]["FireRating"] == "2HR"
    assert index.bounds_for_guid("NEAR") == ((0.0, 0.0, 5.0), (1.0, 1.0, 5.0))


def test_selection_bvh_respects_level_filter() -> None:
    preview = PreviewGeometry(
        triangles=[
            PreviewTriangle(((0.0, 0.0, 3.0), (1.0, 0.0, 3.0), (0.0, 1.0, 3.0)), "wall", "L2", "L2-WALL"),
            PreviewTriangle(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), "wall", "L1", "L1-WALL"),
        ]
    )
    index = BimSelectionIndex(preview, level_id="L1")

    hit = index.pick((0.2, 0.2, 10.0), (0.0, 0.0, -1.0))

    assert hit is not None
    assert hit.guid == "L1-WALL"


def test_selection_bvh_scales_to_thousands_of_triangles_without_changing_identity() -> None:
    triangles = []
    size = 64
    for y in range(size):
        for x in range(size):
            triangles.append(
                PreviewTriangle(
                    (
                        (float(x), float(y), 0.0),
                        (float(x) + 0.8, float(y), 0.0),
                        (float(x), float(y) + 0.8, 0.0),
                    ),
                    "slab",
                    "L1",
                    f"SLAB-{x}-{y}",
                )
            )
    index = BimSelectionIndex(PreviewGeometry(triangles=triangles), leaf_size=8)

    assert index.triangle_count == 4096
    assert index.element_count == 4096
    hit = index.pick((32.2, 17.2, 20.0), (0.0, 0.0, -1.0))
    assert hit is not None
    assert hit.guid == "SLAB-32-17"
