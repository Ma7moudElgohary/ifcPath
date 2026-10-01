from __future__ import annotations

import math
from collections import defaultdict

from ifcpath.heightfield_repair import rejected_bridge_groups
from ifcpath.raycast_surface import (
    SurfaceDetectionOptions,
    _SupportSample,
    _WalkableSpan,
)
from ifcpath.triangle_heightfield import (
    _barycentric_xy,
    _rasterize_triangle,
    _triangle_normal,
)


def test_barycentric_xy_interpolates_inside_horizontal_triangle():
    a = (0.0, 0.0, 2.0)
    b = (1.0, 0.0, 2.0)
    c = (0.0, 1.0, 2.0)
    area2 = 1.0

    weights = _barycentric_xy(0.25, 0.25, a, b, c, area2)

    assert weights is not None
    assert all(value >= 0.0 for value in weights)
    assert math.isclose(sum(weights), 1.0)
    z = sum(weight * vertex[2] for weight, vertex in zip(weights, (a, b, c)))
    assert math.isclose(z, 2.0)


def test_triangle_normal_preserves_walkable_up_component_magnitude():
    normal = _triangle_normal(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.2),
    )

    assert normal is not None
    assert abs(normal[2]) > 0.9


def test_horizontal_triangle_rasterizes_interpolated_surface_hit():
    columns = defaultdict(list)
    sample_keys = {(0, 0), (1, 0), (0, 1), (1, 1)}

    _rasterize_triangle(
        columns,
        sample_keys,
        (0, 0, 1, 1),
        10,
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (0.0, 1.0, 1.0),
        1.0,
    )

    assert (0, 0) in columns
    assert columns[(0, 0)][0].entity_id == 10
    assert math.isclose(columns[(0, 0)][0].position[2], 1.0)


def test_vertical_triangle_does_not_manufacture_column_intersection():
    columns = defaultdict(list)
    sample_keys = {(0, 0), (1, 0)}

    _rasterize_triangle(
        columns,
        sample_keys,
        (0, 0, 1, 0),
        20,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 2.0),
        (0.0, 1.0, 0.0),
        1.0,
    )

    assert dict(columns) == {}


def _sample(ix: int, iy: int, z: float = 0.0) -> _SupportSample:
    return _SupportSample(ix, iy, (float(ix), float(iy), z), 1, "open")


def _span(ix: int, iy: int, z: float = 0.0) -> _WalkableSpan:
    return _WalkableSpan(
        ix=ix,
        iy=iy,
        position=(float(ix), float(iy), z),
        owner_id=1,
        terrain="open",
        ceiling_z=None,
        free_height_m=math.inf,
        direct_blocked=False,
    )


def test_two_cell_rejected_chain_is_selected_when_it_bridges_components():
    opts = SurfaceDetectionOptions(cell_size_m=1.0, max_climb_m=0.25)
    accepted = [_sample(0, 0), _sample(0, 1), _sample(3, 0), _sample(3, 1)]
    rejected = [_span(1, 0), _span(2, 0)]

    groups = rejected_bridge_groups(rejected, accepted, opts)

    assert len(groups) == 1
    assert [(span.ix, span.iy) for span in groups[0]] == [(1, 0), (2, 0)]


def test_rejected_chain_touching_only_one_component_is_not_repaired():
    opts = SurfaceDetectionOptions(cell_size_m=1.0, max_climb_m=0.25)
    accepted = [_sample(0, 0), _sample(0, 1)]
    rejected = [_span(1, 0), _span(2, 0)]

    assert rejected_bridge_groups(rejected, accepted, opts) == []


def test_rejected_chain_does_not_bridge_different_storeys():
    opts = SurfaceDetectionOptions(cell_size_m=1.0, max_climb_m=0.25)
    accepted = [_sample(0, 0, 0.0), _sample(3, 0, 3.0)]
    rejected = [_span(1, 0, 0.0), _span(2, 0, 0.0)]

    assert rejected_bridge_groups(rejected, accepted, opts) == []
