from __future__ import annotations

import math
from collections import defaultdict

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
