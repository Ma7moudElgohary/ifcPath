from __future__ import annotations

import math

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import InavModel, NavCell
from ifcpath.navmesh_routing import find_navmesh_path


def _model_from_floor(floor: Polygon, z: float = 0.0) -> InavModel:
    cdt = build_floor_cdt_navmesh(floor, z, boundary_spacing_m=1.0)
    ids = [f"cell:{index}" for index in range(len(cdt.cells))]
    model = InavModel()
    model.cells = [
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id="space:test",
            level_id="level:test",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    ]
    return model


def test_funnel_returns_exact_straight_path_in_open_rectangle():
    floor = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (0.0, 4.0)])
    model = _model_from_floor(floor)
    start = (0.5, 2.0, 0.0)
    goal = (9.5, 2.0, 0.0)

    route = find_navmesh_path(model, start, goal, space_id="space:test")

    assert route is not None
    assert route.points == [start, goal]
    assert abs(route.length_m - 9.0) < 1e-9


def test_funnel_matches_known_shortest_detour_around_rectangular_obstacle():
    # Straight line y=2 is blocked by the 2x2 central obstacle. The Euclidean
    # optimum touches either the lower or upper two obstacle corners:
    # sqrt(3^2+1^2) + 2 + sqrt(3^2+1^2).
    floor = Polygon(
        [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (0.0, 4.0)],
        holes=[[(4.0, 1.0), (6.0, 1.0), (6.0, 3.0), (4.0, 3.0)]],
    )
    model = _model_from_floor(floor)
    start = (1.0, 2.0, 0.0)
    goal = (9.0, 2.0, 0.0)
    expected = 2.0 * math.sqrt(10.0) + 2.0

    route = find_navmesh_path(model, start, goal, space_id="space:test")

    assert route is not None
    assert abs(route.length_m - expected) < 1e-6
    assert len(route.points) == 4


def test_funnel_route_is_materially_shorter_than_centroid_chain():
    floor = Polygon(
        [(0.0, 0.0), (12.0, 0.0), (12.0, 6.0), (0.0, 6.0)],
        holes=[[(5.0, 1.0), (7.0, 1.0), (7.0, 5.0), (5.0, 5.0)]],
    )
    cdt = build_floor_cdt_navmesh(floor, 0.0)
    ids = [f"cell:{index}" for index in range(len(cdt.cells))]
    model = InavModel(cells=[
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id="space:test",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    ])

    start = (1.0, 3.0, 0.0)
    goal = (11.0, 3.0, 0.0)
    route = find_navmesh_path(model, start, goal, space_id="space:test")

    assert route is not None
    # Known optimum around either side of the obstacle.
    expected = 2.0 * math.sqrt(17.0) + 2.0
    assert route.length_m <= expected * 1.000001
