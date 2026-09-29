from __future__ import annotations

import math

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import NavCell
from ifcpath.surface_funnel import find_surface_funnel_route
from ifcpath.surface_nav import walkable_surface_cells


def _cells_from_floor(floor: Polygon, z: float = 0.0) -> list[NavCell]:
    cdt = build_floor_cdt_navmesh(floor, z, boundary_spacing_m=1.0)
    ids = [f"cell:{index}" for index in range(len(cdt.cells))]
    return [
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id="space:test",
            level_id="level:test",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    ]


def test_global_surface_funnel_is_straight_in_open_rectangle():
    cells = _cells_from_floor(Polygon([(0, 0), (10, 0), (10, 4), (0, 4)]))
    start = (0.5, 2.0, 0.0)
    goal = (9.5, 2.0, 0.0)

    route = find_surface_funnel_route(cells, start, goal)

    assert route is not None
    assert route.points == [start, goal]
    assert abs(route.length_m - 9.0) < 1e-9


def test_global_surface_funnel_matches_known_obstacle_detour():
    cells = _cells_from_floor(
        Polygon(
            [(0, 0), (10, 0), (10, 4), (0, 4)],
            holes=[[(4, 1), (6, 1), (6, 3), (4, 3)]],
        )
    )
    start = (1.0, 2.0, 0.0)
    goal = (9.0, 2.0, 0.0)
    expected = 2.0 * math.sqrt(10.0) + 2.0

    route = find_surface_funnel_route(cells, start, goal)

    assert route is not None
    assert abs(route.length_m - expected) < 1e-6
    assert len(route.points) == 4


def test_global_surface_funnel_preserves_rising_stair_route():
    vertices = [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
        (0.0, 1.0, 1.0), (1.0, 1.0, 1.0),
        (0.0, 2.0, 2.0), (1.0, 2.0, 2.0),
    ]
    cells = walkable_surface_cells(
        vertices,
        [(0, 1, 2), (1, 3, 2), (2, 3, 4), (3, 5, 4)],
        id_prefix="stair",
        terrain="stair",
        max_slope_deg=55,
    )

    route = find_surface_funnel_route(cells, (0.2, 0.1, 0.1), (0.8, 1.9, 1.9))

    assert route is not None
    assert route.points[0][2] < route.points[-1][2]
    assert route.length_m >= math.dist(route.points[0], route.points[-1]) - 1e-9
