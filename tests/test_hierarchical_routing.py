from __future__ import annotations

import math

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.hierarchical_routing import HierarchicalRouteOptions, find_hierarchical_path
from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space


def _add_rect_space(
    model: InavModel,
    space_id: str,
    *,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    level_id: str,
) -> None:
    model.spaces.append(Space(space_id, space_id, level_id))
    cdt = build_floor_cdt_navmesh(
        Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)]),
        z,
    )
    cell_ids = [f"cell:{space_id}:{index}" for index in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=cell_ids[index],
            vertices_m=cell.vertices,
            space_id=space_id,
            level_id=level_id,
            neighbor_ids=[cell_ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    )


def test_hierarchical_route_selects_geometrically_closer_door():
    model = InavModel(levels=[Level("L1", "L1", 0.0)])
    _add_rect_space(model, "A", x0=0.0, y0=0.0, x1=5.0, y1=10.0, z=0.0, level_id="L1")
    _add_rect_space(model, "B", x0=5.0, y0=0.0, x1=10.0, y1=10.0, z=0.0, level_id="L1")
    model.portals = [
        Portal("door:near", "door", (5.0, 1.0, 0.0), "A", "B", "L1"),
        Portal("door:far", "door", (5.0, 9.0, 0.0), "A", "B", "L1"),
    ]

    route = find_hierarchical_path(model, (1.0, 1.0, 0.0), (9.0, 1.0, 0.0))

    assert route is not None
    assert route.transition_ids == ["transition:door:near"]
    assert route.space_ids == ["A", "B"]
    assert abs(route.length_m - 8.0) < 1e-9


def test_blocked_portal_forces_alternate_door():
    model = InavModel(levels=[Level("L1", "L1", 0.0)])
    _add_rect_space(model, "A", x0=0.0, y0=0.0, x1=5.0, y1=10.0, z=0.0, level_id="L1")
    _add_rect_space(model, "B", x0=5.0, y0=0.0, x1=10.0, y1=10.0, z=0.0, level_id="L1")
    model.portals = [
        Portal("door:near", "door", (5.0, 1.0, 0.0), "A", "B", "L1"),
        Portal("door:far", "door", (5.0, 9.0, 0.0), "A", "B", "L1"),
    ]

    route = find_hierarchical_path(
        model,
        (1.0, 1.0, 0.0),
        (9.0, 1.0, 0.0),
        HierarchicalRouteOptions(blocked_portals={"door:near"}),
    )

    assert route is not None
    assert route.transition_ids == ["transition:door:far"]
    expected = 2.0 * math.sqrt(80.0)
    assert abs(route.length_m - expected) < 1e-6


def test_space_cost_multiplier_changes_building_route():
    model = InavModel(levels=[Level("L1", "L1", 0.0)])
    _add_rect_space(model, "A", x0=0.0, y0=0.0, x1=4.0, y1=8.0, z=0.0, level_id="L1")
    _add_rect_space(model, "B", x0=4.0, y0=0.0, x1=8.0, y1=4.0, z=0.0, level_id="L1")
    _add_rect_space(model, "D", x0=4.0, y0=4.0, x1=8.0, y1=8.0, z=0.0, level_id="L1")
    _add_rect_space(model, "C", x0=8.0, y0=0.0, x1=12.0, y1=8.0, z=0.0, level_id="L1")
    model.portals = [
        Portal("door:AB", "door", (4.0, 2.0, 0.0), "A", "B", "L1"),
        Portal("door:BC", "door", (8.0, 2.0, 0.0), "B", "C", "L1"),
        Portal("door:AD", "door", (4.0, 6.0, 0.0), "A", "D", "L1"),
        Portal("door:DC", "door", (8.0, 6.0, 0.0), "D", "C", "L1"),
    ]

    normal = find_hierarchical_path(model, (1.0, 2.0, 0.0), (11.0, 2.0, 0.0))
    penalized = find_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (11.0, 2.0, 0.0),
        HierarchicalRouteOptions(space_cost_multipliers={"B": 10.0}),
    )

    assert normal is not None
    assert penalized is not None
    assert normal.space_ids == ["A", "B", "C"]
    assert penalized.space_ids == ["A", "D", "C"]
    assert normal.length_m < penalized.length_m
    assert penalized.weighted_cost < 43.0


def test_stacked_floors_use_z_aware_cells_and_actual_stair_geometry():
    model = InavModel(levels=[Level("L1", "L1", 0.0), Level("L2", "L2", 3.0)])
    _add_rect_space(model, "lower", x0=0.0, y0=0.0, x1=4.0, y1=4.0, z=0.0, level_id="L1")
    _add_rect_space(model, "upper", x0=0.0, y0=0.0, x1=4.0, y1=4.0, z=3.0, level_id="L2")
    model.nodes = [
        NavNode("lower-landing", (2.0, 2.0, 0.0), kind="walk", level_id="L1", space_id="lower"),
        NavNode("stair-0", (2.0, 2.0, 0.2), kind="stair"),
        NavNode("stair-1", (2.0, 2.0, 2.8), kind="stair"),
        NavNode("upper-landing", (2.0, 2.0, 3.0), kind="walk", level_id="L2", space_id="upper"),
    ]
    model.edges = [
        NavEdge("lower-landing", "stair-0", 0.2),
        NavEdge("stair-0", "stair-1", 2.6),
        NavEdge("stair-1", "upper-landing", 0.2),
    ]

    route = find_hierarchical_path(model, (1.0, 2.0, 0.0), (3.0, 2.0, 3.0))

    assert route is not None
    assert route.space_ids == ["lower", "upper"]
    assert len(route.transition_ids) == 1
    assert any(segment.kind == "stair" for segment in route.segments)
    assert any(abs(point[2] - 0.2) < 1e-9 for point in route.points)
    assert any(abs(point[2] - 2.8) < 1e-9 for point in route.points)
    assert abs(route.length_m - 5.0) < 1e-9


def test_blocked_destination_space_is_not_entered():
    model = InavModel(levels=[Level("L1", "L1", 0.0)])
    _add_rect_space(model, "A", x0=0.0, y0=0.0, x1=5.0, y1=4.0, z=0.0, level_id="L1")
    _add_rect_space(model, "B", x0=5.0, y0=0.0, x1=10.0, y1=4.0, z=0.0, level_id="L1")
    model.portals = [Portal("door:AB", "door", (5.0, 2.0, 0.0), "A", "B", "L1")]

    route = find_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (9.0, 2.0, 0.0),
        HierarchicalRouteOptions(blocked_spaces={"B"}),
    )

    assert route is None
