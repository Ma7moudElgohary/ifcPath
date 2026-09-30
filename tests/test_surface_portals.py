from __future__ import annotations

from ifcpath.model import InavModel, NavCell, Portal
from ifcpath.routing import find_path_xyz
from ifcpath.surface_portals import bind_semantic_surface_portals


def model_with_door() -> InavModel:
    return InavModel(
        cells=[
            NavCell(
                id="a",
                vertices_m=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                space_id="room-a",
                level_id="L1",
            ),
            NavCell(
                id="b",
                vertices_m=((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 1.0, 0.0)),
                space_id="room-b",
                level_id="L1",
            ),
        ],
        portals=[
            Portal(
                id="door-1",
                kind="door",
                position_m=(1.0, 0.5, 0.0),
                from_space_id="room-a",
                to_space_id="room-b",
                level_id="L1",
                width_m=0.9,
            )
        ],
    )


def test_door_authorizes_surface_crossing_and_is_idempotent():
    model = model_with_door()
    assert bind_semantic_surface_portals(model) == 1
    assert bind_semantic_surface_portals(model) == 0
    a, b = model.cells
    assert b.id in a.neighbor_ids and a.id in b.neighbor_ids
    assert a.portal_ids[b.id] == "door-1"
    assert b.portal_ids[a.id] == "door-1"
    assert b.id in a.portals and a.id in b.portals


def test_surface_router_can_block_semantic_door():
    model = model_with_door()
    route = find_path_xyz(model, (0.2, 0.2, 0.0), (1.8, 0.2, 0.0))
    assert len(route) >= 2
    blocked = find_path_xyz(
        model,
        (0.2, 0.2, 0.0),
        (1.8, 0.2, 0.0),
        blocked_portals={"door-1"},
    )
    assert blocked == []


def test_door_prefers_substantial_component_over_tiny_grid_island_when_both_are_local():
    main_a = NavCell(
        id="a-main-1",
        vertices_m=((0.0, 0.0, 0.0), (0.55, 0.0, 0.0), (0.0, 0.5, 0.0)),
        space_id="room-a",
        level_id="L1",
        neighbor_ids=["a-main-2"],
    )
    main_a_2 = NavCell(
        id="a-main-2",
        vertices_m=((0.55, 0.0, 0.0), (0.82, 0.0, 0.0), (0.55, 0.5, 0.0)),
        space_id="room-a",
        level_id="L1",
        neighbor_ids=["a-main-1"],
    )
    tiny_threshold = NavCell(
        id="a-tiny",
        vertices_m=((0.92, 0.0, 0.0), (1.00, 0.0, 0.0), (0.92, 0.08, 0.0)),
        space_id="room-a",
        level_id="L1",
    )
    room_b = NavCell(
        id="b",
        vertices_m=((1.02, 0.0, 0.0), (1.70, 0.0, 0.0), (1.02, 0.60, 0.0)),
        space_id="room-b",
        level_id="L1",
    )
    model = InavModel(
        cells=[main_a, main_a_2, tiny_threshold, room_b],
        portals=[
            Portal(
                id="door",
                kind="door",
                position_m=(1.0, 0.04, 0.0),
                from_space_id="room-a",
                to_space_id="room-b",
                level_id="L1",
                width_m=0.9,
            )
        ],
    )

    assert bind_semantic_surface_portals(model) == 1

    assert tiny_threshold.portal_ids == {}
    assert main_a_2.portal_ids.get("b") == "door"
    assert room_b.portal_ids.get("a-main-2") == "door"


def test_door_keeps_nearest_small_component_when_main_floor_is_outside_local_band():
    far_main = NavCell(
        id="a-main",
        vertices_m=((0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.0, 0.3, 0.0)),
        space_id="room-a",
        level_id="L1",
    )
    threshold = NavCell(
        id="a-threshold",
        vertices_m=((1.9, 0.0, 0.0), (2.0, 0.0, 0.0), (1.9, 0.1, 0.0)),
        space_id="room-a",
        level_id="L1",
    )
    room_b = NavCell(
        id="b",
        vertices_m=((2.02, 0.0, 0.0), (2.7, 0.0, 0.0), (2.02, 0.6, 0.0)),
        space_id="room-b",
        level_id="L1",
    )
    model = InavModel(
        cells=[far_main, threshold, room_b],
        portals=[
            Portal(
                id="door",
                kind="door",
                position_m=(2.0, 0.05, 0.0),
                from_space_id="room-a",
                to_space_id="room-b",
                level_id="L1",
                width_m=0.6,
            )
        ],
    )

    assert bind_semantic_surface_portals(model) == 1

    assert threshold.portal_ids.get("b") == "door"
    assert far_main.portal_ids == {}
