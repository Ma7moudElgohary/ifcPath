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
