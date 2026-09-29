from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, Portal, Space
from ifcpath.open_space_adjacency import connect_open_space_boundaries


def _cell(cell_id: str, space_id: str, x0: float, x1: float) -> NavCell:
    return NavCell(
        id=cell_id,
        vertices_m=((x0, 0.0, 0.0), (x1, 0.0, 0.0), (x0, 2.0, 0.0)),
        space_id=space_id,
        level_id="L1",
        terrain="open",
    )


def test_six_millimetre_open_boundary_is_recovered() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1")],
        cells=[
            _cell("a:0", "a", 0.0, 1.0),
            NavCell(
                id="b:0",
                vertices_m=((1.006, 0.0, 0.0), (2.0, 0.0, 0.0), (1.006, 2.0, 0.0)),
                space_id="b",
                level_id="L1",
                terrain="open",
            ),
        ],
    )

    stats = connect_open_space_boundaries(
        model,
        [],
        max_gap_m=0.02,
        max_vertical_gap_m=0.02,
    )

    assert stats.connected == 1
    assert model.cells[1].id in model.cells[0].neighbor_ids
    portal = next(item for item in model.portals if item.kind == "open_boundary")
    assert {portal.from_space_id, portal.to_space_id} == {"a", "b"}


def test_wall_width_gap_is_not_recovered_by_portable_tolerance() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1")],
        cells=[
            _cell("a:0", "a", 0.0, 1.0),
            NavCell(
                id="b:0",
                vertices_m=((1.12, 0.0, 0.0), (2.0, 0.0, 0.0), (1.12, 2.0, 0.0)),
                space_id="b",
                level_id="L1",
                terrain="open",
            ),
        ],
    )

    stats = connect_open_space_boundaries(
        model,
        [],
        max_gap_m=0.02,
        max_vertical_gap_m=0.02,
    )

    assert stats.connected == 0
    assert not model.portals


def test_existing_semantic_portal_prevents_duplicate_open_boundary() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1")],
        cells=[
            _cell("a:0", "a", 0.0, 1.0),
            NavCell(
                id="b:0",
                vertices_m=((1.006, 0.0, 0.0), (2.0, 0.0, 0.0), (1.006, 2.0, 0.0)),
                space_id="b",
                level_id="L1",
                terrain="open",
            ),
        ],
        portals=[
            Portal(
                id="door:D1",
                kind="door",
                position_m=(1.003, 1.0, 0.0),
                from_space_id="a",
                to_space_id="b",
                level_id="L1",
            )
        ],
    )

    stats = connect_open_space_boundaries(
        model,
        [],
        max_gap_m=0.02,
        max_vertical_gap_m=0.02,
    )

    assert stats.connected == 0
    assert [item.id for item in model.portals] == ["door:D1"]
