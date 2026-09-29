from __future__ import annotations

from ifcpath.model import NavCell
from ifcpath.surface_nav import connect_cells_through_portal


def _cell(cell_id: str, x: float, y: float, space_id: str) -> NavCell:
    return NavCell(
        id=cell_id,
        vertices_m=((x, y, 0.0), (x + 0.9, y, 0.0), (x, y + 0.9, 0.0)),
        space_id=space_id,
        level_id="L1",
        terrain="open",
    )


def test_parallel_doors_do_not_overwrite_the_same_navcell_crossing() -> None:
    cells = [
        _cell("a:0", 0.0, 0.0, "a"),
        _cell("a:1", 0.0, 1.0, "a"),
        _cell("b:0", 1.0, 0.0, "b"),
        _cell("b:1", 1.0, 1.0, "b"),
    ]

    assert connect_cells_through_portal(
        cells,
        point=(0.95, 0.35, 0.0),
        from_space_id="a",
        to_space_id="b",
        portal_id="door:lower",
        level_id="L1",
    )
    assert connect_cells_through_portal(
        cells,
        point=(0.95, 1.35, 0.0),
        from_space_id="a",
        to_space_id="b",
        portal_id="door:upper",
        level_id="L1",
    )

    bound = {
        portal_id
        for cell in cells
        for portal_id in cell.portal_ids.values()
    }
    assert bound == {"door:lower", "door:upper"}


def test_second_door_uses_an_alternate_free_cell_pair_if_nearest_pair_is_occupied() -> None:
    # Both doors are closest to a:0/b:0. The second door must use the next
    # geometrically valid pair rather than silently replacing door:first.
    cells = [
        _cell("a:0", 0.0, 0.0, "a"),
        _cell("a:1", 0.0, 0.8, "a"),
        _cell("b:0", 1.0, 0.0, "b"),
        _cell("b:1", 1.0, 0.8, "b"),
    ]

    assert connect_cells_through_portal(
        cells,
        point=(0.95, 0.25, 0.0),
        from_space_id="a",
        to_space_id="b",
        portal_id="door:first",
        level_id="L1",
    )
    assert connect_cells_through_portal(
        cells,
        point=(0.95, 0.35, 0.0),
        from_space_id="a",
        to_space_id="b",
        portal_id="door:second",
        level_id="L1",
    )

    directed_bindings = [
        (cell.id, neighbor_id, portal_id)
        for cell in cells
        for neighbor_id, portal_id in cell.portal_ids.items()
        if cell.id < neighbor_id
    ]
    assert {binding[2] for binding in directed_bindings} == {"door:first", "door:second"}
