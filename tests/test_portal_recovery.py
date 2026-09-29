from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, Portal, SemanticTransition, Space
from ifcpath.portal_recovery import qualify_surface_portal_sides


def _floor(cell_id: str, space_id: str, level_id: str, vertices) -> NavCell:
    return NavCell(
        id=cell_id,
        vertices_m=vertices,
        space_id=space_id,
        level_id=level_id,
        terrain="open",
    )


def test_wrong_explicit_door_side_is_repaired_from_floor_surface() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[
            Space("a", "A", "L1"),
            Space("b", "B", "L1"),
            Space("c", "C", "L1"),
        ],
        cells=[
            _floor("a:0", "a", "L1", ((-1.0, -1.0, 0.0), (0.0, -1.0, 0.0), (0.0, 1.0, 0.0))),
            _floor("b:0", "b", "L1", ((0.0, -1.0, 0.0), (1.0, -1.0, 0.0), (0.0, 1.0, 0.0))),
            _floor("c:0", "c", "L1", ((1.5, -1.0, 0.0), (2.5, -1.0, 0.0), (1.5, 1.0, 0.0))),
        ],
        portals=[
            Portal(
                id="door:D1",
                kind="door",
                position_m=(0.0, 0.0, 0.0),
                from_space_id="a",
                to_space_id="c",  # stale IFC boundary relation
                level_id="L1",
            )
        ],
        transitions=[
            SemanticTransition(
                id="transition:door:D1",
                kind="door",
                from_space_id="a",
                to_space_id="c",
                portal_id="door:D1",
                from_level_id="L1",
                to_level_id="L1",
            )
        ],
    )

    stats = qualify_surface_portal_sides(model)

    portal = model.portals[0]
    assert stats.repaired == 1
    assert {portal.from_space_id, portal.to_space_id} == {"a", "b"}
    assert {model.transitions[0].from_space_id, model.transitions[0].to_space_id} == {"a", "b"}


def test_exterior_exit_keeps_only_nearest_internal_surface_side() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("inside", "Inside", "L1"), Space("nearby", "Nearby", "L1")],
        cells=[
            _floor("inside:0", "inside", "L1", ((-1.0, -1.0, 0.0), (0.0, -1.0, 0.0), (0.0, 1.0, 0.0))),
            _floor("nearby:0", "nearby", "L1", ((0.4, -1.0, 0.0), (1.4, -1.0, 0.0), (0.4, 1.0, 0.0))),
        ],
        portals=[
            Portal(
                id="door:EXIT",
                kind="door",
                position_m=(0.0, 0.0, 0.0),
                from_space_id="nearby",
                to_space_id="inside",
                level_id="L1",
                is_exit=True,
            )
        ],
    )

    stats = qualify_surface_portal_sides(model)

    portal = model.portals[0]
    assert stats.repaired == 1
    assert portal.from_space_id == "inside"
    assert portal.to_space_id is None
