from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, Space
from ifcpath.vertical_surface import ensure_surface_vertical_transitions, find_surface_vertical_transfer


def test_continuous_stair_preserves_all_landing_spaces() -> None:
    lower_a_contact = (-0.5, 0.0, 0.0)
    lower_b_contact = (0.5, 0.0, 0.0)
    upper_contact = (0.0, 2.0, 3.0)
    shared_a = (1.0, 1.0, 1.5)
    shared_b = (-1.0, 1.0, 1.5)

    lower_a = NavCell(
        id="lower:a",
        vertices_m=((-2.0, -1.0, 0.0), (0.0, -1.0, 0.0), lower_a_contact),
        space_id="a",
        level_id="L1",
        neighbor_ids=["cell:stair:STAIR:0"],
        terrain="open",
        portals={"cell:stair:STAIR:0": (lower_a_contact, lower_a_contact)},
    )
    lower_b = NavCell(
        id="lower:b",
        vertices_m=((0.0, -1.0, 0.0), (2.0, -1.0, 0.0), lower_b_contact),
        space_id="b",
        level_id="L1",
        neighbor_ids=["cell:stair:STAIR:0"],
        terrain="open",
        portals={"cell:stair:STAIR:0": (lower_b_contact, lower_b_contact)},
    )
    stair0 = NavCell(
        id="cell:stair:STAIR:0",
        vertices_m=(lower_a_contact, shared_a, shared_b),
        level_id="L1",
        neighbor_ids=["lower:a", "lower:b", "cell:stair:STAIR:1"],
        terrain="stair",
        portals={
            "lower:a": (lower_a_contact, lower_a_contact),
            "lower:b": (lower_b_contact, lower_b_contact),
            "cell:stair:STAIR:1": (shared_a, shared_b),
        },
    )
    stair1 = NavCell(
        id="cell:stair:STAIR:1",
        vertices_m=(shared_a, shared_b, upper_contact),
        level_id="L2",
        neighbor_ids=["cell:stair:STAIR:0", "upper:c"],
        terrain="stair",
        portals={
            "cell:stair:STAIR:0": (shared_a, shared_b),
            "upper:c": (upper_contact, upper_contact),
        },
    )
    upper = NavCell(
        id="upper:c",
        vertices_m=(upper_contact, (-2.0, 3.0, 3.0), (2.0, 3.0, 3.0)),
        space_id="c",
        level_id="L2",
        neighbor_ids=["cell:stair:STAIR:1"],
        terrain="open",
        portals={"cell:stair:STAIR:1": (upper_contact, upper_contact)},
    )
    model = InavModel(
        levels=[Level("L1", "Lower", 0.0), Level("L2", "Upper", 3.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1"), Space("c", "C", "L2")],
        cells=[lower_a, lower_b, stair0, stair1, upper],
    )

    added = ensure_surface_vertical_transitions(model)
    pairs = {frozenset((item.from_space_id, item.to_space_id or "")) for item in added}

    assert pairs == {
        frozenset(("a", "b")),
        frozenset(("a", "c")),
        frozenset(("b", "c")),
    }
    assert all(item.resource_id == "surface:stair:STAIR" for item in added)
    assert all(find_surface_vertical_transfer(model, item) is not None for item in added)
