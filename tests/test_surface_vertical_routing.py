from __future__ import annotations

from ifcpath.hierarchical_routing import find_hierarchical_path
from ifcpath.model import InavModel, Level, NavCell, Space
from ifcpath.vertical_surface import (
    ensure_surface_vertical_transitions,
    find_surface_vertical_transfer,
    surface_vertical_components,
)


def _surface_only_stair_model() -> InavModel:
    lower_contact = (0.0, 0.0, 0.0)
    upper_contact = (0.0, 2.0, 3.0)
    shared_a = (1.0, 1.0, 1.5)
    shared_b = (-1.0, 1.0, 1.5)

    lower = NavCell(
        id="floor:lower",
        vertices_m=((-2.0, -1.0, 0.0), (2.0, -1.0, 0.0), lower_contact),
        space_id="lower",
        level_id="L1",
        neighbor_ids=["cell:stair:STAIR_GUID:0"],
        terrain="open",
        portals={"cell:stair:STAIR_GUID:0": (lower_contact, lower_contact)},
    )
    stair0 = NavCell(
        id="cell:stair:STAIR_GUID:0",
        vertices_m=(lower_contact, shared_a, shared_b),
        level_id="L1",
        neighbor_ids=["floor:lower", "cell:stair:STAIR_GUID:1"],
        terrain="stair",
        portals={
            "floor:lower": (lower_contact, lower_contact),
            "cell:stair:STAIR_GUID:1": (shared_a, shared_b),
        },
    )
    stair1 = NavCell(
        id="cell:stair:STAIR_GUID:1",
        vertices_m=(shared_a, shared_b, upper_contact),
        level_id="L2",
        neighbor_ids=["cell:stair:STAIR_GUID:0", "floor:upper"],
        terrain="stair",
        portals={
            "cell:stair:STAIR_GUID:0": (shared_a, shared_b),
            "floor:upper": (upper_contact, upper_contact),
        },
    )
    upper = NavCell(
        id="floor:upper",
        vertices_m=(upper_contact, (-2.0, 3.0, 3.0), (2.0, 3.0, 3.0)),
        space_id="upper",
        level_id="L2",
        neighbor_ids=["cell:stair:STAIR_GUID:1"],
        terrain="open",
        portals={"cell:stair:STAIR_GUID:1": (upper_contact, upper_contact)},
    )
    return InavModel(
        levels=[Level("L1", "Lower", 0.0), Level("L2", "Upper", 3.0)],
        spaces=[Space("lower", "Lower", "L1"), Space("upper", "Upper", "L2")],
        cells=[lower, stair0, stair1, upper],
        nodes=[],
        edges=[],
    )


def test_surface_component_finds_semantic_landings_without_nodes() -> None:
    model = _surface_only_stair_model()

    components = surface_vertical_components(model)

    assert len(components) == 1
    component = components[0]
    assert component.kind == "stair"
    assert component.resource_id == "surface:stair:STAIR_GUID"
    assert {(landing.level_id, landing.space_id) for landing in component.landings} == {
        ("L1", "lower"),
        ("L2", "upper"),
    }


def test_surface_transition_and_transfer_require_no_sampled_vertical_nodes() -> None:
    model = _surface_only_stair_model()

    added = ensure_surface_vertical_transitions(model)

    assert len(added) == 1
    transition = added[0]
    assert transition.kind == "stair"
    assert transition.source == "surface_vertical_touch"
    assert transition.resource_id == "surface:stair:STAIR_GUID"

    transfer = find_surface_vertical_transfer(model, transition)
    assert transfer is not None
    assert transfer.cell_ids == (
        "cell:stair:STAIR_GUID:0",
        "cell:stair:STAIR_GUID:1",
    )
    assert transfer.points[0][2] == 0.0
    assert transfer.points[-1][2] == 3.0
    assert transfer.length_m > 3.0


def test_hierarchical_route_uses_surface_stair_with_zero_nodes() -> None:
    model = _surface_only_stair_model()
    assert not model.nodes

    route = find_hierarchical_path(
        model,
        (0.0, -0.5, 0.0),
        (0.0, 2.5, 3.0),
    )

    assert route is not None
    stair_segments = [segment for segment in route.segments if segment.kind == "stair"]
    assert len(stair_segments) == 1
    stair = stair_segments[0]
    assert stair.transition_id is not None
    assert stair.points[0][2] == 0.0
    assert stair.points[-1][2] == 3.0
    assert route.points[0][2] == 0.0
    assert route.points[-1][2] == 3.0
