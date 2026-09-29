from __future__ import annotations

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import (
    InavModel,
    Level,
    NavCell,
    NavEdge,
    NavNode,
    Portal,
    SemanticTransition,
    Space,
)
from ifcpath.route_profiles import (
    RouteProfile,
    find_profiled_hierarchical_path,
    model_for_route_profile,
    resolve_route_profile,
)


def _add_rect_space(
    model: InavModel,
    space_id: str,
    *,
    z: float,
    level_id: str,
    x0: float = 0.0,
    x1: float = 4.0,
) -> None:
    model.spaces.append(Space(space_id, space_id, level_id))
    cdt = build_floor_cdt_navmesh(
        Polygon([(x0, 0.0), (x1, 0.0), (x1, 4.0), (x0, 4.0)]),
        z,
    )
    ids = [f"cell:{space_id}:{index}" for index in range(len(cdt.cells))]
    for index, cell in enumerate(cdt.cells):
        model.cells.append(
            NavCell(
                id=ids[index],
                vertices_m=cell.vertices,
                space_id=space_id,
                level_id=level_id,
                neighbor_ids=[ids[n] for n in cell.neighbor_indices],
            )
        )


def _two_floor_vertical_model(kind: str) -> InavModel:
    model = InavModel(levels=[Level("L1", "L1", 0.0), Level("L2", "L2", 3.0)])
    _add_rect_space(model, "lower", z=0.0, level_id="L1")
    _add_rect_space(model, "upper", z=3.0, level_id="L2")
    model.nodes = [
        NavNode("lower-landing", (2.0, 2.0, 0.0), level_id="L1", space_id="lower"),
        NavNode(f"{kind}-0", (2.0, 2.0, 0.2), kind=kind),
        NavNode(f"{kind}-1", (2.0, 2.0, 2.8), kind=kind),
        NavNode("upper-landing", (2.0, 2.0, 3.0), level_id="L2", space_id="upper"),
    ]
    model.edges = [
        NavEdge("lower-landing", f"{kind}-0", 0.2),
        NavEdge(f"{kind}-0", f"{kind}-1", 2.6),
        NavEdge(f"{kind}-1", "upper-landing", 0.2),
    ]
    model.transitions = [
        SemanticTransition(
            id=f"transition:{kind}",
            kind=kind,
            from_space_id="lower",
            to_space_id="upper",
            from_level_id="L1",
            to_level_id="L2",
            resource_id=("elevator:test" if kind == "elevator" else None),
        )
    ]
    return model


def test_accessible_profile_rejects_stair_only_route() -> None:
    model = _two_floor_vertical_model("stair")

    standard = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (3.0, 2.0, 3.0),
        profile="standard",
    )
    accessible = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (3.0, 2.0, 3.0),
        profile="accessible",
    )

    assert standard is not None
    assert accessible is None


def test_emergency_responder_profile_rejects_elevator_only_route() -> None:
    model = _two_floor_vertical_model("elevator")

    standard = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (3.0, 2.0, 3.0),
        profile="standard",
    )
    responder = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (3.0, 2.0, 3.0),
        profile="emergency_responder",
    )

    assert standard is not None
    assert responder is None


def test_custom_portal_width_policy_filters_narrow_door_without_mutating_model() -> None:
    model = InavModel(levels=[Level("L1", "L1", 0.0)])
    _add_rect_space(model, "A", z=0.0, level_id="L1", x0=0.0, x1=4.0)
    _add_rect_space(model, "B", z=0.0, level_id="L1", x0=4.0, x1=8.0)
    model.portals = [
        Portal(
            "door:AB",
            "door",
            (4.0, 2.0, 0.0),
            "A",
            "B",
            "L1",
            width_m=0.75,
        )
    ]

    assert model.transitions == []
    normal = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (7.0, 2.0, 0.0),
        profile="standard",
    )
    strict = RouteProfile(
        id="custom-wide",
        label="Custom wide",
        description="Require a known portal width of at least 0.90 m.",
        min_portal_width_m=0.90,
        require_known_portal_width=True,
    )
    profiled = model_for_route_profile(model, strict)
    blocked = find_profiled_hierarchical_path(
        model,
        (1.0, 2.0, 0.0),
        (7.0, 2.0, 0.0),
        profile=strict,
    )

    assert normal is not None
    assert blocked is None
    assert profiled.transitions == []
    assert model.transitions == []
    assert model.portals[0].width_m == 0.75


def test_profile_aliases_are_stable() -> None:
    assert resolve_route_profile("wheelchair").id == "accessible"
    assert resolve_route_profile("firefighter").id == "emergency_responder"
    assert resolve_route_profile(None).id == "standard"
