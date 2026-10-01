from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, Portal, SemanticTransition, Space
from ifcpath.validation import validate_model


def _cell(
    cell_id: str,
    x: float,
    *,
    space_id: str | None = None,
    level_id: str | None = None,
    neighbors: list[str] | None = None,
    portal_ids: dict[str, str] | None = None,
) -> NavCell:
    return NavCell(
        id=cell_id,
        vertices_m=((x, 0.0, 0.0), (x + 0.8, 0.0, 0.0), (x, 0.8, 0.0)),
        space_id=space_id,
        level_id=level_id,
        neighbor_ids=list(neighbors or ()),
        terrain="open",
        portal_ids=dict(portal_ids or {}),
    )


def test_surface_only_model_is_ready_without_legacy_nodes() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("room", "Room", "L1")],
        cells=[_cell("room:0", 0.0, space_id="room", level_id="L1")],
        nodes=[],
        edges=[],
    )

    report = validate_model(model)

    assert report.valid
    assert report.stats["surface_authoritative"] is True
    assert report.stats["surface_navigation_ready"] is True
    assert report.stats["legacy_navigation_ready"] is False
    assert report.stats["navigation_ready"] is True
    assert not any(issue.code == "NO_NODES" for issue in report.issues)


def test_disconnected_surface_spaces_are_not_ready_without_a_semantic_connector() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1")],
        cells=[
            _cell("a:0", 0.0, space_id="a", level_id="L1"),
            _cell("b:0", 10.0, space_id="b", level_id="L1"),
        ],
    )

    report = validate_model(model)

    assert report.valid
    assert report.stats["surface_semantic_component_count"] == 2
    assert report.stats["navigation_ready"] is False


def test_unauthorized_cross_space_surface_edge_is_invalid() -> None:
    a = _cell("a:0", 0.0, space_id="a", level_id="L1", neighbors=["b:0"])
    b = _cell("b:0", 1.0, space_id="b", level_id="L1", neighbors=["a:0"])
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("a", "A", "L1"), Space("b", "B", "L1")],
        cells=[a, b],
    )

    report = validate_model(model)

    assert not report.valid
    assert report.stats["navigation_ready"] is False
    assert any(issue.code == "SURFACE_UNAUTHORIZED_SPACE_CROSSING" for issue in report.issues)


def test_elevator_semantics_can_connect_separate_surface_components() -> None:
    model = InavModel(
        levels=[Level("L1", "Lower", 0.0), Level("L2", "Upper", 3.0)],
        spaces=[Space("lower", "Lower", "L1"), Space("upper", "Upper", "L2")],
        portals=[
            Portal(
                id="exit:upper",
                kind="door",
                position_m=(10.2, 0.2, 3.0),
                from_space_id="upper",
                level_id="L2",
                is_exit=True,
            )
        ],
        transitions=[
            SemanticTransition(
                id="elevator:L1:L2",
                kind="elevator",
                from_space_id="lower",
                to_space_id="upper",
                from_level_id="L1",
                to_level_id="L2",
                bidirectional=True,
                source="ifc",
                resource_id="elevator:E1",
            )
        ],
        cells=[
            _cell("lower:0", 0.0, space_id="lower", level_id="L1"),
            NavCell(
                id="upper:0",
                vertices_m=((10.0, 0.0, 3.0), (10.8, 0.0, 3.0), (10.0, 0.8, 3.0)),
                space_id="upper",
                level_id="L2",
                terrain="open",
            ),
        ],
        nodes=[],
        edges=[],
    )

    report = validate_model(model)

    assert report.valid
    assert report.stats["surface_component_count"] == 2
    assert report.stats["surface_semantic_component_count"] == 1
    assert report.stats["surface_cross_level_transitions"] >= 1
    assert report.stats["surface_exit_unreachable_spaces"] == 0
    assert report.stats["navigation_ready"] is True


def test_unreachable_occupant_space_is_identified_not_only_counted() -> None:
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("exit-room", "Exit Room", "L1"), Space("isolated", "Isolated Office", "L1")],
        portals=[
            Portal(
                id="exit:door",
                kind="door",
                position_m=(0.2, 0.2, 0.0),
                from_space_id="exit-room",
                level_id="L1",
                is_exit=True,
            )
        ],
        cells=[
            _cell("exit-room:0", 0.0, space_id="exit-room", level_id="L1"),
            _cell("isolated:0", 10.0, space_id="isolated", level_id="L1"),
        ],
    )

    report = validate_model(model)

    assert report.stats["surface_exit_unreachable_spaces"] == 1
    issue = next(issue for issue in report.issues if issue.code == "SURFACE_SPACE_CANNOT_REACH_EXIT")
    assert issue.entity_id == "isolated"
    assert "Isolated Office" in issue.message


def test_space_continuity_can_pass_through_unowned_stair_surface() -> None:
    lower = NavCell(
        id="lower",
        vertices_m=((0.0, 0.0, 0.0), (0.8, 0.0, 0.0), (0.0, 0.8, 0.0)),
        space_id="room",
        level_id="L1",
        terrain="open",
        neighbor_ids=["stair"],
    )
    stair = NavCell(
        id="stair",
        vertices_m=((0.8, 0.0, 0.0), (1.6, 0.0, 0.3), (0.8, 0.8, 0.3)),
        terrain="stair",
        neighbor_ids=["lower", "upper"],
    )
    upper = NavCell(
        id="upper",
        vertices_m=((1.6, 0.0, 0.3), (2.4, 0.0, 0.3), (1.6, 0.8, 0.3)),
        space_id="room",
        level_id="L1",
        terrain="open",
        neighbor_ids=["stair"],
    )
    model = InavModel(
        levels=[Level("L1", "Level 1", 0.0)],
        spaces=[Space("room", "Room", "L1")],
        cells=[lower, stair, upper],
    )

    report = validate_model(model)

    assert report.valid
    assert report.stats["surface_split_spaces"] == 0
    assert not any(issue.code == "SURFACE_SPACE_SPLIT_COMPONENTS" for issue in report.issues)
