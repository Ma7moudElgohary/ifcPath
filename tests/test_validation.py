from ifcpath.model import InavModel, NavEdge, NavNode, Portal, SemanticTransition, Space
from ifcpath.validation import validate_model


def test_validation_reports_disconnected_graph_and_space_without_nav():
    model = InavModel(
        spaces=[Space(id="s1", name="Room", level_id=None), Space(id="s2", name="Empty", level_id=None)],
        nodes=[
            NavNode(id="a", position_m=(0.0, 0.0, 0.0), space_id="s1"),
            NavNode(id="b", position_m=(1.0, 0.0, 0.0), space_id="s1"),
            NavNode(id="c", position_m=(10.0, 0.0, 0.0)),
        ],
        edges=[NavEdge(a="a", b="b", distance_m=1.0)],
    )

    report = validate_model(model)
    codes = {issue.code for issue in report.issues}

    assert report.valid
    assert report.component_count == 2
    assert "DISCONNECTED_GRAPH" in codes
    assert "ISOLATED_NODE" in codes
    assert "SPACE_NO_NAV" in codes


def test_validation_detects_missing_edge_node_as_error():
    model = InavModel(
        nodes=[NavNode(id="a", position_m=(0.0, 0.0, 0.0))],
        edges=[NavEdge(a="a", b="missing", distance_m=1.0)],
    )

    report = validate_model(model)

    assert not report.valid
    assert any(issue.code == "EDGE_MISSING_NODE" for issue in report.issues)


def test_validation_exit_reachability_stats():
    model = InavModel(
        portals=[Portal(id="exit", kind="door", position_m=(1.0, 0.0, 0.0), from_space_id="s1", is_exit=True)],
        spaces=[Space(id="s1", name="Room", level_id=None)],
        nodes=[
            NavNode(id="a", position_m=(0.0, 0.0, 0.0), space_id="s1"),
            NavNode(id="e", position_m=(1.0, 0.0, 0.0), space_id="s1", portal_id="exit"),
            NavNode(id="x", position_m=(10.0, 0.0, 0.0)),
        ],
        edges=[NavEdge(a="a", b="e", distance_m=1.0, portal_id="exit")],
    )

    report = validate_model(model)

    assert any(issue.code == "NODES_CANNOT_REACH_EXIT" for issue in report.issues)
    assert report.stats["exit_reachable_nodes"] == 2
    assert report.stats["exit_unreachable_nodes"] == 1
    assert report.stats["exit_reachable_ratio"] == 2 / 3


def test_validation_requires_two_sided_portal_attachment():
    model = InavModel(
        spaces=[
            Space(id="a-space", name="A", level_id=None),
            Space(id="b-space", name="B", level_id=None),
        ],
        portals=[Portal(
            id="door",
            kind="door",
            position_m=(1.0, 0.0, 0.0),
            from_space_id="a-space",
            to_space_id="b-space",
        )],
        nodes=[
            NavNode(id="a", position_m=(0.0, 0.0, 0.0), space_id="a-space"),
            NavNode(id="b", position_m=(2.0, 0.0, 0.0), space_id="b-space"),
            NavNode(id="p", position_m=(1.0, 0.0, 0.0), portal_id="door", space_id="a-space"),
        ],
        edges=[NavEdge(a="a", b="p", distance_m=1.0, portal_id="door")],
    )

    report = validate_model(model)

    assert report.stats["portal_side_failures"] == 1
    assert any(issue.code == "PORTAL_MISSING_SIDE" for issue in report.issues)


def test_validation_rejects_broken_semantic_transition_reference():
    model = InavModel(
        spaces=[Space(id="s1", name="Room", level_id="L1")],
        transitions=[SemanticTransition(
            id="t1",
            kind="door",
            from_space_id="s1",
            to_space_id="missing",
            from_level_id="L1",
        )],
    )

    report = validate_model(model)

    assert not report.valid
    assert report.stats["semantic_transition_errors"] == 1
    assert any(issue.code == "TRANSITION_UNKNOWN_TO_SPACE" for issue in report.issues)
