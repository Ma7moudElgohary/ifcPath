from ifcpath.model import InavModel, NavEdge, NavNode, Portal, Space
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


def test_validation_exit_reachability():
    model = InavModel(
        portals=[Portal(id="exit", kind="door", position_m=(1.0, 0.0, 0.0), is_exit=True)],
        nodes=[
            NavNode(id="a", position_m=(0.0, 0.0, 0.0)),
            NavNode(id="e", position_m=(1.0, 0.0, 0.0), portal_id="exit"),
            NavNode(id="x", position_m=(10.0, 0.0, 0.0)),
        ],
        edges=[NavEdge(a="a", b="e", distance_m=1.0, portal_id="exit")],
    )

    report = validate_model(model)

    assert any(issue.code == "NODES_CANNOT_REACH_EXIT" for issue in report.issues)
