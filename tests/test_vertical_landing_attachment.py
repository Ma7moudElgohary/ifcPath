from ifcpath.ifc_loader import _attach_vertical_landings
from ifcpath.model import InavModel, NavEdge, NavNode


def test_stair_component_gets_explicit_landing_edges_on_served_levels():
    model = InavModel(
        nodes=[
            NavNode("w1", (0.0, 0.0, 0.0), kind="walk", level_id="L1", space_id="S1"),
            NavNode("s1", (1.8, 0.0, 0.1), kind="stair", level_id="L1"),
            NavNode("s2", (1.8, 0.0, 2.9), kind="stair", level_id="L2"),
            NavNode("w2", (0.0, 0.0, 3.0), kind="walk", level_id="L2", space_id="S2"),
        ],
        edges=[NavEdge("s1", "s2", 2.8, kind="walk")],
    )

    added = _attach_vertical_landings(model, 2.5)

    assert added == 2
    pairs = [{edge.a, edge.b} for edge in model.edges]
    assert {"w1", "s1"} in pairs
    assert {"w2", "s2"} in pairs


def test_vertical_landing_attachment_does_not_cross_levels():
    model = InavModel(
        nodes=[
            NavNode("wrong", (0.0, 0.0, 0.0), kind="walk", level_id="L2", space_id="S2"),
            NavNode("stair", (0.1, 0.0, 0.0), kind="stair", level_id="L1"),
        ]
    )

    assert _attach_vertical_landings(model, 2.5) == 0
    assert not model.edges


def test_vertical_landing_attachment_fails_closed_when_too_far():
    model = InavModel(
        nodes=[
            NavNode("walk", (10.0, 0.0, 0.0), kind="walk", level_id="L1", space_id="S1"),
            NavNode("stair", (0.0, 0.0, 0.0), kind="stair", level_id="L1"),
        ]
    )

    assert _attach_vertical_landings(model, 2.5) == 0
