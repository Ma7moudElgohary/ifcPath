from ifcpath.model import InavModel, NavEdge, NavNode
from ifcpath.routing import RouteOptions, find_path


def _model():
    m = InavModel()
    m.nodes = [
        NavNode("A", (0, 0, 0), space_id="start"),
        NavNode("B", (1, 0, 0), space_id="risk"),
        NavNode("C", (2, 0, 0), space_id="goal"),
        NavNode("D", (1, 1, 0), space_id="safe"),
    ]
    m.edges = [
        NavEdge("A", "B", 1.0),
        NavEdge("B", "C", 1.0, portal_id="door:1"),
        NavEdge("A", "D", 1.4),
        NavEdge("D", "C", 1.4),
    ]
    return m


def test_shortest_path():
    assert find_path(_model(), "A", "C") == ["A", "B", "C"]


def test_blocked_portal_reroutes():
    path = find_path(_model(), "A", "C", RouteOptions(blocked_portals={"door:1"}))
    assert path == ["A", "D", "C"]


def test_hazard_cost_reroutes():
    path = find_path(_model(), "A", "C", RouteOptions(hazard_costs={"B": 10.0}))
    assert path == ["A", "D", "C"]


def test_blocked_space_is_no_entry_and_reroutes():
    path = find_path(_model(), "A", "C", RouteOptions(blocked_spaces={"risk"}))
    assert path == ["A", "D", "C"]


def test_space_cost_multiplier_discourages_hazard_space():
    path = find_path(
        _model(),
        "A",
        "C",
        RouteOptions(space_cost_multipliers={"risk": 10.0}),
    )
    assert path == ["A", "D", "C"]


def test_occupant_can_escape_space_that_becomes_blocked():
    m = InavModel(
        nodes=[
            NavNode("A", (0, 0, 0), space_id="fire-room"),
            NavNode("B", (1, 0, 0), space_id="fire-room"),
            NavNode("C", (2, 0, 0), space_id="safe"),
        ],
        edges=[
            NavEdge("A", "B", 1.0),
            NavEdge("B", "C", 1.0),
        ],
    )
    path = find_path(m, "A", "C", RouteOptions(blocked_spaces={"fire-room"}))
    assert path == ["A", "B", "C"]


def test_cannot_route_into_blocked_destination_space():
    assert find_path(
        _model(),
        "A",
        "C",
        RouteOptions(blocked_spaces={"goal"}),
    ) == []
