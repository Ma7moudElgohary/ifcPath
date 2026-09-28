from ifcpath.model import InavModel, NavEdge, NavNode
from ifcpath.routing import RouteOptions, find_path


def _model():
    m = InavModel()
    m.nodes = [
        NavNode("A", (0, 0, 0)),
        NavNode("B", (1, 0, 0)),
        NavNode("C", (2, 0, 0)),
        NavNode("D", (1, 1, 0)),
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
