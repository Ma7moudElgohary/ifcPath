from ifcpath.models import NavEdge, NavigationModel, NavNode, NodeKind, Portal, Space
from ifcpath.routing import HazardZone, RouteState, find_route


def sample_model():
    return NavigationModel(
        spaces=[
            Space(id="room", centroid_m=(0, 0, 0)),
            Space(id="corridor", centroid_m=(2, 0, 0)),
        ],
        portals=[Portal(
            id="door", kind="door", position_m=(1, 0, 0),
            from_space_id="room", to_space_id="corridor",
        )],
        nodes=[
            NavNode(id="a", position_m=(0, 0, 0), space_id="room"),
            NavNode(id="door", position_m=(1, 0, 0), kind=NodeKind.DOOR),
            NavNode(id="b", position_m=(2, 0, 0), space_id="corridor"),
            NavNode(id="alt1", position_m=(0, 2, 0)),
            NavNode(id="alt2", position_m=(2, 2, 0)),
        ],
        edges=[
            NavEdge(a="a", b="door", length_m=1, portal_id="door"),
            NavEdge(a="door", b="b", length_m=1, portal_id="door"),
            NavEdge(a="a", b="alt1", length_m=2),
            NavEdge(a="alt1", b="alt2", length_m=2),
            NavEdge(a="alt2", b="b", length_m=2),
        ],
    )


def test_shortest_route_uses_door():
    result = find_route(sample_model(), (0, 0, 0), (2, 0, 0))
    assert result is not None
    assert result.node_ids == ["a", "door", "b"]
    assert result.total_length_m == 2


def test_blocked_door_reroutes():
    state = RouteState(blocked_portal_ids={"door"})
    result = find_route(sample_model(), (0, 0, 0), (2, 0, 0), state)
    assert result is not None
    assert result.node_ids == ["a", "alt1", "alt2", "b"]
    assert result.total_length_m == 6


def test_hazard_can_force_alternate_route():
    state = RouteState(hazards=[HazardZone(center_m=(1, 0, 0), radius_m=0.2, cost_multiplier=20)])
    result = find_route(sample_model(), (0, 0, 0), (2, 0, 0), state)
    assert result is not None
    assert "door" not in result.node_ids
