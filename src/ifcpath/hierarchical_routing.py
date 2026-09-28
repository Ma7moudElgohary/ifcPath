from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque
from dataclasses import dataclass, field

from .geometry import Vec3, distance
from .model import InavModel, NavNode, SemanticTransition
from .navmesh_routing import find_navmesh_cell, find_navmesh_path, snap_point_to_navmesh
from .semantic import ensure_semantic_transitions


_VERTICAL_KINDS = {"stair", "ramp", "elevator", "escalator"}
_EPSILON = 1e-9


@dataclass(slots=True)
class HierarchicalRouteOptions:
    blocked_portals: set[str] | None = None
    blocked_spaces: set[str] | None = None
    space_cost_multipliers: dict[str, float] | None = None
    max_snap_distance_m: float = 3.0
    point_z_tolerance_m: float = 1.5


@dataclass(slots=True)
class HierarchicalRouteSegment:
    kind: str
    points: list[Vec3]
    from_space_id: str | None = None
    to_space_id: str | None = None
    transition_id: str | None = None
    portal_id: str | None = None
    weighted_cost: float = 0.0

    @property
    def length_m(self) -> float:
        return _polyline_length(self.points)


@dataclass(slots=True)
class HierarchicalRoute:
    segments: list[HierarchicalRouteSegment] = field(default_factory=list)
    space_ids: list[str] = field(default_factory=list)
    transition_ids: list[str] = field(default_factory=list)

    @property
    def points(self) -> list[Vec3]:
        result: list[Vec3] = []
        for segment in self.segments:
            for point in segment.points:
                if not result or distance(result[-1], point) > _EPSILON:
                    result.append(point)
        return result

    @property
    def length_m(self) -> float:
        return sum(segment.length_m for segment in self.segments)

    @property
    def weighted_cost(self) -> float:
        return sum(segment.weighted_cost for segment in self.segments)


@dataclass(frozen=True, slots=True)
class _Anchor:
    id: str
    space_id: str
    point: Vec3
    transition_id: str | None = None


@dataclass(slots=True)
class _Transfer:
    transition: SemanticTransition
    from_anchor: _Anchor
    to_anchor: _Anchor
    points: list[Vec3]


@dataclass(slots=True)
class _GraphEdge:
    target: str
    cost: float
    segment: HierarchicalRouteSegment


def find_hierarchical_path(
    model: InavModel,
    start: Vec3,
    goal: Vec3,
    options: HierarchicalRouteOptions | None = None,
) -> HierarchicalRoute | None:
    """Find a building-wide route using semantic transfers and exact local geometry.

    The graph searched here is deliberately *not* the raw centroid graph. Each
    door/vertical transition contributes one anchor on each connected semantic
    space. Anchors within a CDT space are connected by funnel-shortened paths;
    semantic transitions connect anchors between spaces. This lets route choice
    account for the actual distance to competing doors while preserving BIM
    topology and Digital Twin blocked/cost state.
    """
    ensure_semantic_transitions(model)
    options = options or HierarchicalRouteOptions()
    blocked_portals = options.blocked_portals or set()
    blocked_spaces = options.blocked_spaces or set()
    multipliers = options.space_cost_multipliers or {}

    start_location = _locate_position(model, start, options)
    goal_location = _locate_position(model, goal, options)
    if start_location is None or goal_location is None:
        return None

    start_space_id, start_point = start_location
    goal_space_id, goal_point = goal_location
    if goal_space_id in blocked_spaces and goal_space_id != start_space_id:
        return None

    if start_space_id == goal_space_id:
        local = _local_route(
            model,
            start_point,
            goal_point,
            start_space_id,
            options.max_snap_distance_m,
        )
        if local is None:
            return None
        multiplier = max(1.0, multipliers.get(start_space_id, 1.0))
        segment = HierarchicalRouteSegment(
            kind="local",
            points=local,
            from_space_id=start_space_id,
            to_space_id=start_space_id,
            weighted_cost=_polyline_length(local) * multiplier,
        )
        return HierarchicalRoute(segments=[segment], space_ids=[start_space_id])

    portals = {portal.id: portal for portal in model.portals}
    transfers: list[_Transfer] = []
    for transition in model.transitions:
        if not transition.to_space_id:
            continue
        if transition.portal_id and transition.portal_id in blocked_portals:
            continue
        transfer = _build_transfer(
            model,
            transition,
            portals,
            options.max_snap_distance_m,
        )
        if transfer is not None:
            transfers.append(transfer)

    start_anchor = _Anchor("anchor:start", start_space_id, start_point)
    goal_anchor = _Anchor("anchor:goal", goal_space_id, goal_point)
    anchors_by_space: dict[str, list[_Anchor]] = defaultdict(list)
    anchors_by_space[start_space_id].append(start_anchor)
    anchors_by_space[goal_space_id].append(goal_anchor)
    for transfer in transfers:
        anchors_by_space[transfer.from_anchor.space_id].append(transfer.from_anchor)
        anchors_by_space[transfer.to_anchor.space_id].append(transfer.to_anchor)

    adjacency: dict[str, list[_GraphEdge]] = defaultdict(list)

    # Exact intra-space geometry. Intermediate blocked spaces are removed from
    # the graph; a blocked start space remains routable so an occupant can exit.
    for space_id, anchors in anchors_by_space.items():
        if space_id in blocked_spaces and space_id != start_space_id:
            continue
        multiplier = max(1.0, multipliers.get(space_id, 1.0))
        for i in range(len(anchors)):
            for j in range(i + 1, len(anchors)):
                a = anchors[i]
                b = anchors[j]
                points = _local_route(
                    model,
                    a.point,
                    b.point,
                    space_id,
                    options.max_snap_distance_m,
                )
                if points is None:
                    continue
                length_m = _polyline_length(points)
                cost = length_m * multiplier
                forward = HierarchicalRouteSegment(
                    kind="local",
                    points=points,
                    from_space_id=space_id,
                    to_space_id=space_id,
                    weighted_cost=cost,
                )
                reverse = HierarchicalRouteSegment(
                    kind="local",
                    points=list(reversed(points)),
                    from_space_id=space_id,
                    to_space_id=space_id,
                    weighted_cost=cost,
                )
                adjacency[a.id].append(_GraphEdge(b.id, cost, forward))
                adjacency[b.id].append(_GraphEdge(a.id, cost, reverse))

    # Semantic transfer edges are directed according to transition direction and
    # blocked-space entry rules. A blocked start space can be exited but never
    # re-entered during the same route.
    for transfer in transfers:
        transition = transfer.transition
        transfer_length = _polyline_length(transfer.points)
        if transfer.to_anchor.space_id not in blocked_spaces:
            forward = HierarchicalRouteSegment(
                kind=transition.kind,
                points=transfer.points,
                from_space_id=transfer.from_anchor.space_id,
                to_space_id=transfer.to_anchor.space_id,
                transition_id=transition.id,
                portal_id=transition.portal_id,
                weighted_cost=transfer_length,
            )
            adjacency[transfer.from_anchor.id].append(
                _GraphEdge(transfer.to_anchor.id, transfer_length, forward)
            )

        if transition.bidirectional and transfer.from_anchor.space_id not in blocked_spaces:
            reverse_points = list(reversed(transfer.points))
            reverse = HierarchicalRouteSegment(
                kind=transition.kind,
                points=reverse_points,
                from_space_id=transfer.to_anchor.space_id,
                to_space_id=transfer.from_anchor.space_id,
                transition_id=transition.id,
                portal_id=transition.portal_id,
                weighted_cost=transfer_length,
            )
            adjacency[transfer.to_anchor.id].append(
                _GraphEdge(transfer.from_anchor.id, transfer_length, reverse)
            )

    route_edges = _shortest_anchor_route(adjacency, start_anchor.id, goal_anchor.id)
    if route_edges is None:
        return None

    segments = [edge.segment for edge in route_edges]
    transition_ids = [
        segment.transition_id
        for segment in segments
        if segment.transition_id is not None
    ]
    space_ids = [start_space_id]
    for segment in segments:
        if segment.transition_id and segment.to_space_id and segment.to_space_id != space_ids[-1]:
            space_ids.append(segment.to_space_id)

    return HierarchicalRoute(
        segments=segments,
        space_ids=space_ids,
        transition_ids=transition_ids,
    )


def _locate_position(
    model: InavModel,
    point: Vec3,
    options: HierarchicalRouteOptions,
) -> tuple[str, Vec3] | None:
    cell = find_navmesh_cell(
        model,
        point,
        z_tolerance_m=options.point_z_tolerance_m,
    )
    if cell is not None and cell.space_id:
        snapped = snap_point_to_navmesh(
            model,
            point,
            space_id=cell.space_id,
            max_distance_m=options.max_snap_distance_m,
        )
        if snapped is not None:
            return cell.space_id, snapped[0]

    # Sampled-floor fallback: use the nearest semantically owned walk node.
    candidates = [node for node in model.nodes if node.space_id]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda node: distance(point, node.position_m))
    if distance(point, nearest.position_m) > options.max_snap_distance_m:
        return None
    return nearest.space_id or "", nearest.position_m


def _build_transfer(
    model: InavModel,
    transition: SemanticTransition,
    portals,
    max_snap_distance_m: float,
) -> _Transfer | None:
    if not transition.to_space_id:
        return None

    if transition.portal_id:
        portal = portals.get(transition.portal_id)
        if portal is None:
            return None
        from_point = _snap_space_anchor(
            model, portal.position_m, transition.from_space_id, max_snap_distance_m
        )
        to_point = _snap_space_anchor(
            model, portal.position_m, transition.to_space_id, max_snap_distance_m
        )
        if from_point is None or to_point is None:
            return None
        from_anchor = _Anchor(
            f"anchor:{transition.id}:from",
            transition.from_space_id,
            from_point,
            transition.id,
        )
        to_anchor = _Anchor(
            f"anchor:{transition.id}:to",
            transition.to_space_id,
            to_point,
            transition.id,
        )
        points = _dedupe_points([from_point, portal.position_m, to_point])
        return _Transfer(transition, from_anchor, to_anchor, points)

    vertical_points = _vertical_transfer_points(model, transition)
    if not vertical_points:
        return None
    from_point = _snap_space_anchor(
        model, vertical_points[0], transition.from_space_id, max_snap_distance_m
    )
    to_point = _snap_space_anchor(
        model, vertical_points[-1], transition.to_space_id, max_snap_distance_m
    )
    if from_point is None or to_point is None:
        return None

    from_anchor = _Anchor(
        f"anchor:{transition.id}:from",
        transition.from_space_id,
        from_point,
        transition.id,
    )
    to_anchor = _Anchor(
        f"anchor:{transition.id}:to",
        transition.to_space_id,
        to_point,
        transition.id,
    )
    points = _dedupe_points([from_point, *vertical_points, to_point])
    return _Transfer(transition, from_anchor, to_anchor, points)


def _snap_space_anchor(
    model: InavModel,
    point: Vec3,
    space_id: str,
    max_snap_distance_m: float,
) -> Vec3 | None:
    snapped = snap_point_to_navmesh(
        model,
        point,
        space_id=space_id,
        max_distance_m=max_snap_distance_m,
    )
    if snapped is not None:
        return snapped[0]

    nodes = [node for node in model.nodes if node.space_id == space_id]
    if not nodes:
        return None
    nearest = min(nodes, key=lambda node: distance(point, node.position_m))
    if distance(point, nearest.position_m) > max_snap_distance_m:
        return None
    return nearest.position_m


def _local_route(
    model: InavModel,
    start: Vec3,
    goal: Vec3,
    space_id: str,
    max_snap_distance_m: float,
) -> list[Vec3] | None:
    navmesh = find_navmesh_path(model, start, goal, space_id=space_id)
    if navmesh is not None:
        return _dedupe_points(navmesh.points)
    return _local_metric_route(model, start, goal, space_id, max_snap_distance_m)


def _local_metric_route(
    model: InavModel,
    start: Vec3,
    goal: Vec3,
    space_id: str,
    max_snap_distance_m: float,
) -> list[Vec3] | None:
    nodes = {node.id: node for node in model.nodes if node.space_id == space_id}
    if not nodes:
        return None
    start_node = min(nodes.values(), key=lambda node: distance(start, node.position_m))
    goal_node = min(nodes.values(), key=lambda node: distance(goal, node.position_m))
    if (
        distance(start, start_node.position_m) > max_snap_distance_m
        or distance(goal, goal_node.position_m) > max_snap_distance_m
    ):
        return None

    adjacency: dict[str, list[tuple[str, float]]] = defaultdict(list)
    for edge in model.edges:
        if edge.a in nodes and edge.b in nodes:
            adjacency[edge.a].append((edge.b, edge.distance_m))
            adjacency[edge.b].append((edge.a, edge.distance_m))

    path_ids = _dijkstra_ids(adjacency, {start_node.id}, {goal_node.id})
    if not path_ids:
        return None
    points = [start, *[nodes[node_id].position_m for node_id in path_ids], goal]
    return _dedupe_points(points)


def _vertical_transfer_points(
    model: InavModel,
    transition: SemanticTransition,
) -> list[Vec3] | None:
    node_by_id = {node.id: node for node in model.nodes}
    if transition.kind == "vertical":
        vertical_ids = {node.id for node in model.nodes if node.kind in _VERTICAL_KINDS}
    else:
        vertical_ids = {node.id for node in model.nodes if node.kind == transition.kind}
    if not vertical_ids:
        return None

    edge_adjacency: dict[str, list[tuple[str, float]]] = defaultdict(list)
    for edge in model.edges:
        if edge.a in node_by_id and edge.b in node_by_id:
            edge_adjacency[edge.a].append((edge.b, edge.distance_m))
            edge_adjacency[edge.b].append((edge.a, edge.distance_m))

    best: tuple[float, list[str]] | None = None
    for component in _vertical_components(vertical_ids, edge_adjacency):
        from_landings = _landing_nodes_for_component(
            component,
            edge_adjacency,
            node_by_id,
            transition.from_space_id,
            transition.from_level_id,
        )
        to_landings = _landing_nodes_for_component(
            component,
            edge_adjacency,
            node_by_id,
            transition.to_space_id or "",
            transition.to_level_id,
        )
        if not from_landings or not to_landings:
            continue

        allowed = set(component) | from_landings | to_landings
        restricted: dict[str, list[tuple[str, float]]] = defaultdict(list)
        for node_id in allowed:
            for neighbour_id, weight in edge_adjacency.get(node_id, ()):
                if neighbour_id not in allowed:
                    continue
                # Never use a floor-to-floor shortcut; every transfer must touch
                # the actual vertical component.
                if node_id not in component and neighbour_id not in component:
                    continue
                restricted[node_id].append((neighbour_id, weight))

        path_ids = _dijkstra_ids(restricted, from_landings, to_landings)
        if not path_ids:
            continue
        path_length = sum(
            distance(node_by_id[a].position_m, node_by_id[b].position_m)
            for a, b in zip(path_ids, path_ids[1:])
        )
        if best is None or path_length < best[0]:
            best = (path_length, path_ids)

    if best is None:
        return None
    return [node_by_id[node_id].position_m for node_id in best[1]]


def _landing_nodes_for_component(
    component: set[str],
    adjacency: dict[str, list[tuple[str, float]]],
    node_by_id: dict[str, NavNode],
    space_id: str,
    level_id: str | None,
) -> set[str]:
    result: set[str] = set()
    for vertical_id in component:
        for neighbour_id, _ in adjacency.get(vertical_id, ()):
            if neighbour_id in component:
                continue
            node = node_by_id.get(neighbour_id)
            if node is None or node.space_id != space_id:
                continue
            if level_id is not None and node.level_id is not None and node.level_id != level_id:
                continue
            result.add(neighbour_id)
    return result


def _vertical_components(
    vertical_ids: set[str],
    adjacency: dict[str, list[tuple[str, float]]],
) -> list[set[str]]:
    remaining = set(vertical_ids)
    components: list[set[str]] = []
    while remaining:
        start = remaining.pop()
        component = {start}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for neighbour, _ in adjacency.get(current, ()):
                if neighbour not in remaining or neighbour not in vertical_ids:
                    continue
                remaining.remove(neighbour)
                component.add(neighbour)
                queue.append(neighbour)
        components.append(component)
    return components


def _shortest_anchor_route(
    adjacency: dict[str, list[_GraphEdge]],
    start_id: str,
    goal_id: str,
) -> list[_GraphEdge] | None:
    queue: list[tuple[float, str]] = [(0.0, start_id)]
    cost = {start_id: 0.0}
    previous: dict[str, tuple[str, _GraphEdge]] = {}

    while queue:
        current_cost, current = heapq.heappop(queue)
        if current == goal_id:
            break
        if current_cost != cost.get(current):
            continue
        for edge in adjacency.get(current, ()):
            candidate = current_cost + edge.cost
            if candidate >= cost.get(edge.target, math.inf):
                continue
            cost[edge.target] = candidate
            previous[edge.target] = (current, edge)
            heapq.heappush(queue, (candidate, edge.target))

    if goal_id not in cost:
        return None

    result: list[_GraphEdge] = []
    cursor = goal_id
    while cursor != start_id:
        item = previous.get(cursor)
        if item is None:
            return None
        parent, edge = item
        result.append(edge)
        cursor = parent
    result.reverse()
    return result


def _dijkstra_ids(
    adjacency: dict[str, list[tuple[str, float]]],
    start_ids: set[str],
    goal_ids: set[str],
) -> list[str]:
    queue: list[tuple[float, str]] = []
    dist: dict[str, float] = {}
    previous: dict[str, str] = {}
    for start_id in start_ids:
        dist[start_id] = 0.0
        heapq.heappush(queue, (0.0, start_id))

    reached: str | None = None
    while queue:
        current_distance, current = heapq.heappop(queue)
        if current_distance != dist.get(current):
            continue
        if current in goal_ids:
            reached = current
            break
        for neighbour, weight in adjacency.get(current, ()):
            candidate = current_distance + weight
            if candidate >= dist.get(neighbour, math.inf):
                continue
            dist[neighbour] = candidate
            previous[neighbour] = current
            heapq.heappush(queue, (candidate, neighbour))

    if reached is None:
        return []
    path = [reached]
    while path[-1] not in start_ids:
        parent = previous.get(path[-1])
        if parent is None:
            return []
        path.append(parent)
    path.reverse()
    return path


def _polyline_length(points: list[Vec3]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def _dedupe_points(points: list[Vec3]) -> list[Vec3]:
    result: list[Vec3] = []
    for point in points:
        if not result or distance(result[-1], point) > _EPSILON:
            result.append(point)
    return result
