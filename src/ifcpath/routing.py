from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from .models import NavigationModel, Vec3


@dataclass(frozen=True)
class HazardZone:
    center_m: Vec3
    radius_m: float
    cost_multiplier: float = 10.0
    blocked: bool = False


@dataclass
class RouteState:
    blocked_portal_ids: set[str] = field(default_factory=set)
    blocked_space_ids: set[str] = field(default_factory=set)
    hazards: list[HazardZone] = field(default_factory=list)


@dataclass(frozen=True)
class RouteResult:
    node_ids: list[str]
    points_m: list[Vec3]
    total_cost: float
    total_length_m: float


def _hazard_multiplier(point: Vec3, state: RouteState) -> float | None:
    multiplier = 1.0
    for hz in state.hazards:
        if math.dist(point, hz.center_m) <= hz.radius_m:
            if hz.blocked:
                return None
            multiplier *= max(1.0, hz.cost_multiplier)
    return multiplier


def _nearest_node(model: NavigationModel, point: Vec3, state: RouteState) -> str | None:
    best: tuple[float, str] | None = None
    for n in model.nodes:
        if n.space_id and n.space_id in state.blocked_space_ids:
            continue
        if n.id in state.blocked_portal_ids:
            continue
        if _hazard_multiplier(n.position_m, state) is None:
            continue
        d = math.dist(point, n.position_m)
        if best is None or d < best[0]:
            best = (d, n.id)
    return best[1] if best else None


def find_route(
    model: NavigationModel,
    start_m: Vec3,
    end_m: Vec3,
    state: RouteState | None = None,
) -> RouteResult | None:
    state = state or RouteState()
    node_map = {n.id: n for n in model.nodes}
    start = _nearest_node(model, start_m, state)
    end = _nearest_node(model, end_m, state)
    if not start or not end:
        return None

    adjacency: dict[str, list[tuple[str, float, float]]] = {n.id: [] for n in model.nodes}
    for e in model.edges:
        if not e.enabled:
            continue
        if e.portal_id and e.portal_id in state.blocked_portal_ids:
            continue
        a = node_map.get(e.a)
        b = node_map.get(e.b)
        if not a or not b:
            continue
        if a.id in state.blocked_portal_ids or b.id in state.blocked_portal_ids:
            continue
        if (a.space_id and a.space_id in state.blocked_space_ids) or (b.space_id and b.space_id in state.blocked_space_ids):
            continue
        ma = _hazard_multiplier(a.position_m, state)
        mb = _hazard_multiplier(b.position_m, state)
        if ma is None or mb is None:
            continue
        multiplier = e.cost_multiplier * max(ma, mb)
        cost = e.length_m * multiplier
        adjacency[a.id].append((b.id, cost, e.length_m))
        adjacency[b.id].append((a.id, cost, e.length_m))

    queue: list[tuple[float, str]] = [(0.0, start)]
    costs = {start: 0.0}
    lengths = {start: 0.0}
    prev: dict[str, str] = {}

    while queue:
        cost, node = heapq.heappop(queue)
        if cost != costs.get(node):
            continue
        if node == end:
            break
        for nxt, edge_cost, edge_len in adjacency.get(node, []):
            new_cost = cost + edge_cost
            if new_cost < costs.get(nxt, float("inf")):
                costs[nxt] = new_cost
                lengths[nxt] = lengths[node] + edge_len
                prev[nxt] = node
                heapq.heappush(queue, (new_cost, nxt))

    if end not in costs:
        return None

    path = [end]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return RouteResult(
        node_ids=path,
        points_m=[node_map[n].position_m for n in path],
        total_cost=costs[end],
        total_length_m=lengths[end],
    )
