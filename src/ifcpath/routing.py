from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .model import InavModel


@dataclass(slots=True)
class RouteOptions:
    blocked_portals: set[str] | None = None
    blocked_nodes: set[str] | None = None
    hazard_costs: dict[str, float] | None = None


def find_path(model: InavModel, start_id: str, goal_id: str, options: RouteOptions | None = None) -> list[str]:
    options = options or RouteOptions()
    blocked_portals = options.blocked_portals or set()
    blocked_nodes = options.blocked_nodes or set()
    hazard_costs = options.hazard_costs or {}

    adjacency: dict[str, list[tuple[str, float]]] = {}
    for edge in model.edges:
        if edge.portal_id and edge.portal_id in blocked_portals:
            continue
        if edge.a in blocked_nodes or edge.b in blocked_nodes:
            continue
        penalty = max(0.0, hazard_costs.get(edge.a, 0.0), hazard_costs.get(edge.b, 0.0))
        cost = edge.distance_m * (1.0 + penalty)
        adjacency.setdefault(edge.a, []).append((edge.b, cost))
        adjacency.setdefault(edge.b, []).append((edge.a, cost))

    queue: list[tuple[float, str]] = [(0.0, start_id)]
    dist = {start_id: 0.0}
    prev: dict[str, str] = {}

    while queue:
        current_dist, current = heapq.heappop(queue)
        if current == goal_id:
            break
        if current_dist != dist.get(current):
            continue
        for nxt, weight in adjacency.get(current, []):
            candidate = current_dist + weight
            if candidate < dist.get(nxt, math.inf):
                dist[nxt] = candidate
                prev[nxt] = current
                heapq.heappush(queue, (candidate, nxt))

    if goal_id not in dist:
        return []
    path = [goal_id]
    while path[-1] != start_id:
        path.append(prev[path[-1]])
    path.reverse()
    return path
