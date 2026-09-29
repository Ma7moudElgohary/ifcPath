from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .model import InavModel
from .surface_nav import find_surface_route


@dataclass(slots=True)
class RouteOptions:
    blocked_portals: set[str] | None = None
    blocked_nodes: set[str] | None = None
    blocked_spaces: set[str] | None = None
    hazard_costs: dict[str, float] | None = None
    space_cost_multipliers: dict[str, float] | None = None


def find_path(model: InavModel, start_id: str, goal_id: str, options: RouteOptions | None = None) -> list[str]:
    """Find a dynamic route through an INAV model.

    ``blocked_spaces`` are treated as no-entry regions. If the route starts
    inside one of them, movement inside that start space is still allowed so an
    occupant can evacuate out. ``space_cost_multipliers`` are penalties >= 1.0
    suitable for smoke, crowd density, security preference, or other Digital
    Twin state that should discourage rather than completely disable a space.
    """
    options = options or RouteOptions()
    blocked_portals = options.blocked_portals or set()
    blocked_nodes = options.blocked_nodes or set()
    blocked_spaces = options.blocked_spaces or set()
    hazard_costs = options.hazard_costs or {}
    space_cost_multipliers = options.space_cost_multipliers or {}

    nodes = {node.id: node for node in model.nodes}
    start_node = nodes.get(start_id)
    goal_node = nodes.get(goal_id)
    if start_node is None or goal_node is None:
        return []

    start_space_id = start_node.space_id
    if goal_node.space_id in blocked_spaces and goal_node.space_id != start_space_id:
        return []

    adjacency: dict[str, list[tuple[str, float]]] = {}
    for edge in model.edges:
        if edge.portal_id and edge.portal_id in blocked_portals:
            continue
        if edge.a in blocked_nodes or edge.b in blocked_nodes:
            continue

        a_node = nodes.get(edge.a)
        b_node = nodes.get(edge.b)
        if a_node is None or b_node is None:
            continue

        if (
            a_node.space_id in blocked_spaces
            and a_node.space_id != start_space_id
        ) or (
            b_node.space_id in blocked_spaces
            and b_node.space_id != start_space_id
        ):
            continue

        node_penalty = max(0.0, hazard_costs.get(edge.a, 0.0), hazard_costs.get(edge.b, 0.0))
        space_multiplier = max(
            1.0,
            space_cost_multipliers.get(a_node.space_id or "", 1.0),
            space_cost_multipliers.get(b_node.space_id or "", 1.0),
        )
        cost = edge.distance_m * space_multiplier * (1.0 + node_penalty)
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



def find_path_xyz(model: InavModel, start_xyz, goal_xyz, *, terrain_costs=None):
    """Preferred continuous-surface route API for new consumers.

    The legacy node-ID router remains available during migration.
    """
    return find_surface_route(model.cells, start_xyz, goal_xyz, terrain_costs)
