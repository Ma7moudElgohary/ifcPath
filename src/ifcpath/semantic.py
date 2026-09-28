from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .model import InavModel, SemanticTransition


@dataclass(slots=True)
class SemanticRouteOptions:
    blocked_portals: set[str] | None = None
    blocked_spaces: set[str] | None = None
    space_cost_multipliers: dict[str, float] | None = None


def ensure_semantic_transitions(model: InavModel) -> list[SemanticTransition]:
    """Populate the INAV dual graph from qualified semantic portals.

    Horizontal connectivity is already resolved during IFC import from
    `IfcRelSpaceBoundary*` where possible, with geometric inference as fallback.
    This function materializes that result as a target-independent space graph.

    Existing transitions are preserved so future explicit stair/ramp/elevator
    transitions can coexist with door-derived connectivity.
    """
    existing_portals = {transition.portal_id for transition in model.transitions if transition.portal_id}
    spaces = {space.id: space for space in model.spaces}

    for portal in model.portals:
        if portal.id in existing_portals or not portal.from_space_id:
            continue
        from_space = spaces.get(portal.from_space_id)
        to_space = spaces.get(portal.to_space_id) if portal.to_space_id else None
        model.transitions.append(SemanticTransition(
            id=f"transition:{portal.id}",
            kind=portal.kind,
            from_space_id=portal.from_space_id,
            to_space_id=portal.to_space_id,
            portal_id=portal.id,
            from_level_id=from_space.level_id if from_space else portal.level_id,
            to_level_id=to_space.level_id if to_space else None,
            bidirectional=True,
            source="portal",
        ))
        existing_portals.add(portal.id)

    return model.transitions


def find_space_path(
    model: InavModel,
    start_space_id: str,
    goal_space_id: str,
    options: SemanticRouteOptions | None = None,
) -> list[str]:
    """Find a high-level space sequence through the semantic dual graph.

    This is intentionally a topology route, not the final metric route. The
    metric graph is still responsible for exact local distance/geometry between
    portals. Space multipliers allow Digital Twin state (smoke, crowd, access
    preference) to influence coarse route selection.
    """
    ensure_semantic_transitions(model)
    options = options or SemanticRouteOptions()
    blocked_portals = options.blocked_portals or set()
    blocked_spaces = options.blocked_spaces or set()
    multipliers = options.space_cost_multipliers or {}

    known_spaces = {space.id for space in model.spaces}
    if start_space_id not in known_spaces or goal_space_id not in known_spaces:
        return []
    if goal_space_id in blocked_spaces and goal_space_id != start_space_id:
        return []

    adjacency: dict[str, list[tuple[str, float]]] = {}
    for transition in model.transitions:
        if transition.portal_id and transition.portal_id in blocked_portals:
            continue
        a = transition.from_space_id
        b = transition.to_space_id
        if not a or not b:
            # Exterior transitions are retained in INAV but are not space nodes.
            continue
        if a not in known_spaces or b not in known_spaces:
            continue
        if a in blocked_spaces and a != start_space_id:
            continue
        if b in blocked_spaces and b != start_space_id:
            continue

        # The semantic graph deliberately uses unit topological cost. Exact
        # geometry is resolved on the local metric graph. Operational penalties
        # can still bias this coarse choice without inventing centroid distances.
        cost = max(1.0, multipliers.get(a, 1.0), multipliers.get(b, 1.0))
        adjacency.setdefault(a, []).append((b, cost))
        if transition.bidirectional:
            adjacency.setdefault(b, []).append((a, cost))

    queue: list[tuple[float, str]] = [(0.0, start_space_id)]
    dist = {start_space_id: 0.0}
    previous: dict[str, str] = {}

    while queue:
        current_distance, current = heapq.heappop(queue)
        if current == goal_space_id:
            break
        if current_distance != dist.get(current):
            continue
        for neighbour, cost in adjacency.get(current, ()):
            candidate = current_distance + cost
            if candidate < dist.get(neighbour, math.inf):
                dist[neighbour] = candidate
                previous[neighbour] = current
                heapq.heappush(queue, (candidate, neighbour))

    if goal_space_id not in dist:
        return []

    path = [goal_space_id]
    while path[-1] != start_space_id:
        path.append(previous[path[-1]])
    path.reverse()
    return path
