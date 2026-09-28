from __future__ import annotations

import heapq
import math
from collections import defaultdict, deque
from dataclasses import dataclass

from .model import InavModel, SemanticTransition


@dataclass(slots=True)
class SemanticRouteOptions:
    blocked_portals: set[str] | None = None
    blocked_spaces: set[str] | None = None
    space_cost_multipliers: dict[str, float] | None = None


def ensure_semantic_transitions(model: InavModel) -> list[SemanticTransition]:
    """Populate the INAV dual graph from portals and vertical metric components.

    Horizontal connectivity is derived from qualified semantic portals, which in
    turn prefer `IfcRelSpaceBoundary*`. IFC generally lacks explicit storey-to-
    storey navigability relationships, so stairs/ramps use the same geometry-
    assisted principle reported in IFC-Graph research: identify connected
    vertical geometry and the landing spaces it touches, then materialize those
    links as semantic transitions.

    Existing transitions are preserved so explicitly authored/imported vertical
    transitions can override inference in future schema versions.
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
            resource_id=portal.id,
        ))
        existing_portals.add(portal.id)

    _ensure_vertical_transitions(model)
    return model.transitions


def _ensure_vertical_transitions(model: InavModel) -> None:
    vertical_kinds = {"stair", "ramp", "elevator", "escalator"}
    node_by_id = {node.id: node for node in model.nodes}
    vertical_ids = {node.id for node in model.nodes if node.kind in vertical_kinds}
    if not vertical_ids:
        return

    adjacency: dict[str, list[tuple[str, float]]] = defaultdict(list)
    for edge in model.edges:
        if edge.a not in node_by_id or edge.b not in node_by_id:
            continue
        adjacency[edge.a].append((edge.b, edge.distance_m))
        adjacency[edge.b].append((edge.a, edge.distance_m))

    # Do not create a second inferred link between the same spaces/kind when an
    # explicit or previous inferred transition already exists.
    existing_pairs = {
        (transition.kind, *sorted((transition.from_space_id, transition.to_space_id)))
        for transition in model.transitions
        if transition.to_space_id
    }
    level_elevation = {level.id: level.elevation_m for level in model.levels}

    for component_index, component in enumerate(_vertical_components(vertical_ids, adjacency)):
        kinds = {node_by_id[node_id].kind for node_id in component}
        kind = next(iter(kinds)) if len(kinds) == 1 else "vertical"

        # For each level, keep the landing space with the shortest direct metric
        # attachment to this vertical component. This avoids all-to-all shortcuts
        # when a landing is geometrically close to several rooms.
        landing_by_level: dict[str, tuple[float, str, float]] = {}
        for vertical_id in component:
            vertical_node = node_by_id[vertical_id]
            for neighbour_id, edge_distance in adjacency.get(vertical_id, ()):
                if neighbour_id in component:
                    continue
                neighbour = node_by_id.get(neighbour_id)
                if neighbour is None or not neighbour.space_id or not neighbour.level_id:
                    continue
                candidate = (edge_distance, neighbour.space_id, neighbour.position_m[2])
                current = landing_by_level.get(neighbour.level_id)
                if current is None or candidate[0] < current[0]:
                    landing_by_level[neighbour.level_id] = candidate

        if len(landing_by_level) < 2:
            continue

        ordered_levels = sorted(
            landing_by_level,
            key=lambda level_id: (
                level_elevation.get(level_id, landing_by_level[level_id][2]),
                level_id,
            ),
        )

        # A connector serving multiple storeys creates transitions only between
        # adjacent served levels. Routing can chain them for longer travel. All
        # adjacent edges retain one physical resource ID so runtime systems can
        # model a shared elevator car / stair / ramp resource instead of creating
        # an independent resource per floor-to-floor edge.
        representative = min(component)
        resource_id = f"vertical:{kind}:{representative}"
        for lower_level, upper_level in zip(ordered_levels, ordered_levels[1:]):
            lower_space = landing_by_level[lower_level][1]
            upper_space = landing_by_level[upper_level][1]
            if lower_space == upper_space:
                continue
            pair_key = (kind, *sorted((lower_space, upper_space)))
            if pair_key in existing_pairs:
                continue
            model.transitions.append(SemanticTransition(
                id=(
                    f"transition:vertical:{kind}:{component_index}:"
                    f"{lower_space}:{upper_space}:{representative}"
                ),
                kind=kind,
                from_space_id=lower_space,
                to_space_id=upper_space,
                portal_id=None,
                from_level_id=lower_level,
                to_level_id=upper_level,
                bidirectional=True,
                source="metric_vertical_touch",
                resource_id=resource_id,
            ))
            existing_pairs.add(pair_key)


def _vertical_components(
    vertical_ids: set[str],
    adjacency: dict[str, list[tuple[str, float]]],
) -> list[set[str]]:
    remaining = set(vertical_ids)
    result: list[set[str]] = []
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
        result.append(component)
    return result


def find_space_path(
    model: InavModel,
    start_space_id: str,
    goal_space_id: str,
    options: SemanticRouteOptions | None = None,
) -> list[str]:
    """Find a high-level space sequence through the semantic dual graph.

    This is intentionally a topology route, not the final metric route. The
    metric graph is still responsible for exact local distance/geometry between
    transfers. Space multipliers allow Digital Twin state (smoke, crowd, access
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
            continue
        if a not in known_spaces or b not in known_spaces:
            continue
        if a in blocked_spaces and a != start_space_id:
            continue
        if b in blocked_spaces and b != start_space_id:
            continue

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
