from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import asdict, dataclass, field

from .model import InavModel


@dataclass(slots=True)
class ValidationIssue:
    severity: str
    code: str
    message: str
    entity_id: str | None = None


@dataclass(slots=True)
class ValidationReport:
    valid: bool
    node_count: int
    edge_count: int
    component_count: int
    largest_component_nodes: int
    reachable_node_ratio: float
    issues: list[ValidationIssue] = field(default_factory=list)
    stats: dict[str, int | float] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


def validate_model(model: InavModel) -> ValidationReport:
    issues: list[ValidationIssue] = []
    node_ids = {node.id for node in model.nodes}
    node_by_id = {node.id: node for node in model.nodes}
    adjacency: dict[str, set[str]] = {node_id: set() for node_id in node_ids}
    portal_edge_counts: dict[str, int] = defaultdict(int)

    for edge in model.edges:
        if edge.a not in node_ids or edge.b not in node_ids:
            issues.append(ValidationIssue(
                "error",
                "EDGE_MISSING_NODE",
                f"Edge references missing node: {edge.a} <-> {edge.b}",
            ))
            continue
        if edge.a == edge.b:
            issues.append(ValidationIssue("warning", "SELF_EDGE", f"Self edge at node {edge.a}", edge.a))
        if edge.distance_m < 0:
            issues.append(ValidationIssue("error", "NEGATIVE_DISTANCE", "Edge has negative distance", edge.a))
        adjacency[edge.a].add(edge.b)
        adjacency[edge.b].add(edge.a)
        if edge.portal_id:
            portal_edge_counts[edge.portal_id] += 1

    isolated_nodes = [node_id for node_id, neighbours in adjacency.items() if not neighbours]
    for node_id in isolated_nodes[:100]:
        issues.append(ValidationIssue("warning", "ISOLATED_NODE", "Navigation node has no edges", node_id))

    components = _components(adjacency)
    component_count = len(components)
    largest = max((len(component) for component in components), default=0)
    ratio = largest / len(node_ids) if node_ids else 0.0
    component_index = {
        node_id: index
        for index, component in enumerate(components)
        for node_id in component
    }

    if not node_ids:
        issues.append(ValidationIssue("error", "NO_NODES", "Navigation model contains no nodes"))
    elif component_count > 1:
        issues.append(ValidationIssue(
            "warning",
            "DISCONNECTED_GRAPH",
            f"Navigation graph has {component_count} connected components; largest contains {largest}/{len(node_ids)} nodes",
        ))

    known_portals = {portal.id for portal in model.portals}
    for edge in model.edges:
        if edge.portal_id and edge.portal_id not in known_portals:
            issues.append(ValidationIssue(
                "error",
                "UNKNOWN_PORTAL",
                f"Edge references unknown portal {edge.portal_id}",
                edge.portal_id,
            ))

    exits = [portal for portal in model.portals if portal.is_exit]
    portal_side_failures = 0
    for portal in model.portals:
        portal_nodes = [node for node in model.nodes if node.portal_id == portal.id]
        edge_count = portal_edge_counts.get(portal.id, 0)
        if not portal_nodes:
            issues.append(ValidationIssue("warning", "PORTAL_NO_NODE", "Portal has no navigation node", portal.id))
            continue
        if edge_count == 0 and not any(adjacency.get(node.id) for node in portal_nodes):
            issues.append(ValidationIssue("warning", "PORTAL_DISCONNECTED", "Portal is disconnected from navigation graph", portal.id))

        # A semantic two-sided door must actually attach to nodes belonging to
        # both spaces. This is a stronger topology invariant than global graph
        # connectivity because buildings may contain independent wings/units.
        attached_spaces: set[str] = set()
        for portal_node in portal_nodes:
            for neighbour_id in adjacency.get(portal_node.id, ()):
                neighbour = node_by_id.get(neighbour_id)
                if neighbour and neighbour.space_id:
                    attached_spaces.add(neighbour.space_id)
        expected_sides = {
            side for side in (portal.from_space_id, portal.to_space_id) if side
        }
        missing_sides = expected_sides - attached_spaces
        if missing_sides:
            portal_side_failures += 1
            issues.append(ValidationIssue(
                "warning",
                "PORTAL_MISSING_SIDE",
                f"Portal is not attached to semantic side(s): {', '.join(sorted(missing_sides))}",
                portal.id,
            ))

    if model.portals and not exits:
        issues.append(ValidationIssue("warning", "NO_EXIT", "No portal is classified as an exit"))

    nodes_by_space: dict[str, int] = defaultdict(int)
    nodes_by_level: dict[str, int] = defaultdict(int)
    space_components: dict[str, set[int]] = defaultdict(set)
    for node in model.nodes:
        if node.space_id:
            nodes_by_space[node.space_id] += 1
            if node.kind != "portal" and node.id in component_index:
                space_components[node.space_id].add(component_index[node.id])
        if node.level_id:
            nodes_by_level[node.level_id] += 1

    split_spaces = 0
    for space in model.spaces:
        if nodes_by_space.get(space.id, 0) == 0:
            issues.append(ValidationIssue("warning", "SPACE_NO_NAV", "Space has no assigned navigation nodes", space.id))
            continue
        count = len(space_components.get(space.id, ()))
        if count > 1:
            split_spaces += 1
            issues.append(ValidationIssue(
                "warning",
                "SPACE_SPLIT_COMPONENTS",
                f"Space navigation is split across {count} graph components",
                space.id,
            ))

    for level in model.levels:
        if nodes_by_level.get(level.id, 0) == 0:
            issues.append(ValidationIssue("warning", "LEVEL_NO_NAV", "Level has no assigned navigation nodes", level.id))

    exit_node_ids = {
        node.id
        for node in model.nodes
        if node.portal_id and any(p.id == node.portal_id and p.is_exit for p in exits)
    }
    exit_reachable_nodes = 0
    exit_unreachable_nodes = len(node_ids)
    exit_reachable_ratio = 0.0
    if exits and exit_node_ids:
        exit_component_nodes: set[str] = set()
        for component in components:
            if component & exit_node_ids:
                exit_component_nodes |= component
        exit_reachable_nodes = len(exit_component_nodes)
        exit_unreachable_nodes = len(node_ids - exit_component_nodes)
        exit_reachable_ratio = exit_reachable_nodes / len(node_ids) if node_ids else 0.0
        if exit_unreachable_nodes:
            issues.append(ValidationIssue(
                "warning",
                "NODES_CANNOT_REACH_EXIT",
                f"{exit_unreachable_nodes} navigation nodes cannot reach any classified exit",
            ))

    error_count = sum(issue.severity == "error" for issue in issues)
    warning_count = sum(issue.severity == "warning" for issue in issues)
    return ValidationReport(
        valid=error_count == 0,
        node_count=len(model.nodes),
        edge_count=len(model.edges),
        component_count=component_count,
        largest_component_nodes=largest,
        reachable_node_ratio=ratio,
        issues=issues,
        stats={
            "levels": len(model.levels),
            "spaces": len(model.spaces),
            "portals": len(model.portals),
            "exits": len(exits),
            "isolated_nodes": len(isolated_nodes),
            "split_spaces": split_spaces,
            "portal_side_failures": portal_side_failures,
            "exit_reachable_nodes": exit_reachable_nodes,
            "exit_unreachable_nodes": exit_unreachable_nodes,
            "exit_reachable_ratio": exit_reachable_ratio,
            "errors": error_count,
            "warnings": warning_count,
        },
    )


def _components(adjacency: dict[str, set[str]]) -> list[set[str]]:
    remaining = set(adjacency)
    result: list[set[str]] = []
    while remaining:
        start = next(iter(remaining))
        component: set[str] = set()
        queue = deque([start])
        remaining.remove(start)
        while queue:
            current = queue.popleft()
            component.add(current)
            for neighbour in adjacency[current]:
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    queue.append(neighbour)
        result.append(component)
    return result
