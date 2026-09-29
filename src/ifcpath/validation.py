from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import asdict, dataclass, field

from .model import InavModel
from .semantic import ensure_semantic_transitions
from .surface_readiness import assess_surface_readiness
from .vertical_surface import ensure_surface_vertical_transitions


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
    stats: dict[str, int | float | bool] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


def validate_model(model: InavModel) -> ValidationReport:
    """Validate portable semantics and both navigation representations.

    ``NavCell`` surface geometry is authoritative whenever it is present. The
    sampled node/edge graph is still validated and reported because old INAV
    files and several compatibility tools consume it, but its fragmentation no
    longer makes a qualified modern surface model "not routable".
    """
    ensure_surface_vertical_transitions(model)
    ensure_semantic_transitions(model)
    issues: list[ValidationIssue] = []

    # ------------------------------------------------------------------
    # Legacy metric graph telemetry / compatibility validation.
    # ------------------------------------------------------------------
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

    if not node_ids and not model.cells:
        issues.append(ValidationIssue(
            "error",
            "NO_NODES",
            "Navigation model contains neither surface cells nor compatibility nodes",
        ))
    elif node_ids and component_count > 1:
        issues.append(ValidationIssue(
            "warning",
            "DISCONNECTED_GRAPH",
            (
                "Legacy navigation graph has "
                f"{component_count} connected components; largest contains "
                f"{largest}/{len(node_ids)} nodes"
            ),
        ))

    known_portals = {portal.id for portal in model.portals}
    portal_by_id = {portal.id: portal for portal in model.portals}
    known_spaces = {space.id for space in model.spaces}
    space_by_id = {space.id: space for space in model.spaces}

    for edge in model.edges:
        if edge.portal_id and edge.portal_id not in known_portals:
            issues.append(ValidationIssue(
                "error",
                "UNKNOWN_PORTAL",
                f"Edge references unknown portal {edge.portal_id}",
                edge.portal_id,
            ))

    # ------------------------------------------------------------------
    # Semantic dual-graph integrity is representation-independent.
    # ------------------------------------------------------------------
    semantic_transition_errors = 0
    transition_ids: set[str] = set()
    for transition in model.transitions:
        if transition.id in transition_ids:
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "DUPLICATE_TRANSITION",
                "Semantic transition ID is duplicated",
                transition.id,
            ))
        transition_ids.add(transition.id)

        if transition.from_space_id not in known_spaces:
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "TRANSITION_UNKNOWN_FROM_SPACE",
                f"Transition references unknown from-space {transition.from_space_id}",
                transition.id,
            ))
        if transition.to_space_id and transition.to_space_id not in known_spaces:
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "TRANSITION_UNKNOWN_TO_SPACE",
                f"Transition references unknown to-space {transition.to_space_id}",
                transition.id,
            ))
        if transition.portal_id and transition.portal_id not in known_portals:
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "TRANSITION_UNKNOWN_PORTAL",
                f"Transition references unknown portal {transition.portal_id}",
                transition.id,
            ))
            continue

        if transition.portal_id:
            portal = portal_by_id[transition.portal_id]
            portal_sides = {x for x in (portal.from_space_id, portal.to_space_id) if x}
            transition_sides = {x for x in (transition.from_space_id, transition.to_space_id) if x}
            if portal_sides != transition_sides:
                semantic_transition_errors += 1
                issues.append(ValidationIssue(
                    "error",
                    "TRANSITION_PORTAL_MISMATCH",
                    "Transition space sides do not match its portal",
                    transition.id,
                ))

        from_space = space_by_id.get(transition.from_space_id)
        to_space = space_by_id.get(transition.to_space_id) if transition.to_space_id else None
        if (
            from_space
            and transition.from_level_id
            and from_space.level_id
            and transition.from_level_id != from_space.level_id
        ):
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "TRANSITION_FROM_LEVEL_MISMATCH",
                "Transition from-level does not match the from-space level",
                transition.id,
            ))
        if (
            to_space
            and transition.to_level_id
            and to_space.level_id
            and transition.to_level_id != to_space.level_id
        ):
            semantic_transition_errors += 1
            issues.append(ValidationIssue(
                "error",
                "TRANSITION_TO_LEVEL_MISMATCH",
                "Transition to-level does not match the to-space level",
                transition.id,
            ))

    exits = [portal for portal in model.portals if portal.is_exit]
    if model.portals and not exits:
        issues.append(ValidationIssue("warning", "NO_EXIT", "No portal is classified as an exit"))

    # ------------------------------------------------------------------
    # Legacy portal/space/level attachment diagnostics. These are intentionally
    # skipped for surface-only modern files so we do not complain about missing
    # compatibility nodes that are no longer required for routing.
    # ------------------------------------------------------------------
    portal_side_failures = 0
    nodes_by_space: dict[str, int] = defaultdict(int)
    nodes_by_level: dict[str, int] = defaultdict(int)
    split_spaces = 0
    if model.nodes:
        for portal in model.portals:
            portal_nodes = [node for node in model.nodes if node.portal_id == portal.id]
            edge_count = portal_edge_counts.get(portal.id, 0)
            if not portal_nodes:
                issues.append(ValidationIssue(
                    "warning", "PORTAL_NO_NODE", "Portal has no compatibility navigation node", portal.id
                ))
                continue
            if edge_count == 0 and not any(adjacency.get(node.id) for node in portal_nodes):
                issues.append(ValidationIssue(
                    "warning",
                    "PORTAL_DISCONNECTED",
                    "Portal is disconnected from the compatibility navigation graph",
                    portal.id,
                ))

            attached_spaces: set[str] = set()
            for portal_node in portal_nodes:
                for neighbour_id in adjacency.get(portal_node.id, ()):
                    neighbour = node_by_id.get(neighbour_id)
                    if neighbour and neighbour.space_id:
                        attached_spaces.add(neighbour.space_id)
            expected_sides = {side for side in (portal.from_space_id, portal.to_space_id) if side}
            missing_sides = expected_sides - attached_spaces
            if missing_sides:
                portal_side_failures += 1
                issues.append(ValidationIssue(
                    "warning",
                    "PORTAL_MISSING_SIDE",
                    f"Compatibility portal node is not attached to side(s): {', '.join(sorted(missing_sides))}",
                    portal.id,
                ))

        space_components: dict[str, set[int]] = defaultdict(set)
        for node in model.nodes:
            if node.space_id:
                nodes_by_space[node.space_id] += 1
                if node.kind != "portal" and node.id in component_index:
                    space_components[node.space_id].add(component_index[node.id])
            if node.level_id:
                nodes_by_level[node.level_id] += 1

        for space in model.spaces:
            if nodes_by_space.get(space.id, 0) == 0:
                issues.append(ValidationIssue(
                    "warning", "SPACE_NO_NAV", "Space has no assigned compatibility navigation nodes", space.id
                ))
                continue
            count = len(space_components.get(space.id, ()))
            if count > 1:
                split_spaces += 1
                issues.append(ValidationIssue(
                    "warning",
                    "SPACE_SPLIT_COMPONENTS",
                    f"Compatibility navigation for the space is split across {count} graph components",
                    space.id,
                ))

        for level in model.levels:
            if nodes_by_level.get(level.id, 0) == 0:
                issues.append(ValidationIssue(
                    "warning", "LEVEL_NO_NAV", "Level has no assigned compatibility navigation nodes", level.id
                ))

    # ------------------------------------------------------------------
    # Legacy node-to-exit telemetry.
    # ------------------------------------------------------------------
    exit_node_ids = {
        node.id
        for node in model.nodes
        if node.portal_id and any(portal.id == node.portal_id and portal.is_exit for portal in exits)
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
                f"{exit_unreachable_nodes} compatibility navigation nodes cannot reach a classified exit",
            ))

    vertical_node_counts = {
        kind: sum(node.kind == kind for node in model.nodes)
        for kind in ("stair", "ramp", "elevator")
    }
    vertical_transitions = [
        transition
        for transition in model.transitions
        if transition.kind in {"stair", "ramp", "elevator", "escalator", "vertical"}
    ]
    vertical_levels = {
        node.level_id
        for node in model.nodes
        if node.kind in {"stair", "ramp", "elevator", "escalator"} and node.level_id
    }
    if (
        len(model.levels) > 1
        and sum(vertical_node_counts.values()) > 0
        and not vertical_transitions
    ):
        issues.append(ValidationIssue(
            "warning",
            "VERTICAL_NO_TRANSITION",
            "Vertical compatibility geometry exists but produced no inter-level semantic transition",
        ))

    levels_without_nav = sum(nodes_by_level.get(level.id, 0) == 0 for level in model.levels)
    vertical_required = len(model.levels) > 1
    vertical_ready = (not vertical_required) or bool(vertical_transitions)
    exits_ready = (not exits) or exit_unreachable_nodes == 0
    legacy_navigation_ready = (
        bool(node_ids)
        and component_count == 1
        and levels_without_nav == 0
        and vertical_ready
        and exits_ready
        and portal_side_failures == 0
        and split_spaces == 0
    )

    # ------------------------------------------------------------------
    # Authoritative modern readiness: continuous surface + semantic resources.
    # ------------------------------------------------------------------
    surface = assess_surface_readiness(model)
    issues.extend(
        ValidationIssue(issue.severity, issue.code, issue.message, issue.entity_id)
        for issue in surface.issues
    )
    navigation_ready = surface.ready if surface.available else legacy_navigation_ready

    error_count = sum(issue.severity == "error" for issue in issues)
    warning_count = sum(issue.severity == "warning" for issue in issues)
    stats: dict[str, int | float | bool] = {
        "levels": len(model.levels),
        "spaces": len(model.spaces),
        "portals": len(model.portals),
        "semantic_transitions": len(model.transitions),
        "semantic_transition_errors": semantic_transition_errors,
        "exits": len(exits),
        "isolated_nodes": len(isolated_nodes),
        "split_spaces": split_spaces,
        "portal_side_failures": portal_side_failures,
        "exit_reachable_nodes": exit_reachable_nodes,
        "exit_unreachable_nodes": exit_unreachable_nodes,
        "exit_reachable_ratio": exit_reachable_ratio,
        "stair_nodes": vertical_node_counts["stair"],
        "ramp_nodes": vertical_node_counts["ramp"],
        "elevator_nodes": vertical_node_counts["elevator"],
        "vertical_levels": len(vertical_levels),
        "vertical_transitions": len(vertical_transitions),
        "levels_without_nav": levels_without_nav,
        "legacy_navigation_ready": legacy_navigation_ready,
        "navigation_ready": navigation_ready,
    }
    stats.update(surface.stats)
    stats["errors"] = error_count
    stats["warnings"] = warning_count

    return ValidationReport(
        valid=error_count == 0,
        node_count=len(model.nodes),
        edge_count=len(model.edges),
        component_count=component_count,
        largest_component_nodes=largest,
        reachable_node_ratio=ratio,
        issues=issues,
        stats=stats,
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
