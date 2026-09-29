from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field

from .model import InavModel, NavCell
from .vertical_surface import find_surface_vertical_transfer


@dataclass(frozen=True, slots=True)
class SurfaceReadinessIssue:
    severity: str
    code: str
    message: str
    entity_id: str | None = None


@dataclass(slots=True)
class SurfaceReadiness:
    available: bool
    ready: bool
    issues: list[SurfaceReadinessIssue] = field(default_factory=list)
    stats: dict[str, int | float | bool] = field(default_factory=dict)


def assess_surface_readiness(model: InavModel) -> SurfaceReadiness:
    """Validate the authoritative NavCell + semantic navigation representation.

    Pure surface connected-component count is diagnostic only. Elevators are
    intentionally off-mesh resources, so routability is decided by the semantic
    space graph layered over qualified surface spaces, while doors/stairs/ramps
    still have to be backed by valid NavCell topology.

    Exterior or explicitly service-only spaces remain valid navigation geometry
    but are not occupant-egress domains. Their lack of an indoor evacuation exit
    therefore does not make an otherwise qualified building unroutable.
    """
    if not model.cells:
        return SurfaceReadiness(
            available=False,
            ready=False,
            stats={
                "surface_authoritative": False,
                "surface_cell_count": 0,
                "surface_component_count": 0,
                "surface_navigation_ready": False,
            },
        )

    issues: list[SurfaceReadinessIssue] = []
    cell_ids = [cell.id for cell in model.cells]
    unique_ids = set(cell_ids)
    duplicate_cell_ids = len(cell_ids) - len(unique_ids)
    if duplicate_cell_ids:
        issues.append(SurfaceReadinessIssue(
            "error",
            "SURFACE_DUPLICATE_CELL_ID",
            f"Navigation surface contains {duplicate_cell_ids} duplicate cell ID(s)",
        ))

    by_id = {cell.id: cell for cell in model.cells}
    known_portals = {portal.id: portal for portal in model.portals}
    adjacency: dict[str, set[str]] = {cell_id: set() for cell_id in by_id}
    topology_errors = 0
    asymmetric_edges = 0
    semantic_crossings: dict[str, set[tuple[str, str]]] = defaultdict(set)

    seen_pairs: set[tuple[str, str]] = set()
    for cell in model.cells:
        for neighbor_id in cell.neighbor_ids:
            neighbor = by_id.get(neighbor_id)
            if neighbor is None:
                topology_errors += 1
                issues.append(SurfaceReadinessIssue(
                    "error",
                    "SURFACE_UNKNOWN_NEIGHBOR",
                    f"Surface cell references missing neighbor {neighbor_id}",
                    cell.id,
                ))
                continue
            adjacency[cell.id].add(neighbor_id)
            if cell.id not in neighbor.neighbor_ids:
                asymmetric_edges += 1
                topology_errors += 1
                issues.append(SurfaceReadinessIssue(
                    "error",
                    "SURFACE_ASYMMETRIC_ADJACENCY",
                    f"Surface adjacency is not reciprocal: {cell.id} -> {neighbor_id}",
                    cell.id,
                ))

            pair = tuple(sorted((cell.id, neighbor_id)))
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)
            _validate_surface_crossing(
                cell,
                neighbor,
                known_portals,
                semantic_crossings,
                issues,
            )

    topology_errors += sum(issue.severity == "error" for issue in issues) - topology_errors

    components = _components(adjacency)
    component_count = len(components)
    largest_component_cells = max((len(component) for component in components), default=0)
    surface_reachable_ratio = (
        largest_component_cells / len(model.cells) if model.cells else 0.0
    )

    open_cells = [cell for cell in model.cells if cell.terrain == "open"]
    surface_spaces = {cell.space_id for cell in open_cells if cell.space_id}
    space_by_id = {space.id: space for space in model.spaces}
    external_surface_spaces = {
        space_id
        for space_id in surface_spaces
        if space_by_id.get(space_id) is not None and space_by_id[space_id].is_external
    }
    service_surface_spaces = {
        space_id
        for space_id in surface_spaces
        if space_by_id.get(space_id) is not None
        and not space_by_id[space_id].is_external
        and not space_by_id[space_id].egress_required
    }
    egress_exempt_surface_spaces = external_surface_spaces | service_surface_spaces
    required_surface_spaces = surface_spaces - egress_exempt_surface_spaces
    surface_levels = {cell.level_id for cell in open_cells if cell.level_id}
    required_surface_levels = {
        cell.level_id
        for cell in open_cells
        if cell.level_id and cell.space_id in required_surface_spaces
    }
    unowned_open_cells = sum(cell.space_id is None for cell in open_cells)

    split_surface_spaces = 0
    for space_id in sorted(surface_spaces):
        owned_ids = {cell.id for cell in open_cells if cell.space_id == space_id}
        local_adjacency = {
            cell_id: {neighbor for neighbor in adjacency.get(cell_id, ()) if neighbor in owned_ids}
            for cell_id in owned_ids
        }
        local_components = _components(local_adjacency)
        if len(local_components) > 1:
            split_surface_spaces += 1
            issues.append(SurfaceReadinessIssue(
                "warning",
                "SURFACE_SPACE_SPLIT_COMPONENTS",
                f"Space navigation surface is split across {len(local_components)} components",
                space_id,
            ))

    surface_portal_side_failures = 0
    for portal in model.portals:
        expected = {side for side in (portal.from_space_id, portal.to_space_id) if side}
        if len(expected) != 2 or not expected.issubset(surface_spaces):
            continue
        observed = semantic_crossings.get(portal.id, set())
        if not any(set(pair) == expected for pair in observed):
            surface_portal_side_failures += 1
            issues.append(SurfaceReadinessIssue(
                "warning",
                "SURFACE_PORTAL_MISSING_SIDE",
                "Semantic portal is not backed by a surface crossing between both spaces",
                portal.id,
            ))

    surface_vertical_transitions = [
        transition
        for transition in model.transitions
        if transition.source == "surface_vertical_touch"
    ]
    surface_vertical_transfer_failures = 0
    for transition in surface_vertical_transitions:
        if find_surface_vertical_transfer(model, transition) is None:
            surface_vertical_transfer_failures += 1
            issues.append(SurfaceReadinessIssue(
                "error",
                "SURFACE_VERTICAL_TRANSFER_UNROUTABLE",
                "Surface-derived vertical transition does not produce a continuous funnel route",
                transition.id,
            ))

    semantic_undirected: dict[str, set[str]] = {space_id: set() for space_id in surface_spaces}
    reverse_reachability: dict[str, set[str]] = {space_id: set() for space_id in surface_spaces}
    cross_level_transitions = 0
    for transition in model.transitions:
        a = transition.from_space_id
        b = transition.to_space_id
        if not b or a not in surface_spaces or b not in surface_spaces:
            continue
        semantic_undirected[a].add(b)
        semantic_undirected[b].add(a)
        reverse_reachability[b].add(a)
        if transition.bidirectional:
            reverse_reachability[a].add(b)
        if (
            transition.from_level_id
            and transition.to_level_id
            and transition.from_level_id != transition.to_level_id
        ):
            cross_level_transitions += 1

    semantic_components = _components(semantic_undirected)
    semantic_component_count = len(semantic_components)

    exit_spaces: set[str] = set()
    for portal in model.portals:
        if not portal.is_exit:
            continue
        for space_id in (portal.from_space_id, portal.to_space_id):
            if space_id in surface_spaces:
                exit_spaces.add(space_id)

    reachable_exit_spaces: set[str] = set()
    if exit_spaces:
        reachable_exit_spaces = _reachable_from(exit_spaces, reverse_reachability)
    required_reachable_spaces = reachable_exit_spaces & required_surface_spaces
    unreachable_required_spaces = (
        required_surface_spaces - reachable_exit_spaces if exit_spaces else set()
    )
    surface_exit_reachable_ratio = (
        len(required_reachable_spaces) / len(required_surface_spaces)
        if required_surface_spaces and exit_spaces
        else (1.0 if not required_surface_spaces else 0.0)
    )

    exits = [portal for portal in model.portals if portal.is_exit]
    if exits and not exit_spaces:
        issues.append(SurfaceReadinessIssue(
            "warning",
            "SURFACE_EXIT_UNBOUND",
            "Classified exits are not attached to any surfaced navigation space",
        ))
    elif unreachable_required_spaces:
        issues.append(SurfaceReadinessIssue(
            "warning",
            "SURFACE_SPACES_CANNOT_REACH_EXIT",
            f"{len(unreachable_required_spaces)} occupant-egress surfaced space(s) cannot reach a classified exit",
        ))

    multi_level_surface = len(required_surface_levels) > 1
    cross_level_ready = (not multi_level_surface) or cross_level_transitions > 0
    if multi_level_surface and not cross_level_ready:
        issues.append(SurfaceReadinessIssue(
            "warning",
            "SURFACE_NO_CROSS_LEVEL_TRANSITION",
            "Multiple occupant-egress surfaced levels exist but no semantic cross-level transition connects them",
        ))

    if required_surface_spaces:
        if exits:
            connectivity_ready = bool(exit_spaces) and not unreachable_required_spaces
        else:
            required_components = _components({
                space_id: {
                    neighbor for neighbor in semantic_undirected.get(space_id, ())
                    if neighbor in required_surface_spaces
                }
                for space_id in required_surface_spaces
            })
            connectivity_ready = len(required_components) <= 1
    else:
        connectivity_ready = True

    surface_errors = sum(issue.severity == "error" for issue in issues)
    ready = (
        bool(model.cells)
        and surface_errors == 0
        and split_surface_spaces == 0
        and surface_portal_side_failures == 0
        and surface_vertical_transfer_failures == 0
        and cross_level_ready
        and connectivity_ready
    )

    return SurfaceReadiness(
        available=True,
        ready=ready,
        issues=issues,
        stats={
            "surface_authoritative": True,
            "surface_cell_count": len(model.cells),
            "surface_open_cell_count": len(open_cells),
            "surface_unowned_open_cells": unowned_open_cells,
            "surface_component_count": component_count,
            "surface_largest_component_cells": largest_component_cells,
            "surface_reachable_ratio": surface_reachable_ratio,
            "surface_space_count": len(surface_spaces),
            "surface_internal_space_count": len(required_surface_spaces),
            "surface_external_space_count": len(external_surface_spaces),
            "surface_service_space_count": len(service_surface_spaces),
            "surface_egress_exempt_space_count": len(egress_exempt_surface_spaces),
            "surface_level_count": len(surface_levels),
            "surface_internal_level_count": len(required_surface_levels),
            "surface_semantic_component_count": semantic_component_count,
            "surface_split_spaces": split_surface_spaces,
            "surface_portal_crossings": sum(len(pairs) for pairs in semantic_crossings.values()),
            "surface_portal_side_failures": surface_portal_side_failures,
            "surface_vertical_transitions": len(surface_vertical_transitions),
            "surface_vertical_transfer_failures": surface_vertical_transfer_failures,
            "surface_cross_level_transitions": cross_level_transitions,
            "surface_exit_spaces": len(exit_spaces),
            "surface_exit_reachable_spaces": len(required_reachable_spaces),
            "surface_exit_unreachable_spaces": len(unreachable_required_spaces),
            "surface_exit_reachable_ratio": surface_exit_reachable_ratio,
            "surface_topology_errors": topology_errors,
            "surface_asymmetric_edges": asymmetric_edges,
            "surface_navigation_ready": ready,
        },
    )


def _validate_surface_crossing(
    a: NavCell,
    b: NavCell,
    known_portals,
    semantic_crossings: dict[str, set[tuple[str, str]]],
    issues: list[SurfaceReadinessIssue],
) -> None:
    if not a.space_id or not b.space_id or a.space_id == b.space_id:
        return
    if a.terrain != "open" or b.terrain != "open":
        return

    left_id = a.portal_ids.get(b.id)
    right_id = b.portal_ids.get(a.id)
    if left_id and right_id and left_id != right_id:
        issues.append(SurfaceReadinessIssue(
            "error",
            "SURFACE_PORTAL_ID_MISMATCH",
            f"Surface crossing stores different portal IDs: {left_id} vs {right_id}",
            a.id,
        ))
        return
    portal_id = left_id or right_id
    if not portal_id:
        issues.append(SurfaceReadinessIssue(
            "error",
            "SURFACE_UNAUTHORIZED_SPACE_CROSSING",
            f"Open surface crosses directly from {a.space_id} to {b.space_id} without a semantic portal",
            a.id,
        ))
        return
    portal = known_portals.get(portal_id)
    if portal is None:
        issues.append(SurfaceReadinessIssue(
            "error",
            "SURFACE_UNKNOWN_PORTAL",
            f"Surface crossing references unknown portal {portal_id}",
            a.id,
        ))
        return
    expected = {side for side in (portal.from_space_id, portal.to_space_id) if side}
    observed = {a.space_id, b.space_id}
    if expected != observed:
        issues.append(SurfaceReadinessIssue(
            "error",
            "SURFACE_PORTAL_SPACE_MISMATCH",
            "Surface crossing spaces do not match the semantic portal sides",
            portal_id,
        ))
        return
    semantic_crossings[portal_id].add(tuple(sorted(observed)))


def _components(adjacency: dict[str, set[str]]) -> list[set[str]]:
    remaining = set(adjacency)
    result: list[set[str]] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        component = {start}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for neighbor in adjacency.get(current, ()):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)
        result.append(component)
    return result


def _reachable_from(starts: set[str], adjacency: dict[str, set[str]]) -> set[str]:
    reached = set(starts)
    queue = deque(sorted(starts))
    while queue:
        current = queue.popleft()
        for neighbor in adjacency.get(current, ()):
            if neighbor in reached:
                continue
            reached.add(neighbor)
            queue.append(neighbor)
    return reached
