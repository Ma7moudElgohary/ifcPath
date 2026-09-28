from __future__ import annotations

from collections import defaultdict, deque

from .models import NavigationModel


def validate_navigation(model: NavigationModel) -> list[str]:
    warnings = list(model.warnings)
    node_ids = {n.id for n in model.nodes}
    adjacency: dict[str, set[str]] = defaultdict(set)

    for e in model.edges:
        if e.a not in node_ids or e.b not in node_ids:
            warnings.append(f"Edge references a missing node: {e.a} <-> {e.b}")
            continue
        adjacency[e.a].add(e.b)
        adjacency[e.b].add(e.a)

    for p in model.portals:
        if p.id not in adjacency:
            warnings.append(f"Portal {p.id} is disconnected from the navigation graph.")

    exits = [p for p in model.portals if p.is_external or p.kind == "exit"]
    if not exits:
        warnings.append("No external/exit portals were detected.")
        return _dedupe(warnings)

    reachable: set[str] = set()
    q = deque(p.id for p in exits if p.id in node_ids)
    reachable.update(q)
    while q:
        u = q.popleft()
        for v in adjacency.get(u, ()):
            if v not in reachable:
                reachable.add(v)
                q.append(v)

    for n in model.nodes:
        if n.id not in reachable:
            warnings.append(f"Navigation node {n.id} cannot reach any detected exit.")
            if len(warnings) > 200:
                warnings.append("Additional unreachable-node warnings omitted.")
                break

    node_spaces = {n.space_id for n in model.nodes if n.space_id}
    for s in model.spaces:
        if s.id not in node_spaces:
            warnings.append(f"Space {s.id} has no sampled navigation node.")

    return _dedupe(warnings)


def _dedupe(items: list[str]) -> list[str]:
    return list(dict.fromkeys(items))
