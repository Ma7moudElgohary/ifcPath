from __future__ import annotations

import json
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path

from ifcpath.build_service import build_inav_payload


def _split_space_components(cells: list[dict]) -> dict[str, list[int]]:
    by_space: dict[str, set[str]] = defaultdict(set)
    by_id = {str(cell.get("id")): cell for cell in cells if cell.get("id")}
    for cell_id, cell in by_id.items():
        space_id = cell.get("space_id")
        if space_id:
            by_space[str(space_id)].add(cell_id)

    result: dict[str, list[int]] = {}
    for space_id, ids in by_space.items():
        remaining = set(ids)
        sizes: list[int] = []
        while remaining:
            start = remaining.pop()
            size = 0
            queue = deque([start])
            while queue:
                current_id = queue.popleft()
                size += 1
                for neighbor_id in by_id[current_id].get("neighbor_ids", []):
                    neighbor_id = str(neighbor_id)
                    if neighbor_id in remaining and neighbor_id in ids:
                        remaining.remove(neighbor_id)
                        queue.append(neighbor_id)
            sizes.append(size)
        if len(sizes) > 1:
            result[space_id] = sorted(sizes, reverse=True)
    return dict(sorted(result.items()))


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_local_app_build.py <model.ifc>")
    path = Path(sys.argv[1])
    result = build_inav_payload(path.read_bytes(), path.name)
    report = result["qualification"]
    model = result.get("model", {})
    stats = report.get("stats", {})
    metadata = model.get("metadata", {})
    cells = list(model.get("cells", []))

    bound_portal_ids = {
        str(portal_id)
        for cell in cells
        for portal_id in dict(cell.get("portal_ids", {})).values()
        if portal_id
    }
    missing_internal_doors = [
        {
            "id": portal.get("id"),
            "from_space_id": portal.get("from_space_id"),
            "to_space_id": portal.get("to_space_id"),
            "level_id": portal.get("level_id"),
            "position_m": portal.get("position_m"),
        }
        for portal in model.get("portals", [])
        if portal.get("kind") == "door"
        and portal.get("from_space_id")
        and portal.get("to_space_id")
        and str(portal.get("id")) not in bound_portal_ids
    ]
    surface_transitions = [
        {
            "id": transition.get("id"),
            "kind": transition.get("kind"),
            "from_space_id": transition.get("from_space_id"),
            "to_space_id": transition.get("to_space_id"),
            "from_level_id": transition.get("from_level_id"),
            "to_level_id": transition.get("to_level_id"),
            "resource_id": transition.get("resource_id"),
        }
        for transition in model.get("transitions", [])
        if transition.get("source") == "surface_vertical_touch"
    ]
    terrain_counts = Counter(str(cell.get("terrain", "open")) for cell in cells)

    reconstruction = metadata.get("surface_reconstruction")
    diagnostic = {
        "surface_source": metadata.get("surface_source"),
        "surface_reconstruction": reconstruction,
        "surface_fragment_pruning": metadata.get("surface_fragment_pruning"),
        "surface_postbind_fragment_pruning": metadata.get("surface_postbind_fragment_pruning"),
        "surface_cell_count": stats.get("surface_cell_count"),
        "surface_component_count": stats.get("surface_component_count"),
        "surface_space_count": stats.get("surface_space_count"),
        "surface_portal_crossings": stats.get("surface_portal_crossings"),
        "surface_portal_side_failures": stats.get("surface_portal_side_failures"),
        "surface_vertical_transitions": stats.get("surface_vertical_transitions"),
        "surface_vertical_transfer_failures": stats.get("surface_vertical_transfer_failures"),
        "surface_exit_reachable_spaces": stats.get("surface_exit_reachable_spaces"),
        "surface_exit_unreachable_spaces": stats.get("surface_exit_unreachable_spaces"),
        "surface_navigation_ready": stats.get("surface_navigation_ready"),
        "terrain_cell_counts": dict(sorted(terrain_counts.items())),
        "missing_internal_doors": missing_internal_doors,
        "split_space_component_sizes": _split_space_components(cells),
        "surface_transition_details": surface_transitions,
        "valid": report.get("valid", False),
        "issues": [
            {
                "severity": issue.get("severity"),
                "code": issue.get("code"),
                "entity_id": issue.get("entity_id"),
                "message": issue.get("message"),
            }
            for issue in report.get("issues", [])
            if issue.get("severity") == "error"
            or str(issue.get("code", "")).startswith("SURFACE_")
        ],
    }
    print("local-app physical surface diagnostics:")
    print(json.dumps(diagnostic, indent=2, sort_keys=True))

    if not report.get("valid", False):
        raise SystemExit("local-app IFC build validation failed")
    if not stats.get("surface_authoritative", False):
        raise SystemExit("local-app IFC build did not produce authoritative surface cells")
    if not stats.get("surface_navigation_ready", False):
        raise SystemExit("local-app IFC build did not produce a navigation-ready surface")
    if stats.get("surface_exit_unreachable_spaces", 1) != 0:
        raise SystemExit("local-app IFC build left occupant-egress spaces unreachable")
    print(
        "local-app IFC build passed: "
        f"source={metadata.get('surface_source')} "
        f"cells={stats.get('surface_cell_count')} "
        f"spaces={stats.get('surface_internal_space_count')} "
        f"portals={stats.get('portals')}"
    )


if __name__ == "__main__":
    main()
