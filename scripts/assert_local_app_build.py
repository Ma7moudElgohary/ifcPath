from __future__ import annotations

import json
import math
import os
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path

from ifcpath.build_service import build_inav_payload
from ifcpath.exporter import model_from_dict
from ifcpath.hierarchical_routing import find_hierarchical_path
from ifcpath.surface_nav import cell_centroid
from ifcpath.vertical_surface import surface_vertical_components


_VERTICAL_KINDS = {"stair", "ramp", "escalator"}


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


def _qualify_physical_multilevel_route(raw_model: dict) -> dict | None:
    """Route between actual surface-vertical landings on the final physical mesh."""
    model = model_from_dict(raw_model)
    level_elevation = {level.id: level.elevation_m for level in model.levels}
    cross_level = [
        transition
        for transition in model.transitions
        if transition.source == "surface_vertical_touch"
        and transition.kind in _VERTICAL_KINDS
        and transition.to_space_id
        and transition.from_level_id
        and transition.to_level_id
        and transition.from_level_id != transition.to_level_id
    ]
    if not cross_level:
        return None

    components = {
        component.resource_id: component
        for component in surface_vertical_components(model)
    }
    by_id = {cell.id: cell for cell in model.cells}
    surface_transition_ids = {
        transition.id
        for transition in model.transitions
        if transition.source == "surface_vertical_touch"
    }

    failures: list[str] = []
    for transition in cross_level:
        component = components.get(transition.resource_id or "")
        if component is None:
            failures.append(f"{transition.id}: resource missing")
            continue
        from_landings = [
            landing
            for landing in component.landings
            if landing.space_id == transition.from_space_id
            and landing.level_id == transition.from_level_id
            and landing.open_cell_id in by_id
        ]
        to_landings = [
            landing
            for landing in component.landings
            if landing.space_id == transition.to_space_id
            and landing.level_id == transition.to_level_id
            and landing.open_cell_id in by_id
        ]
        if not from_landings or not to_landings:
            failures.append(f"{transition.id}: landing cell missing")
            continue

        start = cell_centroid(by_id[from_landings[0].open_cell_id])
        goal = cell_centroid(by_id[to_landings[0].open_cell_id])
        route = find_hierarchical_path(model, start, goal)
        if route is None or len(route.points) < 2:
            failures.append(f"{transition.id}: no hierarchical route")
            continue
        used_surface_transition = any(
            transition_id in surface_transition_ids
            for transition_id in route.transition_ids
        )
        if not used_surface_transition:
            failures.append(f"{transition.id}: route did not use physical vertical semantics")
            continue

        zs = [point[2] for point in route.points]
        vertical_rise = max(zs) - min(zs)
        expected_rise = abs(
            level_elevation.get(transition.to_level_id, goal[2])
            - level_elevation.get(transition.from_level_id, start[2])
        )
        required_rise = min(0.50, expected_rise * 0.50) if expected_rise > 0.0 else 0.0
        if vertical_rise + 1e-6 < required_rise:
            failures.append(
                f"{transition.id}: route rise {vertical_rise:.3f}m below {required_rise:.3f}m"
            )
            continue

        return {
            "source_transition_id": transition.id,
            "route_transition_ids": route.transition_ids,
            "space_ids": route.space_ids,
            "length_m": route.length_m,
            "waypoint_count": len(route.points),
            "vertical_rise_m": vertical_rise,
            "expected_level_rise_m": expected_rise,
            "start": start,
            "goal": goal,
        }

    raise SystemExit(
        "local-app physical multi-level route qualification failed: "
        + "; ".join(failures[:8])
    )


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_local_app_build.py <model.ifc>")
    path = Path(sys.argv[1])
    result = build_inav_payload(path.read_bytes(), path.name)
    report = result["qualification"]
    model = result.get("model", {})

    # Keep the geometry-first build itself as a CI artifact. The qualification
    # summary is intentionally strict, but when it fails we still need the exact
    # physical cells/adjacency to diagnose where topology was cut.
    diagnostic_model_path = Path(
        os.environ.get("IFCPATH_PHYSICAL_MODEL_OUTPUT", "/tmp/local_app_physical.inav")
    )
    diagnostic_model_path.write_text(
        json.dumps(model, separators=(",", ":")),
        encoding="utf-8",
    )
    print(f"wrote physical qualification model: {diagnostic_model_path}")

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

    physical_route = _qualify_physical_multilevel_route(model)
    if physical_route is not None:
        print("local-app physical multi-level route:")
        print(json.dumps(physical_route, indent=2, sort_keys=True))

    print(
        "local-app IFC build passed: "
        f"source={metadata.get('surface_source')} "
        f"cells={stats.get('surface_cell_count')} "
        f"spaces={stats.get('surface_internal_space_count')} "
        f"portals={stats.get('portals')}"
    )


if __name__ == "__main__":
    main()
