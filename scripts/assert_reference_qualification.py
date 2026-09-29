from __future__ import annotations

import json
import sys
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(f"reference IFC qualification failed: {message}")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_reference_qualification.py <qualification.json>")

    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    stats = report.get("stats", {})
    nodes = report.get("node_count", 0)
    edges = report.get("edge_count", 0)
    components = report.get("component_count", 0)

    if not report.get("valid", False):
        fail("validator reported errors")
    if nodes < 50:
        fail(f"too few compatibility navigation nodes: {nodes}")
    if nodes > 4000:
        fail(f"reference graph regressed to too many compatibility nodes: {nodes}")
    if edges < 40:
        fail(f"too few compatibility navigation edges: {edges}")
    if edges > nodes * 8:
        fail(f"compatibility graph is too dense: {edges} edges for {nodes} nodes")
    if stats.get("levels", 0) < 2:
        fail(f"expected a multi-level building, got {stats.get('levels', 0)} levels")
    if stats.get("spaces", 0) < 5:
        fail(f"too few IFC spaces extracted: {stats.get('spaces', 0)}")
    if stats.get("portals", 0) < 5:
        fail(f"too few semantic portals extracted: {stats.get('portals', 0)}")
    if stats.get("semantic_transition_errors", 0) != 0:
        fail(f"semantic transition graph has {stats.get('semantic_transition_errors')} validation errors")

    if stats.get("surface_authoritative", False):
        if stats.get("surface_cell_count", 0) < 50:
            fail(f"too few navigation surface cells: {stats.get('surface_cell_count', 0)}")
        if stats.get("surface_topology_errors", 0) != 0:
            fail(f"surface topology has {stats.get('surface_topology_errors')} error(s)")
        if stats.get("surface_asymmetric_edges", 0) != 0:
            fail(f"surface topology has {stats.get('surface_asymmetric_edges')} asymmetric edge(s)")
        if stats.get("surface_split_spaces", 0) != 0:
            fail(f"{stats.get('surface_split_spaces')} surfaced spaces are internally split")
        if stats.get("surface_portal_side_failures", 0) != 0:
            fail(
                f"{stats.get('surface_portal_side_failures')} semantic portals are not backed by qualified surface crossings"
            )
        if stats.get("surface_vertical_transfer_failures", 0) != 0:
            fail(
                f"{stats.get('surface_vertical_transfer_failures')} surface vertical transfers are unroutable"
            )
        if stats.get("surface_internal_space_count", 0) < 5:
            fail(f"too few internal surfaced spaces: {stats.get('surface_internal_space_count', 0)}")
        if stats.get("surface_exit_unreachable_spaces", 0) != 0:
            fail(
                f"{stats.get('surface_exit_unreachable_spaces')} internal surfaced spaces cannot reach a classified exit"
            )
        if not stats.get("surface_navigation_ready", False):
            fail("continuous navigation surface is not ready")
    else:
        if components > 6:
            fail(f"reference graph fragmented into too many components: {components}")
        if stats.get("portal_side_failures", 0) != 0:
            fail(f"{stats.get('portal_side_failures')} semantic portals are missing one or more connected sides")
        if stats.get("isolated_nodes", 0) != 0:
            fail(f"reference graph contains isolated navigation nodes: {stats.get('isolated_nodes')}")
        if stats.get("split_spaces", 0) != 0:
            fail(f"IFC spaces are internally split across graph components: {stats.get('split_spaces')}")
        if stats.get("exit_reachable_ratio", 0.0) < 0.90:
            fail(
                "too little compatibility navigation can reach a classified exit: "
                f"{stats.get('exit_reachable_ratio', 0.0):.1%}"
            )

    print("reference IFC qualification passed")
    if stats.get("surface_authoritative", False):
        print(
            f"cells={stats.get('surface_cell_count')} internal_spaces={stats.get('surface_internal_space_count')} "
            f"external_spaces={stats.get('surface_external_space_count')} portals={stats.get('portals')} "
            f"surface_transitions={stats.get('surface_vertical_transitions')} "
            f"exit_reachable={stats.get('surface_exit_reachable_ratio', 0.0):.1%} "
            f"legacy_nodes={nodes} legacy_components={components}"
        )
    else:
        print(
            f"nodes={nodes} edges={edges} edge_ratio={edges / max(nodes, 1):.2f} "
            f"levels={stats.get('levels')} spaces={stats.get('spaces')} "
            f"portals={stats.get('portals')} transitions={stats.get('semantic_transitions')} "
            f"components={components} exit_reachable={stats.get('exit_reachable_ratio', 0.0):.1%} "
            f"split_spaces={stats.get('split_spaces')}"
        )


if __name__ == "__main__":
    main()
