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

    if not report.get("valid", False):
        fail("validator reported errors")
    if report.get("node_count", 0) < 50:
        fail(f"too few navigation nodes: {report.get('node_count', 0)}")
    if report.get("edge_count", 0) < 40:
        fail(f"too few navigation edges: {report.get('edge_count', 0)}")
    if stats.get("levels", 0) < 2:
        fail(f"expected a multi-level building, got {stats.get('levels', 0)} levels")
    if stats.get("spaces", 0) < 5:
        fail(f"too few IFC spaces extracted: {stats.get('spaces', 0)}")
    if stats.get("portals", 0) < 5:
        fail(f"too few door portals extracted: {stats.get('portals', 0)}")
    if report.get("reachable_node_ratio", 0.0) < 0.50:
        fail(f"largest connected component is only {report.get('reachable_node_ratio', 0.0):.1%} of nodes")

    print("reference IFC qualification passed")
    print(
        f"nodes={report['node_count']} edges={report['edge_count']} "
        f"levels={stats.get('levels')} spaces={stats.get('spaces')} "
        f"portals={stats.get('portals')} components={report.get('component_count')} "
        f"largest_component={report.get('reachable_node_ratio', 0.0):.1%}"
    )


if __name__ == "__main__":
    main()
