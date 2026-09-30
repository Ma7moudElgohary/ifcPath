from __future__ import annotations

import json
import sys
from pathlib import Path

from ifcpath.build_service import build_inav_payload


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_local_app_build.py <model.ifc>")
    path = Path(sys.argv[1])
    result = build_inav_payload(path.read_bytes(), path.name)
    report = result["qualification"]
    model = result.get("model", {})
    stats = report.get("stats", {})
    metadata = model.get("metadata", {})

    reconstruction = metadata.get("surface_reconstruction")
    diagnostic = {
        "surface_source": metadata.get("surface_source"),
        "surface_reconstruction": reconstruction,
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
