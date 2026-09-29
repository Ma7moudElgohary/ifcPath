from __future__ import annotations

import sys
from pathlib import Path

from ifcpath.build_service import build_inav_payload


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_local_app_build.py <model.ifc>")
    path = Path(sys.argv[1])
    result = build_inav_payload(path.read_bytes(), path.name)
    report = result["qualification"]
    stats = report.get("stats", {})
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
        f"cells={stats.get('surface_cell_count')} "
        f"spaces={stats.get('surface_internal_space_count')} "
        f"portals={stats.get('portals')}"
    )


if __name__ == "__main__":
    main()
