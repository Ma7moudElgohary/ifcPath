from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from ifcpath.exporter import load_inav
from ifcpath.surface_funnel import find_surface_funnel_route


def fail(message: str) -> None:
    raise SystemExit(f"real IFC surface-route benchmark failed: {message}")


def centroid(cell):
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )


def main() -> None:
    if len(sys.argv) not in {2, 3}:
        raise SystemExit("usage: assert_surface_route_benchmark.py <model.inav> [report.json]")

    model = load_inav(sys.argv[1])
    by_space: dict[str, list] = {}
    for cell in model.cells:
        if cell.space_id and cell.terrain == "open":
            by_space.setdefault(cell.space_id, []).append(cell)

    cases = []
    for space_id, cells in sorted(by_space.items()):
        if len(cells) < 2:
            continue
        points = [centroid(cell) for cell in cells]
        best = None
        for index, start in enumerate(points):
            for goal in points[index + 1 :]:
                direct = math.dist(start, goal)
                if direct < 1.0:
                    continue
                route = find_surface_funnel_route(cells, start, goal)
                if route is None:
                    fail(f"no funnel route in {space_id}")
                ratio = route.length_m / direct
                candidate = (direct, ratio, route.length_m, len(route.points), start, goal)
                if best is None or candidate[0] > best[0]:
                    best = candidate
        if best is not None:
            direct, ratio, route_length, waypoint_count, start, goal = best
            cases.append(
                {
                    "space_id": space_id,
                    "cell_count": len(cells),
                    "direct_distance_m": direct,
                    "route_length_m": route_length,
                    "stretch_ratio": ratio,
                    "waypoint_count": waypoint_count,
                    "start": start,
                    "goal": goal,
                }
            )

    if len(cases) < 5:
        fail(f"too few benchmarkable CDT spaces: {len(cases)}")

    worst = max(cases, key=lambda item: item["stretch_ratio"])
    if worst["stretch_ratio"] > 2.5:
        fail(
            f"excessive funnel-route stretch in {worst['space_id']}: "
            f"{worst['stretch_ratio']:.3f}x"
        )

    report = {
        "router": "surface-cell-a-star+portal-funnel",
        "case_count": len(cases),
        "worst_stretch_ratio": worst["stretch_ratio"],
        "worst_space_id": worst["space_id"],
        "cases": cases,
    }
    if len(sys.argv) == 3:
        Path(sys.argv[2]).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(
        "real IFC surface-route benchmark passed: "
        f"spaces={len(cases)} worst_stretch={worst['stretch_ratio']:.3f}x "
        f"space={worst['space_id']}"
    )


if __name__ == "__main__":
    main()
