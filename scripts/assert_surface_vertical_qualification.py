from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from ifcpath.exporter import load_inav
from ifcpath.vertical_surface import (
    find_surface_vertical_transfer,
    surface_vertical_components,
)


def fail(message: str) -> None:
    raise SystemExit(f"surface vertical qualification failed: {message}")


def main() -> None:
    if len(sys.argv) not in {2, 3}:
        raise SystemExit(
            "usage: assert_surface_vertical_qualification.py <model.inav> [report.json]"
        )

    model = load_inav(sys.argv[1])
    components = surface_vertical_components(model)
    walkable = [component for component in components if component.kind in {"stair", "ramp", "escalator", "vertical"}]
    multi_level = []
    for component in walkable:
        levels = sorted({landing.level_id for landing in component.landings if landing.level_id})
        if len(levels) >= 2:
            multi_level.append((component, levels))

    transitions = [
        transition
        for transition in model.transitions
        if transition.source == "surface_vertical_touch"
        and transition.kind in {"stair", "ramp", "escalator", "vertical"}
    ]
    routed = []
    for transition in transitions:
        transfer = find_surface_vertical_transfer(model, transition)
        if transfer is None:
            continue
        rise = abs(transfer.points[-1][2] - transfer.points[0][2]) if transfer.points else 0.0
        routed.append(
            {
                "transition_id": transition.id,
                "kind": transition.kind,
                "resource_id": transition.resource_id,
                "from_space_id": transition.from_space_id,
                "to_space_id": transition.to_space_id,
                "from_level_id": transition.from_level_id,
                "to_level_id": transition.to_level_id,
                "cell_count": len(transfer.cell_ids),
                "point_count": len(transfer.points),
                "length_m": transfer.length_m,
                "rise_m": rise,
            }
        )

    report = {
        "surface_vertical_cells": sum(
            1 for cell in model.cells if cell.terrain in {"stair", "ramp", "escalator"}
        ),
        "component_count": len(walkable),
        "multi_level_component_count": len(multi_level),
        "surface_transition_count": len(transitions),
        "routed_surface_transition_count": len(routed),
        "components": [
            {
                "kind": component.kind,
                "resource_id": component.resource_id,
                "cell_count": len(component.cell_ids),
                "landing_spaces": sorted({landing.space_id for landing in component.landings}),
                "landing_levels": levels,
            }
            for component, levels in multi_level
        ],
        "transfers": routed,
    }

    if report["surface_vertical_cells"] < 1:
        fail("reference IFC exported no walkable stair/ramp/escalator cells")
    if not multi_level:
        fail("no continuous vertical surface component touches two or more levels")
    if not transitions:
        fail("no surface_vertical_touch semantic transition was exported")
    if not routed:
        fail("surface semantic transitions exist but none produce a funnel transfer")
    if not any(item["rise_m"] > 1.0 for item in routed):
        fail("surface transfers do not contain a meaningful vertical rise")
    if any(not math.isfinite(item["length_m"]) or item["length_m"] <= 0 for item in routed):
        fail("surface transfer has invalid length")

    if len(sys.argv) == 3:
        Path(sys.argv[2]).write_text(json.dumps(report, indent=2), encoding="utf-8")

    best = max(routed, key=lambda item: item["rise_m"])
    print(
        "surface vertical qualification passed: "
        f"cells={report['surface_vertical_cells']} components={len(walkable)} "
        f"multi_level={len(multi_level)} transitions={len(transitions)} "
        f"routed={len(routed)} max_rise={best['rise_m']:.2f}m"
    )


if __name__ == "__main__":
    main()
