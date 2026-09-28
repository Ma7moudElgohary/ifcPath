from __future__ import annotations

import argparse
import json
import sys

from .exporter import load_inav, save_inav
from .ifc_loader import BuildOptions, build_from_ifc
from .validation import ValidationReport, validate_model


def main() -> None:
    parser = argparse.ArgumentParser(prog="ifcpath", description="Generate and qualify portable indoor navigation from IFC")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="Build an .inav file from IFC")
    build.add_argument("ifc")
    build.add_argument("-o", "--output", required=True)
    build.add_argument("--floor-spacing", type=float, default=0.8)
    build.add_argument("--stair-spacing", type=float, default=0.25)
    build.add_argument("--connect-distance", type=float, default=1.25)
    build.add_argument("--strict", action="store_true", help="Return non-zero when generated INAV has validation errors")

    validate = sub.add_parser("validate", help="Validate an existing .inav file")
    validate.add_argument("inav")
    validate.add_argument("--json", action="store_true", help="Print machine-readable JSON report")
    validate.add_argument("--warnings-as-errors", action="store_true")

    args = parser.parse_args()
    if args.command == "build":
        model = build_from_ifc(
            args.ifc,
            BuildOptions(
                floor_spacing_m=args.floor_spacing,
                stair_spacing_m=args.stair_spacing,
                connect_distance_m=args.connect_distance,
            ),
        )
        output = save_inav(model, args.output)
        report = validate_model(model)
        print(f"Wrote {output}")
        print(f"nodes={len(model.nodes)} edges={len(model.edges)} spaces={len(model.spaces)} portals={len(model.portals)}")
        _print_report(report)
        if args.strict and not report.valid:
            raise SystemExit(2)
        return

    if args.command == "validate":
        report = validate_model(load_inav(args.inav))
        if args.json:
            print(json.dumps(report.to_dict(), indent=2))
        else:
            _print_report(report)
        has_warnings = any(issue.severity == "warning" for issue in report.issues)
        if not report.valid or (args.warnings_as_errors and has_warnings):
            raise SystemExit(2)


def _print_report(report: ValidationReport) -> None:
    print(
        "validation: "
        f"valid={report.valid} components={report.component_count} "
        f"largest={report.largest_component_nodes}/{report.node_count} "
        f"coverage={report.reachable_node_ratio:.1%} "
        f"errors={report.stats.get('errors', 0)} warnings={report.stats.get('warnings', 0)}"
    )
    for issue in report.issues:
        entity = f" [{issue.entity_id}]" if issue.entity_id else ""
        print(f"  {issue.severity.upper():7} {issue.code}{entity}: {issue.message}", file=sys.stderr)


if __name__ == "__main__":
    main()
